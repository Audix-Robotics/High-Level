#!/usr/bin/env python3

import math
import random
import subprocess

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


def clamp(value, low, high):
    return max(low, min(high, value))


class ArenaObstacleManager(Node):
    def __init__(self):
        super().__init__('arena_obstacle_manager')

        self.declare_parameter('world_name', 'arena10')
        self.declare_parameter('click_topic', '/clicked_point')
        self.declare_parameter('preset_topic', '/arena_spawn_preset')
        self.declare_parameter('command_topic', '/arena_spawn_command')
        self.declare_parameter('arena_min_x', -4.55)
        self.declare_parameter('arena_max_x', 4.55)
        self.declare_parameter('arena_min_y', -4.55)
        self.declare_parameter('arena_max_y', 4.55)
        self.declare_parameter('obstacle_z_margin', 0.01)
        self.declare_parameter('dynamic_update_period', 0.12)
        self.declare_parameter('dynamic_speed_min', 0.16)
        self.declare_parameter('dynamic_speed_max', 0.45)
        self.declare_parameter('small_box_size', [0.25, 0.25, 0.35])
        self.declare_parameter('medium_box_size', [0.45, 0.45, 0.60])
        self.declare_parameter('large_box_size', [0.80, 0.55, 0.95])
        self.declare_parameter('random_min_size', [0.20, 0.20, 0.25])
        self.declare_parameter('random_max_size', [1.10, 0.90, 1.10])

        self.world_name = str(self.get_parameter('world_name').value)
        self.click_topic = str(self.get_parameter('click_topic').value)
        self.preset_topic = str(self.get_parameter('preset_topic').value)
        self.command_topic = str(self.get_parameter('command_topic').value)
        self.arena_min_x = float(self.get_parameter('arena_min_x').value)
        self.arena_max_x = float(self.get_parameter('arena_max_x').value)
        self.arena_min_y = float(self.get_parameter('arena_min_y').value)
        self.arena_max_y = float(self.get_parameter('arena_max_y').value)
        self.obstacle_z_margin = float(self.get_parameter('obstacle_z_margin').value)
        self.dynamic_update_period = float(self.get_parameter('dynamic_update_period').value)
        self.dynamic_speed_min = float(self.get_parameter('dynamic_speed_min').value)
        self.dynamic_speed_max = float(self.get_parameter('dynamic_speed_max').value)
        self.small_box_size = tuple(float(v) for v in self.get_parameter('small_box_size').value)
        self.medium_box_size = tuple(float(v) for v in self.get_parameter('medium_box_size').value)
        self.large_box_size = tuple(float(v) for v in self.get_parameter('large_box_size').value)
        self.random_min_size = tuple(float(v) for v in self.get_parameter('random_min_size').value)
        self.random_max_size = tuple(float(v) for v in self.get_parameter('random_max_size').value)

        self.current_preset = 'dynamic_medium'
        self.dynamic_enabled = True
        self.spawn_index = 0
        self.randomizer = random.Random(11)
        self.obstacles = {}
        self.spawn_order = []

        self.marker_pub = self.create_publisher(MarkerArray, '/debug/arena_obstacles', 10)

        self.create_subscription(PointStamped, self.click_topic, self._clicked_point_cb, 10)
        self.create_subscription(String, self.preset_topic, self._preset_cb, 10)
        self.create_subscription(String, self.command_topic, self._command_cb, 10)
        self.create_timer(self.dynamic_update_period, self._update_dynamic_obstacles)
        self.create_timer(0.2, self._publish_markers)

        self.get_logger().info(
            'Arena obstacle manager ready. Use RViz Publish Point to spawn obstacles at the clicked location.'
        )

    def _run_gz_service(self, service_name, reqtype, reptype, request_text, timeout_ms=3000):
        command = [
            'gz', 'service',
            '-s', service_name,
            '--reqtype', reqtype,
            '--reptype', reptype,
            '--timeout', str(timeout_ms),
            '--req', request_text,
        ]
        try:
            result = subprocess.run(
                command,
                capture_output=True,
                text=True,
                check=False,
                timeout=max(1.0, timeout_ms / 1000.0 + 1.0),
            )
        except (OSError, subprocess.SubprocessError) as exc:
            return False, str(exc)

        output = (result.stdout or '') + (result.stderr or '')
        success = result.returncode == 0 and 'data: true' in output.lower()
        return success, output.strip()

    @staticmethod
    def _escape_proto_string(value):
        return value.replace('\\', '\\\\').replace('"', '\\"').replace('\n', '\\n')

    def _spawn_obstacle_gz(self, name, x, y, profile):
        z = 0.5 * profile['size'][2] + self.obstacle_z_margin
        sdf_text = self._escape_proto_string(self._build_box_sdf(name, profile).strip())
        request_text = (
            f'name: "{name}" '
            f'sdf: "{sdf_text}" '
            f'pose {{ position {{ x: {x:.4f} y: {y:.4f} z: {z:.4f} }} orientation {{ w: 1.0 }} }} '
            f'relative_to: "world" allow_renaming: false'
        )
        return self._run_gz_service(
            f'/world/{self.world_name}/create',
            'gz.msgs.EntityFactory',
            'gz.msgs.Boolean',
            request_text,
            timeout_ms=4000,
        )

    def _set_obstacle_pose_gz(self, obstacle):
        sz = obstacle['size'][2]
        z = 0.5 * sz + self.obstacle_z_margin
        request_text = (
            f'name: "{obstacle["name"]}" '
            f'position {{ x: {obstacle["x"]:.4f} y: {obstacle["y"]:.4f} z: {z:.4f} }} '
            f'orientation {{ z: {math.sin(0.5 * obstacle["yaw"]):.6f} w: {math.cos(0.5 * obstacle["yaw"]):.6f} }}'
        )
        return self._run_gz_service(
            f'/world/{self.world_name}/set_pose',
            'gz.msgs.Pose',
            'gz.msgs.Boolean',
            request_text,
            timeout_ms=1000,
        )

    def _delete_obstacle_gz(self, name):
        request_text = f'name: "{name}" type: MODEL'
        return self._run_gz_service(
            f'/world/{self.world_name}/remove',
            'gz.msgs.Entity',
            'gz.msgs.Boolean',
            request_text,
            timeout_ms=3000,
        )

    def _preset_cb(self, msg):
        preset = msg.data.strip()
        if preset:
            self.current_preset = preset
            self.get_logger().info('Spawn preset set to %s' % self.current_preset)

    def _command_cb(self, msg):
        command = msg.data.strip().lower()
        if command == 'clear_all':
            self._clear_all()
        elif command == 'remove_last':
            self._remove_last()
        elif command == 'pause_dynamic':
            self.dynamic_enabled = False
            self.get_logger().info('Dynamic obstacle motion paused')
        elif command == 'resume_dynamic':
            self.dynamic_enabled = True
            self.get_logger().info('Dynamic obstacle motion resumed')
        elif command == 'randomize_dynamic':
            self._randomize_dynamic_velocities()

    def _clicked_point_cb(self, msg):
        x = clamp(msg.point.x, self.arena_min_x, self.arena_max_x)
        y = clamp(msg.point.y, self.arena_min_y, self.arena_max_y)

        profile = self._profile_for_current_preset()
        self.spawn_index += 1
        name = 'arena_obstacle_%03d' % self.spawn_index
        success, output = self._spawn_obstacle_gz(name, x, y, profile)
        if not success:
            self.get_logger().warn('Gazebo rejected obstacle spawn for %s: %s' % (name, output))
            return

        obstacle = {
            'name': name,
            'x': x,
            'y': y,
            'yaw': 0.0,
            'size': profile['size'],
            'dynamic': profile['dynamic'],
            'vx': profile['vx'],
            'vy': profile['vy'],
        }
        self.obstacles[name] = obstacle
        self.spawn_order.append(name)
        self.get_logger().info('Spawned %s at (%.2f, %.2f) as %s' % (name, x, y, self.current_preset))

    def _profile_for_current_preset(self):
        preset = self.current_preset.lower()
        dynamic = 'dynamic' in preset
        if 'small' in preset:
            size = self.small_box_size
        elif 'large' in preset:
            size = self.large_box_size
        elif 'random' in preset:
            size = tuple(
                self.randomizer.uniform(low, high)
                for low, high in zip(self.random_min_size, self.random_max_size)
            )
        else:
            size = self.medium_box_size

        if dynamic:
            heading = self.randomizer.uniform(-math.pi, math.pi)
            speed = self.randomizer.uniform(self.dynamic_speed_min, self.dynamic_speed_max)
            vx = speed * math.cos(heading)
            vy = speed * math.sin(heading)
        else:
            vx = 0.0
            vy = 0.0

        return {
            'dynamic': dynamic,
            'size': size,
            'vx': vx,
            'vy': vy,
            'color': self._color_for_preset(dynamic, size),
        }

    def _color_for_preset(self, dynamic, size):
        largest = max(size[0], size[1], size[2])
        if dynamic:
            return (0.95, 0.28 + 0.15 * min(1.0, largest), 0.20, 1.0)
        return (0.15, 0.55 + 0.10 * min(1.0, largest), 0.95, 1.0)

    def _build_box_sdf(self, name, profile):
        sx, sy, sz = profile['size']
        r, g, b, a = profile['color']
        static_text = 'false' if profile['dynamic'] else 'true'
        gravity_text = 'false' if profile['dynamic'] else 'true'
        mass = max(2.0, 24.0 * sx * sy * sz)
        ixx = mass * (sy * sy + sz * sz) / 12.0
        iyy = mass * (sx * sx + sz * sz) / 12.0
        izz = mass * (sx * sx + sy * sy) / 12.0
        return f"""
<sdf version='1.8'>
  <model name='{name}'>
    <static>{static_text}</static>
    <allow_auto_disable>false</allow_auto_disable>
    <link name='body'>
      <gravity>{gravity_text}</gravity>
      <inertial>
        <mass>{mass:.4f}</mass>
        <inertia>
          <ixx>{ixx:.6f}</ixx>
          <iyy>{iyy:.6f}</iyy>
          <izz>{izz:.6f}</izz>
        </inertia>
      </inertial>
      <collision name='collision'>
        <geometry>
          <box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box>
        </geometry>
      </collision>
      <visual name='visual'>
        <geometry>
          <box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box>
        </geometry>
        <material>
          <ambient>{r:.3f} {g:.3f} {b:.3f} {a:.3f}</ambient>
          <diffuse>{r:.3f} {g:.3f} {b:.3f} {a:.3f}</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

    def _update_dynamic_obstacles(self):
        if not self.dynamic_enabled:
            return

        dt = self.dynamic_update_period
        for obstacle in self.obstacles.values():
            if not obstacle['dynamic']:
                continue

            sx, sy, sz = obstacle['size']
            half_x = 0.5 * sx
            half_y = 0.5 * sy
            next_x = obstacle['x'] + obstacle['vx'] * dt
            next_y = obstacle['y'] + obstacle['vy'] * dt

            if next_x <= self.arena_min_x + half_x or next_x >= self.arena_max_x - half_x:
                obstacle['vx'] *= -1.0
                next_x = clamp(next_x, self.arena_min_x + half_x, self.arena_max_x - half_x)
            if next_y <= self.arena_min_y + half_y or next_y >= self.arena_max_y - half_y:
                obstacle['vy'] *= -1.0
                next_y = clamp(next_y, self.arena_min_y + half_y, self.arena_max_y - half_y)

            obstacle['x'] = next_x
            obstacle['y'] = next_y
            obstacle['yaw'] = math.atan2(obstacle['vy'], obstacle['vx']) if abs(obstacle['vx']) + abs(obstacle['vy']) > 1e-6 else obstacle['yaw']

            success, output = self._set_obstacle_pose_gz(obstacle)
            if not success:
                self.get_logger().warn('Failed to update %s pose: %s' % (obstacle['name'], output))

    def _remove_last(self):
        if not self.spawn_order:
            return
        self._delete_obstacle(self.spawn_order[-1])

    def _clear_all(self):
        for name in list(self.spawn_order):
            self._delete_obstacle(name)

    def _delete_obstacle(self, name):
        if name not in self.obstacles:
            return
        success, output = self._delete_obstacle_gz(name)
        if success:
            self.obstacles.pop(name, None)
            self.spawn_order = [item for item in self.spawn_order if item != name]
            self.get_logger().info('Deleted obstacle %s' % name)
        else:
            self.get_logger().warn('Failed to delete %s: %s' % (name, output))

    def _randomize_dynamic_velocities(self):
        for obstacle in self.obstacles.values():
            if not obstacle['dynamic']:
                continue
            heading = self.randomizer.uniform(-math.pi, math.pi)
            speed = self.randomizer.uniform(self.dynamic_speed_min, self.dynamic_speed_max)
            obstacle['vx'] = speed * math.cos(heading)
            obstacle['vy'] = speed * math.sin(heading)

    def _publish_markers(self):
        now = self.get_clock().now().to_msg()
        markers = []

        clear = Marker()
        clear.header.frame_id = 'arena10'
        clear.header.stamp = now
        clear.action = Marker.DELETEALL
        markers.append(clear)

        for index, obstacle in enumerate(self.obstacles.values()):
            sx, sy, sz = obstacle['size']

            marker = Marker()
            marker.header.frame_id = 'arena10'
            marker.header.stamp = now
            marker.ns = 'arena_obstacles'
            marker.id = index
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = obstacle['x']
            marker.pose.position.y = obstacle['y']
            marker.pose.position.z = 0.5 * sz + self.obstacle_z_margin
            marker.pose.orientation.z = math.sin(0.5 * obstacle['yaw'])
            marker.pose.orientation.w = math.cos(0.5 * obstacle['yaw'])
            marker.scale.x = sx
            marker.scale.y = sy
            marker.scale.z = sz
            if obstacle['dynamic']:
                marker.color.r = 0.96
                marker.color.g = 0.38
                marker.color.b = 0.22
            else:
                marker.color.r = 0.18
                marker.color.g = 0.70
                marker.color.b = 0.96
            marker.color.a = 0.60
            markers.append(marker)

            label = Marker()
            label.header.frame_id = 'arena10'
            label.header.stamp = now
            label.ns = 'arena_obstacle_labels'
            label.id = 1000 + index
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = obstacle['x']
            label.pose.position.y = obstacle['y']
            label.pose.position.z = sz + 0.20
            label.pose.orientation.w = 1.0
            label.scale.z = 0.18
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 0.90
            label.text = obstacle['name']
            markers.append(label)

        self.marker_pub.publish(MarkerArray(markers=markers))


def main():
    rclpy.init()
    node = ArenaObstacleManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()