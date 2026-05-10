#!/usr/bin/env python3

import json
import math
import os
import subprocess

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


def clamp(value, low, high):
    return max(low, min(high, value))


class ArenaCourseSpawner(Node):
    def __init__(self):
        super().__init__('arena_course_spawner')

        pkg_share = get_package_share_directory('audix')
        default_course_file = os.path.join(pkg_share, 'config', 'arena_stress_course.json')

        self.declare_parameter('world_name', 'warehouse')
        self.declare_parameter('course_file', default_course_file)
        self.declare_parameter('start_delay_sec', 2.0)
        self.declare_parameter('dynamic_update_period', 0.12)
        self.declare_parameter('obstacle_z_margin', 0.01)
        self.declare_parameter('obstacle_collision_enabled', False)
        self.declare_parameter('arena_min_x', -4.55)
        self.declare_parameter('arena_max_x', 4.55)
        self.declare_parameter('arena_min_y', -4.55)
        self.declare_parameter('arena_max_y', 4.55)

        self.world_name = str(self.get_parameter('world_name').value)
        self.course_file = str(self.get_parameter('course_file').value)
        self.start_delay_sec = float(self.get_parameter('start_delay_sec').value)
        self.dynamic_update_period = float(self.get_parameter('dynamic_update_period').value)
        self.obstacle_z_margin = float(self.get_parameter('obstacle_z_margin').value)
        self.obstacle_collision_enabled = bool(self.get_parameter('obstacle_collision_enabled').value)
        self.arena_min_x = float(self.get_parameter('arena_min_x').value)
        self.arena_max_x = float(self.get_parameter('arena_max_x').value)
        self.arena_min_y = float(self.get_parameter('arena_min_y').value)
        self.arena_max_y = float(self.get_parameter('arena_max_y').value)

        self.marker_pub = self.create_publisher(MarkerArray, '/debug/arena_obstacles', 10)

        self.obstacles = []
        self.spawned = False
        self._load_course()

        self.create_timer(self.start_delay_sec, self._spawn_course_once)
        self.create_timer(self.dynamic_update_period, self._update_dynamic_obstacles)
        self.create_timer(0.2, self._publish_markers)

        self.get_logger().info('Arena course spawner ready: %s' % self.course_file)

    def _load_course(self):
        with open(self.course_file, 'r', encoding='utf-8') as handle:
            data = json.load(handle)

        obstacles = data.get('obstacles', [])
        self.obstacles = []
        for index, obstacle in enumerate(obstacles):
            size = obstacle['size']
            vx = float(obstacle.get('vx', 0.0))
            vy = float(obstacle.get('vy', 0.0))
            yaw = math.atan2(vy, vx) if abs(vx) + abs(vy) > 1e-6 else float(obstacle.get('yaw', 0.0))
            self.obstacles.append({
                'id': index,
                'name': str(obstacle['name']),
                'level': str(obstacle.get('level', 'L?')),
                'note': str(obstacle.get('note', '')),
                'x': float(obstacle['x']),
                'y': float(obstacle['y']),
                'yaw': yaw,
                'size': (float(size[0]), float(size[1]), float(size[2])),
                'dynamic': bool(obstacle.get('dynamic', False)),
                'vx': vx,
                'vy': vy,
                'color': tuple(float(c) for c in obstacle.get('color', [0.2, 0.7, 0.95, 1.0])),
            })

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

    def _build_box_sdf(self, obstacle):
        sx, sy, sz = obstacle['size']
        r, g, b, a = obstacle['color']
        static_text = 'false' if obstacle['dynamic'] else 'true'
        gravity_text = 'false' if obstacle['dynamic'] else 'true'
        mass = max(2.0, 24.0 * sx * sy * sz)
        ixx = mass * (sy * sy + sz * sz) / 12.0
        iyy = mass * (sx * sx + sz * sz) / 12.0
        izz = mass * (sx * sx + sy * sy) / 12.0
        collision = ''
        if self.obstacle_collision_enabled:
            collision = f"""
      <collision name='collision'>
        <geometry>
          <box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box>
        </geometry>
      </collision>"""
        return f"""
<sdf version='1.8'>
  <model name='{obstacle['name']}'>
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
{collision}
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

    def _spawn_obstacle_gz(self, obstacle):
        z = 0.5 * obstacle['size'][2] + self.obstacle_z_margin
        sdf_text = self._escape_proto_string(self._build_box_sdf(obstacle).strip())
        request_text = (
            f'name: "{obstacle["name"]}" '
            f'sdf: "{sdf_text}" '
            f'pose {{ position {{ x: {obstacle["x"]:.4f} y: {obstacle["y"]:.4f} z: {z:.4f} }} '
            f'orientation {{ z: {math.sin(0.5 * obstacle["yaw"]):.6f} w: {math.cos(0.5 * obstacle["yaw"]):.6f} }} }} '
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

    def _spawn_course_once(self):
        if self.spawned:
            return
        success_count = 0
        for obstacle in self.obstacles:
            success, output = self._spawn_obstacle_gz(obstacle)
            if success:
                success_count += 1
            else:
                self.get_logger().warn('Failed to spawn %s: %s' % (obstacle['name'], output))
        if success_count == len(self.obstacles):
            self.spawned = True
            self.get_logger().info('Spawned %d/%d course obstacles.' % (success_count, len(self.obstacles)))

    def _update_dynamic_obstacles(self):
        if not self.spawned:
            return
        dt = self.dynamic_update_period
        for obstacle in self.obstacles:
            if not obstacle['dynamic']:
                continue
            sx, sy, _ = obstacle['size']
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
            if abs(obstacle['vx']) + abs(obstacle['vy']) > 1e-6:
                obstacle['yaw'] = math.atan2(obstacle['vy'], obstacle['vx'])

            success, output = self._set_obstacle_pose_gz(obstacle)
            if not success:
                self.get_logger().warn('Failed to update %s: %s' % (obstacle['name'], output))

    def _publish_markers(self):
        now = self.get_clock().now().to_msg()
        markers = []

        clear = Marker()
        clear.header.frame_id = self.world_name
        clear.header.stamp = now
        clear.action = Marker.DELETEALL
        markers.append(clear)

        for obstacle in self.obstacles:
            sx, sy, sz = obstacle['size']
            marker = Marker()
            marker.header.frame_id = self.world_name
            marker.header.stamp = now
            marker.ns = 'arena_obstacles'
            marker.id = obstacle['id']
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
            marker.color.r = obstacle['color'][0]
            marker.color.g = obstacle['color'][1]
            marker.color.b = obstacle['color'][2]
            marker.color.a = 0.60
            markers.append(marker)

            label = Marker()
            label.header.frame_id = self.world_name
            label.header.stamp = now
            label.ns = 'arena_obstacle_labels'
            label.id = 1000 + obstacle['id']
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = obstacle['x']
            label.pose.position.y = obstacle['y']
            label.pose.position.z = sz + 0.25
            label.pose.orientation.w = 1.0
            label.scale.z = 0.16
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 0.95
            label.text = '%s %s' % (obstacle['level'], obstacle['name'])
            markers.append(label)

        self.marker_pub.publish(MarkerArray(markers=markers))


def main():
    rclpy.init()
    node = ArenaCourseSpawner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
