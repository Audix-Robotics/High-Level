#!/usr/bin/env python3

import math
import os
import tkinter as tk
from tkinter import messagebox, ttk

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped, Pose
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

from audix.msg import FleetStatus
from audix.srv import AssignMission, DespawnRobot, GetRobotStatus, SpawnRobot


class WarehouseFleetSpawnPanel(Node):
    def __init__(self):
        super().__init__('warehouse_fleet_spawn_panel')

        pkg_share = get_package_share_directory('audix')
        self.declare_parameter(
            'warehouse_config_path',
            os.path.join(pkg_share, 'config', 'warehouse_lanes.yaml'),
        )
        self.declare_parameter(
            'missions_config_path',
            os.path.join(pkg_share, 'config', 'warehouse_missions.yaml'),
        )

        self.warehouse_config = self._load_yaml('warehouse_config_path')
        self.missions_config = self._load_yaml('missions_config_path')
        self.saved_routines = dict(self.missions_config.get('saved_routines', {}))
        self.latest_status = {}
        self.robot_status_widgets = {}
        self.previous_active_robot_ids = set()
        self.rviz_waypoint_drafts = {}
        self.lane_choices = self._build_lane_choices()
        self.lane_display_to_id = {label: lane_id for label, lane_id in self.lane_choices}
        self.lane_id_to_display = {lane_id: label for label, lane_id in self.lane_choices}

        self.spawn_robot_client = self.create_client(SpawnRobot, '/fleet/spawn_robot')
        self.assign_mission_client = self.create_client(AssignMission, '/fleet/assign_mission')
        self.get_status_client = self.create_client(GetRobotStatus, '/fleet/get_robot_status')
        self.despawn_robot_client = self.create_client(DespawnRobot, '/fleet/despawn_robot')

        self.fleet_status_sub = self.create_subscription(
            FleetStatus,
            '/fleet/status',
            self.on_fleet_status,
            10,
        )
        self.clicked_point_sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.on_clicked_point,
            10,
        )

        self.preset_pub = self.create_publisher(String, '/arena_spawn_preset', 10)
        self.command_pub = self.create_publisher(String, '/arena_spawn_command', 10)
        self.rviz_command_pub = self.create_publisher(String, '/fleet/rviz_command', 10)
        self.mission_control_pubs = {}

        self.root = tk.Tk()
        self.root.title('Audix Warehouse Fleet Manager')
        self.root.geometry('1080x820')
        self.root.configure(bg='#eef3f7')

        style = ttk.Style()
        style.theme_use('clam')
        style.configure('TNotebook', background='#eef3f7')
        style.configure('TFrame', background='#eef3f7')
        style.configure('TLabelframe', background='#eef3f7')
        style.configure('TLabelframe.Label', background='#eef3f7', font=('Arial', 10, 'bold'))
        style.configure('TButton', font=('Arial', 10))

        self.selected_robot_var = tk.IntVar(value=0)
        self.rviz_mode_var = tk.StringVar(value='idle')

        self.create_fleet_header_frame()
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill='both', expand=True, padx=10, pady=6)

        self.operations_tab = ttk.Frame(self.notebook)
        self.robots_tab = ttk.Frame(self.notebook)
        self.obstacles_tab = ttk.Frame(self.notebook)

        self.notebook.add(self.operations_tab, text='Fleet Control')
        self.notebook.add(self.robots_tab, text='Robot Missions')
        self.notebook.add(self.obstacles_tab, text='Obstacles')

        self.create_spawn_controls_frame(self.operations_tab)
        self.create_rviz_controls_frame(self.operations_tab)
        self.create_quick_help_frame(self.operations_tab)
        self.create_robot_area(self.robots_tab)
        self.create_obstacle_frame(self.obstacles_tab)

        self.root.protocol('WM_DELETE_WINDOW', self._close)
        self.create_timer(0.5, self.update_robot_display)
        self.create_timer(0.05, self._pump_tk)

    def _load_yaml(self, parameter_name):
        path = str(self.get_parameter(parameter_name).value)
        with open(path, 'r', encoding='utf-8') as config_file:
            return yaml.safe_load(config_file) or {}

    def _missions_config_path(self):
        return str(self.get_parameter('missions_config_path').value)

    def _build_lane_choices(self):
        choices = []
        for lane_key, lane_cfg in sorted(
            self.warehouse_config.get('lanes', {}).items(),
            key=lambda item: int(item[0].split('_')[-1]),
        ):
            lane_id = int(lane_key.split('_')[-1])
            label = f'{lane_id + 1} - {lane_cfg.get("name", f"Lane {lane_id + 1}")}'
            choices.append((label, lane_id))
        return choices

    def _saved_routine_names(self):
        return [''] + sorted(self.saved_routines.keys())

    def _lane_choice_from_name(self, lane_name):
        lane_name = (lane_name or '').strip().lower()
        for label, lane_id in self.lane_choices:
            lane_cfg = self.warehouse_config.get('lanes', {}).get(f'lane_{lane_id}', {})
            if lane_cfg.get('name', '').strip().lower() == lane_name:
                return label
        return self.lane_choices[0][0] if self.lane_choices else '1 - Lane'

    def _lane_center_y(self, lane_id):
        lane_cfg = self.warehouse_config.get('lanes', {}).get(f'lane_{lane_id}', {})
        return float(lane_cfg.get('center_y', 0.0))

    def _default_start_x(self):
        bounds = self.warehouse_config.get('warehouse', {}).get('bounds', {})
        return float(bounds.get('min_x', -2.9)) + 0.4

    def _lift_height_for_level(self, level):
        mapping = {1: 0.34, 2: 0.67, 3: 1.0}
        return mapping.get(int(level), 1.0)

    def _scan_yaw_for_direction(self, direction):
        lookup = {
            'left': math.pi / 2.0,
            'right': -math.pi / 2.0,
            'forward': 0.0,
            'back': math.pi,
        }
        return lookup.get(direction, 0.0)

    def _write_missions_config(self):
        data = dict(self.missions_config)
        data['missions'] = data.get('missions', {})
        data['saved_routines'] = self.saved_routines
        with open(self._missions_config_path(), 'w', encoding='utf-8') as config_file:
            yaml.safe_dump(data, config_file, sort_keys=False)

    def _refresh_saved_routine_selectors(self):
        values = self._saved_routine_names()
        for widgets in self.robot_status_widgets.values():
            combo = widgets.get('saved_routine_combo')
            if combo is not None:
                combo.configure(values=values)

    def create_fleet_header_frame(self):
        header_frame = ttk.LabelFrame(self.root, text='Fleet Status', padding=10)
        header_frame.pack(fill='x', padx=10, pady=5)

        self.header_label = tk.Label(
            header_frame,
            text='Warehouse Fleet Manager  |  Active Robots: 0  |  Status: IDLE',
            font=('Arial', 12, 'bold'),
        )
        self.header_label.pack()

        self.header_hint = tk.Label(
            header_frame,
            text='Recommended flow: spawn the robot, build a lane routine, generate shelf stops, then run or save the routine. RViz click mode stays available for quick manual routes.',
            font=('Arial', 10),
            anchor='w',
        )
        self.header_hint.pack(fill='x', pady=(6, 0))

    def create_spawn_controls_frame(self, parent):
        frame = ttk.LabelFrame(parent, text='Spawn Fleet', padding=10)
        frame.pack(fill='x', padx=10, pady=5)

        tk.Label(frame, text='Number of Robots:').pack(side='left')
        self.num_robots_var = tk.IntVar(value=1)
        tk.Spinbox(frame, from_=1, to=5, textvariable=self.num_robots_var, width=5).pack(side='left', padx=5)
        tk.Button(frame, text='Spawn Fleet', command=self.spawn_fleet).pack(side='left', padx=5)
        tk.Button(frame, text='Clear All Robots', command=self.clear_all_robots).pack(side='left', padx=5)

        tk.Label(frame, text='  OR  Robot ID:').pack(side='left', padx=20)
        self.single_robot_id_var = tk.IntVar(value=0)
        tk.Spinbox(frame, from_=0, to=10, textvariable=self.single_robot_id_var, width=5).pack(side='left', padx=5)

        tk.Label(frame, text='Lane:').pack(side='left')
        self.lane_var = tk.StringVar(value='Center')
        lane_options = ['North', 'Mid-North', 'Center', 'Mid-South', 'South']
        ttk.Combobox(
            frame,
            textvariable=self.lane_var,
            values=lane_options,
            state='readonly',
            width=12,
        ).pack(side='left', padx=5)
        tk.Button(frame, text='Spawn Robot', command=self.spawn_single_robot).pack(side='left', padx=5)

    def create_rviz_controls_frame(self, parent):
        frame = ttk.LabelFrame(parent, text='RViz Click Controls', padding=10)
        frame.pack(fill='x', padx=10, pady=5)

        tk.Label(frame, text='Target Robot:').pack(side='left')
        tk.Spinbox(frame, from_=0, to=10, textvariable=self.selected_robot_var, width=5).pack(side='left', padx=5)
        tk.Button(frame, text='Select Robot', command=self.select_target_robot).pack(side='left', padx=4)

        ttk.Separator(frame, orient='vertical').pack(side='left', fill='y', padx=8)

        tk.Button(frame, text='Spawn Click Mode', command=lambda: self.set_rviz_mode('spawn')).pack(side='left', padx=4)
        tk.Button(frame, text='Select Click Mode', command=lambda: self.set_rviz_mode('select')).pack(side='left', padx=4)
        tk.Button(frame, text='Waypoint Click Mode', command=lambda: self.set_rviz_mode('waypoint')).pack(side='left', padx=4)

        ttk.Separator(frame, orient='vertical').pack(side='left', fill='y', padx=8)

        tk.Button(frame, text='Commit Route', command=self.commit_rviz_waypoints).pack(side='left', padx=4)
        tk.Button(frame, text='Clear Route', command=self.clear_rviz_waypoints).pack(side='left', padx=4)
        tk.Button(frame, text='Remove Robot', command=self.remove_selected_robot).pack(side='left', padx=4)

        self.rviz_status_label = tk.Label(
            parent,
            text='RViz mode: idle | target: robot_0 | draft points: 0',
            font=('Arial', 10),
            anchor='w',
            bg='#eef3f7',
        )
        self.rviz_status_label.pack(fill='x', padx=14, pady=(0, 4))

    def create_quick_help_frame(self, parent):
        frame = ttk.LabelFrame(parent, text='Operator Flow', padding=12)
        frame.pack(fill='both', expand=True, padx=10, pady=5)

        help_text = (
            '1. Spawn a robot into lane 1..5.\n'
            '2. In Robot Programs, choose the lane, number of shelf stops, spacing, and scan duration.\n'
            '3. Generate the waypoint array, then set shelf direction and number of lift levels for each stop.\n'
            '4. Run the program for that robot or save it as a reusable routine.\n'
            '5. RViz click mode is only for quick manual paths now, not the main workflow.'
        )
        label = tk.Label(
            frame,
            text=help_text,
            justify='left',
            anchor='nw',
            font=('Arial', 10),
            bg='#eef3f7',
        )
        label.pack(fill='both', expand=True)

    def create_robot_area(self, parent):
        container = ttk.LabelFrame(parent, text='Robot Missions', padding=10)
        container.pack(fill='both', expand=True, padx=10, pady=5)

        self.robot_canvas = tk.Canvas(container, highlightthickness=0)
        scrollbar = ttk.Scrollbar(container, orient='vertical', command=self.robot_canvas.yview)
        self.robots_frame = ttk.Frame(self.robot_canvas)

        self.robots_frame.bind(
            '<Configure>',
            lambda event: self.robot_canvas.configure(scrollregion=self.robot_canvas.bbox('all')),
        )
        self.robot_canvas.create_window((0, 0), window=self.robots_frame, anchor='nw')
        self.robot_canvas.configure(yscrollcommand=scrollbar.set)
        self.robot_canvas.pack(side='left', fill='both', expand=True)
        scrollbar.pack(side='right', fill='y')

    def create_robot_status_widget(self, robot_id, robot_data):
        card = ttk.LabelFrame(
            self.robots_frame,
            text=f'Robot_{robot_id} ({robot_data.get("lane_name", "Unknown")})',
            padding=10,
        )
        card.pack(fill='x', pady=5)

        pos_label = tk.Label(card, text='Position: X=0.00, Y=0.00, Z=0.00', font=('Arial', 9))
        pos_label.pack(anchor='w')
        mission_label = tk.Label(card, text='Current Mission: None (Idle)', font=('Arial', 9))
        mission_label.pack(anchor='w')
        progress_bar = ttk.Progressbar(card, length=320, mode='determinate', value=0)
        progress_bar.pack(fill='x', pady=5)
        wp_label = tk.Label(card, text='Waypoint: 0/0', font=('Arial', 8))
        wp_label.pack(anchor='w')

        program_frame = ttk.LabelFrame(card, text='Program Route', padding=8)
        program_frame.pack(fill='x', pady=(6, 0))

        program_vars = {
            'routine_name': tk.StringVar(value=f'robot_{robot_id}_routine'),
            'saved_routine': tk.StringVar(value=''),
            'lane_choice': tk.StringVar(value=self._lane_choice_from_name(robot_data.get('lane_name'))),
            'waypoint_count': tk.IntVar(value=3),
            'start_x': tk.DoubleVar(value=self._default_start_x()),
            'spacing': tk.DoubleVar(value=1.0),
            'scan_duration': tk.DoubleVar(value=2.0),
        }

        top_row = ttk.Frame(program_frame)
        top_row.pack(fill='x', pady=2)
        tk.Label(top_row, text='Routine:').pack(side='left')
        ttk.Entry(top_row, textvariable=program_vars['routine_name'], width=18).pack(side='left', padx=4)
        tk.Label(top_row, text='Saved:').pack(side='left', padx=(10, 0))
        saved_combo = ttk.Combobox(
            top_row,
            textvariable=program_vars['saved_routine'],
            values=self._saved_routine_names(),
            state='readonly',
            width=18,
        )
        saved_combo.pack(side='left', padx=4)
        tk.Button(top_row, text='Load', command=lambda: self.load_saved_routine(robot_id)).pack(side='left', padx=2)
        tk.Button(top_row, text='Save Routine', command=lambda: self.save_program_routine(robot_id)).pack(side='left', padx=2)

        config_row = ttk.Frame(program_frame)
        config_row.pack(fill='x', pady=2)
        tk.Label(config_row, text='Lane:').pack(side='left')
        ttk.Combobox(
            config_row,
            textvariable=program_vars['lane_choice'],
            values=[label for label, _ in self.lane_choices],
            state='readonly',
            width=18,
        ).pack(side='left', padx=4)
        tk.Label(config_row, text='Stops:').pack(side='left')
        tk.Spinbox(config_row, from_=1, to=8, textvariable=program_vars['waypoint_count'], width=4).pack(side='left', padx=4)
        tk.Label(config_row, text='Start X:').pack(side='left')
        ttk.Entry(config_row, textvariable=program_vars['start_x'], width=7).pack(side='left', padx=4)
        tk.Label(config_row, text='Spacing:').pack(side='left')
        ttk.Entry(config_row, textvariable=program_vars['spacing'], width=7).pack(side='left', padx=4)
        tk.Label(config_row, text='Scan s:').pack(side='left')
        ttk.Entry(config_row, textvariable=program_vars['scan_duration'], width=7).pack(side='left', padx=4)
        tk.Button(config_row, text='Generate Stops', command=lambda: self.generate_program_rows(robot_id)).pack(side='left', padx=4)

        rows_frame = ttk.Frame(program_frame)
        rows_frame.pack(fill='x', pady=(6, 2))

        preview_label = tk.Label(
            program_frame,
            text='No routine generated yet.',
            justify='left',
            anchor='w',
            font=('Arial', 9),
        )
        preview_label.pack(fill='x', pady=(4, 0))

        controls = ttk.Frame(program_frame)
        controls.pack(fill='x', pady=5)
        tk.Button(controls, text='Run Program', command=lambda: self.run_program(robot_id)).pack(side='left', padx=2)
        tk.Button(controls, text='Pause', command=lambda: self.pause_mission(robot_id)).pack(side='left', padx=2)
        tk.Button(controls, text='Resume', command=lambda: self.resume_mission(robot_id)).pack(side='left', padx=2)
        tk.Button(controls, text='Cancel', command=lambda: self.cancel_mission(robot_id)).pack(side='left', padx=2)
        tk.Button(controls, text='Remove', command=lambda: self.remove_robot(robot_id)).pack(side='left', padx=2)

        self.robot_status_widgets[robot_id] = {
            'card': card,
            'pos_label': pos_label,
            'mission_label': mission_label,
            'progress_bar': progress_bar,
            'wp_label': wp_label,
            'program_vars': program_vars,
            'stop_rows_frame': rows_frame,
            'stop_rows': [],
            'preview_label': preview_label,
            'saved_routine_combo': saved_combo,
        }
        self.generate_program_rows(robot_id)

    def update_robot_display(self):
        active_robot_ids = set(self.latest_status.keys())
        stale_widgets = [robot_id for robot_id in self.robot_status_widgets if robot_id not in active_robot_ids]
        for robot_id in stale_widgets:
            self.robot_status_widgets[robot_id]['card'].destroy()
            self.robot_status_widgets.pop(robot_id, None)

        for robot_id, robot_data in sorted(self.latest_status.items()):
            if robot_id not in self.robot_status_widgets:
                self.create_robot_status_widget(robot_id, robot_data)
            widgets = self.robot_status_widgets[robot_id]
            widgets['card'].configure(text=f'Robot_{robot_id} ({robot_data.get("lane_name", "Unknown")})')
            widgets['pos_label'].configure(
                text=(
                    f'Position: X={robot_data.get("x", 0.0):.2f}, '
                    f'Y={robot_data.get("y", 0.0):.2f}, '
                    f'Z={robot_data.get("z", 0.0):.2f}'
                )
            )
            mission_name = robot_data.get('mission') or 'None (Idle)'
            widgets['mission_label'].configure(text=f'Current Mission: {mission_name}')
            widgets['progress_bar']['value'] = robot_data.get('progress_pct', 0.0)
            widgets['wp_label'].configure(
                text=(
                    f'Waypoint: {robot_data.get("waypoints_done", 0)}/'
                    f'{robot_data.get("total_waypoints", 0)}  '
                    f'State: {robot_data.get("robot_state", "IDLE")}'
                )
            )

    def spawn_fleet(self):
        for robot_id in range(self.num_robots_var.get()):
            request = SpawnRobot.Request()
            request.robot_id = robot_id
            request.lane_id = min(robot_id * 2, 4)
            future = self.spawn_robot_client.call_async(request)
            future.add_done_callback(lambda fut, rid=robot_id: self._handle_spawn_response(fut, rid))

    def clear_all_robots(self):
        for robot_id in list(self.latest_status.keys()):
            request = DespawnRobot.Request()
            request.robot_id = robot_id
            future = self.despawn_robot_client.call_async(request)
            future.add_done_callback(lambda fut, rid=robot_id: self._handle_despawn_response(fut, rid))

    def spawn_single_robot(self):
        lane_map = {
            'North': 0,
            'Mid-North': 1,
            'Center': 2,
            'Mid-South': 3,
            'South': 4,
        }
        request = SpawnRobot.Request()
        request.robot_id = self.single_robot_id_var.get()
        request.lane_id = lane_map.get(self.lane_var.get(), 2)
        self.selected_robot_var.set(request.robot_id)
        self._update_rviz_status_label()
        future = self.spawn_robot_client.call_async(request)
        future.add_done_callback(lambda fut: self._handle_spawn_response(fut, request.robot_id))

    def assign_mission(self, robot_id, mission_name, custom_waypoints=None):
        request = AssignMission.Request()
        request.robot_id = int(robot_id)
        request.mission_name = mission_name
        if custom_waypoints:
            request.custom_waypoints = custom_waypoints
        future = self.assign_mission_client.call_async(request)
        future.add_done_callback(self._handle_assign_response)

    def generate_program_rows(self, robot_id):
        widgets = self.robot_status_widgets[robot_id]
        program_vars = widgets['program_vars']
        count = max(1, int(program_vars['waypoint_count'].get()))
        start_x = float(program_vars['start_x'].get())
        spacing = float(program_vars['spacing'].get())

        for child in widgets['stop_rows_frame'].winfo_children():
            child.destroy()

        header = ttk.Frame(widgets['stop_rows_frame'])
        header.pack(fill='x', pady=(0, 2))
        tk.Label(header, text='Stop', width=6, anchor='w').pack(side='left')
        tk.Label(header, text='X', width=8, anchor='w').pack(side='left')
        tk.Label(header, text='Shelf Dir', width=12, anchor='w').pack(side='left')
        tk.Label(header, text='Levels', width=8, anchor='w').pack(side='left')

        stop_rows = []
        for index in range(count):
            row_frame = ttk.Frame(widgets['stop_rows_frame'])
            row_frame.pack(fill='x', pady=1)
            tk.Label(row_frame, text=f'{index + 1}', width=6, anchor='w').pack(side='left')

            x_var = tk.DoubleVar(value=start_x + index * spacing)
            direction_var = tk.StringVar(value='left')
            levels_var = tk.IntVar(value=3)

            ttk.Entry(row_frame, textvariable=x_var, width=8).pack(side='left')
            ttk.Combobox(
                row_frame,
                textvariable=direction_var,
                values=['left', 'right', 'forward', 'back'],
                state='readonly',
                width=10,
            ).pack(side='left', padx=4)
            tk.Spinbox(row_frame, from_=1, to=3, textvariable=levels_var, width=5).pack(side='left')

            stop_rows.append(
                {
                    'x': x_var,
                    'direction': direction_var,
                    'levels': levels_var,
                }
            )

        widgets['stop_rows'] = stop_rows
        self._update_program_preview(robot_id)

    def _routine_config_from_widgets(self, robot_id):
        widgets = self.robot_status_widgets[robot_id]
        program_vars = widgets['program_vars']
        lane_id = self.lane_display_to_id.get(program_vars['lane_choice'].get(), 0)
        return {
            'lane_id': lane_id,
            'start_x': float(program_vars['start_x'].get()),
            'spacing': float(program_vars['spacing'].get()),
            'scan_duration': float(program_vars['scan_duration'].get()),
            'waypoints': [
                {
                    'x': float(row['x'].get()),
                    'direction': row['direction'].get(),
                    'levels': int(row['levels'].get()),
                }
                for row in widgets['stop_rows']
            ],
        }

    def _build_program_poses(self, robot_id):
        config = self._routine_config_from_widgets(robot_id)
        lane_y = self._lane_center_y(config['lane_id'])
        scan_duration = float(config['scan_duration'])
        poses = []
        for waypoint in config['waypoints']:
            x_pos = float(waypoint['x'])
            direction = waypoint.get('direction', 'left')
            levels = max(1, int(waypoint.get('levels', 1)))
            yaw = self._scan_yaw_for_direction(direction)
            for level in range(1, levels + 1):
                pose = Pose()
                pose.position.x = x_pos
                pose.position.y = lane_y
                pose.position.z = 0.0
                pose.orientation.x = self._lift_height_for_level(level)
                pose.orientation.y = scan_duration
                pose.orientation.z = math.sin(0.5 * yaw)
                pose.orientation.w = math.cos(0.5 * yaw)
                poses.append(pose)
        return poses

    def _update_program_preview(self, robot_id):
        widgets = self.robot_status_widgets[robot_id]
        config = self._routine_config_from_widgets(robot_id)
        total_scans = sum(max(1, int(waypoint['levels'])) for waypoint in config['waypoints'])
        x_values = ', '.join(f'{float(waypoint["x"]):.2f}' for waypoint in config['waypoints'])
        widgets['preview_label'].configure(
            text=(
                f'Lane {config["lane_id"] + 1} | shelf stops: {len(config["waypoints"])} | '
                f'total scan points: {total_scans} | scan hold: {config["scan_duration"]:.1f}s\n'
                f'X array: {x_values}'
            )
        )

    def run_program(self, robot_id):
        if robot_id not in self.latest_status:
            messagebox.showwarning('Run Program', f'robot_{robot_id} is not active')
            return
        poses = self._build_program_poses(robot_id)
        if not poses:
            messagebox.showwarning('Run Program', 'Generate at least one shelf stop first')
            return
        routine_name = self.robot_status_widgets[robot_id]['program_vars']['routine_name'].get().strip()
        self.assign_mission(robot_id, routine_name or f'robot_{robot_id}_routine', poses)
        self._update_program_preview(robot_id)

    def save_program_routine(self, robot_id):
        widgets = self.robot_status_widgets[robot_id]
        routine_name = widgets['program_vars']['routine_name'].get().strip()
        if not routine_name:
            messagebox.showwarning('Save Routine', 'Routine name is required')
            return
        self.saved_routines[routine_name] = self._routine_config_from_widgets(robot_id)
        self.missions_config['saved_routines'] = self.saved_routines
        self._write_missions_config()
        widgets['program_vars']['saved_routine'].set(routine_name)
        self._refresh_saved_routine_selectors()
        messagebox.showinfo('Save Routine', f'Saved routine: {routine_name}')

    def load_saved_routine(self, robot_id):
        widgets = self.robot_status_widgets[robot_id]
        routine_name = widgets['program_vars']['saved_routine'].get().strip()
        routine = self.saved_routines.get(routine_name)
        if not routine:
            return
        program_vars = widgets['program_vars']
        program_vars['routine_name'].set(routine_name)
        program_vars['lane_choice'].set(self.lane_id_to_display.get(int(routine.get('lane_id', 0)), self.lane_choices[0][0]))
        program_vars['start_x'].set(float(routine.get('start_x', self._default_start_x())))
        program_vars['spacing'].set(float(routine.get('spacing', 1.0)))
        program_vars['scan_duration'].set(float(routine.get('scan_duration', 2.0)))
        waypoints = routine.get('waypoints', [])
        program_vars['waypoint_count'].set(max(1, len(waypoints)))
        self.generate_program_rows(robot_id)
        for row_vars, waypoint in zip(widgets['stop_rows'], waypoints):
            row_vars['x'].set(float(waypoint.get('x', self._default_start_x())))
            row_vars['direction'].set(waypoint.get('direction', 'left'))
            row_vars['levels'].set(int(waypoint.get('levels', 3)))
        self._update_program_preview(robot_id)

    def pause_mission(self, robot_id):
        pub = self._mission_control_pub(robot_id)
        pub.publish(String(data='pause'))

    def resume_mission(self, robot_id):
        pub = self._mission_control_pub(robot_id)
        pub.publish(String(data='resume'))

    def cancel_mission(self, robot_id):
        pub = self._mission_control_pub(robot_id)
        pub.publish(String(data='cancel'))

    def remove_robot(self, robot_id):
        self.rviz_waypoint_drafts.pop(int(robot_id), None)
        self._update_rviz_status_label()
        request = DespawnRobot.Request()
        request.robot_id = int(robot_id)
        future = self.despawn_robot_client.call_async(request)
        future.add_done_callback(lambda fut, rid=robot_id: self._handle_despawn_response(fut, rid))

    def select_target_robot(self):
        robot_id = int(self.selected_robot_var.get())
        self.rviz_command_pub.publish(String(data=f'select {robot_id}'))
        self._update_rviz_status_label()

    def set_rviz_mode(self, mode):
        self.rviz_mode_var.set(mode)
        self.rviz_command_pub.publish(String(data=f'mode {mode}'))
        self._update_rviz_status_label()

    def commit_rviz_waypoints(self):
        robot_id = int(self.selected_robot_var.get())
        draft_waypoints = self.rviz_waypoint_drafts.get(robot_id, [])
        if draft_waypoints:
            poses = []
            for x_pos, y_pos, z_pos in draft_waypoints:
                pose = Pose()
                pose.position.x = x_pos
                pose.position.y = y_pos
                pose.position.z = z_pos
                pose.orientation.w = 1.0
                poses.append(pose)
            self.assign_mission(robot_id, f'rviz_robot_{robot_id}', poses)
            self.rviz_waypoint_drafts.pop(robot_id, None)
            self.rviz_command_pub.publish(String(data=f'clear_waypoints {robot_id}'))
            self._update_rviz_status_label()
            return
        self.rviz_command_pub.publish(String(data=f'commit_waypoints {robot_id}'))

    def clear_rviz_waypoints(self):
        robot_id = int(self.selected_robot_var.get())
        self.rviz_waypoint_drafts.pop(robot_id, None)
        self.rviz_command_pub.publish(String(data=f'clear_waypoints {robot_id}'))
        self._update_rviz_status_label()

    def remove_selected_robot(self):
        robot_id = int(self.selected_robot_var.get())
        self.remove_robot(robot_id)

    def on_clicked_point(self, msg):
        if self.rviz_mode_var.get() != 'waypoint':
            return
        robot_id = int(self.selected_robot_var.get())
        if robot_id not in self.latest_status:
            return
        self.rviz_waypoint_drafts.setdefault(robot_id, []).append(
            (float(msg.point.x), float(msg.point.y), float(msg.point.z))
        )
        self._update_rviz_status_label()

    def on_fleet_status(self, msg):
        self.latest_status = {}
        for robot in msg.robots:
            self.latest_status[int(robot.robot_id)] = {
                'lane_name': robot.lane_name,
                'x': robot.position.x,
                'y': robot.position.y,
                'z': robot.position.z,
                'mission': robot.current_mission,
                'waypoints_done': robot.waypoints_completed,
                'total_waypoints': robot.total_waypoints,
                'progress_pct': robot.mission_progress_pct,
                'robot_state': robot.robot_state,
            }
        active_robot_ids = set(self.latest_status.keys())
        for robot_id in list(self.rviz_waypoint_drafts.keys()):
            if robot_id not in active_robot_ids:
                self.rviz_waypoint_drafts.pop(robot_id, None)
        if self.rviz_mode_var.get() == 'spawn':
            new_robot_ids = sorted(active_robot_ids - self.previous_active_robot_ids)
            if new_robot_ids:
                self.selected_robot_var.set(new_robot_ids[-1])
                self.rviz_mode_var.set('waypoint')
        if self.latest_status and int(self.selected_robot_var.get()) not in self.latest_status:
            self.selected_robot_var.set(sorted(self.latest_status.keys())[0])
        self.previous_active_robot_ids = active_robot_ids
        self.header_label.configure(
            text=(
                f'Warehouse Fleet Manager  |  Active Robots: {len(msg.robots)}  |  '
                f'Status: {msg.fleet_mode}'
            )
        )
        self._update_rviz_status_label()

    def create_obstacle_frame(self, parent):
        frame = ttk.LabelFrame(parent, text='Spawn Obstacles', padding=10)
        frame.pack(fill='x', padx=10, pady=5)

        button_rows = [
            [('static_small', 'Static Small'), ('static_medium', 'Static Medium'), ('static_large', 'Static Large')],
            [('dynamic_small', 'Dynamic Small'), ('dynamic_medium', 'Dynamic Medium'), ('dynamic_large', 'Dynamic Large')],
        ]
        for row in button_rows:
            line = ttk.Frame(frame)
            line.pack(fill='x', pady=3)
            for preset_key, label in row:
                tk.Button(line, text=label, command=lambda key=preset_key: self._send_preset(key)).pack(
                    side='left', expand=True, fill='x', padx=2
                )

        cmd_line = ttk.Frame(frame)
        cmd_line.pack(fill='x', pady=3)
        for command_key, label in [
            ('remove_last', 'Remove Last'),
            ('clear_all', 'Clear All'),
            ('pause_dynamic', 'Pause Dynamic'),
            ('resume_dynamic', 'Resume Dynamic'),
        ]:
            tk.Button(cmd_line, text=label, command=lambda key=command_key: self._send_command(key)).pack(
                side='left', expand=True, fill='x', padx=2
            )

    def _send_preset(self, preset):
        self.preset_pub.publish(String(data=preset))

    def _send_command(self, command):
        self.command_pub.publish(String(data=command))

    def _update_rviz_status_label(self):
        robot_id = int(self.selected_robot_var.get())
        draft_count = len(self.rviz_waypoint_drafts.get(robot_id, []))
        self.rviz_status_label.configure(
            text=(
                f'RViz mode: {self.rviz_mode_var.get()} | '
                f'target: robot_{robot_id} | '
                f'draft points: {draft_count}'
            )
        )

    def _mission_control_pub(self, robot_id):
        if robot_id not in self.mission_control_pubs:
            self.mission_control_pubs[robot_id] = self.create_publisher(
                String,
                f'/robot_{robot_id}/mission_control',
                10,
            )
        return self.mission_control_pubs[robot_id]

    def _handle_spawn_response(self, future, robot_id):
        try:
            result = future.result()
        except Exception as exc:
            messagebox.showerror('Spawn Robot', f'robot_{robot_id}: {exc}')
            return
        if result and not result.success:
            messagebox.showwarning('Spawn Robot', result.message)

    def _handle_despawn_response(self, future, robot_id):
        try:
            result = future.result()
        except Exception as exc:
            messagebox.showerror('Despawn Robot', f'robot_{robot_id}: {exc}')
            return
        if result and not result.success:
            messagebox.showwarning('Despawn Robot', result.message)

    def _handle_assign_response(self, future):
        try:
            result = future.result()
        except Exception as exc:
            messagebox.showerror('Assign Mission', str(exc))
            return
        if result and not result.success:
            messagebox.showwarning('Assign Mission', result.message)

    def _pump_tk(self):
        try:
            self.root.update_idletasks()
            self.root.update()
        except tk.TclError:
            pass

    def _close(self):
        self.root.destroy()
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WarehouseFleetSpawnPanel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()