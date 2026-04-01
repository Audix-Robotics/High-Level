#!/usr/bin/env python3

import json
import math
import os
import tkinter as tk
from tkinter import messagebox, ttk

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped, Pose
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from audix.msg import FleetStatus
from audix.srv import AssignMission, DespawnRobot, GetRobotStatus, SpawnRobot


class WarehouseFleetSpawnPanel(Node):
    MODE_LABELS = {
        'lane_inspection': 'Lane Inspection',
        'free_roam': 'Free Roam',
    }

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
        self.latest_status = {}
        self.robot_status_widgets = {}
        self.previous_active_robot_ids = set()
        self.rviz_waypoint_drafts = {}
        self.spawn_lane_choices = self._build_lane_choices(include_default=False)
        self.mission_lane_choices = self._build_lane_choices(include_default=True)
        self.spawn_lane_display_to_id = {label: lane_id for label, lane_id in self.spawn_lane_choices}
        self.lane_display_to_id = {label: lane_id for label, lane_id in self.mission_lane_choices}
        self.lane_id_to_display = {lane_id: label for label, lane_id in self.mission_lane_choices}

        self.spawn_robot_client = self.create_client(SpawnRobot, '/fleet/spawn_robot')
        self.assign_mission_client = self.create_client(AssignMission, '/fleet/assign_mission')
        self.get_status_client = self.create_client(GetRobotStatus, '/fleet/get_robot_status')
        self.despawn_robot_client = self.create_client(DespawnRobot, '/fleet/despawn_robot')

        self.create_subscription(FleetStatus, '/fleet/status', self.on_fleet_status, 10)
        self.create_subscription(PointStamped, '/clicked_point', self.on_clicked_point, 10)

        self.preset_pub = self.create_publisher(String, '/arena_spawn_preset', 10)
        self.command_pub = self.create_publisher(String, '/arena_spawn_command', 10)
        self.obstacle_click_pub = self.create_publisher(PointStamped, '/arena_clicked_point', 10)
        self.rviz_command_pub = self.create_publisher(String, '/fleet/rviz_command', 10)
        self.mission_control_pubs = {}
        self.robot_enable_pubs = {}

        self.root = tk.Tk()
        self.root.title('Audix Warehouse Fleet Manager')
        self.root.geometry('1280x880')
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
        default_lane_choice = self.lane_id_to_display.get(2)
        if default_lane_choice is None and self.spawn_lane_choices:
            default_lane_choice = self.spawn_lane_choices[0][0]
        self.spawn_lane_var = tk.StringVar(value=default_lane_choice or '')

        self.create_fleet_header_frame()
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill='both', expand=True, padx=10, pady=6)

        self.operations_tab = ttk.Frame(self.notebook)
        self.obstacles_tab = ttk.Frame(self.notebook)

        self.notebook.add(self.operations_tab, text='Fleet Control')
        self.notebook.add(self.obstacles_tab, text='Obstacles')
        self.notebook.bind('<<NotebookTabChanged>>', self._on_tab_changed)

        self.create_spawn_controls_frame(self.operations_tab)
        self.create_quick_help_frame(self.operations_tab)
        self.create_robot_area(self.operations_tab)
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

    def _build_lane_choices(self, include_default=False):
        choices = []
        if include_default:
            choices.append(('Default', -1))
        for lane_key, lane_cfg in sorted(
            self.warehouse_config.get('lanes', {}).items(),
            key=lambda item: int(item[0].split('_')[-1]),
        ):
            lane_id = int(lane_key.split('_')[-1])
            operator_name = lane_cfg.get('operator_name', f'Lane {lane_id + 1}')
            choices.append((operator_name, lane_id))
        return choices

    def _mode_display_names(self):
        return [self.MODE_LABELS[key] for key in ('lane_inspection', 'free_roam')]

    def _mode_key_from_display(self, display_name):
        for key, value in self.MODE_LABELS.items():
            if value == display_name:
                return key
        return 'lane_inspection'

    def _lane_choice_from_name(self, lane_name):
        lane_name = (lane_name or '').strip().lower()
        for label, lane_id in self.mission_lane_choices:
            if lane_id < 0:
                continue
            lane_cfg = self.warehouse_config.get('lanes', {}).get(f'lane_{lane_id}', {})
            if lane_cfg.get('operator_name', '').strip().lower() == lane_name:
                return label
        return self.mission_lane_choices[1][0] if len(self.mission_lane_choices) > 1 else 'Lane 1'

    def _lane_config(self, lane_id):
        return self.warehouse_config.get('lanes', {}).get(f'lane_{lane_id}', {})

    def _default_scan_heights(self):
        scan_levels = self.warehouse_config.get('scan_levels', {})
        return [
            float(scan_levels.get('low', 0.05)),
            float(scan_levels.get('mid', 0.17)),
            float(scan_levels.get('high', 0.34)),
        ]

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
            text='Spawn robots into lanes, keep them idle, then start either a compact lane inspection or a free-roam mission when you are ready.',
            font=('Arial', 10),
            anchor='w',
        )
        self.header_hint.pack(fill='x', pady=(6, 0))

    def create_spawn_controls_frame(self, parent):
        frame = ttk.LabelFrame(parent, text='Fleet Control', padding=10)
        frame.pack(fill='x', padx=10, pady=5)

        tk.Label(frame, text='Number of Robots:').pack(side='left')
        self.num_robots_var = tk.IntVar(value=1)
        tk.Spinbox(frame, from_=1, to=6, textvariable=self.num_robots_var, width=5).pack(side='left', padx=5)
        tk.Button(frame, text='Spawn Fleet', command=self.spawn_fleet).pack(side='left', padx=5)
        tk.Button(frame, text='Clear All Robots', command=self.clear_all_robots).pack(side='left', padx=5)

        tk.Label(frame, text='  OR  Robot ID:').pack(side='left', padx=20)
        self.single_robot_id_var = tk.IntVar(value=0)
        tk.Spinbox(frame, from_=0, to=10, textvariable=self.single_robot_id_var, width=5).pack(side='left', padx=5)

        tk.Label(frame, text='Lane:').pack(side='left')
        ttk.Combobox(
            frame,
            textvariable=self.spawn_lane_var,
            values=[label for label, _ in self.spawn_lane_choices],
            state='readonly',
            width=10,
        ).pack(side='left', padx=5)
        tk.Button(frame, text='Spawn Robot', command=self.spawn_single_robot).pack(side='left', padx=5)

        self.rviz_status_label = tk.Label(
            parent,
            text='RViz capture: idle | target: robot_0 | points: 0',
            font=('Arial', 10),
            anchor='w',
            bg='#eef3f7',
        )
        self.rviz_status_label.pack(fill='x', padx=14, pady=(0, 4))

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

        tk.Button(frame, text='Preview Draft', command=self.preview_selected_robot).pack(side='left', padx=4)
        tk.Button(frame, text='Start Mission', command=self.commit_rviz_waypoints).pack(side='left', padx=4)
        tk.Button(frame, text='Clear Draft', command=self.clear_rviz_waypoints).pack(side='left', padx=4)
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
        frame = ttk.LabelFrame(parent, text='Workflow', padding=10)
        frame.pack(fill='x', padx=10, pady=5)

        help_text = (
            'Each spawned robot stays idle. Use the compact card below to pick Lane Inspection or Free Roam. '
            'Free Roam is the only mode that enables RViz waypoint capture.'
        )
        label = tk.Label(
            frame,
            text=help_text,
            justify='left',
            anchor='nw',
            font=('Arial', 10),
            bg='#eef3f7',
        )
        label.pack(fill='x')

    def create_robot_area(self, parent):
        container = ttk.LabelFrame(parent, text='Mission Control', padding=10)
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
            padding=8,
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

        workflow_frame = ttk.LabelFrame(card, text='Mission', padding=8)
        workflow_frame.pack(fill='x', pady=(6, 0))

        default_lane = self._lane_choice_from_name(robot_data.get('lane_name'))
        program_vars = {
            'mode': tk.StringVar(value=self.MODE_LABELS['lane_inspection']),
            'lane_choice': tk.StringVar(value=default_lane),
            'stop_count': tk.IntVar(value=3),
            'scan_levels': tk.IntVar(value=3),
        }

        top_row = ttk.Frame(workflow_frame)
        top_row.pack(fill='x', pady=2)
        tk.Label(top_row, text='Mode:').pack(side='left')
        mode_combo = ttk.Combobox(
            top_row,
            textvariable=program_vars['mode'],
            values=self._mode_display_names(),
            state='readonly',
            width=16,
        )
        mode_combo.pack(side='left', padx=4)
        tk.Label(top_row, text='Lane:').pack(side='left', padx=(12, 0))
        lane_combo = ttk.Combobox(
            top_row,
            textvariable=program_vars['lane_choice'],
            values=[label for label, _ in self.mission_lane_choices],
            state='readonly',
            width=10,
        )
        lane_combo.pack(side='left', padx=4)

        config_row = ttk.Frame(workflow_frame)
        config_row.pack(fill='x', pady=2)
        tk.Label(config_row, text='Stops:').pack(side='left')
        stop_spin = tk.Spinbox(config_row, from_=1, to=8, textvariable=program_vars['stop_count'], width=4)
        stop_spin.pack(side='left', padx=4)
        tk.Label(config_row, text='Levels:').pack(side='left')
        levels_spin = tk.Spinbox(config_row, from_=1, to=3, textvariable=program_vars['scan_levels'], width=4)
        levels_spin.pack(side='left', padx=4)
        levels_hint = ', '.join(f'{height:.2f}m' for height in self._default_scan_heights())
        tk.Label(config_row, text=f'Heights: {levels_hint}', font=('Arial', 9)).pack(side='left', padx=(10, 0))

        preview_label = tk.Label(
            workflow_frame,
            text='Idle.',
            justify='left',
            anchor='w',
            font=('Arial', 9),
        )
        preview_label.pack(fill='x', pady=(4, 0))

        free_roam_frame = ttk.LabelFrame(workflow_frame, text='Free Roam', padding=6)
        free_roam_frame.pack(fill='x', pady=(6, 0))
        listbox = tk.Listbox(free_roam_frame, height=5, exportselection=False)
        listbox.pack(side='left', fill='x', expand=True)
        list_controls = ttk.Frame(free_roam_frame)
        list_controls.pack(side='left', fill='y', padx=(8, 0))
        capture_button = tk.Button(
            list_controls,
            text='Capture RViz',
            command=lambda: self.toggle_free_roam_capture(robot_id),
        )
        capture_button.pack(fill='x', pady=1)
        tk.Button(list_controls, text='Up', command=lambda: self.move_rviz_waypoint(robot_id, -1)).pack(fill='x', pady=1)
        tk.Button(list_controls, text='Down', command=lambda: self.move_rviz_waypoint(robot_id, 1)).pack(fill='x', pady=1)
        tk.Button(list_controls, text='Delete', command=lambda: self.delete_rviz_waypoint(robot_id)).pack(fill='x', pady=1)
        tk.Button(list_controls, text='Clear', command=lambda: self.clear_robot_draft(robot_id)).pack(fill='x', pady=1)

        controls = ttk.Frame(workflow_frame)
        controls.pack(fill='x', pady=5)
        tk.Button(controls, text='Start Mission', command=lambda: self.start_mission(robot_id)).pack(side='left', padx=2)
        tk.Button(controls, text='Pause', command=lambda: self.pause_mission(robot_id)).pack(side='left', padx=2)
        tk.Button(controls, text='Resume', command=lambda: self.resume_mission(robot_id)).pack(side='left', padx=2)
        tk.Button(controls, text='Cancel', command=lambda: self.cancel_mission(robot_id)).pack(side='left', padx=2)
        tk.Button(controls, text='Remove', command=lambda: self.remove_robot(robot_id)).pack(side='left', padx=2)

        mode_combo.bind('<<ComboboxSelected>>', lambda _event, rid=robot_id: self._update_robot_mode_ui(rid))
        lane_combo.bind('<<ComboboxSelected>>', lambda _event, rid=robot_id: self._update_robot_mode_ui(rid))

        self.robot_status_widgets[robot_id] = {
            'card': card,
            'pos_label': pos_label,
            'mission_label': mission_label,
            'progress_bar': progress_bar,
            'wp_label': wp_label,
            'program_vars': program_vars,
            'preview_label': preview_label,
            'free_roam_listbox': listbox,
            'free_roam_frame': free_roam_frame,
            'capture_button': capture_button,
            'lane_combo': lane_combo,
            'stop_spin': stop_spin,
            'levels_spin': levels_spin,
        }
        self._refresh_rviz_listbox(robot_id)
        self._update_robot_mode_ui(robot_id)

    def _update_robot_mode_ui(self, robot_id):
        widgets = self.robot_status_widgets.get(robot_id)
        if widgets is None:
            return
        mode_key = self._mode_key_from_display(widgets['program_vars']['mode'].get())
        lane_id = self.lane_display_to_id.get(widgets['program_vars']['lane_choice'].get(), 0)
        free_roam_frame = widgets['free_roam_frame']
        widgets['lane_combo'].configure(state='readonly' if mode_key == 'lane_inspection' else 'disabled')
        widgets['stop_spin'].configure(state='normal' if mode_key == 'lane_inspection' and lane_id >= 0 else 'disabled')
        widgets['levels_spin'].configure(state='normal' if mode_key == 'lane_inspection' else 'disabled')
        if mode_key == 'free_roam':
            free_roam_frame.pack(fill='x', pady=(6, 0))
        else:
            free_roam_frame.pack_forget()
            if self.rviz_mode_var.get() == 'waypoint' and int(self.selected_robot_var.get()) == int(robot_id):
                self.set_rviz_mode('idle')
        self._update_program_preview(robot_id)
        self._update_capture_button(robot_id)

    def _update_capture_button(self, robot_id):
        widgets = self.robot_status_widgets.get(robot_id)
        if widgets is None:
            return
        capturing = self.rviz_mode_var.get() == 'waypoint' and int(self.selected_robot_var.get()) == int(robot_id)
        widgets['capture_button'].configure(text='Stop Capture' if capturing else 'Capture RViz')

    def toggle_free_roam_capture(self, robot_id):
        widgets = self.robot_status_widgets.get(robot_id)
        if widgets is None:
            return
        mode_key = self._mode_key_from_display(widgets['program_vars']['mode'].get())
        if mode_key != 'free_roam':
            widgets['program_vars']['mode'].set(self.MODE_LABELS['free_roam'])
            mode_key = 'free_roam'
        if mode_key != 'free_roam':
            return
        if self.rviz_mode_var.get() == 'waypoint' and int(self.selected_robot_var.get()) == int(robot_id):
            self.set_rviz_mode('idle')
        else:
            self.selected_robot_var.set(int(robot_id))
            self.rviz_command_pub.publish(String(data=f'select {int(robot_id)}'))
            self.set_rviz_mode('waypoint')
        self._update_robot_mode_ui(robot_id)

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
            self._refresh_rviz_listbox(robot_id)
            self._update_capture_button(robot_id)

    def spawn_fleet(self):
        if not self._wait_for_service(self.spawn_robot_client, 'Spawn Robot'):
            return
        lane_count = max(1, len(self.spawn_lane_choices))
        for robot_id in range(self.num_robots_var.get()):
            request = SpawnRobot.Request()
            request.robot_id = robot_id
            request.lane_id = robot_id % lane_count
            future = self.spawn_robot_client.call_async(request)
            future.add_done_callback(lambda fut, rid=robot_id: self._handle_spawn_response(fut, rid))

    def clear_all_robots(self):
        if not self._wait_for_service(self.despawn_robot_client, 'Despawn Robot'):
            return
        for robot_id in list(self.latest_status.keys()):
            request = DespawnRobot.Request()
            request.robot_id = robot_id
            future = self.despawn_robot_client.call_async(request)
            future.add_done_callback(lambda fut, rid=robot_id: self._handle_despawn_response(fut, rid))

    def spawn_single_robot(self):
        if not self._wait_for_service(self.spawn_robot_client, 'Spawn Robot'):
            return
        request = SpawnRobot.Request()
        request.robot_id = self.single_robot_id_var.get()
        request.lane_id = self.spawn_lane_display_to_id.get(self.spawn_lane_var.get(), 2)
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

    def _sanitize_heights(self, robot_id):
        program_vars = self.robot_status_widgets[robot_id]['program_vars']
        count = max(1, min(3, int(program_vars['scan_levels'].get())))
        return self._default_scan_heights()[:count]

    def _lane_descriptor(self, robot_id):
        widgets = self.robot_status_widgets[robot_id]
        program_vars = widgets['program_vars']
        lane_id = self.lane_display_to_id.get(program_vars['lane_choice'].get(), 0)
        if lane_id < 0:
            return self._default_original_descriptor(robot_id)
        lane_cfg = self._lane_config(lane_id)
        stop_count = max(1, int(program_vars['stop_count'].get()))
        scan_heights = self._sanitize_heights(robot_id)
        x_min, x_max = lane_cfg.get('stop_range_x', [-2.0, 2.0])
        if stop_count == 1:
            x_positions = [0.5 * (float(x_min) + float(x_max))]
        else:
            span = float(x_max) - float(x_min)
            x_positions = [float(x_min) + span * index / float(stop_count - 1) for index in range(stop_count)]

        descriptor = {
            'mission_name': f'robot_{robot_id}_lane_{lane_id + 1}',
            'source': 'lane_inspection',
            'mission_mode': 'lane_inspection',
            'lane_id': lane_id,
            'stop_count': stop_count,
            'scan_heights': scan_heights,
            'waypoints': [],
        }

        for x_pos in x_positions:
            for height in scan_heights:
                descriptor['waypoints'].append(
                    {
                        'position': [float(x_pos), float(lane_cfg.get('center_y', 0.0)), 0.0],
                        'lift_height': float(height),
                        'scan_yaw': float(lane_cfg.get('scan_yaw', 0.0)),
                        'dwell_time': 0.0,
                        'position_tolerance': 0.05,
                    }
                )

        return_pad = lane_cfg.get('return_pad', {})
        home_pad = lane_cfg.get('home_pad', {})
        descriptor['waypoints'].append(
            {
                'position': [
                    float(return_pad.get('x', home_pad.get('x', lane_cfg.get('center_x', 0.0)))),
                    float(return_pad.get('y', home_pad.get('y', lane_cfg.get('center_y', 0.0)))),
                    0.0,
                ],
                'dwell_time': 0.0,
                'position_tolerance': 0.05,
            }
        )
        return descriptor

    def _active_lane_id_for_robot(self, robot_id):
        lane_name = self.latest_status.get(robot_id, {}).get('lane_name', '').strip().lower()
        for label, lane_id in self.mission_lane_choices:
            if lane_id < 0:
                continue
            lane_cfg = self._lane_config(lane_id)
            operator_name = lane_cfg.get('operator_name', '').strip().lower()
            if lane_name and lane_name == operator_name:
                return lane_id
        widgets = self.robot_status_widgets.get(robot_id)
        if widgets is not None:
            return self.lane_display_to_id.get(widgets['program_vars']['lane_choice'].get(), 0)
        return 0

    def _default_original_descriptor(self, robot_id):
        scan_heights = self._sanitize_heights(robot_id)
        descriptor = {
            'mission_name': f'robot_{robot_id}_default_original',
            'source': 'default_original',
            'mission_mode': 'default_original',
            'waypoints': [],
        }
        for shelf_key, shelf_cfg in sorted(self.warehouse_config.get('scan_shelves', {}).items()):
            position = shelf_cfg.get('position', [0.0, 0.0, 0.0])
            shelf_yaw = float(shelf_cfg.get('yaw', math.pi / 2.0))
            for height in scan_heights:
                descriptor['waypoints'].append(
                    {
                        'position': [float(position[0]), float(position[1]), float(position[2]) if len(position) > 2 else 0.0],
                        'lift_height': float(height),
                        'scan_yaw': shelf_yaw,
                        'dwell_time': 0.0,
                        'position_tolerance': 0.05,
                    }
                )

        lane_cfg = self._lane_config(self._active_lane_id_for_robot(robot_id))
        return_pad = lane_cfg.get('return_pad', {})
        home_pad = lane_cfg.get('home_pad', {})
        descriptor['waypoints'].append(
            {
                'position': [
                    float(return_pad.get('x', home_pad.get('x', lane_cfg.get('center_x', 0.0)))),
                    float(return_pad.get('y', home_pad.get('y', lane_cfg.get('center_y', 0.0)))),
                    0.0,
                ],
                'dwell_time': 0.0,
                'position_tolerance': 0.05,
            }
        )
        return descriptor

    def _free_roam_descriptor(self, robot_id):
        descriptor = {
            'mission_name': f'robot_{robot_id}_free_roam',
            'source': 'free_roam',
            'mission_mode': 'free_roam',
            'waypoints': [],
        }
        for x_pos, y_pos, z_pos in self.rviz_waypoint_drafts.get(robot_id, []):
            descriptor['waypoints'].append(
                {
                    'position': [x_pos, y_pos, z_pos],
                    'dwell_time': 0.0,
                    'position_tolerance': 0.05,
                }
            )
        return descriptor

    def _descriptor_for_robot(self, robot_id):
        mode_key = self._mode_key_from_display(self.robot_status_widgets[robot_id]['program_vars']['mode'].get())
        if mode_key == 'free_roam':
            return self._free_roam_descriptor(robot_id)
        return self._lane_descriptor(robot_id)

    def _descriptor_to_poses(self, descriptor):
        poses = []
        for waypoint in descriptor.get('waypoints', []):
            pose = Pose()
            position = waypoint.get('position', [0.0, 0.0, 0.0])
            pose.position.x = float(position[0])
            pose.position.y = float(position[1])
            pose.position.z = float(position[2]) if len(position) > 2 else 0.0
            if 'scan_yaw' in waypoint or 'lift_height' in waypoint:
                yaw = float(waypoint.get('scan_yaw', 0.0))
                pose.orientation.x = float(waypoint.get('lift_height', 0.0))
                pose.orientation.y = float(waypoint.get('dwell_time', 0.0))
                pose.orientation.z = math.sin(0.5 * yaw)
                pose.orientation.w = math.cos(0.5 * yaw)
            else:
                pose.orientation.w = 1.0
            poses.append(pose)
        return poses

    def _preview_waypoint_points(self, descriptor):
        points = []
        for waypoint in descriptor.get('waypoints', []):
            position = waypoint.get('position', [0.0, 0.0, 0.0])
            points.append((float(position[0]), float(position[1]), float(position[2]) if len(position) > 2 else 0.0))
        return points

    def _update_program_preview(self, robot_id):
        widgets = self.robot_status_widgets.get(robot_id)
        if widgets is None:
            return
        program_vars = widgets['program_vars']
        mode_key = self._mode_key_from_display(program_vars['mode'].get())
        if mode_key == 'free_roam':
            draft_points = self.rviz_waypoint_drafts.get(robot_id, [])
            widgets['preview_label'].configure(
                text=f'Free Roam | points: {len(draft_points)} | press Capture RViz, click points in RViz, then Start Mission'
            )
            return

        lane_id = self.lane_display_to_id.get(program_vars['lane_choice'].get(), 0)
        if lane_id < 0:
            descriptor = self._default_original_descriptor(robot_id)
            scan_heights = self._sanitize_heights(robot_id)
            shelf_names = ', '.join(
                shelf_cfg.get('name', shelf_key)
                for shelf_key, shelf_cfg in sorted(self.warehouse_config.get('scan_shelves', {}).items())
            )
            return_pad = descriptor['waypoints'][-1]['position'] if descriptor.get('waypoints') else [0.0, 0.0, 0.0]
            widgets['preview_label'].configure(
                text=(
                    f'Default / Original | scan shelves: {shelf_names} | '
                    f'levels: {len(scan_heights)} | heights: {", ".join(f"{height:.2f}" for height in scan_heights)}\n'
                    f'Return pad: ({float(return_pad[0]):.2f}, {float(return_pad[1]):.2f})'
                )
            )
            return

        descriptor = self._lane_descriptor(robot_id)
        lane_cfg = self._lane_config(descriptor['lane_id'])
        scan_heights = descriptor.get('scan_heights', [])
        stride = max(1, len(scan_heights))
        stop_points = descriptor['waypoints'][:-1]
        x_values = ', '.join(f'{stop_points[index]["position"][0]:.2f}' for index in range(0, len(stop_points), stride))
        return_pad = lane_cfg.get('return_pad', {})
        widgets['preview_label'].configure(
            text=(
                f'{lane_cfg.get("operator_name", "Lane")} | stops: {descriptor.get("stop_count", 0)} | '
                f'levels: {len(scan_heights)} | heights: {", ".join(f"{height:.2f}" for height in scan_heights)}\n'
                f'Stops on X: {x_values} | idle pad: '
                f'({float(return_pad.get("x", 0.0)):.2f}, {float(return_pad.get("y", 0.0)):.2f})'
            )
        )

    def preview_mission(self, robot_id):
        if robot_id not in self.latest_status:
            messagebox.showwarning('Preview Mission', f'robot_{robot_id} is not active')
            return
        descriptor = self._descriptor_for_robot(robot_id)
        if not descriptor.get('waypoints'):
            messagebox.showwarning('Preview Mission', 'This mission has no waypoints to preview')
            return
        self._sync_preview_waypoints(robot_id, self._preview_waypoint_points(descriptor))
        self._update_program_preview(robot_id)

    def preview_selected_robot(self):
        self.preview_mission(int(self.selected_robot_var.get()))

    def start_mission(self, robot_id):
        if robot_id not in self.latest_status:
            messagebox.showwarning('Start Mission', f'robot_{robot_id} is not active')
            return
        descriptor = self._descriptor_for_robot(robot_id)
        if not descriptor.get('waypoints'):
            messagebox.showwarning('Start Mission', 'This mission has no waypoints to execute')
            return
        self.assign_mission(
            robot_id,
            descriptor.get('mission_name', f'robot_{robot_id}_mission'),
            self._descriptor_to_poses(descriptor),
        )
        self._set_robot_enabled(robot_id, True)
        self.set_rviz_mode('idle')
        self._sync_preview_waypoints(robot_id, [])

    def pause_mission(self, robot_id):
        pub = self._mission_control_pub(robot_id)
        pub.publish(String(data='pause'))

    def resume_mission(self, robot_id):
        pub = self._mission_control_pub(robot_id)
        pub.publish(String(data='resume'))
        self._set_robot_enabled(robot_id, True)

    def cancel_mission(self, robot_id):
        pub = self._mission_control_pub(robot_id)
        pub.publish(String(data='cancel'))

    def remove_robot(self, robot_id):
        self.rviz_waypoint_drafts.pop(int(robot_id), None)
        self._sync_preview_waypoints(int(robot_id), [])
        self._update_rviz_status_label()
        if not self._wait_for_service(self.despawn_robot_client, 'Despawn Robot'):
            return
        request = DespawnRobot.Request()
        request.robot_id = int(robot_id)
        future = self.despawn_robot_client.call_async(request)
        future.add_done_callback(lambda fut, rid=robot_id: self._handle_despawn_response(fut, rid))

    def _wait_for_service(self, client, action_name):
        if client.wait_for_service(timeout_sec=1.5):
            return True
        messagebox.showerror(action_name, 'Fleet service is not available yet')
        return False

    def _next_available_robot_id(self):
        for robot_id in range(0, 11):
            if robot_id not in self.latest_status:
                return robot_id
        return self.single_robot_id_var.get()

    def select_target_robot(self):
        robot_id = int(self.selected_robot_var.get())
        self.rviz_command_pub.publish(String(data=f'select {robot_id}'))
        self._update_rviz_status_label()

    def set_rviz_mode(self, mode):
        if mode not in {'idle', 'waypoint'}:
            mode = 'idle'
        self.rviz_mode_var.set(mode)
        self.rviz_command_pub.publish(String(data=f'mode {mode}'))
        self._update_rviz_status_label()
        for robot_id in list(self.robot_status_widgets):
            self._update_capture_button(robot_id)

    def commit_rviz_waypoints(self):
        robot_id = int(self.selected_robot_var.get())
        if robot_id in self.robot_status_widgets:
            self.robot_status_widgets[robot_id]['program_vars']['mode'].set(self.MODE_LABELS['free_roam'])
            self._update_program_preview(robot_id)
        self.start_mission(robot_id)

    def clear_rviz_waypoints(self):
        self.clear_robot_draft(int(self.selected_robot_var.get()))
        self._update_rviz_status_label()

    def remove_selected_robot(self):
        robot_id = int(self.selected_robot_var.get())
        self.remove_robot(robot_id)

    def _obstacles_tab_active(self):
        return self.notebook.select() == str(self.obstacles_tab)

    def _on_tab_changed(self, _event=None):
        if self._obstacles_tab_active() and self.rviz_mode_var.get() != 'idle':
            self.rviz_mode_var.set('idle')
            self.rviz_command_pub.publish(String(data='mode idle'))
        self._update_rviz_status_label()

    def on_clicked_point(self, msg):
        if self._obstacles_tab_active():
            self.obstacle_click_pub.publish(msg)
            return
        if self.rviz_mode_var.get() != 'waypoint':
            return
        robot_id = int(self.selected_robot_var.get())
        if robot_id not in self.latest_status:
            return
        self.rviz_waypoint_drafts.setdefault(robot_id, []).append(
            (float(msg.point.x), float(msg.point.y), float(msg.point.z))
        )
        if robot_id in self.robot_status_widgets:
            self.robot_status_widgets[robot_id]['program_vars']['mode'].set(self.MODE_LABELS['free_roam'])
            self._refresh_rviz_listbox(robot_id)
            self._update_program_preview(robot_id)
        self._sync_preview_waypoints(robot_id, self.rviz_waypoint_drafts[robot_id])
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

    def _routine_config_from_widgets(self, robot_id):
        descriptor = self._descriptor_for_robot(robot_id)
        mode_key = self._mode_key_from_display(self.robot_status_widgets[robot_id]['program_vars']['mode'].get())
        config = {
            'mode': mode_key,
            'descriptor': descriptor,
        }
        if mode_key == 'lane_inspection' and descriptor.get('mission_mode') == 'lane_inspection':
            config.update(
                {
                    'lane_id': descriptor.get('lane_id', 0),
                    'stop_count': int(self.robot_status_widgets[robot_id]['program_vars']['stop_count'].get()),
                    'scan_levels': int(self.robot_status_widgets[robot_id]['program_vars']['scan_levels'].get()),
                    'scan_heights': self._sanitize_heights(robot_id),
                }
            )
        if descriptor.get('mission_mode') == 'default_original':
            config['scan_heights'] = self._sanitize_heights(robot_id)
        if mode_key == 'free_roam':
            config['draft_points'] = [list(point) for point in self.rviz_waypoint_drafts.get(robot_id, [])]
        return config

    def _sync_preview_waypoints(self, robot_id, points):
        payload = json.dumps(
            {
                'command': 'set_waypoints',
                'robot_id': int(robot_id),
                'waypoints': [list(point) for point in points],
            }
        )
        self.rviz_command_pub.publish(String(data=payload))

    def _refresh_rviz_listbox(self, robot_id):
        widgets = self.robot_status_widgets.get(robot_id)
        if widgets is None:
            return
        listbox = widgets['free_roam_listbox']
        listbox.delete(0, tk.END)
        for index, point in enumerate(self.rviz_waypoint_drafts.get(robot_id, []), start=1):
            listbox.insert(tk.END, f'{index}. x={point[0]:.2f}  y={point[1]:.2f}  z={point[2]:.2f}')

    def move_rviz_waypoint(self, robot_id, delta):
        widgets = self.robot_status_widgets.get(robot_id)
        if widgets is None:
            return
        listbox = widgets['free_roam_listbox']
        selection = listbox.curselection()
        if not selection:
            return
        current_index = selection[0]
        new_index = current_index + delta
        points = self.rviz_waypoint_drafts.get(robot_id, [])
        if new_index < 0 or new_index >= len(points):
            return
        points[current_index], points[new_index] = points[new_index], points[current_index]
        self._refresh_rviz_listbox(robot_id)
        listbox.selection_set(new_index)
        self._sync_preview_waypoints(robot_id, points)

    def delete_rviz_waypoint(self, robot_id):
        widgets = self.robot_status_widgets.get(robot_id)
        if widgets is None:
            return
        listbox = widgets['free_roam_listbox']
        selection = listbox.curselection()
        if not selection:
            return
        points = self.rviz_waypoint_drafts.get(robot_id, [])
        points.pop(selection[0])
        self._refresh_rviz_listbox(robot_id)
        self._sync_preview_waypoints(robot_id, points)
        self._update_program_preview(robot_id)

    def clear_robot_draft(self, robot_id):
        self.rviz_waypoint_drafts.pop(int(robot_id), None)
        self._refresh_rviz_listbox(int(robot_id))
        self._sync_preview_waypoints(int(robot_id), [])
        if int(robot_id) in self.robot_status_widgets:
            self._update_program_preview(int(robot_id))

    def create_obstacle_frame(self, parent):
        frame = ttk.LabelFrame(parent, text='Spawn Obstacles', padding=10)
        frame.pack(fill='x', padx=10, pady=5)

        help_label = tk.Label(
            frame,
            text='Open this tab, choose a preset, then click in RViz to place obstacles. Robot click modes are forced idle while this tab is active.',
            justify='left',
            anchor='w',
            font=('Arial', 10),
            bg='#eef3f7',
        )
        help_label.pack(fill='x', pady=(0, 8))

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
        if not hasattr(self, 'rviz_status_label'):
            return
        robot_id = int(self.selected_robot_var.get())
        draft_count = len(self.rviz_waypoint_drafts.get(robot_id, []))
        self.rviz_status_label.configure(
            text=(
                f'RViz capture: {self.rviz_mode_var.get()} | '
                f'target: robot_{robot_id} | '
                f'points: {draft_count}'
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

    def _robot_enable_pub(self, robot_id):
        if robot_id not in self.robot_enable_pubs:
            qos = QoSProfile(depth=1)
            qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            qos.reliability = ReliabilityPolicy.RELIABLE
            self.robot_enable_pubs[robot_id] = self.create_publisher(
                Bool,
                f'/robot_{robot_id}/robot_enable',
                qos,
            )
        return self.robot_enable_pubs[robot_id]

    def _set_robot_enabled(self, robot_id, enabled):
        self._robot_enable_pub(robot_id).publish(Bool(data=bool(enabled)))

    def _handle_spawn_response(self, future, robot_id):
        try:
            result = future.result()
        except Exception as exc:
            messagebox.showerror('Spawn Robot', f'robot_{robot_id}: {exc}')
            return
        if result and not result.success:
            messagebox.showwarning('Spawn Robot', result.message)
            return
        self.selected_robot_var.set(int(robot_id))
        self.single_robot_id_var.set(self._next_available_robot_id())
        self._update_rviz_status_label()

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
