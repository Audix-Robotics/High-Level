#!/usr/bin/env python3

import tkinter as tk
from tkinter import messagebox, simpledialog, ttk

from geometry_msgs.msg import Point, Pose
from visualization_msgs.msg import Marker, MarkerArray


class WarehouseWaypointEditor(tk.Toplevel):
    def __init__(
        self,
        parent,
        robot_id,
        current_mission=None,
        validate_callback=None,
        send_callback=None,
        marker_pub=None,
    ):
        super().__init__(parent)
        self.title(f'Custom Waypoints for Robot_{robot_id}')
        self.geometry('540x470')

        self.robot_id = robot_id
        self.validate_callback = validate_callback
        self.send_callback = send_callback
        self.marker_pub = marker_pub
        self.waypoints = []

        if current_mission:
            self.load_mission(current_mission)

        self.create_waypoint_list_frame()
        self.create_add_waypoint_frame()
        self.create_mission_name_frame()
        self.create_button_frame()
        self.visualize_path()

    def load_mission(self, current_mission):
        for waypoint in current_mission:
            position = waypoint.get('position', [0.0, 0.0])
            self.waypoints.append(
                {
                    'x': float(position[0]),
                    'y': float(position[1]),
                    'dwell_time': float(waypoint.get('dwell_time', 1.0)),
                }
            )

    def create_waypoint_list_frame(self):
        frame = ttk.LabelFrame(self, text='Waypoint List', padding=10)
        frame.pack(fill='both', expand=True, padx=10, pady=5)

        self.waypoint_listbox = tk.Listbox(frame, height=10)
        self.waypoint_listbox.pack(fill='both', expand=True, side='left')

        scrollbar = ttk.Scrollbar(frame, orient='vertical', command=self.waypoint_listbox.yview)
        scrollbar.pack(side='right', fill='y')
        self.waypoint_listbox.config(yscrollcommand=scrollbar.set)

        button_frame = ttk.Frame(self)
        button_frame.pack(fill='x', padx=10, pady=5)
        ttk.Button(button_frame, text='Edit Selected', command=self.edit_waypoint).pack(side='left', padx=2)
        ttk.Button(button_frame, text='Delete Selected', command=self.delete_waypoint).pack(side='left', padx=2)
        ttk.Button(button_frame, text='Move Up', command=self.move_waypoint_up).pack(side='left', padx=2)
        ttk.Button(button_frame, text='Move Down', command=self.move_waypoint_down).pack(side='left', padx=2)

        self.refresh_waypoint_list()

    def create_add_waypoint_frame(self):
        frame = ttk.LabelFrame(self, text='Add New Waypoint', padding=10)
        frame.pack(fill='x', padx=10, pady=5)

        ttk.Label(frame, text='X:').grid(row=0, column=0)
        self.x_entry = ttk.Entry(frame, width=10)
        self.x_entry.grid(row=0, column=1, padx=5)

        ttk.Label(frame, text='Y:').grid(row=0, column=2)
        self.y_entry = ttk.Entry(frame, width=10)
        self.y_entry.grid(row=0, column=3, padx=5)

        ttk.Label(frame, text='Dwell (s):').grid(row=0, column=4)
        self.dwell_entry = ttk.Entry(frame, width=8)
        self.dwell_entry.insert(0, '1.0')
        self.dwell_entry.grid(row=0, column=5, padx=5)

        ttk.Button(frame, text='Add Waypoint', command=self.add_waypoint).grid(row=0, column=6, padx=10)

    def create_mission_name_frame(self):
        frame = ttk.LabelFrame(self, text='Mission Details', padding=10)
        frame.pack(fill='x', padx=10, pady=5)

        ttk.Label(frame, text='Mission Name:').pack(side='left')
        self.mission_name_entry = ttk.Entry(frame, width=30)
        self.mission_name_entry.pack(side='left', padx=5)
        self.mission_name_entry.insert(0, f'custom_robot_{self.robot_id}')

    def create_button_frame(self):
        frame = ttk.Frame(self)
        frame.pack(fill='x', padx=10, pady=10)

        ttk.Button(frame, text='Validate Path', command=self.validate_path).pack(side='left', padx=5)
        ttk.Button(frame, text='Cancel', command=self.cancel).pack(side='right', padx=5)
        ttk.Button(frame, text='OK (Send to Robot)', command=self.ok).pack(side='right', padx=5)

    def refresh_waypoint_list(self):
        self.waypoint_listbox.delete(0, 'end')
        for index, waypoint in enumerate(self.waypoints):
            self.waypoint_listbox.insert(
                'end',
                f'[{index}] ({waypoint["x"]:.2f}, {waypoint["y"]:.2f}) dwell={waypoint["dwell_time"]:.1f}s',
            )

    def add_waypoint(self):
        try:
            x_pos = float(self.x_entry.get())
            y_pos = float(self.y_entry.get())
            dwell = float(self.dwell_entry.get())
        except ValueError:
            messagebox.showerror('Invalid Input', 'Please enter valid numeric waypoint values')
            return

        self.waypoints.append({'x': x_pos, 'y': y_pos, 'dwell_time': dwell})
        self.refresh_waypoint_list()
        self.visualize_path()
        self.x_entry.delete(0, 'end')
        self.y_entry.delete(0, 'end')
        self.dwell_entry.delete(0, 'end')
        self.dwell_entry.insert(0, '1.0')

    def edit_waypoint(self):
        selection = self.waypoint_listbox.curselection()
        if not selection:
            return
        index = selection[0]
        waypoint = self.waypoints[index]
        value = simpledialog.askstring(
            'Edit Waypoint',
            'Enter x,y,dwell',
            initialvalue=f'{waypoint["x"]},{waypoint["y"]},{waypoint["dwell_time"]}',
            parent=self,
        )
        if not value:
            return
        try:
            x_pos, y_pos, dwell = [float(item.strip()) for item in value.split(',')]
        except ValueError:
            messagebox.showerror('Invalid Input', 'Format must be x,y,dwell')
            return
        self.waypoints[index] = {'x': x_pos, 'y': y_pos, 'dwell_time': dwell}
        self.refresh_waypoint_list()
        self.visualize_path()

    def delete_waypoint(self):
        selection = self.waypoint_listbox.curselection()
        if not selection:
            return
        self.waypoints.pop(selection[0])
        self.refresh_waypoint_list()
        self.visualize_path()

    def move_waypoint_up(self):
        selection = self.waypoint_listbox.curselection()
        if not selection or selection[0] == 0:
            return
        index = selection[0]
        self.waypoints[index - 1], self.waypoints[index] = self.waypoints[index], self.waypoints[index - 1]
        self.refresh_waypoint_list()
        self.waypoint_listbox.selection_set(index - 1)
        self.visualize_path()

    def move_waypoint_down(self):
        selection = self.waypoint_listbox.curselection()
        if not selection or selection[0] >= len(self.waypoints) - 1:
            return
        index = selection[0]
        self.waypoints[index + 1], self.waypoints[index] = self.waypoints[index], self.waypoints[index + 1]
        self.refresh_waypoint_list()
        self.waypoint_listbox.selection_set(index + 1)
        self.visualize_path()

    def validate_path(self):
        if self.validate_callback is None:
            messagebox.showinfo('Validate Path', 'No validator is connected')
            return True
        is_valid, message = self.validate_callback(self.robot_id, self.waypoints)
        if is_valid:
            messagebox.showinfo('Validate Path', message or 'Mission path is valid')
        else:
            messagebox.showerror('Validate Path', message or 'Mission path is invalid')
        return is_valid

    def ok(self):
        if not self.waypoints:
            messagebox.showwarning('No Waypoints', 'Add at least one waypoint before sending')
            return
        if self.validate_callback is not None:
            is_valid, message = self.validate_callback(self.robot_id, self.waypoints)
            if not is_valid:
                messagebox.showerror('Invalid Mission', message or 'Mission path failed validation')
                return
        if self.send_callback is not None:
            poses = []
            for waypoint in self.waypoints:
                pose = Pose()
                pose.position.x = waypoint['x']
                pose.position.y = waypoint['y']
                pose.position.z = 0.0
                pose.orientation.w = 1.0
                poses.append(pose)
            self.send_callback(self.robot_id, self.mission_name_entry.get().strip(), poses)
        self.clear_markers()
        self.destroy()

    def visualize_path(self):
        if self.marker_pub is None:
            return

        markers = MarkerArray()
        clear = Marker()
        clear.header.frame_id = 'odom'
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        line = Marker()
        line.header.frame_id = 'odom'
        line.ns = f'custom_path_{self.robot_id}'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.04
        line.color.r = 1.0
        line.color.g = 0.6
        line.color.b = 0.0
        line.color.a = 0.95
        for waypoint in self.waypoints:
            line.points.append(Point(x=waypoint['x'], y=waypoint['y'], z=0.08))
        markers.markers.append(line)

        for index, waypoint in enumerate(self.waypoints, start=1):
            marker = Marker()
            marker.header.frame_id = 'odom'
            marker.ns = f'custom_path_points_{self.robot_id}'
            marker.id = index
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = waypoint['x']
            marker.pose.position.y = waypoint['y']
            marker.pose.position.z = 0.1
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.12
            marker.scale.y = 0.12
            marker.scale.z = 0.12
            marker.color.r = 1.0
            marker.color.g = 0.85
            marker.color.b = 0.1
            marker.color.a = 0.95
            markers.markers.append(marker)

        self.marker_pub.publish(markers)

    def clear_markers(self):
        if self.marker_pub is None:
            return
        clear = Marker()
        clear.header.frame_id = 'odom'
        clear.action = Marker.DELETEALL
        self.marker_pub.publish(MarkerArray(markers=[clear]))

    def cancel(self):
        self.clear_markers()
        self.destroy()


def main():
    root = tk.Tk()
    root.withdraw()
    WarehouseWaypointEditor(root, 0)
    root.mainloop()


if __name__ == '__main__':
    main()