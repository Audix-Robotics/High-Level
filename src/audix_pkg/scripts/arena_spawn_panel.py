#!/usr/bin/env python3

import tkinter as tk

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ArenaSpawnPanel(Node):
    def __init__(self):
        super().__init__('arena_spawn_panel')

        self.declare_parameter('default_preset', 'dynamic_medium')
        self.default_preset = str(self.get_parameter('default_preset').value)

        self.preset_pub = self.create_publisher(String, '/arena_spawn_preset', 10)
        self.command_pub = self.create_publisher(String, '/arena_spawn_command', 10)

        self.root = tk.Tk()
        self.root.title('Audix Arena Spawn Panel')
        self.root.geometry('360x360')
        self.root.resizable(False, False)

        self.status_var = tk.StringVar(value='Preset: %s' % self.default_preset)
        header = tk.Label(self.root, text='Click in RViz with Publish Point to spawn obstacles', wraplength=320, justify='center')
        header.pack(pady=(12, 8))
        status = tk.Label(self.root, textvariable=self.status_var, fg='#114b5f')
        status.pack(pady=(0, 12))

        preset_frame = tk.LabelFrame(self.root, text='Spawn Presets', padx=10, pady=10)
        preset_frame.pack(fill='x', padx=12)
        for row, presets in enumerate([
            ['static_small', 'static_medium', 'static_large'],
            ['dynamic_small', 'dynamic_medium', 'dynamic_large'],
            ['random_static', 'random_dynamic'],
        ]):
            line = tk.Frame(preset_frame)
            line.pack(fill='x', pady=4)
            for preset in presets:
                button = tk.Button(line, text=preset.replace('_', ' '), command=lambda p=preset: self._send_preset(p))
                button.pack(side='left', expand=True, fill='x', padx=3)

        command_frame = tk.LabelFrame(self.root, text='Commands', padx=10, pady=10)
        command_frame.pack(fill='x', padx=12, pady=(12, 0))
        for row, commands in enumerate([
            [('Remove Last', 'remove_last'), ('Clear All', 'clear_all')],
            [('Pause Dynamic', 'pause_dynamic'), ('Resume Dynamic', 'resume_dynamic')],
            [('Randomize Motion', 'randomize_dynamic')],
        ]):
            line = tk.Frame(command_frame)
            line.pack(fill='x', pady=4)
            for label, command in commands:
                button = tk.Button(line, text=label, command=lambda c=command: self._send_command(c))
                button.pack(side='left', expand=True, fill='x', padx=3)

        self.root.protocol('WM_DELETE_WINDOW', self._close)
        self._send_preset(self.default_preset)
        self.create_timer(0.05, self._pump_tk)

    def _send_preset(self, preset):
        msg = String()
        msg.data = preset
        self.preset_pub.publish(msg)
        self.status_var.set('Preset: %s' % preset)

    def _send_command(self, command):
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)

    def _pump_tk(self):
        try:
            self.root.update_idletasks()
            self.root.update()
        except tk.TclError:
            pass

    def _close(self):
        self.root.destroy()
        self.destroy_node()


def main():
    rclpy.init()
    node = ArenaSpawnPanel()
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