#!/usr/bin/env python3

import tkinter as tk
import math

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

        # Direction state: angle in degrees (0=East, 90=North, 180=West, 270=South)
        self.direction_angle = 0.0

        self.root = tk.Tk()
        self.root.title('Audix Arena Spawn Panel')
        self.root.geometry('420x540')
        self.root.resizable(False, False)

        header = tk.Label(self.root,
            text='Click in RViz with Publish Point to spawn obstacles',
            wraplength=380, justify='center')
        header.pack(pady=(10, 4))

        self.status_var = tk.StringVar(value='Preset: %s' % self.default_preset)
        status = tk.Label(self.root, textvariable=self.status_var, fg='#114b5f')
        status.pack(pady=(0, 8))

        # Preset buttons
        preset_frame = tk.LabelFrame(self.root, text='Spawn Presets', padx=8, pady=8)
        preset_frame.pack(fill='x', padx=12)
        for presets in [
            ['static_small', 'static_medium', 'static_large'],
            ['dynamic_small', 'dynamic_medium', 'dynamic_large'],
            ['random_static', 'random_dynamic'],
        ]:
            line = tk.Frame(preset_frame)
            line.pack(fill='x', pady=3)
            for preset in presets:
                btn = tk.Button(line, text=preset.replace('_', ' '),
                    command=lambda p=preset: self._send_preset(p))
                btn.pack(side='left', expand=True, fill='x', padx=2)

        # Direction keypad
        dir_frame = tk.LabelFrame(self.root, text='Dynamic Obstacle Direction', padx=8, pady=8)
        dir_frame.pack(fill='x', padx=12, pady=(10, 0))

        self.dir_label = tk.Label(dir_frame, text='Direction: East (0°)', fg='#005f8a')
        self.dir_label.pack(pady=(0, 6))

        keypad = tk.Frame(dir_frame)
        keypad.pack()

        btn_nw = tk.Button(keypad, text='↖', width=3, command=lambda: self._set_direction(135))
        btn_n  = tk.Button(keypad, text='↑', width=3, command=lambda: self._set_direction(90))
        btn_ne = tk.Button(keypad, text='↗', width=3, command=lambda: self._set_direction(45))
        btn_w  = tk.Button(keypad, text='←', width=3, command=lambda: self._set_direction(180))
        btn_c  = tk.Button(keypad, text='○', width=3, command=lambda: self._set_direction(None))
        btn_e  = tk.Button(keypad, text='→', width=3, command=lambda: self._set_direction(0))
        btn_sw = tk.Button(keypad, text='↙', width=3, command=lambda: self._set_direction(225))
        btn_s  = tk.Button(keypad, text='↓', width=3, command=lambda: self._set_direction(270))
        btn_se = tk.Button(keypad, text='↘', width=3, command=lambda: self._set_direction(315))

        btn_nw.grid(row=0, column=0, padx=2, pady=2)
        btn_n .grid(row=0, column=1, padx=2, pady=2)
        btn_ne.grid(row=0, column=2, padx=2, pady=2)
        btn_w .grid(row=1, column=0, padx=2, pady=2)
        btn_c .grid(row=1, column=1, padx=2, pady=2)
        btn_e .grid(row=1, column=2, padx=2, pady=2)
        btn_sw.grid(row=2, column=0, padx=2, pady=2)
        btn_s .grid(row=2, column=1, padx=2, pady=2)
        btn_se.grid(row=2, column=2, padx=2, pady=2)

        # Command buttons
        cmd_frame = tk.LabelFrame(self.root, text='Commands', padx=8, pady=8)
        cmd_frame.pack(fill='x', padx=12, pady=(10, 0))
        for commands in [
            [('Remove Last', 'remove_last'), ('Clear All', 'clear_all')],
            [('Pause Dynamic', 'pause_dynamic'), ('Resume Dynamic', 'resume_dynamic')],
            [('Randomize Motion', 'randomize_dynamic')],
        ]:
            line = tk.Frame(cmd_frame)
            line.pack(fill='x', pady=3)
            for label, command in commands:
                btn = tk.Button(line, text=label,
                    command=lambda c=command: self._send_command(c))
                btn.pack(side='left', expand=True, fill='x', padx=2)

        self.root.protocol('WM_DELETE_WINDOW', self._close)
        self._send_preset(self.default_preset)
        self.create_timer(0.05, self._pump_tk)

    def _set_direction(self, angle_deg):
        if angle_deg is None:
            # Random — send no direction override
            self.direction_angle = None
            self.dir_label.config(text='Direction: Random')
            self._send_command('direction_random')
        else:
            self.direction_angle = float(angle_deg)
            names = {0:'East',45:'NE',90:'North',135:'NW',
                     180:'West',225:'SW',270:'South',315:'SE'}
            label = names.get(angle_deg, '%d°' % angle_deg)
            self.dir_label.config(text='Direction: %s (%d°)' % (label, angle_deg))
            self._send_command('direction_%d' % angle_deg)

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