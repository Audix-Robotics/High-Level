#!/usr/bin/env python3
"""
Simple Start/Stop GUI for /robot_enable

Creates a small Tkinter window with Start and Stop buttons. Publishes
std_msgs/Bool on `/robot_enable` with TRANSIENT_LOCAL QoS so late
subscribers (controllers) see the current state. The node starts in
the STOP state and repeatedly republishes the state at 2 Hz.
"""

import threading
import sys
try:
    import tkinter as tk
except Exception:
    tk = None

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool


class StartStopGUI(Node):
    def __init__(self):
        super().__init__('start_stop_gui')

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE

        self.pub = self.create_publisher(Bool, '/robot_enable', qos)
        self.state = False  # start in STOP state

        # Periodic heartbeat to ensure state is known (2 Hz)
        self.create_timer(0.5, self._publish_state)

    def _publish_state(self):
        msg = Bool()
        msg.data = bool(self.state)
        try:
            self.pub.publish(msg)
        except Exception:
            pass

    def set_state(self, on: bool):
        self.state = bool(on)
        self.get_logger().info('START' if self.state else 'STOP')
        self._publish_state()


def _run_gui(node: StartStopGUI):
    if tk is None:
        node.get_logger().error('Tkinter not available; GUI cannot run.')
        return

    root = tk.Tk()
    root.title('Robot Start/Stop')
    root.geometry('220x110')

    state_var = tk.StringVar()
    state_var.set('STOPPED')

    def on_set(on: bool):
        node.set_state(on)
        state_var.set('RUNNING' if on else 'STOPPED')

    lbl = tk.Label(root, textvariable=state_var, font=('Helvetica', 14), fg='red')
    lbl.pack(pady=(10, 6))

    btn_frame = tk.Frame(root)
    btn_frame.pack(pady=(0, 10))

    start_btn = tk.Button(btn_frame, text='START', width=10, bg='#4CAF50', fg='white', command=lambda: on_set(True))
    start_btn.grid(row=0, column=0, padx=6)

    stop_btn = tk.Button(btn_frame, text='STOP', width=10, bg='#F44336', fg='white', command=lambda: on_set(False))
    stop_btn.grid(row=0, column=1, padx=6)

    def on_close():
        try:
            node.get_logger().info('Shutting down start_stop_gui.')
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        root.destroy()

    root.protocol('WM_DELETE_WINDOW', on_close)
    root.mainloop()


def main(argv=None):
    rclpy.init(args=argv)
    node = StartStopGUI()

    # Start ROS spinning in a background thread so Tk mainloop stays responsive
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Ensure initial STOP message is published immediately
    node.set_state(False)

    try:
        _run_gui(node)
    finally:
        # Ensure shutdown if GUI loop exits
        try:
            if node is not None:
                node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main(sys.argv)
