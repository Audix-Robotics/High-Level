#!/usr/bin/env python3

import threading
import tkinter as tk

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class ScissorSliderGui(Node):
    def __init__(self):
        super().__init__('scissor_slider_gui')
        self.pub = self.create_publisher(Float64, '/scissor_lift/slider', 10)
        self.current_value = 0.0

        self.get_logger().info('Scissor slider GUI ready on /scissor_lift/slider.')

    def publish_value(self, value: float) -> None:
        msg = Float64()
        msg.data = float(value)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScissorSliderGui()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    root = tk.Tk()
    root.title('Scissor Lift')
    root.geometry('380x170')

    label = tk.Label(root, text='Scissor Lift (0–100%)')
    label.pack(pady=8)

    value_label = tk.Label(root, text='0.00')
    value_label.pack(pady=2)

    def on_change(v: str) -> None:
        percent = float(v)
        node.current_value = max(0.0, min(1.0, percent / 100.0))
        value_label.config(text=f'{percent:.0f}%')
        node.publish_value(node.current_value)

    slider = tk.Scale(
        root,
        from_=0.0,
        to=100.0,
        resolution=1.0,
        orient='horizontal',
        length=300,
        command=on_change,
    )
    slider.set(0.0)
    slider.pack(pady=4)

    def on_center() -> None:
        slider.set(0.0)

    center_btn = tk.Button(root, text='Center', command=on_center)
    center_btn.pack(pady=6)

    def on_close() -> None:
        node.publish_value(0.0)
        root.destroy()

    root.protocol('WM_DELETE_WINDOW', on_close)
    root.mainloop()

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
