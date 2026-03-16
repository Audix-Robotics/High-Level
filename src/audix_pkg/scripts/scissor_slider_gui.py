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
    root.title('Scissor Lift Slider')
    root.geometry('360x140')

    label = tk.Label(root, text='scissor_lift/slider (-1.0 to 1.0)')
    label.pack(pady=8)

    value_label = tk.Label(root, text='0.00')
    value_label.pack(pady=2)

    def on_change(v: str) -> None:
        node.current_value = float(v)
        value_label.config(text=f'{node.current_value:.2f}')
        node.publish_value(node.current_value)

    slider = tk.Scale(
        root,
        from_=-1.0,
        to=1.0,
        resolution=0.01,
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
