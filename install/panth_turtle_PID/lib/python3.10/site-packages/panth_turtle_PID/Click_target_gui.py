#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

# Uses matplotlib for a lightweight click UI
import matplotlib.pyplot as plt

class ClickTargetGUI(Node):
    def __init__(self):
        super().__init__('click_target_gui')
        self.pub = self.create_publisher(Point, '/target_point', 10)

        # Build a simple 11x11 board (turtlesim coords approx 0..11)
        self.fig, self.ax = plt.subplots()
        self.ax.set_title('Click to set target (turtlesim ~ 0..11)')
        self.ax.set_xlim(0, 11)
        self.ax.set_ylim(0, 11)
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.grid(True, linestyle='--', alpha=0.3)

        self.cid = self.fig.canvas.mpl_connect('button_press_event', self.onclick)
        self.get_logger().info("Click anywhere in the square to send a target point.")

    def onclick(self, event):
        if event.inaxes != self.ax:
            return
        p = Point()
        p.x = float(event.xdata)
        p.y = float(event.ydata)
        p.z = 0.0
        self.pub.publish(p)
        self.get_logger().info(f"Published target: ({p.x:.2f}, {p.y:.2f})")

def main():
    rclpy.init()
    node = ClickTargetGUI()
    try:
        # Spin ROS in the background while matplotlib mainloop runs
        import threading
        t = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        t.start()
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
