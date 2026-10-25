
#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import threading
from std_msgs.msg import Float64MultiArray


class SemicircleGaitNode(Node):

    def __init__(self):
        super().__init__('semicircle_gait_node')

        # Gait parameters (metres)
        self.step_length = 0.08
        self.front_step_height = 0.04  # 4 cm
        self.rear_step_height = 0.06   # 6 cm
        self.gait_period = 1.0

        # Diagonal trot: front-left/rear-right,
        # then front-right/rear-left.
        self.legs = {
            'front_left': 0.0,
            'rear_right': 0.0,
            'front_right': 0.5,
            'rear_left': 0.5,
        }
        self.foot_publishers = {}
        for leg in self.legs:
            topic = f'/{leg}/foot_target'
            self.foot_publishers[leg] = self.create_publisher(
                Point, topic, 10
            )

        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.02, self.update_gait)

        self.get_logger().info(
            'Semicircle gait started: publishing at 50 Hz'
        )


    def calculate_trajectory(self, phase, leg):
        L = self.step_length

        is_front = leg.startswith('front')
        is_right = leg.endswith('right')

        H = (
            self.front_step_height
            if is_front
            else self.rear_step_height
        )

        # Nominal foot position from the crawl node
        x = 0.05 if is_right else -0.05
        y0 = 0.010 if is_front else -0.010
        z0 = 0.15

        phase %= 1.0

        if phase < 0.5:
            # Stance: move backward along the ground
            s = phase / 0.5
            dy = L / 2.0 - L * s
            lift = 0.0
        else:
            # Swing: semicircular arc
            s = (phase - 0.5) / 0.5
            theta = math.pi * s

            dy = -L / 2.0 + (
                L / 2.0
            ) * (1.0 - math.cos(theta))

            lift = H * math.sin(theta)

        y = y0 + dy
        z = z0 - lift

        return x, y, z


    def update_gait(self):
        elapsed = (
            self.get_clock().now() - self.start_time
        ).nanoseconds / 1e9

        for leg, offset in self.legs.items():
            phase = (
                elapsed / self.gait_period + offset
            ) % 1.0

            if leg.startswith('front'):
                height = self.front_step_height
            else:
                height = self.rear_step_height

            x, z = self.calculate_trajectory(phase, height)

            msg = Point()
            msg.x = float(x)
            msg.y = 0.0
            msg.z = float(z)

        self.foot_publishers[leg].publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SemicircleGaitNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
