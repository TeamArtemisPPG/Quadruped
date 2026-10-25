#!/usr/bin/env python3

import math
import threading

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState


class QuadraController(Node):

    def __init__(self):

        super().__init__('quadra_controller')

        # --------------------------------
        # Publisher
        # --------------------------------
        self.pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        # --------------------------------
        # Joint Names
        # --------------------------------
        self.joint_names = [
            'CCUP1', 'THIGH1', 'FOOT1',
            'CCUP2', 'THIGH2', 'FOOT2',
            'CCUP3', 'THIGH3', 'FOOT3',
            'CCUP4', 'THIGH4', 'FOOT4'
        ]

        self.positions = [0.0] * 12

        # --------------------------------
        # Link Lengths
        # --------------------------------
        self.L1 = 0.05
        self.L2 = 0.135
        self.L3 = 0.1575

        # --------------------------------
        # Base Height
        # --------------------------------
        self.base_height = 0.015

        # --------------------------------
        # Timing
        # --------------------------------
        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.update)

        self.step_time = 0.2
        self.step_timer = 0.0

        self.step_index = 0
        self.num_steps = 4

        # --------------------------------
        # Current Command
        # --------------------------------
        self.command = 'f'

        # --------------------------------
        # WALK GAIT
        # (x, y, z)
        # --------------------------------

        self.forward_gait = {

            # FRONT RIGHT
            4: [
                (0.065, 0.00, 0.15),
                (0.065, -0.025, 0.15),
                (0.065, -0.05, 0.15),
                (0.065, 0.00, 0.08)
            ],

            # FRONT LEFT
            3: [
                (-0.065, 0.00, 0.15),
                (-0.065, -0.025, 0.15),
                (-0.065, -0.05, 0.15),
                (-0.065, 0.00, 0.08)
            ],

            # BACK LEFT
            2: [
                (-0.065, 0.00, 0.15),
                (-0.065, -0.025, 0.15),
                (-0.065, -0.05, 0.15),
                (-0.065, 0.00, 0.08)
            ],

            # BACK RIGHT
            1: [
                (0.065, 0.00, 0.15),
                (0.065, -0.025, 0.15),
                (0.065, -0.05, 0.15),
                (0.065, 0.00, 0.08)
            ]
        }

        # --------------------------------
        # BACKWALK GAIT
        # --------------------------------

        self.backward_gait = {

            # FRONT RIGHT
            4: [
                (0.065, -0.05, 0.15),
                (0.065, -0.025, 0.15),
                (0.065, 0.00, 0.15),
                (0.065, -0.05, 0.08)
            ],

            # FRONT LEFT
            3: [
                (-0.065, -0.05, 0.15),
                (-0.065, -0.025, 0.15),
                (-0.065, 0.00, 0.15),
                (-0.065, -0.05, 0.08)
            ],

            # BACK LEFT
            2: [
                (-0.065, -0.05, 0.15),
                (-0.065, -0.025, 0.15),
                (-0.065, 0.00, 0.15),
                (-0.065, -0.05, 0.08)
            ],

            # BACK RIGHT
            1: [
                (0.065, -0.05, 0.15),
                (0.065, -0.025, 0.15),
                (0.065, 0.00, 0.15),
                (0.065, -0.05, 0.08)
            ]
        }

        # --------------------------------
        # RIGHT TURN GAIT
        # --------------------------------
        self.right_gait = {

            4: [
                (0.05, 0.0, 0.15),# (0.05, -0.03, 0.15),
                (0.075, -0.025, 0.15),# (0.075, -0.03, 0.10),
                (0.1, -0.05, 0.15),# (0.1, -0.03, 0.15),
                (0.075, -0.025, 0.08)# (0.075, -0.03, 0.15)
            ],

            3: [
                (-0.05, 0.0, 0.15),# (-0.1, -0.03, 0.15),
                (-0.075, -0.025, 0.15),# (-0.075, -0.03, 0.10),
                (-0.1, -0.05, 0.15),# (-0.05, -0.03, 0.15),
                (-0.075, -0.025, 0.08)# (-0.075, -0.03, 0.15)
            ],

            2: [
                (-0.05, 0.0, 0.15),# (-0.05, -0.03, 0.15),
                (-0.075, 0.025, 0.08),# (-0.075, -0.03, 0.10),
                (-0.1, 0.05, 0.15),# (-0.1, -0.03, 0.15),
                (-0.075, 0.025, 0.15)# (-0.075, -0.03, 0.15)
            ],

            1: [
                (0.05, 0.0, 0.15),# (0.1, -0.03, 0.15),
                (0.075, 0.025, 0.15),# (0.075, -0.03, 0.10),
                (0.1, 0.05, 0.15),# (0.05, -0.03, 0.15),
                (0.075, 0.025, 0.08)# (0.075, -0.03, 0.15)
            ]
        }

        # --------------------------------
        # Diagonal Pairs
        # --------------------------------
        self.group_A = [1, 3]
        self.group_B = [2, 4]

        # --------------------------------
        # Keyboard Thread
        # --------------------------------
        threading.Thread(
            target=self.keyboard_input,
            daemon=True
        ).start()

        self.get_logger().info(
            "Commands: f b l r"
        )

    # ====================================
    # Keyboard Input
    # ====================================
    def keyboard_input(self):

        while True:

            cmd = input("Enter command: ").strip().lower()

            if cmd in ['f', 'b', 'l', 'r']:
                self.command = cmd
                print(f"Switched to: {cmd}")

    # ====================================
    # Gait Selection
    # ====================================
    def get_gait(self):

        if self.command == 'f':
            return self.forward_gait

        elif self.command == 'b':
            return self.backward_gait

        elif self.command == 'r':
            return self.right_gait

        elif self.command == 'l':
            return {
                leg: list(reversed(points))
                for leg, points in self.right_gait.items()
            }

        return self.forward_gait

    # ====================================
    # IK LEFT
    # ====================================
    def IK_left(self, x, y, z):

        L1, L2, L3 = self.L1, self.L2, self.L3

        C = math.sqrt(x**2 + z**2)
        D = math.sqrt(max(C**2 - L1**2, 0.0))
        G = math.sqrt(D**2 + y**2)

        def clamp(v): return max(-1.0, min(1.0, v))

        J3 = math.acos(clamp((L2**2 + L3**2 - G**2) / (2 * L2 * L3))) - math.pi/2

        J2 = math.atan2(-y, D) + math.acos(
            clamp((L2**2 + G**2 - L3**2) / (2 * L2 * G))
        )

        J1 = math.pi/2 - math.atan2(-x, z) - math.atan2(D, L1)

        return J1, J2, J3

    # ====================================
    # IK RIGHT
    # ====================================
    def IK_right(self, x, y, z):

        L1, L2, L3 = self.L1, self.L2, self.L3

        C = math.sqrt(x**2 + z**2)
        D = math.sqrt(max(C**2 - L1**2, 0.0))
        G = math.sqrt(D**2 + y**2)

        def clamp(v): return max(-1.0, min(1.0, v))

        J3 = math.acos(clamp((L2**2 + L3**2 - G**2) / (2 * L2 * L3))) - math.pi/2

        J2 = -(math.atan2(-y, D) + math.acos(
            clamp((L2**2 + G**2 - L3**2) / (2 * L2 * G))
        ))

        J1 = math.atan2(x, z) + math.atan2(D, L1) - math.pi/2

        return J1, J2, J3

    # ====================================
    # Gait Timing
    # ====================================
    def update_gait(self):

        self.step_timer += self.dt

        if self.step_timer >= self.step_time:

            self.step_timer = 0.0

            self.step_index = (
                self.step_index + 1
            ) % self.num_steps

    # ====================================
    # Main Update
    # ====================================
    def update(self):

        self.update_gait()

        gait = self.get_gait()

        for leg_id in range(1, 5):

            # Diagonal phase shift
            if leg_id in self.group_A:
                idx = self.step_index
            else:
                idx = (self.step_index + self.num_steps // 2) % self.num_steps

            x, y, z_offset = gait[leg_id][idx]
            z = self.base_height + z_offset

            # Right side legs
            if leg_id in [1, 4]:
                q1, q2, q3 = self.IK_right(x, y, z)

            # Left side legs
            else:
                q1, q2, q3 = self.IK_left(x, y, z)

            base = (leg_id - 1) * 3

            self.positions[base] = q1
            self.positions[base + 1] = q2
            self.positions[base + 2] = q3

        self.publish_joint_states()

    # ====================================
    # Publish
    # ====================================
    def publish_joint_states(self):

        msg = JointState()

        msg.header.stamp = self.get_clock().now().to_msg()

        msg.name = self.joint_names
        msg.position = self.positions

        self.pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = QuadraController()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()