#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class WalkSequence(Node):

    def __init__(self):
        super().__init__('walk_sequence')

        # -------- Publisher --------
        self.pub = self.create_publisher(
            Float64MultiArray,
            '/joint_group_controller/commands',
            10
        )

        self.q = [0.0]*12

        # -------- Link Lengths --------
        self.L1 = 0.05
        self.L2 = 0.135
        self.L3 = 0.1575

        # -------- USER DEFINED POSITIONS --------
        # Initial pose
        self.initial_targets = [
            [0.05, 0.0, 0.0],
            [-0.05, 0.0, 0.0],
            [-0.05, 0.0, 0.0],
            [0.05, 0.0, 0.0],
        ]

        # Second pose (before walking)
        self.second_targets = [
            [0.05, 0.0, 0.15],
            [-0.05, 0.0, 0.15],
            [-0.05, 0.0, 0.15],
            [0.05, 0.0, 0.15],
        ]

        # -------- State Machine --------
        self.time = 0.0
        self.dt = 0.05
        self.state = "INIT"   # INIT → HOLD → TRANSITION → WALK

        # -------- Gait --------
        self.step = 0
        self.cycle_length = 40

        self.timer = self.create_timer(self.dt, self.update)

    # -------- IK LEFT --------
    def IK_left(self, x, y, z):

        L1, L2, L3 = self.L1, self.L2, self.L3

        C = math.sqrt(x**2 + z**2)
        D = math.sqrt(max(C**2 - L1**2, 0.0))
        G = math.sqrt(D**2 + y**2)

        def clamp(v): return max(-1.0, min(1.0, v))

        J3 = math.acos(clamp((L2**2 + L3**2 - G**2)/(2*L2*L3))

)
        J2 = -math.pi/2 + math.atan2(-y, D) + math.asin(
            clamp((L3 * math.sin(J3)) / G if G != 0 else 0.0)
        )

        J1 = math.pi - math.atan2(-x, z) - math.atan2(D, L1)

        return J1, J2, J3

    # -------- IK RIGHT --------
    def IK_right(self, x, y, z):

        L1, L2, L3 = self.L1, self.L2, self.L3

        C = math.sqrt(x**2 + z**2)
        D = math.sqrt(max(C**2 - L1**2, 0.0))
        G = math.sqrt(D**2 + y**2)

        def clamp(v): return max(-1.0, min(1.0, v))

        J3 = math.acos(clamp((L2**2 + L3**2 - G**2)/(2*L2*L3)))

        J2 = math.pi/2 - math.atan2(-y, D) - math.asin(
            clamp((L3 * math.sin(J3)) / G if G != 0 else 0.0)
        )

        J1 = math.atan2(x, z) + math.atan2(D, L1)

        return J1, J2, J3

    # -------- APPLY TARGETS --------
    def apply_targets(self, targets):

        for i in range(4):

            x, y, z = targets[i]

            if i in [0, 3]:
                J1, J2, J3 = self.IK_right(x, y, z)
            else:
                J1, J2, J3 = self.IK_left(x, y, z)

            base = i * 3
            self.q[base]   = J1
            self.q[base+1] = J2
            self.q[base+2] = J3

    # -------- FOOT TRAJECTORY --------
    def foot_trajectory(self, phase):

        step_height = 0.05
        step_length = 0.04
        base_z = 0.15

        t = 2 * math.pi * phase

        y = step_length * math.cos(t)
        z = base_z + step_height * max(0, math.sin(t))

        return y, z

    # -------- MAIN LOOP --------
    def update(self):

        self.time += self.dt

        # -------- STATE MACHINE --------
        if self.state == "INIT":
            self.apply_targets(self.initial_targets)

            if self.time > 2.0:
                self.state = "HOLD"
                self.time = 0.0

        elif self.state == "HOLD":
            self.apply_targets(self.initial_targets)

            if self.time > 10.0:
                self.state = "TRANSITION"
                self.time = 0.0

        elif self.state == "TRANSITION":
            self.apply_targets(self.second_targets)

            if self.time > 2.0:
                self.state = "WALK"
                self.time = 0.0

        elif self.state == "WALK":

            phase_offsets = [0.0, 0.5, 0.0, 0.5]

            for i in range(4):

                phase = ((self.step / self.cycle_length) + phase_offsets[i]) % 1.0

                x = 0.05 if i in [0, 3] else -0.05
                y, z = self.foot_trajectory(phase)

                if i in [0, 3]:
                    J1, J2, J3 = self.IK_right(x, y, z)
                else:
                    J1, J2, J3 = self.IK_left(x, y, z)

                base = i * 3
                self.q[base]   = J1
                self.q[base+1] = J2
                self.q[base+2] = J3

            self.step += 1

        # -------- Publish --------
        msg = Float64MultiArray()
        msg.data = self.q
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WalkSequence()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()