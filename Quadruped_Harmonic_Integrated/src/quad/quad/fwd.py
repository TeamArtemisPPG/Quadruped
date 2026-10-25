#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class GazeboForwardGait(Node):

    def __init__(self):
        super().__init__('gazebo_forward_gait')

        # -------- Publisher (Gazebo controller) --------
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

        # -------- Gait params --------
        self.step = 0
        self.cycle_length = 40

        self.timer = self.create_timer(0.05, self.update)

    # -------- Foot trajectory --------
    def forward(self, phase):

        step_height = 0.05
        step_length = 0.04
        base_z = 0.15

        t = 2 * math.pi * phase

        # forward motion (Y axis)
        y = step_length * math.cos(t)

        # lift during swing
        z = base_z + step_height * max(0, math.sin(t))

        return y, z

    # -------- IK LEFT --------
    def IK_left(self, x, y, z):

        L1, L2, L3 = self.L1, self.L2, self.L3

        C = math.sqrt(x**2 + z**2)
        D = math.sqrt(max(C**2 - L1**2, 0.0))
        G = math.sqrt(D**2 + y**2)

        def clamp(v): return max(-1.0, min(1.0, v))

        J3 = math.acos(clamp((L2**2 + L3**2 - G**2)/(2*L2*L3)))

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

    # -------- MAIN LOOP --------
    def update(self):

        # ✅ Diagonal trot gait
        phase_offsets = [0.0, 0.5, 0.0, 0.5]

        for i in range(4):

            phase = ((self.step / self.cycle_length) + phase_offsets[i]) % 1.0

            # ✅ FIXED lateral offsets
            x = 0.05 if i in [0, 3] else -0.05

            # ✅ gait motion (Y, Z)
            y, z = self.forward(phase)

            # ✅ correct IK
            if i in [0, 3]:
                J1, J2, J3 = self.IK_right(x, y, z)
            else:
                J1, J2, J3 = self.IK_left(x, y, z)

            base = i * 3
            self.q[base]   = J1
            self.q[base+1] = J2
            self.q[base+2] = J3

        self.step += 1

        # -------- Publish to Gazebo --------
        msg = Float64MultiArray()
        msg.data = self.q

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboForwardGait()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()