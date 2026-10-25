#!/usr/bin/env python3

import math
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class GazeboQuadIK(Node):

    def __init__(self):
        super().__init__('gazebo_quad_ik')

        # -------- Publisher (IMPORTANT CHANGE) --------
        self.pub = self.create_publisher(
            Float64MultiArray,
            '/joint_group_controller/commands',
            10
        )

        # -------- Joint Angles --------
        self.q = [0.0]*12

        # -------- Link Lengths --------
        self.L1 = 0.05
        self.L2 = 0.135
        self.L3 = 0.1575

        # -------- Targets --------
        self.targets = [
            [0.05, 0.0, 0.15],
            [-0.05, 0.0, 0.15],
            [-0.05, 0.0, 0.15],
            [0.05, 0.0, 0.15],
        ]

        # -------- Timer --------
        self.timer = self.create_timer(0.05, self.update)

        # -------- Input Thread --------
        threading.Thread(target=self.input_loop, daemon=True).start()

        self.get_logger().info(
            "\nEnter 4 leg coordinates:\n"
            "x1 y1 z1, x2 y2 z2, x3 y3 z3, x4 y4 z4\n"
        )

    # -------- INPUT --------
    def input_loop(self):
        while True:
            try:
                data = input("Enter 4 legs: ")
                sets = data.split(',')

                if len(sets) != 4:
                    print("❌ Need 4 sets")
                    continue

                new_targets = []

                for s in sets:
                    vals = list(map(float, s.strip().split()))
                    if len(vals) != 3:
                        print("❌ Each set needs 3 values")
                        break
                    new_targets.append(vals)

                if len(new_targets) == 4:
                    self.targets = new_targets
                    print("✅ Targets updated")

            except Exception as e:
                print("Error:", e)

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

        for i in range(4):

            x, y, z = self.targets[i]

            if i in [0, 3]:  # RIGHT
                J1, J2, J3 = self.IK_right(x, y, z)
            else:            # LEFT
                J1, J2, J3 = self.IK_left(x, y, z)

            base = i * 3
            self.q[base]   = J1
            self.q[base+1] = J2
            self.q[base+2] = J3

        # -------- Publish to Gazebo --------
        msg = Float64MultiArray()
        msg.data = self.q

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboQuadIK()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()