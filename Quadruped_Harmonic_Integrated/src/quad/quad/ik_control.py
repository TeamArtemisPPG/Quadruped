# #!/usr/bin/env python3

# import math
# import threading
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import JointState


# class IKController(Node):

#     def __init__(self):
#         super().__init__('ik_controller')

#         # -------- Publisher --------
#         self.joint_pub = self.create_publisher(
#             JointState,
#             'joint_states',
#             10
#         )

#         # -------- ALL joints --------
#         self.joint_names = [
#             "CCUP1","THIGH1","FOOT1",
#             "CCUP2","THIGH2","FOOT2",
#             "CCUP3","THIGH3","FOOT3",
#             "CCUP4","THIGH4","FOOT4"
#         ]

#         self.q = [0.0]*12

#         # -------- Default target --------
#         self.X, self.Y, self.Z = 0.1, 0.0, -0.15

#         # -------- Timer --------
#         self.timer = self.create_timer(0.05, self.publish_joint_states)

#         # -------- Input Thread --------
#         threading.Thread(target=self.input_loop, daemon=True).start()

#         self.get_logger().info("Control ONLY leg 1 → Enter X Y Z")

#     # -------- IK --------
#     def inverse_kinematics(self, X, Y, Z):

#         L1 = 0.05
#         L2 = 0.135
#         L3 = 0.1575

#         C = math.sqrt(X**2 + Z**2)
#         D = math.sqrt(max(C**2 - L1**2, 0.0))
#         G = math.sqrt(D**2 + Y**2)

#         def clamp(v):
#             return max(-1.0, min(1.0, v))

#         # Reachability check
#         if G > (L2 + L3):
#             self.get_logger().warn("⚠️ Target out of reach!")
#             G = L2 + L3 - 1e-6

#         J3 = math.acos(
#             clamp((L2**2 + L3**2 - G**2) / (2 * L2 * L3))
#         )

#         J2 = math.pi/2 - math.atan2(Y, D) - math.asin(
#             clamp((L3 * math.sin(J3)) / G if G != 0 else 0.0)
#         )

#         J1 = math.atan2(X, Z) + math.atan2(D, L1)

#         return J1, J2, J3

#     # -------- INPUT --------
#     def input_loop(self):
#         while True:
#             try:
#                 user_input = input("Enter X Y Z: ")
#                 vals = user_input.split()

#                 if len(vals) != 3:
#                     print("❌ Enter 3 values")
#                     continue

#                 self.X, self.Y, self.Z = map(float, vals)

#                 # 👉 Compute IK immediately for logging
#                 q1, q2, q3 = self.inverse_kinematics(self.X, self.Y, self.Z)

#                 # 👉 Print ONLY ONCE
#                 self.get_logger().info(
#                     f"\nTarget Position:\n"
#                     f"X={self.X:.3f}, Y={self.Y:.3f}, Z={self.Z:.3f}\n\n"
#                     f"Leg 1 Angles:\n"
#                     f"J1 = {math.degrees(q1):.2f}°\n"
#                     f"J2 = {math.degrees(q2):.2f}°\n"
#                     f"J3 = {math.degrees(q3):.2f}°"
#                 )

#             except Exception as e:
#                 print("Error:", e)

#     # -------- MAIN LOOP --------
#     def publish_joint_states(self):

#         # Compute IK continuously for motion
#         q1, q2, q3 = self.inverse_kinematics(self.X, self.Y, self.Z)

#         self.q[0] = q1
#         self.q[1] = q2
#         self.q[2] = q3

#         msg = JointState()
#         msg.header.stamp = self.get_clock().now().to_msg()
#         msg.name = self.joint_names
#         msg.position = self.q

#         self.joint_pub.publish(msg)


# def main(args=None):
#     rclpy.init(args=args)
#     node = IKController()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()
#!/usr/bin/env python3

# import math
# import threading
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import JointState


# class RVizQuadIK(Node):

#     def __init__(self):
#         super().__init__('rviz_quad_ik')

#         # -------- Publisher (RViz uses JointState) --------
#         self.pub = self.create_publisher(
#             JointState,
#             '/joint_states',
#             10
#         )

#         # -------- Joint Angles --------
#         self.q = [0.0] * 12

#         # -------- Link Lengths (meters) --------
#         self.L1 = 0.05
#         self.L2 = 0.135
#         self.L3 = 0.1575

#         # -------- Default Targets --------
#         self.targets = [
#             [0.05, -0.05, 0.15],   # FR
#             [-0.05,  0.05, 0.15],  # FL
#             [-0.05,  0.05, 0.15],  # BL
#             [0.05, -0.05, 0.15],   # BR
#         ]

#         # -------- Joint Names (MATCH URDF EXACTLY) --------
#         self.joint_names = [
#             'CCUP4', 'THIGH4', 'FOOT4',
#             'CCUP3', 'THIGH3', 'FOOT3',
#             'CCUP2', 'THIGH2', 'FOOT2',
#             'CCUP1', 'THIGH1', 'FOOT1'

#             # # FRONT RIGHT
#             # 'fr_hip_yaw', 'fr_hip_pitch', 'fr_knee',

#             # # FRONT LEFT
#             # 'fl_hip_yaw', 'fl_hip_pitch', 'fl_knee',

#             # # BACK LEFT
#             # 'bl_hip_yaw', 'bl_hip_pitch', 'bl_knee',

#             # # BACK RIGHT
#             # 'br_hip_yaw', 'br_hip_pitch', 'br_knee',
#         ]

#         # -------- Timer --------
#         self.timer = self.create_timer(0.05, self.update)

#         # -------- Input Thread --------
#         threading.Thread(target=self.input_loop, daemon=True).start()

#         self.get_logger().info(
#             "\nEnter 4 leg coordinates:\n"
#             "x1 y1 z1, x2 y2 z2, x3 y3 z3, x4 y4 z4\n"
#         )

#     # -------- INPUT --------
#     def input_loop(self):
#         while True:
#             try:
#                 data = input("Enter 4 legs: ")
#                 sets = data.split(',')

#                 if len(sets) != 4:
#                     print("❌ Need 4 sets")
#                     continue

#                 new_targets = []

#                 for s in sets:
#                     vals = list(map(float, s.strip().split()))
#                     if len(vals) != 3:
#                         print("❌ Each set needs 3 values")
#                         break
#                     new_targets.append(vals)

#                 if len(new_targets) == 4:
#                     self.targets = new_targets
#                     print("✅ Targets updated")

#             except Exception as e:
#                 print("Error:", e)

#     # -------- IK LEFT --------
#     def IK_left(self, x, y, z):

#         L1, L2, L3 = self.L1, self.L2, self.L3

#         C = math.sqrt(x**2 + z**2)
#         D = math.sqrt(max(C**2 - L1**2, 0.0))
#         G = math.sqrt(D**2 + y**2)

#         def clamp(v): return max(-1.0, min(1.0, v))

#         J3 = math.acos(clamp((L2**2 + L3**2 - G**2) / (2 * L2 * L3))) - math.pi/2

#         J2 = math.atan2(y, D) + math.acos(
#             clamp((L2**2 + G**2 - L3**2) / (2 * L2 * G))
#         )

#         J1 = math.pi/2 - math.atan2(-x, z) - math.atan2(D, L1)

#         return J1, J2, J3

#     # -------- IK RIGHT --------
#     def IK_right(self, x, y, z):

#         L1, L2, L3 = self.L1, self.L2, self.L3

#         C = math.sqrt(x**2 + z**2)
#         D = math.sqrt(max(C**2 - L1**2, 0.0))
#         G = math.sqrt(D**2 + y**2)

#         def clamp(v): return max(-1.0, min(1.0, v))

#         J3 = math.acos(clamp((L2**2 + L3**2 - G**2) / (2 * L2 * L3))) - math.pi/2

#         J2 = -(math.atan2(y, D) + math.acos(
#             clamp((L2**2 + G**2 - L3**2) / (2 * L2 * G))
#         ))

#         J1 = math.atan2(x, z) + math.atan2(D, L1) - math.pi/2

#         return J1, J2, J3

#     # -------- MAIN LOOP --------
#     def update(self):

#         for i in range(4):

#             x, y, z = self.targets[i]

#             if i in [0, 3]:  # RIGHT: FR, BR
#                 J1, J2, J3 = self.IK_right(x, y, z)
#             else:            # LEFT: FL, BL
#                 J1, J2, J3 = self.IK_left(x, y, z)

#             base = i * 3
#             self.q[base]   = J1
#             self.q[base+1] = J2
#             self.q[base+2] = J3

#         # -------- Publish to RViz --------
#         msg = JointState()
#         msg.header.stamp = self.get_clock().now().to_msg()
#         msg.name = self.joint_names
#         msg.position = self.q

#         self.pub.publish(msg)


# def main(args=None):
#     rclpy.init(args=args)
#     node = RVizQuadIK()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()


import math
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class RVizQuadIK(Node):

    def __init__(self):
        super().__init__('rviz_quad_ik')

        # -------- Publisher (RViz uses JointState) --------
        self.pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        # -------- Joint Angles --------
        self.q = [0.0] * 12

        # -------- Link Lengths (meters) --------
        self.L1 = 0.05
        self.L2 = 0.135
        self.L3 = 0.1575

        # -------- Default Targets --------
        self.targets = [
            [0.05, -0.05, 0.15],   # FR
            [-0.05,  0.05, 0.15],  # FL
            [-0.05,  0.05, 0.15],  # BL
            [0.05, -0.05, 0.15],   # BR
        ]

        # -------- Joint Names (MATCH URDF EXACTLY) --------
        self.joint_names = [
            # FRONT RIGHT
            'fr_hip_yaw', 'fr_hip_pitch', 'fr_knee',

            # FRONT LEFT
            'fl_hip_yaw', 'fl_hip_pitch', 'fl_knee',

            # BACK LEFT
            'bl_hip_yaw', 'bl_hip_pitch', 'bl_knee',

            # BACK RIGHT
            'br_hip_yaw', 'br_hip_pitch', 'br_knee',
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

        J3 = math.acos(clamp((L2**2 + L3**2 - G**2) / (2 * L2 * L3)))

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

        J3 = math.acos(clamp((L2**2 + L3**2 - G**2) / (2 * L2 * L3)))

        J2 = math.pi/2 - math.atan2(-y, D) - math.asin(
            clamp((L3 * math.sin(J3)) / G if G != 0 else 0.0)
        )

        J1 = math.atan2(x, z) + math.atan2(D, L1)

        return J1, J2, J3

    # -------- MAIN LOOP --------
    def update(self):

        for i in range(4):

            x, y, z = self.targets[i]

            if i in [0, 3]:  # RIGHT: FR, BR
                J1, J2, J3 = self.IK_right(x, y, z)
            else:            # LEFT: FL, BL
                J1, J2, J3 = self.IK_left(x, y, z)

            base = i * 3
            self.q[base]   = J1
            self.q[base+1] = J2
            self.q[base+2] = J3

        # -------- Publish to RViz --------
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.q

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RVizQuadIK()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()