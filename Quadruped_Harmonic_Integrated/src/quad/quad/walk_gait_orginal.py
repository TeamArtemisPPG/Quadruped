# #!/usr/bin/env python3

# import math
# import threading
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float64MultiArray
# from sensor_msgs.msg import JointState


# class GazeboForwardGait(Node):

#     def __init__(self):
#         super().__init__('gazebo_forward_gait')

#         # self.pub = self.create_publisher(
#         #     Float64MultiArray,
#         #     '/joint_group_controller/commands',
#         #     10
#         # )

#         self.pub = self.create_publisher(
#             JointState,
#             '/joint_states',
#             10
#         )

#         self.joint_names = [
#             'CCUP1', 'THIGH1', 'FOOT1',
#             'CCUP2', 'THIGH2', 'FOOT2',
#             'CCUP3', 'THIGH3', 'FOOT3',
#             'CCUP4', 'THIGH4', 'FOOT4'
#         ]

#         # 👉 OPTIONAL (clean)
#         self.msg = JointState()
#         self.msg.name = self.joint_names

#         self.q = [0.0]*12

#         self.L1 = 0.05
#         self.L2 = 0.135
#         self.L3 = 0.1575

#         self.step = 0
#         self.cycle_length = 40

#         # 🔥 CURRENT MODE
#         self.mode = "f"   # default forward

#         self.timer = self.create_timer(0.05, self.update)

#         # 🔥 INPUT THREAD
#         threading.Thread(target=self.input_loop, daemon=True).start()

#         self.get_logger().info("Type: f (forward), b (backward), r (right), l (left), s (stop)")

#     # -------- INPUT THREAD --------
#     def input_loop(self):
#         while True:
#             cmd = input("Command: ").strip().lower()

#             if cmd in ['f', 'b', 'r', 'l', 's']:
#                 self.mode = cmd
#                 print(f"Switched to: {cmd}")
#             else:
#                 print("Invalid input (use f/b/r/l/s)")

#     # -------- TRAJECTORIES --------
#     def forward(self, phase):
#         step_height = 0.05
#         step_length = 0.04
#         base_z = 0.15

#         t = 2 * math.pi * phase
#         y = step_length * math.cos(t)
#         z = base_z + step_height * max(0, math.sin(t))
#         return y, z

#     def backward(self, phase):
#         step_height = 0.05
#         step_length = 0.04
#         base_z = 0.15

#         t = 2 * math.pi * phase
#         y = -step_length * math.cos(t)
#         z = base_z + step_height * max(0, math.sin(t))
#         return y, z

#     def right(self, phase):
#         step_height = 0.05
#         step_length = 0.02
#         base_z = 0.15

#         t = 2 * math.pi * phase
#         x = step_length * math.cos(t)
#         z = base_z + step_height * max(0, math.sin(t))
#         return x, z

#     def left(self, phase):
#         step_height = 0.05
#         step_length = 0.02
#         base_z = 0.15

#         t = 2 * math.pi * phase
#         x = -step_length * math.cos(t)
#         z = base_z + step_height * max(0, math.sin(t))
#         return x, z

#     def stop(self, phase):
#         return 0.0, 0.15

#     # -------- IK --------
#     def IK_left(self, x, y, z):
#         L1, L2, L3 = self.L1, self.L2, self.L3

#         C = math.sqrt(x**2 + z**2)
#         D = math.sqrt(max(C**2 - L1**2, 0.0))
#         G = math.sqrt(D**2 + y**2)

#         def clamp(v): return max(-1.0, min(1.0, v))

#         J3 = -math.pi/2 + math.acos(clamp((L2**2 + L3**2 - G**2)/(2*L2*L3)))

#         J2 = math.pi/2 + math.atan2(-y, D) + math.asin(
#             clamp((L3 * math.sin(J3)) / G if G != 0 else 0.0)
#         )

#         J1 = math.pi/2 - math.atan2(-x, z) - math.atan2(D, L1)

#         return J1, J2, J3

#     def IK_right(self, x, y, z):
#         L1, L2, L3 = self.L1, self.L2, self.L3

#         C = math.sqrt(x**2 + z**2)
#         D = math.sqrt(max(C**2 - L1**2, 0.0))
#         G = math.sqrt(D**2 + y**2)

#         def clamp(v): return max(-1.0, min(1.0, v))

#         J3 = -math.pi/2 + math.acos(clamp((L2**2 + L3**2 - G**2)/(2*L2*L3)))

#         J2 = -math.pi/2 - math.atan2(-y, D) - math.asin(
#             clamp((L3 * math.sin(J3)) / G if G != 0 else 0.0)
#         )

#         J1 = -math.pi/2 + math.atan2(x, z) + math.atan2(D, L1)

#         return J1, J2, J3

#     # -------- MAIN LOOP --------
#     def update(self):

#         phase_offsets = [0.0, 0.5, 0.0, 0.5]

#         for i in range(4):

#             phase = ((self.step / self.cycle_length) + phase_offsets[i]) % 1.0

#             # Default stance
#             x = 0.05 if i in [0, 3] else -0.05
#             y = 0.0
#             z = 0.15

#             # 🔥 MODE SWITCH
#             if self.mode == 'f':
#                 y, z = self.forward(phase)

#             elif self.mode == 'b':
#                 y, z = self.backward(phase)

#             elif self.mode == 'r':
#                 dx, z = self.right(phase)
#                 x += dx

#             elif self.mode == 'l':
#                 dx, z = self.left(phase)
#                 x += dx

#             elif self.mode == 's':
#                 y, z = self.stop(phase)

#             # IK
#             if i in [0, 3]:
#                 J1, J2, J3 = self.IK_right(x, y, z)
#             else:
#                 J1, J2, J3 = self.IK_left(x, y, z)

#             base = i * 3
#             self.q[base]   = J1
#             self.q[base+1] = J2
#             self.q[base+2] = J3

#         self.step += 1

#         # if self.step % 10 == 0:
#         #     for i in range(4):
#         #         base = i * 3
#         #         self.get_logger().info(
#         #             f"Leg {i}: J1={self.q[base]:.3f}, "
#         #             f"J2={self.q[base+1]:.3f}, "
#         #             f"J3={self.q[base+2]:.3f}"
#         #         )

#         # msg = Float64MultiArray()
#         # msg.data = self.q
#         # self.pub.publish(msg)

#         msg = JointState()
#         msg.header.stamp = self.get_clock().now().to_msg()
#         msg.name = self.joint_names
#         msg.position = self.q

#         self.pub.publish(msg)


# def main(args=None):
#     rclpy.init(args=args)
#     node = GazeboForwardGait()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()


#!/usr/bin/env python3

import math
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


class GazeboForwardGait(Node):

    def __init__(self):
        super().__init__('gazebo_forward_gait')

        # self.pub = self.create_publisher(
        #     Float64MultiArray,
        #     '/joint_group_controller/commands',
        #     10
        # )

        self.pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        self.joint_names = [
            'CCUP1', 'THIGH1', 'FOOT1',
            'CCUP2', 'THIGH2', 'FOOT2',
            'CCUP3', 'THIGH3', 'FOOT3',
            'CCUP4', 'THIGH4', 'FOOT4'
        ]

        # 👉 OPTIONAL (clean)
        self.msg = JointState()
        self.msg.name = self.joint_names

        self.q = [0.0]*12

        self.L1 = 0.05
        self.L2 = 0.135
        self.L3 = 0.1575

        self.step = 0
        self.cycle_length = 40

        # 🔥 CURRENT MODE
        self.mode = "f"   # default forward

        self.timer = self.create_timer(0.05, self.update)

        # 🔥 INPUT THREAD
        threading.Thread(target=self.input_loop, daemon=True).start()

        self.get_logger().info("Type: f (forward), b (backward), r (right), l (left), s (stop)")

    # -------- INPUT THREAD --------
    def input_loop(self):
        while True:
            cmd = input("Command: ").strip().lower()

            if cmd in ['f', 'b', 'r', 'l', 's']:
                self.mode = cmd
                print(f"Switched to: {cmd}")
            else:
                print("Invalid input (use f/b/r/l/s)")

    # -------- TRAJECTORIES --------
    def forward(self, phase):
        step_height= 0.04
        step_length = 0.05
        base_z = 0.15

        t = 2 * math.pi * phase
        y = step_length * math.cos(t)
        z = base_z + step_height* max(0, math.sin(t))
        return y, z

    def backward(self, phase):
        step_height= 0.04
        step_length = 0.05
        base_z = 0.15

        t = 2 * math.pi * phase
        y = -step_length * math.cos(t)
        z = base_z + step_height* max(0, math.sin(t))
        return y, z

    def right(self, phase):
        step_height= 0.04
        step_length = 0.05
        base_z = 0.15

        t = 2 * math.pi * phase
        x = step_length * math.cos(t)
        z = base_z + step_height* max(0, math.sin(t))
        return x, z

    def left(self, phase):
        step_height = 0.04
        step_length = 0.05
        base_z = 0.15

        t = 2 * math.pi * phase
        x = -step_length * math.cos(t)
        z = base_z + step_height* max(0, math.sin(t))
        return x, z

    def stop(self, phase):
        return 0.0, 0.15

    # -------- IK --------
    def IK_left(self, x, y, z):
        L1, L2, L3 = self.L1, self.L2, self.L3

        C = math.sqrt(x**2 + z**2)
        D = math.sqrt(max(C**2 - L1**2, 0.0))
        G = math.sqrt(D**2 + y**2)

        def clamp(v): return max(-1.0, min(1.0, v))

        J3 = -math.pi/2 + math.acos(clamp((L2**2 + L3**2 - G**2)/(2*L2*L3)))

        J2 = math.pi/2 + math.atan2(-y, D) + math.asin(
            clamp((L3 * math.sin(J3)) / G if G != 0 else 0.0)
        )

        J1 = math.pi/2 - math.atan2(-x, z) - math.atan2(D, L1)

        return J1, J2, J3

    def IK_right(self, x, y, z):
        L1, L2, L3 = self.L1, self.L2, self.L3

        C = math.sqrt(x**2 + z**2)
        D = math.sqrt(max(C**2 - L1**2, 0.0))
        G = math.sqrt(D**2 + y**2)

        def clamp(v): return max(-1.0, min(1.0, v))

        J3 = -math.pi/2 + math.acos(clamp((L2**2 + L3**2 - G**2)/(2*L2*L3)))

        J2 = -math.pi/2 - math.atan2(-y, D) - math.asin(
            clamp((L3 * math.sin(J3)) / G if G != 0 else 0.0)
        )

        J1 = -math.pi/2 + math.atan2(x, z) + math.atan2(D, L1)

        return J1, J2, J3

    # -------- MAIN LOOP --------
    def update(self):

        phase_offsets = [0.0, 0.5, 0.0, 0.5]

        for i in range(4):

            phase = ((self.step / self.cycle_length) + phase_offsets[i]) % 1.0

            # Default stance
            x = 0.05 if i in [0, 3] else -0.05
            y = 0.0
            z = 0.15

            # 🔥 MODE SWITCH
            if self.mode == 'f':
                y, z = self.forward(phase)

            elif self.mode == 'b':
                y, z = self.backward(phase)

            elif self.mode == 'r':
                dx, z = self.right(phase)
                x += dx

            elif self.mode == 'l':
                dx, z = self.left(phase)
                x += dx

            elif self.mode == 's':
                y, z = self.stop(phase)

            # IK
            if i in [0, 3]:
                J1, J2, J3 = self.IK_right(x, y, z)
            else:
                J1, J2, J3 = self.IK_left(x, y, z)

            base = i * 3
            self.q[base]   = J1
            self.q[base+1] = J2
            self.q[base+2] = J3

        self.step += 1

        # if self.step % 10 == 0:
        #     for i in range(4):
        #         base = i * 3
        #         self.get_logger().info(
        #             f"Leg {i}: J1={self.q[base]:.3f}, "
        #             f"J2={self.q[base+1]:.3f}, "
        #             f"J3={self.q[base+2]:.3f}"
        #         )

        # msg = Float64MultiArray()
        # msg.data = self.q
        # self.pub.publish(msg)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.q

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboForwardGait()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()