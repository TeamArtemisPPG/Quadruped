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

#         # sim:=true  -> command the Gazebo ros2_control position controller
#         # sim:=false -> publish fake /joint_states for RViz only (old behaviour)
#         self.declare_parameter('sim', True)
#         self.sim = self.get_parameter('sim').value

#         self.declare_parameter('left_scale', 1.0)
#         self.declare_parameter('right_scale', 1.0)

#         if self.sim:
#             self.pub = self.create_publisher(
#                 Float64MultiArray, '/joint_group_controller/commands', 10)
#         else:
#             self.pub = self.create_publisher(JointState, '/joint_states', 10)

#         self.joint_names = [
#             'CCUP1', 'THIGH1', 'FOOT1',
#             'CCUP2', 'THIGH2', 'FOOT2',
#             'CCUP3', 'THIGH3', 'FOOT3',
#             'CCUP4', 'THIGH4', 'FOOT4'
#         ]

#         self.q = [0.0] * 12

#         self.L1 = 0.05
#         self.L2 = 0.135
#         self.L3 = 0.1575

#         # Stance pose (same as mode 's'); matches the command sent by
#         # gz_harmonic.launch.py once the controller is active
#         self.stand = []
#         for i in range(4):
#             x = 0.05 if i in [0, 3] else -0.05
#             ik = self.IK_right if i in [0, 3] else self.IK_left
#             self.stand += list(ik(x, 0.0 - 0.04, 0.15))

#         self.step = 0
#         self.cycle_length = 20

#         # Current mode
#         self.mode = "f"   # default forward

#         self.timer = self.create_timer(0.05, self.update)

#         # Input thread
#         threading.Thread(target=self.input_loop, daemon=True).start()

#         self.get_logger().info(
#             "Type: f (forward), b (backward), r (right), l (left), s (stop)")

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
#     def forward(self, phase, scale=1.0):
#         step_height = 0.02
#         step_length = 0.02 * scale
#         base_z = 0.200

#         t = 2 * math.pi * phase
#         y = step_length * math.cos(t)
#         z = base_z + step_height * max(0, math.sin(t))
#         return y, z

#     def backward(self, phase, scale=1.0):
#         step_height = 0.06
#         step_length = 0.04 * scale
#         base_z = 0.200

#         t = 2 * math.pi * phase
#         y = -step_length * math.cos(t)
#         z = base_z + step_height * max(0, math.sin(t))
#         return y, z

#     def right(self, phase):
#         step_height = 0.05
#         step_length = 0.02
#         base_z = 0.175

#         t = 2 * math.pi * phase
#         x = step_length * math.cos(t)
#         z = base_z + step_height * max(0, math.sin(t))
#         return x, z

#     def left(self, phase):
#         step_height = 0.05
#         step_length = 0.02
#         base_z = 0.175

#         t = 2 * math.pi * phase
#         x = -step_length * math.cos(t)
#         z = base_z + step_height * max(0, math.sin(t))
#         return x, z

#     def stop(self, phase):
#         return 0.0, 0.15

#     # -------- IK LEFT --------
#     def IK_left(self, x, y, z):

#         L1, L2, L3 = self.L1, self.L2, self.L3

#         C = math.sqrt(x**2 + z**2)
#         D = math.sqrt(max(C**2 - L1**2, 0.0))
#         G = math.sqrt(D**2 + y**2)

#         def clamp(v): return max(-1.0, min(1.0, v))

#         J3 = math.acos(clamp((L2**2 + L3**2 - G**2) / (2 * L2 * L3))) - math.pi / 2

#         J2 = math.atan2(-y, D) + math.acos(
#             clamp((L2**2 + G**2 - L3**2) / (2 * L2 * G))
#         )

#         J1 = math.pi / 2 - math.atan2(-x, z) - math.atan2(D, L1)

#         return J1, J2, J3

#     # -------- IK RIGHT --------
#     def IK_right(self, x, y, z):

#         L1, L2, L3 = self.L1, self.L2, self.L3

#         C = math.sqrt(x**2 + z**2)
#         D = math.sqrt(max(C**2 - L1**2, 0.0))
#         G = math.sqrt(D**2 + y**2)

#         def clamp(v): return max(-1.0, min(1.0, v))

#         J3 = math.acos(clamp((L2**2 + L3**2 - G**2) / (2 * L2 * L3))) - math.pi / 2

#         J2 = -(math.atan2(-y, D) + math.acos(
#             clamp((L2**2 + G**2 - L3**2) / (2 * L2 * G))
#         ))

#         J1 = math.atan2(x, z) + math.atan2(D, L1) - math.pi / 2

#         return J1, J2, J3

#     # -------- MAIN LOOP --------
#     def update(self):

#         phase_offsets = [0.0, 0.5, 1.0, 1.5]

#         for i in range(4):

#             phase = ((self.step / self.cycle_length) + phase_offsets[i]) % 1.0

#             # Default stance
#             x = 0.05 if i in [0, 3] else -0.05
#             y = 0.0
#             z = 0.15

#             # Mode switch
#             side = 'left_scale' if i in [1, 2] else 'right_scale'
#             scale = self.get_parameter(side).value

#             if self.mode == 'f':
#                 y, z = self.forward(phase, scale)

#             elif self.mode == 'b':
#                 y, z = self.backward(phase, scale)

#             elif self.mode == 'r':
#                 dx, z = self.right(phase)
#                 if i in [2, 3]:   # front legs
#                     x += dx
#                     y += 0.01
#                 else:             # back legs
#                     x -= dx
#                     y -= 0.01

#             elif self.mode == 'l':
#                 dx, z = self.left(phase)
#                 if i in [2, 3]:   # front legs
#                     x += dx
#                     y += 0.01
#                 else:
#                     x -= dx
#                     y -= 0.01

#             elif self.mode == 's':
#                 y, z = self.stop(phase)

#             y -= 0.04

#             # IK
#             if i in [0, 3]:
#                 J1, J2, J3 = self.IK_right(x, y, z)
#             else:
#                 J1, J2, J3 = self.IK_left(x, y, z)

#             base = i * 3
#             self.q[base] = J1
#             self.q[base + 1] = J2
#             self.q[base + 2] = J3

#         self.step += 1

#         if self.sim:
#             # Ease in from the stance pose into the gait (~5 s at 20 Hz)
#             a = min(1.0, self.step / 100.0)
#             out = Float64MultiArray()
#             out.data = [s + a * (v - s) for s, v in zip(self.stand, self.q)]
#             self.pub.publish(out)
#             return

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


class QuadCrawl(Node):

    def __init__(self):
        super().__init__('quad_crawl')

        # Robot geometry (metres)
        self.L1 = 0.050
        self.L2 = 0.135
        self.L3 = 0.1575

        # Foot-path parameters (metres)
        self.step_length = 0.010
        self.step_height = 0.010

        # Centre of the foot path in the IK coordinate frame.
        # Required because IK needs absolute foot coordinates.
        self.center_y = 0.010
        self.center_z = 0.150

        # Timing — same style as the hexapod controller
        self.dt = 0.05
        self.step_time = 0.5
        self.step_timer = 0.0
        self.step_index = 0
        self.num_steps = 4

        # Leg order: BR, BL, FL, FR
        self.joint_names = [
            'CCUP1', 'THIGH1', 'FOOT1',
            'CCUP2', 'THIGH2', 'FOOT2',
            'CCUP3', 'THIGH3', 'FOOT3',
            'CCUP4', 'THIGH4', 'FOOT4'
        ]

        # Two diagonal pairs, offset by half a cycle
        self.phase_offsets = [0, 2, 0, 2]

        self.STAND = [
            0.0, -1.4002, -0.46137,  # BR
            0.0,  1.4002, -0.46137,  # BL
            0.0,  1.4002, -0.46137,  # FL
            0.0, -1.4002, -0.46137   # FR
        ]

        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/joint_group_controller/commands',
            10
        )

        self.command = 's'

        self.timer = self.create_timer(self.dt, self.update)

        threading.Thread(
            target=self.keyboard_input,
            daemon=True
        ).start()

        self.get_logger().info(
            'Quadruped controller ready: f = move, s = stand'
        )

    # ---------- Inverse kinematics ----------

    @staticmethod
    def clamp(value):
        return max(-1.0, min(1.0, value))

    def IK_left(self, x, y, z):
        L1, L2, L3 = self.L1, self.L2, self.L3

        C = math.sqrt(x**2 + z**2)
        D = math.sqrt(max(C**2 - L1**2, 0.0))
        G = math.sqrt(D**2 + y**2)

        J3 = math.acos(self.clamp(
            (L2**2 + L3**2 - G**2) / (2 * L2 * L3)
        )) - math.pi / 2

        J2 = math.atan2(-y, D) + math.acos(self.clamp(
            (L2**2 + G**2 - L3**2) / (2 * L2 * G)
        ))

        J1 = math.pi / 2 - math.atan2(-x, z) - math.atan2(D, L1)

        return J1, J2, J3

    def IK_right(self, x, y, z):
        L1, L2, L3 = self.L1, self.L2, self.L3

        C = math.sqrt(x**2 + z**2)
        D = math.sqrt(max(C**2 - L1**2, 0.0))
        G = math.sqrt(D**2 + y**2)

        J3 = math.acos(self.clamp(
            (L2**2 + L3**2 - G**2) / (2 * L2 * L3)
        )) - math.pi / 2

        J2 = -(math.atan2(-y, D) + math.acos(self.clamp(
            (L2**2 + G**2 - L3**2) / (2 * L2 * G)
        )))

        J1 = math.atan2(x, z) + math.atan2(D, L1) - math.pi / 2

        return J1, J2, J3

    # ---------- Leg helpers ----------

    def is_right(self, leg):
        return leg in (0, 3)

    def ik_for_leg(self, leg, x, y, z):
        if self.is_right(leg):
            return self.IK_right(x, y, z)
        return self.IK_left(x, y, z)

    # ---------- Four calculated path points ----------

    def foot_points(self, leg):
        x = 0.05 if self.is_right(leg) else -0.05

        yc = self.center_y
        zc = self.center_z
        L = self.step_length
        H = self.step_height

        # A = centre, B/C = horizontal endpoints, D = raised point.
        # Lower z means lifting the foot in this robot's coordinate setup.
        A = (x, yc, zc)
        B = (x, yc - L, zc)
        D = (x, yc, zc - H)
        C = (x, yc + L, zc)

        # Repeating order: A -> B -> D -> C -> A
        return [A, B, D, C]

    # ---------- Gait update ----------

    def update_gait(self):
        if self.command != 'f':
            return

        self.step_timer += self.dt

        if self.step_timer >= self.step_time:
            self.step_timer = 0.0
            self.step_index = (
                self.step_index + 1
            ) % self.num_steps

    # ---------- Main loop ----------

    def update(self):
        self.update_gait()

        if self.command == 's':
            positions = list(self.STAND)

        else:
            positions = []

            for leg in range(4):
                idx = (
                    self.step_index + self.phase_offsets[leg]
                ) % self.num_steps

                x, y, z = self.foot_points(leg)[idx]
                q1, q2, q3 = self.ik_for_leg(leg, x, y, z)

                positions.extend([q1, q2, q3])

        msg = Float64MultiArray()
        msg.data = positions
        self.publisher.publish(msg)

    # ---------- Keyboard input ----------

    def keyboard_input(self):
        while rclpy.ok():
            try:
                cmd = input('Command (f=move, s=stand): ').strip().lower()
            except EOFError:
                break

            if cmd == 'f':
                self.step_index = 0
                self.step_timer = 0.0
                self.command = 'f'
                self.get_logger().info('Path cycling started.')

            elif cmd == 's':
                self.command = 's'
                self.get_logger().info('Standing pose commanded.')

            else:
                print("Enter 'f' or 's'.")


def main(args=None):
    rclpy.init(args=args)
    node = QuadCrawl()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
