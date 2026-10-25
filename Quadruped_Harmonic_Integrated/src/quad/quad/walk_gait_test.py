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
#         # sim:=false -> publish fake /joint_states for RViz only
#         self.declare_parameter('sim', True)
#         self.sim = self.get_parameter('sim').value

#         # Differential drive scaling to trim any left/right drift
#         self.declare_parameter('left_scale', 0.96)
#         self.declare_parameter('right_scale', 1.0)

#         # Stride & clearance tuning
#         self.declare_parameter('step_length', 0.025)
#         self.declare_parameter('step_height', 0.03)

#         if self.sim:
#             self.pub = self.create_publisher(
#                 Float64MultiArray, '/joint_group_controller/commands', 10)
#         else:
#             self.pub = self.create_publisher(JointState, '/joint_states', 10)

#         # Joint order matching controllers_harmonic.yaml and URDF:
#         # Index 0: Leg 1 - Back Right (BR)
#         # Index 1: Leg 2 - Back Left  (BL)
#         # Index 2: Leg 3 - Front Left (FL)
#         # Index 3: Leg 4 - Front Right(FR)
#         self.joint_names = [
#             'CCUP1', 'THIGH1', 'FOOT1',
#             'CCUP2', 'THIGH2', 'FOOT2',
#             'CCUP3', 'THIGH3', 'FOOT3',
#             'CCUP4', 'THIGH4', 'FOOT4'
#         ]

#         self.q = [0.0] * 12

#         # Link lengths (meters)
#         self.L1 = 0.05
#         self.L2 = 0.135
#         self.L3 = 0.1575

#         # Nominal height
#         self.nominal_z = 0.15

#         # Longitudinal offsets:
#         # Back legs (legs 1 & 2): -0.045 m positions the rear feet at X ≈ -0.180 m (rear chassis boundary is -0.150 m).
#         # With step_length = 0.025 m, the back feet oscillate between -0.188 m and -0.169 m, GUARANTEEING they always stay BEHIND the chassis.
#         # Front legs (legs 3 & 4): +0.070 m positions the front feet forward (X ≈ +0.082 m) to support the front half and eliminate pitch shaking.
#         self.nominal_y_back = -0.045
#         self.nominal_y_front = +0.010
#         self.nominal_z_back = +0.00
#         self.nominal_z_front = +0.0


#         self.nominal_z_left = +0.0
#         self.nominal_z_right = 0.0

#         # Baseline stance pose (mode 's')
#         self.stand = []
#         for i in range(4):
#             x = 0.05 if i in [0, 3] else -0.05
#             nom_y = self.nominal_y_front if i in [2, 3] else self.nominal_y_back
#             ik = self.IK_right if i in [0, 3] else self.IK_left
#             nom_z_fb = self.nominal_z_front if i in [2, 3] else self.nominal_z_back
#             nom_z_lr = self.nominal_z_left if i in [1, 2] else self.nominal_z_right
#             self.stand += list(ik(x, nom_y, self.nominal_z + nom_z_fb + nom_z_lr))

#         self.step = 0
#         # 30 steps @ 20 Hz = 1.5 s cycle (0.75 s per half-stride)
#         self.cycle_length = 30

#         # Current mode
#         self.mode = "f"   # default forward

#         self.timer = self.create_timer(0.05, self.update)

#         # Input thread
#         threading.Thread(target=self.input_loop, daemon=True).start()

#         self.get_logger().info(
#             "Gait controller active. Back legs positioned behind chassis. Commands: f (forward), b (backward), r (right), l (left), s (stop)")

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
#         step_height = self.get_parameter('step_height').value
#         step_length = self.get_parameter('step_length').value * scale
#         base_z = self.nominal_z

#         t = 2.0 * math.pi * phase
#         # Longitudinal motion: swings forward in air (0->pi), pushes backward on ground (pi->2pi)
#         y = -step_length * math.cos(t)
#         # Vertical motion: lifts foot by reducing z depth during swing
#         z = base_z - step_height * max(0.0, math.sin(t))
#         return y, z

#     def backward(self, phase, scale=1.0):
#         step_height = self.get_parameter('step_height').value
#         step_length = self.get_parameter('step_length').value * scale
#         base_z = self.nominal_z

#         t = 2.0 * math.pi * phase
#         y = step_length * math.cos(t)
#         z = base_z - step_height * max(0.0, math.sin(t))
#         return y, z

#     def right(self, phase):
#         step_height = self.get_parameter('step_height').value
#         step_length = 0.02
#         base_z = self.nominal_z

#         t = 2.0 * math.pi * phase
#         x = -step_length * math.cos(t)
#         z = base_z - step_height * max(0.0, math.sin(t))
#         return x, z

#     def left(self, phase):
#         step_height = self.get_parameter('step_height').value
#         step_length = 0.02
#         base_z = self.nominal_z

#         t = 2.0 * math.pi * phase
#         x = step_length * math.cos(t)
#         z = base_z - step_height * max(0.0, math.sin(t))
#         return x, z

#     def stop(self, phase):
#         return 0.0, self.nominal_z

#     # -------- IK LEFT (Legs 2 and 3) --------
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

#     # -------- IK RIGHT (Legs 1 and 4) --------
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
#         # Diagonal trot pairing:
#         # Leg 1 (BR, idx 0) & Leg 3 (FL, idx 2) -> Pair A (phase offset 0.0)
#         # Leg 2 (BL, idx 1) & Leg 4 (FR, idx 3) -> Pair B (phase offset 0.5)
#         phase_offsets = [0.0, 0.5, 0.75, 1.25]

#         for i in range(4):
#             phase = ((self.step / self.cycle_length) + phase_offsets[i]) % 1.0

#             # Lateral stance offsets
#             x = 0.05 if i in [0, 3] else -0.05
#             y = 0.0
#             z = self.nominal_z

#             # Apply scale per side to trim left/right heading drift
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

#             # Apply separate front/back stance offsets:
#             # - Back legs: stays strictly behind the rear chassis line (X <= -0.15 m)
#             # - Front legs: extends forward to support front weight
#             # Apply front/back Y offsets
#             nom_y = self.nominal_y_front if i in [2, 3] else self.nominal_y_back
#             y += nom_y

#             # Apply front/back Z offsets
#             nom_z_fb = self.nominal_z_front if i in [2, 3] else self.nominal_z_back

#             # Apply left/right Z offsets
#             nom_z_lr = self.nominal_z_left if i in [1, 2] else self.nominal_z_right

#             z += nom_z_fb + nom_z_lr

#             # Inverse kinematics
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
#             # Ease in smoothly from stance pose to gait (~2.5 s at 20 Hz)
#             a = min(1.0, self.step / 50.0)
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
from sensor_msgs.msg import JointState


class GazeboForwardGait(Node):

    def __init__(self):
        super().__init__('gazebo_forward_gait')

        # sim:=true  -> command the Gazebo ros2_control position controller
        # sim:=false -> publish fake /joint_states for RViz only
        self.declare_parameter('sim', True)
        self.sim = self.get_parameter('sim').value

        # Differential drive scaling to trim any left/right drift
        self.declare_parameter('left_scale', 1.0)
        self.declare_parameter('right_scale', 1.0)

        # Stride & clearance tuning
        self.declare_parameter('step_length', 0.025)
        self.declare_parameter('front_step_height', 0.20)
        self.declare_parameter('back_step_height', 0.25)

        if self.sim:
            self.pub = self.create_publisher(
                Float64MultiArray, '/joint_group_controller/commands', 10)
        else:
            self.pub = self.create_publisher(JointState, '/joint_states', 10)

        # Joint order matching controllers_harmonic.yaml and URDF:
        # Index 0: Leg 1 - Back Right (BR)
        # Index 1: Leg 2 - Back Left  (BL)
        # Index 2: Leg 3 - Front Left (FL)
        # Index 3: Leg 4 - Front Right(FR)
        self.joint_names = [
            'CCUP1', 'THIGH1', 'FOOT1',
            'CCUP2', 'THIGH2', 'FOOT2',
            'CCUP3', 'THIGH3', 'FOOT3',
            'CCUP4', 'THIGH4', 'FOOT4'
        ]

        self.q = [0.0] * 12

        # Link lengths (meters)
        self.L1 = 0.05
        self.L2 = 0.135
        self.L3 = 0.1575

        # Nominal height
        self.nominal_z = 0.25

        # Longitudinal offsets:
        # Back legs (legs 1 & 2): -0.045 m positions the rear feet at X ≈ -0.180 m (rear chassis boundary is -0.150 m).
        # With step_length = 0.025 m, the back feet oscillate between -0.188 m and -0.169 m, GUARANTEEING they always stay BEHIND the chassis.
        # Front legs (legs 3 & 4): +0.070 m positions the front feet forward (X ≈ +0.082 m) to support the front half and eliminate pitch shaking.
        self.nominal_y_back = -0.045
        self.nominal_y_front = +0.010

        # Baseline stance pose (mode 's')
        self.stand = []
        for i in range(4):
            x = 0.1 if i in [0, 3] else -0.1
            nom_y = self.nominal_y_front if i in [2, 3] else self.nominal_y_back
            ik = self.IK_right if i in [0, 3] else self.IK_left
            self.stand += list(ik(x, nom_y, self.nominal_z))

        self.step = 0
        # 30 steps @ 20 Hz = 1.5 s cycle (0.75 s per half-stride)
        self.cycle_length = 30

        # Current mode
        self.mode = "f"   # default forward

        self.timer = self.create_timer(0.05, self.update)

        # Input thread
        threading.Thread(target=self.input_loop, daemon=True).start()

        self.get_logger().info(
            "Gait controller active. Back legs positioned behind chassis. Commands: f (forward), b (backward), r (right), l (left), s (stop)")

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
    # def forward(self, phase, scale=1.0):
    #     step_height = self.get_parameter('step_height').value
    #     step_length = self.get_parameter('step_length').value * scale

    #     # Half the cycle is swing; half is stance.
    #     if phase < 0.5:
    #         # Swing: move from rear to front while lifting the foot.
    #         u = phase / 0.5

    #         # Smooth horizontal movement from -step_length to +step_length.
    #         s = 3.0 * u**2 - 2.0 * u**3
    #         y = -step_length + 2.0 * step_length * s

    #         # Zero vertical velocity at both ends of the swing.
    #         z = self.nominal_z - step_height * math.sin(math.pi * u)

    #     else:
    #         # Stance: move from front to rear at ground-contact height.
    #         u = (phase - 0.5) / 0.5

    #         s = 3.0 * u**2 - 2.0 * u**3
    #         y = step_length - 2.0 * step_length * s
    #         z = self.nominal_z

    #     return y, z


    def forward(self, phase, scale=1.0, step_height=None):
        if step_height is None:
            step_height = self.get_parameter('front_step_height').value
        step_length = self.get_parameter('step_length').value * scale
        base_z = self.nominal_z

        # Swing phase: move the foot forward and lift it.
        if phase < 0.5:
            u = phase / 0.5

            # Smooth horizontal motion.
            s = 3.0 * u**2 - 2.0 * u**3
            y = -step_length + 2.0 * step_length * s

            # Vertical clearance.
            z = base_z - step_height * math.sin(math.pi * u)

        # Stance phase: move the foot backward at nominal height.
        else:
            u = (phase - 0.5) / 0.5
            s = 3.0 * u**2 - 2.0 * u**3
            y = step_length - 2.0 * step_length * s
            z = base_z
        return y, z


    def backward(self, phase, scale=1.0):
        step_height = self.get_parameter('step_height').value
        step_length = self.get_parameter('step_length').value * scale
        base_z = self.nominal_z

        t = 2.0 * math.pi * phase
        y = step_length * math.cos(t)
        z = base_z - step_height * max(0.0, math.sin(t))
        return y, z

    def right(self, phase):
        step_height = self.get_parameter('step_height').value
        step_length = 0.02
        base_z = self.nominal_z

        t = 2.0 * math.pi * phase
        x = -step_length * math.cos(t)
        z = base_z - step_height * max(0.0, math.sin(t))
        return x, z

    def left(self, phase):
        step_height = self.get_parameter('step_height').value
        step_length = 0.02
        base_z = self.nominal_z

        t = 2.0 * math.pi * phase
        x = step_length * math.cos(t)
        z = base_z - step_height * max(0.0, math.sin(t))
        return x, z

    def stop(self, phase):
        return 0.0, self.nominal_z

    # -------- IK LEFT (Legs 2 and 3) --------
    def IK_left(self, x, y, z):
        L1, L2, L3 = self.L1, self.L2, self.L3

        C = math.sqrt(x**2 + z**2)
        D = math.sqrt(max(C**2 - L1**2, 0.0))
        G = math.sqrt(D**2 + y**2)

        def clamp(v): return max(-1.0, min(1.0, v))

        J3 = math.acos(clamp((L2**2 + L3**2 - G**2) / (2 * L2 * L3))) - math.pi / 2

        J2 = math.atan2(-y, D) + math.acos(
            clamp((L2**2 + G**2 - L3**2) / (2 * L2 * G))
        )

        J1 = math.pi / 2 - math.atan2(-x, z) - math.atan2(D, L1)

        return J1, J2, J3

    # -------- IK RIGHT (Legs 1 and 4) --------
    def IK_right(self, x, y, z):
        L1, L2, L3 = self.L1, self.L2, self.L3

        C = math.sqrt(x**2 + z**2)
        D = math.sqrt(max(C**2 - L1**2, 0.0))
        G = math.sqrt(D**2 + y**2)

        def clamp(v): return max(-1.0, min(1.0, v))

        J3 = math.acos(clamp((L2**2 + L3**2 - G**2) / (2 * L2 * L3))) - math.pi / 2

        J2 = -(math.atan2(-y, D) + math.acos(
            clamp((L2**2 + G**2 - L3**2) / (2 * L2 * G))
        ))

        J1 = math.atan2(x, z) + math.atan2(D, L1) - math.pi / 2

        return J1, J2, J3

    # -------- MAIN LOOP --------
    def update(self):
        phase_offsets = [0.0, 0.5, 0.75, 1.25]

        for i in range(4):
            phase = ((self.step / self.cycle_length) + phase_offsets[i]) % 1.0

            # Lateral stance offsets
            x = 0.05 if i in [0, 3] else -0.05
            y = 0.0
            z = self.nominal_z

            # Apply scale per side to trim left/right heading drift
            side = 'left_scale' if i in [1, 2] else 'right_scale'
            scale = self.get_parameter(side).value

            if self.mode == 'f':
                if i in [0, 1]:  # Rear legs: BR and BL
                    height = self.get_parameter('back_step_height').value
                else:            # Front legs: FL and FR
                    height = self.get_parameter('front_step_height').value
                y, z = self.forward(phase,scale,step_height=height)

            elif self.mode == 'b':
                y, z = self.backward(phase, scale)

            elif self.mode == 'r':
                dx, z = self.right(phase)
                if i in [2, 3]:   # front legs
                    x += dx
                    y += 0.01
                else:             # back legs
                    x -= dx
                    y -= 0.01

            elif self.mode == 'l':
                dx, z = self.left(phase)
                if i in [2, 3]:   # front legs
                    x += dx
                    y += 0.01
                else:
                    x -= dx
                    y -= 0.01

            elif self.mode == 's':
                y, z = self.stop(phase)

            # Apply separate front/back stance offsets:
            # - Back legs: stays strictly behind the rear chassis line (X <= -0.15 m)
            # - Front legs: extends forward to support front weight
            nom_y = self.nominal_y_front if i in [2, 3] else self.nominal_y_back
            y += nom_y

            # Inverse kinematics
            if i in [0, 3]:
                J1, J2, J3 = self.IK_right(x, y, z)
            else:
                J1, J2, J3 = self.IK_left(x, y, z)

            base = i * 3
            self.q[base] = J1
            self.q[base + 1] = J2
            self.q[base + 2] = J3

        self.step += 1

        if self.sim:
            # Ease in smoothly from stance pose to gait (~2.5 s at 20 Hz)
            a = min(1.0, self.step / 50.0)
            out = Float64MultiArray()
            out.data = [s + a * (v - s) for s, v in zip(self.stand, self.q)]
            self.pub.publish(out)
            return

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