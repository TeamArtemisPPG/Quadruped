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

        # --------------------------------------------------
        # 1. PARAMETERS
        # --------------------------------------------------
        self.declare_parameter('sim', True)
        self.declare_parameter('left_scale', 1.0)
        self.declare_parameter('right_scale', 1.0)

        self.declare_parameter('step_length', 0.025)
        self.declare_parameter('lateral_step_length', 0.020)
        self.declare_parameter('front_step_height', 0.022)
        self.declare_parameter('back_step_height', 0.035)

        self.declare_parameter('cycle_length', 30)
        self.declare_parameter('control_period', 0.05)
        self.declare_parameter('startup_blend_steps', 50)

        # Leg order: BR, BL, FL, FR.
        # Each successive leg is offset by 0.75 cycle modulo 1.
        self.declare_parameter('phase_offsets', [0.0, 0.5, 0.75, 0.25])
        self.sim = self.get_parameter('sim').value
        self.cycle_length = max(1, int(self.get_parameter('cycle_length').value))
        self.blend_steps = max(1, int(self.get_parameter('startup_blend_steps').value))
        self.phase_offsets = list(self.get_parameter('phase_offsets').value)
        if len(self.phase_offsets) != 4:
            raise ValueError('phase_offsets must contain exactly four values.')

        # --------------------------------------------------
        # 2. PUBLISHER
        # --------------------------------------------------
        if self.sim:
            self.pub = self.create_publisher(
                Float64MultiArray,
                '/joint_group_controller/commands',
                10
            )
        else:
            self.pub = self.create_publisher(
                JointState, '/joint_states', 10
            )

        # Joint order must match controllers_harmonic.yaml.
        self.joint_names = [
            'CCUP1', 'THIGH1', 'FOOT1',  # BR
            'CCUP2', 'THIGH2', 'FOOT2',  # BL
            'CCUP3', 'THIGH3', 'FOOT3',  # FL
            'CCUP4', 'THIGH4', 'FOOT4'   # FR
        ]

        # --------------------------------------------------
        # 3. ROBOT GEOMETRY AND NOMINAL FOOT TARGETS
        # --------------------------------------------------
        self.L1 = 0.050
        self.L2 = 0.135
        self.L3 = 0.1575

        self.nominal_z = 0.15
        self.nominal_y_back = -0.045
        self.nominal_y_front = 0.010

        self.q = [0.0] * 12
        self.step = 0
        self.mode = 'f'

        # Calculate the initial standing joint configuration.
        self.stand = self._calculate_stand_pose()

        # Run the controller at the configured update rate.
        period = float(self.get_parameter('control_period').value)
        if period <= 0:
            raise ValueError('control_period must be positive.')
        self.timer = self.create_timer(period, self.update)
        # Keyboard input runs separately so it cannot block the timer.
        threading.Thread(target=self.input_loop, daemon=True).start()
        self.get_logger().info('Gait controller ready: f/b/r/l/s')

    # ======================================================
    # 4. LEG HELPERS
    # ======================================================

    def _is_front(self, leg):
        """Leg indices: 0=BR, 1=BL, 2=FL, 3=FR."""
        return leg in (2, 3)

    def _step_height(self, leg):
        """Return the independently configured front/rear height."""
        name = ('front_step_height' if self._is_front(leg) else 'back_step_height')
        return float(self.get_parameter(name).value)

    def _side_scale(self, leg):
        """Get the stride scale for the leg's side."""
        name = ('left_scale' if leg in (1, 2) else 'right_scale')
        return float(self.get_parameter(name).value)

    def _nominal_y(self, leg):
        """Return the nominal local-coordinate offset."""
        return (
            self.nominal_y_front
            if self._is_front(leg)
            else self.nominal_y_back
        )

    def _ik_for_leg(self, leg, x, y, z):
        """Select the original IK equation for the leg."""
        if leg in (0, 3):
            return self.IK_right(x, y, z)
        return self.IK_left(x, y, z)

    # ======================================================
    # 5. TRAJECTORY GENERATION
    # ======================================================

    @staticmethod
    def _smoothstep(u):
        """Smooth interpolation from 0 to 1 with zero end slopes."""
        return 3.0 * u**2 - 2.0 * u**3

    def _forward_trajectory(self, phase, scale, height):
        """Forward foot trajectory: return local y and z."""
        length = (float(self.get_parameter('step_length').value) * scale)
        base_z = self.nominal_z

        if phase < 0.5:
            # Swing: move from -length to +length and lift.
            u = phase / 0.5
            s = self._smoothstep(u)
            y = -length + 2.0 * length * s
            z = base_z - height * math.sin(math.pi * u)
        else:
            # Stance: return from +length to -length.
            u = (phase - 0.5) / 0.5
            s = self._smoothstep(u)
            y = length - 2.0 * length * s
            z = base_z
        return y, z

    def _backward_trajectory(self, phase, scale, height):
        """Backward trajectory, retaining the original equations."""
        length = (float(self.get_parameter('step_length').value) * scale)
        angle = 2.0 * math.pi * phase
        y = length * math.cos(angle)
        z = self.nominal_z - height * max(0.0, math.sin(angle))
        return y, z

    def _lateral_trajectory(self, phase, direction, height):
        """
        Sideways trajectory.

        direction = -1 for right, +1 for left.
        Lateral amplitude remains independent of stride scaling,
        matching the original controller.
        """
        length = float(self.get_parameter('lateral_step_length').value)
        angle = 2.0 * math.pi * phase

        dx = direction * length * math.cos(angle)
        z = self.nominal_z - height * max(0.0, math.sin(angle))

        return dx, z

    def _foot_target(self, leg, phase):
        """Generate the target coordinates for one leg."""
        # Preserve the original base target convention.
        x = 0.05 if leg in (0, 3) else -0.05
        y = 0.0
        z = self.nominal_z

        height = self._step_height(leg)
        scale = self._side_scale(leg)

        if self.mode == 'f':
            y, z = self._forward_trajectory(phase, scale, height)

        elif self.mode == 'b':
            y, z = self._backward_trajectory(phase, scale, height)

        elif self.mode in ('r', 'l'):
            direction = -1.0 if self.mode == 'r' else 1.0
            dx, z = self._lateral_trajectory(phase, direction, height)

            # Preserve the original opposite front/rear
            # lateral adjustments.
            if self._is_front(leg):
                x += dx
                y += 0.01
            else:
                x -= dx
                y -= 0.01

        elif self.mode == 's':
            # Hold the nominal target pose.
            y, z = 0.0, self.nominal_z

        # Apply the original front/rear local-coordinate offset.
        y += self._nominal_y(leg)

        return x, y, z

    # ======================================================
    # 6. INVERSE KINEMATICS
    # Original equations preserved from your code.
    # ======================================================

    @staticmethod
    def _clamp(value):
        return max(-1.0, min(1.0, value))

    def IK_left(self, x, y, z):
        L1, L2, L3 = self.L1, self.L2, self.L3

        C = math.sqrt(x**2 + z**2)
        D = math.sqrt(max(C**2 - L1**2, 0.0))
        G = math.sqrt(D**2 + y**2)

        J3 = math.acos(self._clamp((L2**2 + L3**2 - G**2) / (2 * L2 * L3))) - math.pi / 2
        J2 = math.atan2(-y, D) + math.acos(self._clamp((L2**2 + G**2 - L3**2) / (2 * L2 * G)))
        J1 = (math.pi / 2 - math.atan2(-x, z) - math.atan2(D, L1))
        return J1, J2, J3

    def IK_right(self, x, y, z):
        L1, L2, L3 = self.L1, self.L2, self.L3

        C = math.sqrt(x**2 + z**2)
        D = math.sqrt(max(C**2 - L1**2, 0.0))
        G = math.sqrt(D**2 + y**2)

        J3 = math.acos(self._clamp((L2**2 + L3**2 - G**2) / (2 * L2 * L3))) - math.pi / 2
        J2 = -(math.atan2(-y, D) + math.acos(self._clamp((L2**2 + G**2 - L3**2) / (2 * L2 * G))))
        J1 = (math.atan2(x, z) + math.atan2(D, L1)- math.pi / 2)
        return J1, J2, J3

    def _calculate_stand_pose(self):
        """Calculate the initial 12-joint standing configuration."""
        stand = []
        for leg in range(4):
            x = 0.05 if leg in (0, 3) else -0.05
            y = self._nominal_y(leg)
            J1, J2, J3 = self._ik_for_leg(leg, x, y, self.nominal_z)
            stand.extend([J1, J2, J3])

        return stand

    # ======================================================
    # 7. MAIN CONTROL LOOP
    # ======================================================

    def update(self):
        """Calculate and publish all 12 joint positions."""
        for leg in range(4):
            phase = (
                self.step / self.cycle_length
                + self.phase_offsets[leg]
            ) % 1.0

            # 1. Generate the desired foot position.
            x, y, z = self._foot_target(leg, phase)

            # 2. Convert foot position into joint angles.
            J1, J2, J3 = self._ik_for_leg(leg, x, y, z)

            # 3. Store the angles in the correct joint slots.
            base = leg * 3
            self.q[base:base + 3] = [J1, J2, J3]

        self.step += 1

        if self.sim:
            # Blend the standing pose into the gait at startup.
            blend = min(1.0, self.step / self.blend_steps)

            command = Float64MultiArray()
            command.data = [
                start + blend * (target - start)
                for start, target in zip(self.stand, self.q)
            ]
            self.pub.publish(command)

        else:
            # Publish joint states for visualization.
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.joint_names
            msg.position = self.q
            self.pub.publish(msg)

    # ======================================================
    # 8. KEYBOARD INPUT
    # ======================================================

    def input_loop(self):
        commands = {
            'f': 'forward',
            'b': 'backward',
            'r': 'right',
            'l': 'left',
            's': 'stop'
        }

        while True:
            try:
                command = input('Command (f/b/r/l/s): ').strip().lower()
            except EOFError:
                return

            if command in commands:
                self.mode = command
                self.get_logger().info(f'Movement mode: {commands[command]}')
            else:
                print('Invalid command. Use f/b/r/l/s.')


def main(args=None):
    rclpy.init(args=args)
    node = GazeboForwardGait()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()