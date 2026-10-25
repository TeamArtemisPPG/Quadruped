#!/usr/bin/env python3

import math
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class GazeboForwardGait(Node):

    def __init__(self):
        super().__init__('gazebo_forward_gait')

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

        self.q = [0.0]*12

        self.L1 = 0.05
        self.L2 = 0.135
        self.L3 = 0.1575

        self.step = 0
        self.cycle_length = 80

        self.mode = "f"

        self.timer = self.create_timer(0.05, self.update)

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
        step_height = 0.05
        step_length = 0.04
        base_z = 0.15

        if phase < 0.5:
            y = step_length * (phase / 0.5)
            z = base_z + step_height * (phase / 0.5)
        else:
            y = step_length * (1 - (phase - 0.5) / 0.5)
            z = base_z + step_height * (1 - (phase - 0.5) / 0.5)

        return y, z

    def backward(self, phase):
        step_height = 0.05
        step_length = 0.04
        base_z = 0.15

        if phase < 0.5:
            y = -step_length * (phase / 0.5)
            z = base_z + step_height * (phase / 0.5)
        else:
            y = -step_length * (1 - (phase - 0.5) / 0.5)
            z = base_z + step_height * (1 - (phase - 0.5) / 0.5)

        return y, z

    def right(self, phase):
        step_height = 0.05
        step_length = 0.04
        base_z = 0.15

        if phase < 0.5:
            x = step_length * (phase / 0.5)
            z = base_z + step_height * (phase / 0.5)
        else:
            x = step_length * (1 - (phase - 0.5) / 0.5)
            z = base_z + step_height * (1 - (phase - 0.5) / 0.5)

        return x, z

    def left(self, phase):
        step_height = 0.05
        step_length = 0.04
        base_z = 0.15

        if phase < 0.5:
            x = -step_length * (phase / 0.5)
            z = base_z + step_height * (phase / 0.5)
        else:
            x = -step_length * (1 - (phase - 0.5) / 0.5)
            z = base_z + step_height * (1 - (phase - 0.5) / 0.5)

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

        J3 = math.acos(clamp((L2**2 + L3**2 - G**2) / (2 * L2 * L3))) - math.pi/2

        J2 = math.atan2(-y, D) + math.acos(
            clamp((L2**2 + G**2 - L3**2) / (2 * L2 * G))
        )

        J1 = math.pi/2 - math.atan2(-x, z) - math.atan2(D, L1)

        return J1, J2, J3

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

    # -------- MAIN LOOP --------
    def update(self):

        # GLOBAL PHASE
        phase = (self.step / self.cycle_length) % 1.0
        leg_phase = phase * 4
        active_leg = int(leg_phase)
        local_phase = leg_phase - active_leg

        for i in range(4):

            # Default stance
            x = 0.05 if i in [0, 3] else -0.05
            y = 0.0
            z = 0.15

            if self.mode == 'f':

                if i == active_leg:
                    if local_phase < 0.6:
                        y, z = self.forward(local_phase / 0.6)
                    else:
                        y = 0.0
                        z = 0.15
                else:
                    y = 0.0
                    z = 0.15

            elif self.mode == 'b':

                if i == active_leg:
                    if local_phase < 0.6:
                        y, z = self.backward(local_phase / 0.6)
                    else:
                        y = 0.0
                        z = 0.15
                else:
                    y = 0.0
                    z = 0.15

            elif self.mode == 'r':

                if i == active_leg:
                    if local_phase < 0.6:
                        dx, z = self.right(local_phase / 0.6)
                        x += dx
                    else:
                        z = 0.15
                else:
                    z = 0.15

            elif self.mode == 'l':

                if i == active_leg:
                    if local_phase < 0.6:
                        dx, z = self.left(local_phase / 0.6)
                        x += dx
                    else:
                        z = 0.15
                else:
                    z = 0.15

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