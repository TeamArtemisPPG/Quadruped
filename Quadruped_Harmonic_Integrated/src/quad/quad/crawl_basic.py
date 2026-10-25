            s = self.smoothstep(u)
            y = y0 + amplitude - 2.0 * amplitude * s
            z = z0

        return x, y, z

    # ---------- Supplied inverse-kinematics equations, preserved ----------
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

    def ik_for_leg(self, leg, x, y, z):
        return (
            self.IK_right(x, y, z)
            if self.is_right(leg)
            else self.IK_left(x, y, z)
        )

    # ---------- Main timer callback ----------
    def update(self):
        # Handle mode changes in the timer thread, not the keyboard thread.
        if self.requested_mode != self.mode:
            self.mode = self.requested_mode
            if self.mode == "f":
                self.step = 0
                self.walk_ticks = 0
                self.get_logger().info("Starting crawl with a gradual ramp.")
            else:
                # Capture the last command and return to stance over 0.5 s.
                self.transition_from = list(self.last_command_positions)
                self.transition_ticks = 0
                self.get_logger().info("Returning gradually to the launch stance.")

        command = Float64MultiArray()

        if self.mode == "s":
            # Smoothly return to the known launch stance instead of snapping.
            alpha = min(1.0, self.transition_ticks / self.transition_steps)
            command.data = [
                start + alpha * (target - start)
                for start, target in zip(self.transition_from, self.STAND)
            ]
            self.transition_ticks = min(
                self.transition_steps, self.transition_ticks + 1
            )
            self.last_command_positions = list(command.data)
            self.publisher.publish(command)
            return

        # First walking update has ramp=0, so it starts at the stance target.
        ramp = min(1.0, self.walk_ticks / self.ramp_steps)
        positions = []

        for leg in range(4):
            phase = (
                self.step / self.cycle_steps + self.PHASE_OFFSETS[leg]
            ) % 1.0
            x, y, z = self.foot_target(leg, phase, ramp)
            angles = self.ik_for_leg(leg, x, y, z)
            positions.extend(angles)

        # Blend joint commands from the exact launch stance to the IK targets.
        # This matters because the launch stance values are not exactly equal
        # to the IK angles calculated from the nominal target coordinates.
        command.data = [
            start + ramp * (target - start)
            for start, target in zip(self.STAND, positions)
        ]
        self.publisher.publish(command)
        self.last_command_positions = list(command.data)

        self.step += 1
        self.walk_ticks += 1

    # ---------- Keyboard input ----------
    def keyboard_input(self):
        while rclpy.ok():
            try:
                command = input("Command (f=forward crawl, s=stand): ").strip().lower()
            except EOFError:
                return

            if command in ("f", "s"):
                self.requested_mode = command
            else:
                print("For this first test, use only 'f' or 's'.")


def main(args=None):
    rclpy.init(args=args)
    node = QuadCrawlController()
    try: