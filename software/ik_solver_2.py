import math
from serial_comms_comp_1 import RobotArm


# =============================================================================
# IKController — inverse kinematics + angle-to-PWM conversion
# =============================================================================

class IKController:
    """
    Wraps a RobotArm instance and exposes move_to(x, y) which:
      1. Solves inverse kinematics for the target position
      2. Runs a forward-kinematics verify to catch math errors
      3. Converts angles to PWM ticks using per-servo calibration
      4. Sends all four servos to the Arduino in one move_multi call
    """

    # --- Link lengths (mm) ---
    L1 = 120   # shoulder shaft → elbow shaft
    L2 = 120   # elbow shaft    → wrist shaft
    L3 = 125   # wrist shaft    → fingertip center (closed gripper, horizontal)

    # --- Fixed Z height (mm) ---
    # Z = 0 means level with the shoulder shaft.
    # Shoulder is 100mm above the table, so Z = -100.
    Z = -90
    WRIST_MINIMUM_X = 60
    ELBOW_HORIZONTAL_OFFSET = 18 # horizontal in CALIBRATION is estimated based on this angle offset

    # --- Per-servo calibration ---
    # horizontal : tick where the joint/link is physically horizontal (IK 0°)
    # min/max    : physical travel limits
    # inverted   : True if increasing angle should decrease tick value
    # deg_per_tick: physical degrees per tick, measured from min to max
    CALIBRATION = {
        "base":     {"horizontal": 325, "min": 120, "max": 530, "inverted": False, "deg_per_tick": 178 / 410},
        "shoulder": {"horizontal": 209, "min": 120, "max": 530, "inverted": False, "deg_per_tick": 162 / 410},
        "elbow":    {"horizontal": 81, "min": 120, "max": 409, "inverted": True,  "deg_per_tick": 133 / 289},
        "claw":     {"horizontal": 310, "min": 120, "max": 500, "inverted": False, "deg_per_tick": 230 / 380},
    }

    def __init__(self, arm: RobotArm):
        self.arm = arm

    # -------------------------------------------------------------------------
    # Inverse kinematics
    # -------------------------------------------------------------------------

    def _ik(self, x: float, y: float, z: float, elbow_up: bool) -> dict:
        """
        Solve IK for fingertip position (x, y, z).
        Returns angles in degrees (0° = horizontal), raises ValueError if unreachable.
        """
        cfg = 1 if elbow_up else -1

        # Waist: rotate base to face target
        waist = math.degrees(math.atan2(y, x))

        # Subtract L3 — claw is always horizontal, so solve for wrist position
        r_xy_tip = math.hypot(x, y)
        if r_xy_tip < self.L3 + self.WRIST_MINIMUM_X:
            raise ValueError(
                f"Target too close for claw offset: "
                f"r_xy={r_xy_tip:.1f} mm < L3={self.L3 + self.WRIST_MINIMUM_X} mm"
            )

        r_xy_wrist = r_xy_tip - self.L3
        r = math.hypot(r_xy_wrist, z)

        reach_min = abs(self.L1 - self.L2 + self.WRIST_MINIMUM_X)

        max_angle = 180 - 18
        max_angle_rad = math.radians(max_angle)
        reach_max = math.sqrt(self.L1**2 + self.L2**2 - 2 * self.L1 * self.L2 * math.cos(max_angle_rad))

        if r < reach_min:
            raise ValueError(
                f"Target too close: r={r:.1f} mm, minimum={reach_min:.1f} mm"
            )
        if r > reach_max:
            raise ValueError(
                f"Target too far: r={r:.1f} mm, maximum={reach_max:.1f} mm"
            )

        # Elbow via law of cosines
        cos_elbow = (self.L1**2 + self.L2**2 - r**2) / (2 * self.L1 * self.L2)
        cos_elbow = max(-1.0, min(1.0, cos_elbow))
        elbow = cfg * math.degrees(math.acos(cos_elbow))

        # Shoulder
        phi   = math.atan2(z, r_xy_wrist)
        cos_a = (r**2 + self.L1**2 - self.L2**2) / (2 * r * self.L1)
        cos_a = max(-1.0, min(1.0, cos_a))
        alpha = math.acos(cos_a)
        shoulder = math.degrees(cfg*alpha - abs(phi))

        # Wrist pitch: cancel out shoulder + elbow tilt to stay level
        wrist_pitch = 180 - self.ELBOW_HORIZONTAL_OFFSET - elbow - shoulder

        return {
            "waist":    round(waist,       2),
            "shoulder": round(shoulder,    2),
            "elbow":    round(elbow,       2),
            "claw":     round(wrist_pitch, 2),
        }

    # -------------------------------------------------------------------------
    # Forward kinematics verify
    # -------------------------------------------------------------------------

    def _verify(self, angles: dict, x: float, y: float, z: float,
                tol: float = 1.0):
        """
        Reconstruct fingertip position from angles and assert it matches
        the target within tol mm. Raises AssertionError on failure.
        """
        w  = math.radians(angles["waist"])
        sh = math.radians(angles["shoulder"])
        el = math.radians(angles["elbow"])

        ex = self.L1 * math.cos(sh) * math.cos(w)
        ey = self.L1 * math.cos(sh) * math.sin(w)
        ez = self.L1 * math.sin(sh)

        arm_angle = -(sh + el - math.pi)
        wx = ex + self.L2 * math.cos(arm_angle) * math.cos(w)
        wy = ey + self.L2 * math.cos(arm_angle) * math.sin(w)
        wz = ez - self.L2 * math.sin(arm_angle)

        fx = wx + self.L3 * math.cos(w)
        fy = wy + self.L3 * math.sin(w)
        fz = wz

        err = math.sqrt((fx - x)**2 + (fy - y)**2 + (fz - z)**2)
        assert err < tol, (
            f"FK check failed: got ({fx:.1f}, {fy:.1f}, {fz:.1f}), "
            f"expected ({x:.1f}, {y:.1f}, {z:.1f}), error={err:.2f} mm"
        )

    # -------------------------------------------------------------------------
    # Angle → PWM conversion
    # -------------------------------------------------------------------------

    def _angle_to_tick(self, servo_name: str, angle_deg: float) -> int:
        """
        Convert an IK angle (degrees, where 0° = joint horizontal) to a
        PWM tick using the servo's horizontal tick as the reference point.
        Result is clamped to the servo's safe range.
        """
        cal = self.CALIBRATION[servo_name]
        ticks_per_deg = 1.0 / cal["deg_per_tick"]

        if cal["inverted"]:
            tick = cal["horizontal"] + (180-angle_deg) * ticks_per_deg
        else:
            tick = cal["horizontal"] + angle_deg * ticks_per_deg

        clamped = max(cal["min"], min(cal["max"], tick))
        if abs(clamped - tick) > 0.5:
            print(f"Warning: {servo_name} angle {angle_deg:.1f}° → tick "
                  f"{tick:.0f} clamped to {clamped:.0f} "
                  f"(range {cal['min']}–{cal['max']})")
            if servo_name == "claw":
                print("  Wrist pitch compensation is at its limit — "
                      "claw may not be perfectly level at this position.")

        return round(clamped)

    # -------------------------------------------------------------------------
    # Public interface
    # -------------------------------------------------------------------------

    def move_to(self, x: float, y: float,
                z: float = None,
                elbow_up: bool = True):
        """
        Move the arm fingertip to (x, y, z).

        x, y : target coordinates in mm from the shoulder origin
        z    : height in mm (defaults to self.Z if not given)
        elbow_up : preferred configuration — falls back to the other
                   automatically if the preferred one is unreachable
        """
        if z is None:
            z = self.Z

        # Try preferred configuration first, fall back to the other
        for up in [elbow_up, not elbow_up]:
            try:
                angles = self._ik(x, y, z, up)
                self._verify(angles, x, y, z)
                break
            except (ValueError, AssertionError):
                continue
        else:
            raise ValueError(
                f"Target ({x}, {y}, {z}) is unreachable in both configurations"
            )

        ticks = {
            self.arm.BASE:     self._angle_to_tick("base",     angles["waist"]),
            self.arm.SHOULDER: self._angle_to_tick("shoulder", angles["shoulder"]),
            self.arm.ELBOW:    self._angle_to_tick("elbow",    angles["elbow"]),
            self.arm.CLAW:     self._angle_to_tick("claw",     angles["claw"]),
        }

        config = "elbow_up" if up else "elbow_down"
        print(f"Moving to ({x}, {y}, {z}) mm [{config}] → angles {angles} → ticks {ticks}")
        self.arm.move_multi(ticks)


# =============================================================================
# Example usage
# =============================================================================

if __name__ == "__main__":
    PORT = "COM6"

    with RobotArm(PORT) as arm:
        controller = IKController(arm)

        arm.rest()
        arm.open_gripper()

        # Move to a position, close gripper, return to rest
        controller.move_to(200, 80)
        arm.close_gripper()
        arm.rest()
        arm.open_gripper()