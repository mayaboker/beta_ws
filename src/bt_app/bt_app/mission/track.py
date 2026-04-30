from dataclasses import dataclass
import math


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def cosd(deg):
    return math.cos(math.radians(deg))


def apply_deadband(x, deadband):
    """
    Removes small camera noise around zero.
    """
    if abs(x) < deadband:
        return 0.0

    if x > 0:
        return x - deadband
    else:
        return x + deadband


@dataclass
class ControllerConfig:
    # -------------------------
    # Drone / throttle settings
    # -------------------------
    hover_throttle: float = 0.45      # throttle needed to hover, 0.0 to 1.0
    min_throttle: float = 0.20
    max_throttle: float = 0.85

    # -------------------------
    # Forward speed command
    # -------------------------
    # Negative pitch = nose down = fly forward.
    # Increase magnitude for faster tracking.
    forward_pitch_deg: float = -30.0

    max_pitch_deg: float = 100.0
    max_roll_deg: float = 10.0

    # -------------------------
    # Camera-error controller
    # -------------------------
    deadband_deg: float = 0.5

    # X error -> yaw
    kp_yaw: float = 3.0               # deg/s yaw per deg image error
    kd_yaw: float = 0.0
    max_yaw_rate_dps: float = 90.0

    # Y error -> small pitch correction
    kp_pitch_y: float = 100          # deg pitch per deg image error
    kd_pitch_y: float = 0.0
    max_visual_pitch_deg: float = 100.0

    # Y error -> throttle correction
    kp_throttle_y: float = 0.006      # throttle per deg image error
    kd_throttle_y: float = 0.0
    max_throttle_y_correction: float = 0.10

    # -------------------------
    # Sign corrections
    # Change these if the drone moves the wrong way.
    # -------------------------
    yaw_sign: float = 1.0
    pitch_y_sign: float = 1.0
    throttle_y_sign: float = 1.0

    # -------------------------
    # RC output conversion
    # -------------------------
    betaflight_angle_limit_deg: float = 60.0
    betaflight_yaw_rate_full_stick_dps: float = 250.0

    rc_roll_sign: float = 1.0
    rc_pitch_sign: float = -1.0
    rc_yaw_sign: float = 1.0


@dataclass
class ControlOutput:
    roll_deg: float
    pitch_deg: float
    yaw_rate_dps: float
    throttle: float

    rc_roll: int
    rc_pitch: int
    rc_yaw: int
    rc_throttle: int


class VisualTargetController:
    def __init__(self, cfg: ControllerConfig):
        self.cfg = cfg
        self.prev_ex = None
        self.prev_ey = None

    def reset(self):
        self.prev_ex = None
        self.prev_ey = None

    def update(self, error_x_deg, error_y_deg, dt, target_visible=True):
        """
        Inputs:
            error_x_deg:
                Camera horizontal angle error.
                Positive = target is right of image center.

            error_y_deg:
                Camera vertical angle error.
                Positive = target is above image center.

            dt:
                Time step in seconds.

            target_visible:
                If False, controller returns neutral roll/pitch/yaw and hover throttle.

        Returns:
            ControlOutput with roll, pitch, yaw rate, throttle and RC-style commands.
        """

        cfg = self.cfg

        if not target_visible:
            self.reset()
            return self._make_output(
                roll_deg=0.0,
                pitch_deg=0.0,
                yaw_rate_dps=0.0,
                throttle=cfg.hover_throttle,
            )

        # Apply deadband to reduce jitter near image center
        ex = apply_deadband(error_x_deg, cfg.deadband_deg)
        ey = apply_deadband(error_y_deg, cfg.deadband_deg)

        # Derivatives, optional
        if self.prev_ex is None or self.prev_ey is None or dt <= 0.0:
            ex_dot = 0.0
            ey_dot = 0.0
        else:
            ex_dot = (ex - self.prev_ex) / dt
            ey_dot = (ey - self.prev_ey) / dt

        self.prev_ex = ex
        self.prev_ey = ey

        # --------------------------------------------------
        # 1. Horizontal image error controls yaw
        # --------------------------------------------------
        yaw_rate_dps = cfg.yaw_sign * (
            cfg.kp_yaw * ex + cfg.kd_yaw * ex_dot
        )

        yaw_rate_dps = clamp(
            yaw_rate_dps,
            -cfg.max_yaw_rate_dps,
            cfg.max_yaw_rate_dps,
        )

        # --------------------------------------------------
        # 2. Vertical image error gives small pitch correction
        # --------------------------------------------------
        pitch_visual_deg = cfg.pitch_y_sign * (
            cfg.kp_pitch_y * ey + cfg.kd_pitch_y * ey_dot
        )

        pitch_visual_deg = clamp(
            pitch_visual_deg,
            -cfg.max_visual_pitch_deg,
            cfg.max_visual_pitch_deg,
        )

        # Main pitch command:
        # negative pitch = nose down = move forward
        pitch_deg = cfg.forward_pitch_deg + pitch_visual_deg

        pitch_deg = clamp(
            pitch_deg,
            -cfg.max_pitch_deg,
            cfg.max_pitch_deg,
        )

        # For this forward-camera controller, roll is not used.
        # Horizontal centering is done with yaw.
        roll_deg = 0.0

        # --------------------------------------------------
        # 3. Feed-forward throttle compensation for pitch/roll
        # --------------------------------------------------
        # When the drone tilts, only part of the thrust points upward.
        # Approximate compensation:
        #
        # throttle_ff = hover_throttle / (cos(roll) * cos(pitch))
        #
        # This is the important part that prevents altitude loss
        # when pitching forward.
        denom = cosd(roll_deg) * cosd(pitch_deg)
        denom = max(denom, 0.35)  # safety against extreme tilt

        throttle_ff = cfg.hover_throttle / denom

        # --------------------------------------------------
        # 4. Extra throttle from vertical image error
        # --------------------------------------------------
        # Positive ey = target above center.
        # Usually this means add throttle / climb.
        # If the target moves the wrong way, change throttle_y_sign to -1.
        throttle_y = cfg.throttle_y_sign * (
            cfg.kp_throttle_y * ey + cfg.kd_throttle_y * ey_dot
        )

        throttle_y = clamp(
            throttle_y,
            -cfg.max_throttle_y_correction,
            cfg.max_throttle_y_correction,
        )

        throttle = throttle_ff + throttle_y

        throttle = clamp(
            throttle,
            cfg.min_throttle,
            cfg.max_throttle,
        )

        return self._make_output(
            roll_deg=roll_deg,
            pitch_deg=pitch_deg,
            yaw_rate_dps=yaw_rate_dps,
            throttle=throttle,
        )

    def _make_output(self, roll_deg, pitch_deg, yaw_rate_dps, throttle):
        """
        Converts physical commands into Betaflight-like RC values.

        RC outputs:
            roll/pitch/yaw: 1000 to 2000, center 1500
            throttle:       1000 to 2000
        """

        cfg = self.cfg

        roll_norm = clamp(
            roll_deg / cfg.betaflight_angle_limit_deg,
            -1.0,
            1.0,
        )

        pitch_norm = clamp(
            pitch_deg / cfg.betaflight_angle_limit_deg,
            -1.0,
            1.0,
        )

        yaw_norm = clamp(
            yaw_rate_dps / cfg.betaflight_yaw_rate_full_stick_dps,
            -1.0,
            1.0,
        )

        rc_roll = int(round(1500 + cfg.rc_roll_sign * 500 * roll_norm))
        rc_pitch = int(round(1500 + cfg.rc_pitch_sign * 500 * pitch_norm))
        rc_yaw = int(round(1500 + cfg.rc_yaw_sign * 500 * yaw_norm))
        rc_throttle = int(round(1000 + 1000 * clamp(throttle, 0.0, 1.0)))

        return ControlOutput(
            roll_deg=roll_deg,
            pitch_deg=pitch_deg,
            yaw_rate_dps=yaw_rate_dps,
            throttle=throttle,
            rc_roll=rc_roll,
            rc_pitch=rc_pitch,
            rc_yaw=rc_yaw,
            rc_throttle=rc_throttle,
        )


# --------------------------------------------------
# Example usage
# --------------------------------------------------

