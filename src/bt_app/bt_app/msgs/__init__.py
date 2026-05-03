from dataclasses import dataclass


@dataclass(frozen=True)
class LinearVelocity:
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class TrackerResult:
    error_x: float
    error_y: float


__all__ = ["LinearVelocity", "TrackerResult"]
