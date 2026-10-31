"""Shared Gen3 7-DoF sequence; no ROS or hardware dependencies."""
import math


def angular_delta(target, current):
    """Signed shortest angular distance in degrees, including 315 -> 0 = +45."""
    return (target - current + 180.0) % 360.0 - 180.0


def build_poses(first_tilt_deg=120.0):
    # 120 is below the manufacturer's SOFTWARE limit of 120.3 (guide r12, p109).
    # A smaller first ring can be selected explicitly; the other rings stay fixed.
    if not math.isfinite(first_tilt_deg) or not 90.0 < first_tilt_deg <= 120.0:
        raise ValueError("first_tilt_deg must be finite and in (90, 120]")
    poses = [[0.0] * 7]
    poses.append([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 270.0])
    for tilt in (first_tilt_deg, 90.0, 60.0, 30.0, 0.0):
        for azimuth in range(0, 360, 45):
            poses.append([0.0, 0.0, 0.0, 0.0, float(azimuth), tilt, 270.0])
    poses.append([0.0, 0.0, 0.0, 0.0, 360.0, 0.0, 0.0])
    return poses

