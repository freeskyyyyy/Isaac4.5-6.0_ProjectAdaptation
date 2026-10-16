"""Conservative trial abort budgets; these do not certify physical braking."""
import math


def stopping_reserve(speed, feedback_age):
    # 0.10 m/s² assumed braking floor is below configured 0.15 per axis.
    # Feedback age + 0.40s covers query/control/cancel delay; 25mm uncertainty.
    return speed * (max(0., feedback_age) + .40) + speed**2 / .20 + .025


def tracking_error(point, points):
    best = math.inf
    for a, b in zip(points, points[1:]):
        dx, dy = b[0]-a[0], b[1]-a[1]
        t = max(0., min(1., ((point[0]-a[0])*dx + (point[1]-a[1])*dy)/max(dx*dx+dy*dy, 1e-24)))
        best = min(best, math.hypot(point[0]-a[0]-t*dx, point[1]-a[1]-t*dy))
    return best


def guard_reason(clearance, deviation, speed, feedback_age, outward_speed):
    if not all(math.isfinite(value) for value in (clearance, deviation, speed, feedback_age, outward_speed)):
        return 'Non-finite navigation observation; stop control'
    if clearance <= .55 + stopping_reserve(speed, feedback_age):
        return 'Predictive clearance stop: insufficient braking/latency reserve above 0.55m'
    tracking_budget = .005 + outward_speed*(max(0., feedback_age)+.40) + outward_speed**2/.20
    if deviation + tracking_budget >= .05:
        return 'Predictive tracking stop: insufficient reserve below 0.05m deviation'
    return None


def command_reason(values):
    """Independent world-trial output observation; does not clamp the controller."""
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        return 'Non-finite or malformed velocity command; stop control'
    if any(abs(value) > limit + 1e-6 for value, limit in zip(values, (.08, .08, .15))):
        return 'World velocity output exceeds configured per-axis limits; stop control'
    return None


def capture_tf_wait_allowed(capture_time, simulation_time, elapsed_wall):
    """Wait for the original capture stamp, bounded in both clock domains."""
    if not all(math.isfinite(v) for v in (capture_time, simulation_time, elapsed_wall)):
        return False
    return 0. <= simulation_time-capture_time <= .15 and 0. <= elapsed_wall < 1.5


def odometry_feedback_reason(source_stamp, receipt_sim, simulation_time, elapsed_wall):
    """Distinguish renderer stalls from stale closed-loop feedback.

    Source freshness retains the smoother's 150ms limit. Independent DDS
    callbacks may deliver odometry before /clock; bound that lead by one
    configured 50ms smoother step. The wall watchdog still bounds a pause.
    """
    if not all(math.isfinite(v) for v in (source_stamp, receipt_sim, simulation_time, elapsed_wall)):
        return 'Non-finite odometry timing; stop control'
    if source_stamp <= 0. or source_stamp-receipt_sim > .05:
        return 'Invalid or future odometry source stamp; stop control'
    if not capture_tf_wait_allowed(receipt_sim, simulation_time, elapsed_wall):
        return 'Odometry exceeded simulation freshness or wall watchdog; stop control'
    if simulation_time-source_stamp > .15:
        return 'Closed-loop odometry source became stale; stop control'
    return None


def tracking_observation(point, previous_point, dt, accepted_path, initial_path):
    """Measure against an accepted active path; compare both points to that path.

    A replan is allowed to change the route after the whole-path clearance gate.
    Keep initial-route deviation as evidence, without treating a route change as
    robot motion or continuing to enforce an obsolete route.
    """
    error = tracking_error(point, accepted_path)
    previous_error = tracking_error(previous_point, accepted_path)
    return {
        'tracking_error_m': error,
        'initial_path_error_m': tracking_error(point, initial_path),
        'outward_speed_m_s': max(0., (error-previous_error)/dt) if dt > 0 else 0.,
    }
