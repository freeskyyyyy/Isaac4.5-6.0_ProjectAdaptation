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
    if clearance <= .55 + stopping_reserve(speed, feedback_age):
        return 'Predictive clearance stop: insufficient braking/latency reserve above 0.55m'
    tracking_budget = .005 + outward_speed*(max(0., feedback_age)+.40) + outward_speed**2/.20
    if deviation + tracking_budget >= .05:
        return 'Predictive tracking stop: insufficient reserve below 0.05m deviation'
    return None
