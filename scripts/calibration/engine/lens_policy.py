"""Lens-shape reference for constrained Brown calibration, without lens presets.

Two residuals penalize curvature of radial gain over the output field; two
more steer away from a singular mapping. None fix k3. The hard test certifies
a positive-definite mapping Jacobian on the disk containing the output image,
including a conservative bound for tangential terms.
"""
import math

CURVATURE_SIGMA = 0.05
JACOBIAN_MARGIN = 1e-4


def field_radius2(state, width, height):
    fx, fy = math.exp(state[0]), math.exp(state[1])
    cx, cy = state[2]*width, state[3]*height
    x = max(abs(cx), abs(width-1-cx))/fx
    y = max(abs(cy), abs(height-1-cy))/fy
    return x*x+y*y


def curvature_residuals(state, width, height, points):
    t = field_radius2(state, width, height)
    b, c = state[5]*t*t, state[8]*t*t*t
    weight = CURVATURE_SIGMA*math.sqrt(points)
    # Integral_0^1 (d²/dq² [1+Aq+Bq²+Cq³])² dq
    # = (2B+3C)² + 3C². No camera/distortion target is supplied.
    return [weight*(2*b+3*c), weight*math.sqrt(3)*c]


def cubic_minimum(c, end):
    def value(t): return ((c[3]*t+c[2])*t+c[1])*t+c[0]
    candidates = [0., end]
    a, b, d = 3*c[3], 2*c[2], c[1]
    if a == 0:
        if b != 0: candidates.append(-d/b)
    else:
        discriminant = b*b-4*a*d
        if discriminant >= 0:
            q = -.5*(b+math.copysign(math.sqrt(discriminant), b))
            if q != 0: candidates.extend((q/a, d/q))
            else: candidates.append(-b/(2*a))
    return min(value(t) for t in candidates if 0 <= t <= end)


def shape_margins(state, width, height):
    t = field_radius2(state, width, height)
    a, b, c = state[4], state[5], state[8]
    tangential = 6*math.sqrt(t*(state[6]**2+state[7]**2))
    return [cubic_minimum((1,a,b,c), t)-tangential,
            cubic_minimum((1,3*a,5*b,7*c), t)-tangential]


def regularization_residuals(state, width, height, points):
    result = curvature_residuals(state, width, height, points)
    for margin in shape_margins(state, width, height):
        # The barrier gradient steers away from folding BEFORE a hard rejection.
        # Extension below epsilon keeps central-difference probes finite.
        e = 1e-5
        q = .1/max(margin, e)-1
        if margin < e: q += (e-margin)*.1/(e*e)
        result.append(.01*math.sqrt(points)*max(0.,q))
    return result


def feasible(state, width, height):
    try:
        if not all(math.isfinite(x) for x in state): return False
        minimum = min(shape_margins(state, width, height))
        return math.isfinite(minimum) and minimum > JACOBIAN_MARGIN
    except (OverflowError, ValueError, ZeroDivisionError):
        return False
