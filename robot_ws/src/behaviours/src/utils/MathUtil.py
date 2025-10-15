import math
from functools import reduce
from src.datamodels.RobotPose import RobotPose

EPSILON = 0.00001


def angleDiff(angle0, angle1):
    return min(math.fabs(angle0 - angle1), math.fabs(math.fabs(angle0 - angle1) - 2.0 * math.pi))


def angleSignedDiff(target, current):
    diff = target - current
    return reduce(  # noqa
        (lambda a, b: a if math.fabs(a) < math.fabs(b) else b), [diff, diff + math.pi * 2, diff - math.pi * 2]
    )


def distancePointToPoint(point_1, point_2):
    if point_1.point is None or point_2.point is None:
        raise ValueError("Point object cannot be None")

    return math.sqrt((point_1.point.x - point_2.point.x) ** 2 + (point_1.point.y - point_2.point.y) ** 2)


def normalisedTheta(theta):
    r = math.fmod(theta - math.pi, 2 * math.pi)
    if r > 0:
        return r - math.pi
    else:
        return r + math.pi


def clipToRange(var, min, max):
    if var < min:
        return min
    elif var > max:
        return max
    else:
        return var


# Check if two line segments intersect.
# All inputs are of type RobotPose (Vector2D), returns a boolean.
def doLinesIntersect(start1, end1, start2, end2):
    denominator = (end2.y - start2.y) * (end1.x - start1.x) - (end2.x - start2.x) * (end1.y - start1.y)

    # lines are parallel.
    if math.fabs(denominator) < EPSILON:
        return False

    numerator1 = (end2.x - start2.x) * (start1.y - start2.y) - (end2.y - start2.y) * (start1.x - start2.x)

    numerator2 = (end1.x - start1.x) * (start1.y - start2.y) - (end1.y - start1.y) * (start1.x - start2.x)

    u1 = numerator1 / denominator
    u2 = numerator2 / denominator

    # intesect point falls outside the segment.
    if u1 < -EPSILON or u1 > 1.0 + EPSILON or u2 < -EPSILON or u2 > 1.0 + EPSILON:  # noqa
        return False

    return True


# NOTE: This has been converted to use RobotPose instead of Vector2D (need to check)
# Find the intersetion point of two line segments.
# All inputs are of type RobotPose (Vector2D), returns None if no intersection, otherwise
# a RobotPose(Vector2D) of intersection point.
def intersectLines(start1, end1, start2, end2):
    denominator = (end2.y - start2.y) * (end1.x - start1.x) - (end2.x - start2.x) * (end1.y - start1.y)

    # lines are parallel.
    if math.fabs(denominator) < EPSILON:
        return None

    numerator1 = (end2.x - start2.x) * (start1.y - start2.y) - (end2.y - start2.y) * (start1.x - start2.x)

    numerator2 = (end1.x - start1.x) * (start1.y - start2.y) - (end1.y - start1.y) * (start1.x - start2.x)

    u1 = numerator1 / denominator
    u2 = numerator2 / denominator

    # intesect point falls outside the segment.
    if u1 < -EPSILON or u1 > 1.0 + EPSILON or u2 < -EPSILON or u2 > 1.0 + EPSILON:  # noqa
        return None

    point = Point()
    point.x = start1.x + u1 * (end1.x - start1.x)
    point.y = start1.y + u1 * (end1.y - start1.y)

    return RobotPose(point)


# returns a tuple of (Vector2D, Boolean), where the first is the closest point
# on the line to the given point, and the second is the distance to that point.
def pointSegmentDist(point, start, end):
    v = end.subtract(start)
    w = point.subtract(start)

    c1 = w.dotProduct(v)
    if c1 <= 0.0:
        return (start, w.length())

    c2 = v.dotProduct(v)
    if c2 <= c1:
        return (end, point.subtract(end).length())

    b = c1 / c2
    Pb = start.add(v.multiply(b))

    return (Pb, point.subtract(Pb).length())


def rrToAbs(robotPos, rr):
    x = robotPos.point.x + math.cos(robotPos.heading + rr.heading) * rr.distance
    y = robotPos.point.y + math.sin(robotPos.heading + rr.heading) * rr.distance
    return (x, y)


def absToRr(fromPos, toPos):
    dist = math.hypot(toPos[0] - fromPos[0], toPos[1] - fromPos[1])
    heading = normalisedTheta(math.atan2(toPos[1] - fromPos[1], toPos[0] - fromPos[0]) - fromPos[2])
    return (dist, heading)


def stdev(list):
    return math.sqrt(variance(list))


def variance(list):
    if len(list) != 0:
        m = mean(list)
        return sum(pow(x - m, 2) for x in list) / len(list)
    else:
        return 0


def mean(list):
    if len(list) != 0:
        return sum(list) / len(list)
    else:
        return 0


def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)

# Compute closest point on a segment to a point
# X: Point
# P: Segment starting point
# Q: Segment finishing point
# NOTE: P and Q are interchangeable
# https://diego.assencio.com/?index=ec3d5dfdfc0b6a0d147a656f0af332bd
def closest_point_on_segment(X, P, Q):
    # If P and Q are the same point, a segment cannot be formed,
    # and results in undefined behaviour. Just return one of the points
    if P.distanceTo(Q) < 1:
        return P

    lam_s_num = X.subtract(P).dotProduct(Q.subtract(P))
    lam_s_den = Q.subtract(P).dotProduct(Q.subtract(P))
    lam_s = lam_s_num / lam_s_den

    if lam_s <= 0:
        S = P
    elif lam_s >= 1:
        S = Q
    else:
        S = P.add(Q.subtract(P).scale(lam_s))

    return S
