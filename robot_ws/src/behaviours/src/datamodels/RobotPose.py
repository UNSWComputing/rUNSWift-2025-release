import math
from geometry_msgs.msg import Point


class RobotPose:
    """
    Represents the pose of a robot, including position and heading.

    Attributes:
        point (Point): Position of the robot relative to the world.
        heading (float): Angle between the robot's body and its head.
    """

    def __init__(self, point=Point(), heading=0):
        """
        Initialises a new RobotPose instance.

        Args:
            point (Point): Position of the robot.
            heading (float): Heading angle of the robot.
        """
        self.point = point  # Position relative to world where centre is (0,0,0)
        self.heading = heading  # Heading is the angle relative to world

    def clone(self):
        """
        Creates a copy of the current RobotPose.

        Returns:
            RobotPose: A clone of the current pose.
        """
        new_point = Point()
        new_point.x = self.point.x
        new_point.y = self.point.y
        new_point.z = self.point.z  # Copy all attributes
        return RobotPose(new_point, self.heading)

    def length(self):
        """
        Computes the Euclidean distance from the origin.

        Returns:
            float: Distance from the origin.
        """
        return math.sqrt(self.point.x**2 + self.point.y**2)

    def length2(self):
        """
        Computes the squared Euclidean distance from the origin.

        Returns:
            float: Squared distance from the origin.
        """
        return self.point.x**2 + self.point.y**2

    def normalise(self, length=1):
        """
        Normalises the vector to a specified length.

        Args:
            length (float): Desired length of the vector. Defaults to 1.

        Returns:
            RobotPose: The normalised pose.
        """
        if self.length2() == 0:
            return self
        self.scale(length / self.length())
        return self

    def normalise_copy(self, length=1):
        """
        Creates a normalised copy of the pose.

        Args:
            length (float): Desired length of the vector. Defaults to 1.

        Returns:
            RobotPose: A normalised copy of the pose.
        """
        return self.clone().normalise(length)

    def dotProduct(self, var):
        """
        Computes the dot product with another point.

        Args:
            var (Point): The other point.

        Returns:
            float: The dot product.
        """
        return self.point.x * var.x + self.point.y * var.y

    def absThetaTo(self, var):
        """
        Computes the absolute angle difference to another point.

        Args:
            var (Point): The target point.

        Returns:
            float: Absolute angle difference in radians.
        """
        norm = self.normalise_copy()
        norm_var = var.normalise()
        dp = norm.dotProduct(norm_var)
        if dp >= 1.0:
            return 0.0
        if dp <= -1.0:
            return math.pi
        return math.acos(dp)

    def rotate(self, theta):
        """
        Rotates the pose by a specified angle.
        https://middletonlee1994.wordpress.com/2015/05/25/rotation-matrix/

        Args:
            theta (float): Angle to rotate by in radians.

        Returns:
            RobotPose: The rotated pose.
        """
        sinTheta = math.sin(theta)
        cosTheta = math.cos(theta)

        newX = (cosTheta * self.point.x) - (sinTheta * self.point.y)
        newY = (sinTheta * self.point.x) + (cosTheta * self.point.y)

        self.point.x = newX
        self.point.y = newY
        return self

    def rotate_copy(self, theta):
        """
        Creates a rotated copy of the pose.

        Args:
            theta (float): Angle to rotate by in radians.

        Returns:
            RobotPose: A rotated copy of the pose.
        """
        result = RobotPose(self.point, self.heading)
        result.rotate(theta)
        return result

    def angleTo(self, var):
        """
        Computes the angle to another point.

        Args:
            var (Point): The target point.

        Returns:
            float: Angle in radians to the target point.
        """
        dx = self.point.x - var.x
        dy = self.point.y - var.y
        return math.atan2(dy, dx) % (2 * math.pi)

    def subtract(self, var):
        """
        Subtracts another point from the pose.

        Args:
            var (Point): The point to subtract.

        Returns:
            RobotPose: Resulting pose after subtraction. Heading stays the same.
        """
        point = Point()
        point.x = self.point.x - var.x
        point.y = self.point.y - var.y
        return RobotPose(point, self.heading)

    def subtract_in_place(self, var):
        """
        Subtracts another point in-place.

        Args:
            var (Point): The point to subtract.

        Returns:
            RobotPose: The updated pose.
        """
        self.point.x -= var.x
        self.point.y -= var.y
        return self

    def add(self, var):
        """
        Adds another point to the pose.

        Args:
            var (Point): The point to add.

        Returns:
            RobotPose: Resulting pose after addition.
        """
        point = Point()
        point.x = self.point.x + var.x
        point.y = self.point.y + var.y
        return RobotPose(point, self.heading)

    def add_in_place(self, var):
        """
        Adds another point in-place.

        Args:
            var (Point): The point to add.

        Returns:
            RobotPose: The updated pose.
        """
        self.point.x += var.x
        self.point.y += var.y
        return self

    def scale(self, s):
        """
        Scales the pose by a scalar.

        Args:
            s (float): Scaling factor.

        Returns:
            RobotPose: The scaled pose.
        """
        self.point.x *= s
        self.point.y *= s
        return self

    def scale_copy(self, s):
        """
        Creates a scaled copy of the pose.

        Args:
            s (float): Scaling factor.

        Returns:
            RobotPose: A scaled copy of the pose.
        """
        return self.clone().scale(s)

    def fitLimits(self, rect, grow=True):
        """
        Scales the vector to fit within a rectangle.

        Args:
            rect (Point): Maximum dimensions of the rectangle.
            grow (bool): Whether to allow growing the vector. Defaults to True.

        Returns:
            RobotPose: The adjusted pose.
        """
        scale_x = abs(rect.x / float(self.point.x)) if self.point.x else 1e10
        scale_y = abs(rect.y / float(self.point.y)) if self.point.y else 1e10

        scale = min(scale_x, scale_y)
        if not grow:
            scale = min(1.0, scale)
        self.scale(scale)

        return self

    def fitEllipse(self, ellipse, grow=True):
        """
        Scales the vector to fit within an ellipse.

        Args:
            ellipse (Point): Radii of the ellipse (x for major, y for minor).
            grow (bool): Whether to allow growing the vector. Defaults to True.

        Returns:
            RobotPose: The adjusted pose.
        """
        ratio = ellipse.y / float(ellipse.x)
        elliptical_length = math.sqrt(self.point.y**2 + self.point.x**2 * ratio**2)

        scale = ellipse.y / elliptical_length
        if not grow:
            scale = min(1.0, scale)

        self.scale(scale)
        return self

    def normal_vector(self):
        """
        Computes a normal vector to the current RobotPose.

        Returns:
            RobotPose: A new RobotPose with the normal vector (perpendicular direction)
            by rotating counterclockwise by 90 degrees.
        """
        new_point = Point()
        new_point.x = -self.point.y
        new_point.y = self.point.x
        return RobotPose(new_point, self.heading)

    def signed_angle_between(self, other):
        """
        Computes the signed angle between this vector and another RobotPose.

        Args:
            other (RobotPose): The second vector.

        Returns:
            float: Signed angle in radians (-π to π), positive for counterclockwise.
        """
        # Compute angles using atan2
        angle1 = math.atan2(self.point.y, self.point.x)
        angle2 = math.atan2(other.point.y, other.point.x)

        # Compute signed orientation difference
        angle = angle1 - angle2

        # Normalize to range [-π, π]
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi

        return angle


def makeRobotPoseFromDistHeading(distance, heading):
    """
    Creates a RobotPose from a distance and heading angle.

    Args:
        distance (float): Distance from the origin.
        heading (float): Heading angle in radians.

    Returns:
        RobotPose: The created pose.
    """
    point = Point()
    point.x = distance * math.cos(heading)
    point.y = distance * math.sin(heading)
    return RobotPose(point, heading)


def SeBallToPoint(SeBallPoint):
    """
    Translate a point in the SeBall format to Point
    """
    new_point = Point()
    new_point.x = SeBallPoint.pos_x
    new_point.y = SeBallPoint.pos_y
    new_point.z = 0.0
    return new_point


def SeBallToRobotPose(SeBallPoint):
    """
    Translate a point in the SeBall format to RobotPose
    """
    new_point = SeBallToPoint(SeBallPoint)
    return RobotPose(new_point)


def VBallToPoint(VBallPoint):
    new_point = Point()
    new_point.x = VBallPoint.x
    new_point.y = VBallPoint.y
    new_point.z = 0.0
    return new_point


def VBallToRobotPose(VBallPoint):
    """
    Translate a point in the SeBall format to RobotPose
    """
    new_point = VBallToPoint(VBallPoint)
    return RobotPose(new_point)
