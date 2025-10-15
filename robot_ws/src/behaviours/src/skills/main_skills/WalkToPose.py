from src.skills.Skill import Skill
from src.skills.leaf_skills.Walk import Walk
from src.skills.leaf_skills.Stand import Stand
from src.skills.main_skills.WalkToPoint import WalkToPoint
from src.datamodels.RobotPose import RobotPose
from math import radians
from geometry_msgs.msg import Point
from src.utils.MathUtil import normalisedTheta
from visualization_msgs.msg import Marker


class WalkToPose(Skill):
    """
    Description:
    Walk to a desired point and turn on the spot until match desired heading.
    """

    # Distance that we consider to be close enough to the point
    CLOSE_DISTANCE = 50  # mm

    # Turn speed once at point
    TURN_RATE = 1.5  # rad / s

    # How much we allow the heading to differ from desired heading
    ANGLE_DIFF_ERROR = radians(40)
    ANGLE_SLOW_ERROR = radians(55)

    # Forward only parameters
    FORWARD_ONLY_DISTANCE = 1000  # mm
    FORWARD_ONLY_HEADING = radians(40)

    def _initialise_sub_skills(self):
        self._sub_skills = {"Stand": Stand(self), "Walk": Walk(self), "WalkToPoint": WalkToPoint(self)}

    def _reset(self):
        self._current_sub_skill = "WalkToPoint"
        self._heading_close = False
        self._heading = None
        self._headingError = None
        self._debug_markers = self.ctx.create_publisher(Marker, "/walktopose_marker", 2)

    def _check_finished(self, final_pos=None, *args, **kwargs):
        if final_pos is None:
            return False
        if not self._sub_skill_finished(skill="WalkToPoint", final_pos=final_pos):
            return False
        self._update_heading_close(final_pos)
        if not self._heading_close:
            return False
        return True

    def _tick(self, final_pos=RobotPose(), heading_tolerance=10, forward_only=True):
        self.ANGLE_DIFF_ERROR = radians(heading_tolerance)
        # print(f"        {self._current_sub_skill}: {self._heading_close}")
        self._update_heading_close(final_pos)
        marker = Marker()
        marker.type = Marker.CUBE
        marker.header.frame_id = "world"
        marker.header.stamp = self.ctx.get_clock().now().to_msg()
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.id = 1
        marker.pose.position.x = final_pos.point.x / 1000
        marker.pose.position.y = final_pos.point.y / 1000
        marker.pose.position.z = 0.05
        marker.color.r = 1.0
        marker.color.g = 0.062745098
        marker.color.b = 0.9411764706
        marker.color.a = 1.0
        self._debug_markers.publish(marker)

        self._pos_difference = self.ctx.blackboard.robot_world.subtract(final_pos.point)
        if self._pos_difference.length() < self.FORWARD_ONLY_DISTANCE and self._headingError < self.FORWARD_ONLY_HEADING:
            forward_only = False

        if not self._sub_skill_finished(skill="WalkToPoint", final_pos=final_pos):
            self._current_sub_skill = "WalkToPoint"
        elif not self._heading_close:
            self._current_sub_skill = "Walk"
        else:
            self._current_sub_skill = "Stand"

        if self._current_sub_skill == "WalkToPoint":
            self._tick_sub_skill(final_pos=final_pos, forward_only=forward_only)
        elif self._current_sub_skill == "Walk":
            turn_factor = 1.0
            if abs(self._headingError) < self.ANGLE_SLOW_ERROR:
                turn_factor = 0.4
            turn = self.TURN_RATE * turn_factor if self._headingError > 0 else -self.TURN_RATE * turn_factor
            self._tick_sub_skill(turn=turn)
        else:
            self._tick_sub_skill()

    def _update_heading_close(self, final_pos):
        self._heading = self.ctx.blackboard.robot_world.heading
        self._headingError = normalisedTheta(final_pos.heading - self._heading)
        if abs(self._headingError) < self.ANGLE_DIFF_ERROR:
            self._heading_close = True
        else:
            self._heading_close = False
