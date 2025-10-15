import math
from geometry_msgs.msg import Point
from src.datamodels.RobotPose import RobotPose
from tf_transformations import euler_from_quaternion

from src.skills.Skill import Skill
from src.skills.leaf_skills.Stand import Stand
from src.skills.main_skills.WalkToPose import WalkToPose
from src.skills.main_skills.WalkToPoint import WalkToPoint

class MapTeleop(Skill):
    """
    Description:
    Makes the robot walk to a published point on foxglove
    """

    def _initialise_sub_skills(self):
        self._sub_skills = {"Stand": Stand(self), "WalkToPose": WalkToPose(self), "WalkToPoint": WalkToPoint(self)}

    def _reset(self):
        self._current_sub_skill = "WalkToPose"

    def _transition(self):
        if self.ctx.blackboard._debug_behaviours_pose is None:
            self._current_sub_skill = "Stand"
            return

        self._current_sub_skill = "WalkToPose"

        pose = self.ctx.blackboard._debug_behaviours_pose.pose
        point = Point()
        point.x = pose.position.x * 1000
        point.y = pose.position.y * 1000

        orientation_q = pose.orientation
        _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

        self.final_pose = RobotPose(point, yaw % (2 * math.pi))

    def _tick(self):
        if self._current_sub_skill == "WalkToPoint":
            self._tick_sub_skill(self.final_pose, forward_only=True)
        elif self._current_sub_skill == "WalkToPose":
            self._tick_sub_skill(self.final_pose)
        else:
            self._tick_sub_skill()
