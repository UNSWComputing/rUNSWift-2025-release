from src.skills.Skill import Skill
from src.skills.leaf_skills.Stand import Stand
from src.skills.main_skills.WalkToPose import WalkToPose
from src.datamodels.RobotPose import RobotPose
from src.skills.head_skills.HeadLookStraight import head_look_straight
from runswift_interfaces.msg import VisionPipelineSwitchAsk

from math import atan2, radians

class CheckRef(Skill):
    """
    Description:
    Once a free kick is called, look for ref and determine who the kicking team is
    """

    RUNSWIFT = 18
    NOT_RUNSWIFT = 0

    def _initialise_sub_skills(self):
        self._sub_skills = {
            "Stand": Stand(self),
            "WalkToPose": WalkToPose(self),
        }

    def _reset(self):
        self._current_sub_skill = "Stand"
        self._players = None
        self._final_pos = RobotPose()
        self._my_pose = None

    def _transition(self):
        if self._sub_skill_finished(skill="WalkToPose", final_pos=self._final_pos):
            self._current_sub_skill = "Stand"

    def _tick(self):
        self._my_pose = self.ctx.blackboard.robot_world
        
        # get vision to start ref detection mode 2
        referee_signal_msg = VisionPipelineSwitchAsk()
        referee_signal_msg.is_ref = True
        referee_signal_msg.check_mode = 2
        self.ctx.blackboard.referee_signal = referee_signal_msg
        
        self._current_sub_skill = "WalkToPose"

        head_look_straight(self.ctx.blackboard)

        self._players = self.ctx.blackboard.player_info.current_players
        field_players = self._players[1:]
        active_players = [p for p in field_players if p != 0]

        half = len(active_players) // 2
        # Split the active players so half the robots can look at each side of the pitch
        first_half = active_players[:half]

        if self.ctx.blackboard.player_info.player_number in first_half:
            ref_x, ref_y = 0, 3000
        else:
            ref_x, ref_y = 0, -3000
        heading_to_ref = atan2(ref_y - self._my_pose.point.y, ref_x - self._my_pose.point.x)

        self._final_pos.point = self._my_pose.point
        self._final_pos.heading = heading_to_ref

        # ref pointing left
        if self.ctx.blackboard.ref_result.referee_signal == 2:
            # facing towards the left 
            if self.ctx.blackboard.robot_world.heading < 0:
                self.ctx.blackboard.kicking_team = self.NOT_RUNSWIFT
            else:
                self.ctx.blackboard.kicking_team = self.RUNSWIFT
            referee_signal_msg = VisionPipelineSwitchAsk()
            referee_signal_msg.is_ref = False
            # doesnt matter
            referee_signal_msg.check_mode = 2
            self.ctx.blackboard.referee_signal = referee_signal_msg
        # ref pointing right
        elif self.ctx.blackboard.ref_result.referee_signal == 3:
            if self.ctx.blackboard.robot_world.heading > 0:
                self.ctx.blackboard.kicking_team = self.RUNSWIFT
            else:
                self.ctx.blackboard.kicking_team = self.NOT_RUNSWIFT
            referee_signal_msg = VisionPipelineSwitchAsk()
            referee_signal_msg.is_ref = False
            # doesnt matter
            referee_signal_msg.check_mode = 2
            self.ctx.blackboard.referee_signal = referee_signal_msg
        self.ctx.get_logger().debug(f"KICKING TEAM: {self.ctx.blackboard.kicking_team}")
        self._tick_sub_skill(final_pos=self._final_pos)
