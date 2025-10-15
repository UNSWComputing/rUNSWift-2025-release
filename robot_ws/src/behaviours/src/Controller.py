import time
import math
import copy
import subprocess
from src.utils import Constants
from runswift_interfaces.msg import (
    VisionBallFeature,
    CommsRCGCD,
    BehavioursRobotInfo,
    ButtonEvent,
    VisionPipelineSwitchAsk,
)
from src.skills.Skill import Skill
from src.skills.leaf_skills.Walk import Walk
from src.skills.leaf_skills.Sit import Sit
from src.skills.leaf_skills.Stand import Stand
from src.datamodels.RobotPose import RobotPose
from geometry_msgs.msg import Point
from src.skills.main_skills.WalkToPoint import WalkToPoint
from src.skills.main_skills.WalkToPose import WalkToPose
from src.skills.main_skills.WalkInCircle import WalkInCircle
from src.skills.main_skills.Ready import Ready
from src.skills.main_skills.Demo import Demo
from src.skills.main_skills.ApproachBall import ApproachBall
from src.skills.leaf_skills.Kick import Kick
from src.skills.main_skills.BallAdjustment import BallAdjustment
from src.skills.main_skills.Playing import Playing
from src.skills.main_skills.Kickoff import Kickoff
from src.skills.debug_skills.MapTeleop import MapTeleop
from src.utils.FieldGeometry import ENEMY_GOAL_BEHIND_CENTER
from src.utils.Timer import Timer
from src.utils.Ball import getAbsoluteBall
from math import radians
from src.skills.head_skills.HeadScanBall import head_scan_ball
from src.skills.head_skills.HeadLookStraight import head_look_straight
from src.skills.head_skills.HeadTrackBallPixel import head_track_ball_pixel
from src.skills.roles.Defender import Defender
from src.skills.head_skills.SeHeadTrackBall import se_head_track_ball
from std_msgs.msg import String
from rclpy.clock import Clock
from rclpy.time import Time
import subprocess

from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import PoseWithCovarianceStamped


class Controller(Skill):
    """
    This is the main entry point to rUNSWift's behaviour tree.
    """

    # How long we determine kickoff phase to be
    KICKOFF_DURATINO = 20
    MOTION_PUB_RATE = 30  # Hz

    def _reset(self):
        # TODO: Get transition strategy from config file
        strategy_id = 0
        self._default_strategy_id = 1
        self._transition_strategy = self._load_transition_strategy(strategy_id)
        self._current_sub_skill = "Walk"
        self.kickoff = True
        self.timer_started = False
        self._timer = Timer()
        self._started = False
        self._prev_motion_command = None
        self._motion_last_pub_time = time.time()
        self._played_penalised_sound = False
        self._playing_anthem = False

    def _initialise_sub_skills(self):
        self._sub_skills = {
            "Walk": Walk(self),
            "Stand": Stand(self),
            "Sit": Sit(self),
            "WalkToPoint": WalkToPoint(self),
            "WalkToPose": WalkToPose(self),
            "WalkInCircle": WalkInCircle(self),
            "Ready": Ready(self),
            "Kick": Kick(self),
            "BallAdjustment": BallAdjustment(self),
            "ApproachBall": ApproachBall(self),
            "Playing": Playing(self),
            "Kickoff": Kickoff(self),
            "Demo": Demo(self),
            "MapTeleop": MapTeleop(self),
            "Defender": Defender(self),
        }

    def _transition(self):
        if not self._started:
            self._timer.restart()
            self._started = True

        self._transition_strategy()

    def _tick(self):
        msg = String()
        msg.data = str(self.ctx.behaviour_hierarchy)
        self.ctx.current_behaviour.publish(msg)
        if self.ctx.blackboard.penalised:
            self._current_sub_skill = "Stand"
        if self._current_sub_skill == "Walk":
            self._tick_sub_skill(forward=200)
        else:
            self._tick_sub_skill()

        if time.time() - self._motion_last_pub_time > 1 / self.MOTION_PUB_RATE:
            self.ctx.motion_command.publish(self.ctx.blackboard.motion_command)
            self._motion_last_pub_time = time.time()

    def _load_transition_strategy(self, strategy_id):
        strategies = {
            0: self._legacy_transition,
            1: self._static_transition,
            2: self._dynamic_transition,
        }
        return strategies.get(strategy_id, strategies[self._default_strategy_id])

    def _legacy_transition(self):
        # self.ctx.get_logger().error(f"Behaviour: {self.ctx.behaviour_hierarchy}")
        button_presses = self.ctx.blackboard.button_event.press_count

        # if not on GC
        if self.ctx.blackboard.gameinfo is None or self.ctx.blackboard.player_info is None:
            self._current_sub_skill = "Demo"
            # se_head_track_ball(self.ctx.blackboard)
            head_track_ball_pixel(self.ctx.blackboard)
            if button_presses == 1:
                self.ctx.blackboard.penalised = not self.ctx.blackboard.penalised
                self.ctx.blackboard.button_event = ButtonEvent()
            return

        gamestate = self.ctx.blackboard.gameinfo.state
        player_number = self.ctx.blackboard.player_info.player_number
        team_number = self.ctx.blackboard.player_info.team_number

        if self.ctx.blackboard.penalised or gamestate in [CommsRCGCD.STATE_READY, CommsRCGCD.STATE_SET]:
            head_look_straight(self.ctx.blackboard)
        if self.ctx.blackboard.penalised or gamestate in [CommsRCGCD.STATE_STANDBY]:
            self.ctx.get_logger().error("In STANDBY")
            head_look_straight(self.ctx.blackboard, pitch=-0.3)
            if not self._playing_anthem:
                self._playing_anthem = True
                subprocess.Popen(["aplay", "/home/nao/bin/anthem.wav"])
                self.ctx.get_logger().error("\n\nPLAYING ANTHEM\n\n")

        # elif (
        #     self.ctx.blackboard.vision_ball is None
        #     or len(self.ctx.blackboard.vision_ball.ball_features) < 1
        #     or time.time() - self.ctx.blackboard.vision_ball.header.stamp.sec > 3
        # ):
        #     head_scan_ball(self.ctx.blackboard)
        else:
            head_track_ball_pixel(self.ctx.blackboard)

        if self.ctx.blackboard.gameinfo.teams[0].team_number == team_number:
            self.ctx.blackboard.penalised = self.ctx.blackboard.gameinfo.teams[0].players[player_number - 1].penalty
        elif self.ctx.blackboard.gameinfo.teams[1].team_number == team_number:
            self.ctx.blackboard.penalised = self.ctx.blackboard.gameinfo.teams[1].players[player_number - 1].penalty
        else:
            self.ctx.get_logger().error(f"Error: team number not on GC.")
            return

        if self.ctx.blackboard.penalised:
            self.ctx.get_logger().error(f"Penalised! Facing: {math.degrees(self.ctx.blackboard.persistent_heading)}")
            if self._played_penalised_sound == False:
                subprocess.Popen(["aplay", "/home/nao/bin/penalised.wav"])
                self._played_penalised_sound = True
                self.ctx.get_logger().error("\n\nPENALISED\n\n")

            x = -Constants.PENALTY_CROSS_ABS_X
            y = -3500
            h = math.pi * 0.5

            # get angular distance between persistent_heading and pi/2
            if math.pi <= self.ctx.blackboard.persistent_heading < math.pi * 2.0:
                # facing up probably, penalised on bottom line
                y = 3500
                h = -math.pi * 0.5

            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = "world"
            msg.header.stamp = self.ctx.get_clock().now().to_msg()

            msg.pose.pose.position.x = float(x) / 1000.0
            msg.pose.pose.position.y = float(y) / 1000.0

            x, y, z, w = quaternion_from_euler(0.0, 0.0, h)

            msg.pose.pose.orientation.x = float(x)
            msg.pose.pose.orientation.y = float(y)
            msg.pose.pose.orientation.z = float(z)
            msg.pose.pose.orientation.w = float(w)

            self.ctx.initial_pose_pub.publish(msg)

        if gamestate != CommsRCGCD.STATE_PLAYING:
            self.ctx.blackboard.kick_success = False

        referee_signal_msg = VisionPipelineSwitchAsk()

        match gamestate:
            case CommsRCGCD.STATE_INITIAL:
                self._current_sub_skill = "Stand"
                referee_signal_msg.is_ref = False
                referee_signal_msg.check_mode = 0
                self.ctx.blackboard.referee_signal = referee_signal_msg
                self.ctx.ref_switch_pub.publish(referee_signal_msg)
            case CommsRCGCD.STATE_STANDBY:
                # ask to signal
                referee_signal_msg.is_ref = True
                referee_signal_msg.check_mode = 1
                self.ctx.blackboard.referee_signal = referee_signal_msg
                self._current_sub_skill = "Stand"

                # check if the signal is detected
                if self.ctx.blackboard.ref_result.referee_signal == 1:
                    self.ctx.blackboard.gameinfo.state = CommsRCGCD.STATE_READY
                if any(info.ref_detected == 1 for info in self.ctx.blackboard.robot_comms.values()):
                    self.ctx.blackboard.gameinfo.state = CommsRCGCD.STATE_READY
            case CommsRCGCD.STATE_READY:
                self._current_sub_skill = "Ready"
                referee_signal_msg.is_ref = False
                referee_signal_msg.check_mode = 0
                self.ctx.blackboard.referee_signal = referee_signal_msg
                self.ctx.ref_switch_pub.publish(referee_signal_msg)
            case CommsRCGCD.STATE_SET:
                self._current_sub_skill = "Stand"
                referee_signal_msg.is_ref = False
                referee_signal_msg.check_mode = 0
                self.ctx.blackboard.referee_signal = referee_signal_msg
                self.ctx.ref_switch_pub.publish(referee_signal_msg)
            case CommsRCGCD.STATE_PLAYING:
                self._current_sub_skill = "Playing"
                referee_signal_msg.is_ref = False
                referee_signal_msg.check_mode = 0
                self.ctx.blackboard.referee_signal = referee_signal_msg
                self.ctx.ref_switch_pub.publish(referee_signal_msg)
            case CommsRCGCD.STATE_FINISHED:
                self._playing_anthem = False
                self._current_sub_skill = "Sit"
            case _:
                self._current_sub_skill = "Stand"

    def _static_transition(self):
        pass

    def _dynamic_transition(self):
        pass

    def within_duration(self, ros_timestamp):
        if ros_timestamp is None:
            return False
        return abs(time.time() - ros_timestamp.sec) < 3

    def publish_comms_command(self):
        if self.ctx.blackboard.player_info is None or self.ctx.blackboard.robot_world is None:
            return

        self.ctx.blackboard.comms_command.player_number = self.ctx.blackboard.player_info.player_number
        self.ctx.blackboard.comms_command.robot_pos_x = self.ctx.blackboard.robot_world.point.x
        self.ctx.blackboard.comms_command.robot_pos_y = self.ctx.blackboard.robot_world.point.y
        self.ctx.blackboard.comms_command.heading = float(self.ctx.blackboard.robot_world.heading)

        ball = getAbsoluteBall(self.ctx.blackboard)
        if ball is not None:
            self.ctx.blackboard.comms_command.ball_pos_x = ball.pos_x
            self.ctx.blackboard.comms_command.ball_pos_y = ball.pos_y
            self.ctx.blackboard.comms_command.confidence = ball.confidence

        clock = Clock()
        self.ctx.blackboard.comms_command.timestamp = clock.now().to_msg()

        self.ctx.blackboard.comms_command.kick_success = self.ctx.blackboard.kick_success
        self.ctx.blackboard.comms_command.player_role = 1
        self.ctx.blackboard.comms_command.my_ball = self.ctx.blackboard.my_ball
        if self.ctx.blackboard.ref_result is not None:
            self.ctx.blackboard.comms_command.ref_detected = self.ctx.blackboard.ref_result.referee_signal
        else:
            self.ctx.blackboard.comms_command.ref_detected = 0
        self.ctx.comms_command.publish(self.ctx.blackboard.comms_command)
