import re
import time
import numpy as np
np.float = float

import re
import math
import rclpy
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import Point
from tf_transformations import euler_from_quaternion
from runswift_interfaces.msg import (
    SeBallsAbsolute,
    SeBallsRelative,
    CommsRCGCD,
    CommsTeamstate,
    MotionCommand,
    VisionBalls,
    CommsRobotPlayerInfo,
    CommsWhistle,
    BehavioursRobotInfo,
    BehavioursGamestate,
    VisionPipelineSwitchAsk,
    VisionPipelineSwitchResult,
    ButtonEvent,
)
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from src.utils.Constants import PENALTY_CROSS_ABS_X
from src.utils.TeamInfo import get_robot_roles

# ws_path = os.environ.get("ws_path", "/workspace/robot_ws")
# sys.path.insert(0, os.path.join(ws_path, "src"))

from src.datamodels.RobotPose import RobotPose

class Blackboard:
    _instance = None
    BALL_CONFIDENCE_THRESHOLD = 190

    def __new__(cls, ctx):
        if not cls._instance:
            cls._instance = super(Blackboard, cls).__new__(cls)
        return cls._instance

    def __init__(self, ctx):
        if not hasattr(self, "_initialized"):
            self.ctx = ctx
            self._robot_world = None
            self._ball_world = None
            self._ball_base = None
            self._gameinfo = None
            self._teaminfo = None
            self._motion_status = MotionCommand()
            self._vision_ball = VisionBalls()
            self._motion_command = MotionCommand()
            self._player_info = None
            self._last_vision_ball = 0
            self._whistle_info = CommsWhistle()
            self._referee_signal = VisionPipelineSwitchAsk()
            self._head_pitch_increment = 0.01
            self._head_yaw_increment = 0.01
            self._joint_positions = JointState()
            self._robot_comms = {}
            self._button_event = ButtonEvent()
            self._comms_command = BehavioursRobotInfo()
            self._ref_result = VisionPipelineSwitchResult()
            self._kick_success = False
            self._player_role = 1
            self._in_zone = False
            self._roles = {1: "Goalkeeper", 2: "Defender", 3: "Defender", 4: "Attacker", 5: "Attacker"}
            self._my_ball = False
            self._not_my_ball = False
            self._penalised = False
            self._unpenalised = False
            self._debug_behaviours_pose = None
            self._kicking_team = None
            self._debug_behaviours_pose = None

            self._initial_se_heading = 0.0
            self._initial_madgwick_heading = 0.0

            # our heading globally across the entire match. only "initial" can ever reset it
            self._madgwick_heading = 0.0

            # Not used for performance reasons
            # self._tf2_buffer = Buffer()
            # self._tf2_listener = TransformListener(self._tf2_buffer, self.ctx)

    # this should be a heading that is actually globally accurate over a long period of time
    # it is given to us by a madgwick filter at the moment.
    # the value is never negative, it's in radians, and 0/2pi means facing towards enemy, pi means facing towards our own goal.
    @property
    def persistent_heading(self):
        return (self._madgwick_heading - self._initial_madgwick_heading + self._initial_se_heading) % (2.0 * math.pi)

    @persistent_heading.setter
    def persistent_heading(self, raw_value):
        # negate here is necessary
        self._madgwick_heading = -raw_value

    @property
    def joint_positions(self):
        return dict(zip(self._joint_positions.name, self._joint_positions.position))

    @joint_positions.setter
    def joint_positions(self, value):
        if not isinstance(value, JointState):
            raise ValueError("joint_positions must be of type JointPositions")
        self._joint_positions = value

    @property
    def head_pitch_increment(self):
        return self._head_pitch_increment
    @head_pitch_increment.setter
    def head_pitch_increment(self, value):
        if not isinstance(value, float):
            raise ValueError("head_pitch_increment increment must be float")
        self._head_pitch_increment = value

    @property
    def head_yaw_increment(self):
        return self._head_yaw_increment
    @head_yaw_increment.setter
    def head_yaw_increment(self, value):
        if not isinstance(value, float):
            raise ValueError("head_yaw_increment increment must be float")
        self._head_yaw_increment = value

    @property
    def robot_world(self):
        if self._robot_world is None:
            return RobotPose(point=Point(), heading=-1.0)

        point = self._robot_world.pose.position
        point = Point(x=point.x * 1000, y=point.y * 1000, z=point.z * 1000)
        orientation_q = self._robot_world.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        yaw %= (2.0 * math.pi)

        return RobotPose(point, yaw)

        # Timeout added in case tf needs to catch up and load the buffer becuase the default timeout is very long
        try:
            namespace = self.ctx.get_namespace().strip("/")
            if "robot" in namespace:
                base_frame = f"{namespace}/base_footprint"
            else:
                base_frame = "base_footprint"
            transform = self._tf2_buffer.lookup_transform(
                "world", base_frame, rclpy.time.Time(seconds=0), timeout=rclpy.duration.Duration(seconds=0.0)
            )
            # import pdb; pdb.set_trace()
            # self.ctx.get_logger().info(f"Transform: {transform.transform.translation.x}, {transform.transform.translation.y}")
        except Exception as e:
            # self.ctx.get_logger().info(str(e))
            return RobotPose(point=Point(), heading=-1)  # Hopefully yaw can't be more than 10

        point = Point()
        point.x = transform.transform.translation.x * 1000
        point.y = transform.transform.translation.y * 1000
        point.z = transform.transform.translation.z * 1000

        orientation_q = transform.transform.rotation
        _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

        return RobotPose(point, yaw % (2 * math.pi))
    @robot_world.setter
    def robot_world(self, value):
        if not isinstance(value, PoseStamped):
            raise ValueError("robot_world must be of type PoseStamped")

        self._robot_world = value

        if self.gameinfo is not None and self.gameinfo.state == CommsRCGCD.STATE_INITIAL:
            yaw = float(self.robot_world.heading)
            # self.ctx.get_logger().error(f"!!!!! IN INITIAL AND RECEIVED A HEADING FROM SE: {math.degrees(yaw)} !!!!!")
            self._initial_se_heading = yaw
            self._initial_madgwick_heading = self._madgwick_heading

    @property
    def ball_world(self):
        return self._ball_world

    @ball_world.setter
    def ball_world(self, value):
        if not isinstance(value, SeBallsAbsolute):
            raise ValueError("ball_world must be of type SeBallsAbsolute")
        self._ball_world = value

    @property
    def ball_base(self):
        return self._ball_base

    @ball_base.setter
    def ball_base(self, value):
        if not isinstance(value, SeBallsRelative):
            raise ValueError("ball_base must be of type SeBallsRelative")
        self._ball_base = value

    @property
    def gameinfo(self):
        return self._gameinfo

    @gameinfo.setter
    def gameinfo(self, value):
        if not isinstance(value, CommsRCGCD):
            raise ValueError("gameinfo must be of type CommsRCGCD")
        if self._gameinfo is not None:
            if self._gameinfo.state == CommsRCGCD.STATE_PLAYING and value.state == CommsRCGCD.STATE_SET:
                value.state = CommsRCGCD.STATE_PLAYING
            if self._gameinfo.state == CommsRCGCD.STATE_READY and value.state == CommsRCGCD.STATE_STANDBY:
                value.state = CommsRCGCD.STATE_READY
            if self._gameinfo.state == CommsRCGCD.STATE_SET and value.state == CommsRCGCD.STATE_PLAYING:
                # Update everyone on starting positions
                self.ctx.controller.publish_comms_command()
        msg = BehavioursGamestate()  # Create a new message
        msg.state = value.state
        self.ctx.behaviours_gamestate.publish(msg)
        self._gameinfo = value

    @property
    def teaminfo(self):
        return self._teaminfo

    @teaminfo.setter
    def teaminfo(self, value):
        if not isinstance(value, CommsTeamstate):
            raise ValueError("teaminfo must be of type Commsteaminfo")
        self._teaminfo = value

    @property
    def motion_status(self):
        return self._motion_status

    @motion_status.setter
    def motion_status(self, value):
        if not isinstance(value, MotionCommand):
            raise ValueError("motion_status must be of type MotionCommand")
        self._motion_status = value

    @property
    def motion_command(self):
        return self._motion_command

    @motion_command.setter
    def motion_command(self, value):
        if not isinstance(value, MotionCommand):
            raise ValueError("motion_command must be of type MotionCommand")
        self._motion_command = value

    @property
    def vision_ball(self):
        if (
            self._vision_ball is None
            or len(self._vision_ball.ball_features) < 1
            or time.time() - self._vision_ball.header.stamp.sec > 5
        ):
            return None

        self._vision_ball.ball_features.sort(key=lambda msg: msg.confidence_score, reverse=True)
        ball = self._vision_ball.ball_features[0]
        if ball.confidence_score < self.BALL_CONFIDENCE_THRESHOLD:
            return
        return self._vision_ball

    @vision_ball.setter
    def vision_ball(self, value):
        if not isinstance(value, VisionBalls):
            raise ValueError("vision_ball must be of type VisionBall")
        # hax
        value.header.stamp.sec = int(time.time())
        self._vision_ball = value
        self._last_vision_ball = time.time()


    @property
    def roles(self):
        return self._roles

    @property
    def player_info(self):
        if self._player_info is not None:
            return self._player_info

        namespace = self.ctx.get_namespace().strip("/")
        match = re.search(r"robot(\d+)", namespace)
        if match:
            info = CommsRobotPlayerInfo()
            info.player_number = int(match.group(1))
            info.team_number = 18
            return info
        return None

    @player_info.setter
    def player_info(self, value):
        if not isinstance(value, CommsRobotPlayerInfo):
            raise ValueError("player_info must be of type CommsRobotPlayerInfo")
        self._player_info = value
        self._roles = get_robot_roles(self, value.current_players)

    @property
    def whistle_info(self):
        return self._whistle_info

    @whistle_info.setter
    def whistle_info(self, value):
        if not isinstance(value, CommsWhistle):
            raise ValueError("whistle_info must be of type CommsWhistle")
        if value.timestamp.sec - self._whistle_info.timestamp.sec > 3:
            if self._gameinfo.state == CommsRCGCD.STATE_SET:
                self._gameinfo.state = CommsRCGCD.STATE_PLAYING
            elif self._gameinfo.state == CommsRCGCD.STATE_PLAYING:
                self._gameinfo.state = CommsRCGCD.STATE_READY
        self._whistle_info = value

    @property
    def robot_comms(self):
        return self._robot_comms

    @robot_comms.setter
    def robot_comms(self, value):
        if not isinstance(value, BehavioursRobotInfo):
            raise ValueError("robot_comms must be of type BehavioursRobotInfo")
        player_number = value.player_number
        self._robot_comms[player_number] = value
        if value.kick_success is True:
            self._kick_success = True


    @property
    def comms_command(self):
        return self._comms_command

    @comms_command.setter
    def comms_command(self, value):
        if not isinstance(value, BehavioursRobotInfo):
            raise ValueError("comms_command must be of type BehavioursRobotInfo")
        self._comms_command = value

    @property
    def referee_signal(self):
        return self._referee_signal

    @referee_signal.setter
    def referee_signal(self, value):
        if not isinstance(value, VisionPipelineSwitchAsk):
            raise ValueError("referee_signal must be of type VisionPipelineSwitchAsk")
        if (
            self._referee_signal.check_mode != value.check_mode
            or self._referee_signal.is_ref != value.is_ref
        ):
            self.ctx.ref_switch_pub.publish(value)
        self._referee_signal = value

    @property
    def ref_result(self):
        return self._ref_result

    @ref_result.setter
    def ref_result(self, value):
        if not isinstance(value, VisionPipelineSwitchResult):
            raise ValueError("ref_result must be of type VisionPipelineSwitchResult")
        self._ref_result = value
        if (value.referee_signal == 1):
            self.ctx.controller.publish_comms_command()

    @property
    def button_event(self):
        return self._button_event

    @button_event.setter
    def button_event(self, value):
        if not isinstance(value, ButtonEvent):
            raise ValueError("button_event must be of type ButtonEvent")
        self._button_event = value

    @property
    def kick_success(self):
        return self._kick_success

    @kick_success.setter
    def kick_success(self, value):
        if not isinstance(value, bool):
            raise ValueError("kick_success must be of type bool")
        self._kick_success = value

    @property
    def player_role(self):
        return self._player_role

    @player_role.setter
    def player_role(self, value):
        if not isinstance(value, int):
            raise ValueError("player_role must be of type int")
        self._player_role = value

    @property
    def my_ball(self):
        return self._my_ball

    @my_ball.setter
    def my_ball(self, value):
        if not isinstance(value, bool):
            raise ValueError("my_ball must be of type bool")
        if self._my_ball != value:
            self._my_ball = value
            self.ctx.controller.publish_comms_command()
        else:
            self._my_ball = value

    @property
    def penalised(self):
        return self._penalised

    @penalised.setter
    def penalised(self, value):
        if not isinstance(value, int):
            raise ValueError("penalised must be of type int")
        if self._penalised and not value:
            self._unpenalised = True
        self._penalised = value

    @property
    def unpenalised(self):
        return self._unpenalised

    @unpenalised.setter
    def unpenalised(self, value):
        if not isinstance(value, bool):
            raise ValueError("unpenalised must be of type bool")
        self._unpenalised = value

    @property
    def kicking_team(self):
        return self._kicking_team

    @kicking_team.setter
    def kicking_team(self, value):
        if value is not None and not isinstance(value, int):
            raise ValueError("kicking_team must be of type int")
        self._kicking_team = value
