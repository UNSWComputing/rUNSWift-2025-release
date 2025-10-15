import math
from nao_lola_sensor_msgs.msg._joint_statuses import JointStatuses
import rclpy
from rclpy.node import Node
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
    CommsRCGCRRD,
    BehavioursGamestate,
    VisionPipelineSwitchAsk,
    VisionPipelineSwitchResult,
    ButtonEvent,
)
from sensor_msgs.msg import JointState
from src.Blackboard import Blackboard
from src.Controller import Controller
from std_msgs.msg import String, Float32

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class BehaviourNode(Node):
    """
    A ROS2 node to handle robot behaviours. It subscribes to various topics,
    updates a shared Blackboard with data, and uses a Controller to manage
    behaviour logic.
    """

    def __init__(self):
        super().__init__("behaviour_node")

        self.TICK_RATE = 30  # in Hz

        self.debug_behaviours_pose = self.create_subscription(PoseStamped, '/debug_behaviours_pose', self.pose_callback, 1)

        self.ball_world_sub = self.create_subscription(SeBallsAbsolute, "/stateestimation/SeBallsAbsolute", self.callback, 1)
        self.ball_base_sub = self.create_subscription(SeBallsRelative, "/ball_base", self.callback, 1)
        self.gameinfo_sub = self.create_subscription(CommsRCGCD, "/gc/data", self.callback, 1)
        self.teaminfo_sub = self.create_subscription(CommsTeamstate, "/teaminfo", self.callback, 1)
        # self.motion_status_sub = self.create_subscription(MotionCommand, "/motion_status", self.callback, 1)
        self.vision_ball = self.create_subscription(VisionBalls, "vision/VBalls", self.callback, 1)
        self.robot_info_sub = self.create_subscription(CommsRobotPlayerInfo, "/robot_info", self.callback, 1)
        self.whistle_info_sub = self.create_subscription(CommsWhistle, "/whistle_info", self.callback, 1)
        # self.joint_values_sub = self.create_subscription(JointState, "joint_states", self.callback, 1)
        self.self_world_sub = self.create_subscription(PoseStamped, "/stateestimation/self", self.callback,  QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1
            ))
        self.robot_comms_sub = self.create_subscription(CommsRCGCRRD, "gc/team_data", self.callback, 1)
        self.ref_result_sub = self.create_subscription(VisionPipelineSwitchResult, "vision/ref_result", self.callback, 1)
        self.button_subscription = self.create_subscription(ButtonEvent, '/hri/button_event', self.callback, 1)

        self.comms_command = self.create_publisher(BehavioursRobotInfo, "/robot_comms", 1)
        self.motion_command = self.create_publisher(MotionCommand, "/motion_command", QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=3
            ))
        self.current_behaviour = self.create_publisher(String, "/current_behaviour", 1)
        self.behaviours_gamestate = self.create_publisher(BehavioursGamestate, "/behaviours_gamestate", 1)
        self.ref_switch_pub = self.create_publisher(VisionPipelineSwitchAsk, "vision/ref_switch", 1)
        
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 1)
        self.madwick_sub = self.create_subscription(Imu, "/imu_madgwick", self.callback, 1)
        
        # self.yaw_debub_pub = self.create_publisher(Float32, "/debug/behaviours_yaw", 1)

        self.blackboard = Blackboard(self)
        self.controller = Controller(ctx=self)

        self.behaviour_hierarchy = ""

        self.tick_timer = self.create_timer(1.0 / self.TICK_RATE, self.controller.tick)
        self.get_logger().debug("Initalised Behaviour Node")

    def callback(self, msg):
        if isinstance(msg, SeBallsAbsolute):
            self.blackboard.ball_world = msg
        elif isinstance(msg, SeBallsRelative):
            self.blackboard.ball_base = msg
        elif isinstance(msg, CommsRCGCD):
            self.blackboard.gameinfo = msg
        elif isinstance(msg, CommsTeamstate):
            self.blackboard.teamstate = msg
        elif isinstance(msg, MotionCommand):
            self.blackboard.motion_status = msg
        elif isinstance(msg, VisionBalls):
            self.blackboard.vision_ball = msg
        elif isinstance(msg, CommsRobotPlayerInfo):
            self.blackboard.player_info = msg
        elif isinstance(msg, CommsWhistle):
            self.blackboard.whistle_info = msg
        elif isinstance(msg, JointState):
            self.blackboard.joint_positions = msg
        elif isinstance(msg, PoseStamped):
            self.blackboard.robot_world = msg
        elif isinstance(msg, CommsRCGCRRD):
            self.blackboard.robot_comms = msg.behaviours_info
        elif isinstance(msg, VisionPipelineSwitchResult):
            self.blackboard.ref_result = msg
        elif isinstance(msg, ButtonEvent):
            self.blackboard.button_event = msg
        elif isinstance(msg, Imu):
            q = msg.orientation
            _, _, raw_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            # self.get_logger().error(f"Received IMU q: {q.x} {q.y} {q.z} {q.w}, angle: {math.degrees(raw_yaw)}")
            self.blackboard.persistent_heading = raw_yaw
            # self.get_logger().error(f"  Angle is now: {math.degrees(self.blackboard.persistent_heading)}, initial_se_heading: {math.degrees(self.blackboard._initial_se_heading)}, initial_madgwick_heading: {math.degrees(self.blackboard._initial_madgwick_heading)}")
            # self.yaw_debub_pub.publish(Float32(data=math.degrees(self.blackboard.persistent_heading)))
        else:
            raise ValueError("Behaviour message type is not defined in BehaviourNode")

    def pose_callback(self, msg):
        self.blackboard._debug_behaviours_pose = msg


def main(args=None):
    rclpy.init(args=args)

    behaviour_node = BehaviourNode()

    rclpy.spin(behaviour_node)

    behaviour_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
