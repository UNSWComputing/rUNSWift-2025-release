// the topic id should match whats in the sql file
pub const ROS2_TOP_FRAME_TOPIC_ID: u32 = 1;
pub const ROS2_BOT_FRAME_TOPIC_ID: u32 = 2;
pub const ROS2_TOP_FRAME_WIDTH: u32 = 1280;
pub const ROS2_TOP_FRAME_HEIGHT: u32 = 960;
pub const ROS2_BOT_FRAME_WIDTH: u32 = 640;
pub const ROS2_BOT_FRAME_HEIGHT: u32 = 480;
pub const ROS2_FRAME_ID: &str = "default_cam";
pub const ROS2_IMAGE_ENCODING: &str = "yuv422_yuy2";

pub const ROS2_FIELD_FEATURES_TOPIC_ID: u32 = 3;
pub const ROS2_ODOMETRY_TOPIC_ID: u32 = 4;
pub const ROS2_VISION_BALLS_TOPIC_ID: u32 = 5;
pub const ROS2_JOINT_STATE_TOPIC_ID: u32 = 6;

pub const NAO_JOINT_NAMES: [&str; 25] = [
    "HeadYaw",
    "HeadPitch",
    "LShoulderPitch",
    "LShoulderRoll",
    "LElbowYaw",
    "LElbowRoll",
    "LWristYaw",
    "LHipYawPitch",
    "LHipRoll",
    "LHipPitch",
    "LKneePitch",
    "LAnklePitch",
    "LAnkleRoll",
    // "RHipYawPitch", present in joint state publisher gui, absent in legacy code
    "RHipRoll",
    "RHipPitch",
    "RKneePitch",
    "RAnklePitch",
    "RAnkleRoll",
    "RShoulderPitch",
    "RShoulderRoll",
    "RElbowYaw",
    "RElbowRoll",
    "RWristYaw",
    "LHand",
    "RHand",
];

pub const ROS2_ANGLE_STATE_TOPIC_ID: u32 = 7;
pub const ROS2_GYROSCOPE_STATE_TOPIC_ID: u32 = 8;

