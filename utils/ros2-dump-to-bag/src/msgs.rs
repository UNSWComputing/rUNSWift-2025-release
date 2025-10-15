// these come from the standard ROS2 message definitions defined here: https://github.com/ros2/common_interfaces
// we also include custom runswift msgs defined in robot_ws/src/runswift_interfaces/msg
// we will use serde to convert these to and from the cdr binary format

pub mod builtin_interfaces {
    use serde::{Deserialize, Serialize};
    // https://github.com/ros2/rcl_interfaces/blob/humble/builtin_interfaces/msg/Time.msg
    #[derive(Serialize, Deserialize, Debug)]
    pub struct Time {
        pub sec: i32,
        pub nanosec: u32,
    }
}

pub mod std_msgs {
    use crate::{constants::ROS2_FRAME_ID, msgs::builtin_interfaces};
    use serde::{Deserialize, Serialize};

    // https://github.com/ros2/common_interfaces/blob/humble/std_msgs/msg/String.msg
    #[derive(Serialize, Deserialize, Debug)]
    pub struct String {
        pub data: std::string::String,
    }

    // https://github.com/ros2/common_interfaces/blob/humble/std_msgs/msg/Header.msg
    #[derive(Serialize, Deserialize, Debug)]
    pub struct Header {
        pub stamp: builtin_interfaces::Time,
        pub frame_id: String,
    }

    impl Header {
        pub fn from_timestamp(timestamp: u64) -> Header {
            Header {
                stamp: builtin_interfaces::Time {
                    // seconds since epoch
                    sec: (timestamp / 1_000_000_000) as i32,
                    // rest of the timestamp
                    nanosec: (timestamp % 1_000_000_000) as u32,
                },
                frame_id: String {
                    data: ROS2_FRAME_ID.to_string(),
                },
            }
        }
    }
}

pub mod sensor_msgs {
    use serde::{Deserialize, Serialize};
    use crate::msgs::std_msgs::Header;

    // https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/Image.msg
    #[derive(Serialize, Deserialize, Debug)]
    pub struct Image {
        pub header: Header,
        pub height: u32,
        pub width: u32,
        pub encoding: crate::msgs::std_msgs::String,
        pub is_bigendian: u8,
        pub step: u32,
        pub data: Vec<u8>,
    }

    // https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/JointState.msg
    #[derive(Serialize, Deserialize, Debug)]
    pub struct JointState {
        pub header: Header,
        pub name: Vec<crate::msgs::std_msgs::String>,
        pub position: Vec<f64>,
        pub velocity: Vec<f64>,
        pub effort: Vec<f64>,
    }
}

pub mod geometry_msgs {
    use serde::{Deserialize, Serialize};

    #[derive(Serialize, Deserialize, Debug)]
    pub struct Quaternion {
        x: f64,
        y: f64,
        z: f64,
        w: f64,
    }

    impl Quaternion {
        pub fn from_rotation_z(theta: f64) -> Self {
            let half_theta = theta / 2.0;
            Quaternion {
                x: 0.0,
                y: 0.0,
                z: half_theta.sin(),
                w: half_theta.cos(),
            }
        }
    }

    #[derive(Serialize, Deserialize, Debug)]
    pub struct Point {
        pub x: f64,
        pub y: f64,
        pub z: f64,
    }

    #[derive(Serialize, Deserialize, Debug)]
    pub struct Pose {
        pub point: Point,
        pub quaternion: Quaternion,
    }
}

pub mod vision_msgs {
    use serde::{Deserialize, Serialize};

    #[derive(Serialize, Deserialize, Debug)]
    pub struct Point2D {
        pub x: f64,
        pub y: f64,
    }
}

//// begin custom messages
// expose the old blackboard prost structs
pub mod offnao {
    include!(concat!(env!("OUT_DIR"), "/offnao.rs"));
}

pub mod runswift_interfaces {
    use serde::{Deserialize, Serialize};

    use super::{geometry_msgs, std_msgs, vision_msgs};

    #[repr(u8)]
    #[derive(Serialize, Deserialize, Debug)]
    pub enum FieldFeatureType {
        None = 0,
        Line = 1,              // used
        Corner = 2,            // used
        TJunction = 3,         // used
        PenaltySpot = 4,       // not used for localisation
        CentreCircle = 5,      // used
        FieldLinePoint = 6,    // not used for localisation
        XJunction = 7,         // not used for localisation
        ParallelLine = 8,      // not used for localisation
        GoalBoxCorner = 9,     // not used for localisation
        PenaltyBoxCorner = 10, // not used for localisation
    }

    #[derive(Serialize, Deserialize, Debug)]
    pub struct VisionFieldFeature {
        pub r#type: u8,
        pub confidence_score: u8,
        pub feature_coordinates: geometry_msgs::Pose,
        pub feature_pixel_coordinates: vision_msgs::Point2D,
    }

    #[derive(Serialize, Deserialize, Debug)]
    pub struct VisionFieldFeatures {
        pub header: std_msgs::Header,
        pub field_features: Vec<VisionFieldFeature>,
    }

    #[derive(Serialize, Deserialize, Debug)]
    pub struct MotionOdometry {
        pub header: std_msgs::Header,
        pub forward: f32,
        pub left: f32,
        pub turn: f32,
    }

    #[derive(Serialize, Deserialize, Debug)]
    pub struct VisionBallFeature {
        pub ball_coordinates: geometry_msgs::Point,
        pub ball_pixel_coordinates: vision_msgs::Point2D,
        pub pixel_radius: u16,
        pub confidence_score: u8,
    }

    #[derive(Serialize, Deserialize, Debug)]
    pub struct VisionBalls {
        pub header: std_msgs::Header,
        pub ball_features: Vec<VisionBallFeature>,
    }
}

pub mod nao_lola_sensor_msgs {
    use serde::{Deserialize, Serialize};

    #[derive(Serialize, Deserialize, Debug)]
    pub struct Angle {
        pub x: f32,
        pub y: f32,
    }
    
    #[derive(Serialize, Deserialize, Debug)]
    pub struct Gyroscope {
        pub x: f32,
        pub y: f32,
        pub z: f32,
    }
}
