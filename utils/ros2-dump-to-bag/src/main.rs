mod constants;
mod display_util;
mod msgs;
mod reader_util;

use crate::constants::{ROS2_BOT_FRAME_HEIGHT, ROS2_BOT_FRAME_TOPIC_ID, ROS2_BOT_FRAME_WIDTH,
                       ROS2_JOINT_STATE_TOPIC_ID, ROS2_TOP_FRAME_HEIGHT, ROS2_TOP_FRAME_TOPIC_ID, ROS2_TOP_FRAME_WIDTH, ROS2_VISION_BALLS_TOPIC_ID, 
                       ROS2_ANGLE_STATE_TOPIC_ID, ROS2_GYROSCOPE_STATE_TOPIC_ID, BlackboardSensorCodeEnum};
use cdr::{CdrLe, Infinite};
use clap::ArgAction;
use clap::Parser;
use constants::ROS2_FIELD_FEATURES_TOPIC_ID;
use constants::ROS2_ODOMETRY_TOPIC_ID;
use indicatif::ProgressBar;
use minifb::{Key, Window, WindowOptions};
use msgs::geometry_msgs::Point;
use msgs::geometry_msgs::Pose;
use msgs::geometry_msgs::Quaternion;
use msgs::runswift_interfaces;
use msgs::std_msgs::Header;
use msgs::vision_msgs::Point2D;
use rusqlite::params;
use std::io::{self};
use crate::msgs::{sensor_msgs, std_msgs, nao_lola_sensor_msgs};

#[derive(Parser)]
#[command(version, about, long_about = None)]
struct Args {
    /// Name of file to read
    #[arg(required = true, index = 1)]
    blackboard_file: String,
    /// Name of file to write
    #[arg(short, long, default_value = "rosbag2.db3")]
    output_file: String,
    /// if set, the program won't spawn a display window to show the current frame its up to
    #[arg(short, long, action=ArgAction::SetFalse)]
    display: bool,
    /// start offset (inclusive)
    #[arg(short, long, default_value_t = 0)]
    start_offset: i32,
    /// end offset (exclusive) [default inf]
    #[arg(short, long)]
    end_offset: Option<i32>,
    /// how many bbd frames until the cam is rendered to the display. 1 means every frame, 2 means every second frame etc.
    /// Set to a high number to improve performance (improvement is marginal past 5)
    #[arg(short, long, default_value_t = 5)]
    frame_modulo: i32,
}

fn main() -> io::Result<()> {
    // Parse command line arguments
    let args = Args::parse();
    let mut rosbag_file = reader_util::setup_new_ros2_bag(&args.output_file).unwrap();
    // Create a window for displaying the images
    let mut window: Option<Window> = if args.display {
        Some(
            Window::new(
                "Image Viewer",
                1920, // Window width
                1080, // Window height
                WindowOptions::default(),
            )
            .unwrap_or_else(|e| {
                panic!("Failed to create a window: {}", e);
            }),
        )
    } else {
        None
    };

    let mut bbd_reader: reader_util::BlackboardReader =
        reader_util::BlackboardReader::new(&args.blackboard_file);
    let mut topic_msg_count = 1;
    println!(
        "Reading blackboard file: {} (skipping to offset {})",
        args.blackboard_file, args.start_offset
    );
    let mut progress_bar: Option<ProgressBar> = None;
    loop {
        let mut done = false;
        // we want to process in batches of 50 messages (as sqlite transactions to improve performance)
        let transaction = rosbag_file.transaction().unwrap();
        {
            let mut stmt = transaction
                .prepare(
                    "INSERT INTO messages (id, topic_id, timestamp, data) VALUES (?1, ?2, ?3, ?4)",
                )
                .unwrap();
            for _ in 0..50 {
                let currently_seeking = bbd_reader.current_frame() < args.start_offset - 1;
                match if currently_seeking {
                    bbd_reader.next_frame(true)
                } else {
                    bbd_reader.next_frame(false)
                } {
                    Some(message) => {
                        if currently_seeking {
                            if progress_bar.is_none() {
                                progress_bar = Some(ProgressBar::new(args.start_offset as u64));
                            }
                            progress_bar.as_ref().unwrap().inc(1);
                            continue;
                        } else if bbd_reader.current_frame() == args.start_offset {
                            println!("Finished seeking to frame {}", args.start_offset);
                            let bytes_left =
                                bbd_reader.file_size() - bbd_reader.current_file_position();
                            let frames_left_guess = if args.end_offset.is_some() { (args.end_offset.unwrap() - args.start_offset) as u64 } else { bytes_left / bbd_reader.message_size() as u64 };
                            progress_bar = Some(ProgressBar::new(frames_left_guess));
                        }
                        // processing
                        let vision = message.vision.unwrap();
                        let mut top_buffer: Option<Vec<u32>> = None;
                        let mut bot_buffer: Option<Vec<u32>> = None;
                        // whether we want to spend compute time on a frame this cycle
                        let committing_to_frame =
                            args.display && bbd_reader.current_frame() % args.frame_modulo == 0;
                        // bbd gives it in micros so turn to nanos
                        let timestamp = vision.timestamp.unwrap() as u64 * 1000;

                        if let Some(top_frame) = vision.top_frame {
                            if committing_to_frame {
                                top_buffer = Some(display_util::yuyv_to_rgb(
                                    &top_frame,
                                    ROS2_TOP_FRAME_WIDTH as usize,
                                    ROS2_TOP_FRAME_HEIGHT as usize,
                                ));
                            }
                            let image = reader_util::bbd_frame_to_std_img(
                                top_frame,
                                ROS2_TOP_FRAME_WIDTH,
                                ROS2_TOP_FRAME_HEIGHT,
                                timestamp,
                            );
                            let cdr_bytes =
                                cdr::serialize::<_, _, CdrLe>(&image, Infinite).unwrap();
                            stmt.execute(params![
                                topic_msg_count,
                                ROS2_TOP_FRAME_TOPIC_ID,
                                timestamp,
                                cdr_bytes
                            ])
                            .unwrap();
                            topic_msg_count += 1;
                        }

                        if let Some(bot_frame) = vision.bot_frame {
                            if committing_to_frame {
                                bot_buffer = Some(display_util::yuyv_to_rgb(
                                    &bot_frame,
                                    ROS2_BOT_FRAME_WIDTH as usize,
                                    ROS2_BOT_FRAME_HEIGHT as usize,
                                ));
                            }
                            let image = reader_util::bbd_frame_to_std_img(
                                bot_frame,
                                ROS2_BOT_FRAME_WIDTH,
                                ROS2_BOT_FRAME_HEIGHT,
                                timestamp,
                            );
                            let cdr_bytes =
                                cdr::serialize::<_, _, CdrLe>(&image, Infinite).unwrap();
                            stmt.execute(params![
                                topic_msg_count,
                                ROS2_BOT_FRAME_TOPIC_ID,
                                timestamp,
                                cdr_bytes
                            ])
                            .unwrap();
                            topic_msg_count += 1;
                        }

                        // !! add new messages here !!
                        // field features
                        if !vision.field_features.is_empty() {
                            let field_features: Vec<_> = vision
                                .field_features
                                .into_iter()
                                .map(|ffi| {
                                    let rr = ffi.rr.unwrap();
                                    let distance = rr.vec[0];
                                    let heading: f32 = rr.vec[1];
                                    let orientation: f32 = rr.vec[2];

                                    runswift_interfaces::VisionFieldFeature {
                                        r#type: ffi.r#type.unwrap() as u8,
                                        confidence_score: 255, // we don't have this data
                                        feature_coordinates: Pose {
                                            point: Point {
                                                x: (distance * heading.cos()) as f64,
                                                y: (distance * heading.sin()) as f64,
                                                z: 0.0,
                                            },
                                            quaternion: Quaternion::from_rotation_z(
                                                orientation as f64,
                                            ),
                                        },
                                        feature_pixel_coordinates: Point2D {
                                            // make it really obvious these aren't real values
                                            x: -1.0,
                                            y: -1.0,
                                        },
                                    }
                                })
                                .collect();

                            let field_features = runswift_interfaces::VisionFieldFeatures {
                                header: Header::from_timestamp(timestamp),
                                field_features,
                            };

                            let cdr_bytes =
                                cdr::serialize::<_, _, CdrLe>(&field_features, Infinite).unwrap();

                            // insert to field features topic
                            stmt.execute(params![
                                topic_msg_count,
                                ROS2_FIELD_FEATURES_TOPIC_ID,
                                timestamp,
                                cdr_bytes
                            ])
                            .unwrap();

                            topic_msg_count += 1;
                        }
                        // vision balls
                        if !vision.balls.is_empty() {
                            let ball_features: Vec<_> = vision
                                .balls
                                .into_iter()
                                .map(|ball: msgs::offnao::vision::BallInfo| {
                                    let ball_rr_coords = ball.rr.unwrap().vec;
                                    runswift_interfaces::VisionBallFeature {
                                        ball_coordinates: Point {
                                            x: ball_rr_coords[0] as f64,
                                            y: ball_rr_coords[1] as f64,
                                            z: ball_rr_coords[2] as f64,
                                        },
                                        ball_pixel_coordinates: Point2D { x: -1.0, y: -1.0 }, // not set anything as it isn't clear if it's top or bottom cam coord space
                                        pixel_radius: ball.radius.unwrap() as u16,
                                        confidence_score: 255, // we don't have this data
                                    }
                                })
                                .collect();
                            let ball_features = runswift_interfaces::VisionBalls {
                                header: Header::from_timestamp(timestamp),
                                ball_features,
                            };

                            let cdr_bytes =
                                cdr::serialize::<_, _, CdrLe>(&ball_features, Infinite).unwrap();

                            // insert to field features topic
                            stmt.execute(params![
                                topic_msg_count,
                                ROS2_VISION_BALLS_TOPIC_ID,
                                timestamp,
                                cdr_bytes
                            ])
                            .unwrap();

                            topic_msg_count += 1;
                        }

                        let motion = message.motion.unwrap();
                        let sensors = motion.sensors.unwrap();
                        // odometer
                        if let Some(odometry) = motion.odometry {
                            let odometry = runswift_interfaces::MotionOdometry {
                                header: Header::from_timestamp(timestamp),
                                forward: odometry.forward(),
                                left: odometry.left(),
                                turn: odometry.turn(),
                            };

                            let cdr_bytes =
                                cdr::serialize::<_, _, CdrLe>(&odometry, Infinite).unwrap();

                            // insert to field features topic
                            stmt.execute(params![
                                topic_msg_count,
                                ROS2_ODOMETRY_TOPIC_ID,
                                timestamp,
                                cdr_bytes
                            ])
                            .unwrap();

                            topic_msg_count += 1;
                        }
                        // joint values
                        {
                            let joint_positions: Vec<f64> = sensors.joints.unwrap().angles.iter().map(|f| *f as f64).collect();
                            let joint_names: Vec<std_msgs::String> = constants::NAO_JOINT_NAMES.to_vec().iter().map(|s| std_msgs::String { data: s.to_string() }).collect();
                            if joint_positions.len() != joint_names.len() {
                                panic!("Joint info length mismatch. Expected {} got {}", constants::NAO_JOINT_NAMES.len(), joint_names.len());
                            }
                            let joint_msg = sensor_msgs::JointState {
                                header: Header::from_timestamp(timestamp + 3.154e+16 as u64),
                                name: joint_names,
                                position: joint_positions,
                                velocity: Vec::new(),
                                effort: Vec::new(),
                            };

                            let cdr_bytes =
                                cdr::serialize::<_, _, CdrLe>(&joint_msg, Infinite).unwrap();
                            stmt.execute(params![
                                    topic_msg_count,
                                    ROS2_JOINT_STATE_TOPIC_ID,
                                    timestamp,
                                    cdr_bytes
                                ])
                                .unwrap();

                            topic_msg_count += 1;
                        }
                        // angle and gyro
                        {
                            // println!("{:?}", message.behaviour.unwrap().request);
                            // mapping here: https://github.com/UNSWComputing/rUNSWift/blob/848d0e1570be8c082993a8cd6b8cac0a2388c4f6/robot/blackboard/serialise.cpp#L212
                            let sensor_values: Vec<f32> = sensors.sensors;
                            // println!("{:?}", sensor_values);
                            let angle_msg = nao_lola_sensor_msgs::Angle {
                                x: sensor_values[0],
                                y: sensor_values[1],
                            };

                            let gyro_msg = nao_lola_sensor_msgs::Gyroscope {
                                x: sensor_values[0],
                                y: sensor_values[1],
                                z: sensor_values[2],
                            };

                            
                            let angle_msg_bytes =
                                cdr::serialize::<_, _, CdrLe>(&angle_msg, Infinite).unwrap();
                            let gyro_msg_bytes =
                                cdr::serialize::<_, _, CdrLe>(&gyro_msg, Infinite).unwrap();
                            stmt.execute(params![
                                    topic_msg_count,
                                    ROS2_ANGLE_STATE_TOPIC_ID,
                                    timestamp,
                                    angle_msg_bytes
                                ])
                                .unwrap();
                            topic_msg_count += 1;
                            
                            stmt.execute(params![
                                    topic_msg_count,
                                    ROS2_GYROSCOPE_STATE_TOPIC_ID,
                                    timestamp,
                                    gyro_msg_bytes
                                ])
                                .unwrap();

                            topic_msg_count += 1;
                        }
                        // display the image
                        if let Some(window) = &mut window {
                            if committing_to_frame {
                                let stacked_buffer = display_util::stack_images(
                                    top_buffer.unwrap_or(vec![
                                        0;
                                        (ROS2_TOP_FRAME_WIDTH * ROS2_TOP_FRAME_HEIGHT)
                                            as usize
                                    ]),
                                    ROS2_TOP_FRAME_WIDTH as usize,
                                    bot_buffer.unwrap_or(vec![
                                        0;
                                        (ROS2_BOT_FRAME_WIDTH * ROS2_BOT_FRAME_HEIGHT)
                                            as usize
                                    ]),
                                    ROS2_BOT_FRAME_WIDTH as usize,
                                );
                                window
                                    .update_with_buffer(
                                        &*stacked_buffer.0,
                                        stacked_buffer.1,
                                        stacked_buffer.2,
                                    )
                                    .unwrap();
                            } else {
                                window.update();
                            }

                            if !window.is_open() || window.is_key_down(Key::Escape) {
                                done = true; // Exit if the window is closed or Escape is pressed
                                println!("Exiting at frame {}", bbd_reader.current_frame());
                            }
                        }

                        if bbd_reader.current_frame() + 1 == args.end_offset.unwrap_or(i32::MAX) {
                            done = true;
                        }
                    }
                    None => {
                        done = true;
                    }
                }
                if progress_bar.is_some() {
                    progress_bar.as_ref().unwrap().inc(1);
                }
                if done {
                    break;
                }
            }
        }

        transaction.commit().unwrap();
        if done {
            break;
        }
    }
    if progress_bar.is_some() {
        progress_bar.as_ref().unwrap().finish();
    }
    Ok(())
}
