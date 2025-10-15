use crate::constants::ROS2_IMAGE_ENCODING;
use crate::msgs::{sensor_msgs, std_msgs};
use prost::Message;
use rusqlite::{Connection, Result};
use std::io::{Read, Seek};

/// setup a new sqlite database for storing ROS2 messages (as a ros2 bag is just a sqlite db)
pub fn setup_new_ros2_bag(filename: &str) -> Result<Connection> {
    // if file already exists, ask the user if they want to delete it
    if std::path::Path::new(filename).exists() {
        println!(
            "File {} already exists. Do you want to delete it? (y/n)",
            filename
        );
        let mut input = String::new();
        std::io::stdin().read_line(&mut input).unwrap();
        if input.trim() == "y" {
            std::fs::remove_file(filename).unwrap();
        } else {
            panic!("File already exists. Exiting.");
        }
    }
    let conn = Connection::open(filename)?;
    let setup_script = include_str!("sql/ros2bag_setup.sql");
    conn.execute_batch(setup_script)?;
    Ok(conn)
}

/// turn a bbd frame into a sensor_msgs::Image
///
/// # Arguments
///
/// * `frame`: frame data in yuyv format
/// * `width`: one of ROS2_TOP_FRAME_WIDTH or ROS2_BOT_FRAME_WIDTH
/// * `height`: one of ROS2_TOP_FRAME_HEIGHT or ROS2_BOT_FRAME_HEIGHT
/// * `timestamp`: timestamp in nanoseconds
///
/// returns a sensor_msgs::Image
///
pub fn bbd_frame_to_std_img(
    frame: Vec<u8>,
    width: u32,
    height: u32,
    timestamp: u64,
) -> sensor_msgs::Image {
    sensor_msgs::Image {
        header: std_msgs::Header::from_timestamp(timestamp),
        height,
        width,
        encoding: std_msgs::String {
            data: ROS2_IMAGE_ENCODING.to_string(),
        },
        is_bigendian: 0,
        // for yuyv 4:2:2, the step is the width * 2 (2 bytes per pixel)
        step: width * 2,
        data: frame,
    }
}

/// read a hex dump of cdr and convert it to a sensor_msgs::Image
pub fn read_image_from_file(file_path: &str) -> sensor_msgs::Image {
    let file = std::fs::File::open(file_path).unwrap();
    let mut reader = std::io::BufReader::new(file);
    // convert hex to bytes
    let mut hex_string = String::new();
    reader.read_to_string(&mut hex_string).unwrap();
    let bytes = hex::decode(hex_string.trim()).unwrap();
    cdr::de::deserialize_data::<_, cdr::LittleEndian>(&bytes).unwrap()
}

pub struct BlackboardReader {
    file_size: u64,
    reader: std::io::BufReader<std::fs::File>,
    current_frame: i32,
    message_size: usize,
}

impl BlackboardReader {
    pub fn new(file_path: &str) -> Self {
        let file = std::fs::File::open(file_path).unwrap();
        BlackboardReader {
            file_size: file.metadata().unwrap().len(),
            reader: std::io::BufReader::new(file),
            current_frame: -1,
            message_size: 0,
        }
    }

    /// Read the next frame in a blackboard file
    ///
    /// # Arguments
    ///
    /// * `skip_parse`: if true, we will save compute and not try deserialise the bytes. Useful for seeking
    ///
    /// returns: Option<Blackboard> if successful, None if we reach the end of the file. Returns empty Blackboard if `skip_parse` is true
    ///
    pub fn next_frame(&mut self, skip_parse: bool) -> Option<crate::msgs::offnao::Blackboard> {
        // Step 1: Read the size of the next protobuf message
        let mut size_buffer = [0u8; 4]; // Assuming unsigned 32-bit integer for size
        if self.reader.read_exact(&mut size_buffer).is_err() {
            return None; // Exit loop when reaching the end of the file
        }
        self.message_size = u32::from_le_bytes(size_buffer) as usize;

        if skip_parse {
            self.reader
                .seek(std::io::SeekFrom::Current(self.message_size as i64))
                .unwrap();
            self.current_frame += 1;
            return Some(crate::msgs::offnao::Blackboard::default());
        }
        // Step 2: Read the protobuf message based on the size
        let mut message_buffer = vec![0u8; self.message_size];
        if self.reader.read_exact(&mut message_buffer).is_err() {
            eprintln!("Error: Unable to read the full message");
            return None;
        }

        // Step 3: Decode the protobuf message
        match crate::msgs::offnao::Blackboard::decode(&*message_buffer) {
            Ok(message) => {
                self.current_frame += 1;
                if self.current_frame == 0 {
                    if !self.is_bb_raw_image_flag_set(*message.mask.as_ref().unwrap()) {
                        // print error warning that the mask is not set
                        eprintln!("Warning: The raw image mask is not set. The bag file will likely be missing raw images!");
                    }
                }
                Some(message)
            }
            Err(e) => {
                eprintln!(
                    "Error: Unable to decode the message: {} at seek position {:?} (in hex: {:x?})",
                    e,
                    self.reader.stream_position(),
                    self.reader.stream_position()
                );
                eprintln!("Attempting to recover...");
                self.seek_to_next_valid_frame();
                self.next_frame(false)
            }
        }
    }

    pub fn current_frame(&self) -> i32 {
        self.current_frame
    }
    pub fn message_size(&self) -> usize {
        self.message_size
    }

    pub fn current_file_position(&mut self) -> u64 {
        self.reader.stream_position().unwrap()
    }
    pub fn file_size(&self) -> u64 {
        self.file_size
    }
    // see TransmitterDefs: https://github.com/UNSWComputing/rUNSWift/blob/2069658c3c14701b112e42884512ae67f37ae354/robot/transmitter/TransmitterDefs.hpp#L14
    // checks if 3rd most significant bit is set
    fn is_bb_raw_image_flag_set(&self, mask: u64) -> bool {
        let num_bits = std::mem::size_of::<u64>() * 8; // Total number of bits in the type
        let binary_string: String = (0..num_bits)
            .rev() // Iterate from MSB to LSB
            .map(|i| if (mask & (1 << i)) != 0 { '1' } else { '0' })
            .collect();
        println!(
            "Blackboard binary mask is: {binary_string} (decimal as {})",
            mask
        );
        (mask & (1 << 2)) != 0
    }

    /// an error recovery process where we use the historic message size, take the 2 MSB and search for them in a rolling window
    fn seek_to_next_valid_frame(&mut self) {
        let current_pos = self.current_file_position();
        // use the last msg size's 2 MSB as a hint
        let last_2_bytes = (self.message_size >> 16) as u16;

        // search for the last 2 bytes in a rolling window
        let mut found = false;
        for i in (current_pos..self.file_size).step_by(2) {
            let mut rolling_window = [0u8; 2];
            self.reader.read_exact(&mut rolling_window).unwrap();
            if u16::from_le_bytes(rolling_window) == last_2_bytes {
                found = true;
                println!("Found a potential recovery point at {} (0x{:x})!", i, i);
                self.reader.seek(std::io::SeekFrom::Current(-4)).unwrap(); // move the reader to the start of the message (extra 2 offset as size is 4 bytes)
                break;
            }
        }
        if !found {
            panic!("Error: Unable to recover :( Exiting.");
        }
    }
}
