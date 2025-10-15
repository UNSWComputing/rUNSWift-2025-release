# ROS2 Blackboard dumps to ROS2 bags

The goal of this program is to take our existing blackboard dump files (bbd2) and convert them to ROS bags (for humble)

To learn how to use, do `make ros-dump-to-bag -- --help`

After making a bag file, you can play it in ros2 with `ros2 bag play <bag file>` (use --help on these commands to learn more)
You can confirm everything works by listing the topics (you should see the topics this program outputs) and echoing them.
```bash
ros2 bag info <bag file>
ros2 bag play <bag file>
# in another terminal
ros2 topic list
ros2 topic echo <topic>
```

Features:

- Converts bbd2 files to ROS2 bags
- Can offset to specific points in the bb2d file with start and end offset params
- Can display the current images its up to in the bbd2 file while processing (can turn off to improve performance)
- Progress bars

This isn't designed to be a complete replacement of the bbd2 file, as we will incrementally be adding topic replacements for all data in the bbd2 file.

Therefore, keeping the bbd2 file around would be generally important.

To add new topics:

1. Add the topic schema to the `msgs.rs`
2. Add the topic to the sql file in `src/sql/ros2bag_setup.sql`. To know exactly what to put here, it's recommended to
   make a ROS bag of the topic you want and introspect it with `sqlite3` to see what the row looks like.
3. For the ID you chose for the new topic in step 2, add a new constant for the topic id in `src/constants.rs`
4. The process will require some significant code change but could be made easier if required: examine how images are
   added to the bag in `src/main.rs` and copy the pattern for your new topic.

## Other

Cross compile for windows to improve performance:
```bash
sudo apt-get install mingw-w64 
rustup target add x86_64-pc-windows-gnu
cargo build --release --target x86_64-pc-windows-gnu
```