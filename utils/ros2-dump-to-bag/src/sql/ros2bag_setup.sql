-- schema
PRAGMA foreign_keys=OFF;
BEGIN TRANSACTION;

CREATE TABLE schema(schema_version INTEGER PRIMARY KEY,ros_distro TEXT NOT NULL);
INSERT INTO schema VALUES(3,'humble');

CREATE TABLE metadata(id INTEGER PRIMARY KEY,metadata_version INTEGER NOT NULL,metadata TEXT NOT NULL);

CREATE TABLE topics(id INTEGER PRIMARY KEY,name TEXT NOT NULL,type TEXT NOT NULL,serialization_format TEXT NOT NULL,offered_qos_profiles TEXT NOT NULL);
INSERT INTO topics VALUES(1, '/camera/top/raw_image',  'sensor_msgs/msg/Image',                       'cdr', replace('- history: 3\n  depth: 0\n  reliability: 1\n  durability: 2\n  deadline:\n    sec: 9223372036\n    nsec: 854775807\n  lifespan:\n    sec: 9223372036\n    nsec: 854775807\n  liveliness: 1\n  liveliness_lease_duration:\n    sec: 9223372036\n    nsec: 854775807\n  avoid_ros_namespace_conventions: false','\n',char(10)));
INSERT INTO topics VALUES(2, '/camera/bot/raw_image',  'sensor_msgs/msg/Image',                       'cdr', replace('- history: 3\n  depth: 0\n  reliability: 1\n  durability: 2\n  deadline:\n    sec: 9223372036\n    nsec: 854775807\n  lifespan:\n    sec: 9223372036\n    nsec: 854775807\n  liveliness: 1\n  liveliness_lease_duration:\n    sec: 9223372036\n    nsec: 854775807\n  avoid_ros_namespace_conventions: false','\n',char(10)));
INSERT INTO topics VALUES(3, '/vision/field_features', 'runswift_interfaces/msg/VisionFieldFeatures', 'cdr', replace('- history: 3\n  depth: 0\n  reliability: 1\n  durability: 2\n  deadline:\n    sec: 9223372036\n    nsec: 854775807\n  lifespan:\n    sec: 9223372036\n    nsec: 854775807\n  liveliness: 1\n  liveliness_lease_duration:\n    sec: 9223372036\n    nsec: 854775807\n  avoid_ros_namespace_conventions: false','\n',char(10)));
INSERT INTO topics VALUES(4, '/motion/odometry',       'runswift_interfaces/msg/MotionOdometry',      'cdr', replace('- history: 3\n  depth: 0\n  reliability: 1\n  durability: 2\n  deadline:\n    sec: 9223372036\n    nsec: 854775807\n  lifespan:\n    sec: 9223372036\n    nsec: 854775807\n  liveliness: 1\n  liveliness_lease_duration:\n    sec: 9223372036\n    nsec: 854775807\n  avoid_ros_namespace_conventions: false','\n',char(10)));
INSERT INTO topics VALUES (5, '/vision/VBalls', 'runswift_interfaces/msg/VisionBalls', 'cdr', replace('- history: 3\n  depth: 0\n  reliability: 1\n  durability: 2\n  deadline:\n    sec: 9223372036\n    nsec: 854775807\n  lifespan:\n    sec: 9223372036\n    nsec: 854775807\n  liveliness: 1\n  liveliness_lease_duration:\n    sec: 9223372036\n    nsec: 854775807\n  avoid_ros_namespace_conventions: false','\n', char(10)));
INSERT INTO topics VALUES (6, '/custom_joint_states', 'sensor_msgs/msg/JointState', 'cdr', replace('- history: 3\n  depth: 0\n  reliability: 1\n  durability: 2\n  deadline:\n    sec: 9223372036\n    nsec: 854775807\n  lifespan:\n    sec: 9223372036\n    nsec: 854775807\n  liveliness: 1\n  liveliness_lease_duration:\n    sec: 9223372036\n    nsec: 854775807\n  avoid_ros_namespace_conventions: false','\n', char(10)));
INSERT INTO topics VALUES (7, '/sensors/angle', 'nao_lola_sensor_msgs/msg/Angle', 'cdr', replace('- history: 3\n  depth: 0\n  reliability: 1\n  durability: 2\n  deadline:\n    sec: 9223372036\n    nsec: 854775807\n  lifespan:\n    sec: 9223372036\n    nsec: 854775807\n  liveliness: 1\n  liveliness_lease_duration:\n    sec: 9223372036\n    nsec: 854775807\n  avoid_ros_namespace_conventions: false','\n', char(10)));
INSERT INTO topics VALUES (8, '/sensors/gyroscope', 'nao_lola_sensor_msgs/msg/Gyroscope', 'cdr', replace('- history: 3\n  depth: 0\n  reliability: 1\n  durability: 2\n  deadline:\n    sec: 9223372036\n    nsec: 854775807\n  lifespan:\n    sec: 9223372036\n    nsec: 854775807\n  liveliness: 1\n  liveliness_lease_duration:\n    sec: 9223372036\n    nsec: 854775807\n  avoid_ros_namespace_conventions: false','\n', char(10)));

-- messages (empty by default as we're going to populate using this program)
CREATE TABLE messages(id INTEGER PRIMARY KEY,topic_id INTEGER NOT NULL,timestamp INTEGER NOT NULL, data BLOB NOT NULL);
CREATE INDEX timestamp_idx ON messages (timestamp ASC);

COMMIT;
-- sample data of deserialized messages for the image topic:
-- sec: 1732956823, nanosec: 456634000, frame_id: default_cam, height: 960, width: 1280, encoding: yuv422_yuy2, isBigEndian: 0, step: 2560
