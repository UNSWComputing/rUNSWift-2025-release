import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float32MultiArray
from nao_lola_command_msgs.msg import JointPositions
from nao_lola_sensor_msgs.msg import FSR, Accelerometer, Gyroscope, JointPositions as SensorJointPositions, JointStiffnesses, JointIndexes

# Mapping from JointIndexes.msg
INDEX_TO_NAME = [
    'HeadYaw', 'HeadPitch',
    'LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw',
    'LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll',
    'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll',
    'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw',
    'LHand', 'RHand'
]

class IsaacSimBridge(Node):
    def __init__(self):
        super().__init__('isaacsim_bridge')
        
        # Declare parameters
        self.declare_parameter('bridge_joint_positions', False)
        self.declare_parameter('bridge_joint_stiffnesses', False)
        
        # Get parameter values
        self.bridge_joint_positions = self.get_parameter('bridge_joint_positions').get_parameter_value().bool_value
        self.bridge_joint_stiffnesses = self.get_parameter('bridge_joint_stiffnesses').get_parameter_value().bool_value
        
        # Joint state bridge
        self.publisher_ = self.create_publisher(JointState, '/effectors/joint_states', 10)
        self.subscription = self.create_subscription(
            JointPositions,
            '/effectors/joint_positions',
            self.joint_positions_callback,
            10)
        
        # Joint states to sensor messages bridge
        if self.bridge_joint_positions:
            self.sensor_joint_positions_publisher = self.create_publisher(SensorJointPositions, '/sensors/joint_positions', 10)
            
        if self.bridge_joint_stiffnesses:
            self.sensor_joint_stiffnesses_publisher = self.create_publisher(JointStiffnesses, '/sensors/joint_stiffnesses', 10)
            
        if self.bridge_joint_positions or self.bridge_joint_stiffnesses:
            self.joint_states_subscription = self.create_subscription(
                JointState,
                '/joint_states',
                self.joint_states_callback,
                10)
        
        # FSR bridge
        self.fsr_publisher = self.create_publisher(FSR, '/sensors/fsr', 10)
        self.fsr_subscription = self.create_subscription(
            Float32MultiArray,
            '/sensors/contact_sensors',
            self.fsr_callback,
            10)
        
        # Accelerometer bridge
        self.accelerometer_publisher = self.create_publisher(Accelerometer, '/sensors/accelerometer', 10)
        self.accelerometer_subscription = self.create_subscription(
            Imu,
            '/imu/accelerometer',
            self.accelerometer_callback,
            10)
        
        # Gyroscope bridge
        self.gyroscope_publisher = self.create_publisher(Gyroscope, '/sensors/gyroscope', 10)
        self.gyroscope_subscription = self.create_subscription(
            Imu,
            '/imu/gyroscope',
            self.gyroscope_callback,
            10)

    def joint_positions_callback(self, msg):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = [INDEX_TO_NAME[i] for i in msg.indexes]
        joint_state.position = list(msg.positions)
        self.publisher_.publish(joint_state)

    def joint_states_callback(self, msg):
        """Convert JointState to sensor joint positions and stiffnesses."""
        # Create a mapping from joint name to index
        name_to_index = {name: i for i, name in enumerate(INDEX_TO_NAME)}
        
        if self.bridge_joint_positions:
            sensor_joint_positions = SensorJointPositions()
            # Initialize with zeros - sensor message has fixed NUMJOINTS-element array
            sensor_joint_positions.positions = [0.0] * JointIndexes.NUMJOINTS
            
            # Map joint positions from JointState to sensor message
            for i, joint_name in enumerate(msg.name):
                if joint_name in name_to_index and i < len(msg.position):
                    joint_index = name_to_index[joint_name]
                    sensor_joint_positions.positions[joint_index] = msg.position[i]
            
            self.sensor_joint_positions_publisher.publish(sensor_joint_positions)
        
        if self.bridge_joint_stiffnesses:
            sensor_joint_stiffnesses = JointStiffnesses()
            # Initialize with default stiffness values (0.8 is a reasonable default)
            sensor_joint_stiffnesses.stiffnesses = [0.8] * JointIndexes.NUMJOINTS
            
            # If effort values are available in JointState, use them to estimate stiffness
            # Otherwise, use the default values
            if msg.effort and len(msg.effort) > 0:
                for i, joint_name in enumerate(msg.name):
                    if joint_name in name_to_index and i < len(msg.effort):
                        joint_index = name_to_index[joint_name]
                        # Simple mapping from effort to stiffness (can be adjusted)
                        # Clamp between 0.0 and 1.0
                        stiffness = max(0.0, min(1.0, abs(msg.effort[i]) / 100.0))
                        sensor_joint_stiffnesses.stiffnesses[joint_index] = stiffness
            
            self.sensor_joint_stiffnesses_publisher.publish(sensor_joint_stiffnesses)

    def fsr_callback(self, msg):
        fsr = FSR()
        # Map Float32MultiArray data to FSR fields
        # Expected order: l_foot_front_left, l_foot_front_right, l_foot_back_left, l_foot_back_right,
        #                 r_foot_front_left, r_foot_front_right, r_foot_back_left, r_foot_back_right
        if len(msg.data) >= 8:
            fsr.l_foot_front_left = msg.data[0]
            fsr.l_foot_front_right = msg.data[1]
            fsr.l_foot_back_left = msg.data[2]
            fsr.l_foot_back_right = msg.data[3]
            fsr.r_foot_front_left = msg.data[4]
            fsr.r_foot_front_right = msg.data[5]
            fsr.r_foot_back_left = msg.data[6]
            fsr.r_foot_back_right = msg.data[7]
        else:
            self.get_logger().warn(f"FSR data array too short: expected 8 values, got {len(msg.data)}")
            return
        
        self.fsr_publisher.publish(fsr)

    def accelerometer_callback(self, msg):
        accelerometer = Accelerometer()
        # Extract linear acceleration from IMU message
        # IMU message has linear_acceleration.x, linear_acceleration.y, linear_acceleration.z
        accelerometer.x = msg.linear_acceleration.x
        accelerometer.y = msg.linear_acceleration.y
        accelerometer.z = msg.linear_acceleration.z
        
        self.accelerometer_publisher.publish(accelerometer)

    def gyroscope_callback(self, msg):
        gyroscope = Gyroscope()
        # Extract angular velocity from IMU message
        # IMU message has angular_velocity.x, angular_velocity.y, angular_velocity.z
        gyroscope.x = msg.angular_velocity.x
        gyroscope.y = msg.angular_velocity.y
        gyroscope.z = msg.angular_velocity.z
        
        self.gyroscope_publisher.publish(gyroscope)

def main(args=None):
    rclpy.init(args=args)
    node = IsaacSimBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
