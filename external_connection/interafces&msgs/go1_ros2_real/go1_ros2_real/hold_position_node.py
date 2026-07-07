#!/usr/bin/env python3
"""
Hold Position Node for Unitree Go1 Robot
This node maintains the robot's legs in a fixed position by publishing
low-level motor commands to the /unitree_go1/low_cmd topic.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from unitree_legged_msgs.msg import LowCmd, MotorCmd, BmsCmd
from sensor_msgs.msg import JointState

# Motor modes and constants
PMSM = 0x0A        # Position-Velocity-Torque mode
VelStopF = 16000.0  # Velocity stop constant

# Joint name mapping to motor index
JOINT_NAMES = [
    'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
    'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
    'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
    'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint'
]


class HoldPositionNode(Node):
    """ROS2 Node to hold Go1 robot legs at a fixed position"""

    def __init__(self):
        super().__init__('hold_position_node')
        
        # Declare parameters
        self.declare_parameter('kp_default', 500.0)
        self.declare_parameter('kd_default', 1.0)
        self.declare_parameter('publish_rate', 100.0)  # Hz
        self.declare_parameter('topic', '/unitree_go1/low_cmd')
        self.declare_parameter('joint_state_topic', '/joint_states')
        
        # Get parameters
        self.kp = self.get_parameter('kp_default').get_parameter_value().double_value
        self.kd = self.get_parameter('kd_default').get_parameter_value().double_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.topic_name = self.get_parameter('topic').get_parameter_value().string_value
        self.joint_state_topic = self.get_parameter('joint_state_topic').get_parameter_value().string_value
        
        self.get_logger().info(f'Hold Position Node starting')
        self.get_logger().info(f'Publishing to: {self.topic_name}')
        self.get_logger().info(f'Listening to: {self.joint_state_topic}')
        self.get_logger().info(f'Gains - kp: {self.kp}, kd: {self.kd}')
        
        # Current joint positions (will be updated from /joint_states)
        self.current_joint_positions = [0.0] * 12
        self.hold_positions = [0.0] * 12  # Fixed hold positions
        self.received_joint_state = False
        
        # Create mapping from joint names to motor indices
        self.joint_name_to_index = {name: i for i, name in enumerate(JOINT_NAMES)}
        
        # Initialize low_cmd message
        self.low_cmd = LowCmd()
        self.initialize_low_cmd()
        
        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.joint_state_callback,
            10
        )
        
        # Create publisher for low_cmd
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )
        self.low_cmd_pub = self.create_publisher(
            LowCmd,
            self.topic_name,
            qos_profile
        )
        
        # Create timer for publishing at fixed rate
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_command)
        
        self.get_logger().info('Hold Position Node initialized. Waiting for joint state...')
    
    def initialize_low_cmd(self):
        """Initialize the low_cmd message structure"""
        self.low_cmd.head = [0xFE, 0xEF]     # Standard header
        self.low_cmd.level_flag = 0x01       # Normal level
        self.low_cmd.frame_reserve = 0       # Reserved
        self.low_cmd.sn = [0, 0]
        self.low_cmd.version = [1, 0]
        self.low_cmd.band_width = int(self.publish_rate)
        
        # Initialize 12 motor commands
        self.low_cmd.motor_cmd = [MotorCmd() for _ in range(12)]
        for i in range(12):
            self.low_cmd.motor_cmd[i].mode = PMSM
            self.low_cmd.motor_cmd[i].q = 0.0  # Will be updated from joint_state
            self.low_cmd.motor_cmd[i].dq = 0.0
            self.low_cmd.motor_cmd[i].tau = 0.0
            self.low_cmd.motor_cmd[i].kp = self.kp
            self.low_cmd.motor_cmd[i].kd = self.kd
        
        # Initialize BMS command
        self.low_cmd.bms = BmsCmd()
        self.low_cmd.bms.off = 0
        self.low_cmd.bms.reserve = [0, 0, 0]
        
        # CRC (simplified)
        self.low_cmd.crc = 0
    
    def joint_state_callback(self, msg):
        """Callback to receive and store current joint positions"""
        if not self.received_joint_state:
            self.get_logger().info('Received first joint state message. Robot position locked!')
            self.received_joint_state = True
        
        # Map joint positions from JointState message to motor indices
        for i, joint_name in enumerate(msg.name):
            if joint_name in self.joint_name_to_index:
                motor_idx = self.joint_name_to_index[joint_name]
                if i < len(msg.position):
                    self.hold_positions[motor_idx] = float(msg.position[i])
                    self.current_joint_positions[motor_idx] = float(msg.position[i])
    
    def publish_command(self):
        """Publish the hold position command"""
        # Only publish after receiving first joint state message
        if not self.received_joint_state:
            return
        
        # Update motor commands with current hold positions
        for i in range(12):
            self.low_cmd.motor_cmd[i].q = float(self.hold_positions[i])
        
        self.low_cmd_pub.publish(self.low_cmd)


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    try:
        node = HoldPositionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
