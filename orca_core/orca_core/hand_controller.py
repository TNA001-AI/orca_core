#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from orca_core.orca_core.core import OrcaHand
import os
from ament_index_python.packages import get_package_share_directory
import math

class OrcaHandController(Node):
    def __init__(self):
        super().__init__('orca_hand_controller')

        # Initialize the hand
        package_share_dir = get_package_share_directory('orca_core')
        model_path = os.path.join(package_share_dir, 'models', 'orcahand_v1_right')
        self.hand = OrcaHand(model_path)
        status = self.hand.connect()
        if not status[0]:
            self.get_logger().error('Failed to connect to the hand')
            return
        self.get_logger().info('Successfully connected to Orca hand')

        # Create QoS profile with Best Effort reliability
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to joint states (excluding wrist)
        self.joint_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            qos_profile
        )
        
        # Subscribe to wrist angle from gello publisher
        self.wrist_subscription = self.create_subscription(
            Float32,
            '/gello/wrist_angle',
            self.wrist_angle_callback,
            qos_profile
        )
        # Create publisher for joint angles
        self.joint_angles_publisher = self.create_publisher(
            JointState,
            '/orca_hand/act_joint_states',
            qos_profile
        )
        
        # Create publisher for real joint states
        self.real_joint_states_publisher = self.create_publisher(
            JointState,
            '/orca_hand/obs_joint_states',
            qos_profile
        )

        # Joint name mapping (excluding wrist)
        self.joint_mapping = {
            'right_thumb_mcp': 'thumb_mcp', 'right_thumb_abd': 'thumb_abd',
            'right_thumb_pip': 'thumb_pip', 'right_thumb_dip': 'thumb_dip',
            'right_index_mcp': 'index_mcp', 'right_index_pip': 'index_pip',
            'right_index_abd': 'index_abd', 'right_middle_mcp': 'middle_mcp',
            'right_middle_pip': 'middle_pip', 'right_ring_mcp': 'ring_mcp',
            'right_ring_pip': 'ring_pip', 'right_ring_abd': 'ring_abd',
            'right_pinky_mcp': 'pinky_mcp', 'right_pinky_pip': 'pinky_pip',
            'right_pinky_abd': 'pinky_abd'
        }
        

        # Joint range of motion limits (in degrees) - excluding wrist
        self.joint_roms = {
            'thumb_mcp': [-50, 50], 'thumb_abd': [-20, 42],
            'thumb_pip': [-12, 108], 'thumb_dip': [-20, 112],
            'index_mcp': [-20, 95], 'index_pip': [-20, 108],
            'index_abd': [-37, 37], 'middle_mcp': [-20, 91],
            'middle_pip': [-20, 107], 'ring_mcp': [-20, 91],
            'ring_pip': [-20, 107], 'ring_abd': [-37, 37],
            'pinky_mcp': [-20, 98], 'pinky_pip': [-20, 108],
            'pinky_abd': [-37, 37]
        }

        
        # Wrist range of motion limits (in degrees)
        self.wrist_rom = [-24, 24]  # Adjust this range as needed
        
        # Store current joint positions
        self.current_joint_positions = {}
        self.current_wrist_position = 0
        
        # Create timer to send all joint positions together (50Hz)
        self.timer = self.create_timer(0.02, self.send_all_joint_positions)
        
        self.get_logger().info('Orca hand controller node initialized')

    def clamp_joint_value(self, joint_name, value):
        return value
    
    def clamp_wrist_value(self, value):
        min_val, max_val = self.wrist_rom
        return max(min_val, min(max_val, value))

    def joint_state_callback(self, msg):
        self.get_logger().debug(f"Received joint_states: names={msg.name}, positions={msg.position}")
        for name, position in zip(msg.name, msg.position):
            # Skip wrist joint as it's handled separately
            if name == 'right_wrist':
                continue
            if name in self.joint_mapping:
                orca_name = self.joint_mapping[name]
                position_deg = math.degrees(float(position))
                position_deg = self.clamp_joint_value(orca_name, position_deg)
                self.current_joint_positions[orca_name] = position_deg
    
    def wrist_angle_callback(self, msg):
        """Handle wrist angle from gello publisher"""
        try:
            # Convert from radians to degrees and clamp to range
            wrist_angle_deg = math.degrees(float(msg.data))
            wrist_angle_deg = self.clamp_wrist_value(wrist_angle_deg)
            self.current_wrist_position = wrist_angle_deg
            self.get_logger().debug(f"Received wrist position: {wrist_angle_deg}")
        except Exception as e:
            self.get_logger().error(f"wrist_angle_callback error: {e}")
    

    def send_all_joint_positions(self):
        """Send all joint positions together via timer"""
        try:
            # Combine joint positions and wrist position
            act_joint_states = self.current_joint_positions.copy()
            act_joint_states['wrist'] = self.current_wrist_position
            
            self.hand.set_joint_pos(act_joint_states)
            
            # Publish joint angles with timestamp
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.header.frame_id = 'orca_hand'
            joint_state_msg.name = list(act_joint_states.keys())
            joint_state_msg.position = [math.radians(angle) for angle in act_joint_states.values()]
            
            self.joint_angles_publisher.publish(joint_state_msg)
            self.get_logger().debug(f"Sent all joint positions: {act_joint_states}")
            
            # Read and publish real joint states
            self.publish_real_joint_states()

        except Exception as e:
            self.get_logger().error(f"send_all_joint_positions error: {e}")

    def publish_real_joint_states(self):
        """Read and publish the hand's real joint states"""
        try:
            # Get real joint positions from the hand
            real_joint_positions = self.hand.get_joint_pos(as_list=False)
            
            if real_joint_positions:
                # Create JointState message for real joint states
                real_joint_state_msg = JointState()
                real_joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                real_joint_state_msg.header.frame_id = 'orca_hand_real'
                
                # Filter out None values and convert to radians
                valid_joints = {name: pos for name, pos in real_joint_positions.items() if pos is not None}
                
                real_joint_state_msg.name = list(valid_joints.keys())
                real_joint_state_msg.position = [math.radians(angle) for angle in valid_joints.values()]
                
                self.real_joint_states_publisher.publish(real_joint_state_msg)
                self.get_logger().debug(f"Published real joint states: {valid_joints}")
                
        except Exception as e:
            self.get_logger().error(f"publish_real_joint_states error: {e}")

    def __del__(self):
        if hasattr(self, 'hand'):
            neutral = {jid: 0 for jid in self.hand.joint_ids}
            self.hand.set_joint_pos(neutral)


def main(args=None):
    rclpy.init(args=args)
    controller = OrcaHandController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
