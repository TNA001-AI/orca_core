#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import JointState
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
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            qos_profile
        )

        # Joint name mapping
        self.joint_mapping = {
            'right_thumb_mcp': 'thumb_mcp', 'right_thumb_abd': 'thumb_abd',
            'right_thumb_pip': 'thumb_pip', 'right_thumb_dip': 'thumb_dip',
            'right_index_mcp': 'index_mcp', 'right_index_pip': 'index_pip',
            'right_index_abd': 'index_abd', 'right_middle_mcp': 'middle_mcp',
            'right_middle_pip': 'middle_pip', 'right_ring_mcp': 'ring_mcp',
            'right_ring_pip': 'ring_pip', 'right_ring_abd': 'ring_abd',
            'right_pinky_mcp': 'pinky_mcp', 'right_pinky_pip': 'pinky_pip',
            'right_pinky_abd': 'pinky_abd', 'right_wrist': 'wrist'
        }

        # Joint range of motion limits (in degrees)
        self.joint_roms = {
            'thumb_mcp': [-50, 50], 'thumb_abd': [-20, 42],
            'thumb_pip': [-12, 108], 'thumb_dip': [-20, 112],
            'index_mcp': [-20, 95], 'index_pip': [-20, 108],
            'index_abd': [-37, 37], 'middle_mcp': [-20, 91],
            'middle_pip': [-20, 107], 'ring_mcp': [-20, 91],
            'ring_pip': [-20, 107], 'ring_abd': [-37, 37],
            'pinky_mcp': [-20, 98], 'pinky_pip': [-20, 108],
            'pinky_abd': [-37, 37], 'wrist': [-50, 30]
        }
        self.get_logger().info('Orca hand controller node initialized')

    def clamp_joint_value(self, joint_name, value):
        if joint_name in self.joint_roms:
            min_val, max_val = self.joint_roms[joint_name]
            return max(min_val, min(max_val, value))
        return value

    def joint_state_callback(self, msg):
        self.get_logger().debug(f"Received joint_states: names={msg.name}, positions={msg.position}")
        joint_positions = {}
        for name, position in zip(msg.name, msg.position):
            if name in self.joint_mapping:
                orca_name = self.joint_mapping[name]
                position_deg = math.degrees(float(position))
                position_deg = self.clamp_joint_value(orca_name, position_deg)
                joint_positions[orca_name] = position_deg
        if joint_positions:
            try:
                self.hand.set_joint_pos(joint_positions)
                self.get_logger().debug(f"Sent joint positions to hand: {joint_positions}")
            except Exception as e:
                self.get_logger().error(f"set_joint_pos error: {e}")

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
