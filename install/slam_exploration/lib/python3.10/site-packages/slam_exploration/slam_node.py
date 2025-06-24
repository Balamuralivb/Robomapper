#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
import numpy as np

class SlamMonitorNode(Node):
    """
    Monitor node for SLAM operations
    Provides additional functionality for map analysis and robot control
    """
    
    def __init__(self):
        super().__init__('slam_monitor')
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Parameters
        self.declare_parameter('exploration_mode', False)
        self.declare_parameter('auto_explore_speed', 0.2)
        self.declare_parameter('obstacle_distance_threshold', 0.5)
        
        # Variables
        self.current_scan = None
        self.current_map = None
        self.exploration_active = False
        
        # Timer for status updates
        self.status_timer = self.create_timer(5.0, self.status_callback)
        
        self.get_logger().info('SLAM Monitor Node initialized')
    
    def scan_callback(self, msg):
        """Process laser scan data"""
        self.current_scan = msg
        
        # Basic obstacle detection
        min_distance = min([r for r in msg.ranges if r > msg.range_min and r < msg.range_max])
        
        if min_distance < self.get_parameter('obstacle_distance_threshold').value:
            self.get_logger().warn(f'Obstacle detected at {min_distance:.2f}m')
    
    def map_callback(self, msg):
        """Process map updates"""
        self.current_map = msg
        
        # Calculate map statistics
        total_cells = len(msg.data)
        occupied_cells = sum(1 for cell in msg.data if cell > 50)
        free_cells = sum(1 for cell in msg.data if cell >= 0 and cell <= 50)
        unknown_cells = sum(1 for cell in msg.data if cell < 0)
        
        self.get_logger().info(
            f'Map updated - Size: {msg.info.width}x{msg.info.height}, '
            f'Occupied: {occupied_cells}, Free: {free_cells}, Unknown: {unknown_cells}'
        )
    
    def status_callback(self):
        """Periodic status updates"""
        if self.current_scan and self.current_map:
            self.get_logger().info(
                f'SLAM Status - Scan ranges: {len(self.current_scan.ranges)}, '
                f'Map resolution: {self.current_map.info.resolution:.3f}m/cell'
            )
    
    def emergency_stop(self):
        """Emergency stop function"""
        stop_msg = Twist()
        self.cmd_pub.publish(stop_msg)
        self.get_logger().warn('Emergency stop activated!')

def main(args=None):
    rclpy.init(args=args)
    node = SlamMonitorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.emergency_stop()
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()