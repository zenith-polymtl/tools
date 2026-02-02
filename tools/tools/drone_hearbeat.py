#!/usr/bin/env python3
"""
GCS Heartbeat Node
Publishes periodic heartbeat messages to indicate the ground control station is alive.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from builtin_interfaces.msg import Time
from custom_interfaces.msg import DroneHealth


class DroneHeartbeat(Node):
    """
    A node that publishes heartbeat messages at a regular interval.
    """

    def __init__(self):
        super().__init__('drone_heartbeat')
        
        # Declare parameters
        self.declare_parameter('heartbeat_rate', 1.0)  # Hz
        self.declare_parameter('topic_name', 'drone/heartbeat')
        
        # Get parameters
        heartbeat_rate = self.get_parameter('heartbeat_rate').value
        topic_name = self.get_parameter('topic_name').value
        
        # Create publisher
        self.publisher_ = self.create_publisher(Bool, topic_name, 10)
        self.mavros_sub = self.create_subscription(Bool, 'mavros/heartbeat', self.mavros_callback, 10)
        self.zed_sub = self.create_subscription(Bool, 'zed/heartbeat', self.zed_callback, 10)

        self.mavros_up = False
        self.zed_up = False

        # Create timer
        timer_period = 1.0 / heartbeat_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.heartbeat_count = 0
        
        self.get_logger().info(f'GCS Heartbeat node started - publishing at {heartbeat_rate} Hz on {topic_name}')

    def mavros_callback(self, msg):
        """
        Callback function for MAVROS heartbeat subscription.
        """
        self.get_logger().debug('Received MAVROS heartbeat message.')
        if msg.data:
            pass
    
    def zed_callback(self, msg):
        """
        Callback function for ZED heartbeat subscription.
        """
        self.get_logger().debug('Received ZED heartbeat message.')
        if msg.data:
            pass

    def timer_callback(self):
        """
        Callback function for the timer - publishes heartbeat message.
        """
        
        msg = DroneHealth()
        msg.zed_healthy = self.zed_up
        msg.mavros_healthy = self.mavros_up
        
        self.publisher_.publish(msg)
        self.heartbeat_count += 1
        
        # Log every 10th heartbeat to avoid spam
        if self.heartbeat_count % 10 == 0:
            self.get_logger().debug(f'Published heartbeat #{self.heartbeat_count}')


def main(args=None):
    rclpy.init(args=args)
    
    heartbeat_node = DroneHeartbeat()
    
    try:
        rclpy.spin(heartbeat_node)
    except KeyboardInterrupt:
        pass
    finally:
        heartbeat_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
