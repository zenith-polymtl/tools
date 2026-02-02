#!/usr/bin/env python3
"""
GCS Heartbeat Node
Publishes periodic heartbeat messages to indicate the ground control station is alive.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from builtin_interfaces.msg import Time



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
        
        # Create timer
        timer_period = 1.0 / heartbeat_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.heartbeat_count = 0
        
        self.get_logger().info(f'GCS Heartbeat node started - publishing at {heartbeat_rate} Hz on {topic_name}')

    def timer_callback(self):
        """
        Callback function for the timer - publishes heartbeat message.
        """
        
        msg = Bool()
        msg.data = True
        
        self.publisher_.publish(msg)
        self.heartbeat_count += 1
        
        # Log every 10th heartbeat to avoid spam
        if self.heartbeat_count % 10 == 0:
            self.get_logger().debug(f'Published heartbeat #{self.heartbeat_count}')


def main(args=None):
    rclpy.init(args=args)
    
    heartbeat_node = GCSHeartbeat()
    
    try:
        rclpy.spin(heartbeat_node)
    except KeyboardInterrupt:
        pass
    finally:
        heartbeat_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
