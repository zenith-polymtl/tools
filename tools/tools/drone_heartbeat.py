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
from rclpy.qos import QoSProfile, ReliabilityPolicy
from mavros_msgs.msg import State
from zed_msgs.msg import Heartbeat


class DroneHeartbeat(Node):
    """
    A node that publishes heartbeat messages at a regular interval.
    """

    def __init__(self):
        super().__init__('drone_heartbeat')
        
        # Define a reliable QoS profile
        reliable_qos = QoSProfile(depth=10)
        reliable_qos.reliability = ReliabilityPolicy.RELIABLE
        
        # Declare parameters
        self.declare_parameter('heartbeat_rate', 1.0)  # Hz
        self.declare_parameter('timeout_threshold', 2.0) # Seconds before we consider a node "dead"
        self.declare_parameter('topic_name', 'drone/heartbeat')
        
        # Get parameters
        heartbeat_rate = self.get_parameter('heartbeat_rate').value
        self.timeout_threshold = self.get_parameter('timeout_threshold').value
        topic_name = self.get_parameter('topic_name').value
        
        # Create publisher
        self.hearthbeath_publisher_ = self.create_publisher(DroneHealth, topic_name, reliable_qos)
        self.mavros_sub = self.create_subscription(State, '/mavros/state', self.mavros_callback, 10)
        self.zed_sub = self.create_subscription(Heartbeat, '/zed/zed_node/status/heartbeat', self.zed_callback, 10)

        self.mavros_up = False
        self.zed_up = False
        
        self.last_mavros_time = self.get_clock().now()
        self.last_zed_time = self.get_clock().now()

        # Create timer
        timer_period = 1.0 / heartbeat_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.heartbeat_count = 0
        self.get_logger().info(f'GCS Heartbeat node started - publishing at {heartbeat_rate} Hz on {topic_name}')

    def mavros_callback(self, msg:State):
        """Update the timestamp whenever a message is received."""
        if msg.connected:
            self.last_mavros_time = self.get_clock().now()

    
    def zed_callback(self, msg:Heartbeat):
        """Update the timestamp whenever a message is received."""
        self.last_zed_time = self.get_clock().now()    
    
    
    def check_health(self):
        """Calculates health based on time elapsed since the last heartbeat."""
        now = self.get_clock().now()
        
        # Calculate time since last messages in seconds
        mavros_delta = (now - self.last_mavros_time).nanoseconds / 1e9
        zed_delta = (now - self.last_zed_time).nanoseconds / 1e9
        
        # If time since last message is within threshold, component is healthy
        self.mavros_up = mavros_delta < self.timeout_threshold
        self.zed_up = zed_delta < self.timeout_threshold

    def timer_callback(self):
        """Callback function for the timer - publishes heartbeat message."""
        self.check_health()
        
        msg = DroneHealth()
        msg.zed_healthy = self.zed_up
        msg.mavros_healthy = self.mavros_up
        
        self.hearthbeath_publisher_.publish(msg)
        self.heartbeat_count += 1
        
        # Log every 10th heartbeat to avoid spam
        if self.heartbeat_count % 10 == 0:
            status = "OK" if (self.mavros_up and self.zed_up) else "DEGRADED"
            self.get_logger().info(f'[{status}] MAVROS: {"Live" if self.mavros_up else "LOST"}, '
                                   f'ZED: {"Live" if self.zed_up else "LOST"}')


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
