#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from mavros_msgs.msg import RCIn
from std_msgs.msg import String
from custom_interfaces.msg import TargetPosePolar


class RCChannelReader(Node):
    """ROS2 node for reading RC channel inputs and publishing control commands."""

    def __init__(self):
        super().__init__('rc_channel_reader')

        # Create QoS profiles
        qos_best_effort = self._create_qos_profile(QoSReliabilityPolicy.BEST_EFFORT)
        qos_reliable = self._create_qos_profile(QoSReliabilityPolicy.RELIABLE)

        # Declare parameters
        self._declare_parameters()
        self._load_parameters()

        # Initialize state variables
        self.active = False
        self.last_active = False
        self.servo_1_open = False
        self.servo_2_open = False
        self.pitch = None
        self.roll = None
        self.yaw = None
        self.throttle = None

        # Create subscriptions
        self.rc_sub = self.create_subscription(
            RCIn,
            '/mavros/rc/in',
            self.rc_callback,
            qos_best_effort
        )

        self.start_sub = self.create_subscription(
            String,
            '/controller_activation',
            self.start_callback,
            qos_best_effort
        )

        # Create publishers
        self.target_pub = self.create_publisher(
            TargetPosePolar,
            '/goal_pose_polar',
            qos_reliable
        )

        self.servo_pub = self.create_publisher(
            String,
            '/servo_topic',
            qos_reliable
        )

        self.activation_pub = self.create_publisher(
            String,
            '/approach_activation',
            qos_reliable
        )

        self.get_logger().info("RC Channel Reader started")

    @staticmethod
    def _create_qos_profile(reliability_policy):
        """Create a QoS profile with specified reliability policy."""
        return QoSProfile(
            reliability=reliability_policy,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

    def _declare_parameters(self):
        """Declare all ROS2 parameters."""
        self.declare_parameter('v_r_max', 1.5)
        self.declare_parameter('v_theta_max', 2.0)
        self.declare_parameter('v_z_max', 0.5)
        self.declare_parameter('yaw_max', 3.14159 / 4)
        self.declare_parameter('talk', True)
        self.declare_parameter('servo_1_channel_controller', 8)
        self.declare_parameter('servo_2_channel_controller', 9)

    def _load_parameters(self):
        """Load parameter values from ROS2 parameter server."""
        self.v_r_max = self.get_parameter('v_r_max').value
        self.v_theta_max = self.get_parameter('v_theta_max').value
        self.v_z_max = self.get_parameter('v_z_max').value
        self.yaw_max = self.get_parameter('yaw_max').value
        self.talk = self.get_parameter('talk').value
        self.servo_1_channel = self.get_parameter('servo_1_channel_controller').value
        self.servo_2_channel = self.get_parameter('servo_2_channel_controller').value  
            

    def publish_target(self):
        """Publish target pose in polar coordinates."""
        msg = TargetPosePolar()
        msg.v_r = self.v_r_max * self.pitch
        msg.v_theta = -self.v_theta_max * self.roll
        msg.v_z = self.v_z_max * self.throttle
        msg.yaw_rate = self.yaw_max * self.yaw
        msg.relative = True
        self.target_pub.publish(msg)

    def start_callback(self, msg):
        """Handle start/stop commands from controller activation topic."""
        if msg.data == "start":
            self.get_logger().info("Controller Activated")
            self.active = True
        elif msg.data == "stop":
            self.get_logger().info("Controller Deactivated")
            self.active = False

    def handle_polar_controls(self, msg):
        """Process polar control inputs from RC channels."""
        if len(msg.channels) < 4:
            self.get_logger().warn(f"Insufficient RC channels: {len(msg.channels)}")
            return

        # Extract raw PWM values
        roll_raw = msg.channels[0]      # Channel 1
        pitch_raw = msg.channels[1]     # Channel 2
        throttle_raw = msg.channels[2]  # Channel 3
        yaw_raw = msg.channels[3]       # Channel 4
        activation = msg.channels[6]

        # Update activation state based on channel 7
        if activation > 1700:
            self.active = True
        elif activation < 1300:
            self.active = False

        # Convert PWM values to normalized range [-1, 1]
        self.roll = self.pwm_to_normalized(roll_raw)
        self.pitch = self.pwm_to_normalized(pitch_raw)
        self.throttle = self.pwm_to_normalized(throttle_raw)
        self.yaw = self.pwm_to_normalized(yaw_raw)

        if self.talk:
            self.get_logger().info(
                f"Roll: {self.roll:.3f}, Pitch: {self.pitch:.3f}, Throttle: {self.throttle:.3f} "
                f"(Raw: {roll_raw}, {pitch_raw}, {throttle_raw}) "
                f"(Activation PWM: {activation})"
            )

        # Handle activation state transitions
        if self.active and not self.last_active:
            self.activate_polar_system()
        elif not self.active and self.last_active:
            self.deactivate_polar_system()

        self.last_active = self.active

        # Only publish target if active
        if self.active:
            self.publish_target()

    def publish_open_servo(self, servo_number):
        """Publish servo open command."""
        # TODO: Implement servo topic publishing with correct command format
        pass

    def handle_servo_controls(self, msg):
        """Process servo control inputs from RC channels."""
        if self.servo_1_channel - 1 < len(msg.channels):
            if msg.channels[self.servo_1_channel - 1] > 1700 and not self.servo_1_open:
                self.servo_1_open = True
                self.publish_open_servo(1)

        if self.servo_2_channel - 1 < len(msg.channels):
            if msg.channels[self.servo_2_channel - 1] > 1700 and not self.servo_2_open:
                self.servo_2_open = True
                self.publish_open_servo(2)
      
    def rc_callback(self, msg):
        """
        Process incoming RC channel data.

        Standard RC channel mapping:
        - Channel 1: Roll (aileron)
        - Channel 2: Pitch (elevator)
        - Channel 3: Throttle
        - Channel 4: Yaw (rudder)
        """
        self.handle_polar_controls(msg)
        self.handle_servo_controls(msg)
        
    def deactivate_polar_system(self):
        """Publish stop command to approach system."""
        msg = String()
        msg.data = 'stop'
        self.activation_pub.publish(msg)

    def activate_polar_system(self):
        """Publish start command to approach system."""
        msg = String()
        msg.data = 'start'
        self.activation_pub.publish(msg)
      
    def pwm_to_normalized(self, pwm_value, center=1500, deadband=50):
        """
        Convert PWM value to normalized range [-1, 1].

        Args:
            pwm_value: Raw PWM value (typically 1000-2000)
            center: Center PWM value (default 1500)
            deadband: Deadband around center value (default 50)

        Returns:
            Normalized value in range [-1, 1]
        """
        if abs(pwm_value - center) < deadband:
            return 0.0
        return (pwm_value - center) / 500.0  

def main(args=None):
    """Main entry point for the RC Channel Reader node."""
    rclpy.init(args=args)
    node = RCChannelReader()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()