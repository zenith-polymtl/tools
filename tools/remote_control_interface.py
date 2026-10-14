#!/usr/bin/env python3
"""
Remote Control Interface Node
Turns the switches of the pilot's RC transmitter (/mavros/rc/in) into mission actions:
a servo position, or a Bool published on a topic of tools.topics.

Everything is set in config/<mission>.yaml, under the node's name:

    remote_control_interface:
      ros__parameters:
        threshold_low: 1300            # PWM below = LOW, above threshold_high = HIGH, else MIDDLE
        threshold_high: 1700
        switches: [camera, action, trigger]
        camera:
          channel: 12                  # index in RCIn.channels (0 = channel 1)
          servo: 13                    # servo_num sent to the ServoState service
          pwm: [1850, 1000, 700]       # servo PWM for LOW, MIDDLE, HIGH
        action:
          channel: 6
          events: [AUTO_SHOOT, SHOOT, TAKE_PICTURE]   # tools.topics constants for LOW, MIDDLE, HIGH ('' = nothing)
          trigger: trigger             # optional: publish only when switch `trigger` goes HIGH
        trigger:
          channel: 7

A switch with `events` and no `trigger` publishes Bool(True) when it moves to a position.
A switch with a `trigger` publishes the event of its current position when the trigger
switch moves to `trigger_position` (HIGH by default). The first reading after startup sets
the servos to the switch position and publishes nothing.

Guide for the team, in French: doc/remote_control_interface.md
"""

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Bool
from mavros_msgs.msg import RCIn
from custom_interfaces.srv import ServoState

from tools import topics

RC_IN = '/mavros/rc/in'
POSITIONS = ('LOW', 'MIDDLE', 'HIGH')


class RemoteControlInterface(Node):
    def __init__(self):
        super().__init__('remote_control_interface')

        self.declare_parameter('threshold_low', 1300)
        self.declare_parameter('threshold_high', 1700)
        self.declare_optional('switches')
        self.low_threshold = self.get_parameter('threshold_low').value
        self.high_threshold = self.get_parameter('threshold_high').value

        self.switches = self.load_switches()
        self.last_position = {name: None for name in self.switches}

        qos_be = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        qos_re = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, depth=10)

        event_topics = {t for s in self.switches.values() for t in s['events'].values() if t}
        self.event_pubs = {t: self.create_publisher(Bool, t, qos_re) for t in event_topics}

        self.servo_cli = None
        if any(s['servo'] >= 0 for s in self.switches.values()):
            self.servo_cli = self.create_client(ServoState, topics.PAYLOAD_SET_STATE)

        self.rc_sub = self.create_subscription(RCIn, RC_IN, self.rc_callback, qos_be)

        if self.switches:
            self.get_logger().info(f'Interface manette démarrée : {", ".join(self.switches)}')
        else:
            self.get_logger().warn('Interface manette démarrée sans interrupteur : rien dans switches')

    def load_switches(self):
        """Reads and checks the switch table from the parameters. Raises on a bad config."""
        names = self.get_parameter('switches').value or []
        switches = {}
        for name in names:
            p = f'{name}.'
            self.declare_optional(p + 'channel')
            self.declare_parameter(p + 'servo', -1)
            self.declare_optional(p + 'pwm')
            self.declare_optional(p + 'events')
            self.declare_parameter(p + 'trigger', '')
            self.declare_parameter(p + 'trigger_position', 'HIGH')

            channel = self.get_parameter(p + 'channel').value
            if channel is None:
                raise ValueError(f'interrupteur {name} : channel manquant')
            servo = self.get_parameter(p + 'servo').value
            pwm = list(self.get_parameter(p + 'pwm').value or [])
            if servo >= 0 and len(pwm) != 3:
                raise ValueError(f'interrupteur {name} : pwm doit donner 3 valeurs (LOW, MIDDLE, HIGH)')

            events = list(self.get_parameter(p + 'events').value or ['', '', ''])
            if len(events) != 3:
                raise ValueError(f'interrupteur {name} : events doit donner 3 valeurs (LOW, MIDDLE, HIGH)')
            events = [self.resolve_topic(name, e) for e in events]

            trigger_position = self.get_parameter(p + 'trigger_position').value
            if trigger_position not in POSITIONS:
                raise ValueError(f'interrupteur {name} : trigger_position doit être LOW, MIDDLE ou HIGH')

            switches[name] = {
                'channel': channel,
                'servo': servo,
                'pwm': dict(zip(POSITIONS, pwm)),
                'events': dict(zip(POSITIONS, events)),
                'trigger': self.get_parameter(p + 'trigger').value,
                'trigger_position': trigger_position,
            }

        for name, s in switches.items():
            if s['trigger'] and s['trigger'] not in switches:
                raise ValueError(f'interrupteur {name} : trigger {s["trigger"]} absent de switches')
            if s['trigger'] and not any(s['events'].values()):
                raise ValueError(f'interrupteur {name} : trigger sans events')
        return switches

    def declare_optional(self, name):
        """Parameter with no default and no fixed type: None when the config does not give it."""
        self.declare_parameter(name, None, ParameterDescriptor(dynamic_typing=True))

    def resolve_topic(self, switch, constant):
        """'' stays '', a tools.topics constant name becomes its topic."""
        if not constant:
            return ''
        topic = getattr(topics, constant, None)
        if not isinstance(topic, str) or not topic.startswith('/'):
            raise ValueError(f'interrupteur {switch} : {constant} n\'est pas une constante de tools.topics')
        return topic

    def rc_callback(self, msg):
        # All positions are updated before any action, so a trigger reads its selector's new position
        changes = []
        for name, s in self.switches.items():
            if s['channel'] >= len(msg.channels) or msg.channels[s['channel']] == 0:
                continue   # channel absent or no signal on it
            position = self.get_switch_state(msg.channels[s['channel']])
            previous = self.last_position[name]
            if position != previous:
                self.last_position[name] = position
                changes.append((name, position, previous is None))
        for name, position, first in changes:
            self.handle_switch_change(name, position, first)

    def get_switch_state(self, pwm):
        """Maps PWM values to discrete states."""
        if pwm < self.low_threshold:
            return 'LOW'
        elif pwm <= self.high_threshold:
            return 'MIDDLE'
        return 'HIGH'

    def handle_switch_change(self, name, position, first):
        s = self.switches[name]
        if first:
            self.get_logger().info(f'{name} : position initiale {position}')
        else:
            self.get_logger().info(f'{name} : {position}')

        if s['servo'] >= 0:
            self.send_servo(name, position)
        if first:
            return

        if s['events'][position] and not s['trigger']:
            self.publish_event(name, s['events'][position])

        # Switches gated by this one fire the event of their current position
        for other, o in self.switches.items():
            if o['trigger'] == name and position == o['trigger_position']:
                selected = self.last_position[other]
                if selected is None or not o['events'][selected]:
                    self.get_logger().warn(f'{name} déclenché, mais rien à faire pour {other} en {selected}')
                    continue
                self.publish_event(other, o['events'][selected])

    def publish_event(self, name, topic):
        self.get_logger().info(f'{name} : publication sur {topic}')
        self.event_pubs[topic].publish(Bool(data=True))

    def send_servo(self, name, position):
        if not self.servo_cli.service_is_ready():
            self.get_logger().warn(f'{name} : service {topics.PAYLOAD_SET_STATE} absent, servo non commandé')
            return
        request = ServoState.Request()
        request.servo_num = self.switches[name]['servo']
        request.pwm = self.switches[name]['pwm'][position]
        self.get_logger().info(f'{name} : servo {request.servo_num} à {request.pwm}')
        future = self.servo_cli.call_async(request)
        future.add_done_callback(lambda fut: self.servo_response_callback(fut, name))

    def servo_response_callback(self, future, name):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f'{name} : échec de l\'appel au servo : {e}')
            return
        if not response.success:
            self.get_logger().warn(f'{name} : servo refusé : {response.message}')


def main(args=None):
    rclpy.init(args=args)
    try:
        node = RemoteControlInterface()
    except ValueError as e:
        rclpy.logging.get_logger('remote_control_interface').error(f'Config invalide : {e}')
        rclpy.shutdown()
        raise SystemExit(1)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
