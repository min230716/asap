#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import platform
import time

# ============ Mock class for non-Pi environments ============
class MockPigpio:
    def __init__(self):
        self.counter = 0

    def read_encoder(self):
        self.counter += 1
        return self.counter

# ============ Real pigpio class for Raspberry Pi ============
class RealPigpio:
    def __init__(self, pin_a=17, pin_b=27):
        import pigpio
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpio daemon not running. Run: sudo pigpiod")
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.counter = 0
        self.last_a = 0
        self.pi.set_mode(self.pin_a, pigpio.INPUT)
        self.pi.set_pull_up_down(self.pin_a, pigpio.PUD_UP)
        self.pi.set_mode(self.pin_b, pigpio.INPUT)
        self.pi.set_pull_up_down(self.pin_b, pigpio.PUD_UP)
        self.callback_a = self.pi.callback(self.pin_a, pigpio.EITHER_EDGE, self._pulse)

    def _pulse(self, gpio, level, tick):
        a = self.pi.read(self.pin_a)
        b = self.pi.read(self.pin_b)
        if a != self.last_a:
            if b != a:
                self.counter += 1
            else:
                self.counter -= 1
            self.last_a = a

    def read_encoder(self):
        return self.counter

# ============ ROS2 Node ============
class EncoderPublisher(Node):
    def __init__(self):
        super().__init__('encoder_publisher')
        if ("rasp" in platform.uname().machine.lower()) or ("aarch64" in platform.uname().machine.lower()):
            self.get_logger().info("Raspberry Pi detected → Real GPIO")
            self.encoder = RealPigpio(17, 27)
        else:
            self.get_logger().info("Mock mode (No real GPIO)")
            self.encoder = MockPigpio()

        self.publisher_ = self.create_publisher(Int32, 'encoder_count', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        count = self.encoder.read_encoder()
        msg = Int32()
        msg.data = count
        self.publisher_.publish(msg)
        self.get_logger().info(f'Encoder Count: {count}')

def main(args=None):
    rclpy.init(args=args)
    node = EncoderPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

