#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import pigpio
import time

class EncoderGPIO:
    def __init__(self, pin_a=17, pin_b=27):
        self.pi = pigpio.pi("localhost", 8888)
        if not self.pi.connected:
            raise RuntimeError("Can't connect to pigpio daemon. Run: sudo pigpiod")

        self.pin_a = pin_a  # AO
        self.pin_b = pin_b  # A1
        self.counter = 0
        self.last_a = 0

        # 입력 핀 설정
        self.pi.set_mode(self.pin_a, pigpio.INPUT)
        self.pi.set_pull_up_down(self.pin_a, pigpio.PUD_UP)
        self.pi.set_mode(self.pin_b, pigpio.INPUT)
        self.pi.set_pull_up_down(self.pin_b, pigpio.PUD_UP)

        # 콜백 등록
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

class EncoderPublisher(Node):
    def __init__(self):
        super().__init__('encoder_test_pub')
        self.get_logger().info("Real GPIO mode activated (AO/A1 pins)")
        self.encoder = EncoderGPIO(pin_a=17, pin_b=27)

        self.publisher_ = self.create_publisher(Int32, 'encoder_count', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)  # 50ms 주기

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

