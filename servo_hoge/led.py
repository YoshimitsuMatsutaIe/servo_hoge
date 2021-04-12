import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Int16
import time
import pigpio

SERVO_PIN = 23

class PikaPikaLed(Node):

    def __init__(self, SERVO_PIN):
        super().__init__("led")
        self.SERVO_PIN = SERVO_PIN
        self.init_led()  # led初期化
        self.create_subscription(
            Int16,
            "led",
            self.led_callback,
            10
        )
    
    def init_led(self):
        self.pi = pigpio.pi()
        self.pi.set_mode(self.SERVO_PIN, pigpio.OUTPUT)
        self.pi(self.SERVO_PIN, 0)
        time.sleep(1)
        self.pi(self.SERVO_PIN, 1)
        time.sleep(1)
    
    def led_callback(self):
        pass

def main():
    rclpy.init()
    node = PikaPikaLed(SERVO_PIN)
    try:
        rclpy.spin(node)  # 無限ループ？
    except KeyboardInterrupt:  # Ctrl-C
        pass
    
    ## ノード殺す
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()


