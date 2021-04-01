import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Int16
import time
import pigpio

#SERVO_PIN = 23

PULSE_MIN = 500 + 10
PULSE_MAX = 2500 - 10

SERVO_PIN_1 = 23
SERVO_PIN_2 = 24

def pulse(ANGLE):
    """ANGLE[degree]からpwnパルス値を計算"""
    return (ANGLE / 180) * (PULSE_MAX - PULSE_MIN) + PULSE_MIN


class ServoNode(Node):
    
    def __init__(self, SERVO_PIN):
        super().__init__("Servo")
        self.SERVO_PIN = SERVO_PIN
        self.sub_servo = self.create_subscription(
            Int16, 
            "servo", 
            self.servo_callback, 
            10,
            )
    
    def servo_callback(self, ANGLE):
        """ANGLEにINT16？の信号がくる？"""
        pi = pigpio.pi()
        pi.set_servo_pulsewidth(
            self.SERVO_PIN, 
            pulse(ANGLE),
            )
        self.get_logger().info('servo_exec: %d' % ANGLE)


def main():
    rclpy.init()
    executor = SingleThreadedExecutor()
    
    ## ノードをインスタンス化しexecutorにadd
    # 1個目
    servo_1 = ServoNode(SERVO_PIN_1)
    executor.add_node(servo_1)
    
    # # 2個目
    # servo_2 = ServoNode(SERVO_PIN_2)
    # executor.add_node(servo_2)
    
    try:
        executor.spin()  # 無限ループ？
    except KeyboardInterrupt:  # Ctrl-C
        pass
    
    ## ノード殺す
    executor.shutdown()
    servo_1.destroy_node()
    #servo_2.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()