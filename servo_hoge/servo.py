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
    z = (ANGLE / 180) * (PULSE_MAX - PULSE_MIN) + PULSE_MIN
    return int(z)

class ServoNode(Node):
    
    def __init__(self, SERVO_PIN):
        super().__init__("servo")
        self.SERVO_PIN = SERVO_PIN
        self.init_motor()
        self.create_subscription(
            Int16, 
            "countup", 
            self.servo_callback, 
            10,
            )
    
    def init_motor(self):
        self.pi = pigpio.pi()
        self.pi.set_mode(self.SERVO_PIN, pigpio.OUTPUT)
        self.pi.set_servo_pulsewidth(self.SERVO_PIN, 500)
    
    def servo_callback(self, msg):
        """ANGLEにINT16？の信号がくる？"""
        self.pi = pigpio.pi()
        self.pi.set_mode(self.SERVO_PIN, pigpio.OUTPUT)
        self.pi.set_servo_pulsewidth(
            self.SERVO_PIN, 
            pulse(msg.data),
            )
        self.get_logger().info('servo_exec: %d' % msg.data)


# def main():
#     rclpy.init()
#     executor = SingleThreadedExecutor()
    
#     ## ノードをインスタンス化しexecutorにadd
#     # 1個目
#     servo_1 = ServoNode(SERVO_PIN_1)
#     executor.add_node(servo_1)
    
#     # # 2個目
#     # servo_2 = ServoNode(SERVO_PIN_2)
#     # executor.add_node(servo_2)
    
#     try:
#         executor.spin()  # 無限ループ？
#     except KeyboardInterrupt:  # Ctrl-C
#         pass
    
#     ## ノード殺す
#     executor.shutdown()
#     servo_1.destroy_node()
#     #servo_2.destroy_node()
#     rclpy.shutdown()



def main():
    rclpy.init()
    node = ServoNode(SERVO_PIN_1)

    try:
        rclpy.spin(node)  # 無限ループ？
    except KeyboardInterrupt:  # Ctrl-C
        pass
    
    ## ノード殺す
    node.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()
