import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Int16
import time
import pigpio


PULSE_MIN = 500
PULSE_MAX = 2500

SERVO_PIN_1 = 17
SERVO_PIN_2 = 27
SERVO_PIN_3 = 22
SERVO_PIN_4 = 23
SERVO_PIN_5 = 24
SERVO_PINS = [
    SERVO_PIN_1,
    SERVO_PIN_2,
    SERVO_PIN_3,
    SERVO_PIN_4,
    SERVO_PIN_5
    ]
SERVO_PINS = [23, 24, 22]  # 臨時


def pulse(ANGLE):
    """ANGLE[degree]からpwnパルス値を計算"""
    z = (ANGLE / 180) * (PULSE_MAX - PULSE_MIN) + PULSE_MIN
    return int(z)


class ServoNode(Node):
    """サーボモータのnode
    ・モーター指令角度[degree]をsubscriしてモーターを動かす
    """

    def __init__(self, SERVO_NAME, SERVO_PIN):
        super().__init__('servo_' + SERVO_NAME)
        self.SERVO_PIN = SERVO_PIN
        self.init_motor()
        self.create_subscription(
            Int16,
            'command_angle_' + SERVO_NAME,
            self.servo_callback,
            10,
            )
    
    def init_motor(self):
        """?を初期化"""
        self.pi = pigpio.pi()
        self.pi.set_mode(self.SERVO_PIN, pigpio.OUTPUT)
    
    def servo_callback(self, msg):
        """ANGLEにINT16？の信号がくる？"""
        self.pi.set_servo_pulsewidth(
            self.SERVO_PIN, 
            pulse(msg.data),
            )
        self.get_logger().info('servo_exec: %d' % msg.data)


def main():
    """n個同時実行"""
    rclpy.init()
    executor = SingleThreadedExecutor()
    
    ## ノードをインスタンス化しexecutorにadd
    nodes = []
    for i in range(len(SERVO_PINS)):
        node_name = str(i)
        nodes.append(ServoNode(node_name, SERVO_PINS[i]))
    
    for node in nodes:
        executor.add_node(node)
    
    try:
        executor.spin()  # 無限ループ
    except KeyboardInterrupt:  # Ctrl-C
        pass
    
    ## ノード終了
    executor.shutdown()
    
    for node in nodes:
        node.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
