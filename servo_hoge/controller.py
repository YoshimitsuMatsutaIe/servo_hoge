import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16

#import time

angle = 0

class ControllerNode(Node):
    """制御器のノード
    ・サーボモータ司令を返す
    """
    
    def __init__(self):
        super().__init__('controller')
        self.controller = self.create_publisher(
            Int16, 
            "countup", 
            10,
            )
        self.timer = self.create_timer(
            0.5, 
            self.callback,
            )
    
    
    def callback(self):
        global angle
        command = Int16()
        command.data = angle
        self.controller.publish(command)
        self.get_logger().info('controller: %d' % angle)
        if angle > 179:
            angle = 180
        else:
            angle += 1


def main():
    rclpy.init()  # 初期化？
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    # ノード殺す
    node.destroy_node()
    rclpy.shutdown()


if __name__ == 'main':
    main()