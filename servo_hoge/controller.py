import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16

#import time

# class ServoController(Node):
#     def __init__(self):
#         super().__init__('controller')
#         self.create_publisher(Int16, "countup", 10)
    
#     def callback(self, msg):
#         self.publish(msg)

def main():
    rclpy.init()
    node = Node("controller")
    pub = node.create_publisher(Int16, "countup", 10)

    n = 0  # パブリッシュする値。まずは初期化。

    def callback():
        global n
        msg = Int16()
        msg.data = n
        pub.publish(msg)
        if n > 179:
            n = 180
        else:
            n += 1

    node.create_timer(0.5, callback)
    rclpy.spin(node)

if __name__ == 'main':
    main()