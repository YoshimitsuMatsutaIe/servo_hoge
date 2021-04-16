import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Int16



class ControllerNode(Node):
    """制御器のノード
    ・サーボモータ司令角度[degree]をpublish
    """
    
    def __init__(self, CONTROLLER_NAME):
        super().__init__('controller_' + CONTROLLER_NAME)
        self.controller = self.create_publisher(
            Int16, 
            'command_angle_' + CONTROLLER_NAME, 
            10,
            )
        self.timer = self.create_timer(
            0.1,
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
    rclpy.init()
    executor = SingleThreadedExecutor()

    nodes = []
    for i in range(5):
        node_name = str(i)
        nodes.append(node_name)
    
    for node in nodes:
        executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    
    # ノード終了
    executor.shutdown()

    for node in nodes:
        node.destroy_node()

    rclpy.shutdown()


if __name__ == 'main':
    main()