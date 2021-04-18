import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Int16, Float32MultiArray


time_interval = 0.1  #[s]


class ControllerNode(Node):
    """制御器のノード
    ・カメラからの位置情報をsubscrib
    ・サーボモータ司令角度[degree]をpublish
    """
    
    def __init__(self):
        super().__init__('controller')
        self.RMPalgebra_init()
        self.pub_controllers = []
        for i in range(5):
            self.pub_controllers.append(
                self.create_publisher(
                    Int16, 
                    'command_angle_' + str(i), 
                    10,
                )  # サーボノードへの指令値を発信
            )
        self.sub_camera = self.create_subscription(
            Float32MultiArray,
            'marker_position',
            self.cam_callback,
            10,
        )  # カメラノードからマーカー位置を受信
        
        self.timer = self.create_timer(
            time_interval,
            self.callback_controller,
            )
    
    def RMPalgebra_init(self):  # RMPalgebraをインスタンス化
        
    
    def callback_controller(self, msg):
        goal_position = msg.data
        
        
        for i in range(5):
            self.pub_controllers[i].publish(command_angle[i])
    
    def command_calc()
    
    def callback_test(self):
        """テスト用"""
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
    node = ControllerNode()
    
    try:
        rclpy.spin(node)  # 無限ループ
    except KeyboardInterrupt:
        pass
    
    # ノード終了
    node.destroy_node()
    rclpy.shutdown()


if __name__ == 'main':
    main()