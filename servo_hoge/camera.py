import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Float32MultiArray  # 配列を渡せるはず

import numpy as np
import cv2
aruco = cv2.aruco
p_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)


def detect_mark_test():
    cap = cv2.VideoCapture(0)
    return [0.0, 1.0]

def detect_mark(n):
    """マーカーの位置座標x,yを検出する関数
    引数：マーカーの数．int型．
    戻り値：位置座標x,y．（2）配列．
    """

    n_mark = n+4
    X = np.empty((n_mark-4,2))
    m = np.empty((4,2))
    m2 = np.empty((n_mark-4,2))
    corners0 = [np.empty((1,4,2))]*n_mark
    corners1 = [np.empty((1,4,2))]*n_mark
    corners2 = [np.empty((1,4,2))]*(n_mark-4)
    corners3 = [np.empty((1,4,2))]*(n_mark-4)
    
    # VideoCaptureのインスタンスを作成する
    # 引数でカメラを選ぶ
    cap = cv2.VideoCapture(0)
    
    while True:
        # VideoCaptureから1フレーム読み込む
        ret, frame = cap.read()
        
        windowsize = (640, 480)
        frame = cv2.resize(frame, windowsize)
    
        # 元画像を表示する
        #cv2.imshow('Raw Frame', frame)
    
        # 検出
        corners0, ids0, rejectedImgPoints0 = aruco.detectMarkers(frame, p_dict)
        
        # マーカーが検出できなかった場合はcontinue
        if len(ids0.ravel()) != n_mark:
            # print(
            #     '検出1 マーカーの検出に失敗orマーカー個数不一致 設定したマーカーの個数:',
            #     n_mark,' 検出したマーカーの個数:',len(ids0.ravel())
            # )
            continue
        
        for i0,c0 in zip(ids0.ravel(), corners0):
            corners1[i0] = c0.copy()
        
        m[0] = corners1[0][0][2]
        m[1] = corners1[1][0][3]
        m[2] = corners1[2][0][0]
        m[3] = corners1[3][0][1]
        
        # 変形後画像サイズ
        width, height = (800,405)
        marker_coordinates = np.float32(m)
        true_coordinates   = np.float32([[0,0],[width,0],[width,height],[0,height]])
        trans_mat = cv2.getPerspectiveTransform(marker_coordinates,true_coordinates)
        edframe = cv2.warpPerspective(frame,trans_mat,(width, height))
        
        # 検出
        corners2, ids2, rejectedImgPoints2 = aruco.detectMarkers(edframe, p_dict)
        
        # マーカーが検出できなかった場合はcontinue
        if len(ids2.ravel()) != n_mark-4:
            print(
                '検出2 マーカーの検出に失敗orマーカー個数不一致 設定したマーカーの個数:',
                n_mark-4,' 検出したマーカーの個数:',len(ids2.ravel())
            )
            continue
        else:
            break
        
    # 検出結果をオーバーレイ・表示
    #edframe2 = aruco.drawDetectedMarkers(edframe.copy(), corners2, ids2)
    #cv2.imshow('marker', edframe2)
    
    # 座標変換関数
    tr_x = lambda x : x - 400 # X軸 画像座標→実座標 
    tr_y = lambda y : 405 - y # Y軸 　〃
    
    # マーカー中心座標算出
    for i2,c2 in zip(ids2.ravel(), corners2):
        corners3[i2-4] = c2.copy()
        #print(corners3[i2-4])
        m2[i2-4] = corners3[i2-4][0].mean(axis=0)
        #print(m2[i2-4])
        X[i2-4][0]= round(tr_x(m2[i2-4][0]), 1)
        X[i2-4][1]= round(tr_y(m2[i2-4][1]), 1)
        
    '''
    # 座標算出・表示
    for k in range(n_mark-4):
        x = round(tr_x(m2[k][0]), 1)
        y = round(tr_y(m2[k][1]), 1)
        #print(f'■ マーカー {k+4} の中心位置 X={x}mm Y={y}mm ',end='')
        
    #print('')
    
    # キー入力を1ms待って、k が27（ESC）だったらBreakする
    k = cv2.waitKey(1)
    if k == 27:
        break
    '''
    
    # キャプチャをリリースして、ウィンドウをすべて閉じる
    cap.release()
    #cv2.destroyAllWindows()
    return X


class CameraNode(Node):
    """webカメラのnode
    ・マーカー位置をpublish
    """

    def __init__(self, CAMERA_NAME):
        super().__init__('camera_' + CAMERA_NAME)
        self.cam = self.create_publisher(
            Float32MultiArray,
            'marker_position',
            10,
        )
        self.timer = self.create_timer(
            1,
            self.camera_callback,
        )
    
    def camera_callback(self):
        x = Float32MultiArray()
        #x.data = detect_mark(1)
        x.data = detect_mark_test()
        self.cam.publish(x)
        self.get_logger().info('marker_position_from_camera_x: %e' % x.data[0])
        self.get_logger().info('marker_position_from_camera_y: %e' % x.data[1])



def main():
    rclpy.init()
    executor = SingleThreadedExecutor()

    nodes = []
    for i in range(1):
        node_name = str(i)
        nodes.append(CameraNode(node_name))
    
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