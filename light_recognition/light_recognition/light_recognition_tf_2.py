import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Point, PointStamped
from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs

class WhiteLightDetector(Node):
    def __init__(self):
        super().__init__('white_light_detector')
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10)
        self.subscription

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(PointStamped, 'light_point', 10)  # World座標系に変換したPointStamped型のメッセージを使用

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # カメラのTransformを取得するためのCameraInfoのSubscription
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            'camera/camera_info',
            self.camera_info_callback,
            10)

        self.camera_info = None

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def image_callback(self, msg):
        if self.camera_info is None:
            self.get_logger().info('Camera information not available yet.')
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().info(f'Error converting image: {e}')
            return

        # 座標をWorld座標系に変換するためのTransformを取得
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',  # カメラのフレームIDに置き換えてください
                msg.header.frame_id,  # カメラ画像のフレームID
                rclpy.time.Time().to_msg(),
                rclpy.duration.Duration(seconds=1)
            )
        except Exception as e:
            self.get_logger().info(f'Error looking up transform: {e}')
            return

        # 画像処理: 白い光を検出
        lower_white = np.array([200, 200, 200])
        upper_white = np.array([255, 255, 255])
        mask = cv2.inRange(cv_image, lower_white, upper_white)

        # 白い光の輪郭を見つける
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # 最大の輪郭を見つける（一番明るい場所）
            max_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(max_contour)

            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])

                # 画像の中心座標を計算
                image_center_x = cv_image.shape[1] / 2
                image_center_y = cv_image.shape[0] / 2

                # 中心座標を(0, 0)を中心として変換
                ccx = cx - image_center_x
                ccy = cy - image_center_y

                #座標修正
                camera_h = 1.7 # ここにカメラの高さを入力してください
                set_val = 370 / camera_h
                ccx = round(ccx / set_val ,2)
                ccy = round(ccy / set_val ,2) 

                # 中心座標をWorld座標系に変換
                camera_point = PointStamped()
                camera_point.point.x = ccx
                camera_point.point.y = ccy
                camera_point.header.frame_id = msg.header.frame_id

                world_point = tf2_geometry_msgs.do_transform_point(camera_point, transform)

                world_point.point.z = 0.0
                # World座標系の座標をパブリッシュ
                self.publisher.publish(world_point)

def main(args=None):
    rclpy.init(args=args)
    node = WhiteLightDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
