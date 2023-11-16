import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Point
import tf2_ros
from geometry_msgs.msg import TransformStamped

class WhiteLightDetector(Node):
    def __init__(self):
        super().__init__('white_light_detector')
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',  # カメラ画像のトピック名に置き換えてください
            self.image_callback,
            10)
        self.subscription

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Point, 'center_point_topic', 10)  # Point型のメッセージを使用
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().info(f'Error converting image: {e}')
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

                # 中心座標をパブリッシュ
                point_msg = Point()
                point_msg.x = float(ccx)
                point_msg.y = float(ccy)
                self.publisher.publish(point_msg)

                # TF変換を作成
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = 'camera_frame'  # カメラ座標系のフレームID
                t.child_frame_id = 'other_frame'  # 変換後の座標系のフレームID
                t.transform.translation.x = ccx  # 中心座標をセット
                t.transform.translation.y = ccy
                t.transform.translation.z = 0.0
                t.transform.rotation.x = 0.0
                t.transform.rotation.y = 0.0
                t.transform.rotation.z = 0.0
                t.transform.rotation.w = 1.0

                # TF変換を送信
                self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = WhiteLightDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

