import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Bool, Int32MultiArray

class ImageProcessingNode(Node):
    def __init__(self):
        super().__init__('image_processing_node')
        self.subscription = self.create_subscription(
            Image,
            'camera_image_topic',  # カメラ画像のトピック名に置き換えてください
            self.image_callback,
            10
        )
        self.publisher = self.create_publisher(Bool, 'white_light_detected', 10)
        self.center_publisher = self.create_publisher(Int32MultiArray, 'center_coordinates', 10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            # ROSメッセージをOpenCVイメージに変換
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # 画像処理を行い、白い光を検出
            lower_white = np.array([200, 200, 200], dtype=np.uint8)
            upper_white = np.array([255, 255, 255], dtype=np.uint8)
            mask = cv2.inRange(cv_image, lower_white, upper_white)

            # 白い光を検出した場合
            if cv2.countNonZero(mask) > 0:
                moments = cv2.moments(mask)
                center_x = int(moments["m10"] / moments["m00"])
                center_y = int(moments["m01"] / moments["m00"])
                center_coordinates = Int32MultiArray(data=[center_x, center_y])
                self.center_publisher.publish(center_coordinates)
                self.publisher.publish(Bool(data=True))
            else:
                # 白い光を検出しなかった場合
                self.publisher.publish(Bool(data=False))

        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    image_processing_node = ImageProcessingNode()
    rclpy.spin(image_processing_node)
    image_processing_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
