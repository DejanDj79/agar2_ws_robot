import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from zed_msgs.msg import ObjectsStamped


class CameraParser(Node):
    def __init__(self):
        super().__init__('camera_parser')
        self.get_logger().info('Initialization completed')

        self.cv_image = []

        self.img_counter = 0

        self.capture_image      = self.create_subscription(Image, '/zedx/zed_node/rgb/image_rect_color', self.capture_image_callback, 10)
        self.detected_object    = self.create_subscription(ObjectsStamped, '/zedx/zed_node/obj_det/objects', self.detected_object_callback, 10)

    def capture_image_callback(self, msg):
        # self.get_logger().info('Received image with width: %d and height: %d' % (msg.width, msg.height))
        bridge = CvBridge()
        self.cv_image = bridge.imgmsg_to_cv2(msg, 'bgr8')
        # cv2.imwrite("/home/nikolica/output_image.jpg", cv_image)

    def detected_object_callback(self, msg):
        if len(self.cv_image) > 0:
            for obj in msg.objects:
                # self.get_logger().info(f'{obj.bounding_box_2d.corners}')

                xmin = int(obj.bounding_box_2d.corners[0].kp[0] / 2)
                ymin = int(obj.bounding_box_2d.corners[0].kp[1] / 2)
                xmax = int(obj.bounding_box_2d.corners[2].kp[0] / 2)
                ymax = int(obj.bounding_box_2d.corners[2].kp[1] / 2)  

                cv2.rectangle(self.cv_image, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)

                x1 = xmin
                x2 = xmax
                y1 = ymin
                y2 = ymax

                cv2.rectangle(self.cv_image, (x1, y1), (x2, y1 + 20), (0, 255, 0), -1)

                cv2.putText(self.cv_image, 'Detected person', (x1 + 7, y1 + 15), cv2.FONT_HERSHEY_PLAIN, 1.0, (0, 0, 0), 1)
                
                cv2.imwrite(f"/home/nikolica/Pictures/stereolabs/zedx_{self.img_counter}.jpg", self.cv_image)
                # self.img_counter += 1

def main(args=None):
    rclpy.init(args=args)
    camera_parser = CameraParser()
    rclpy.spin(camera_parser)
    camera_parser.destroy_node()
    rclpy.shutdown()

    camera_parser.get_logger().info('Closed')

if __name__ == '__main__':
    main()