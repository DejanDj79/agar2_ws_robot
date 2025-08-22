import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge

from std_msgs.msg import String
from sensor_msgs.msg import Image
from zed_msgs.msg import ObjectsStamped


class CameraParser(Node):
    def __init__(self):
        super().__init__('camera_parser')
        self.get_logger().info('Initialization completed')

        self.timer_check     = self.create_timer(20.0, self.check_object)
        self.timer_check.cancel()

        self.cv_image = []
        self.bridge = CvBridge()

        self.img_counter = 0

        self.object_in_stop_zone    = False
        self.publish_notification   = False

        self.capture_image      = self.create_subscription(Image, '/zedx/zed_node/rgb/image_rect_color', self.capture_image_callback, 10)
        self.detected_object    = self.create_subscription(ObjectsStamped, '/zedx/zed_node/obj_det/objects', self.detected_object_callback, 10)

        self.notification_pub   = self.create_publisher(String, '/agar_notifications', 10)
        self.image_pub          = self.create_publisher(Image, '/image/detected_object', 10)

        self.image_pub = self.create_publisher(Image, '/image/detected_object', 10)

        self.images_for_video = []

    def __del__(self):
        print('Closing camera parser...')

        fourcc = cv2.VideoWriter_fourcc(*'XVID') 
        video = cv2.VideoWriter('/home/nikolica/Pictures/stereolabs/video.avi', fourcc, 5, (960, 600))
        for img in self.images_for_video:
            video.write(img)
        video.release()

    def capture_image_callback(self, msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def detected_object_callback(self, msg):
        if len(self.cv_image) > 0:
            # curr_detection = 0
            # self.get_logger().info(f'Detected {len(msg.objects)} objects')
            for obj in msg.objects:
                top_left_x = round(float(obj.bounding_box_3d.corners[0].kp[0]), 4)
                top_left_y = round(float(obj.bounding_box_3d.corners[0].kp[1]), 4)
                top_left_z = round(float(obj.bounding_box_3d.corners[0].kp[2]), 4)

                bottom_left_x = round(float(obj.bounding_box_3d.corners[4].kp[0]), 4)
                bottom_left_y = round(float(obj.bounding_box_3d.corners[4].kp[1]), 4)
                bottom_left_z = round(float(obj.bounding_box_3d.corners[4].kp[2]), 4)

                top_right_x = round(float(obj.bounding_box_3d.corners[3].kp[0]), 4)
                top_right_y = round(float(obj.bounding_box_3d.corners[3].kp[1]), 4)
                top_right_z = round(float(obj.bounding_box_3d.corners[3].kp[2]), 4)

                bottom_right_x = round(float(obj.bounding_box_3d.corners[7].kp[0]), 4)
                bottom_right_y = round(float(obj.bounding_box_3d.corners[7].kp[1]), 4)
                bottom_right_z = round(float(obj.bounding_box_3d.corners[7].kp[2]), 4)

                # self.get_logger().info(f'Object detected at top: ({top_left_x}, {top_left_y}, {top_left_z})')
                # self.get_logger().info(f'Object detected at bot: ({bottom_left_x}, {bottom_left_y}, {bottom_left_z})')

                xmin = int(obj.bounding_box_2d.corners[0].kp[0] / 2)
                ymin = int(obj.bounding_box_2d.corners[0].kp[1] / 2)
                xmax = int(obj.bounding_box_2d.corners[2].kp[0] / 2)
                ymax = int(obj.bounding_box_2d.corners[2].kp[1] / 2)

                color = (0, 255, 0)
                text_msg = 'Object detected in SAFE zone.'

                if (top_left_y > -0.75 and top_left_y < 0.75 and top_left_x > 2 and top_left_x < 5) or \
                    (bottom_left_y > -0.75 and bottom_left_y < 0.75 and bottom_left_x > 2 and bottom_left_x < 5) or \
                    (top_right_y > -0.75 and top_right_y < 0.75 and top_right_x > 2 and top_right_x < 5) or \
                    (bottom_right_y > -0.75 and bottom_right_y < 0.75 and bottom_right_x > 2 and bottom_right_x < 5):
                        color = (28,153,255)
                        distance = min(top_left_x, bottom_left_x, top_right_x, bottom_right_x)
                        text_msg = 'Object detected in SLOW zone. Distance: {:.4f} m.'.format(distance)
                elif (top_left_y > -0.75 and top_left_y < 0.75 and top_left_x > 1 and top_left_x < 2) or \
                    (bottom_left_y > -0.75 and bottom_left_y < 0.75 and bottom_left_x > 1 and bottom_left_x < 2) or \
                    (top_right_y > -0.75 and top_right_y < 0.75 and top_right_x > 1 and top_right_x < 2) or \
                    (bottom_right_y > -0.75 and bottom_right_y < 0.75 and bottom_right_x > 1 and bottom_right_x < 2):
                    color = (0,0,255)
                    distance = min(top_left_x, bottom_left_x, top_right_x, bottom_right_x)
                    text_msg = 'Object detected in STOP zone. Distance: {:.4f} m.'.format(top_left_x)

                    if self.object_in_stop_zone == False:
                        self.object_in_stop_zone    = True
                        self.publish_notification   = True
                        self.get_logger().info('Object detected in STOP zone. Stopping navigation...')
                        self.timer_check.reset()
                else:
                    self.object_in_stop_zone = False

                cv2.rectangle(self.cv_image, (xmin, ymin), (xmax, ymax), color, 2)

                x1 = xmin
                x2 = xmax
                y1 = ymin
                y2 = ymax

                cv2.putText(self.cv_image, text_msg, (10, 15), cv2.FONT_HERSHEY_PLAIN, 1.0, (0, 0, 0), 2)

                # cv2.putText(self.cv_image, f'TL: x: {top_left_x}, y: {top_left_y}, z: {top_left_z}', (10, 30), cv2.FONT_HERSHEY_PLAIN, 1.0, (0, 0, 0), 2)
                # cv2.putText(self.cv_image, f'BL: x: {bottom_left_x}, y: {bottom_left_y}, z: {bottom_left_z}', (10, 45), cv2.FONT_HERSHEY_PLAIN, 1.0, (0, 0, 0), 2)

                # cv2.putText(self.cv_image, f'TR: x: {top_right_x}, y: {top_right_y}, z: {top_right_z}', (10, 60), cv2.FONT_HERSHEY_PLAIN, 1.0, (0, 0, 0), 2)
                # cv2.putText(self.cv_image, f'BR: x: {bottom_right_x}, y: {bottom_right_y}, z: {bottom_right_z}', (10, 75), cv2.FONT_HERSHEY_PLAIN, 1.0, (0, 0, 0), 2)

                cv2.rectangle(self.cv_image, (x1, y1), (x2, y1 + 20), color, -1)

                cv2.putText(self.cv_image, 'Person', (x1 + 7, y1 + 15), cv2.FONT_HERSHEY_PLAIN, 1.0, (0, 0, 0), 1)

                self.images_for_video.append(self.cv_image.copy())
                
                cv2.imwrite(f"/home/nikolica/Pictures/stereolabs/zedx_{self.img_counter}.jpg", self.cv_image)
                self.img_counter += 1

                if self.publish_notification:
                    img = self.bridge.cv2_to_imgmsg(self.cv_image, encoding="bgr8")
                    self.image_pub.publish(img)
                    text = String()
                    text.data = text_msg
                    self.notification_pub.publish(text)
                    self.publish_notification = False


                # curr_detection += 1
            self.cv_image = []

    def check_object(self):
        if self.object_in_stop_zone == False:
            self.get_logger().info('No object in STOP zone. Resuming navigation...')
            self.timer_check.cancel()


def main(args=None):
    rclpy.init(args=args)
    camera_parser = CameraParser()
    rclpy.spin(camera_parser)

    camera_parser.get_logger().info('Closed')

    camera_parser.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()