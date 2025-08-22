import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image

import telegram
from telegram import InputFile
import requests
import asyncio
from datetime import datetime

from io import BytesIO

import cv2
from PIL import Image as PILImage
from cv_bridge import CvBridge

class Notifications(Node):
    def __init__(self):
        super().__init__('notifications')
        self.get_logger().info('Initialization completed')

        timer_period        = 1 / 20  # seconds
        self.timer          = self.create_timer(timer_period, self.run)

        self.notification_sub   = self.create_subscription(String, '/agar_notifications', self.parse_notification, 10)
        self.image_sub          = self.create_subscription(Image, '/image/detected_object', self.received_image, 10)

        self.navigation_active_pub  = self.create_publisher(Bool, '/navigation_active', 10)

        self.channel_id     = -1002486788098 #-4224036322
        self.token          = '6727479436:AAGEyfUib-0lv42rZCtpuZJoL54gxJezfuU'
        self.noty_text      = ''
        self.bot            = telegram.Bot(token=self.token)

        self.bridge = CvBridge()
        self.image_to_send = None

    def __del__(self):
        print('Closing notifications...')

    def parse_notification(self, msg):
        now = datetime.now()
        self.noty_text += '{0}\r\n{1}\r\n'.format(msg.data, now.strftime("%d.%m.%Y, %H:%M:%S"))
        # self.get_logger().info(f'Received notification: {self.noty_text}')

    def received_image(self, msg):
        # self.get_logger().info('Received image for notification')
        self.image_to_send = cv2.cvtColor(self.bridge.imgmsg_to_cv2(msg, 'bgr8'), cv2.COLOR_BGR2RGB)
        
    def is_internet_connected(self):
        try:
            requests.head("http://www.google.com", timeout=1)
            return True
        except requests.ConnectionError:
            return False

    async def send_telegram_msg(self, msg):
        try:
            # self.get_logger().info(f'Sending message: {msg}')
            await self.bot.send_message(chat_id=self.channel_id, text=msg)
            self.noty_text = ''
        except Exception as ex:
            msg = ''

    async def send_telegram_photo(self, msg):
        try:
            img = PILImage.fromarray(msg)
            img_bytes_io = BytesIO()
            img.save(img_bytes_io, format='PNG')
            img_bytes_io.seek(0)

            photo_file = InputFile(img_bytes_io, filename="agar.png")
            await self.bot.send_photo(chat_id=self.channel_id, photo=photo_file)
            self.image_to_send = None
        except Exception as ex:
            msg = ''

    def run(self):
        if len(self.noty_text) > 0 and self.is_internet_connected():
            try:
                asyncio.run(self.send_telegram_msg(self.noty_text))
                # self.noty_text = ''
            except Exception as ex:
                self.get_logger().error(ex)
        if self.image_to_send is not None and self.is_internet_connected():
            try:
                asyncio.run(self.send_telegram_photo(self.image_to_send))
                # self.image_to_send = None
            except Exception as ex:
                self.get_logger().error(ex)
            
def main(args=None):
    rclpy.init(args=args)
    notifications = Notifications()
    notifications.run()
    rclpy.spin(notifications)

    # executor = rclpy.executors.MultiThreadedExecutor()
    # executor.add_node(notifications)
    # executor.spin()

    # notifications.get_logger().info('Closed')
    notifications.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()