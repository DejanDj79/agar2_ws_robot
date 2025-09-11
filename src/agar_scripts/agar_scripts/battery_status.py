import rclpy
import asyncio

from rclpy.node import Node
from std_msgs.msg import String

from dalybms import DalyBMSBluetooth
from rclpy.callback_groups import ReentrantCallbackGroup


class BatteryStatus(Node):
    def __init__(self):
        super().__init__('notifications')
        self.get_logger().info('Initialization completed')

        self.callback_group = ReentrantCallbackGroup()

        timer_period        = 30  # seconds
        self.timer          = self.create_timer(timer_period, self.run)

        self.bt_bms = DalyBMSBluetooth()
        self.mac_address = 'B7:7B:02:02:0B:32'

        self.soc                        = None
        self.notification_timeout       = 300
        self.send_initial_notification  = True

        self.notification_pub   = self.create_publisher(String, '/agar_notifications', 10, callback_group=self.callback_group)

    def __del__(self):
        print('Closing battery status...')

    async def connect(self):
        try:
            await self.bt_bms.connect(mac_address=self.mac_address)
        except Exception as ex:
            print(ex)

    async def update_soc(self):
        try:
            self.soc = await self.bt_bms.get_soc()
        except Exception as ex:
            print(ex)

    async def read_soc(self):
        self.get_logger().info("Bluetooth connecting...")
        await self.connect()
        self.get_logger().info("Bluetooth connected...")

        try:
            await self.update_soc()
            self.get_logger().info(self.soc['total_voltage'])
            self.get_logger().info(self.soc['current'])
            self.get_logger().info(self.soc['soc_percent'] / 100.0)
            await self.bt_bms.disconnect()

            text = String()
            text.data = 'Battery level: {0}, total voltage: {1}.'.format(round(self.soc['soc_percent'], 2), round(self.soc['total_voltage'], 2))
            self.notification_pub.publish(text)
        except Exception as ex:
            self.get_logger().error(ex)

    def run(self):
        try:
            asyncio.run(self.read_soc())
        except Exception as ex:
            self.get_logger().error(ex)

def main(args=None):
    rclpy.init(args=args)
    battery_status = BatteryStatus()
    rclpy.spin(battery_status)

    battery_status.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()