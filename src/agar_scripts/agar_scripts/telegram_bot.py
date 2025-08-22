import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool

from telegram import Update
from telegram.ext import Application, MessageHandler, filters   

class Telegram_bot(Node):
    def __init__(self):
        super().__init__('telgram_bot')
        self.get_logger().info('Initialization completed')

        self.navigation_active_pub  = self.create_publisher(Bool, '/navigation_active', 10)

        self.token          = '6727479436:AAGEyfUib-0lv42rZCtpuZJoL54gxJezfuU'

        self.app = Application.builder().token(self.token).build()
        self.app.add_handler(MessageHandler(filters.UpdateType.CHANNEL_POSTS & filters.COMMAND, self.handle_channel_command))
        self.app.run_polling(allowed_updates=Update.ALL_TYPES)

    async def handle_channel_command(self, update, context):
        """Processes commands received in a channel."""
        message_text = update.channel_post.text
        if message_text == "/help":
            await update.channel_post.reply_text("List of available commands:\n"
                "/start_nav - Start navigation\n"
                "/stop_nav - Stop navigation\n"
                "/brake_on - Activate brake\n"
                "/brake_off - Deactivate brake\n"
                "/light_on - Bridge light ON\n"
                "/light_off - Bridge light OFF\n"
                "/help - Show this help message")
        elif message_text == "/start_nav":
            msg = Bool()
            msg.data = True
            self.navigation_active_pub.publish(msg)
            await update.channel_post.reply_text("Navigation started!")
        elif message_text == "/stop_nav":
            msg = Bool()
            msg.data = False
            self.navigation_active_pub.publish(msg)
            await update.channel_post.reply_text("Navigation stopped!")
        else:
            await update.channel_post.reply_text(f"Unknown command!!!\nList of available commands:\n"
                "/start_nav - Start navigation\n"
                "/stop_nav - Stop navigation\n"
                "/brake_on - Activate brake\n"
                "/brake_off - Deactivate brake\n"
                "/light_on - Bridge light ON\n"
                "/light_off - Bridge light OFF\n"
                "/help - Show this help message")

    def __del__(self):
        self.get_logger().info('Closing telegram bot...')
        self.app.stop_running()

def main(args=None):
    rclpy.init(args=args)
    telegram_bot = Telegram_bot()
    rclpy.spin(telegram_bot)
    telegram_bot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()