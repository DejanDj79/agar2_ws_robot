import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool

class JetsonStatusPub(Node):
    def __init__(self):
        super().__init__('jetson_status_pub')

        timer_period = 1.0 / 10
        self.timer_publisher = self.create_timer(timer_period, self.publish_status)
        self.status_pub = self.create_publisher(Bool, "/jetson_online", 10)

    def publish_status(self):
        msg = Bool()
        msg.data = True
        self.status_pub.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)
    jetson_status_pub = JetsonStatusPub()
    rclpy.spin(jetson_status_pub)
    jetson_status_pub.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()