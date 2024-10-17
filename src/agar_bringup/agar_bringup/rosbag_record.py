import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from std_msgs.msg import String

import rosbag2_py
from datetime import datetime

class SimpleBagRecorder(Node):
    def __init__(self):
        super().__init__('bag_recorder')
        self.writer = rosbag2_py.SequentialWriter()

        file_name = 'agar_rosbag/{0}'.format(datetime.now().strftime('%Y%m%d%H%M%S'))

        storage_options = rosbag2_py._storage.StorageOptions(
            uri=file_name, # 'agar_rosbag'
            storage_id='sqlite3')
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)

        topic_info = rosbag2_py._storage.TopicMetadata(
            name='talker',
            type='std_msgs/msg/String',
            serialization_format='cdr')
        self.writer.create_topic(topic_info)

        self.subscription = self.create_subscription(
            String,
            'talker',
            self.topic_callback,
            10)
        self.subscription

    def topic_callback(self, msg):
        self.writer.write(
            'talker',
            serialize_message(msg),
            self.get_clock().now().nanoseconds)


def main(args=None):
    rclpy.init(args=args)
    sbr = SimpleBagRecorder()
    rclpy.spin(sbr)
    rclpy.shutdown()


if __name__ == '__main__':
    main()