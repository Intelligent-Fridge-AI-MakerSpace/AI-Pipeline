import rclpy as r
import message_filters as m
from rclpy.node import Node
from ai_msgs.msg import Model

class Collator(Node):
    def __init__(self):
        super(Collator, self).__init__(node_name='collator')
        self.declare_parameter('camera_ids', [0])
        result_publishers = [m.Subscriber(self, Model, 'results') for _ in self.get_parameter('camera_ids').get_parameter_value().integer_array_value]
        synchronizer = m.ApproximateTimeSynchronizer(result_publishers, queue_size=10, slop=1)
        synchronizer.registerCallback(self.callback)
        self.notification_timestamp = None
        self.streamwise_objects = {}

    def merge_streamwise_objects(self):
        pass

    def send_notification(self, objects):
        pass

    def callback(self, *result_messages: Model):
        if all([result_message.status == 'START' for result_message in result_messages]):
            seconds, nanoseconds = self.get_clock().now().seconds_nanoseconds()
            self.notification_timestamp = seconds + (nanoseconds * 1e-9)
        elif all([result_message.status == 'END' for result_message in result_messages]):
            self.notification_timestamp = None
            self.merge_streamwise_objects()
            self.send_notification()
            self.streamwise_objects = {}
        for i, result_message in enumerate(result_messages):
            pass

def main(args=None):
    r.init(args=args)
    node = Collator()
    r.spin(node)
    node.destroy_node()
    r.shutdown()

if __name__ == '__main__':
    main()
