import rclpy as r
import redis as rd
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
        self.streamwise_objects = {camera_id: {} for camera_id in self.get_parameter('camera_ids').get_parameter_value().integer_array_value}

    def merge_streamwise_objects(self):
        merged = {
            (outer_k, inner_k): val for outer_k, inner_dict in self.streamwise_objects.items() for inner_k, val in inner_dict.items()
            }
        return merged

    def send_notification(self, objects):
        db = rd.Redis(host='localhost', port=6379, db=0)
        if db.set(self.notification_timestamp, objects):
            self.get_logger().info("Entry put in database.")
        else:
            self.get_logger().error("Entry in database failed.")

    def callback(self, *result_messages: Model):
        if all([result_message.status == 'START' for result_message in result_messages]):
            seconds, nanoseconds = self.get_clock().now().seconds_nanoseconds()
            self.notification_timestamp = seconds + (nanoseconds * 1e-9)
        elif all([result_message.status == 'END' for result_message in result_messages]):
            self.notification_timestamp = None
            merged_objects = self.merge_streamwise_objects()
            self.send_notification(merged_objects)
            self.streamwise_objects = {}
        for result_message in result_messages:
            for i, cls in zip(result_message.ids, result_message.tags):
                self.streamwise_objects[result_message.camera_id][i] = cls

def main(args=None):
    r.init(args=args)
    node = Collator()
    r.spin(node)
    node.destroy_node()
    r.shutdown()

if __name__ == '__main__':
    main()
