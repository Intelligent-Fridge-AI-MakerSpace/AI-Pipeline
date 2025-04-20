import rclpy as r
import pynput as p
from rclpy.node import Node
from ai_msgs.msg import Status

class Signal(Node):
    def __init__(self):
        super(Signal, self).__init__(node_name='signal')
        self.status = 'IDLE'
        self.status_changed = False
        self.status_publisher = self.create_publisher(Status, '/status', 10)
        self.create_timer(1 / self.declare_parameter('frequency', 30).value, self.publish_status)
        listener = p.keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        listener.start()

    def publish_status(self):
        if self.status_changed:
            if self.status == 'START':
                self.status = 'CONTINUE'
            elif self.status == 'END':
                self.status = 'IDLE'
            self.status_changed = False
        status_message = Status()
        status_message.header = self.get_clock().now().to_msg()
        status_message.status = self.status
        self.status_publisher.publish(status_message)
        self.status_changed = True

    def on_press(self, key):
        try:
            self.get_logger().info(f'{key.char} PRESSED')
            if self.status == 'IDLE' and key.char == 's':
                self.status = 'START'
            elif self.status == 'CONTINUE' and key.char == 'e':
                self.status = 'END'
            self.status_changed = False
        except Exception as e:
            self.get_logger().error(f"{e}")

    def on_release(self, key):
        try:
            self.get_logger().info(f'{key.char} RELEASED')
        except Exception as e:
            self.get_logger().error(f"{e}")

def main(args=None):
    r.init(args=args)
    signal = Signal()
    r.spin(signal)
    signal.destroy_node()
    r.shutdown()

if __name__ == '__main__':
    main()
