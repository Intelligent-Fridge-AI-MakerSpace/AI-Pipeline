import cv2 as cv
import rclpy as r
from rclpy.node import Node
from ai_msgs.msg import Status, Camera

class CameraProcessor(Node):
    def __init__(self):
        super(CameraProcessor, self).__init__(node_name='camera_processor')
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('width', 512)
        self.declare_parameter('heighth', 512)
        self.camera = cv.VideoCapture(self.get_parameter('camera_id').get_parameter_value().integer_value)
        self.camera_publisher = self.create_publisher(Camera, 'camera', 10)
        self.create_subscription(Status, '/status', self.callback, 10)

    def callback(self, status_message: Status):
        ret, frame = self.camera.read()
        if ret:
            frame = cv.resize(frame, (self.get_parameter('width').get_parameter_value().integer_value, self.get_parameter('height').get_parameter_value().integer_value))
            camera_message = Camera()
            camera_message.header = status_message.header
            camera_message.image = frame
            camera_message.status = status_message
            self.camera_publisher.publish(camera_message)
        else:
            self.camera = cv.VideoCapture(self.get_parameter('camera_id').get_parameter_value().integer_value)

def main(args=None):
    r.init(args=args)
    camera_processor = CameraProcessor()
    r.spin(camera_processor)
    camera_processor.destroy_node()
    r.shutdown()

if __name__ == '__main__':
    main()
