import cv2 as cv
import rclpy as r
import numpy as np
import cv_bridge as c
from rclpy.node import Node
from ai_msgs.msg import Status, Camera

class CameraProcessor(Node):
    def __init__(self):
        super(CameraProcessor, self).__init__(node_name='camera_processor')
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 640)
        self.convertor = c.CvBridge()
        self.camera = cv.VideoCapture(self.get_parameter('camera_id').get_parameter_value().integer_value)
        self.camera_publisher = self.create_publisher(Camera, 'camera', 10)
        self.create_subscription(Status, '/status', self.callback, 10)

    def callback(self, status_message: Status):
        camera_message = Camera()
        camera_message.header = status_message.header
        camera_message.status = status_message.status
        ret, frame = self.camera.read()
        if ret:
            frame = cv.resize(frame, (self.get_parameter('width').get_parameter_value().integer_value, self.get_parameter('height').get_parameter_value().integer_value))
        else:
            frame = np.zeros(shape=(self.get_parameter('height').get_parameter_value().integer_value, self.get_parameter('width').get_parameter_value().integer_value, 3), dtype='uint8')
            self.camera = cv.VideoCapture(self.get_parameter('camera_id').get_parameter_value().integer_value)
            # self.callback(status_message)
        camera_message.image = self.convertor.cv2_to_imgmsg(frame, encoding='bgr8', header=status_message.header)
        self.camera_publisher.publish(camera_message)

def main(args=None):
    r.init(args=args)
    camera_processor = CameraProcessor()
    r.spin(camera_processor)
    camera_processor.destroy_node()
    r.shutdown()

if __name__ == '__main__':
    main()
