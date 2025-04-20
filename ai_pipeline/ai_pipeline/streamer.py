import rclpy as r
import cv_bridge as c
import subprocess as s
from rclpy.node import Node
from sensor_msgs.msg import Image

class Streamer(Node):
    def __init__(self):
        super(Streamer, self).__init__(node_name='streamer')
        self.declare_parameter('width', 512)
        self.declare_parameter('height', 512)
        self.declare_parameter('fps', 30)
        self.declare_parameter('ip_address', '127.0.0.1')
        self.declare_parameter('port', 8554)
        self.declare_parameter('stream_id', 'stream')
        width = self.get_parameter('width').get_parameter_value().integer_value
        height = self.get_parameter('height').get_parameter_value().integer_value
        fps = self.get_parameter('fps').get_parameter_value().integer_value
        ip_address = self.get_parameter('ip_address').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value
        stream_id = self.get_parameter('stream_id').get_parameter_value().string_value
        self.rtsp_url = f'rtsp://{ip_address}:{port}/{stream_id}'
        command = [
            'ffmpeg',
            '-re',
            '-f', 'rawvideo',
            '-pix_fmt', 'bgr24',
            '-s', f'{width}x{height}', # Adjust resolution as needed
            '-r', f'{fps}',
            '-i', '-',
            '-f', 'rtsp',
            '-rtsp_transport', 'tcp',
            '-c:v', 'libx264',
            '-preset', 'ultrafast',
            '-tune', 'zerolatency',
            '-q', '5',
            '-an',
            '-x264-params', 'keyint=30:scenecut=0',
            '-g', '30',
            self.rtsp_url
        ]
        self.ffmpeg_process = s.Popen(command, stdin=s.PIPE)
        self.converter = c.CvBridge()
        self.create_subscription(Image, f'stream', self.callback)

    def callback(self, stream_message: Image):
        image = self.converter.imgmsg_to_cv2(stream_message, desired_encoding='bgr8')
        try:
            self.ffmpeg_process.stdin.write(image.tobytes())
            self.ffmpeg_process.stdin.flush()
        except Exception as e:
            self.get_logger().error(f"{e}")

    def __del__(self):
        if hasattr(self, 'ffmpeg_process') and self.ffmpeg_process:
            self.ffmpeg_process.stdin.close()
            self.ffmpeg_process.wait()

def main(args=None):
    r.init(args=args)
    streamer = Streamer()
    r.spin(streamer)
    streamer.destroy_node()
    r.shutdown()

if __name__ == '__main__':
    main()
