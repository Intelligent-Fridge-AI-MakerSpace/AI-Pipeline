import rclpy as r
import cv_bridge as c
import subprocess as s
from rclpy.node import Node
from sensor_msgs.msg import Image

class Streamer(Node):
    def __init__(self):
        super(Streamer, self).__init__(node_name='streamer')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 640)
        self.declare_parameter('fps', 30)
        self.declare_parameter('ip_address', '127.0.0.1')
        self.declare_parameter('port', 8554)
        self.declare_parameter('stream_id', 'stream')
        width = self.get_parameter('width').get_parameter_value().integer_value
        height = self.get_parameter('height').get_parameter_value().integer_value
        self.fps = self.get_parameter('fps').get_parameter_value().integer_value
        ip_address = self.get_parameter('ip_address').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value
        stream_id = self.get_parameter('stream_id').get_parameter_value().string_value
        self.rtsp_url = f'rtsp://{ip_address}:{port}/{stream_id}'
        command = [
            'ffmpeg',
            '-re',
            '-f', 'rawvideo',
            '-pix_fmt', 'bgr24',
            '-s', f'{width}x{height}',
            '-r', f'{self.fps}',
            '-i', '-',
            '-an',
            '-c:v', 'libx264',
            '-pix_fmt', 'yuv420p',
            '-preset', 'ultrafast',
            '-tune', 'zerolatency',
            '-x264-params', 'keyint=30:scenecut=0',
            '-g', '30',
            '-f', 'rtsp',
            '-rtsp_transport', 'tcp',
            # '-q', '30',
            self.rtsp_url
        ]
        self.ffmpeg_process = s.Popen(command, stdin=s.PIPE)
        self.converter = c.CvBridge()
        self.create_subscription(Image, 'stream', self.callback, 10)

    def callback(self, stream_message: Image):
        image = self.converter.imgmsg_to_cv2(stream_message, desired_encoding='bgr8')
        # self.get_logger().info(f"{image.shape}")
        try:
            self.ffmpeg_process.stdin.write(image.tobytes())
            # self.ffmpeg_process.stdin.flush()
        except Exception as e:
            self.get_logger().error(f"ERROR: {e}")

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

######################################################################################################################################

# import time
# import rclpy as r
# import queue as q
# import cv_bridge as c
# import threading as t
# import subprocess as s
# from rclpy.node import Node
# from sensor_msgs.msg import Image

# class StreamerThread(t.Thread):
#     def __init__(self, exit_event: t.Event, camera_id: int, width: int, height: int, fps: int, rtsp_url: str, image_queue: q.Queue):
#         super(StreamerThread, self).__init__(name=f'streamer_thread_{camera_id}', daemon=False)
#         self.exit_event = exit_event
#         self.image_queue = image_queue
#         command = [
#             'ffmpeg',
#             # '-fflags', 'nobuffer',
#             # '-re',
#             '-f', 'rawvideo',
#             '-pix_fmt', 'bgr24',
#             '-s', f'{width}x{height}', # Adjust resolution as needed
#             '-r', f'{fps}',
#             '-i', '-',
#             '-f', 'rtsp',
#             '-rtsp_transport', 'tcp',
#             '-c:v', 'libx264',
#             '-preset', 'ultrafast',
#             '-tune', 'zerolatency',
#             '-q', '5',
#             '-an',
#             '-x264-params', 'keyint=1:scenecut=0',
#             '-g', '30',
#             rtsp_url
#         ]
#         self.ffmpeg_process = s.Popen(command, stdin=s.PIPE)

#     def run(self):
#         while not self.exit_event.is_set():
#             try:
#                 image = self.image_queue.get(timeout=1)
#                 self.ffmpeg_process.stdin.write(image.tobytes())
#                 self.ffmpeg_process.stdin.flush()
#                 time.sleep(1.0 / self.fps)
#             except Exception as e:
#                 pass

#     def join(self):
#         if hasattr(self, 'ffmpeg_process') and self.ffmpeg_process:
#             self.ffmpeg_process.stdin.close()
#             self.ffmpeg_process.wait()
#         self.join()

# class Streamer(Node):
#     def __init__(self):
#         super(Streamer, self).__init__(node_name='streamer')
#         self.declare_parameter('camera_id', 0)
#         self.declare_parameter('width', 640)
#         self.declare_parameter('height', 640)
#         self.declare_parameter('fps', 30)
#         self.declare_parameter('ip_address', '127.0.0.1')
#         self.declare_parameter('port', 8554)
#         self.declare_parameter('stream_id', 'stream')
#         self.exit_event = t.Event()
#         width = self.get_parameter('width').get_parameter_value().integer_value
#         height = self.get_parameter('height').get_parameter_value().integer_value
#         fps = self.get_parameter('fps').get_parameter_value().integer_value
#         ip_address = self.get_parameter('ip_address').get_parameter_value().string_value
#         port = self.get_parameter('port').get_parameter_value().integer_value
#         stream_id = self.get_parameter('stream_id').get_parameter_value().string_value
#         rtsp_url = f'rtsp://{ip_address}:{port}/{stream_id}'
#         self.converter = c.CvBridge()
#         self.image_queue = q.Queue(maxsize=10)
#         self.streamer_thread = StreamerThread(self.exit_event, self.get_parameter('camera_id').get_parameter_value().integer_value, width, height, fps, rtsp_url, self.image_queue)
#         self.streamer_thread.start()
#         self.create_subscription(Image, 'stream', self.callback, 10)

#     def callback(self, stream_message: Image):
#         image = self.converter.imgmsg_to_cv2(stream_message, desired_encoding='bgr8')
#         if not self.image_queue.full():
#             self.image_queue.put(image)
#         # try:
#         #     self.ffmpeg_process.stdin.write(image.tobytes())
#         #     self.ffmpeg_process.stdin.flush()
#         # except Exception as e:
#         #     self.get_logger().error(f"ERROR: {e}")

#     def __del__(self):
#         self.exit_event.set()
#         self.streamer_thread.join()
#         # if hasattr(self, 'ffmpeg_process') and self.ffmpeg_process:
#         #     self.ffmpeg_process.stdin.close()
#         #     self.ffmpeg_process.wait()

# def main(args=None):
#     r.init(args=args)
#     streamer = Streamer()
#     r.spin(streamer)
#     streamer.destroy_node()
#     r.shutdown()

# if __name__ == '__main__':
#     main()
