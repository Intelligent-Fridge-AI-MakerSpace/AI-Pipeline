import os
import cv2 as cv
import rclpy as r
import cv_bridge as c
import ament_index_python as aip
from rclpy.node import Node
from ultralytics import YOLO
from sensor_msgs.msg import Image
from ai_msgs.msg import Camera, Model

class AIModel:
    def __init__(self):
        self.model = None

    def predict(self, image=None):
        result = None
        return result

    def draw(self, image=None, result=None):
        return image

class YoloModel(AIModel):
    def __init__(self, weights='yolo11n.pt', conf=0.5, iou=0.5, use_gpu=True):
        super(YoloModel, self).__init__()
        self.conf = conf
        self.iou = iou
        self.use_gpu = use_gpu
        self.model = YOLO(model=os.path.join(aip.get_package_share_directory('ai_pipeline'), 'weights', weights), verbose=False)

    def predict(self, image):
        bboxes = self.model(image, conf=self.conf, iou=self.iou)[0]
        results = []
        for xmin, ymin, xmax, ymax, conf, c in bboxes.boxes.data.cpu().numpy().tolist():
            results.append([int(xmin), int(ymin), int(xmax), int(ymax), conf, bboxes.names[int(c)], None])
        return results

    def draw(self, image, result):
        for xmin, ymin, xmax, ymax, conf, c, i in result:
            image = cv.rectangle(image, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (0, 255, 0), 2)
            image = cv.putText(image, f'{c} (ID: {i})', (int(xmin), int(ymin)-10), cv.FONT_HERSHEY_PLAIN, 1, (255, 0, 255), 1)
        return image

class BarcodeReader(AIModel):
    def __init__(self):
        super(BarcodeReader, self).__init__()
        self.model = cv.barcode.BarcodeDetector()

    def predict(self, image):
        _, text, points, _ = self.model.detectAndDecodeMulti(image)
        points = cv.boundingRect(points)
        results = []
        for text, (xmin, ymin, width, height) in zip(text, points):
            results.append([int(xmin), int(ymin), int(xmin + width), int(ymin + height), 0.0, text, None])
        return results

    def draw(self, image, result):
        for xmin, ymin, xmax, ymax, _, c, i in result:
            image = cv.rectangle(image, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (0, 255, 0), 2)
            image = cv.putText(image, f'{c} (ID: {i})', (int(xmin), int(ymin)-10), cv.FONT_HERSHEY_PLAIN, 1, (255, 0, 255), 1)
        return image

class ModelProcessor(Node):
    def __init__(self):
        super(ModelProcessor, self).__init__(node_name='model_processor')
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('model_name', 'yolo')
        self.declare_parameter('conf', 0.5)
        self.declare_parameter('iou', 0.5)
        self.declare_parameter('use_gpu', True)
        self.declare_parameter('iou_threshold', 0.9)
        self.declare_parameter('tracker_type', 'median_flow')
        self.tracker_types = {
            "csrt": cv.legacy.TrackerCSRT_create,
            "kcf": cv.legacy.TrackerKCF_create,
            "boosting": cv.legacy.TrackerBoosting_create,
            "mil": cv.legacy.TrackerMIL_create,
            "tld": cv.legacy.TrackerTLD_create,
            "median_flow": cv.legacy.TrackerMedianFlow_create,
            "mosse": cv.legacy.TrackerMOSSE_create
        }
        self.tracker_type = self.get_parameter('tracker_type').get_parameter_value().string_value
        self.convertor = c.CvBridge()
        if self.get_parameter('model_name').get_parameter_value().string_value == 'yolo':
            self.model = YoloModel(weights='best.pt', conf=self.get_parameter('conf').get_parameter_value().double_value, iou=self.get_parameter('iou').get_parameter_value().double_value, use_gpu=self.get_parameter('use_gpu').get_parameter_value().bool_value)
        else:
            self.model = BarcodeReader()
        self.trackers = {}
        self.new_tracker_id = 0
        self.result_publisher = self.create_publisher(Model, 'results', 10)
        self.stream_publisher = self.create_publisher(Image, 'stream', 10)
        self.door_open = False
        self.create_subscription(Camera, f'camera', self.callback, 10)

    def calculate_iou(self, xmin1, ymin1, xmax1, ymax1, xmin2, ymin2, xmax2, ymax2):
        x1 = max(xmin1, xmin2)
        y1 = max(ymin1, ymin2)
        x2 = min(xmax1, xmax2)
        y2 = min(ymax1, ymax2)
        area = max(0, x2 - x1) * max(0, y2 - y1)
        true_area = (xmax1 - xmin1) * (ymax1 - ymin1)
        pred_area = (xmax2 - xmin2) * (ymax2 - ymin2)
        iou = area / float(true_area + pred_area - area)
        return iou

    def initialize_new_tracker(self, image, xmin, ymin, xmax, ymax):
        tracker = self.tracker_types[self.tracker_type]()
        tracker.init(image, (xmin, ymin, xmax-xmin, ymax-ymin))
        return tracker

    def infer_and_track(self, image):
        final_results = []
        results = self.model.predict(image)
        for xmin_true, ymin_true, xmax_true, ymax_true, conf, cls, _ in results:
            matched = False
            for tracker_id in self.trackers:
                tracker, _ = self.trackers[tracker_id]
                successfully_tracked, bbox = tracker.update(image)
                if successfully_tracked:
                    xmin_pred, ymin_pred, w, h = bbox
                    xmax_pred, ymax_pred = xmin_pred + w, ymin_pred + h
                    if self.calculate_iou(xmin_true, ymin_true, xmax_true, ymax_true, xmin_pred, ymin_pred, xmax_pred, ymax_pred) < self.get_parameter('iou_threshold').get_parameter_value().double_value:
                        self.trackers[tracker_id] = [self.initialize_new_tracker(image, xmin_true, ymin_true, xmax_true, ymax_true), cls]
                    final_results.append([xmin_true, ymin_true, xmax_true, ymax_true, conf, cls, tracker_id])
                    matched = True
                    break
            if not matched:
                self.trackers[self.new_tracker_id] = [self.initialize_new_tracker(image, xmin_true, ymin_true, xmax_true, ymax_true), cls]
                final_results.append([xmin_true, ymin_true, xmax_true, ymax_true, conf, cls, self.new_tracker_id])
                self.new_tracker_id = self.new_tracker_id + 1
        return final_results

    def track(self, image):
        results = []
        for tracker_id in self.trackers:
            tracker, cls = self.trackers[tracker_id]
            successfully_tracked, bbox = tracker.update(image)
            if successfully_tracked:
                xmin, ymin, w, h = bbox
                results.append([xmin, ymin, xmin+w, ymin+h, 0.0, cls, tracker_id])
        return results

    def callback(self, camera_message: Camera):
        image = self.convertor.imgmsg_to_cv2(camera_message.image, desired_encoding='bgr8')
        if camera_message.status in ['START', 'CONTINUE', 'END']:
            results = self.infer_and_track(image)
            image = self.model.draw(image, results)
            result_message = Model()
            result_message.header = camera_message.header
            result_message.camera_id = self.get_parameter('camera_id').get_parameter_value().integer_value
            result_message.objects = len(results)
            result_message.xmins = [xmin for xmin, _, _, _, _, _, _ in results]
            result_message.ymins = [ymin for _, ymin, _, _, _, _, _ in results]
            result_message.xmaxs = [xmax for _, _, xmax, _, _, _, _ in results]
            result_message.ymaxs = [ymax for _, _, _, ymax, _, _, _ in results]
            result_message.confs = [conf for _, _, _, _, conf, _, _ in results]
            result_message.tags = [cls for _, _, _, _, _, cls, _ in results]
            result_message.ids = [i for _, _, _, _, _, _, i in results]
            result_message.status = camera_message.status
            self.result_publisher.publish(result_message)
        else:
            results = self.track(image)
            image = self.model.draw(image, results)
        stream_message = Image()
        stream_message = self.convertor.cv2_to_imgmsg(image, encoding='bgr8')
        self.stream_publisher.publish(stream_message)

def main(args=None):
    r.init(args=args)
    model_processor = ModelProcessor()
    r.spin(model_processor)
    model_processor.destroy_node()
    r.shutdown()

if __name__ == '__main__':
    main()
