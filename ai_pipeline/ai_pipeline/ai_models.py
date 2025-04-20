import cv2 as cv
from ultralytics import YOLO

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
        self.model = YOLO(model_name=weights)

    def predict(self, image):
        bboxes = self.model(image, conf=self.conf, iou=self.iou)[0]
        results = []
        for xmin, ymin, xmax, ymax, conf, c in bboxes.boxes.data.cpu().numpy().tolist():
            results.append([int(xmin), int(ymin), int(xmax), int(ymax), conf, bboxes.names[int(c), None]])
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
