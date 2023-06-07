import cv2
import numpy as np
from drone import Drone, DETECT_FRAME_INTERVAL, DESCEND_RANGE

class TrackPoint:
    def __init__(self, label, confidence, bbox) -> None:
        self.label = label
        self.confidence = confidence
        self.bbox = bbox

        (x1, y1, x2, y2) = bbox
        self.center = (int((x1 + x2) / 2), int((y1 + y2) / 2))

    def draw(self, img, color):
        (x1, y1, x2, y2) = self.bbox

        # Add a text label to the image
        label = f"{self.label}: {self.confidence:.2f}"
        cv2.putText(img, label, (x1 + 5, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.circle(img, self.center, radius=10, color=color, thickness=-1)

        # Draw a bounding box around the animal
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)

# Load the pre-trained SSD model
net = cv2.dnn.readNetFromCaffe("./models/mobilenet-ssd/mobilenet-ssd.prototxt", "./models/mobilenet-ssd/mobilenet-ssd.caffemodel")
labels = open("./models/mobilenet-ssd/labels.txt", "r").readlines()

# net = cv2.dnn.readNetFromCaffe("./public/ssd300/deploy.prototxt", "./public/ssd300/VGG_VOC0712Plus_SSD_300x300_ft_iter_160000.caffemodel")
# labels = open("./public/ssd300/labels.txt", "r").readlines()

# video=cv2.VideoCapture("test.webm")
video=cv2.VideoCapture(2)

drone = Drone()

def detect(img):
    width = img.shape[1]
    height = img.shape[0]
    tracker_points = []

    # Get the blob from the image
    blob = cv2.dnn.blobFromImage(img, 0.007843, (300, 300), (127.5, 127.5, 127.5), False)

    # Feed the blob through the network
    net.setInput(blob)
    detections = net.forward()

    # detections is a 4 dimensional array where [0, 0, i, :] is the i'th dections of the first channel of the first image
    # Loop over detections
    for i in range(detections.shape[2]):
        confidence = detections[0, 0, i, 2]
        
        # Filter detections with low confidence
        if confidence >= 0.2:
            # Get the class label and coordinates of the bounding box
            class_id = int(detections[0, 0, i, 1])
            bbox = detections[0, 0, i, 3:7] * np.array([width, height, width, height])
            
            tracker_points.append(TrackPoint(labels[class_id], confidence, bbox.astype("int")))

    return tracker_points

def track_video(video):
    framecount = 0
    tracker_points = []

    while cv2.waitKey(20) != ord('q'):
        # Get frame from video feed
        ok,frame=video.read()
        frame = cv2.resize(frame, (1080, 720)) 
        if not ok: 
            break

        center = (int(frame.shape[1] / 2), int(frame.shape[0] / 2))

        # Every few frames based on DETECT_FRAME_INTERVAL, detect objects with opencv and move the drone
        # This can't be done every frame because it is too computationally expensive
        if framecount % DETECT_FRAME_INTERVAL == 0:
            tracker_points = detect(frame)
            drone.control_drone(tracker_points, center)

        # Draw all tracker points for debugging
        for point in tracker_points:
            point.draw(frame, (255, 0, 0))

        if drone.target_point != None:
            drone.target_point.draw(frame, (0, 0, 255))

        # Draw decend/deploy range
        decend_range_vec = (DESCEND_RANGE, DESCEND_RANGE)
        cv2.rectangle(frame, np.subtract(center, decend_range_vec), np.add(center, decend_range_vec), (0, 255, 0), 2)

        cv2.imshow("Image", frame)
        framecount += 1

if __name__ == "__main__":
    track_video(video)

# img = cv2.imread("./test2.png")
# for point in detect(img):
#     draw_point(img, point)
# cv2.imshow("Image", img)
# cv2.waitKey()
