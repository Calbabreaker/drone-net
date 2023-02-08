import cv2
from math import sqrt
import numpy as np

detect_frame_interval = 30

# Load the pre-trained SSD model
net = cv2.dnn.readNetFromCaffe("./models/mobilenet-ssd/mobilenet-ssd.prototxt", "./models/mobilenet-ssd/mobilenet-ssd.caffemodel")
labels = open("./models/mobilenet-ssd/labels.txt", "r").readlines()

# net = cv2.dnn.readNetFromCaffe("./public/ssd300/deploy.prototxt", "./public/ssd300/VGG_VOC0712Plus_SSD_300x300_ft_iter_160000.caffemodel")
# labels = open("./public/ssd300/labels.txt", "r").readlines()

class TrackPoint:
    def __init__(self, label, confidence, bbox) -> None:
        self.label = label
        self.confidence = confidence
        self.bbox = bbox
        self.center = (int((bbox[2] + bbox[0]) / 2), int((bbox[3] + bbox[1]) / 2))

video=cv2.VideoCapture("test.webm")
# video=cv2.VideoCapture(src=0).start()

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

def draw_point(img, point, color):
    (x1, y1, x2, y2) = point.bbox

    # Add a text label to the image
    label = f"{point.label}: {point.confidence:.2f}"
    cv2.putText(img, label, (x1 + 5, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.circle(img, point.center, radius=10, color=color, thickness=-1)

    # Draw a bounding box around the animal
    cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)

def get_closest_point(target_point, points):
    (x1, y1) = target_point.center
    closest_point = None
    closest_distance = None

    for point in points:
        (x2, y2) = point.center
        
        distance = sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        if closest_distance == None or distance < closest_distance:
            closest_distance = distance
            closest_point = point

    print("Point moved", closest_distance)
    return closest_point

def calculate_target(target_point, tracker_points):
    if len(tracker_points) == 0:
        print("Found no points. Waiting...")
        return

    if target_point == None:
        target_point = tracker_points[0]
        print("No previous point. Picking random point to follow.")
    else:
        target_point = get_closest_point(target_point, tracker_points)
    return target_point

def track_video(video):
    framecount = 0
    tracker_points = []
    target_point = None

    while cv2.waitKey(20) != ord('q'):
        ok,frame=video.read()
        if not ok: 
            break

        if framecount % detect_frame_interval == 0:
            tracker_points = detect(frame)
            target_point = calculate_target(target_point, tracker_points)

        for point in tracker_points:
            draw_point(frame, point, (255, 0, 0))

        if target_point != None:
            draw_point(frame, target_point, (0, 0, 255))

        cv2.imshow("Image", frame)
        framecount += 1

track_video(video)

# img = cv2.imread("./test2.png")
# for point in detect(img):
#     draw_point(img, point)
# cv2.imshow("Image", img)
# cv2.waitKey()
