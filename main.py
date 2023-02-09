import cv2
from math import sqrt
import numpy as np

class TrackPoint:
    def __init__(self, label, confidence, bbox) -> None:
        self.label = label
        self.confidence = confidence
        self.bbox = bbox

        (x1, y1, x2, y2) = bbox
        self.center = (int((x1 + x2) / 2), int((y1 + y2) / 2))
        self.times_within_range = 0

    def draw(self, img, color):
        (x1, y1, x2, y2) = self.bbox

        # Add a text label to the image
        label = f"{self.label}: {self.confidence:.2f}"
        cv2.putText(img, label, (x1 + 5, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.circle(img, self.center, radius=10, color=color, thickness=-1)

        # Draw a bounding box around the animal
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)

detect_frame_interval = 30
times_in_range_to_deploy = 4
deploy_altitude = 10

# Load the pre-trained SSD model
net = cv2.dnn.readNetFromCaffe("./models/mobilenet-ssd/mobilenet-ssd.prototxt", "./models/mobilenet-ssd/mobilenet-ssd.caffemodel")
labels = open("./models/mobilenet-ssd/labels.txt", "r").readlines()

# net = cv2.dnn.readNetFromCaffe("./public/ssd300/deploy.prototxt", "./public/ssd300/VGG_VOC0712Plus_SSD_300x300_ft_iter_160000.caffemodel")
# labels = open("./public/ssd300/labels.txt", "r").readlines()

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

def get_distance(center_a, center_b):
    (x1, y1) = center_a
    (x2, y2) = center_b
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def get_closest_point(target_point, points):
    closest_point = None
    closest_distance = None

    for point in points:
        distance = get_distance(target_point.center, point.center)
        if closest_distance == None or distance < closest_distance:
            closest_distance = distance
            closest_point = point

    return closest_point

def deploy_net():
    print("DEPLOYED")
    # stop and go back up
    exit()

def move_drone(target_point, screen_center):
    if target_point == None:
        print("Lost target point. Ascending...")
        return False
        
    (tx, ty) = target_point.center
    (sx, sy) = screen_center
    altitude = 5
    # Need to adjust this
    move_scale = altitude ** 2
    move_amount = ((tx - sx) * move_scale, (sy - ty) * move_scale)
    print("Moving drone relatively", move_amount)

    distance = get_distance(target_point.center, screen_center)
    print("Distance remaning", distance)

    if distance < 500:
        # decend down ????
        print("Within range decending...")

        return altitude < deploy_altitude
    else:
        return False


def track_video(video):
    framecount = 0
    tracker_points = []
    target_point = None
    times_within_range = 0

    while cv2.waitKey(20) != ord('q'):
        # Get frame from video feed
        ok,frame=video.read()
        if not ok: 
            break

        # Every few frames based on detect_frame_interval, detect objects with opencv and move the drone
        # This can't be done every frame because it is too computationally expensive
        if framecount % detect_frame_interval == 0:
            tracker_points = detect(frame)

            if len(tracker_points) == 0:
                # If we haven't detected any points, reset the target_point so that we can recalibrate
                target_point = None
            elif target_point == None:
                print("Picking random point as target.")
                target_point = tracker_points[0]
            else:
                # Get the closest point to the previous point we're targeting to make sure we're following the same thing (hopefully)
                target_point = get_closest_point(target_point, tracker_points)

            width = frame.shape[1]
            height = frame.shape[0]
            within_deploy_range = move_drone(target_point, (width / 2, height / 2))

            # If the target_point is within range for x consecutive times, DEPLOY
            if within_deploy_range:
                times_within_range += 1
                print(f"Within deploy range {times_within_range}/{times_in_range_to_deploy} times")

                if times_within_range == times_in_range_to_deploy:
                    deploy_net()
                

        # Draw all tracker points for debugging
        for point in tracker_points:
            point.draw(frame, (255, 0, 0))

        if target_point != None:
            target_point.draw(frame, (0, 0, 255))

        cv2.imshow("Image", frame)
        framecount += 1

track_video(video)

# img = cv2.imread("./test2.png")
# for point in detect(img):
#     draw_point(img, point)
# cv2.imshow("Image", img)
# cv2.waitKey()
