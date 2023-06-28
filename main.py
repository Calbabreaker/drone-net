from drone import Drone 
import argparse
import cv2
import numpy as np

parser = argparse.ArgumentParser(description="Drone controller for detecting animals and dropping nets on them.")
parser.add_argument("--fov", type=int, default=90,
                    help="The vertical FOV of the camera")
parser.add_argument("--blob-size", type=int, default=300,
                    help="How big the image that will be fed neural network will be (higher is more accurate but slower).")
parser.add_argument("--interval", type=int, default=10,
                    help="How often the to detect objects in the video feed (in number of frames).")
parser.add_argument("--min-confidence", type=float, default=0.3,
                    help="The minimum confidence that is allowed for the drone to move to (number between 0 and 1).")
parser.add_argument("--visualize", action=argparse.BooleanOptionalAction,
                    help="Whether or not to visualize the tracking points.")
parser.add_argument("--video", required=True,
                    help="The index of the video camera (/dev/videoX) or a video file.")
parser.add_argument("--video-height", type=int, 
                    help="The height the input video feed should be resized to. This mostly improves performance with --visualize. Use --blob_size instead.")
parser.add_argument("--descend-range-div", type=float, default=3,
                    help="The range calculated by dividing the screen width/height (whatever is smaller) that will make the drone descend/deploy if points move within.")
args = parser.parse_args()

TARGET_LABLES = { "bird", "cat", "cow", "dog", "horse", "sheep", "person" }

class TrackPoint:
    def __init__(self, label, confidence, bbox) -> None:
        self.label = label
        self.confidence = confidence
        self.bbox = bbox
        self.is_valid = confidence > args.min_confidence and label in TARGET_LABLES

        (x1, y1, x2, y2) = bbox
        self.center = (int((x1 + x2) / 2), int((y1 + y2) / 2))

    def draw(self, img, color):
        (x1, y1, x2, y2) = self.bbox

        if not self.is_valid:
            color = (50, 0, 0)

        # Add a text label to the image
        label = f"{self.label}: {self.confidence:.2f}"
        cv2.putText(img, label, (x1 + 5, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.circle(img, self.center, radius=10, color=color, thickness=-1)

        # Draw a bounding box around the animal
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)

# Load the pre-trained SSD model
net = cv2.dnn.readNetFromCaffe("./models/mobilenet-ssd/mobilenet-ssd.prototxt", "./models/mobilenet-ssd/mobilenet-ssd.caffemodel")
labels = []
with open('./models/mobilenet-ssd/labels.txt') as file:
    for line in file:
        labels.append(line.strip())

# net = cv2.dnn.readNetFromCaffe("./public/ssd300/deploy.prototxt", "./public/ssd300/VGG_VOC0712Plus_SSD_300x300_ft_iter_160000.caffemodel")
# labels = open("./public/ssd300/labels.txt", "r").readlines()

# video=cv2.VideoCapture("test.webm")
video_path = int(args.video) if args.video.isdigit() else args.video
video=cv2.VideoCapture(video_path)

drone = Drone(args)

def detect(img):
    width = img.shape[1]
    height = img.shape[0]
    tracker_points = []

    # Convert the image to a format the neural network can recognise
    # The parameters here are used to normalize the 8-bit color channels between 0 and 1
    blob = cv2.dnn.blobFromImage(img, 1/127.5, (args.blob_size, args.blob_size), (127.5, 127.5, 127.5), False)

    # Feed the blob through the network
    net.setInput(blob)
    detections = net.forward()

    # detections is a 4 dimensional array where [0, 0, i, ...] is the i'th dectection of the first channel of the first image
    # Loop over detections
    for i in range(detections.shape[2]):
        confidence = detections[0, 0, i, 2]
        class_id = int(detections[0, 0, i, 1])

        # The neural networks gives back the bounding box of the detection as coordinates x1, y1, x2, y2 normalized between 0 and 1
        # Multiple by width and height to get back to the screen coordinates
        bbox = detections[0, 0, i, 3:7] * np.array([width, height, width, height])
        
        tracker_points.append(TrackPoint(labels[class_id], confidence, bbox.astype("int")))

    return tracker_points

def track_video(video):
    framecount = 0
    tracker_points = []

    while cv2.waitKey(20) != ord('q'):
        # Get frame from video feed
        ok,frame=video.read()
        if not ok: 
            break

        # If specified, resize the input camera feed while keeping the aspect ratio
        # Improves performance slightly
        if args.video_height:
            aspect_ratio = frame.shape[1] / frame.shape[0]
            frame = cv2.resize(frame, (int(args.video_height * aspect_ratio), args.video_height)) 

        # frame.shape[1] = width, frame.shape[0] = height 
        center = (int(frame.shape[1] / 2), int(frame.shape[0] / 2))

        # Every few frames based on DETECT_FRAME_INTERVAL, detect objects with opencv and move the drone
        # This can't be done every frame because it is too computationally expensive
        if framecount % args.interval == 0:
            tracker_points = detect(frame)
            drone.control_drone(tracker_points, center)

        if args.visualize:
            visualize_points(frame, tracker_points, center)

        framecount += 1

def visualize_points(frame, tracker_points, center):
    # Draw all tracker points for debugging
    for point in tracker_points:
        point.draw(frame, (255, 0, 0))

    if drone.target_point != None:
        drone.target_point.draw(frame, (0, 0, 255))

    # Draw decend/deploy range
    size = min(frame.shape[1], frame.shape[0])
    cv2.circle(frame, center, int(size / args.descend_range_div), color=(0, 255, 0), thickness=2)

    cv2.imshow("Image", frame)

if __name__ == "__main__":
    track_video(video)
