from drone import Drone 
import argparse
import cv2
import time
import numpy as np
import threading

parser = argparse.ArgumentParser(description="Drone controller for detecting animals and dropping nets on them.")
parser.add_argument("--fov", type=int, default=45,
                    help="The vertical FOV of the camera")
parser.add_argument("--blob-size", type=int, default=500,
                    help="How big the image that will be fed neural network will be (higher is more accurate but slower).")
parser.add_argument("--frame-time", type=float, default=1,
                    help="Target time between each frame detection.")
parser.add_argument("--min-confidence", type=float, default=0.3,
                    help="The minimum confidence that is allowed for the drone to move to (number between 0 and 1).")
parser.add_argument("--show-visualize", action=argparse.BooleanOptionalAction,
                    help="Whether or not to show the the tracking points in a OpenCV window.")
parser.add_argument("--save-visualize", action=argparse.BooleanOptionalAction,
                    help="Whether or not to save the visualization of the tracking points to output.avi.")
parser.add_argument("--video", required=True,
                    help="The index of the video camera (/dev/videoX) or a video file.")
parser.add_argument("--servo-pin", type=int, required=True,
                    help="The pin of the servo(s) used for deploying.")
parser.add_argument("--descend-range-div", type=float, default=6,
                    help="The range calculated by dividing the screen width/height (whatever is smaller) that will make the drone descend/deploy if points move within.")
parser.add_argument("--deploy-altitude", type=float, default=10,
                    help="The altitude in meters where the drone will stop descending and be able to deploy the net")
parser.add_argument("--max-altitude", type=float, default=40,
                    help="The maxium and takeoff altitude of the drone")
parser.add_argument("--address", type=str, default="127.0.0.1:14550",
                    help="Address for drone connection")
parser.add_argument("--ascend-amount", type=float, default=1,
                    help="How much to ascend each time the program loses the tracker point in meters.")
parser.add_argument("--descend-amount", type=float, default=4,
                    help="How much to descend each time the tracker point goes inside the descend/deploy range.")
parser.add_argument("--deploy-ready-time", type=float, default=1,
                    help="Amount of time needed in seconds for the drone in the state where it should deploy before deploying.")
parser.add_argument("--leeway-frames", type=int, default=2,
                    help="Amount of frames allowed for dectetion to be lost before ascending/resetting deploying time.")
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
            color = (0, 0, 0)

        # Add a text label to the image
        label = f"{self.label}: {self.confidence:.2f}"
        cv2.putText(img, label, (x1 + 5, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.circle(img, self.center, radius=10, color=color, thickness=-1)

        # Draw a bounding box around the animal
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)

# Load the pre-trained SSD model
net = cv2.dnn.readNetFromCaffe("./models/mobilenet-ssd/mobilenet-ssd.prototxt", "./models/mobilenet-ssd/mobilenet-ssd.caffemodel")
labels = []
with open("./models/mobilenet-ssd/labels.txt") as file:
    for line in file:
        labels.append(line.strip())

video_path = int(args.video) if args.video.isdigit() else args.video
video = cv2.VideoCapture(video_path)
if args.save_visualize:
    dimensions = (int(video.get(3)),int(video.get(4)))
    video_out = cv2.VideoWriter("output.avi", cv2.VideoWriter_fourcc(*"MJPG"), 30, dimensions)

def get_center(frame):
    # frame.shape[1] is width, frame.shape[0] is height 
    return (int(frame.shape[1] / 2), int(frame.shape[0] / 2))

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

    # detections is a 4 dimensional array where [0, 0, i, ...] is the i'th detection of the first channel of the first image
    # Loop over detections
    for i in range(detections.shape[2]):
        confidence = detections[0, 0, i, 2]
        class_id = int(detections[0, 0, i, 1])

        # The neural networks gives back the bounding box of the detection as coordinates x1, y1, x2, y2 normalized between 0 and 1
        # Multiple by width and height to get back to the screen coordinates
        bbox = detections[0, 0, i, 3:7] * np.array([width, height, width, height])
        
        tracker_points.append(TrackPoint(labels[class_id], confidence, bbox.astype("int")))

    return tracker_points

current_frame = None
tracker_points = []
drone = Drone(args)

def track_thread():
    global current_frame
    global tracker_points

    drone.start()
    time.sleep(0.1)

    if current_frame is None:
        print("WARN: No frame obtained from video, detecting will not execute")

    while current_frame is not None and not drone.deployed:
        last_frame_time = time.time()

        center = get_center(current_frame)
        tracker_points = detect(current_frame)
        drone.control_drone(tracker_points, center)

        frame_time = time.time() - last_frame_time
        print(f"TRACE: Frame took {frame_time} seconds")
        if frame_time < args.frame_time:
            # Wait remaining amount of time for the target frame_time
            time.sleep(args.frame_time - frame_time)

def track_video():
    global current_frame
    global tracker_points

    # We need a different since the drone controller uses blocks the thread
    thread = threading.Thread(target=track_thread)
    thread.start()

    while not drone.deployed and cv2.waitKey(20) != ord("q"):
        # Get frame from video feed
        ok,frame=video.read()
        if not ok: 
            break

        current_frame = frame

        if args.show_visualize or args.save_visualize:
            visualize_points(frame)

    current_frame = None
    thread.join()
    print("INFO: Program finished exiting")

def visualize_points(frame):
    # Draw all tracker points for debugging
    for point in tracker_points:
        point.draw(frame, (255, 0, 0))

    if drone.target_point != None:
        drone.target_point.draw(frame, (0, 0, 255))

    # Draw decend/deploy range
    center = get_center(frame)
    descend_range_size = drone.get_descend_range_size(frame.shape[1],frame.shape[0])
    cv2.circle(frame, center, descend_range_size, color=(0, 255 if drone.is_facing_down() else 80, 0), thickness=2)

    if args.save_visualize:
        video_out.write(frame)
    if args.show_visualize:
        cv2.imshow("Image", frame)

track_video()
