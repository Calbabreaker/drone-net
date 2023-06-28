from math import sqrt, tan
import numpy as np
import sys
import subprocess

ASCEND_AMOUNT = 0.2
DESCEND_AMOUNT = 0.5
DEPLOY_ALTITUDE = 2
MAX_ALTITUDE = 50
TIMES_IN_RANGE_TO_DEPLOY = 4

class Drone:
    def __init__(self, args) -> None:
        self.altitude = 4
        self.position = (100, 100)
        self.target_point = None
        self.times_within_range = 0
        self.args = args

    def ascend_by(self, altitude_delta):
        # TODO: change with actual move function
        self.altitude = min(self.altitude + altitude_delta, MAX_ALTITUDE)
        print(f"DRONE: Altitude changed to {self.altitude}")

    def move_by(self, move_delta):
        # TODO: change with actual move function
        self.position = np.add(self.position, move_delta)
        print(f"DRONE: Moving by {move_delta}")

    # Calculates where to move the drone based on target_point
    # Returns whether the target_point and altitude is in the range
    def calc_move(self, screen_center):
        if self.target_point == None:
            print("Lost target point. Ascending...")
            self.ascend_by(ASCEND_AMOUNT)
            return False
            
        (tx, ty) = self.target_point.center
        (sx, sy) = screen_center
        (width, height) = np.multiply(screen_center, 2)

        # Figure angluar extent for the distance (how much that distance convers in degrees)
        angle_x = (tx - sx) * (self.args.fov / (width))
        angle_y = (ty - sy) * (self.args.fov / (height))

        #  a = altitude
        #  θ = angle
        #  x = distance to move
        #  d = a * tan(θ)
        #
        #     camera 
        #      /|
        #     /θ|
        #    /  | a
        #   /___|
        #     d
        dist_x = self.altitude * tan(angle_x)
        dist_y = self.altitude * tan(angle_y)
        self.move_by((dist_x, dist_y))

        distance = get_distance(self.target_point.center, screen_center)
        print("Distance remaning", distance)

        if distance < (width / self.args.descend_range_div):
            if self.altitude > DEPLOY_ALTITUDE:
                print("Within range decending...")
                self.ascend_by(-DESCEND_AMOUNT)
            else:
                return True

        return False

    def control_drone(self, tracker_points, center):
        # if len(tracker_points) == 0:
        #     # If we haven't detected any points, reset the target_point so that we can recalibrate
        #     self.target_point = None
        # elif self.target_point == None:
        #     print("Picking random point as target.")
        #     self.target_point = tracker_points[0]
        # else:
        #     # Get the closest point to the previous point we're targeting to make sure we're following the same thing (hopefully)
        #     self.target_point = get_closest_point(self.target_point, tracker_points)

        # Gets the closest point to the center
        self.target_point = get_closest_point(center, tracker_points)

        within_deploy_range = self.calc_move(center)

        # If the target_point is within range for x consecutive times, DEPLOY
        if within_deploy_range:
            self.times_within_range += 1
            print(f"Within deploy range {self.times_within_range}/{TIMES_IN_RANGE_TO_DEPLOY} times")

            if self.times_within_range == TIMES_IN_RANGE_TO_DEPLOY:
                self.deploy_net()
        else:
            self.times_within_range = 0

    def deploy_net(self):
        subprocess.Popen([sys.executable, "deploy.py", str(self.args.servo_pin)], start_new_session=True)
        exit()

def get_distance(center_a, center_b):
    (x1, y1) = center_a
    (x2, y2) = center_b
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def get_closest_point(position, points):
    closest_point = None
    closest_distance = None

    for point in points:
        if not point.is_valid:
            continue

        distance = get_distance(position, point.center)
        if closest_distance == None or distance < closest_distance:
            closest_distance = distance
            closest_point = point

    return closest_point
