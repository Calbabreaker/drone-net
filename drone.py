import numpy as np
from math import sqrt, tan

TIMES_IN_RANGE_TO_DEPLOY = 4
DEPLOY_ALTITUDE = 10
DESCEND_RANGE = 100
DETECT_FRAME_INTERVAL = 10
ASCEND_AMOUNT = 10
CAMERA_FOV = 90

class Drone:
    def __init__(self) -> None:
        self.altitude = 100
        self.position = (100, 100)
        self.target_point = None
        self.times_within_range = 0

    def ascend_by(self, altitude_delta):
        self.altitude += altitude_delta
        print(f"DRONE: Ascending to {self.altitude}")

    def move_by(self, move_delta):
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

        # Figure angluar extent for the distance (how much that distance convers in degrees)
        angle_x = (tx - sx) * (CAMERA_FOV / (sx * 2))
        angle_y = (ty - sy) * (CAMERA_FOV / (sy * 2))

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

        if distance < DESCEND_RANGE:
            # decend down ????
            print("Within range decending...")
            self.ascend_by(-ASCEND_AMOUNT)

            return self.altitude < DEPLOY_ALTITUDE
        else:
            return False

    def control_drone(self, tracker_points, center):
        if len(tracker_points) == 0:
            # If we haven't detected any points, reset the target_point so that we can recalibrate
            self.target_point = None
        elif self.target_point == None:
            print("Picking random point as target.")
            self.target_point = tracker_points[0]
        else:
            # Get the closest point to the previous point we're targeting to make sure we're following the same thing (hopefully)
            self.target_point = get_closest_point(self.target_point, tracker_points)

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
        print("DEPLOYED")
        # stop and go back up
        exit()

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

