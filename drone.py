import numpy as np
from math import sqrt

TIMES_IN_RANGE_TO_DEPLOY = 4
DEPLOY_ALTITUDE = 10
DESCEND_RANGE = 100
DETECT_FRAME_INTERVAL = 1
ASCEND_AMOUNT = 10

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
        print(f"DRONE: Moving to {self.position}")

    def calc_move(self, target_point, screen_center):
        if self.target_point == None:
            print("Lost target point. Ascending...")
            self.ascend_by(ASCEND_AMOUNT)
            return False
            
        (tx, ty) = self.target_point.center
        (sx, sy) = screen_center
        # Need to adjust this
        move_scale = (self.altitude / 1000) ** 2
        move_amount = ((tx - sx) * move_scale, (sy - ty) * move_scale)
        print("Moving drone relatively", move_amount)
        self.move_by(move_amount)

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

        within_deploy_range = self.calc_move(self.target_point, center)

        # If the target_point is within range for x consecutive times, DEPLOY
        if within_deploy_range:
            self.times_within_range += 1
            print(f"Within deploy range {self.times_within_range}/{TIMES_IN_RANGE_TO_DEPLOY} times")

            if self.times_within_range == TIMES_IN_RANGE_TO_DEPLOY:
                deploy_net()
        else:
            self.times_within_range = 0

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

