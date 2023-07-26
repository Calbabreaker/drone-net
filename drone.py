import math
import numpy as np
import sys
import time
import subprocess
import dronekit

# Altitude is in feet
ASCEND_AMOUNT = 0.2
DESCEND_AMOUNT = 0.5
TIMES_IN_RANGE_TO_DEPLOY = 4

# Radius of the Earth in meters
EARTH_RADIUS = 6378137.0


class Drone:
    def __init__(self, args) -> None:
        self.target_point = None
        self.times_within_range = 0
        self.args = args

        self.vehicle = dronekit.connect(args.address)
        print(f"Connected to {args.address}")

        if self.vehicle.mode.name != "GUIDED":
            self.takeoff()

    def takeoff(self):
        while not self.vehicle.is_armable:
            print("Waiting for vehicle to initialize...")
            time.sleep(1)

        # Arm the vehicle and takeoff
        self.vehicle.mode = dronekit.VehicleMode("GUIDED")
        self.vehicle.armed = True

        while not self.vehicle.armed or not self.vehicle.mode.name == "GUIDED":
            print("Waiting for vehicle to be armed...")
            time.sleep(1)

        self.vehicle.simple_takeoff(self.args.max_altitude)
        print("Starting to ascend.")

        while self.get_altitude() < self.args.max_altitude:
            print(f"Waiting to reach altitude... (Alitude: {self.get_altitude()}/{self.args.max_altitude}ft)")
            time.sleep(1)

        print("Target altitude reached!!")


    def land(self):
        # Set the vehicle to start landing
        self.vehicle.mode = dronekit.VehicleMode('LAND')
        while not self.vehicle.mode =="LAND":
            print("Waiting for drone to enter LAND mode.")
            time.sleep(1)

        # Wait until near ground
        while self.get_altitude() > 1:
            print(f"Waiting for vehicle to land... (Altitude: {self.get_altitude()}ft)")
            time.sleep(1)

        # Disarm the vehicle to complete the landing process
        self.vehicle.armed = False
        print("Drone has landed and disarmed.")

    def get_altitude(self):
        return self.vehicle.location.global_relative_frame.alt

    def get_lat_lon(self):
        position = self.vehicle.location.global_relative_frame
        return (position.lat, position.lon)

    def ascend_by(self, altitude_delta):
        # TODO: change with actual move function
        print(f"DRONE: Altitude changing to {self.get_altitude()}")
        (lat, lon) = self.get_lat_lon()
        target_altitude = min(self.get_altitude() + altitude_delta, self.args.max_altitude)
        target_position = dronekit.LocationGlobalRelative(lat, lon, target_altitude)
        self.vehicle.simple_goto(target_position)

    # Move drone to the target location x meters ahead of the current position
    def move_by(self, north_dist, east_dist):
        print(f"DRONE: Moving by {(north_dist, east_dist)}")
        (lat, lon) = self.get_lat_lon()
        lat_dist = north_dist / EARTH_RADIUS

        lon_radius = EARTH_RADIUS * math.cos(math.radians(lat)) # Longitude based on latitude
        lon_dist = east_dist / lon_radius 

        # Calculate new position in decimal degrees
        new_lat = lat + math.degrees(lat_dist)
        new_lon = lon + math.degrees(lon_dist)
        target_position =  dronekit.LocationGlobalRelative(new_lat, new_lon, self.get_altitude())
        self.vehicle.simple_goto(target_position)

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
        #  d = a * tan(θ) (distance to move)
        #
        #     camera 
        #      /|
        #     /θ|
        #    /  | a
        #   /___|
        #     d
        dist_x = self.get_altitude() * math.tan(angle_x)
        dist_y = self.get_altitude() * math.tan(angle_y)
        self.move_by(dist_y, dist_x)

        distance = get_distance(self.target_point.center, screen_center)
        print("Distance remaning", distance)

        if distance < self.get_descend_range_size(width, height):
            if self.get_altitude() < self.args.deploy_altitude:
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

    def get_descend_range_size(self, width, height):
        size = min(width, height)
        return int(size / self.args.descend_range_div)

    def deploy_net(self):
        subprocess.Popen([sys.executable, "deploy.py", str(self.args.servo_pin)], start_new_session=True)
        exit()

def get_distance(center_a, center_b):
    (x1, y1) = center_a
    (x2, y2) = center_b
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

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
