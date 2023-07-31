import math
import numpy as np
import gpiozero 
import time
import os
import dronekit
import copy

# Radius of the Earth in meters
EARTH_RADIUS = 6378137.0

class Drone:
    def __init__(self, args) -> None:
        self.target_point = None
        self.times_within_range = 0
        self.args = args
        self.deploy_ready_start_time = None
        self.frames_since_lost = 0

        self.vehicle = dronekit.connect(args.address)
        print(f"INFO: Connected to {args.address}")

    def start(self):
        # Take off to safe altitude
        if self.vehicle.mode.name != "GUIDED":
            self.takeoff()
        else:
            self.reset_target_position()
            self.set_altitude(self.args.max_altitude, True)

        self.wait_reach_alitude(self.args.max_altitude, True)
        self.reset_target_position()
        print("INFO: Target altitude reached!")

    def takeoff(self):
        while not self.vehicle.is_armable:
            print("INFO: Waiting for vehicle to initialize...")
            time.sleep(1)

        # Arm the vehicle and takeoff
        self.vehicle.mode = dronekit.VehicleMode("GUIDED")
        self.vehicle.armed = True

        while not self.vehicle.armed or self.vehicle.mode.name != "GUIDED":
            print("INFO: Waiting for vehicle to be armed...")
            time.sleep(1)

        self.vehicle.simple_takeoff(self.args.max_altitude)
        print("INFO: Starting to takeoff...")

    def wait_reach_alitude(self, target_altitude, is_ascent):
        while True:
            if is_ascent and self.get_altitude() > target_altitude * 0.99:
                break
            elif not is_ascent and self.get_altitude() < target_altitude * 1.01:
                break;

            print(f"TRACE: Waiting to reach altitude... (Alitude: {self.get_altitude():.2f}/{target_altitude:.2f}m)")
            time.sleep(1)

    def reset_target_position(self):
        self.target_position = copy.copy(self.vehicle.location.global_relative_frame)

    def get_altitude(self):
        return self.vehicle.location.global_relative_frame.alt

    def get_lat_lon(self):
        position = self.vehicle.location.global_relative_frame
        return (position.lat, position.lon)

    def is_facing_down(self):
        try:
            return abs(self.vehicle.attitude.roll) < 0.02 and abs(self.vehicle.attitude.pitch) < 0.02
        except TypeError:
            return False

    # Move the drone to specified altitude
    def set_altitude(self, target_altitude, force=False):
        if not self.is_facing_down() and not force:
            return

        self.target_position.alt = np.clip(float(target_altitude), self.args.deploy_altitude, self.args.max_altitude)
        print(f"TRACE: Altitude changing to {self.target_position.alt}m")
        self.vehicle.simple_goto(self.target_position)

    # Move drone to the target location x meters ahead of the current position
    def move_by(self, x_dist, y_dist):
        if not self.is_facing_down():
            return

        print(f"TRACE: Moving by {x_dist:.2f}m, {y_dist:.2f}m")

        # Convert XY coordinates to body-relative coordinates (multiply by rotation matrix)
        heading_rad = math.radians(self.vehicle.heading)
        x_dist = x_dist * math.cos(heading_rad) - y_dist * math.sin(heading_rad)
        y_dist = x_dist * math.sin(heading_rad) + y_dist * math.cos(heading_rad)

        # Convert meters to Earth coordinates
        (lat, lon) = self.get_lat_lon()
        lat_dist = -y_dist / EARTH_RADIUS
        lon_dist =  x_dist / EARTH_RADIUS * math.cos(math.radians(lat)) # Longitude based on latitude 

        # Calculate new position in decimal degrees
        self.target_position.lat = lat + math.degrees(lat_dist)
        self.target_position.lon = lon + math.degrees(lon_dist)
        self.vehicle.simple_goto(self.target_position)

    # Calculates where to move the drone based on target_point
    # Returns whether the target_point and altitude is in the deploy range
    def calc_move(self, screen_center):
        if self.target_point == None:
            self.frames_since_lost += 1
            if self.frames_since_lost > self.args.leeway_frames:
                print("WARN: Lost target point. Ascending...")
                self.set_altitude(self.get_altitude() + self.args.ascend_amount)
                self.deploy_ready_start_time = None
            return False

        self.frames_since_lost = 0
            
        (tx, ty) = self.target_point.center
        (sx, sy) = screen_center
        (width, height) = np.multiply(screen_center, 2)

        # Figure angluar extent for the distance (how much that distance convers in degrees)
        angle_x = (tx - sx) * (self.args.fov / 2 / width)
        angle_y = (ty - sy) * (self.args.fov / 2 / height)

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
        dist_x = self.get_altitude() * math.tan(math.radians(angle_x))
        dist_y = self.get_altitude() * math.tan(math.radians(angle_y))
        self.move_by(dist_x, dist_y)

        distance = get_distance(self.target_point.center, screen_center)

        if distance < self.get_descend_range_size(width, height) and self.is_facing_down():
            if self.get_altitude() > self.args.deploy_altitude * 1.1:
                print("INFO: Within range decending...")
                self.set_altitude(self.get_altitude() - self.args.descend_amount)
            else:
                return True
        else:
            self.deploy_ready_start_time = None

        return False

    def control_drone(self, tracker_points, center):
        # Gets the closest point to the center
        self.target_point = get_closest_point(center, tracker_points)

        self.ready_to_deploy = self.calc_move(center)

        # If the target_point is within range for a number of consecutive seconds, DEPLOY
        if self.ready_to_deploy:
            if self.deploy_ready_start_time == None:
                self.deploy_ready_start_time = time.time()

            time_since_deploy_ready = time.time() - self.deploy_ready_start_time 
            print(f"INFO: Ready to deploy for {time_since_deploy_ready:.2f}/{self.args.deploy_ready_time} seconds")

            if time_since_deploy_ready >= self.args.deploy_ready_time:
                self.deploy_net()

    def get_descend_range_size(self, width, height):
        size = min(width, height)
        return int(size / self.args.descend_range_div)

    def deploy_net(self):
        print(f"INFO: DEPLOY NET (pin {self.args.servo_pin})")
        try:
            servo = gpiozero.AngularServo(self.args.servo_pin, min_pulse_width=0.0006, max_pulse_width=0.0023)
            servo.angle = 90
            time.sleep(2)
        except gpiozero.BadPinFactory:
            print("WARN: Unable to load pins, still continuing")

        self.vehicle.mode = dronekit.VehicleMode("RTL")
        while self.vehicle.armed:
            print("TRACE: Returning to home location (RTL mode)...")
            time.sleep(1)

        print("INFO: Drone succesfully landed")
        self.vehicle.close()
        print("INFO: Program finished exiting")
        os._exit(0)

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
