import math
import numpy as np
import gpiozero 
import time
import dronekit
import copy

# Altitude is in meters
ASCEND_AMOUNT = 0.7
DESCEND_AMOUNT = 1

# Radius of the Earth in meters
EARTH_RADIUS = 6378137.0

class Drone:
    def __init__(self, args) -> None:
        self.target_point = None
        self.times_within_range = 0
        self.args = args
        self.deploy_ready_start_time = None

        self.vehicle = dronekit.connect(args.address)
        print(f"INFO: Connected to {args.address}")

        self.target_position = copy.copy(self.vehicle.location.global_relative_frame)

        # Take off to safe altitude
        if self.vehicle.mode.name != "GUIDED":
            self.takeoff()
        else:
            self.set_altitude(self.args.max_altitude)

        self.wait_reach_alitude(self.args.max_altitude, True)
        print("INFO: Target altitude reached!")


    def takeoff(self):
        while not self.vehicle.is_armable:
            print("INFO: Waiting for vehicle to initialize...")
            time.sleep(1)

        # Arm the vehicle and takeoff
        self.vehicle.mode = dronekit.VehicleMode("GUIDED")
        self.vehicle.armed = True

        while not self.vehicle.armed or not self.vehicle.mode.name == "GUIDED":
            print("INFO: Waiting for vehicle to be armed...")
            time.sleep(1)

        self.vehicle.simple_takeoff(self.args.max_altitude)
        print("INFO: Starting to takeoff...")

    def land(self):
        # Set the vehicle to start landing
        self.vehicle.mode = dronekit.VehicleMode('LAND')
        while not self.vehicle.mode =="LAND":
            print("INFO: Waiting for drone to enter LAND mode.")
            time.sleep(1)

        self.wait_reach_alitude(2, False)

        # Disarm the vehicle to complete the landing process
        self.vehicle.armed = False
        print("INFO: Drone has landed and disarmed.")

    def wait_reach_alitude(self, target_altitude, is_ascent):
        while True:
            if is_ascent and self.get_altitude() > target_altitude * 0.99:
                break
            elif not is_ascent and self.get_altitude() < target_altitude * 1.01:
                break;

            print(f"TRACE: Waiting to reach altitude... (Alitude: {self.get_altitude():.2f}/{target_altitude:.2f}m)")
            time.sleep(1)

    def get_altitude(self):
        return self.vehicle.location.global_relative_frame.alt

    def get_lat_lon(self):
        position = self.vehicle.location.global_relative_frame
        return (position.lat, position.lon)

    # Move the drone to specified altitude
    # Set the wait_ascend to true (ascending) or false (descending) to wait until the drone has reached the alittude
    def set_altitude(self, target_altitude):
        # TODO: change with actual move function
        print(f"TRACE: Altitude changing to {target_altitude}")
        self.target_position.alt = min(float(target_altitude), self.args.max_altitude)
        self.vehicle.simple_goto(self.target_position)

    # Move drone to the target location x meters ahead of the current position
    def move_by(self, east_dist, north_dist):
        print(f"TRACE: Moving by ({east_dist:.2f}m, {north_dist:.2f}m)")
        (lat, lon) = self.get_lat_lon()
        lat_dist = north_dist / EARTH_RADIUS

        lon_radius = EARTH_RADIUS * math.cos(math.radians(lat)) # Longitude based on latitude
        lon_dist = east_dist / lon_radius 

        # Calculate new position in decimal degrees
        self.target_position.lat = lat + math.degrees(lat_dist)
        self.target_position.lon = lon + math.degrees(lon_dist)
        self.vehicle.simple_goto(self.target_position)

    # Calculates where to move the drone based on target_point
    # Returns whether the target_point and altitude is in the range
    def calc_move(self, screen_center):
        if self.target_point == None:
            print("WARN: Lost target point. Ascending...")
            self.set_altitude(self.get_altitude() + ASCEND_AMOUNT)
            return False
            
        (tx, ty) = self.target_point.center
        (sx, sy) = screen_center
        (width, height) = np.multiply(screen_center, 2)

        # Figure angluar extent for the distance (how much that distance convers in degrees)
        angle_x = (tx - sx) * (self.args.fov / width)
        angle_y = (ty - sy) * (self.args.fov / height)

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

        if distance < self.get_descend_range_size(width, height):
            if self.get_altitude() > self.args.deploy_altitude:
                print("INFO: Within range decending...")
                self.set_altitude(self.get_altitude() - DESCEND_AMOUNT)
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

        ready_to_deploy = self.calc_move(center)

        # If the target_point is within range for x consecutive times, DEPLOY
        if ready_to_deploy:
            if self.deploy_ready_start_time == None:
                self.deploy_ready_start_time = time.time()

            time_since_deploy_ready = time.time() - self.deploy_ready_start_time 
            print(f"INFO: Ready to deploy for {time_since_deploy_ready:.2f}/{self.args.deploy_ready_time} seconds")

            if time_since_deploy_ready > self.args.deploy_ready_time:
                self.deploy_net()
        else:
            self.deploy_ready_start_time = None

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

        print("INFO: Ascending to safe altitude...")
        self.set_altitude(self.args.max_altitude)
        self.wait_reach_alitude(self.args.max_altitude, True)
        print("INFO: Finished program exiting")
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
