# Import DroneKit-Python
from dronekit import connect, VehicleMode

connection_string = "/dev/ttyACM0"
# Connect to the Vehicle.
print(("Connecting to vehicle on: %s" % (connection_string,)))
vehicle = connect(connection_string)

# Get some vehicle attributes (state)
print("Get some vehicle attribute values:")
print(" GPS: %s" % vehicle.gps_0)
print(" Battery: %s" % vehicle.battery)
print(" Last Heartbeat: %s" % vehicle.last_heartbeat)
print(" Is Armable?: %s" % vehicle.is_armable)
print(" System status: %s" % vehicle.system_status.state)
print(" Mode: %s" % vehicle.mode.name)    # settable

# Close vehicle object before exiting script
vehicle.close()

