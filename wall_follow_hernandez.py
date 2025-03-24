import time
import roslibpy

# ROS Connection Details
ip = '192.168.8.104'
port = 9012
robot_name = 'foxtrot'

# Establish connection to ROS
ros = roslibpy.Ros(host=ip, port=port)
ros.run()
print(f"Connected to ROS: {ros.is_connected}")

# Topics for IR sensors and robot velocity
ir_topic = roslibpy.Topic(ros, f'/{robot_name}/ir_intensity', 'irobot_create_msgs/IrIntensityVector')
cmd_vel_topic = roslibpy.Topic(ros, f'/{robot_name}/cmd_vel', 'geometry_msgs/msg/Twist')

# Proportional gain
Kp = 0.01  # Adjust this value for better performance

# Set the desired distance from both walls (in meters)
desired_distance = 20  

def ir_to_distance(intensity):
    """Convert IR intensity to estimated distance (meters)."""
    if intensity > 2:
        return (5000 / intensity) / 100  # Convert cm to meters
    else:
        return None  # Ignore invalid readings

def callback_ir(message):
    """Uses IR sensors to control the robot's movement in a walled path."""
    if 'readings' in message:
        ir_readings = message['readings']

        if len(ir_readings) == 7:
            # Convert intensities to distances
            left_distance = ir_readings[0].get('value', 0)   # Sensor 1 (leftmost)
            right_distance = ir_readings[6].get('value', 0)  # Sensor 7 (rightmost)
            front_distance = ir_readings[3].get('value', 0)  # Sensor 4 (middle/front)

            # Validate sensor readings
            if left_distance is None or right_distance is None:
                print("Invalid readings, skipping control update.")
                return

            # Calculate error (difference between left and right distances)
            error = (left_distance - right_distance)

            # Proportional control output
            angular_z = -Kp * error  # Negative sign because robot should steer in the opposite direction of the error

            # Forward speed adjustment based on front distance (slow down if too close)
            linear_x = 0.15  # Default speed
            if front_distance > desired_distance:
                linear_x = 0.05  # Slow down if an obstacle is ahead
            else:
                linear_x = 0.15

            # Create velocity command message
            twist = roslibpy.Message({
                'linear': {'x': linear_x, 'y': 0, 'z': 0},
                'angular': {'x': 0, 'y': 0, 'z': angular_z}
            })

            # Publish the velocity command
            cmd_vel_topic.publish(twist)

            # Print status
            print(f"Front: {front_distance:.2f} | Left: {left_distance:.2f} | Right: {right_distance:.2f} | Error: {error:.2f} | Angular Z: {angular_z:.2f}")

        else:
            print(f"Warning: Expected 7 IR readings, but received {len(ir_readings)}")
    else:
        print("No IR readings received.")

# Subscribe to IR topic
ir_topic.subscribe(callback_ir)

# Keep running until interrupted
try:
    while ros.is_connected:
        time.sleep(1)
except KeyboardInterrupt:
    print("Disconnecting from ROS...")
    ir_topic.unsubscribe()
    cmd_vel_topic.unadvertise()
    ros.close()
    print("Disconnected.")