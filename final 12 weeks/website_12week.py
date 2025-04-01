import time
import roslibpy
from lights_function import play_lights
import threading

# ROS Connection Details
ip = '192.168.8.104'
port = 9012
robot_name = 'omega'

# Establish connection to ROS
ros_node = roslibpy.Ros(host=ip, port=port)

try:
    ros_node.run()
    if not ros_node.is_connected:
        raise ConnectionError("Failed to connect to ROS at {}:{}".format(ip, port))
    print(f"Connected to ROS: {ros_node.is_connected}")
except Exception as e:
    print(f"Error connecting to ROS: {e}")
    exit(1)

# Robot class
class RobotController:
    def __init__(self, ros_node, robot_name):
        self.ros_node = ros_node
        self.robot_name = robot_name

        self.stop_event = threading.Event()

        self.ir_flag = True
        self.Kp = 0.01  # Proportional control gain for lane-keeping

        # ROS publishers and subscribers
        try:
            self.led_pub = roslibpy.Topic(self.ros_node, f'/{self.robot_name}/cmd_lightring', 'irobot_create_msgs/LightringLeds')
            self.drive_pub = roslibpy.Topic(self.ros_node, f'/{self.robot_name}/cmd_vel', 'geometry_msgs/Twist')
            self.audio_pub = roslibpy.Topic(self.ros_node, f'/{self.robot_name}/cmd_audio', 'irobot_create_msgs/AudioNoteVector')

            self.ir_topic = roslibpy.Topic(self.ros_node, f'/{self.robot_name}/ir_intensity', 'irobot_create_msgs/IrIntensityVector')
            self.ir_topic.subscribe(self.callback_ir)

            self.mode_sub_topic = roslibpy.Topic(self.ros_node, f'/{self.robot_name}/mode', 'std_msgs/msg/String')
            self.mode_sub_topic.subscribe(self.mode_callback)

        except Exception as e:
            print(f"Error initializing ROS topics: {e}")
            ros_node.terminate()
            exit(1)

        # Create and start threads
        self.led_thread = threading.Thread(target=self.leds, daemon=True)
        self.audio_thread = threading.Thread(target=self.audio, daemon=True)
        self.auto_mode_thread = threading.Thread(target=self.auto_mode, daemon=True)

        self.mode = "manual"  # Default mode
        self.manual_mode = False
        self.mode_lock = threading.Lock()  # Lock to ensure thread-safe access to self.mode

        self.threads = [
            self.audio_thread,
            self.led_thread,
            self.auto_mode_thread
        ]

        print(f"Mode initialized as: {self.mode}")

    def get_commands(self):
        while not self.stop_event.is_set():
            with self.mode_lock:
                if self.mode == "manual":
                    self.manual_mode = True
                    self.idle_mode = False
                    self.autonomous_mode = False
                    print(f"Manual mode activated")
                    time.sleep(0.3)
                    self.color = 'Red'

                if self.mode == "auto":
                    self.idle_mode = True
                    self.manual_mode = False
                    self.autonomous_mode = False
                    print(f"Autonomous mode activated with {self.mode}")
                    time.sleep(0.3)
                    self.color = 'Green'

            time.sleep(0.1)  # Loop at 5 Hz

    def callback_ir(self, message):
        self.values = [reading['value'] for reading in message['readings']]

    def mode_callback(self, message):
        with self.mode_lock:
            self.mode = message['data']
        print(f"Mode changed to {self.mode}")
        if self.mode == "auto":
            print(f'{self.mode} mode activated')
        elif self.mode == "manual":
            print('Robot is in manual mode')

    def auto_mode(self):
        while not self.stop_event.is_set():
            if self.mode == "auto":
                values = self.values
                left_value = values[0]  # Adjust index based on sensor configuration
                left_value2 = values[1]
                left_value3 = values[2]
                right_value = values[6]
                right_value2 = values[5]
                right_value3 = values[4]
                center_value = values[3]

                left_values = left_value + left_value2 + left_value3
                right_values = right_value + right_value2 + right_value3
                center_values = right_value3 + left_value3 + center_value
                center_value_right = right_value3 + center_value
                center_value_left = left_value3 + center_value
                

                # Compute errors
                error_left = 0 - left_values
                error_right = 0- right_values  
                error_center= 0-center_value
                error_center_right = 0 - center_value_right
                error_center_left = 0 - center_value_left

                # Compute proportional angular velocity adjustment
                angular_correction = -self.Kp * (error_right - error_left) 
                error_detection = -0.01* (error_right - error_left) + 0.007*(error_center)

                # Proportional control for linear speed
                Kp_speed = 0.04  # Adjust based on tuning
                min_speed = 0.5
                max_speed = 3.0
                speed = max_speed - Kp_speed * center_values
                speed = max(min_speed, min(speed, max_speed))  # Clamping speed

                if center_values > 20:
                    
                    drive_message = {
                    "linear": {"x": speed*0.5, "y": 0.0, "z": 0.0},  
                    "angular": {"x": 0.0, "y": 0.0, "z": error_detection}  
                    }

                    self.drive_pub.publish(roslibpy.Message(drive_message))
                else:

                # Apply correction and drive forward
                    drive_message = {
                        "linear": {"x": speed, "y": 0.0, "z": 0.0},  
                        "angular": {"x": 0.0, "y": 0.0, "z": angular_correction}  
                    }

                    self.drive_pub.publish(roslibpy.Message(drive_message))

                time.sleep(0.01)  # Loop at 10Hz


    def leds(self):
        while not self.stop_event.is_set():
            with self.mode_lock:
                if self.mode == "manual":
                    play_lights(self.ros_node, self.robot_name, 'Red')
                    time.sleep(0.5)  # Blink on
                    play_lights(self.ros_node, self.robot_name, 'Off')
                    time.sleep(0.5)  # Blink off
                    print(f"LEDs in {self.mode} mode")
                elif self.mode == "auto":
                    play_lights(self.ros_node, self.robot_name, 'Green')
                    print(f"LEDs in {self.mode} mode")
                    time.sleep(1)  # Keep the LED color

            time.sleep(0.1)

    def audio(self):
        last_mode = None
        while not self.stop_event.is_set():
            current_mode = None
            notes = []
            sleep_duration = 0

            with self.mode_lock:
                if self.mode == 'manual':
                    current_mode = "manual"
                    notes = [
                        {'frequency': 600, 'max_runtime': {'sec': 0, 'nanosec': int(5e8)}},
                        {'frequency': 750, 'max_runtime': {'sec': 0, 'nanosec': int(5e8)}}
                    ]
                    sleep_duration = 1

                elif self.mode == 'auto':
                    current_mode = "autonomous"
                    notes = [
                        {'frequency': 600, 'max_runtime': {'sec': 0, 'nanosec': int(3e8)}},
                        {'frequency': 750, 'max_runtime': {'sec': 0, 'nanosec': int(3e8)}},
                        {'frequency': 900, 'max_runtime': {'sec': 0, 'nanosec': int(3e8)}}
                    ]
                    sleep_duration = 0.9

            if current_mode and current_mode != last_mode:
                audio_message = {'notes': notes, 'append': False}
                self.audio_pub.publish(roslibpy.Message(audio_message))
                time.sleep(sleep_duration)
                last_mode = current_mode

            time.sleep(0.1)

    def start_threads(self):
        """Starts control and LED threads."""
        for t in self.threads:
            t.start()

    def end_threads(self):
        """Stops all threads gracefully."""
        self.stop_event.set()
        for t in self.threads:
            t.join()
        print("All threads stopped.")

# Main loop
if __name__ == "__main__":
    robot_controller = RobotController(ros_node, robot_name)
    robot_controller.start_threads()
    try:
        while True:
            time.sleep(1)  # Keep running indefinitely
    except KeyboardInterrupt:
        print("\nManual interrupt received. Shutting down...")
        robot_controller.end_threads()
        ros_node.terminate()
        print("Shutdown complete.")
