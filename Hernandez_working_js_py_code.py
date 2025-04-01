import time
import roslibpy
from lights_function import play_lights
import threading

# ROS Connection Details
ip = '192.168.8.104'
port = 9012
robot_name = 'echo'

# Establish connection to ROS
ros_node = roslibpy.Ros(host=ip, port=port)
ros_node.run()

print(f"Connected to ROS: {ros_node.is_connected}")

# Robot class
class RobotController:
    def __init__(self, ros_node, robot_name):
        self.ros_node = ros_node
        self.robot_name = robot_name

        self.stop_event = threading.Event()

        self.ir_flag = True
        self.Kp = 0.01  # Proportional control gain for lane-keeping

        # ROS publishers
        self.led_pub = roslibpy.Topic(self.ros_node, f'/{self.robot_name}/cmd_lightring', 'irobot_create_msgs/LightringLeds')
        self.drive_pub = roslibpy.Topic(self.ros_node, f'/{self.robot_name}/cmd_vel', 'geometry_msgs/Twist')
        self.audio_pub = roslibpy.Topic(self.ros_node, f'/{self.robot_name}/cmd_audio', 'irobot_create_msgs/AudioNoteVector')

        self.ir_topic = roslibpy.Topic(self.ros_node, f'/{self.robot_name}/ir_intensity', 'irobot_create_msgs/IrIntensityVector')
        self.ir_topic.subscribe(self.callback_ir)

        self.mode_sub_topic = roslibpy.Topic(self.ros_node, f'/{self.robot_name}/mode', 'std_msgs/msg/String')
        self.mode_sub_topic.subscribe(self.mode_callback)

        # Create and start threads
        self.led_thread = threading.Thread(target=self.leds, daemon=True)
        self.audio_thread = threading.Thread(target=self.audio, daemon=True)
        self.auto_mode_thread = threading.Thread(target=self.auto_mode, daemon=True)

        self.mode = "manual"  # Default mode
        self.manual_mode = False
        self.mode_lock = threading.Lock()  # Lock to ensure thread-safe access to self.mode

                # Create list for all threads
        self.threads = [
            self.audio_thread,
            self.led_thread,
            self.auto_mode_thread
        ]
        # Print to verify mode initialization
        print(f"Mode initialized as: {self.mode}")

    def get_commands(self):
        while not self.stop_event.is_set():
            with self.mode_lock:  # Ensure thread-safe access to self.mode
                if self.mode == "manual":  # manual mode on website
                    self.manual_mode = not self.manual_mode
                    self.idle_mode = False
                    self.autonomous_mode = False
                    print(f"Manual mode {'activated' if self.manual_mode else 'deactivated'}")
                    time.sleep(0.3)  # Debounce delay
                    self.color = 'Red'

                if self.mode == "auto":
                    self.idle_mode = not self.idle_mode
                    self.manual_mode = False
                    self.autonomous_mode = False
                    print(f"RACE mode activated with {self.mode}")
                    time.sleep(0.3)
                    self.color = 'Green'

            time.sleep(0.1)  # Loop at 5 Hz

    def callback_ir(self, message):  # Read IR data
        self.values = [reading['value'] for reading in message['readings']]

    def mode_callback(self, message):
        with self.mode_lock:  # Ensure thread-safe access to self.mode
            self.mode = message['data']
        print(f"Mode {self.mode}")
        if self.mode == "auto":
            print(f'{self.mode} mode activated')
        elif self.mode == "manual":
            print('Robot is in manual mode, uses buttons')

    def auto_mode(self):  # Autonomous mode (Lane-Keeping)
        while not self.stop_event.is_set():
            if self.mode == "auto":
                # Read IR sensor values
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
                error_right = 0 - right_values  
                error_center_right = 0 - center_value_right
                error_center_left = 0 - center_value_left
                error_center = 0 - center_value

                # Compute proportional angular velocity adjustment
                error_detection = -0.007 * (error_right - error_left) + 0.005 * (error_center_left) + 0.005 * (-error_center_right)
                print(f"Error detection: {error_detection}")
                # Proportional control for linear speed
                Kp_speed = 0.01  # Adjust based on tuning
                min_speed = 0.1
                max_speed = 3.0
                speed = max_speed - Kp_speed * center_values
                speed = max(min_speed, min(speed, max_speed))  # Clamping speed

                if center_values > 35:
                    speed = 0.1  # Reduce speed significantly when very close to obstacles

                # Apply correction and drive forward
                drive_message = {
                    "linear": {"x": speed, "y": 0.0, "z": 0.0},  
                    "angular": {"x": 0.0, "y": 0.0, "z": error_detection}  
                }

                self.drive_pub.publish(roslibpy.Message(drive_message))

                time.sleep(0.01)  # Loop at 10Hz

    def leds(self):  # Control light ring
        while not self.stop_event.is_set():
            with self.mode_lock:  # Ensure thread-safe access to self.mode
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

    def audio(self):  # Play audio
        last_mode = None  # Track the last executed mode
        while not self.stop_event.is_set():
            # Detect the current mode
            current_mode = None
            notes = []
            sleep_duration = 0

            with self.mode_lock:  # Ensure thread-safe access to self.mode
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

            # Play notes only if the mode has changed
            if current_mode and current_mode != last_mode:
                audio_message = {'notes': notes, 'append': False}
                self.audio_pub.publish(roslibpy.Message(audio_message))
                time.sleep(sleep_duration)
                last_mode = current_mode  # Update last mode to prevent re-triggering

            time.sleep(0.1)
 
    def start_threads(self):
        """Starts control and LED threads."""
        for t in self.threads:
            t.start()
    
    #stop running all the threads for the robot
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
        #when the Keyboard is pressed while in the terminal, the robot will stop all threads and shutdown
    except KeyboardInterrupt:
        print("\nManual interrupt received. Shutting down...")
        robot_controller.end_threads()
        ros_node.terminate()
        print("Shutdown complete.")