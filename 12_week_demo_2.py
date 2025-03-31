import time
import roslibpy
import pygame
from lights_function import play_lights
import threading
import math

# Initialize pygame and joystick control
pygame.init()
pygame.joystick.init()
if pygame.joystick.get_count() == 0:
    print("No joystick detected. Please connect a joystick and restart.")
    exit(1)

# Connect to the ROS bridge
ros_node = roslibpy.Ros(host='192.168.8.104', port=9012)
ros_node.run()

robot_name = 'echo'

# Joystick class
class Joystick:
    def __init__(self):
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.stop_event = threading.Event()

        # State variables
        self.manual_mode = False
        self.idle_mode = True
        self.autonomous_mode = False
        self.armed = False
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.color = 'Off'
        self.blink = False

        # Create and start thread
        self.thread = threading.Thread(target=self.get_commands, daemon=True)
        self.thread.start()

    def get_commands(self):
        while not self.stop_event.is_set():
            pygame.event.pump()  # Process joystick events

            if self.joystick.get_button(0):  # "A" button
                self.manual_mode = not self.manual_mode
                self.idle_mode = False
                self.autonomous_mode = False
                print(f"Manual mode {'activated' if self.manual_mode else 'deactivated'}")
                time.sleep(0.3)  # Debounce delay

            if self.joystick.get_button(2):  # "X" button
                self.idle_mode = not self.idle_mode
                self.manual_mode = False
                self.autonomous_mode = False
                print(f"Secondary mode {'activated' if self.idle_mode else 'deactivated'}")
                time.sleep(0.3)

            if self.joystick.get_button(1):  # "B" button
                self.autonomous_mode = not self.autonomous_mode
                self.manual_mode = False
                self.idle_mode = False
                print(f"Autonomous mode {'activated' if self.autonomous_mode else 'deactivated'}")
                time.sleep(0.3)

            if self.joystick.get_button(4):  # Left bumper
                self.armed = not self.armed
                print(f"Robot {'armed' if self.armed else 'disarmed'}")
                time.sleep(0.3)

            # Determine movement & LED state
            if self.manual_mode:
                self.linear_x = -self.joystick.get_axis(1)  # Invert Y-axis for forward/backward
                self.angular_z = -self.joystick.get_axis(0)  # X-axis for rotation
                self.color = 'Green'
            elif self.idle_mode:
                self.linear_x = 0.0
                self.angular_z = 0.0
                self.color = 'Blue'
            elif self.autonomous_mode:
                self.color = 'Yellow'

            self.blink = self.armed  # Blink if armed
            if self.armed == False:
                self.linear_x = 0
                self.angular_z = 0

            time.sleep(0.1)  # Loop at 5 Hz

    def stop(self):
        self.stop_event.set()
        self.thread.join()
        self.joystick.quit()

# Robot class
class RobotController:
    def __init__(self, joystick):
        self.joystick = joystick
        self.stop_event = threading.Event()

        self.ir_flag = True
        self.Kp = 0.007  # Proportional control gain for lane-keeping

        # ROS publishers
        self.led_pub = roslibpy.Topic(ros_node, f'/{robot_name}/cmd_lightring', 'irobot_create_msgs/LightringLeds')
        self.drive_pub = roslibpy.Topic(ros_node, f'/{robot_name}/cmd_vel', 'geometry_msgs/Twist')
        self.audio_pub = roslibpy.Topic(ros_node, f'/{robot_name}/cmd_audio', 'irobot_create_msgs/AudioNoteVector')
        self.ir_topic = roslibpy.Topic(ros_node, f'/{robot_name}/ir_intensity', 'irobot_create_msgs/IrIntensityVector')
        self.ir_topic.subscribe(self.callback_ir)

        # Create and start threads
        self.drive_thread = threading.Thread(target=self.drive, daemon=True)
        self.led_thread = threading.Thread(target=self.leds, daemon=True)
        self.audio_thread = threading.Thread(target=self.audio, daemon=True)
        self.auto_mode_thread = threading.Thread(target=self.auto_mode, daemon=True)
        self.drive_thread.start()
        self.led_thread.start()
        self.audio_thread.start()
        self.auto_mode_thread.start()
    
    def callback_ir(self, message): # Read IR data
        self.values = [reading['value'] for reading in message['readings']]

    def auto_mode(self):  # Autonomous mode (Lane-Keeping)
        while not self.stop_event.is_set():
            if self.joystick.autonomous_mode:
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

                # Compute proportional angular velocity adjustment
                angular_correction = -self.Kp * (error_right - error_left) 
                error_detection = -0.007 * (error_right - error_left) + 0.005 * (error_center_left) + 0.005 * (-error_center_right)

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


    def drive(self): # Manual mode
        while not self.stop_event.is_set():
            if self.joystick.manual_mode:
                if self.joystick.armed == False:
                    drive_message = {
                    "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
                    }
                    self.drive_pub.publish(roslibpy.Message(drive_message))
                elif self.joystick.armed == True:
                    drive_message = {
                    "linear": {"x": self.joystick.linear_x, "y": 0.0, "z": 0.0},
                    "angular": {"x": 0.0, "y": 0.0, "z": self.joystick.angular_z}
                    }
                    self.drive_pub.publish(roslibpy.Message(drive_message))
            
            time.sleep(0.2)  # 10Hz

    def leds(self): # Control light ring
        while not self.stop_event.is_set():
            play_lights(ros_node, robot_name, self.joystick.color)
            if self.joystick.armed:
                time.sleep(0.5)  # Blink on
                play_lights(ros_node, robot_name, 'Off')
                time.sleep(0.5)  # Blink off
            else:
                time.sleep(1)  # Keep the LED color

    def audio(self): # Play audio
        last_mode = None  # Track the last executed mode

        while not self.stop_event.is_set():
            # Detect the current mode
            current_mode = None
            notes = []
            sleep_duration = 0

            if self.joystick.manual_mode:
                current_mode = "manual"
                notes = [
                    {'frequency': 600, 'max_runtime': {'sec': 0, 'nanosec': int(5e8)}},
                    {'frequency': 750, 'max_runtime': {'sec': 0, 'nanosec': int(5e8)}}
                ]
                sleep_duration = 1
            elif self.joystick.idle_mode:
                current_mode = "idle"
                notes = [
                    {'frequency': 600, 'max_runtime': {'sec': 0, 'nanosec': int(5e8)}},
                    {'frequency': 450, 'max_runtime': {'sec': 0, 'nanosec': int(5e8)}}
                ]
                sleep_duration = 1

            elif self.joystick.autonomous_mode:
                current_mode = "autonomous"
                notes = [
                    {'frequency': 600, 'max_runtime': {'sec': 0, 'nanosec': int(3e8)}},
                    {'frequency': 750, 'max_runtime': {'sec': 0, 'nanosec': int(3e8)}},
                    {'frequency': 900, 'max_runtime': {'sec': 0, 'nanosec': int(3e8)}}
                ]
                sleep_duration = 0.9
            
            elif self.joystick.armed == True:
                current_mode = 'armed'
                notes = [{'frequency': 300, 'max_runtime': {'sec': 3, 'nanosec': 0}}]
                sleep_duration = 3

            # Play notes only if the mode has changed
            if current_mode and current_mode != last_mode:
                audio_message = {'notes': notes, 'append': False}
                self.audio_pub.publish(roslibpy.Message(audio_message))
                time.sleep(sleep_duration)
                last_mode = current_mode  # Update last mode to prevent re-triggering

            time.sleep(0.1)

    def stop(self):
        self.stop_event.set()
        self.drive_thread.join()
        self.led_thread.join()
        self.audio_thread.join()
        self.auto_mode_thread.join()
        self.cleanup()

    def cleanup(self):
        play_lights(ros_node, robot_name, 'Off')
        self.drive_pub.publish(roslibpy.Message({"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}))
        self.led_pub.unadvertise()
        self.drive_pub.unadvertise()
        self.audio_pub.unadvertise()
        self.ir_topic.unsubscribe()


# Main loop
if __name__ == "__main__":
    try:
        joystick = Joystick()
        robot = RobotController(joystick)

        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    raise KeyboardInterrupt  

            time.sleep(0.1)  # 10Hz loop

    except KeyboardInterrupt:
        print("\nShutting down...")
        robot.stop()
        joystick.stop()
        pygame.quit()
        ros_node.terminate()
        print("Shutdown complete.")
