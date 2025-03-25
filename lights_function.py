import roslibpy
import time
import random

class Lights:
    
    def __init__ (self, ros_node, robot_name, color):
        #ros_node: name of node that initializes connection to ROS2 library
        #robot_name: name of robot
        #color: color of lights, input arguments include ('RGB', 'Random', 'Red', 'Orange', 'Yellow', 'Green', 'Blue', 'Violet', 'White', 'Off')
        self.led_pub = roslibpy.Topic(ros_node, f'/{robot_name}/cmd_lightring', 'irobot_create_msgs/LightringLeds')
        self.ros_node = ros_node
        self.robot_name = robot_name
        self.color = color
        self.RGB = [[{"red": 255, "green": 0, "blue": 0}]*6,  # Red
                    [{"red": 0, "green": 255, "blue": 0}]*6,  # Green
                    [{"red": 0, "green": 0, "blue": 255}]*6   # Blue
                    ]
        self.Rand = [{"red": random.randint(0, 255), "green": random.randint(0, 255), "blue": random.randint(0, 255)}]*6
        self.Red = [{"red": 255, "green": 0, "blue": 0}]*6
        self.Orange = [{"red": 255, "green": 165, "blue": 0}]*6
        self.Yellow = [{"red": 255, "green": 255, "blue": 0}]*6
        self.Green = [{"red": 0, "green": 255, "blue": 0}]*6
        self.Blue = [{"red": 0, "green": 0, "blue": 255}]*6
        self.Violet = [{"red": 148, "green": 0, "blue": 211}]*6
        self.White = [{"red": 255, "green": 255, "blue": 255}]*6
        self.Off = [{"red": 0, "green": 0, "blue": 0}]*6
    
    def fetch_color(self):
        if self.color == 'RGB':
            color_des = self.RGB
        elif self.color == 'Random':
            color_des = self.Rand
        elif self.color == 'Red':
            color_des = self.Red
        elif self.color == 'Orange':
            color_des = self.Orange
        elif self.color == 'Yellow':
            color_des = self.Yellow
        elif self.color == 'Green':
            color_des = self.Green
        elif self.color == 'Blue':
            color_des = self.Blue
        elif self.color == 'Violet':
            color_des = self.Violet
        elif self.color == 'White':
            color_des = self.White
        elif self.color == 'Off':
            color_des = self.Off
        return color_des
    
    def init(self):
        # Initialize the LED publisher
        self.led_pub.advertise()

    def pub(self, color):
        # Compose a LightringLeds message
        lightring_led_message = {"leds": color, "override_system": True}
        lightring_ros_msg = roslibpy.Message(lightring_led_message)

        # Publish the message
        if self.color == 'RGB':
            for colors in color:
                self.led_pub.publish(roslibpy.Message({"leds": colors, "override_system": True}))
                time.sleep(1)
        else:
            self.led_pub.publish(lightring_ros_msg)
    
# Main loop
def play_lights(ros_node, robot_name, color):
    #create instance of Lights
    lightring = Lights(ros_node, robot_name, color)
    #fetch desired color pattern
    color_pattern = lightring.fetch_color()
    #initialize lights
    lightring.init()
    #publish lights
    lightring.pub(color_pattern)