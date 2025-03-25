import time
import roslibpy

class RobotController:
    def __init__(self, ip, port, robot_name):
        self.ip = ip
        self.port = port
        self.robot_name = robot_name
        self.mode = 'manual'
        
        # Establish connection to ROS
        self.ros = roslibpy.Ros(host=self.ip, port=self.port)
        self.ros.run()
        print(f"Connected to ROS: {self.ros.is_connected}")
        
        # Topics for IR sensors and robot velocity
        self.cmd_vel_topic = roslibpy.Topic(self.ros, f'/{self.robot_name}/cmd_vel', 'geometry_msgs/msg/Twist')
        self.mode_sub_topic = roslibpy.Topic(self.ros, f'/{self.robot_name}/mode', 'std_msgs/msg/String')

        # Subscribe to mode changes
        self.mode_sub_topic.subscribe(self.mode_callback)

    def mode_callback(self, message):
        self.mode = message['data']
        print(f"Mode {self.mode}")
        if self.mode == "auto":
            print(f'{self.mode} mode activated')
        elif self.mode == "manual":
            print('Robot is in manual mode, will not move')

    def move_forward(self):
        linear_x = 0
        angular_z = 0

        if self.mode == 'auto':
            linear_x = 0.15
            angular_z = 0.00
        
        print(f"Moving in {self.mode} mode")
        # Publish the velocity command
        twist = roslibpy.Message({'linear': {'x': linear_x, 'y': 0, 'z': 0}, 'angular': {'x': 0, 'y': 0, 'z': angular_z}})
        self.cmd_vel_topic.publish(twist)
    
    def stop(self):
        twist = roslibpy.Message({'linear': {'x': 0, 'y': 0, 'z': 0}, 'angular': {'x': 0, 'y': 0, 'z': 0}})
        self.cmd_vel_topic.publish(twist)
    def run(self):
        try:
            while self.ros.is_connected:
                self.move_forward()
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("Disconnecting from ROS...")
            self.cmd_vel_topic.unadvertise()
            self.ros.close()
            print("Disconnected.")

# Usage example:
if __name__ == "__main__":
    robot_controller = RobotController(ip='192.168.8.104', port=9012, robot_name='india')
    robot_controller.run()