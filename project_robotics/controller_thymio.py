import rclpy
from rclpy.node import Node
import tf_transformations

from geometry_msgs.msg import Twist, Pose
from diagnostic_msgs.msg._diagnostic_array import DiagnosticArray
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
import numpy as np
import sys
import time
from rclpy.service import Service


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.angular_vel_z= 15.
        

    def start(self):
        # Create and immediately start a timer that will regularly publish commands
        self.timer = self.create_timer(1./60, self.update_callback)
        

    def stop(self):
        # Set all velocities to zero
        cmd_vel = Twist()
        self.vel_publisher.publish(cmd_vel)

    

    def update_callback(self):
        
        
        cmd_vel = Twist()
        cmd_vel.angular.z = self.angular_vel_z  # [rad/s] it was set to 0.
        
        
        
        self.vel_publisher.publish(cmd_vel)
        


def main():
    # Initialize the ROS client library
    rclpy.init(args=sys.argv)

    # Create an instance of your node class
    node = ControllerNode()
    node.start()

    # Keep processings events until someone manually shuts down the node
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Ensure the Thymio is stopped before exiting
    node.stop()


if __name__ == '__main__':
    main()
