from cv2 import KeyPoint
import rclpy
from rclpy.node import Node
import tf_transformations

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range

import sys

from math import pow, atan2, sqrt, sin, cos
import time
import random 

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class ControllerNode(Node):

    FORWARD=0
    ROTATE=1
    ADJUST=2
    AUAI=3

    NO_WALL = 0.05

    def __init__(self,gui):
        super().__init__('controller_node')

        self.gui = gui

        self.no_wall = ControllerNode.NO_WALL
        
        # Create attributes to store odometry pose and velocity
        self.odom_pose = None
        self.odom_velocity = None
        self.pose2d=None
                
        # Create a publisher for the topic 'cmd_vel'
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create a subscriber to the topic 'odom', which will call 
        # self.odom_callback every time a message is received
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        
        self.center_sensor_subscriber = self.create_subscription(Range, 'proximity/center', self.center_sensor_callback, 10)
        self.center_right_sensor_subscriber = self.create_subscription(Range, 'proximity/center_right', self.center_right_sensor_callback, 10)
        self.center_left_sensor_subscriber = self.create_subscription(Range, 'proximity/center_left', self.center_left_sensor_callback, 10)
        self.center_sensor_subscriber = self.create_subscription(Range, 'proximity/center', self.center_sensor_callback, 10)
        self.rear_right_sensor_subscriber = self.create_subscription(Range, 'proximity/rear_right', self.rear_right_sensor_callback, 10)
        self.rear_left_sensor_subscriber = self.create_subscription(Range, 'proximity/rear_left', self.rear_left_sensor_callback, 10)
        self.right_sensor_subscriber = self.create_subscription(Range, 'proximity/right', self.right_sensor_callback, 10)
        self.left_sensor_subscriber = self.create_subscription(Range, 'proximity/left', self.left_sensor_callback, 10)

        self.right_ground_sensor_subscriber = self.create_subscription(Range, 'ground/right', self.right_ground_sensor_callback, 10)
        self.left_ground_sensor_subscriber = self.create_subscription(Range, 'ground/left', self.left_ground_sensor_callback, 10)
        
        self.center_SensorRange=-1
        self.center_left_SensorRange=-1
        self.center_right_SensorRange=-1
        self.rear_left_SensorRange=-1
        self.rear_right_SensorRange=-1
        self.left_SensorRange=-1
        self.right_SensorRange=-1

        self.left_ground_SensorRange=-1
        self.right_ground_SensorRange=-1
        self.state = ControllerNode.FORWARD
        self.wallposition=None
        self.direction = 1
        self.adjust_counter=0

        # NOTE: we're using relative names to specify the topics (i.e., without a 
        # leading /). ROS resolves relative names by concatenating them with the 
        # namespace in which this node has been started, thus allowing us to 
        # specify which Thymio should be controlled.

        
    def start(self):
        # Create and immediately start a timer that will regularly publish commands
        self.timer = self.create_timer(1/120, self.update_callback)
        #self.timer = self.create_timer(1, self.update_callback)
    
    def stop(self):
        # Set all velocities to zero
        cmd_vel = Twist()
        self.vel_publisher.publish(cmd_vel)
    
    def odom_callback(self, msg):
        self.odom_pose = msg.pose.pose
        self.odom_velocity = msg.twist.twist
        #self.get_logger().info("ODOM: vel_x:"+str(round(self.odom_velocity.linear.x,2))+", theta:"+str(round(self.odom_velocity.angular.z,2)),  throttle_duration_sec=0.5)
        
        self.pose2d = self.pose3d_to_2d(self.odom_pose)
        #self.gui.append(self.pose2d.x,self.pose2d.y)
        #self.gui.update()
        #self.get_logger().info(str(self.gui.x_data)+""+str(self.gui.y_data),throttle_duration_sec=0.5)
        #self.get_logger().info(str(self.pose2d.x)+""+str(self.pose2d.y),throttle_duration_sec=0.5)
        
        """
        self.get_logger().info(
            "odometry: received pose (x"+str(self.pose2d.x)+", y:"+str(self.pose2d.y)+", theta:"+str(self.pose2d.theta),
             throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
        )
        """

    def center_sensor_callback(self,msg):
        self.center_SensorRange=msg.range
        #self.get_logger().info("center: "+str(msg.range),throttle_duration_sec=0.5) 

    def center_right_sensor_callback(self,msg):
        self.center_right_SensorRange=msg.range
        #self.get_logger().info("centerright: "+str(msg.range),throttle_duration_sec=0.5) 

    def center_left_sensor_callback(self,msg):
        self.center_left_SensorRange=msg.range
        #self.get_logger().info("centerleft: "+str(msg.range),throttle_duration_sec=0.5) 

    def rear_left_sensor_callback(self,msg):
        self.rear_left_SensorRange=msg.range
        #self.get_logger().info("centerleft: "+str(msg.range),throttle_duration_sec=0.5) 
    
    def rear_right_sensor_callback(self,msg):
        self.rear_right_SensorRange=msg.range
        #self.get_logger().info("centerleft: "+str(msg.range),throttle_duration_sec=0.5) 

    def right_sensor_callback(self,msg):
        self.right_SensorRange=msg.range
        #self.get_logger().info("centerleft: "+str(msg.range),throttle_duration_sec=0.5) 
    
    def left_sensor_callback(self,msg):
        self.left_SensorRange=msg.range
        #self.get_logger().info("centerleft: "+str(msg.range),throttle_duration_sec=0.5) 

    def right_ground_sensor_callback(self,msg):
        self.right_ground_SensorRange=msg.range
        #self.get_logger().info("centerleft: "+str(msg.range),throttle_duration_sec=0.5) 
    
    def left_ground_sensor_callback(self,msg):
        self.left_ground_SensorRange=msg.range
        #self.get_logger().info("centerleft: "+str(msg.range),throttle_duration_sec=0.5) 


    def pose3d_to_2d(self, pose3):
        quaternion = (
            pose3.orientation.x,
            pose3.orientation.y,
            pose3.orientation.z,
            pose3.orientation.w
        )
        
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(quaternion)
        """
        pose2 = (
            pose3.position.x,  # x position
            pose3.position.y,  # y position
            yaw                # theta orientation
        )
        """
        pose2 = Pose()
        pose2.x=pose3.position.x
        pose2.y=pose3.position.y
        pose2.theta=yaw

        return pose2
        
    def update_callback(self):
      
        if(self.pose2d is None):
            return

        
        self.get_logger().info("Gleft"+str(self.left_ground_SensorRange), throttle_duration_sec=0.5)
        self.get_logger().info("Gright"+str(self.right_ground_SensorRange), throttle_duration_sec=0.5)
        
        cmd_vel = Twist() 
        cmd_vel.linear.x  = 0.1 # [m/s]
        cmd_vel.angular.z = 0.0 # [rad/s]                
        # Publish the command
        self.vel_publisher.publish(cmd_vel)

        if(self.left_ground_SensorRange == 0 ): #turn right
            cmd_vel = Twist() 
            cmd_vel.linear.x  = 0.0 # [m/s]
            cmd_vel.angular.z = -0.1 # [rad/s]                
            # Publish the command
            self.vel_publisher.publish(cmd_vel)

        
        if(self.right_ground_SensorRange == 0 ): #turn left
           
            cmd_vel = Twist() 
            cmd_vel.linear.x  = 0.0 # [m/s]
            cmd_vel.angular.z = 0.1 # [rad/s]                
            # Publish the command
            self.vel_publisher.publish(cmd_vel)
                
     
               
    def euclidean_distance(self, goal_pose, current_pose):
        return sqrt(pow((goal_pose.x - current_pose.x), 2) +
                    pow((goal_pose.y - current_pose.y), 2))

class Pose:
    def __init__(self):
        self.x=0
        self.y=0
        self.theta=0



def main():
    # Initialize the ROS client library
    rclpy.init(args=sys.argv)

    #gui = GUI()
    #print("DIOCAN") 
    gui=0
    # Create an instance of your node class
    node = ControllerNode(gui)
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
