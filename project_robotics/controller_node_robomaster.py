from cv2 import KeyPoint
import rclpy
from rclpy.node import Node
import tf_transformations

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range,Image

import sys

from math import pow, atan2, sqrt, sin, cos
import time
import random 

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


from PIL import Image as image_pil

import torch

import torch.nn as nn
import torch.nn.functional as F

import torchvision
from torchvision import transforms

import os

class ControllerNode(Node):

    FORWARD=0
    ROTATE=1
    ADJUST=2
    AUAI=3

    NO_WALL = 0.05

    def __init__(self):
        super().__init__('controller_node')

        model_path = os.path.join(os.getcwd(), './src/thymio_example/thymio_example/model/test.pth')
        state_dict = torch.load(model_path, map_location=torch.device('cpu'))

        self.model=ConvNet()
        self.model.load_state_dict(state_dict)

        
        self.no_wall = ControllerNode.NO_WALL
        
        # Create attributes to store odometry pose and velocity
        self.odom_pose = None
        self.odom_velocity = None
        self.pose2d=None
                
        # Create a publisher for the topic 'cmd_vel'
        self.vel_publisher = self.create_publisher(Twist, 'RoboMaster/cmd_vel', 10)

        # Create a subscriber to the topic 'odom', which will call 
        # self.odom_callback every time a message is received

        self.camera_subscriber = self.create_subscription(
            Image, 'RoboMaster/camera/image_raw', self.camera_callback, 10)

        self.time_step_camera = 0

        self.image = None
        
        

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

    def camera_callback(self, msg):
        image = msg.data
        height = msg.height
        width = msg.width
        time_step_nano = msg.header.stamp.nanosec
        time_step_sec = msg.header.stamp.sec
        #self.get_logger().info(f"time step: {time_step_sec} {time_step_nano}")
        image = np.uint8(image)
        image = np.reshape(image, (height, width, 3))

        #self.get_logger().info("image shape"+str(image.shape), throttle_duration_sec=0.5)
      
        #image = self.model.normalize(image)
        #plt.imshow(image)
        #img = image_pil.fromarray(image, 'RGB')
        
        #lr = 'right'
        #if(time_step_sec != self.time_step_camera):
        #    img.save(os.path.join(os.getcwd(),  './src/thymio_example/thymio_example/images/' +lr+
        #             str(time_step_sec)+'_'+str(time_step_nano)+'.png'))
        #self.time_step_camera = time_step_sec
        
        mean_r =  0.56988657
        mean_g =  0.55153894
        mean_b =  0.5138332
        std_r =  0.12706465
        std_g =  0.1545275
        std_b =  0.2091071
      

        #image[:,:,0] = (image[:,:,0] - mean_r) / std_r
        #image[:,:,1] = (image[:,:,1] - mean_g) / std_g
        #image[:,:,2] = (image[:,:,2] - mean_b) / std_b
        
        transform_norm = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize(( mean_r,mean_g,mean_b), (std_r,std_g, std_b))
        ])
        
        # get normalized image
        image_normalized = transform_norm(image)
       
        self.image=image_normalized

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
      
        if(self.image is None):
            return

        #self.get_logger().info("Gleft"+str(self.left_ground_SensorRange), throttle_duration_sec=0.5)
        #self.get_logger().info("Gright"+str(self.right_ground_SensorRange), throttle_duration_sec=0.5)
        
        cmd_vel = Twist() 
        cmd_vel.linear.x  = 0.025 # [m/s]
        cmd_vel.angular.z = 0.0 # [rad/s]                
         #Publish the command
        self.vel_publisher.publish(cmd_vel)
        
        
        with torch.no_grad():
            self.model.eval()
            outputs = self.model(self.image) # shape (B, num_classes)
            _, predicted = outputs.max(dim=1)

            self.get_logger().info("shape: "+str(self.image.shape), throttle_duration_sec=0.5)
            self.get_logger().info("outs"+str(outputs), throttle_duration_sec=0.5)
            self.get_logger().info("Predicted"+str(predicted[0]), throttle_duration_sec=0.5)
        
        
        if(predicted[0] == 1 ): #turn right
            self.get_logger().info("RIGHT", throttle_duration_sec=0.5)
            cmd_vel = Twist() 
            cmd_vel.linear.x  = 0.0 # [m/s]
            cmd_vel.angular.z = -0.5 # [rad/s]                
            # Publish the command
            self.vel_publisher.publish(cmd_vel)

        if(predicted[0] == 0 ): #turn left
            self.get_logger().info("LEFT", throttle_duration_sec=0.5)
           
            cmd_vel = Twist() 
            cmd_vel.linear.x  = 0.0 # [m/s]
            cmd_vel.angular.z = 0.5 # [rad/s]                
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


class ConvNet(nn.Module):
  
    def __init__(self): # just example. super(ConvNet, self).__init__()
        super(ConvNet, self).__init__()
        
        #print()
        #print("in")
        #input = torch.randn(1,3,32,32) #(batch,channels,H,W)
        #print(input.size())

        #print("conv1")
        self.conv1 = nn.Conv2d(3, 32, 3) # 3 channels, 32 filters, 3x3 filter ,input shape (3, 32, 32). 
        #input = self.conv1(input)
        #print(input.size())
        
        #print("conv2")
        self.conv2 = nn.Conv2d(32, 32, 3)
        #input = self.conv2(input)
        #print(input.size())

        #print("pool1")
        self.pool1 = nn.MaxPool2d(2, 2)
        #input = self.pool1(input)
        #print(input.size())

        self.dropout1 = nn.Dropout(0.10)

        #print("conv3")
        self.conv3 = nn.Conv2d(32, 64, 3)
        #input = self.conv3(input)
        #print(input.size())

        #print("conv4")
        self.conv4 = nn.Conv2d(64, 64, 3)
        #input = self.conv4(input)
        #print(input.size())

        #print("pool2")
        self.pool2 = nn.MaxPool2d(2, 2)
        #input = self.pool2(input)
        #print(input.size())

        self.dropout2 = nn.Dropout(0.20)

        #input = input.view(-1,64 * 5 * 5)
        #print("flatten")
        #print(input.size())

        #print("fc1")
        self.fc1 = nn.Linear(64 * 87 * 157, 512)
        #input = self.fc1(input)
        #print(input.size())
        self.dropout3 = nn.Dropout(0.50)
        
        #print("fc2")
        self.fc2 = nn.Linear(512, 2) # 10 output classes. #  new 3 output classes.
        #input = self.fc2(input)
        #print(input.size())

    def normalize(self, image):
        #constants from training dataset
        mean_r =0.27381533 
        mean_g = 0.28483894 
        mean_b= 0.29391152
        std_r = 0.35407916 
        std_g = 0.3666874 
        std_b = 0.3760119

        #image[:,:,0] = (image[:,:,0] - mean_r) / std_r
        #image[:,:,1] = (image[:,:,1] - mean_g) / std_g
        #image[:,:,2] = (image[:,:,2] - mean_b) / std_b

        transform_norm = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize(( mean_r,mean_g,mean_b), (std_r,std_g, std_b))
        ])
        
        # get normalized image
        img_normalized = transform_norm(image)
        
        # convert normalized image to numpy
        # array
        #img_np = np.array(img_normalized)


        #grid_img = torchvision.utils.make_grid(, nrow=10)
        #print(img_normalized.shape)
        #plt.figure()
        #plt.imshow(img_normalized.permute(1,2,0).numpy())
        #plt.show()

        return img_normalized
     
    def forward(self, x):
        x = F.relu(self.conv1(x)) 
        x = F.relu(self.conv2(x)) 
        x = self.pool1(x) # conv, pool.
        x=self.dropout1(x)
        x = F.relu(self.conv3(x)) 
        x = F.relu(self.conv4(x)) 
        x = self.pool2(x) # conv, pool.
        x=self.dropout2(x)

        #print(x.shape)
        x = x.view(-1,64 * 87 * 157)
        #x = x.view(-1, 64 * 5 * 5) # linearize input "images". 
        #x = x.view(-1, )
        x = F.relu(self.fc1(x)) # fully connected.
        x=self.dropout3(x)
        x= self.fc2(x)
        # second layer does not need activation function;
        # softmax is computed by cross entropy loss.
        #x = F.softmax(self.fc2(x),dim=1) # fully connected. 
        #x= nn.Softmax(self.fc2(x))
        return x
    
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
