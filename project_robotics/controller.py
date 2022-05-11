# imports
from queue import Empty
import rclpy
from rclpy.node import Node
import tf_transformations
from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
import numpy as np
from sensor_msgs.msg import Range, Image, Imu, JointState
import matplotlib.pyplot as plt
import sys
from robomaster_msgs.msg import WheelSpeeds, CameraConfig
import matplotlib
import numpy as np
from PIL import Image as image_pil
import os
from launch.substitutions import LaunchConfiguration
matplotlib.use('Agg')

position = sys.argv
print(f"position passed: {position}")

path_central = os.path.join(
    os.getcwd(), './src/project_robotics/project_robotics/images_central')
path_left = os.path.join(
    os.getcwd(), './src/project_robotics/project_robotics/images_left')
path_right = os.path.join(
    os.getcwd(), './src/project_robotics/project_robotics/images_right')
if(os.path.isdir(path_central) == False):
    os.mkdir(path_central)
if(os.path.isdir(path_left) == False):
    os.mkdir(path_left)
if(os.path.isdir(path_right) == False):
    os.mkdir(path_right)


class ControllerNode(Node):
    '''This is the class for the controller:
    - one publisher for velocity
    - one subscriber for odometry
    - many subscribers for sensors'''

    def __init__(self):

        super().__init__('controller_node')

        # Create attributes to store odometry pose and velocity
        self.odom_pose = None
        self.odom_velocity = None

        # Create a publisher for the topic 'cmd_vel'
        self.vel_publisher = self.create_publisher(Twist, 'RoboMaster/cmd_vel', 10)
        self.camera_subscriber = self.create_subscription(
            Image, 'RoboMaster/camera/image_raw', self.camera_callback, 10)
        self.linear_vel_x = 0.25
        self.time_step_vel = 0
        self.start_counter = 0
        self.time_step_camera = 0
       


    def camera_callback(self, msg):
        image = msg.data
        height = msg.height
        width = msg.width
        time_step_nano = msg.header.stamp.nanosec
        time_step_sec = msg.header.stamp.sec
        
        image = np.uint8(image)
        image = np.reshape(image, (height, width, 3))
        # plt.imshow(image)
        img = image_pil.fromarray(image, 'RGB')
        if(self.start_counter==0):
            self.start_counter=time_step_sec
        if((time_step_sec - self.start_counter)>999 ):
            self.get_logger().info(f"Saving Images: stopped")
            return
        

        img.save(os.path.join(os.getcwd(), './src/project_robotics/project_robotics/images_central/camera_' +
                    str(time_step_sec)+'_'+str(time_step_nano)+'.png'))
        self.get_logger().info(f"time step: {time_step_sec} {time_step_nano}")
        self.time_step_camera = time_step_sec
        '''if((time_step_sec - self.start_counter)%300==0):
            img.show()
        # img.show()
        '''


    def start(self):
        
        self.timer = self.create_timer(1/10., self.update_callback)
        '''self.timer_arm = self.create_timer(1/10., self.update_arm_callback)
        self.time_imu = self.create_timer(1/10., self.update_imu)
        self.time_joint = self.create_timer(1/10., self.update_joint)'''

    def stop(self):
        # Set all velocities to zero
        cmd_vel = Twist()
        self.vel_publisher.publish(cmd_vel)

    def odom_callback(self, msg):
        self.odom_pose = msg.pose.pose
        self.odom_valocity = msg.twist.twist

        pose2d = self.pose3d_to_2d(self.odom_pose)

    def pose3d_to_2d(self, pose3):
        quaternion = (
            pose3.orientation.x,
            pose3.orientation.y,
            pose3.orientation.z,
            pose3.orientation.w
        )

        roll, pitch, yaw = tf_transformations.euler_from_quaternion(quaternion)

        pose2 = (
            pose3.position.x,  # x position
            pose3.position.y,  # y position
            yaw                # theta orientation
        )

        return pose2

    def update_callback(self):

        cmd_vel = Twist()
        self.time_step_vel += 1
        
        cmd_vel.linear.x = self.linear_vel_x
        if(self.time_step_vel % 55 == 0):
            self.linear_vel_x = - self.linear_vel_x
            
            self.vel_publisher.publish(cmd_vel)
        
        elif(self.time_step_vel>999):
            return
        
        
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




'''
Unused methods
     self.arm_publisher = self.create_publisher(PointStamped, 'arm_position', 10)
        self.imu_publisher = self.create_publisher(Imu, 'imu', 10)
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states_p', 10)
        
        self.cameraConfig_subscriber = self.create_subscription(
        Image, 'camera/config', self.cameraConfig_callback, 10)
        
    
    def update_imu(self):
        imu_msg = Imu()
        #imu_msg.orientation.w = 1.
        imu_msg.angular_velocity.z=self.time_step_camera/1000000000.
        #self.imu_publisher.publish(imu_msg)

    def update_joint(self):
        joint_msg =JointState()
        joint_msg.velocity = {10., 10.,10. }
        self.joint_state_publisher.publish(joint_msg)

    def update_arm_callback(self):
        arm_msg = PointStamped()
        arm_msg.point.z = 0.2*self.time_step_camera
        self.arm_publisher.publish(arm_msg)

'''