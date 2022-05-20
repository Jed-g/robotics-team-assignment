#!/usr/bin/env python3

import time
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import sqrt, pow, pi


class Task3:
    FRONT_RAD = 0.8
    LEFT_RAD = 0.8
    ANGULAR_VEL = 0.5
    LINEAR_VEL = 0.3
    def __init__(self):

        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.init_node('task3', anonymous=True)
        self.rate = rospy.Rate(10) # hz
        self.vel_cmd = Twist()
 
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)
        rospy.loginfo("the 'task3' node is active...")
        self.lidar_data = Lidar_data()
        self.odom_data = Odom_data()
        
        self.prev_time = time.time()
         

    # def odom_callback(self, odom_data):
    #     orientation_x = odom_data.pose.pose.orientation.x
    #     orientation_y = odom_data.pose.pose.orientation.y
    #     orientation_z = odom_data.pose.pose.orientation.z
    #     orientation_w = odom_data.pose.pose.orientation.w

    #     self.pos_x = odom_data.pose.pose.position.x
    #     self.pos_y = odom_data.pose.pose.position.y

    #     (roll, pitch, self.yaw) = euler_from_quaternion([orientation_x, orientation_y, orientation_z, orientation_w],'sxyz')


    def shutdownhook(self):
        # publish an empty twist message to stop the robot
        # (by default all velocities will be zero):        
        self.pub.publish(Twist())
        self.ctrl_c = True

    def print_stuff(self, a_message):
        # a function to print information to the terminal (use as you wish):
        # print the message that has been passed in to the method via the "a_message" input:
        print(a_message)
        print(f"current velocity: lin.x = {self.vel_cmd.linear.x:.1f}, ang.z = {self.vel_cmd.angular.z:.1f}")
        # you could also print the current odometry to the terminal here, if you wanted to:
        print(f"current odometry: x = {self.odom_data.x:.3f}, y = {self.odom_data.y:.3f}, theta_z = {self.odom_data.theta_z:.3f}")


    def main_loop(self):
        status = ""
        while not self.ctrl_c:
            if self.lidar_data.initial_data_loaded and self.odom_data.initial_data_loaded:
                #if there is left wall proceed to check of there is wall in the front
                if self.lidar_data.left_p <= 0.4 :
                    status = "Left wall detected"
                    #check if there is left angled wall
                    # if self.lidar_data.left_p < self.lidar_data.left_top_p:
                    #     #stop
                    #     self.vel_cmd.linear.x = 0.0
                    #     self.vel_cmd.angular.z = 0.0
                    #     #rotate
                    #     status = "Left angled wall detected"
    
                    #check if there is rectangular front wall
                    # if self.lidar_data.front_p < 0.8 and self.lidar_data.front2_p == self.lidar_data.front3_p:
                    #     #stop
                    #     self.vel_cmd.linear.x = 0.0
                    #     self.vel_cmd.angular.z = 0.0
                    #     #rotate
                    #     status = "Rectangular fronmt wall detected, turning"

                    #check if its acute wall
                    # elif self.lidar_data.front_p < 0.8 and self.lidar_data.front2_p < self.lidar_data.front3_p :
                    #     #stop
                    #     self.vel_cmd.linear.x = 0.0
                    #     self.vel_cmd.angular.z = 0.0
                    #     status = "Acute front wall detected, turning"
                
                    # #check if its obtuse wall
                    # elif self.lidar_data.front_p < 0.8 and self.lidar_data.front2_p > self.lidar_data.front3_p :
                    #     #stop
                    #     self.vel_cmd.linear.x = 0.0
                    #     self.vel_cmd.angular.z = 0.0
                    #     status = "Obtuse front wall detected, turning"

                    # #if nothing in front move forward 
                    # else:
                    if self.lidar_data.front_p <= 0.5:
                        #stop
                        self.vel_cmd.linear.x = 0.0

                        if abs(self.odom_data.theta_z0 - self.odom_data.theta_z) >= pi/2:
                            # If the robot has turned 90 degrees (in radians) then stop turning
                            self.vel = Twist()
                            self.odom_data.theta_z0 = self.odom_data.theta_z
                            status = "Finished turning"

                        else:
                            self.vel_cmd = Twist()
                            self.vel_cmd.angular.z = -1.6
                            status = "Front wall, turning"
                        
                    else:
                        self.vel_cmd.linear.x = 0.22
                        print("got to this point")
                        self.vel_cmd.angular.z = 0.0
                        status = "No front wall detected, moving forward"
                        
                else:
                     status = "No left wall detected"
                     if self.lidar_data.ranges[35] <=5 and self.lidar_data.ranges[55] <=5:
                        if abs(self.odom_data.theta_z0 - self.odom_data.theta_z) >= pi/2:
                            # If the robot has turned 90 degrees (in radians) then stop turning
                            self.vel = Twist()
                            self.odom_data.theta_z0 = self.odom_data.theta_z
                            print("made it here")
                            self.vel_cmd.linear.x = 0.0
                            self.vel_cmd.linear.x = 0.0

                            status = "Finished turning"

                        else:
                            self.vel_cmd.linear.x = 0.3
                            self.vel_cmd.angular.z = 1
                            print("made it here  222")
                        
                            

                    # if sqrt(pow(self.odom_data.x0 - self.odom_data.x, 2) + pow(self.odom_data.y0 - self.odom_data.y, 2)) >= 0.01:
 
                    #     if abs(self.odom_data.theta_z0 - self.odom_data.theta_z) >= pi/2:
                    #         # If the robot has turned 90 degrees (in radians) then stop turning
                    #         self.vel = Twist()
                    #         self.odom_data.theta_z0 = self.odom_data.theta_z
                    #         self.odom_data.x0 = self.odom_data.x
                    #         self.odom_data.y0 = self.odom_data.y
                    #         status = "Finished turning"

                    #     else:
                    #         self.vel_cmd = Twist()
                    #         self.vel_cmd.angular.z = 1.6
                    #         status = "Front wall, turning"   
                    # else:
                    #     self.vel_cmd.linear.x = 0.1
                    #     status = "Moving forwards"

                    
                    #Rotate to the left 90 degrees
                    #Move forward until it detects a left wall at the minimum distance
                    

            # publish whatever velocity command has been set in your code above:
            self.pub.publish(self.vel_cmd)
            # call a function which prints some information to the terminal:
            self.print_stuff(status)
            # maintain the loop rate @ 10 hz
            self.rate.sleep()

class Lidar_data():

    def __init__(self):
        topic_name = "scan"
        self.lasersub = rospy.Subscriber(topic_name, LaserScan, self.laserscan_callback)
        self.ranges = []
        self.initial_data_loaded = False

    def laserscan_callback(self, scan_data):
        self.ranges = scan_data.ranges
        #left point 
        self.left_p = scan_data.ranges[90]
        #left top point  
        self.left_top_p = scan_data.ranges[45]
        #front point
        self.front_p = scan_data.ranges[0] 
        #2-front point
        self.front2_p = scan_data.ranges[10]
        #3-front point
        self.front3_p = scan_data.ranges[350]

        self.initial_data_loaded = True

class Odom_data():

    def __init__(self):
        topic_name = "odom"
        self.odomsub = rospy.Subscriber(topic_name, Odometry, self.odom_callback)

        # define the robot pose variables and set them all to zero to start with:
        # variables to use for the "current position":
        self.x = 0.0
        self.y = 0.0
        self.theta_z = 0.0
        # variables to use for the "reference position":
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta_z0 = 0.0

        self.initial_data_loaded = False

    def odom_callback(self, odom_data):
       # obtain the orientation and position co-ords:
        or_x = odom_data.pose.pose.orientation.x
        or_y = odom_data.pose.pose.orientation.y
        or_z = odom_data.pose.pose.orientation.z
        or_w = odom_data.pose.pose.orientation.w

        # obtain the position co-ords:
        pos_x = odom_data.pose.pose.position.x
        pos_y = odom_data.pose.pose.position.y

        # convert orientation co-ords to roll, pitch & yaw (theta_x, theta_y, theta_z):
        (roll, pitch, yaw) = euler_from_quaternion([or_x, or_y, or_z, or_w], 'sxyz')
        
        # We are only interested in the x, y and theta_z odometry data for this
        # robot, so we only assign these to class variables (so that we can 
        # access them elsewhere within our Square() class):
        self.x = pos_x
        self.y = pos_y
        self.theta_z = yaw 

        # If this is the first time that this callback_function has run, then 
        # obtain a "reference position" (used to determine how far the robot has moved
        # during its current operation)
        
        # set the reference position:
        self.x0 = self.x
        self.y0 = self.y
        self.theta_z0 = self.theta_z

        self.initial_data_loaded = True

if __name__ == '__main__':
    vel_ctlr = Task3()
    try:
        vel_ctlr.main_loop()
    except rospy.ROSInterruptException:
        pass