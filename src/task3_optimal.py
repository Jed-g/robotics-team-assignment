#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from math import inf, pi, sqrt, sin, cos, atan2
import numpy as np
import time

FREQUENCY = 2
LINEAR_VELOCITY = 0.25
ANGULAR_VELOCITY = 0.5
TURN_CORRECTION_SPEED = 0.3

class Main():
    def __init__(self):
        self.node_name = "explore"

        rospy.init_node(self.node_name)
        self.rate = rospy.Rate(FREQUENCY)  # hz

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        rospy.loginfo(f"The '{self.node_name}' node is active...")

        self.publish_velocity = Publish_velocity()
        self.odom_data = Odom_data()
        self.lidar_data = Lidar_data()

    def shutdownhook(self):
        print(f"Stopping the '{self.node_name}' node at: {rospy.get_time()}")
        self.ctrl_c = True
        self.publish_velocity.shutdown()

    def main_loop(self):
   
        while not self.ctrl_c:

            if self.odom_data.initial_data_loaded and self.lidar_data.initial_data_loaded:

                robot_direction = 0
               
                front = min(self.lidar_data.front_arc)
                left = np.mean(self.lidar_data.ranges[0:90])
                right = np.mean(self.lidar_data.ranges[-270:])
           

                avoid = 0.6

                if front > 1 and left > 1 and right > 1:
                    pass
                elif robot_direction == 0:
                    if right < avoid:
                        robot_direction = 1
                        #print(self.lidar_data.front_arc)
                        #(sum(self.lidar_data.front_arc)/len(self.lidar_data.front_arc)

                        
                        
                        while not (min(self.lidar_data.front_arc) < avoid):
                            self.publish_velocity.publish_velocity(LINEAR_VELOCITY, 0)
                            print("first")
                    if left < avoid:
                        robot_direction = -1
                        while not (min(self.lidar_data.front_arc) < avoid):
                            self.publish_velocity.publish_velocity(LINEAR_VELOCITY, 0)
                            print("left<avoid")
                elif left < 1 and right < 1 and front > 1:
                        while not (min(self.lidar_data.front_arc) < avoid):
                            self.publish_velocity.publish_velocity(LINEAR_VELOCITY, 0)
                            print("left and right")
                
                self.publish_velocity.publish_velocity()

                if left > 1 and right > 1 and front < 0.7:
                    print("1")
                    if self.lidar_data.left_avg > self.lidar_data.right_avg:
                        self.publish_velocity.publish_velocity(0, ANGULAR_VELOCITY)
                    elif self.lidar_data.left_avg < self.lidar_data.right_avg:
                        self.publish_velocity.publish_velocity(0, -ANGULAR_VELOCITY)
                else:
                   
                    if left > right:
                        self.publish_velocity.publish_velocity(0, ANGULAR_VELOCITY)
                        print("2.1")
                    elif front < 1 and left > 1 and right < 1:
                        self.publish_velocity.publish_velocity(0, ANGULAR_VELOCITY)
                        print("2.2")
                    elif right > left:
                        self.publish_velocity.publish_velocity(0, -ANGULAR_VELOCITY)
                        print("2.3")
                    elif front < 1 and left < 1 and right > 1:
                        self.publish_velocity.publish_velocity(0, -ANGULAR_VELOCITY)
                        print("2.4")

            
                #self.publish_velocity.publish_velocity()
               
                self.rate.sleep()


class Publish_velocity():

    def __init__(self):
        topic_name = "cmd_vel"
        self.pub = rospy.Publisher(topic_name, Twist, queue_size=10)

    def publish_velocity(self, linear=0, angular=0):
        vel_cmd = Twist()
        vel_cmd.linear.x = linear
        vel_cmd.angular.z = angular
        self.pub.publish(vel_cmd)

    def shutdown(self):
        self.publish_velocity()


class Odom_data():

    def __init__(self):
        topic_name = "odom"
        self.sub = rospy.Subscriber(topic_name, Odometry, self.callback)
        self.initial_data_loaded = False

    def callback(self, topic_message):
        orientation = topic_message.pose.pose.orientation

        (_, _, yaw) = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w], 'sxyz')

        position = topic_message.pose.pose.position
        self.posx = position.x
        self.posy = position.y
        self.angle = yaw
        self.angle_360 = (self.angle if self.angle >= 0 else self.angle + 2*pi) * 180 / pi

        self.initial_data_loaded = True

class Lidar_data():
    
    def __init__(self):
        topic_name = "scan"
        self.subscriber = rospy.Subscriber(topic_name, LaserScan, self.scan_callback)
        self.ranges = []
        self.initial_data_loaded = False
        self.left_avg = 0
        self.right_avg = 0
        self.front_arc = []


    def scan_callback(self, scan_data):
        self.ranges = scan_data.ranges
        self.initial_data_loaded = True

        front_arc_partial = list(self.ranges[20::-1])
        front_arc_partial.extend(list(self.ranges[:-20:-1]))
        self.front_arc = front_arc_partial
       # print(front_arc_partial)

        min_index = 0
        _min = inf

        for i in range(len(self.front_arc)):
            if self.front_arc[i] < _min:
                _min = self.front_arc[i]
                min_index = i
        
        min_index -= 20
        min_index *= -1

        if min_index >= 0:
            self.left_avg = sum(self.ranges[min_index:min_index+90])/90
            self.right_avg = (sum(self.ranges[:min_index]) + sum(self.ranges[-90+min_index:]))/90
        else:
            min_index *= -1
            self.left_avg = (sum(self.ranges[-min_index:])+sum(self.ranges[:90-min_index]))/90
            self.right_avg = sum(self.ranges[-min_index-90:-min_index])/90



        

if __name__ == '__main__':
    try:
        main_instance = Main()
        main_instance.main_loop()
    except rospy.ROSInterruptException:
        pass
