#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion
from math import inf, pi, sqrt, sin, cos, atan2
import numpy as np
import time
import argparse
import roslaunch
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from pathlib import Path

FREQUENCY = 10
LINEAR_VELOCITY = 0.35
ANGULAR_VELOCITY = 0.8
TURN_CORRECTION_SPEED = 0.3
COLOR_THRESHOLD_VALUE = 250
ANGLE_DETECTION_THRESHOLD = 40
DISTANCE_FROM_START_THRESHOLD = 2
RANGE_THRESHOLD = 1.5
DISTANCE_FACTOR = 0.2
REVERSE_TIME = 1.6
HOMING_PRECISION = 0.1
CLEARANCE_THRESHOLD = 0.32
FORWARD_STOPPING_THRESHOLD = 0.34
HOMING_THRESHOLD = 0.45

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
        self.command_line = CLI_colour()

        self.color = self.command_line.color
        self.color_index = None

        if self.color == "red":
            self.color_index = 1
        elif self.color == "green":
            self.color_index = 2
        elif self.color == "yellow":
            self.color_index = 3
        else:
            self.color = "blue"
            self.color_index = 0

        self.target_found = False

        self.starting_x = None
        self.starting_y = None

        self.visited_points = []

        # Thresholds for ["Blue", "Red", "Green", "Yellow"]
        self.lower = [(115, 224, 100), (0, 185, 100), (25, 150, 100), (155, 35, 225)]
        self.upper = [(130, 255, 255), (10, 255, 255), (70, 255, 255), (255, 50,255)]

    def shutdownhook(self):
        print(f"Stopping the '{self.node_name}' node at: {rospy.get_time()}")
        self.ctrl_c = True
        self.publish_velocity.shutdown()

    def main_loop(self):
        while not self.ctrl_c:
            if self.odom_data.initial_data_loaded and self.lidar_data.initial_data_loaded:
                pass

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

class Lidar_data():
    
    def __init__(self):
        topic_name = "scan"
        self.subscriber = rospy.Subscriber(topic_name, LaserScan, self.scan_callback)
        self.ranges = []
        self.initial_data_loaded = False

    def scan_callback(self, scan_data):
        self.ranges = scan_data.ranges
        self.initial_data_loaded = True

class CLI_colour():
    def __init__(self):
        cli = argparse.ArgumentParser(description=f"...")
        cli.add_argument("-colour", metavar="COL", type=String,
            default="blue", 
            help="The name of a colour (for example)")
        
        self.args = cli.parse_args(rospy.myargv()[1:])
        self.color = self.args.colour.data

if __name__ == '__main__':
    try:
        main_instance = Main()
        main_instance.main_loop()
    except rospy.ROSInterruptException:
        pass
