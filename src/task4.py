#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from math import inf, pi, sqrt, sin, cos, atan2
import numpy as np
import time

from pathlib import Path

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image




FREQUENCY = 10
LINEAR_VELOCITY = 0.25
ANGULAR_VELOCITY = 1.2
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
        self.camera = Take_photo()

        self.acquired_color = False
        self.color = None

    def shutdownhook(self):
        print(f"Stopping the '{self.node_name}' node at: {rospy.get_time()}")
        self.ctrl_c = True
        self.publish_velocity.shutdown()

    def turn_to_angle_360_system(self, angle, lin_speed = None, ang_speed = None):
        self.publish_velocity.publish_velocity()

        difference_clockwise = self.odom_data.angle_360 - angle + 360 if angle > self.odom_data.angle_360 else self.odom_data.angle_360 - angle
        difference_anti_clockwise = 360 - self.odom_data.angle_360 + angle if angle < self.odom_data.angle_360 else angle - self.odom_data.angle_360

        requires_crossing_0_360 = False

        turn_clockwise = True

        if difference_clockwise <= difference_anti_clockwise:
            if angle > self.odom_data.angle_360:
                requires_crossing_0_360 = True
        else:
            turn_clockwise = False
            if angle < self.odom_data.angle_360:
                requires_crossing_0_360 = True

        if turn_clockwise:
            initial_angle = self.odom_data.angle_360

            if lin_speed == None:
                self.publish_velocity.publish_velocity(0, -ANGULAR_VELOCITY if ang_speed == None else -ang_speed)
            else:
                self.publish_velocity.publish_velocity(lin_speed, -ANGULAR_VELOCITY if ang_speed == None else -ang_speed)

            time.sleep(0.6)

            if requires_crossing_0_360:
                while self.odom_data.angle_360 <= initial_angle:
                    if self.ctrl_c:
                        self.publish_velocity.publish_velocity()
                        return
                    
                    if lin_speed == None:
                        self.publish_velocity.publish_velocity(0, -ANGULAR_VELOCITY if ang_speed == None else -ang_speed)
                    else:
                        self.publish_velocity.publish_velocity(lin_speed, -ANGULAR_VELOCITY if ang_speed == None else -ang_speed)

            
            while self.odom_data.angle_360 > angle:
                if self.ctrl_c:
                        self.publish_velocity.publish_velocity()
                        return
                
                if lin_speed == None:
                    self.publish_velocity.publish_velocity(0, -ANGULAR_VELOCITY if ang_speed == None else -ang_speed)
                else:
                    self.publish_velocity.publish_velocity(lin_speed, -ANGULAR_VELOCITY if ang_speed == None else -ang_speed)

            while self.odom_data.angle_360 < angle:
                if self.ctrl_c:
                        self.publish_velocity.publish_velocity()
                        return
                
                if lin_speed == None:
                    self.publish_velocity.publish_velocity(0, TURN_CORRECTION_SPEED*ANGULAR_VELOCITY if ang_speed == None else TURN_CORRECTION_SPEED*ang_speed)
                else:
                    self.publish_velocity.publish_velocity(lin_speed, TURN_CORRECTION_SPEED*ANGULAR_VELOCITY if ang_speed == None else TURN_CORRECTION_SPEED*ang_speed)

            self.publish_velocity.publish_velocity()
        else:
            initial_angle = self.odom_data.angle_360

            if lin_speed == None:
                self.publish_velocity.publish_velocity(0, ANGULAR_VELOCITY if ang_speed == None else ang_speed)
            else:
                self.publish_velocity.publish_velocity(lin_speed, ANGULAR_VELOCITY if ang_speed == None else ang_speed)

            time.sleep(0.6)

            if requires_crossing_0_360:
                while self.odom_data.angle_360 >= initial_angle:
                    if self.ctrl_c:
                        self.publish_velocity.publish_velocity()
                        return
                     # if not self.acquired_color:
                #     angle_to_turn = self.odom_data.angle_360 + 180
                #     self.turn_to_angle_360_system(angle_to_turn if angle_to_turn < 360 else angle_to_turn - 360)

                #     self.set_color()
                #     return
                # else:
                #     pass
                    if lin_speed == None:
                        self.publish_velocity.publish_velocity(0, ANGULAR_VELOCITY if ang_speed == None else ang_speed)
                    else:
                        self.publish_velocity.publish_velocity(lin_speed, ANGULAR_VELOCITY if ang_speed == None else ang_speed)

            while self.odom_data.angle_360 < angle:
                if self.ctrl_c:
                        self.publish_velocity.publish_velocity()
                        return

                if lin_speed == None:
                    self.publish_velocity.publish_velocity(0, ANGULAR_VELOCITY if ang_speed == None else ang_speed)
                else:
                    self.publish_velocity.publish_velocity(lin_speed, ANGULAR_VELOCITY if ang_speed == None else ang_speed)

            while self.odom_data.angle_360 > angle:
                if self.ctrl_c:
                        self.publish_velocity.publish_velocity()
                        return
                
                if lin_speed == None:
                    self.publish_velocity.publish_velocity(0, TURN_CORRECTION_SPEED*(-ANGULAR_VELOCITY) if ang_speed == None else TURN_CORRECTION_SPEED*(-ang_speed))
                else:
                    self.publish_velocity.publish_velocity(lin_speed, TURN_CORRECTION_SPEED*(-ANGULAR_VELOCITY) if ang_speed == None else TURN_CORRECTION_SPEED*(-ang_speed))

            self.publish_velocity.publish_velocity()

    def set_color(self):
        pass

    def main_loop(self):
        while not self.ctrl_c:
            if self.odom_data.initial_data_loaded and self.lidar_data.initial_data_loaded:

                # if not self.acquired_color:
                #     angle_to_turn = self.odom_data.angle_360 + 180
                #     self.turn_to_angle_360_system(angle_to_turn if angle_to_turn < 360 else angle_to_turn - 360)

                #     self.set_color()
                #     return
                # else:
                #     pass
                self.camera.take_picture("test.bmp")
                return


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

    def scan_callback(self, scan_data):
        self.ranges = scan_data.ranges
        self.initial_data_loaded = True

class Take_photo():
   
    def __init__(self):
        self.cvbridge_interface = CvBridge()




        self.bridge = CvBridge()
        self.image_received = False
       # self.base_image_path = Path("~/catkin_ws/src/team16/")
        #self.base_image_path.mkdir(parents=True, exist_ok=True)

        # Connect image topic
        img_topic = "/camera/rgb/image_raw"
        self.image_sub = rospy.Subscriber(img_topic, Image, self.callback)

        # Allow up to one second to connection
        rospy.sleep(1)



    def callback(self, data):

        # Convert image to OpenCV format
        try:
            cv_image = self.cvbridge_interface.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.image = cv_image
        self.image_received = True

    def take_picture(self, img_title):
        base_image_path = Path("\catkin_ws\src\team16\photos")
        full_image_path = base_image_path.joinpath(img_title)
        if self.image_received:
            cv2.imwrite(str(full_image_path), self.image)
            cv2.imshow("hello", self.image)
            cv2.waitKey(0)

     
     
            
           
            
           
       


if __name__ == '__main__':
    try:
        main_instance = Main()
        main_instance.main_loop()
    except rospy.ROSInterruptException:
        pass