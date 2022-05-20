#!/usr/bin/env python3

from ast import Pass
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from math import inf, pi, sqrt, sin, cos, atan2
import numpy as np

FREQUENCY = 10
LINEAR_VELOCITY = 0.25
ANGULAR_VELOCITY = 0.4
CLEARANCE_THRESHOLD = 0.32
FORWARD_STOPPING_THRESHOLD = 0.4
WALL_PROXIMITY_THRESHOLD = 0.25

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

            self.finding_wall = True
            self.locked_onto_wall = False

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

            self.publish_velocity.publish_velocity()
        else:
            initial_angle = self.odom_data.angle_360

            if requires_crossing_0_360:
                while self.odom_data.angle_360 >= initial_angle:
                    if self.ctrl_c:
                        self.publish_velocity.publish_velocity()
                        return
                    
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

            self.publish_velocity.publish_velocity()
    
    def can_move_forward(self):
        data = list(self.lidar_data.ranges[-35:])
        data.extend(list(self.lidar_data.ranges[:35]))

        if min(data) < CLEARANCE_THRESHOLD:
            return False

        data = list(self.lidar_data.ranges[-2:])
        data.extend(list(self.lidar_data.ranges[:2]))

        if min(data) < FORWARD_STOPPING_THRESHOLD:
            return False

        return True

    def correct_wall(self):
        # left_front = sum(self.lidar_data.ranges[75:90])/15
        # left_back = sum(self.lidar_data.ranges[90:105])/15
        # print("test")
        # if left_front >= left_back:
        #     self.publish_velocity.publish_velocity(LINEAR_VELOCITY, 0.8)
        # else:
        #     self.publish_velocity.publish_velocity(LINEAR_VELOCITY, -0.8)

        # distance = min(self.lidar_data.ranges[70:110])
        # print(distance)
        # if distance < WALL_PROXIMITY_THRESHOLD:
        #     self.publish_velocity.publish_velocity(LINEAR_VELOCITY, -0.8)
        # else:
        #     self.publish_velocity.publish_velocity(LINEAR_VELOCITY, 0.8)

        x_of_points = []
        y_of_points = []

        for i, v in enumerate(list(self.lidar_data.ranges[60:110])):
            if v == inf:
                continue

            angle = (i + 60) * pi / 180

            x_of_points.append(v * cos(angle))
            y_of_points.append(v * sin(angle))

        if len(x_of_points) == 0 or len(y_of_points) == 0:
            self.finding_wall = True
            self.locked_onto_wall = False
            return

        a, b = np.polyfit(np.array(x_of_points), np.array(y_of_points), 1)

        distance_from_wall = abs(b)/sqrt(a**2+1)

        angle = 180 * atan2(a, 1) / pi
        # print(angle)
        print(distance_from_wall)
        if distance_from_wall < WALL_PROXIMITY_THRESHOLD:
            self.publish_velocity.publish_velocity(LINEAR_VELOCITY, -0.6)
        else:
            self.publish_velocity.publish_velocity(LINEAR_VELOCITY, 0.6)
        
        if abs(angle) > 5:
            pass


    def main_loop(self):
        while not self.ctrl_c:
            if self.odom_data.initial_data_loaded and self.lidar_data.initial_data_loaded:
                print(self.finding_wall)
                if self.finding_wall:
                    if self.can_move_forward():
                        self.publish_velocity.publish_velocity(LINEAR_VELOCITY, 0)
                    else:
                        self.publish_velocity.publish_velocity()
                        self.finding_wall = False
                else:
                    if not self.locked_onto_wall:
                        angle_to_turn = self.odom_data.angle_360 - 86
                        self.turn_to_angle_360_system(angle_to_turn if angle_to_turn >= 0 else angle_to_turn + 360)
                        self.locked_onto_wall = True
                    else:
                        if self.can_move_forward():
                            self.correct_wall()
                        else:
                            angle_to_turn = self.odom_data.angle_360 - 86
                            self.turn_to_angle_360_system(angle_to_turn if angle_to_turn >= 0 else angle_to_turn + 360)                       

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

if __name__ == '__main__':
    try:
        main_instance = Main()
        main_instance.main_loop()
    except rospy.ROSInterruptException:
        pass
