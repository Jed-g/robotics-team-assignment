#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from math import inf, pi, sqrt

FREQUENCY = 10
LINEAR_VELOCITY = 0.25
ANGULAR_VELOCITY = 0.8

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
        self.initial_data_loaded = True
        orientation = topic_message.pose.pose.orientation

        (_, _, yaw) = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w], 'sxyz')

        position = topic_message.pose.pose.position
        self.posx = position.x
        self.posy = position.y
        self.angle = yaw

class Lidar_data():
    
    def __init__(self):
        topic_name = "scan"
        self.subscriber = rospy.Subscriber(topic_name, LaserScan, self.scan_callback)
        self.ranges = []
        self.initial_data_loaded = False

    def scan_callback(self, scan_data):
        self.initial_data_loaded = True
        self.ranges = scan_data.ranges

    def get_mid_angle(self):
        greatest_space = 0
        current_space = 0
        previous_inf = False
        beginning_of_greatest = 0

        angle_before_first_obstacle = 0
        first_angle_set = False
        angle_after_last_obstacle = 0

        for i, range in enumerate(self.ranges):
            if range == inf:
                if not first_angle_set:
                    angle_before_first_obstacle += 1

                if not previous_inf:
                    previous_inf = True

                current_space += 1

            else:
                first_angle_set = True

                if current_space > greatest_space:
                    greatest_space = current_space
                    beginning_of_greatest = i - current_space
                
                current_space = 0
                previous_inf = False
        
        angle_after_last_obstacle = current_space



        if angle_before_first_obstacle + angle_after_last_obstacle == 0 or greatest_space == 0:
            return None
        elif angle_before_first_obstacle + angle_after_last_obstacle > greatest_space:
            angle = (angle_after_last_obstacle + angle_before_first_obstacle) / 2
            return 360 - angle_after_last_obstacle + angle if 360 - angle_after_last_obstacle + angle < 360 else angle_after_last_obstacle + angle
        else:
            return beginning_of_greatest + greatest_space / 2


if __name__ == '__main__':
    try:
        main_instance = Main()
        main_instance.main_loop()
    except rospy.ROSInterruptException:
        pass
