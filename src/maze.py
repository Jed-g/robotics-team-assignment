#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import pi, sqrt

FREQUENCY = 100

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

    def shutdownhook(self):
        print(f"Stopping the '{self.node_name}' node at: {rospy.get_time()}")
        self.ctrl_c = True
        self.publish_velocity.shutdown()

    def print_data(self):
        self.message_iteration += 1
        if self.message_iteration > FREQUENCY:
            self.message_iteration = 1
            print(self.odom_data.output_string)

    def update_distance_travelled(self):
        self.distance_travelled += sqrt((self.odom_data.posx - self.position_previous_iteration_x)**2+(
            self.odom_data.posy - self.position_previous_iteration_y)**2)

        self.position_previous_iteration_x = self.odom_data.posx
        self.position_previous_iteration_y = self.odom_data.posy

    def main_loop(self):
        while not self.ctrl_c:
            if self.odom_data.initial_data_loaded:
                if self.first_iteration:
                    self.position_previous_iteration_x = self.odom_data.posx
                    self.position_previous_iteration_y = self.odom_data.posy
                    self.first_iteration = False
                    print(self.odom_data.output_string)

                self.print_data()
                self.update_distance_travelled()

                if self.is_loop_1:
                    self.loop_1_completed()
                else:
                    self.loop_2_completed()

                angular_vel = ANGULAR_VELOCITY if self.is_loop_1 else -ANGULAR_VELOCITY  # rad/s
                self.publish_velocity.publish_velocity(
                    LINEAR_VELOCITY, angular_vel)

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
        self.posx = 0
        self.posy = 0
        self.angle = 0
        self.output_string = ""
        self.initial_x = 0
        self.initial_y = 0
        self.initial_angle = 0
        self.offset_angle_var = 0

    def callback(self, topic_message):
        orientation = topic_message.pose.pose.orientation

        (_, _, yaw) = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w], 'sxyz')

        position = topic_message.pose.pose.position
        self.posx = position.x
        self.posy = position.y
        self.angle = yaw

        if not self.initial_data_loaded:
            self.initial_x = self.posx
            self.initial_y = self.posy
            self.initial_angle = self.angle

        self.initial_data_loaded = True
        self.offset_angle()
        self.output_string = f"x={position.x - self.initial_x:.2f} [m], y={position.y - self.initial_y:.2f} [m], yaw={self.offset_angle_var*180/pi:.1f} [degrees]"
    
    def offset_angle(self):
        absolute_angle = self.angle if self.angle >= 0 else self.angle + 2*pi
        absolute_initial_angle = self.initial_angle if self.initial_angle >= 0 else self.initial_angle + 2*pi
        self.offset_angle_var = absolute_angle - absolute_initial_angle + 2*pi if absolute_angle < absolute_initial_angle else absolute_angle - absolute_initial_angle
        if self.offset_angle_var > pi:
            self.offset_angle_var = self.offset_angle_var - 2*pi



if __name__ == '__main__':
    try:
        main_instance = Main()
        main_instance.main_loop()
    except rospy.ROSInterruptException:
        pass
