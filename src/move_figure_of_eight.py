#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import pi, sqrt, atan

INITIAL_DISTANCE_THRESHOLD = 0.3
DESTINATION_THRESHOLD = 0.05
CIRCLE_RADIUS = 0.5
TIME_PER_LOOP = 28
LINEAR_VELOCITY = 2 * pi * CIRCLE_RADIUS / TIME_PER_LOOP
ANGULAR_VELOCITY = LINEAR_VELOCITY / CIRCLE_RADIUS
ANGLE_PRECISION = 0.01
ANGLE_CORRECTION_SPEED = 0.25
DISTANCE_CORRECTION_PRECISION = 0.02
DISTANCE_CORRECTION_SPEED = 0.08
FREQUENCY = 50


class Main():
    def __init__(self):
        self.node_name = "move_figure_of_eight"

        rospy.init_node(self.node_name)
        self.rate = rospy.Rate(FREQUENCY)  # hz

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        rospy.loginfo(f"The '{self.node_name}' node is active...")

        self.publish_velocity = Publish_velocity()
        self.odom_data = Odom_data()

        self.is_loop_1 = True
        self.message_iteration = 1
        self.distance_travelled = 0
        self.position_previous_iteration_x = 0
        self.position_previous_iteration_y = 0

    def loop_1_completed(self):
        if (sqrt(self.odom_data.posx**2+self.odom_data.posy**2) <= DESTINATION_THRESHOLD and self.distance_travelled > INITIAL_DISTANCE_THRESHOLD
            ) or self.distance_travelled >= 2*pi*CIRCLE_RADIUS:

            self.is_loop_1 = False
            self.correct_position()
            self.distance_travelled = 0

    def loop_2_completed(self):
        if (sqrt(self.odom_data.posx**2+self.odom_data.posy**2) <= DESTINATION_THRESHOLD and self.distance_travelled > INITIAL_DISTANCE_THRESHOLD
            ) or self.distance_travelled >= 2*pi*CIRCLE_RADIUS:

            self.correct_position()
            self.distance_travelled = 0
            print("Manoeuver completed successfully")
            self.shutdownhook()

    def shutdownhook(self):
        print(f"Stopping the '{self.node_name}' node at: {rospy.get_time()}")
        self.ctrl_c = True
        self.publish_velocity.shutdown()

    def correct_position(self):
        def sign(x):
            return 1 if x >= 0 else -1

        angle = atan(self.odom_data.posy/self.odom_data.posx)
        reverse = False

        if self.odom_data.posx >= 0:
            reverse = True

        if sqrt(self.odom_data.posx**2+self.odom_data.posy**2) > DISTANCE_CORRECTION_PRECISION:
            while abs(angle - self.odom_data.angle) - ANGLE_PRECISION > 0 and not self.ctrl_c:
                self.publish_velocity.publish_velocity(
                    0, sign(angle - self.odom_data.angle)*ANGLE_CORRECTION_SPEED)
                self.print_data()
                self.rate.sleep()

            while sqrt(self.odom_data.posx**2+self.odom_data.posy**2) > DISTANCE_CORRECTION_PRECISION and not self.ctrl_c:
                self.publish_velocity.publish_velocity(
                    DISTANCE_CORRECTION_SPEED if not reverse else -DISTANCE_CORRECTION_SPEED, 0)
                self.print_data()
                self.rate.sleep()

        while abs(self.odom_data.angle) - ANGLE_PRECISION > 0 and not self.ctrl_c:
            self.publish_velocity.publish_velocity(
                0, sign(-self.odom_data.angle)*ANGLE_CORRECTION_SPEED)
            self.print_data()
            self.rate.sleep()

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

        self.posx = 0
        self.posy = 0
        self.angle = 0
        self.output_string = ""

    def callback(self, topic_message):
        orientation = topic_message.pose.pose.orientation

        (_, _, yaw) = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w], 'sxyz')

        position = topic_message.pose.pose.position
        self.posx = position.x
        self.posy = position.y
        self.angle = yaw

        self.output_string = f"x={position.x:.2f} [m], y={position.y:.2f} [m], yaw={yaw*180/pi:.1f} [degrees]"


if __name__ == '__main__':
    try:
        main_instance = Main()
        main_instance.main_loop()
    except rospy.ROSInterruptException:
        pass
