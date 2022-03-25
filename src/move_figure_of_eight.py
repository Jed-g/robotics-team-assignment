#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import pi, sqrt

INITIAL_DISTANCE_THRESHOLD = 0.3
DESTINATION_THRESHOLD = 0.1
LINEAR_VELOCITY = 0.11
CIRCLE_RADIUS = 0.5
ANGULAR_VELOCITY = LINEAR_VELOCITY / CIRCLE_RADIUS

class Main():
    def __init__(self):
        self.node_name = "move_figure_of_eight"

        rospy.init_node(self.node_name)
        self.rate = rospy.Rate(1)  # hz

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        rospy.loginfo(f"The '{self.node_name}' node is active...")

        self.publish_velocity = Publish_velocity()
        self.odom_data = Odom_data()

        self.is_loop_1 = True
        self.initial_movement = False
    
    def loop_1_completed(self):
        if not self.initial_movement:
            if sqrt(self.odom_data.posx**2+self.odom_data.posy**2) >= INITIAL_DISTANCE_THRESHOLD:
                self.initial_movement = True
        elif sqrt(self.odom_data.posx**2+self.odom_data.posy**2) <= DESTINATION_THRESHOLD:
            self.initial_movement = False
            self.is_loop_1 = False

    def loop_2_completed(self):
        if not self.initial_movement:
            if sqrt(self.odom_data.posx**2+self.odom_data.posy**2) >= INITIAL_DISTANCE_THRESHOLD:
                self.initial_movement = True
        elif sqrt(self.odom_data.posx**2+self.odom_data.posy**2) <= DESTINATION_THRESHOLD:
            print("Manoeuver completed successfully")
            self.shutdownhook()


    def shutdownhook(self):
        print(f"Stopping the '{self.node_name}' node at: {rospy.get_time()}")
        self.ctrl_c = True
        self.publish_velocity.shutdown()


    def main_loop(self):
        while not self.ctrl_c:
            print(self.odom_data.output_string)

            if self.is_loop_1:
                self.loop_1_completed()
            else:
                self.loop_2_completed()

            angular_vel = ANGULAR_VELOCITY if self.is_loop_1 else -ANGULAR_VELOCITY # rad/s
            self.publish_velocity.publish_velocity(LINEAR_VELOCITY, angular_vel)

            self.rate.sleep()

class Publish_velocity():

    def __init__(self):
        topic_name = "cmd_vel"
        self.pub = rospy.Publisher(topic_name, Twist, queue_size=10)

    def publish_velocity(self, linear = 0, angular = 0):
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
