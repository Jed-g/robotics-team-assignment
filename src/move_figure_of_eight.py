#!/usr/bin/env python3

from msilib.schema import PublishComponent
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import pi


class Main():
    def __init__(self):
        self.node_name = "move_figure_of_eight"

        rospy.init_node(self.node_name)
        self.rate = rospy.Rate(10)  # hz

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        rospy.loginfo(f"The '{self.node_name}' node is active...")

        self.publish_velocity = Publish_velocity()
        self.odom_data = Odom_data()

        self.odom_data.odom_print_data()


    def shutdownhook(self):
        print(f"Stopping the '{self.node_name}' node at: {rospy.get_time()}")
        vel_cmd = Twist()
        self.pub.publish(vel_cmd)
        self.ctrl_c = True
        self.odom_data.shutdown()
        self.publish_velocity.shutdown()


    def main_loop(self):
        while not self.ctrl_c:
            vel_cmd = Twist()
            vel_cmd.linear.x = 0.26  # m/s
            vel_cmd.angular.z = 0.26  # rad/s
            self.pub.publish(vel_cmd)
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

        self.rate = rospy.Rate(1)  # Hz
        self.got_data = False
        self.shutdown = False

    def shutdown(self):
        self.shutdown = True

    def odom_print_data(self):
        while not self.shutdown:
            if self.got_data:
                print(self.output_string)
            self.rate.sleep()

    def callback(self, topic_message):
        orientation = topic_message.pose.pose.orientation

        (_, _, yaw) = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w], 'sxyz')

        position = topic_message.pose.pose.position

        self.got_data = True
        self.output_string = f"x={position.x:.2f} [m], y={position.y:.2f} [m], yaw={yaw*180/pi:.1f} [degrees]"


if __name__ == '__main__':
    main_instance = Main()
    try:
        main_instance.main_loop()
    except rospy.ROSInterruptException:
        pass
