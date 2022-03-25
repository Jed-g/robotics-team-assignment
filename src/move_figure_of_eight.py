#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class Publisher():

    def __init__(self):
        self.node_name = "move_figure_of_eight"
        topic_name = "cmd_vel"

        self.pub = rospy.Publisher(topic_name, Twist, queue_size=10)
        rospy.init_node(self.node_name)
        self.rate = rospy.Rate(1)  # hz

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        rospy.loginfo(f"The '{self.node_name}' node is active...")

    def shutdownhook(self):
        print(f"Stopping the '{self.node_name}' node at: {rospy.get_time()}")
        vel_cmd = Twist()
        self.pub.publish(vel_cmd)
        self.ctrl_c = True

    def main_loop(self):
        while not self.ctrl_c:
            vel_cmd = Twist()
            vel_cmd.linear.x = 0.26  # m/s
            vel_cmd.angular.z = 0.26  # rad/s
            self.pub.publish(vel_cmd)
            self.rate.sleep()

class Odom_data():

    def __init__(self):
        self.node_name = "odom_subscriber"
        topic_name = "odom"

        rospy.init_node(self.node_name, anonymous=True)
        self.sub = rospy.Subscriber(topic_name, Odometry, self.callback)
        rospy.loginfo("odom subscriber is active...")

        self.rate = rospy.Rate(1)  # Hz
        self.shutdown = False
        self.got_data = False

        rospy.on_shutdown(self.shutdown_hook)

    def shutdown_hook(self):
        self.shutdown = True

    def main_loop(self):
        # rospy.spin()
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
        self.output_string = f"x = {position.x:.2f}, y = {position.y:.2f}, theta_z = {yaw:.2f}"


if __name__ == '__main__':
    publisher_instance = Publisher()
    try:
        publisher_instance.main_loop()
    except rospy.ROSInterruptException:
        pass
