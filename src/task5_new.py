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
LINEAR_VELOCITY = 0.25
ANGULAR_VELOCITY = 0.8
TURN_CORRECTION_SPEED = 0.3
COLOR_THRESHOLD_VALUE = 150
ANGLE_DETECTION_THRESHOLD = 40
DISTANCE_FROM_START_THRESHOLD = 2
RANGE_THRESHOLD = 1.2
DISTANCE_FACTOR = 0.1
REVERSE_TIME = 1.6
HOMING_PRECISION = 0.1
CLEARANCE_THRESHOLD = 0.3
FORWARD_STOPPING_THRESHOLD = 0.30
HOMING_THRESHOLD = 150

base_image_path = Path("/home/student/catkin_ws/src/team16/snaps")
base_image_path.mkdir(parents=True, exist_ok=True)

map_path = Path("/home/student/catkin_ws/src/team16/maps")
map_path.mkdir(parents=True, exist_ok=True)
map_path = Path("/home/student/catkin_ws/src/team16/maps/task5_map")

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
        self.camera = Camera()
        self.map_saver = Map_saver()

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
        self.picture_taken = False

        self.visited_points = []

        # Thresholds for ["Blue", "Red", "Green", "Yellow"]
        self.lower = [(115, 224, 100), (0, 185, 100), (35, 150, 100), (25, 150, 100)]
        self.upper = [(130, 255, 255), (10, 255, 255), (70, 255, 255), (33, 255, 255)]

    def shutdownhook(self):
        print(f"Stopping the '{self.node_name}' node at: {rospy.get_time()}")
        self.ctrl_c = True
        self.publish_velocity.shutdown()

    def check_if_target_visible(self):

        if self.color_visible()[0] == self.color_index:
            return True
        
        return False

    def is_target_close_enough(self):
        if not self.camera.image_received:
            return None

        cv_img = self.camera.image
        
        height, width, _ = cv_img.shape
        crop_width = width
        crop_height = 400
        crop_x = int((width/2) - (crop_width/2))
        crop_y = int(height/2 + 200)

        crop_img = cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        moments = []

        for i in range(len(self.lower)):
            moments.append(cv2.moments(cv2.inRange(hsv_img, self.lower[i], self.upper[i])))
     
        index_of_best_matching = -1
        highest_matching_color_value = 0

        for i, v in enumerate(moments):
            if v["m00"] > highest_matching_color_value:
                highest_matching_color_value = v["m00"]
                index_of_best_matching = i

        if index_of_best_matching == -1 or moments[index_of_best_matching]["m00"] < COLOR_THRESHOLD_VALUE:
            return False

        return moments[index_of_best_matching]["m00"] > HOMING_THRESHOLD

    def color_visible(self):
        
        if not self.camera.image_received:
            return None

        cv_img = self.camera.image
        
        height, width, _ = cv_img.shape
        crop_width = width
        crop_height = height
        crop_x = int((width/2) - (crop_width/2))
        crop_y = int((height/2) - (crop_height/2))

        crop_img = cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        moments = []

        for i in range(len(self.lower)):
            moments.append(cv2.moments(cv2.inRange(hsv_img, self.lower[i], self.upper[i])))
     
        index_of_best_matching = -1
        highest_matching_color_value = 0

        for i, v in enumerate(moments):
            if v["m00"] > highest_matching_color_value:
                highest_matching_color_value = v["m00"]
                index_of_best_matching = i

        if index_of_best_matching == -1 or moments[index_of_best_matching]["m00"] < COLOR_THRESHOLD_VALUE:
            return -1, 0

        cy = moments[index_of_best_matching]['m10'] / (moments[index_of_best_matching]['m00'] + 1e-5)

        y_error = (crop_width / 2) - cy

        kp = 1.0 / 2000.0

        ang_vel = kp * y_error

        if ang_vel > 1:
            ang_vel = 1
        if ang_vel < -1:
            ang_vel = -1
        
        return index_of_best_matching, ang_vel

    def turn_to_angle_360_system(self, angle, lin_speed = None, ang_speed = None, ignore_color = False):
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

            time.sleep(1)

            if requires_crossing_0_360:
                while self.odom_data.angle_360 <= initial_angle:
                    if self.ctrl_c or (self.check_if_target_visible() and not ignore_color):
                        self.publish_velocity.publish_velocity()
                        return
                    
                    if lin_speed == None:
                        self.publish_velocity.publish_velocity(0, -ANGULAR_VELOCITY if ang_speed == None else -ang_speed)
                    else:
                        self.publish_velocity.publish_velocity(lin_speed, -ANGULAR_VELOCITY if ang_speed == None else -ang_speed)

            
            while self.odom_data.angle_360 > angle:
                if self.ctrl_c or (self.check_if_target_visible() and not ignore_color):
                        self.publish_velocity.publish_velocity()
                        return
                
                if lin_speed == None:
                    self.publish_velocity.publish_velocity(0, -ANGULAR_VELOCITY if ang_speed == None else -ang_speed)
                else:
                    self.publish_velocity.publish_velocity(lin_speed, -ANGULAR_VELOCITY if ang_speed == None else -ang_speed)

            while self.odom_data.angle_360 < angle:
                if self.ctrl_c or (self.check_if_target_visible() and not ignore_color):
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

            time.sleep(1)

            if requires_crossing_0_360:
                while self.odom_data.angle_360 >= initial_angle:
                    if self.ctrl_c or (self.check_if_target_visible() and not ignore_color):
                        self.publish_velocity.publish_velocity()
                        return
                    
                    if lin_speed == None:
                        self.publish_velocity.publish_velocity(0, ANGULAR_VELOCITY if ang_speed == None else ang_speed)
                    else:
                        self.publish_velocity.publish_velocity(lin_speed, ANGULAR_VELOCITY if ang_speed == None else ang_speed)

            while self.odom_data.angle_360 < angle:
                if self.ctrl_c or (self.check_if_target_visible() and not ignore_color):
                        self.publish_velocity.publish_velocity()
                        return

                if lin_speed == None:
                    self.publish_velocity.publish_velocity(0, ANGULAR_VELOCITY if ang_speed == None else ang_speed)
                else:
                    self.publish_velocity.publish_velocity(lin_speed, ANGULAR_VELOCITY if ang_speed == None else ang_speed)

            while self.odom_data.angle_360 > angle:
                if self.ctrl_c or (self.check_if_target_visible() and not ignore_color):
                        self.publish_velocity.publish_velocity()
                        return
                
                if lin_speed == None:
                    self.publish_velocity.publish_velocity(0, TURN_CORRECTION_SPEED*(-ANGULAR_VELOCITY) if ang_speed == None else TURN_CORRECTION_SPEED*(-ang_speed))
                else:
                    self.publish_velocity.publish_velocity(lin_speed, TURN_CORRECTION_SPEED*(-ANGULAR_VELOCITY) if ang_speed == None else TURN_CORRECTION_SPEED*(-ang_speed))

            self.publish_velocity.publish_velocity()

    def offset_space_array(self, array):
        if array == None:
            return None

        for i in range(len(array)):
            offset_angle = array[i][0] + self.odom_data.angle_360
            corrected_offset_angle = offset_angle - 360 if offset_angle >= 360 else offset_angle
            array[i] = (corrected_offset_angle, array[i][1])
        
        return array

    def choose_angle(self):
        current_x, current_y = self.odom_data.posx, self.odom_data.posy

        space_array = self.offset_space_array(self.lidar_data.get_space_array())

        mid_angles = list(map(lambda x: x[0], space_array))
        widths = list(map(lambda x: x[1], space_array))

        visited_points_distance_diff_coefficients = []
        angle_of_points = []

        for point_x, point_y in self.visited_points:
            euc_distance = sqrt((current_x - point_x)**2 + (current_y-point_y)**2)

            distance_inverse = 1
            if euc_distance != 0:
                distance_inverse = 1/(euc_distance)**DISTANCE_FACTOR

            visited_points_distance_diff_coefficients.append(distance_inverse)

            angle_radians = atan2(point_y - current_y, point_x - current_x)
            angle_degrees = (angle_radians if angle_radians >= 0 else angle_radians + 2*pi) * 180 / pi

            angle_of_points.append(angle_degrees)
        
        correction_factor_of_angles = []

        for i in range(len(mid_angles)):
            sum_of_correction_factor_of_points = 1
            for j in range(len(angle_of_points)):
                difference_clockwise = mid_angles[i] - angle_of_points[j] + 360 if angle_of_points[j] > mid_angles[i] else mid_angles[i] - angle_of_points[j]
                difference_anti_clockwise = 360 - mid_angles[i] + angle_of_points[j] if angle_of_points[j] < mid_angles[i] else angle_of_points[j] - mid_angles[i]

                angle_difference = min(difference_clockwise, difference_anti_clockwise)
                angle_difference_radians = angle_difference * pi/180

                cosine_ang_diff = cos(angle_difference_radians)

                correction_factor_of_point = cosine_ang_diff * visited_points_distance_diff_coefficients[j]

                sum_of_correction_factor_of_points += correction_factor_of_point
            
            correction_factor_of_angles.append(1/sum_of_correction_factor_of_points)
        
        for i in range(len(widths)):
            widths[i] *= correction_factor_of_angles[i]
        
        zipped = list(zip(mid_angles, widths))

        zipped.sort(key=lambda x: x[1], reverse=True)

        return zipped[0][0]

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

    def do_360_and_scan(self):
        initial_angle = self.odom_data.angle_360

        self.publish_velocity.publish_velocity(0, ANGULAR_VELOCITY)

        time.sleep(1.5)

        while abs(self.odom_data.angle_360 - initial_angle) > 5:
            if self.check_if_target_visible():
                self.publish_velocity.publish_velocity()
                return True
        
        self.publish_velocity.publish_velocity()
        return False

    def main_loop(self):
        while not self.ctrl_c:
            if self.odom_data.initial_data_loaded and self.lidar_data.initial_data_loaded:

                self.map_saver.save_map()

                if not self.target_found:

                    if not self.check_if_target_visible() or self.picture_taken:
                        if not len(self.lidar_data.get_space_array()) == 0:
                            self.turn_to_angle_360_system(self.choose_angle(), ignore_color=self.picture_taken)
                        else:
                            angle_to_turn = self.odom_data.angle_360 + 180
                            self.turn_to_angle_360_system(angle_to_turn if angle_to_turn < 360 else angle_to_turn - 360, ignore_color=self.picture_taken)

                        self.visited_points.append((self.odom_data.posx, self.odom_data.posy))

                        if self.check_if_target_visible() and not self.picture_taken:
                            self.publish_velocity.publish_velocity()
                        else:
                            while self.can_move_forward():
                                self.publish_velocity.publish_velocity(LINEAR_VELOCITY, 0)

                                if self.ctrl_c or (self.check_if_target_visible() and not self.picture_taken):
                                    self.publish_velocity.publish_velocity()
                                    break
                        
                        if self.check_if_target_visible() and not self.picture_taken:
                            self.publish_velocity.publish_velocity()
                            pass
                        
                        elif self.lidar_data.ranges[180] > 0.7 and self.lidar_data.ranges[130] > 0.7 and self.lidar_data.ranges[230] > 0.7:
                            self.publish_velocity.publish_velocity()
                            time.sleep(1)
                            self.publish_velocity.publish_velocity(-LINEAR_VELOCITY, 0)
                            time.sleep(REVERSE_TIME)
                        elif self.lidar_data.ranges[180] > 0.4 and self.lidar_data.ranges[130] > 0.4 and self.lidar_data.ranges[230] > 0.4:
                            self.publish_velocity.publish_velocity()
                            time.sleep(1)
                            self.publish_velocity.publish_velocity(-LINEAR_VELOCITY, 0)
                            time.sleep(0.5*REVERSE_TIME)
                        else:
                            self.publish_velocity.publish_velocity()
                            time.sleep(1)
                            self.publish_velocity.publish_velocity(-LINEAR_VELOCITY, 0)
                            time.sleep(0.2*REVERSE_TIME)
                        self.publish_velocity.publish_velocity()
                    else:
                        # print("TARGET DETECTED: Beaconing initiated.")
                        self.publish_velocity.publish_velocity()
                        self.target_found = True
                        continue
                else:
                    #Homing
                    # _, angular_vel = self.color_visible()

                    # while abs(angular_vel) > HOMING_PRECISION:
                    #     _, angular_vel = self.color_visible()
                    #     self.publish_velocity.publish_velocity(0, angular_vel)

                    # while self.can_move_forward():
                    #     _, angular_vel = self.color_visible()
                    #     self.publish_velocity.publish_velocity(LINEAR_VELOCITY, angular_vel)

                    # self.publish_velocity.publish_velocity(-LINEAR_VELOCITY, 0)
                    # time.sleep(0.6)

                    # if not self.is_target_close_enough():
                    #     # self.target_found = False

                    #     left_avg = sum(self.lidar_data.ranges[:60])/60
                    #     right_avg = sum(self.lidar_data.ranges[-60:])/60

                    #     if left_avg > right_avg:

                    #         angle_to_turn = self.odom_data.angle_360 + 70
                    #         self.turn_to_angle_360_system(angle_to_turn if angle_to_turn < 360 else angle_to_turn - 360, ignore_color=True)
                    #         while self.can_move_forward():
                    #             self.publish_velocity.publish_velocity(LINEAR_VELOCITY, 0)
                            
                    #         if self.lidar_data.ranges[180] > 0.7 and self.lidar_data.ranges[130] > 0.7 and self.lidar_data.ranges[230] > 0.7:
                    #             self.publish_velocity.publish_velocity()
                    #             time.sleep(1)
                    #             self.publish_velocity.publish_velocity(-LINEAR_VELOCITY, 0)
                    #             time.sleep(REVERSE_TIME)
                    #         elif self.lidar_data.ranges[180] > 0.4 and self.lidar_data.ranges[130] > 0.4 and self.lidar_data.ranges[230] > 0.4:
                    #             self.publish_velocity.publish_velocity()
                    #             time.sleep(1)
                    #             self.publish_velocity.publish_velocity(-LINEAR_VELOCITY, 0)
                    #             time.sleep(0.5*REVERSE_TIME)
                    #         else:
                    #             self.publish_velocity.publish_velocity()
                    #             time.sleep(1)
                    #             self.publish_velocity.publish_velocity(-LINEAR_VELOCITY, 0)
                    #             time.sleep(0.2*REVERSE_TIME)
                    #         self.publish_velocity.publish_velocity()

                    #     else:
                    #         angle_to_turn = self.odom_data.angle_360 - 70
                    #         self.turn_to_angle_360_system(angle_to_turn if angle_to_turn >= 0 else angle_to_turn + 360, ignore_color=True)
                    #         while self.can_move_forward():
                    #             self.publish_velocity.publish_velocity(LINEAR_VELOCITY, 0)
                            
                    #         if self.lidar_data.ranges[180] > 0.7 and self.lidar_data.ranges[130] > 0.7 and self.lidar_data.ranges[230] > 0.7:
                    #             self.publish_velocity.publish_velocity()
                    #             time.sleep(1)
                    #             self.publish_velocity.publish_velocity(-LINEAR_VELOCITY, 0)
                    #             time.sleep(REVERSE_TIME)
                    #         elif self.lidar_data.ranges[180] > 0.4 and self.lidar_data.ranges[130] > 0.4 and self.lidar_data.ranges[230] > 0.4:
                    #             self.publish_velocity.publish_velocity()
                    #             time.sleep(1)
                    #             self.publish_velocity.publish_velocity(-LINEAR_VELOCITY, 0)
                    #             time.sleep(0.5*REVERSE_TIME)
                    #         else:
                    #             self.publish_velocity.publish_velocity()
                    #             time.sleep(1)
                    #             self.publish_velocity.publish_velocity(-LINEAR_VELOCITY, 0)
                    #             time.sleep(0.2*REVERSE_TIME)
                    #         self.publish_velocity.publish_velocity()

                    #     if not self.do_360_and_scan():
                    #         self.target_found = False
                        
                    # else:
                    #     self.do_360_and_scan()

                    #     _, angular_vel = self.color_visible()

                    #     while abs(angular_vel) > HOMING_PRECISION:
                    #         _, angular_vel = self.color_visible()
                    #         self.publish_velocity.publish_velocity(0, angular_vel)

                    _, angular_vel = self.color_visible()

                    while abs(angular_vel) > HOMING_PRECISION:
                        _, angular_vel = self.color_visible()
                        self.publish_velocity.publish_velocity(0, angular_vel)

                    self.publish_velocity.publish_velocity()
                    self.camera.take_picture("the_beacon")
                    self.picture_taken = True
                    self.target_found = False


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

    def get_space_array(self):
        previous_inf = False

        angle_before_first_obstacle = 0
        first_angle_set = False

        angle_array = []
        beginning_of_current_space = None

        for i, _range in enumerate(self.ranges):
            if _range > RANGE_THRESHOLD:
                if not first_angle_set:
                    angle_before_first_obstacle += 1

                if not previous_inf:
                    beginning_of_current_space = i
                    previous_inf = True

            else:
                if not first_angle_set:
                    first_angle_set = True
                    previous_inf = False

                if previous_inf and first_angle_set:
                    mid_angle = beginning_of_current_space + (i - beginning_of_current_space)/2
                    size_of_angle = i - beginning_of_current_space
                    
                    angle_array.append((mid_angle, size_of_angle))
                    beginning_of_current_space = None
                
                if previous_inf:
                    previous_inf = False
        
        if not first_angle_set:
            return [(0, 360)]

        angle_after_last_obstacle = 0 if beginning_of_current_space == None else 360 - beginning_of_current_space

        first_and_last_angle = angle_after_last_obstacle + angle_before_first_obstacle

        beginning_of_last_angle = 360 - angle_after_last_obstacle

        mid_angle_before_correction = beginning_of_last_angle + first_and_last_angle/2

        mid_angle_of_first_and_last_angle = mid_angle_before_correction - 360 if mid_angle_before_correction >= 360 else mid_angle_before_correction
        
        if first_and_last_angle != 0:
            angle_array.append((mid_angle_of_first_and_last_angle, first_and_last_angle))

        angle_array.sort(key=lambda x: x[1], reverse=True)

        return angle_array


class CLI_colour():
    def __init__(self):
        cli = argparse.ArgumentParser(description=f"...")
        cli.add_argument("-colour", metavar="COL", type=String,
            default="blue", 
            help="The name of a colour (for example)")
        
        self.args = cli.parse_args(rospy.myargv()[1:])
        self.color = self.args.colour.data

class Camera():
   
    def __init__(self):
        self.cvbridge_interface = CvBridge()

        self.image_received = False
       # self.base_image_path = Path("~/catkin_ws/src/team16/")
        #self.base_image_path.mkdir(parents=True, exist_ok=True)

        # Connect image topic
        img_topic = "/camera/color/control"
        self.image_sub = rospy.Subscriber(img_topic, Image, self.callback)
        

        self.image = None
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

    def take_picture(self, img_name):
        full_image_path = base_image_path.joinpath(f"{img_name}.jpg")

        cv2.imwrite(str(full_image_path), self.image)
        print(f"Saved an image to '{full_image_path}'\n"
            f"image dims = {self.image.shape[0]}x{self.image.shape[1]}px\n"
            f"file size = {full_image_path.stat().st_size} bytes")

class Map_saver():

    def __init__(self):
        

        # rospy.init_node("map_getter", anonymous=True)

        # launch = roslaunch.scriptapi.ROSLaunch()
        # launch.start()
        pass

    def save_map(self):

        # rospy.init_node("map_getter", anonymous=True)

        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()

        print(f"Saving map at time: {rospy.get_time()}...")
        node = roslaunch.core.Node(package="map_server",
                                node_type="map_saver",
                                args=f"-f {map_path}")
        process = launch.launch(node)

if __name__ == '__main__':
    try:
        main_instance = Main()
        main_instance.main_loop()
    except rospy.ROSInterruptException:
        pass