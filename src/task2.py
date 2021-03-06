#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from math import inf, pi, sqrt, sin, cos, atan2
import numpy as np
import time

FREQUENCY = 10
LINEAR_VELOCITY = 0.25
ANGULAR_VELOCITY = 1.2
RANGE_THRESHOLD = 2.2
CLEARANCE_THRESHOLD = 0.32
FORWARD_STOPPING_THRESHOLD = 0.4
WALL_PROXIMITY_THRESHOLD = 0.4
WALL_PROXIMITY_THRESHOLD_PRECISION = 0.05
MAXIMUM_WALL_DISTANCE_CORRECTION_ANGLE = 30
MAXIMUM_WALL_DISTANCE_CORRECTION_ANGLE_PRECISION = 5
WALL_PARALLEL_THRESHOLD_ANGLE = 5
CONVEX_CORNER_THRESHOLD = 0.3
ANGLE_CORRECTION_SPEED = 1
CONVEX_CORNER_CORRECTION_SPEED = 1
STOP_TO_CORRECT_ANGLE_THRESHOLD = 10
REVERSE_TIME = 1.4
TURN_CORRECTION_SPEED = 0.3
DISTANCE_FACTOR = 0.1

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

        # self.is_moving_forward = False
        # self.correct_distance_to_wall = False
        # self.wall_too_far = False
        self.visited_points = []

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

    def offset_space_array(self, array):
        if array == None:
            return None

        for i in range(len(array)):
            offset_angle = array[i][0] + self.odom_data.angle_360
            corrected_offset_angle = offset_angle - 360 if offset_angle >= 360 else offset_angle
            array[i] = (corrected_offset_angle, array[i][1])
        
        return array

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

    def follow_left_wall(self):

        x_of_points = []
        y_of_points = []

        for i, v in enumerate(list(self.lidar_data.ranges[15:60])):
            if v == inf:
                continue

            if v > 0.7:
                continue

            angle = (i + 15) * pi / 180

            x_of_points.append(v * cos(angle))
            y_of_points.append(v * sin(angle))

        if len(x_of_points) == 0 or len(y_of_points) == 0:
            angle_to_turn = self.odom_data.angle_360 - 1
            self.turn_to_angle_360_system(angle_to_turn if angle_to_turn >= 0 else 358)
            return

        a, b = np.polyfit(np.array(x_of_points), np.array(y_of_points), 1)

        distance_from_wall = abs(b)/sqrt(a**2+1)

        angle = 180 * atan2(a, 1) / pi
        # print(angle)
        # print(distance_from_wall)
        print(distance_from_wall)
        if abs(distance_from_wall - WALL_PROXIMITY_THRESHOLD) > WALL_PROXIMITY_THRESHOLD_PRECISION:

            if distance_from_wall - WALL_PROXIMITY_THRESHOLD < 0:
                if angle < MAXIMUM_WALL_DISTANCE_CORRECTION_ANGLE - MAXIMUM_WALL_DISTANCE_CORRECTION_ANGLE_PRECISION:
                    angle_to_turn = self.odom_data.angle_360 - ANGLE_CORRECTION_SPEED
                    
                    self.turn_to_angle_360_system(angle_to_turn if angle_to_turn >= 0 else 358)
                elif angle > MAXIMUM_WALL_DISTANCE_CORRECTION_ANGLE + MAXIMUM_WALL_DISTANCE_CORRECTION_ANGLE_PRECISION:
                    angle_to_turn = self.odom_data.angle_360 + ANGLE_CORRECTION_SPEED
                    
                    self.turn_to_angle_360_system(angle_to_turn if angle_to_turn < 360 else 1)
                
            else:
                if angle > -MAXIMUM_WALL_DISTANCE_CORRECTION_ANGLE + MAXIMUM_WALL_DISTANCE_CORRECTION_ANGLE_PRECISION:
                    angle_to_turn = self.odom_data.angle_360 + ANGLE_CORRECTION_SPEED
                    
                    self.turn_to_angle_360_system(angle_to_turn if angle_to_turn < 360 else 1)

                elif angle < -MAXIMUM_WALL_DISTANCE_CORRECTION_ANGLE - MAXIMUM_WALL_DISTANCE_CORRECTION_ANGLE_PRECISION:
                    angle_to_turn = self.odom_data.angle_360 - ANGLE_CORRECTION_SPEED
                    
                    self.turn_to_angle_360_system(angle_to_turn if angle_to_turn >= 0 else 358)


        elif abs(angle) > WALL_PARALLEL_THRESHOLD_ANGLE:
            turn_clockwise = True

            if a > 0:
                turn_clockwise = False

            if turn_clockwise:
                angle_to_turn = self.odom_data.angle_360 - ANGLE_CORRECTION_SPEED
                self.turn_to_angle_360_system(angle_to_turn if angle_to_turn >= 0 else 358)
            else:
                angle_to_turn = self.odom_data.angle_360 + ANGLE_CORRECTION_SPEED
                self.turn_to_angle_360_system(angle_to_turn if angle_to_turn < 360 else 1)

    def is_convex_corner(self):
        data = list(self.lidar_data.ranges)[100:80:-1]

        for i in range(len(data)):
            if i != 0:
                if data[i-1] - data[i] < -CONVEX_CORNER_THRESHOLD:
                    return True
        
        return False

    def do_turn(self):
        angle_to_turn = self.odom_data.angle_360 + 90
        angle_to_turn_corrected = angle_to_turn if angle_to_turn < 360 else angle_to_turn - 360

        self.turn_to_angle_360_system(angle_to_turn_corrected, ang_speed=CONVEX_CORNER_CORRECTION_SPEED)

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
        


    def main_loop(self):
        while not self.ctrl_c:
            if self.odom_data.initial_data_loaded and self.lidar_data.initial_data_loaded:

                if not len(self.lidar_data.get_space_array()) == 0:
                    self.turn_to_angle_360_system(self.choose_angle())
                else:
                    angle_to_turn = self.odom_data.angle_360 + 180
                    self.turn_to_angle_360_system(angle_to_turn if angle_to_turn < 360 else angle_to_turn - 360)

                self.visited_points.append((self.odom_data.posx, self.odom_data.posy))

                while self.can_move_forward():
                    self.publish_velocity.publish_velocity(LINEAR_VELOCITY, 0)
                    if self.ctrl_c:
                        break
                
                if self.lidar_data.ranges[180] > 0.7 and self.lidar_data.ranges[130] > 0.7 and self.lidar_data.ranges[230] > 0.7:
                    self.publish_velocity.publish_velocity(-LINEAR_VELOCITY, 0)
                    time.sleep(REVERSE_TIME)
                elif self.lidar_data.ranges[180] > 0.4 and self.lidar_data.ranges[130] > 0.4 and self.lidar_data.ranges[230] > 0.4:
                    self.publish_velocity.publish_velocity(-LINEAR_VELOCITY, 0)
                    time.sleep(0.5*REVERSE_TIME)
                else:
                    self.publish_velocity.publish_velocity(-LINEAR_VELOCITY, 0)
                    time.sleep(0.2*REVERSE_TIME)
                self.publish_velocity.publish_velocity()

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

    # def get_mid_angle(self):
    #     greatest_space = 0
    #     current_space = 0
    #     previous_inf = False
    #     beginning_of_greatest = 0

    #     angle_before_first_obstacle = 0
    #     first_angle_set = False
    #     angle_after_last_obstacle = 0

    #     for i, range in enumerate(self.ranges):
    #         if range == inf:
    #             if not first_angle_set:
    #                 angle_before_first_obstacle += 1

    #             if not previous_inf:
    #                 previous_inf = True

    #             current_space += 1

    #         else:
    #             first_angle_set = True

    #             if current_space > greatest_space:
    #                 greatest_space = current_space
    #                 beginning_of_greatest = i - current_space
                
    #             current_space = 0
    #             previous_inf = False
        
    #     angle_after_last_obstacle = current_space

    #     if angle_before_first_obstacle + angle_after_last_obstacle == 0 or greatest_space == 0:
    #         return None
    #     elif angle_before_first_obstacle + angle_after_last_obstacle > greatest_space:
    #         angle = (angle_after_last_obstacle + angle_before_first_obstacle) / 2
    #         return 360 - angle_after_last_obstacle + angle if 360 - angle_after_last_obstacle + angle < 360 else angle_after_last_obstacle + angle
    #     else:
    #         return beginning_of_greatest + greatest_space / 2

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



if __name__ == '__main__':
    try:
        main_instance = Main()
        main_instance.main_loop()
    except rospy.ROSInterruptException:
        pass
