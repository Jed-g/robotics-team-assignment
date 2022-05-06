import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np

class Forward:

    def _init_(self):
        node_name = "maze_navigation"

        
      

        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10) # hz

        self.vel = Twist()

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        self.wall_distance = []

        self.right_side_wall_distance = []
        self.left_side_wall_distance = []

    def callback_lidar(self, lidar_data):
        left_arc = lidar_data.ranges[0:10]
        right_arc = lidar_data.ranges[-10:]
        front_arc = np.array(left_arc + right_arc)




    def shutdownhook(self):
        self.shutdown_function()
        self.ctrl_c = True    

        
        pass