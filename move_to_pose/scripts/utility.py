import rospy
import math
import tf2_ros

from typing import Tuple
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

Pose2D = Tuple[float, float, float]

class Utility():

    def __init__(self):
        # Create_subscriber to tf broadcaster
        self.__tfBuffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.__tf_listener = tf2_ros.TransformListener(self.__tfBuffer)

        # Publisher:
        self.pub_cmd_vel_ = rospy.Publisher("cmd_vel", Twist, queue_size=5)

        # Subscribers:
        self.sub_cancel = rospy.Subscriber("CANCEL_AMR", Bool, self.sub_cancel_cb)
        self.sub_pause  = rospy.Subscriber("PAUSE_AMR", Bool, self.sub_pause_cb)

        # Variables:
        self.is_cancel = False
        self.is_pause  = False

    
    def reset(self):
        self.is_cancel = False
        self.is_pause  = False


    def sub_pause_cb(self, msg):
        self.is_pause = True if msg.data else False
        
    
    def sub_cancel_cb(self, msg):
        self.is_cancel = True if msg.data else False
    

    def pub_cmd_vel(self, v=0.0, w=0.0):
        """
        Command the robot to move, default param is STOP!
        """
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w

        if (msg.linear.x > 0.2):
            msg.linear.x = 0.2
        elif(msg.linear.x < -0.2):
            msg.linear.x = -0.2

        if (msg.angular.x > 0.15):
            msg.angular.x = 0.15
        elif(msg.angular.x < -0.15):
            msg.angular.x = -0.15

        self.pub_cmd_vel_.publish(msg)


    def pi2pi(self, theta):
        return (theta + math.pi) % (2 * math.pi) - math.pi
    

    def get_2D_pose(self, target_link=None, base_link=None):

        if target_link is None:
            target_link = "charger_frame"
        if base_link is None:
            base_link = "odom"

        try:
            trans = self.__tfBuffer.lookup_transform(
                base_link,
                target_link,
                rospy.Time(0), rospy.Duration(1.0))

        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            rospy.logerr(f"Failed lookup: {target_link}, from {base_link}")
            return None

        return trans