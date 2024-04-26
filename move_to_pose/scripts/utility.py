import rospy
import math
import tf2_ros

from typing import Tuple

Pose2D = Tuple[float, float, float]

class Utility():

    def __init__(self):
        # Create_subscriber to tf broadcaster
        self.__tfBuffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.__tf_listener = tf2_ros.TransformListener(self.__tfBuffer)

    
    def clamp(self, input, min, max):
        if input > max:
            return max
        elif input < min:
            return min
        else:
            return input


    def pi2pi(self, theta):
        return (theta + math.pi) % (2 * math.pi) - math.pi
    

    def flip_yaw(self, yaw: float) -> float:
        """
        Flip yaw angle by 180 degree, input yaw range should be within
        [-pi, pi] radian. Else use set_angle() fn to fix the convention.
        Output will also be within the same range of [-pi, pi] radian.
        """
        if yaw >= 0:
            return yaw - math.pi
        else:
            return yaw + math.pi
    

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