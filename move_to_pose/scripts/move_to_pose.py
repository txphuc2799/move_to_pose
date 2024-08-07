#!/usr/bin/env python3
"""
Reference document: - Robotics Vison and Control Fundamental - Chaper 2
                    - https://github.com/felixchenfy/ros_turtlebot_control
                    - https://github.com/OkDoky/docking_module/blob/master/docking_planner/src/transform.py
                    - https://github.com/AtsushiSakai/PythonRobotics/blob/master/Control/move_to_pose/move_to_pose.py
"""
import rospy
import math
import numpy as np
import tf
import actionlib

from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from move_to_pose_msg.msg import MoveToPoseAction, MoveToPoseFeedback, MoveToPoseResult, MoveToPoseGoal
from utility import Utility
from pid_controller import PIDController


class MoveToPose(Utility):

    def __init__(self):

        super().__init__()

        # Init params:
        self.cmd_vel_topic_        = rospy.get_param("~" + "cmd_vel_topic", "cmd_vel")
        self.target_frame_         = rospy.get_param("~" + "target_frame", "tag_frame")
        self.base_frame_           = rospy.get_param("~" + "base_frame", "base_footprint")
        self.robot_radius_         = rospy.get_param("~" + "robot_radius", 0.1993)
        self.dock_displacement_    = rospy.get_param("~" + "dock_displacement", 0.1)
        self.stop_distance_        = rospy.get_param("~" + "stop_distance", 0.05)
        self.max_linear_vel_       = rospy.get_param("~" + "max_linear_vel", 0.05)
        self.max_angular_vel_      = rospy.get_param("~" + "max_angular_vel", 0.05)
        self.limited_linear_vel_   = rospy.get_param("~" + "limited_linear_vel", 0.2)
        self.limited_angular_vel_  = rospy.get_param("~" + "limited_angular_vel", 0.15)
        self.xy_tolerance_         = rospy.get_param("~" + "xy_tolerance", 0.015)
        self.yaw_tolerance_        = rospy.get_param("~" + "yaw_tolerance", 0.03)
        self.control_period_       = rospy.get_param("~" + "control_period", 0.01)
        self.controller_frequency_ = rospy.get_param("~" + "controller_frequency", 20)
        self.p_rho_                = rospy.get_param("~" + "p_rho", 0.5)
        self.p_alpha_              = rospy.get_param("~" + "p_alpha", 1.0)
        self.p_beta_               = rospy.get_param("~" + "p_beta", -0.2)
        self.angular_pid_          = rospy.get_param("~" + "angular_pid", [0.1, 1.0, 0.001, 0.2])
        self.debug_                = rospy.get_param("~" + "debug", True)
        self.is_reached_goal_      = False
        self.reached_xy_tolerance_ = False
        self.is_cancel             = False
        self.is_pause              = False

        # Create PID controller:
        self.pid_angular_ = PIDController(self.angular_pid_[0], self.angular_pid_[1], self.angular_pid_[2], self.angular_pid_[3]) 

        # Create action server:
        self.as_ = actionlib.SimpleActionServer("move_to_pose", MoveToPoseAction,
                                                execute_cb=self.executeCB, auto_start=False)
        self.as_.start()

        self.feedback_ = MoveToPoseFeedback()
        self.result_   = MoveToPoseResult()
    
        # Publisher:
        self.pub_cmd_vel_ = rospy.Publisher(self.cmd_vel_topic_, Twist, queue_size=5)

        # Subscribers:
        rospy.Subscriber("CANCEL_AMR", Bool, self.cancelCB)
        rospy.Subscriber("PAUSE_AMR", Bool, self.pauseCB)
    

    def showParams(self):
        print("***********************")
        print("cmd_vel_topic        = ", self.cmd_vel_topic_)
        print("base_frame           = ", self.base_frame_)
        print("target_frame         = ", self.target_frame_)
        print("robot_radius         = ", self.robot_radius_)
        print("dock_displacement    = ", self.dock_displacement_)
        print("max_linear_vel       = ", self.max_linear_vel_)
        print("max_angular_vel      = ", self.max_angular_vel_)
        print("xy_tolerance         = ", self.xy_tolerance_)
        print("yaw_tolerance        = ", self.yaw_tolerance_)
        print("control_period       = ", self.control_period_)
        print("controller_frequency = ", self.controller_frequency_)
        print("p_rho                = ", self.p_rho_)
        print("p_alpha              = ", self.p_alpha_)
        print("p_beta               = ", self.p_beta_)
        print("pid_angular          = ", self.angular_pid_)
        print("debug                = ", self.debug_)
        print("***********************")
    

    def reset(self):
        self.is_cancel = False
        self.is_pause  = False
        self.is_reached_goal_ = False
        self.reached_xy_tolerance_ = False

    
    def pauseCB(self, msg:Bool):
        self.is_pause = True if msg.data else False
        
    
    def cancelCB(self, msg:Bool):
        self.is_cancel = True if msg.data else False
    

    def pubCmdVel(self, v=0.0, w=0.0):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w

        msg.linear.x = self.clamp(v, -self.limited_linear_vel_, self.limited_linear_vel_)
        msg.angular.z = self.clamp(w, -self.limited_angular_vel_, self.limited_angular_vel_)

        self.pub_cmd_vel_.publish(msg)
    

    def getRotationAngle(self, p1, p2):
        v = (p2[0]-p1[0], p2[1]-p1[1])

        try:
            # Calculate the length of the vector
            norm_v = math.sqrt(v[0]**2 + v[1]**2)
        except ZeroDivisionError:
            print("Error: Division by zero occurred.")
        
        # Convert to unit vector
        unit_v = (v[0]/norm_v, v[1]/norm_v)

        # Unit vector along the x-axis
        x_unit = (1, 0)

        # Calculate the dot product of two vectors
        dot_product = x_unit[0]*unit_v[0] + x_unit[1]*unit_v[1]

        # Calculate the angle
        if dot_product == 0:
            angle = 90
        else:
            angle = math.degrees(math.acos(dot_product))

        return angle
    
    
    def calStartAndGoal(self, start_frame_name:str, goal_frame_name:str):
        """
        start pose: s_x, s_y, s_yaw
        goal pose: g_x, g_y, g_yaw
        """
        start_pose = self.get_2D_pose(start_frame_name)
        goal_pose  = self.get_2D_pose(goal_frame_name)

        if (start_pose is None or goal_pose is None):
            rospy.logerr(f"Can not transfrom {start_frame_name} or {goal_frame_name}!")
            return None, None, None, None, None, None

        point_origin = Point(x = 0.0, y = 0.0, z = 0.0)
        point        = Point(x = (self.robot_radius_ + self.dock_displacement_), y = 0.0, z = 0.0)

        x = goal_pose.transform.translation.x
        y = goal_pose.transform.translation.y
        z = goal_pose.transform.translation.z

        dock_translation = (x, y, z)
        dock_quaternion  = goal_pose.transform.rotation

        dock_matrix = np.dot(tf.transformations.translation_matrix(dock_translation),
                             tf.transformations.quaternion_matrix([dock_quaternion.x, dock_quaternion.y,
                                                                   dock_quaternion.z, dock_quaternion.w])) 
        
        origin_point = np.array([point_origin.x, point_origin.y, point_origin.z, 1])
        origin_new   = np.dot(dock_matrix, origin_point)

        v         = np.array([point.x, point.y, point.z, 1])
        v_new     = np.dot(dock_matrix, v)
        point_new = Point(x = v_new[0], y = v_new[1], z = v_new[2])

        s_x = start_pose.transform.translation.x
        s_y = start_pose.transform.translation.y
        rotation = euler_from_quaternion([start_pose.transform.rotation.x,
                                          start_pose.transform.rotation.y,
                                          start_pose.transform.rotation.z,
                                          start_pose.transform.rotation.w])
        s_yaw = rotation[2]

        g_x   = point_new.x
        g_y   = point_new.y

        goal_direction = self.pi2pi(math.atan2(g_y - s_y, g_x - s_x) - s_yaw)

        if abs(goal_direction) > math.pi/2:
            is_target_behind_robot_ = True
        else:
            is_target_behind_robot_ = False
        
        if is_target_behind_robot_:
            goal_rotation = euler_from_quaternion([goal_pose.transform.rotation.x,
                                                   goal_pose.transform.rotation.y,
                                                   goal_pose.transform.rotation.z,
                                                   goal_pose.transform.rotation.w])
            g_yaw = goal_rotation[2]

        else:
            g_yaw = np.deg2rad(self.getRotationAngle(v_new,origin_new))

        return s_x, s_y, s_yaw, g_x, g_y, g_yaw
    

    def isCloseToGoal(self, xy=None, s_yaw=None, g_yaw=None):

        b1 = True if xy is None else xy <= self.xy_tolerance_
        b2 = True if s_yaw is None and g_yaw is None else \
             abs(self.pi2pi(s_yaw - g_yaw)) <= self.yaw_tolerance_

        return b1 and b2


    def controlToPose(self):
        """
        Control robot to the target pose
        """
        loop_controller = rospy.Rate(self.controller_frequency_)

        self.reset()

        while not rospy.is_shutdown():
            
            if not self.is_reached_goal_:
                s_x, s_y, s_yaw, g_x, g_y, g_yaw = self.calStartAndGoal(self.base_frame_, self.target_frame_)

                if s_x is None:
                    return False

                delta_x   = g_x - s_x
                delta_y   = g_y - s_y
                delta_yaw = g_yaw - s_yaw 

                rho = math.hypot(delta_x, delta_y)
                alpha = self.pi2pi(math.atan2(delta_y, delta_x) - s_yaw)
                beta = self.pi2pi(delta_yaw) - alpha

                # Check moving direction
                sign = 1
                # Check whether the goal is behind robot
                if (abs(alpha) > math.pi/2):    # The goal is behind robot
                    alpha = self.pi2pi(math.pi - alpha)
                    beta = self.pi2pi(math.pi - beta)
                    sign = -1
                
                # PID Control
                val_rho = self.p_rho_ * rho
                val_alpha = self.p_alpha_ * alpha
                val_beta = self.p_beta_ * beta

                if not self.reached_xy_tolerance_:
                    if self.isCloseToGoal(xy=rho):
                        print("Reached xy!")
                        self.reached_xy_tolerance_ = True
                
                if self.reached_xy_tolerance_:
                    sign = 1  # Default to be forward.
                    val_rho = 0  # No linear motion.
                    val_alpha = 0  # No rotating towards target point.
                    # Rotate towards target orientation.
                    val_beta = self.pi2pi(delta_yaw)
                
                # Get desired speed
                v = sign * val_rho
                w = sign * (val_alpha + val_beta)

                if self.reached_xy_tolerance_:
                    if self.isCloseToGoal(s_yaw=s_yaw, g_yaw=g_yaw):
                        self.is_reached_goal_ = True
            else:
                dock_pose = self.get_2D_pose(self.target_frame_, self.base_frame_)

                # if dock_pose is None:
                #     return False
                
                if dock_pose is None:
                    print("Reached goal!")
                    self.pubCmdVel()
                    return True
                
                x = dock_pose.transform.translation.x
                y = dock_pose.transform.translation.y
                rotation = euler_from_quaternion([dock_pose.transform.rotation.x,
                                                  dock_pose.transform.rotation.y,
                                                  dock_pose.transform.rotation.z,
                                                  dock_pose.transform.rotation.w])
                
                yaw = rotation[2] if x < 0 else self.flip_yaw(rotation[2])

                sign = 1 if x > 0 else -1

                val_rho = (math.hypot(x, y) - self.robot_radius_) * self.p_rho_

                v = sign * val_rho
                w = self.pid_angular_.compute(yaw)

                # if (val_rho / self.p_rho_) < self.stop_distance_:
                #     print("Reached goal!")
                #     self.pubCmdVel()
                #     return True
                
            # Threshold on velocity
            v = min(abs(v), self.max_linear_vel_) * \
                (1 if v > 0 else -1)  # limit v
            w = min(abs(w), self.max_angular_vel_) * \
                (1 if w > 0 else -1)  # limit w
            
            # Publish speed
            self.pubCmdVel(v, w)

            loop_controller.sleep()

    
    def executeCB(self, goal:MoveToPoseGoal):
        print("Start move_to_pose controller.")

        if self.debug_:
            self.showParams()

        if goal.start_controller:
            self.result_.is_success = self.controlToPose()

            if self.result_.is_success:
                self.as_.set_succeeded(self.result_)
            else:
                self.as_.set_aborted(self.result_)


if __name__ == "__main__":
    rospy.init_node("move_to_pose")
    try:
        move_to_pose = MoveToPose()
        rospy.loginfo("Initialized move_to_pose controller.")
        rospy.spin()

    except rospy.ROSInterruptException:
        pass