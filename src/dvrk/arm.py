#  Author(s):  Anton Deguet
#  Created on: 2016-05

# (C) Copyright 2016-2023 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

"""This class presents a arm api for the da Vinci Research Kit.
Remember that for this program to work, you will need to import the
arm class, this can be done by `from dvrk.arm import arm` as well as
initialize the arm. For example, if we want to create a arm called
`r`, for arm `PSM1`, we will simply type `r = arm('PSM1')`.

For arm specific features, import the class psm or mtm (e.g. `from
dvrk.psm import psm`) and initialize your instance using `psm1 =
psm('PSM1')`.
"""

# sphinx-apidoc -F -A "Yijun Hu" -o doc src
import threading
import math

import crtk
import rclpy
import PyKDL
import std_msgs.msg

class arm(object):
    """Simple arm API wrapping around ROS messages
    """

    # class to contain spatial/body cf methods
    class __MeasuredServoCf:
        def __init__(self, ros_node, ros_sub_namespace, expected_interval):
            self.__crtk_utils = crtk.utils(self, ros_node, expected_interval)
            self.__crtk_utils.add_measured_cf(ros_sub_namespace)
            self.__crtk_utils.add_servo_cf(ros_sub_namespace)
            self.__crtk_utils.add_jacobian(ros_sub_namespace)

    # local kinematics
    class __Local:
        def __init__(self, ros_node, ros_sub_namespace, expected_interval):
            self.__crtk_utils = crtk.utils(self, ros_node, expected_interval)
            self.__crtk_utils.add_measured_cp(ros_sub_namespace)
            self.__crtk_utils.add_setpoint_cp(ros_sub_namespace)

    # initialize the arm
    def __init__(self, arm_name,
                 ros_node,
                 expected_interval = 0.01):
        # base class constructor in separate method so it can be called in derived classes
        self.__init_arm(arm_name, ros_node, expected_interval)


    def __init_arm(self, arm_name, ros_node, expected_interval):
        """Constructor.  This initializes a few data members.It
        requires a arm name, this will be used to find the ROS
        topics for the arm being controlled.  For example if the
        user wants `PSM1`, the ROS topics will be from the namespace
        `PSM1`"""
        # data members
        self.__arm_name = arm_name
        self.__ros_node = ros_node

        # crtk features
        self.__crtk_utils = crtk.utils(self, self.__ros_node, expected_interval)

        # add crtk features that we need and are supported by the dVRK
        self.__crtk_utils.add_operating_state()
        self.__crtk_utils.add_setpoint_js()
        self.__crtk_utils.add_setpoint_cp()
        self.__crtk_utils.add_measured_js()
        self.__crtk_utils.add_measured_cp()
        self.__crtk_utils.add_measured_cv()
        self.__crtk_utils.add_hold()
        self.__crtk_utils.add_free()
        self.__crtk_utils.add_servo_jp()
        self.__crtk_utils.add_servo_jr()
        self.__crtk_utils.add_servo_cp()
        self.__crtk_utils.add_servo_jf()
        self.__crtk_utils.add_move_jp()
        self.__crtk_utils.add_move_jr()
        self.__crtk_utils.add_move_cp()
        self.__crtk_utils.add_forward_kinematics()
        self.__crtk_utils.add_inverse_kinematics()

        self.spatial = self.__MeasuredServoCf(self.__ros_node, 'spatial/', expected_interval)
        self.body = self.__MeasuredServoCf(self.__ros_node, 'body/', expected_interval)
        self.local = self.__Local(self.__ros_node, 'local/', expected_interval)

        self.__sub_list = []
        self.__pub_list = []

        # publishers
        frame = PyKDL.Frame()
        self.__body_set_cf_orientation_absolute_pub = self.__ros_node.create_publisher(std_msgs.msg.Bool,
                                                                                       'body/set_cf_orientation_absolute',
                                                                                       10)
        self.__use_gravity_compensation_pub =  self.__ros_node.create_publisher(std_msgs.msg.Bool,
                                                                                'use_gravity_compensation',
                                                                                10)
        self.__trajectory_j_set_ratio_pub =  self.__ros_node.create_publisher(std_msgs.msg.Float64,
                                                                              'trajectory_j/set_ratio',
                                                                              10)
        self.__pub_list = [self.__body_set_cf_orientation_absolute_pub,
                           self.__use_gravity_compensation_pub,
                           self.__trajectory_j_set_ratio_pub]
        # subscribers
        self.__trajectory_j_ratio_sub = self.__ros_node.create_subscription(std_msgs.msg.Float64,
                                                                            'trajectory_j/ratio',
                                                                            self.__trajectory_j_ratio_cb,
                                                                            10)
        self.__sub_list = [self.__trajectory_j_ratio_sub]


    def name(self):
        return self.__arm_name


    def namespace(self):
        return self.__full_ros_namespace


    def body_set_cf_orientation_absolute(self, absolute):
        """Apply body wrench using body orientation (relative/False) or reference frame (absolute/True)"""
        m = std_msgs.msg.Bool()
        m.data = absolute
        self.__body_set_cf_orientation_absolute_pub.publish(m)


    def use_gravity_compensation(self, gravity_compensation):
        "Turn on/off gravity compensation in cartesian effort mode"
        g = std_msgs.msg.Bool()
        g.data = gravity_compensation
        self.__use_gravity_compensation_pub.publish(g)


    def trajectory_j_set_ratio(self, ratio):
        """Set ratio applied to max velocities and accelerations used for
        joint trajectory generation.  Value should be in range ]0,
        1]
        """
        r = std_msgs.msg.Float64()
        r.data = ratio
        self.__trajectory_j_set_ratio_pub.publish(r)

    def __trajectory_j_ratio_cb(self, msg):
        self.__trajectory_j_ratio = msg.data

    def trajectory_j_ratio(self):
        return self.__trajectory_j_ratio


    def unregister(self, verbose = False):
        for sub in self.__sub_list:
            sub.unregister()
        if verbose:
            print('Unregistered {} subs for {}'.format(self.__sub_list.__len__(), self.__arm_name))

        for pub in self.__pub_list:
            pub.unregister()
        if verbose:
            print('Unregistered {} pubs for {}'.format(self.__pub_list.__len__(), self.__arm_name))
