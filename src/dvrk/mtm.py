#  Author(s):  Anton Deguet
#  Created on: 2016-05

#   (C) Copyright 2016-2022 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

from dvrk.arm import *

import geometry_msgs.msg

class mtm(arm):
    """Simple robot API wrapping around ROS messages
    """

    # class to contain gripper methods
    class __Gripper:
        def __init__(self, ros_node, ros_sub_namespace, expected_interval):
            self.__crtk_utils = crtk.utils(self, ros_node, expected_interval)
            self.__crtk_utils.add_measured_js(ros_sub_namespace)

    # initialize the robot
    def __init__(self, arm_name, ros_node = None,
                 expected_interval = 0.01):
        # first call base class constructor
        self._arm__init_arm(arm_name, ros_node, expected_interval)
        self.gripper = self.__Gripper(self._arm__ros_node, 'gripper/', expected_interval)

        # publishers
        self.__lock_orientation_publisher = self._arm__ros_node.create_publisher(geometry_msgs.msg.Quaternion,
                                                                                 'lock_orientation',
                                                                                 10)
        self.__unlock_orientation_publisher = self._arm__ros_node.create_publisher(std_msgs.msg.Empty,
                                                                                   'unlock_orientation',
                                                                                   10)

        self._arm__pub_list.extend([self.__lock_orientation_publisher,
                                    self.__unlock_orientation_publisher])

    def lock_orientation_as_is(self):
        "Lock orientation based on current orientation"
        current = self.setpoint_cp()
        self.lock_orientation(current.M)


    def lock_orientation(self, orientation):
        """Lock orientation, expects a PyKDL rotation matrix (PyKDL.Rotation)"""
        q = geometry_msgs.msg.Quaternion()
        q.x, q.y, q.z, q.w = orientation.GetQuaternion()
        self.__lock_orientation_publisher.publish(q);


    def unlock_orientation(self):
        "Unlock orientation"
        e = std_msgs.msg.Empty()
        self.__unlock_orientation_publisher.publish(e);
