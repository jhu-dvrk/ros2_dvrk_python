#!/usr/bin/env python

# Author: Anton Deguet
# Date: 2017-07-22

# (C) Copyright 2017-2022 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

# Start a single arm using
# > rosrun dvrk_robot dvrk_console_json -j <console-file>

# To communicate with the arm using ROS topics, see the python based example dvrk_arm_test.py:
# > rosrun dvrk_python dvrk_mtm_cartesian_impedance <arm-name>

import argparse
import sys
import time
import threading
import rclpy
import dvrk
import math
import numpy
from sensor_msgs.msg import Joy
from cisst_msgs.msg import CartesianImpedanceGains

# example of application using arm.py
class example_application:

    # configuration
    def configure(self, node, expected_interval):
        print('configuring dvrk_mtm_cartesian_impedance for node %s using namespace %s' % (node.get_name(), node.get_namespace()))
        self.expected_interval = expected_interval
        self.arm = dvrk.mtm(arm_name = node.get_namespace(),
                            ros_node = node,
                            expected_interval = expected_interval)
        self.coag_event = threading.Event()
        node.create_subscription(Joy, '/footpedals/coag',
                                 self.coag_event_cb, 10)
        self.set_gains_publisher = node.create_publisher(CartesianImpedanceGains,
                                                         'set_cartesian_impedance_gains',
                                                         10)

    # homing example
    def home(self):
        print('starting enable')
        if not self.arm.enable(10):
            sys.exit('failed to enable within 10 seconds')
        print('starting home')
        if not self.arm.home(10):
            sys.exit('failed to home within 10 seconds')
        # get current joints just to set size
        print('move to starting position')
        goal = numpy.copy(self.arm.setpoint_jp())
        # go to zero position, make sure 3rd joint is past cannula
        goal.fill(0)
        self.arm.move_jp(goal).wait()

    # foot pedal callback
    def coag_event_cb(self, data):
        if (data.buttons[0] == 1):
            self.coag_event.set()

    # wait for foot pedal
    def wait_for_coag(self):
        self.coag_event.clear()
        self.coag_event.wait(600)


    # tests
    def tests(self):
        # turn on gravity compensation
        self.arm.use_gravity_compensation(True)

        gains = CartesianImpedanceGains()
        # set orientation to identity quaternions
        gains.force_orientation.w = 1.0
        gains.torque_orientation.w = 1.0

        print('press COAG pedal to move to next example')

        print('arm will be constrained in X/Y plane around the current position')
        self.wait_for_coag()
        # set gains in z direction
        gains.pos_stiff_neg.z = -200.0
        gains.pos_stiff_pos.z = -200.0
        gains.pos_damping_neg.z = -5.0
        gains.pos_damping_pos.z = -5.0
        gains.force_position.x = self.arm.measured_cp().p[0]
        gains.force_position.y = self.arm.measured_cp().p[1]
        gains.force_position.z = self.arm.measured_cp().p[2]
        self.set_gains_publisher.publish(gains)

        print('orientation will be locked')
        self.wait_for_coag()
        self.arm.lock_orientation_as_is()

        print('arm will be constrained in X/Y half plane around the current position')
        self.wait_for_coag()
        # set gains in z direction, stiffer in half positive, 0 in negative
        gains.pos_stiff_neg.z = 0.0
        gains.pos_stiff_pos.z = -500.0
        gains.pos_damping_neg.z = 0.0
        gains.pos_damping_pos.z = -15.0
        gains.force_position.x = self.arm.measured_cp().p[0]
        gains.force_position.y = self.arm.measured_cp().p[1]
        gains.force_position.z = self.arm.measured_cp().p[2]
        self.set_gains_publisher.publish(gains)

        print('an horizontal line will be created around the current position, with viscosity along the line')
        self.wait_for_coag()
        # set gains in x, z directions for the line
        gains.pos_stiff_neg.x = -200.0
        gains.pos_stiff_pos.x = -200.0
        gains.pos_damping_neg.x = -5.0
        gains.pos_damping_pos.x = -5.0
        gains.pos_stiff_neg.z = -200.0
        gains.pos_stiff_pos.z = -200.0
        gains.pos_damping_neg.z = -5.0
        gains.pos_damping_pos.z = -5.0
        # viscosity along the line
        gains.pos_damping_neg.y = -10.0
        gains.pos_damping_pos.y = -10.0
        # always start from current position to avoid jumps
        gains.force_position.x = self.arm.measured_cp().p[0]
        gains.force_position.y = self.arm.measured_cp().p[1]
        gains.force_position.z = self.arm.measured_cp().p[2]
        self.set_gains_publisher.publish(gains)

        print('a plane will be created perpendicular to the master gripper')
        self.wait_for_coag()
        # set gains in x, z directions for the line
        gains.pos_stiff_neg.x = 0.0
        gains.pos_stiff_pos.x = 0.0
        gains.pos_damping_neg.x = 0.0
        gains.pos_damping_pos.x = 0.0
        gains.pos_stiff_neg.y = 0.0
        gains.pos_stiff_pos.y = 0.0
        gains.pos_damping_neg.y = 0.0
        gains.pos_damping_pos.y = 0.0
        gains.pos_stiff_neg.z = -200.0
        gains.pos_stiff_pos.z = -200.0
        gains.pos_damping_neg.z = -5.0
        gains.pos_damping_pos.z = -5.0

        stiffOri = -0.2
        dampOri = -0.01
        gains.ori_stiff_neg.x = stiffOri
        gains.ori_stiff_pos.x = stiffOri
        gains.ori_damping_neg.x = dampOri
        gains.ori_damping_pos.x = dampOri
        gains.ori_stiff_neg.y = stiffOri
        gains.ori_stiff_pos.y = stiffOri
        gains.ori_damping_neg.y = dampOri
        gains.ori_damping_pos.y = dampOri
        gains.ori_stiff_neg.z = 0.0
        gains.ori_stiff_pos.z = 0.0
        gains.ori_damping_neg.z = 0.0
        gains.ori_damping_pos.z = 0.0

        # always start from current position to avoid jumps
        gains.force_position.x = self.arm.measured_cp().p[0]
        gains.force_position.y = self.arm.measured_cp().p[1]
        gains.force_position.z = self.arm.measured_cp().p[2]
        orientationQuaternion = self.arm.measured_cp().M.GetQuaternion()
        gains.force_orientation.x = orientationQuaternion[0]
        gains.force_orientation.y = orientationQuaternion[1]
        gains.force_orientation.z = orientationQuaternion[2]
        gains.force_orientation.w = orientationQuaternion[3]
        gains.torque_orientation.x = orientationQuaternion[0]
        gains.torque_orientation.y = orientationQuaternion[1]
        gains.torque_orientation.z = orientationQuaternion[2]
        gains.torque_orientation.w = orientationQuaternion[3]
        self.set_gains_publisher.publish(gains)
        self.arm.unlock_orientation()

        print('keep holding arm, press coag, arm will freeze in position')
        self.wait_for_coag()
        self.arm.move_jp(self.arm.measured_jp()).wait()

        print('press coag to end')
        self.wait_for_coag()


    # main method
    def run(self):
        self.home()
        self.tests()


if __name__ == '__main__':
    # ros init node so we can use default ros arguments (e.g. __ns:= for namespace)
    rclpy.init(args = sys.argv)

    # parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('-a', '--arm', type=str, required=True,
                        choices=['MTML', 'MTMR'],
                        help = 'arm name corresponding to ROS topics without namespace.  Use __ns:= to specify the namespace')
    parser.add_argument('-i', '--interval', type=float, default=0.01,
                        help = 'expected interval in seconds between messages sent by the device')
    args = parser.parse_args(sys.argv[1:]) # skip argv[0], script name

    node = rclpy.create_node('dvrk_mtm_cartesian_impedance', namespace = args.arm)
    application = example_application()
    application.configure(node, args.interval)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor_thread = threading.Thread(target = executor.spin, daemon = True)
    executor_thread.start()

    try:
        application.run()
    except KeyboardInterrupt:
        pass

    print('stopping ROS thread')
    rclpy.shutdown()
    executor_thread.join()
    node.destroy_node()
