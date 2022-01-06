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
# > rosrun dvrk_python dvrk_arm_test.py <arm-name>

import argparse
import sys
import time
import threading
import rclpy
import dvrk
import math
import numpy
import PyKDL
from sensor_msgs.msg import Joy

# example of application using arm.py
class example_application:

    # configuration
    def configure(self, node, expected_interval):
        print('configuring dvrk_mtm_test for node %s using namespace %s' % (node.get_name(), node.get_namespace()))
        self.expected_interval = expected_interval
        self.arm = dvrk.mtm(arm_name = node.get_namespace(),
                            ros_node = node,
                            expected_interval = expected_interval)
        self.coag_event = threading.Event()
        node.create_subscription(Joy, '/footpedals/coag',
                                 self.coag_event_cb, 10)

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
        if data.buttons[0] == 1:
            self.coag_event.set()

    # wait for foot pedal
    def wait_for_coag(self):
        self.coag_event.clear()
        self.coag_event.wait(100000)

    # tests
    def tests(self):
        # turn on gravity compensation
        self.arm.use_gravity_compensation(True)

        print('press COAG pedal to move to the next test')

        print('arm will go limp, hold it and press coag')
        self.wait_for_coag()
        self.arm.body.servo_cf(numpy.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))

        print('keep holding arm, press coag, a force in body frame will be applied (direction depends on wrist orientation)')
        self.wait_for_coag()
        self.arm.body_set_cf_orientation_absolute(False)
        self.arm.body.servo_cf(numpy.array([0.0, 0.0, -3.0, 0.0, 0.0, 0.0]))

        print('keep holding arm, press coag, a force in world frame will be applied (fixed direction)')
        self.wait_for_coag()
        self.arm.body_set_cf_orientation_absolute(True)
        self.arm.body.servo_cf(numpy.array([0.0, 0.0, -3.0, 0.0, 0.0, 0.0]))

        print('keep holding arm, press coag, orientation will be locked')
        self.wait_for_coag()
        self.arm.lock_orientation_as_is()

        print('keep holding arm, press coag, force will be removed')
        self.wait_for_coag()
        self.arm.body.servo_cf(numpy.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))

        print('keep holding arm, press coag, orientation will be unlocked')
        self.wait_for_coag()
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

    node = rclpy.create_node('dvrk_mtm_test', namespace = args.arm)
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
