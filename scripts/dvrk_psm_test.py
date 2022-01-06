#!/usr/bin/env python

# Author: Anton Deguet
# Date: 2015-02-22

# (C) Copyright 2015-2022 Johns Hopkins University (JHU), All Rights Reserved.

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

# example of application using arm.py
class example_application:

    # configuration
    def configure(self, node, expected_interval):
        print('configuring dvrk_psm_test for node %s using namespace %s' % (node.get_name(), node.get_namespace()))
        self.expected_interval = expected_interval
        self.arm = dvrk.psm(arm_name = node.get_namespace(),
                            ros_node = node,
                            expected_interval = expected_interval)

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
        goal[2] = 0.12
        self.arm.move_jp(goal).wait()

    # utility to position tool/camera deep enough before cartesian examples
    def prepare_cartesian(self):
        # make sure the tip is past the cannula and tool vertical
        goal = numpy.copy(self.arm.setpoint_jp())
        if ((self.arm.name().endswith('PSM1')) or (self.arm.name().endswith('PSM2'))
            or (self.arm.name().endswith('PSM3'))):
            print('preparing for cartesian motion')
            # set in position joint mode
            goal[0] = 0.0
            goal[1] = 0.0
            goal[2] = 0.12
            goal[3] = 0.0
            self.arm.move_jp(goal).wait()

    # goal jaw control example
    def run_jaw_move(self):
        print('starting jaw move')
        # try to open and close with the cartesian part of the arm in different modes
        print('close and open without other move command')
        input("    Press Enter to continue...")
        print('closing (1)')
        self.arm.jaw.close().wait()
        print('opening (2)')
        self.arm.jaw.open().wait()
        print('closing (3)')
        self.arm.jaw.close().wait()
        print('opening (4)')
        self.arm.jaw.open().wait()
        # try to open and close with a joint goal
        print('close and open with joint move command')
        input("    Press Enter to continue...")
        print('closing and moving up (1)')
        self.arm.jaw.close().wait(is_busy = True)
        self.arm.insert_jp(0.1).wait()
        print('opening and moving down (2)')
        self.arm.jaw.open().wait(is_busy = True)
        self.arm.insert_jp(0.15).wait()
        print('closing and moving up (3)')
        self.arm.jaw.close().wait(is_busy = True)
        self.arm.insert_jp(0.1).wait()
        print('opening and moving down (4)')
        self.arm.jaw.open().wait(is_busy = True)
        self.arm.insert_jp(0.15).wait()

        print('close and open with cartesian move command')
        input("    Press Enter to continue...")

        # try to open and close with a cartesian goal
        self.prepare_cartesian()

        initial_cartesian_position = PyKDL.Frame()
        initial_cartesian_position.p = self.arm.setpoint_cp().p
        initial_cartesian_position.M = self.arm.setpoint_cp().M
        goal = PyKDL.Frame()
        goal.p = self.arm.setpoint_cp().p
        goal.M = self.arm.setpoint_cp().M

        # motion parameters
        amplitude = 0.05 # 5 cm

        # first motion
        goal.p[0] =  initial_cartesian_position.p[0] - amplitude
        goal.p[1] =  initial_cartesian_position.p[1]
        print('closing and moving right (1)')
        self.arm.move_cp(goal).wait(is_busy = True)
        self.arm.jaw.close().wait()

        # second motion
        goal.p[0] =  initial_cartesian_position.p[0] + amplitude
        goal.p[1] =  initial_cartesian_position.p[1]
        print('opening and moving left (1)')
        self.arm.move_cp(goal).wait(is_busy = True)
        self.arm.jaw.open().wait()

        # back to starting point
        goal.p[0] =  initial_cartesian_position.p[0]
        goal.p[1] =  initial_cartesian_position.p[1]
        print('moving back (3)')
        self.arm.move_cp(goal).wait()


    # goal jaw control example
    def run_jaw_servo(self):
        print('starting jaw servo')
        # try to open and close directly, needs interpolation
        print('close and open without other servo command')
        input("    Press Enter to continue...")
        start_angle = math.radians(50.0)
        self.arm.jaw.open(angle = start_angle).wait()
        # assume we start at 30 the move +/- 30
        amplitude = math.radians(30.0)
        duration = 5  # seconds
        samples = int(duration / self.expected_interval)
        # create a new goal starting with current position
        for i in range(samples * 4):
            tic = time.time()
            goal = start_angle + amplitude * (math.cos(i * math.radians(360.0) / samples) - 1.0)
            self.arm.jaw.servo_jp(numpy.array([goal]))
            dt = time.time() - tic
            time_left = self.expected_interval - dt
            if time_left > 0:
                time.sleep(time_left)

    # main method
    def run(self):
        self.home()
        self.run_jaw_move()
        self.run_jaw_servo()
        self.run_jaw_move() # just to make sure we can transition back to trajectory mode


if __name__ == '__main__':
    # ros init node so we can use default ros arguments (e.g. __ns:= for namespace)
    rclpy.init(args = sys.argv)

    # parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('-a', '--arm', type=str, required=True,
                        choices=['PSM1', 'PSM2', 'PSM3'],
                        help = 'arm name corresponding to ROS topics without namespace.  Use __ns:= to specify the namespace')
    parser.add_argument('-i', '--interval', type=float, default=0.01,
                        help = 'expected interval in seconds between messages sent by the device')
    args = parser.parse_args(sys.argv[1:]) # skip argv[0], script name

    node = rclpy.create_node('dvrk_arm_test', namespace = args.arm)
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
