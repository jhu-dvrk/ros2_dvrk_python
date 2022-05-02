#!/usr/bin/env python

# Authors: Nick Eusman, Anton Deguet
# Date: 2015-09-24
# Copyright JHU 2015-2022

import threading
import time
import math
import sys
import datetime
import numpy
import argparse
import os.path
import xml.etree.ElementTree as ET

import crtk
import rclpy

if sys.version_info.major < 3:
    input = raw_input

def slope(x, y):
    a = []
    for i in range(len(x)):
        a.append(x[i] * y[i])
    sum_a = sum(a)
    final_a = (sum_a * len(x))
    final_b = (sum(x) * sum(y))

    c_squared = []
    for i in x:
        c_squared.append(i**2)

    c_sum = sum(c_squared)
    final_c = (c_sum * len(x))
    final_d = (sum(x)**2)

    result = (final_a - final_b) / (final_c - final_d)
    return result

class arm:
    # simplified class for pots or actuators
    class __pot_or_actuator:
        def __init__(self, ros_node, ros_sub_namespace,
                     expected_interval,
                     operating_state_instance):
            self.__crtk_utils = crtk.utils(self, ros_node,
                                           expected_interval,
                                           operating_state_instance)
            self.__crtk_utils.add_measured_js(ros_sub_namespace)

    def __init__(self, ros_node, expected_interval):
        # populate this class with all the ROS topics we need
        self.crtk_utils = crtk.utils(self, ros_node,
                                     expected_interval)
        self.crtk_utils.add_operating_state()
        self.crtk_utils.add_move_jp()
        self.pot = self.__pot_or_actuator(ros_node,
                                          'io/pot/',
                                          expected_interval,
                                          operating_state_instance = self)
        self.actuator = self.__pot_or_actuator(ros_node,
                                               'io/actuator/',
                                               expected_interval,
                                               operating_state_instance = self)

class potentiometer_calibration:

    def __init__(self, arm_name, ros_node, expected_interval):
        self._arm_name = arm_name
        self._serial_number = ""
        self._arm = arm(ros_node, expected_interval)

    def run(self, calibrate, filename):
        nb_joint_positions = 20 # number of positions between limits
        nb_samples_per_position = 500 # number of values collected at each position
        total_samples = nb_joint_positions * nb_samples_per_position
        samples_so_far = 0

        sleep_time_after_motion = 0.5 # time after motion from position to position to allow potentiometers to stabilize
        sleep_time_between_samples = 0.01 # time between two samples read (potentiometers)

        encoders = []
        potentiometers = []
        range_of_motion_joint = []

        average_encoder = [] # all encoder values collected at a sample position used for averaging the values of that position
        average_potentiometer = [] # all potentiometer values collected at a sample position used for averaging the values of that position
        d2r = math.pi / 180.0 # degrees to radians
        r2d = 180.0 / math.pi # radians to degrees

        slopes = []
        offsets = []
        average_offsets = []

        # Looking in XML assuming following tree structure
        # config > Robot> Actuator > AnalogIn > VoltsToPosSI > Scale = ____   or   Offset = ____
        xmlVoltsToPosSI = {}

        # Make sure file exists
        if not os.path.exists(filename):
            sys.exit("Config file \"%s\" not found" % filename)

        tree = ET.parse(filename)
        root = tree.getroot()

        # Find Robot in config file and make sure name matches
        xpath_search_results = root.findall("./Robot")
        if len(xpath_search_results) == 1:
            xmlRobot = xpath_search_results[0]
        else:
            sys.exit("Can't find \"Robot\" in configuration file")

        if xmlRobot.get("Name") == self._arm_name:
            self._serial_number = xmlRobot.get("SN")
            print("Successfully found robot \"%s\", serial number %s in XML file" % (self._arm_name, self._serial_number))
            robotFound = True
        else:
            sys.exit("Found robot \"%s\" while looking for \"%s\", make sure you're using the correct configuration file!" % (xmlRobot.get("Name"), self._arm_name))

        # Look for all actuators/VoltsToPosSI
        xpath_search_results = root.findall("./Robot/Actuator")
        for actuator in xpath_search_results:
            actuatorId = int(actuator.get("ActuatorID"))
            voltsToPosSI = actuator.find("./AnalogIn/VoltsToPosSI")
            xmlVoltsToPosSI[actuatorId] = voltsToPosSI

        # set joint limits and number of axis based on arm type, using robot name
        if ("").join(list(self._arm_name)[:-1]) == "PSM": #checks to see if the robot being tested is a PSM
            arm_type = "PSM"
            lower_joint_limits = [-60.0 * d2r, -30.0 * d2r, 0.005, -170.0 * d2r, -170.0 * d2r, -170.0 * d2r, -170.0 * d2r]
            upper_joint_limits = [ 60.0 * d2r,  30.0 * d2r, 0.235,  170.0 * d2r,  170.0 * d2r,  170.0 * d2r,  170.0 * d2r]
            nb_axis = 7
        elif self._arm_name == "MTML":
            arm_type = "MTM"
            lower_joint_limits = [-15.0 * d2r, -10.0 * d2r, -10.0 * d2r, -180.0 * d2r, -80.0 * d2r, -40.0 * d2r, -100.0 * d2r]
            upper_joint_limits = [ 35.0 * d2r,  20.0 * d2r,  10.0 * d2r,   80.0 * d2r, 160.0 * d2r,  40.0 * d2r,  100.0 * d2r]
            nb_axis = 7
        elif self._arm_name == "MTMR":
            arm_type = "MTM"
            lower_joint_limits = [-30.0 * d2r, -10.0 * d2r, -10.0 * d2r,  -80.0 * d2r, -80.0 * d2r, -40.0 * d2r, -100.0 * d2r]
            upper_joint_limits = [ 15.0 * d2r,  20.0 * d2r,  10.0 * d2r,  180.0 * d2r, 160.0 * d2r,  40.0 * d2r,  100.0 * d2r]
            nb_axis = 7
        elif self._arm_name == "ECM":
            arm_type = "ECM"
            lower_joint_limits = [-60.0 * d2r, -40.0 * d2r,  0.005, -80.0 * d2r]
            upper_joint_limits = [ 60.0 * d2r,  40.0 * d2r,  0.230,  80.0 * d2r]
            nb_axis = 4

        # resize all arrays
        for axis in range(nb_axis):
            encoders.append([])
            offsets.append([])
            potentiometers.append([])
            average_encoder.append([])
            average_offsets.append([])
            average_potentiometer.append([])
            range_of_motion_joint.append(math.fabs(upper_joint_limits[axis] - lower_joint_limits[axis]))

        # Check that everything is working
        if not self._arm.home(10):
            print("It seems the console for %s is not started or is not publishing the IO topics" % self._arm_name)
            print("Make sure you use \"rosrun dvrk_robot dvrk_console_json\" with the -i option")
            sys.exit("Start the dvrk_console_json with the proper options first")

        print("The serial number found in the XML file is: %s" % self._serial_number)
        print("Make sure the dvrk_console_json is using the same configuration file.  Serial number can be found in GUI tab \"IO\".")
        ok = input("Press `c` and [enter] to continue\n")
        if ok != "c":
            sys.exit("Quitting")

        ######## scale calibration
        now = datetime.datetime.now()
        now_string = now.strftime("%Y-%m-%d-%H:%M")

        if calibrate == "scales":

            print("Calibrating scales using encoders as reference")
            # messages
            input("To start with some initial values, you first need to \"home\" the robot.  When homed, press [enter]\n")

            if arm_type == "PSM":
                input("Since you are calibrating a PSM, make sure there is no tool inserted.  Please remove tool or calibration plate if any and press [enter]\n")
            if arm_type == "ECM":
                input("Since you are calibrating an ECM, remove the endoscope and press [enter]\n")
            input("The robot will make LARGE MOVEMENTS, please hit [enter] to continue once it is safe to proceed\n")

            for position in range(nb_joint_positions):
                # create joint goal
                joint_goal = []
                for axis in range(nb_axis):
                    joint_goal.append(lower_joint_limits[axis] + position * (range_of_motion_joint[axis] / nb_joint_positions))
                    average_encoder[axis] = []
                    average_potentiometer[axis] = []

                # move and sleep
                self._arm.move_jp(numpy.array(joint_goal)).wait()
                time.sleep(sleep_time_after_motion)

                # collect nb_samples_per_position at current position to compute average
                for sample in range(nb_samples_per_position):
                    for axis in range(nb_axis):
                        average_potentiometer[axis].append(self._arm.pot.measured_jp()[axis])
                        average_encoder[axis].append(self._arm.actuator.measured_jp()[axis])
                    time.sleep(sleep_time_between_samples)
                    samples_so_far = samples_so_far + 1
                    sys.stdout.write('\rProgress %02.1f%%' % (float(samples_so_far) / float(total_samples) * 100.0))
                    sys.stdout.flush()

                # compute averages
                for axis in range(nb_axis):
                    potentiometers[axis].append(math.fsum(average_potentiometer[axis]) / nb_samples_per_position)
                    encoders[axis].append(math.fsum(average_encoder[axis]) / nb_samples_per_position)


            # at the end, return to home position
            if arm_type == "PSM" or arm_type == "MTM":
                self._arm.move_jp(numpy.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])).wait()
            elif arm_type == "ECM":
                self._arm.move_jp(numpy.array([0.0, 0.0, 0.0, 0.0])).wait()


        ######## offset calibration
        if calibrate == "offsets":

            print("Calibrating offsets")

            # messages
            print("Please home AND power off the robot first.  Then hold/clamp your arm in zero position.")
            if arm_type == "PSM":
                print("For a PSM, you need to hold at least the last 4 joints in zero position.  If you don't have a way to constrain the first 3 joints, you can still just calibrate the last 4.  This program will ask you later if you want to save all PSM joint offsets");
            input("Press [enter] to continue\n")
            nb_samples = 2 * nb_samples_per_position
            for sample in range(nb_samples):
                for axis in range(nb_axis):
                    average_offsets[axis].append(self._arm.pot.measured_jp()[axis] * r2d)
                time.sleep(sleep_time_between_samples)
                sys.stdout.write('\rProgress %02.1f%%' % (float(sample) / float(nb_samples) * 100.0))
                sys.stdout.flush()
            for axis in range(nb_axis):
                offsets[axis] = (math.fsum(average_offsets[axis]) / (nb_samples) )

        print("")


        if calibrate == "scales":
            print("index | old scale  | new scale  | correction")
            for index in range(nb_axis):
                # find existing values
                oldScale = float(xmlVoltsToPosSI[index].get("Scale"))
                # compute new values
                correction = slope(encoders[index], potentiometers[index])
                newScale = oldScale / correction

                # display
                print(" %d    | % 04.6f | % 04.6f | % 04.6f" % (index, oldScale, newScale, correction))
                # replace values
                xmlVoltsToPosSI[index].set("Scale", str(newScale))

        if calibrate == "offsets":
            newOffsets = []
            print("index | old offset  | new offset | correction")
            for index in range(nb_axis):
                # find existing values
                oldOffset = float(xmlVoltsToPosSI[index].get("Offset"))
                # compute new values
                newOffsets.append(oldOffset - offsets[index])

                # display
                print(" %d    | % 04.6f | % 04.6f | % 04.6f " % (index, oldOffset, newOffsets[index], offsets[index]))

            if arm_type == "PSM":
                all = input("Do you want to save all joint offsets or just the last 4, press 'a' followed by [enter] to save all\n");
                if all == "a":
                    print("This program will save ALL new PSM offsets")
                    for axis in range(nb_axis):
                        # replace values
                        xmlVoltsToPosSI[axis].set("Offset", str(newOffsets[axis]))
                else:
                    print("This program will only save the last 4 PSM offsets")
                    for axis in range(3, nb_axis):
                        # replace values
                        xmlVoltsToPosSI[axis].set("Offset", str(newOffsets[axis]))
            else:
                for axis in range(nb_axis):
                    # replace values
                    xmlVoltsToPosSI[axis].set("Offset", str(newOffsets[axis]))

        save = input("To save this in new file press 'y' followed by [enter]\n")
        if save == "y":
            tree.write(filename + "-new")
            print('Results saved in %s-new. Restart your dVRK application with the new file!' % filename)
            print('To copy the new file over the existing one: cp %s-new %s' % (filename, filename))

if __name__ == '__main__':
    # ros init node so we can use default ros arguments (e.g. __ns:= for namespace)
    rclpy.init(args = sys.argv)

    # parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('-a', '--arm', type=str, required=True,
                        choices=['ECM', 'MTML', 'MTMR', 'PSM1', 'PSM2', 'PSM3'],
                        help = 'arm name corresponding to ROS topics without namespace.  Use __ns:= to specify the namespace')
    parser.add_argument('-i', '--interval', type=float, default=0.01,
                        help = 'expected interval in seconds between messages sent by the device')
    parser.add_argument('-t', '--type', type=str, required=True,
                        choices=['scales', 'offsets'],
                        help = 'use either "scales" or "offsets"')
    parser.add_argument('-c', '--config', type=str, required=True,
                        help = 'arm IO config file, i.e. something like sawRobotIO1394-xwz-12345.xml')
    args = parser.parse_args(sys.argv[1:]) # skip argv[0], script name

    node = rclpy.create_node('dvrk_arm_test', namespace = args.arm)
    calibration = potentiometer_calibration(args.arm, node, args.interval)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor_thread = threading.Thread(target = executor.spin, daemon = True)
    executor_thread.start()

    try:
        calibration.run(args.type, args.config)
    except KeyboardInterrupt:
        pass

    print('stopping ROS thread')
    rclpy.shutdown()
    executor_thread.join()
    node.destroy_node()
