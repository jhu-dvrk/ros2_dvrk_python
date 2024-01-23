#!/usr/bin/env python

# Author: Anton Deguet
# Date: 2024-01-09

# (C) Copyright 2024 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

import crtk
import numpy
import math

import sys

from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QTableWidget, QTableWidgetItem, QPushButton
from PyQt5.QtCore import QTimer

class suj(object):
    """Simple arm API wrapping around ROS messages
    """

    class __Arm:

        class __Voltages:
            def __init__(self, ral, expected_interval):
                self.__crtk_utils = crtk.utils(self, ral, expected_interval)
                self.__crtk_utils.add_measured_js()

        def __init__(self, ral, expected_interval):
            self.__ral = ral
            self.primary_voltage = self.__Voltages(ral.create_child('/primary_voltage'), expected_interval)
            self.secondary_voltage = self.__Voltages(ral.create_child('/secondary_voltage'), expected_interval)

        def ral(self):
            return self.__ral

    # initialize the all SUJ arms
    def __init__(self, ral, expected_interval = 1.0):
        """Constructor.  This initializes a few data members and creates
        instances of classes for each SUJ arm."""
        self.__ral = ral.create_child('SUJ')
        self.__crtk_utils = crtk.utils(self, ral, expected_interval)
        for arm in ('ECM', 'PSM1', 'PSM2', 'PSM3'):
            setattr(self, arm, self.__Arm(self.__ral.create_child(arm), expected_interval))

    def ral(self):
        return self.__ral

class voltages(object):

    def __init__(self, name, sub_name, measured_jp, nb_joints):
        self.name = name
        self.sub_name = sub_name
        self.measured_jp = measured_jp
        self.nb_joints = nb_joints
        self.minimum = numpy.full([nb_joints], math.inf, dtype = float)
        self.maximum = numpy.full([nb_joints], -math.inf, dtype = float)

class main_widget(QWidget):

    def __init__(self, parent = None):
        super(main_widget, self).__init__(parent)
        self.ral = crtk.ral('dvrk_calibrate_suj')
        self.ral.spin()

        self.SUJ = suj(self.ral)
        self.SUJ.ral().check_connections()

        # setup CRTK clients
        self.all_voltages = []
        self.all_voltages.append(voltages("ECM", "primary",   self.SUJ.ECM.primary_voltage.measured_jp, 4))
        self.all_voltages.append(voltages("ECM", "secondary", self.SUJ.ECM.secondary_voltage.measured_jp, 4))
        self.all_voltages.append(voltages("PSM1", "primary",   self.SUJ.PSM1.primary_voltage.measured_jp, 4))
        self.all_voltages.append(voltages("PSM1", "secondary", self.SUJ.PSM1.secondary_voltage.measured_jp, 4))
        self.all_voltages.append(voltages("PSM2", "primary",   self.SUJ.PSM2.primary_voltage.measured_jp, 4))
        self.all_voltages.append(voltages("PSM2", "secondary", self.SUJ.PSM2.secondary_voltage.measured_jp, 4))
        self.all_voltages.append(voltages("PSM3", "primary",   self.SUJ.PSM3.primary_voltage.measured_jp, 5))
        self.all_voltages.append(voltages("PSM3", "secondary", self.SUJ.PSM3.secondary_voltage.measured_jp, 5))

        # GUI
        self.setWindowTitle('dVRK SUJ Calibration')
        self.mainLayout = QVBoxLayout()
        self.setLayout(self.mainLayout)

        self.table = QTableWidget(8, 7)
        self.mainLayout.addWidget(self.table)
        counter = 0
        for v in self.all_voltages:
            newItem = QTableWidgetItem(v.name)
            self.table.setItem(counter, 0, newItem)
            newItem = QTableWidgetItem(v.sub_name)
            self.table.setItem(counter, 1, newItem)
            for i in range(v.nb_joints):
                newItem = QTableWidgetItem(f'[{v.minimum[i]}, {v.maximum[i]}]')
                self.table.setItem(counter, 2 + i, newItem)
            counter += 1

        self.saveButton = QPushButton('Save')
        self.mainLayout.addWidget(self.saveButton)
        self.saveButton.clicked.connect(self.save_cb)

        # timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.timer_cb)
        self.timer.start(20) # in ms

    def timer_cb(self):
        counter = 0
        for v in self.all_voltages:
            jp = v.measured_jp()
            v.minimum = numpy.minimum(v.minimum, jp)
            v.maximum = numpy.maximum(v.maximum, jp)
            for i in range(v.nb_joints):
                item = self.table.item(counter, 2 + i)
                item.setText(f'[{v.minimum[i]}, {v.maximum[i]}] -> {v.maximum[i] - v.minimum[i]}')
            counter += 1
        self.table.resizeColumnsToContents()


    def save_cb(self):
        print('argh')

# GUI
app = QApplication([])
widget = main_widget()
widget.show()

app.exec()
widget.timer.stop()

ral.shutdown()
