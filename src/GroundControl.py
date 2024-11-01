#!/usr/bin/env python

import sys
import subprocess
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QTimer, QThread

import ROS_Node as ros_node
import GUI as gui


if __name__ == "__main__":
    # init rosTrackingReference
    ros_node.rospy.init_node("GUI_Node_py")
    # define the window
    app = QtWidgets.QApplication(sys.argv)
    WaterSamplingGroundControlStation = QtWidgets.QTabWidget()
    ui = gui.Ui_WaterSamplingGroundControlStation()
    ui.setupUi(WaterSamplingGroundControlStation)
    # define ros threads
    rosSingleDroneThread = ros_node.SingleDroneRosThread(ui)
    rosSingleDroneThread.start()
    
    # show the window
    WaterSamplingGroundControlStation.show()
    print("System Started")
    sys.exit(app.exec_())