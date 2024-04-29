#!/usr/bin/env python

from __future__ import division

import itertools
import os
from std_msgs.msg import String
from dobot_driver.dobot_handle import bot

from ament_index_python import get_resource
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, qWarning, Slot, QProcess
from python_qt_binding.QtGui import QIcon
from python_qt_binding.QtWidgets import QHeaderView, QMenu, QTreeWidgetItem, QWidget, QVBoxLayout, QSizePolicy
from rqt_py_common.message_helpers import get_message_class
from PyQt5.QtWidgets import QMessageBox
from PyQt5.QtGui import *
from PyQt5.QtCore import *
import subprocess
import rclpy
from geometry_msgs.msg import PoseStamped 
import math
import tf_transformations
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray
from dobot_msgs.msg import GripperStatus
import sys




class DobotControlPanel(QWidget):

    def __init__(self, node, plugin=None):

        super(DobotControlPanel, self).__init__()

        shell_cmd = subprocess.Popen('lsusb | grep -E "Silicon Labs CP210x UART Bridge|QinHeng Electronics" ', shell=True, stdout=subprocess.PIPE)
        is_connected = shell_cmd.stdout.read().decode('utf-8')
        if not is_connected:
            sys.exit("Dobot is disconnected! Check if the USB cable and power adapter are plugged in.")


        self._node = node
        self._logText = ''
        self.process  = QProcess(self)
        self._plugin = plugin

        _, package_path = get_resource('packages', 'dobot_control_panel')
        ui_file = os.path.join(package_path, 'share', 'dobot_control_panel', 'resource', 'DobotControlPanel_layout.ui')
        loadUi(ui_file, self)


        self.subscription_TCP = self._node.create_subscription(
            Float64MultiArray,
            'dobot_pose_raw',
            self.tcp_position_callback,
            10)

        self.subscription_joints = self._node.create_subscription(
            JointState,
            'dobot_joint_states',
            self.joints_positions_callback,
            10)

        self.gripper_state_publ = self._node.create_publisher(GripperStatus, 'gripper_status_rviz', 10)

        self.dobot_current_joint_states = []

        if os.environ.get('MAGICIAN_RAIL_IN_USE') == "true":
            self.RAIL_IN_USE = True
            bot.set_sliding_rail_status(1,1)
            bot.set_point_to_point_sliding_rail_params(100, 100)
            bot.set_point_to_point_sliding_rail_params(100*2, 100*2) #BUG
            self.subscription_rail_pose = self._node.create_subscription(
                Float64,
                'dobot_rail_pose',
                self.rail_pose_callback,
                10)
        else:
            self.RAIL_IN_USE = False
            self.CurrentPositionRail.setText(str("X"))

        # Control Tab
        self.JT1Plus.pressed.connect(lambda:self.JT1_move(self.JT1Plus))
        self.JT1Plus.released.connect(self.JT_IDLE)
        self.JT1Minus.pressed.connect(lambda:self.JT1_move(self.JT1Minus))
        self.JT1Minus.released.connect(self.JT_IDLE)
        
        self.JT2Plus.pressed.connect(lambda:self.JT2_move(self.JT2Plus))
        self.JT2Plus.released.connect(self.JT_IDLE)
        self.JT2Minus.pressed.connect(lambda:self.JT2_move(self.JT2Minus))
        self.JT2Minus.released.connect(self.JT_IDLE)

        self.JT3Plus.pressed.connect(lambda:self.JT3_move(self.JT3Plus))
        self.JT3Plus.released.connect(self.JT_IDLE)
        self.JT3Minus.pressed.connect(lambda:self.JT3_move(self.JT3Minus))
        self.JT3Minus.released.connect(self.JT_IDLE)

        self.JT4Plus.pressed.connect(lambda:self.JT4_move(self.JT4Plus))
        self.JT4Plus.released.connect(self.JT_IDLE)
        self.JT4Minus.pressed.connect(lambda:self.JT4_move(self.JT4Minus))
        self.JT4Minus.released.connect(self.JT_IDLE)

        self.HomingButton.clicked.connect(self.button_clicked_HomingButton)
        self.EStopButton.clicked.connect(self.button_clicked_EStopButton)

        self.BaseFrame.setChecked(True)
        self.frame = "base"
        self.BaseFrame.toggled.connect(lambda:self.framestate(self.BaseFrame))
        self.JointFrame.toggled.connect(lambda:self.framestate(self.JointFrame))


        # Tool Tab
        self.GripperOpen.pressed.connect(self.open_gripper)
        self.GripperClose.pressed.connect(self.close_gripper)
        self.SuctionCupTurnOn.pressed.connect(self.turn_on_suction_cup)
        self.SuctionCupTurnOff.pressed.connect(self.turn_off_suction_cup)

        # Speed Tab
        self.Joint1VelSlider.valueChanged.connect(lambda:self.valuechange_joints(self.JT1Vel))
        self.Joint2VelSlider.valueChanged.connect(lambda:self.valuechange_joints(self.JT2Vel))
        self.Joint3VelSlider.valueChanged.connect(lambda:self.valuechange_joints(self.JT3Vel))
        self.Joint4VelSlider.valueChanged.connect(lambda:self.valuechange_joints(self.JT4Vel))

        self.XVelSlider.valueChanged.connect(lambda:self.valuechange_cartesian(self.XVel))
        self.YVelSlider.valueChanged.connect(lambda:self.valuechange_cartesian(self.YVel))
        self.ZVelSlider.valueChanged.connect(lambda:self.valuechange_cartesian(self.ZVel))
        self.RVelSlider.valueChanged.connect(lambda:self.valuechange_cartesian(self.RVel))

        # -----------------------------------------------------------------------------

        self.Joint1VelSlider.sliderReleased.connect(lambda:self.change_vel_joints(self.Joint1VelSlider))
        self.Joint2VelSlider.sliderReleased.connect(lambda:self.change_vel_joints(self.Joint2VelSlider))
        self.Joint3VelSlider.sliderReleased.connect(lambda:self.change_vel_joints(self.Joint3VelSlider))
        self.Joint4VelSlider.sliderReleased.connect(lambda:self.change_vel_joints(self.Joint4VelSlider))

        self.XVelSlider.sliderReleased.connect(lambda:self.change_vel_cartesian(self.XVelSlider))
        self.YVelSlider.sliderReleased.connect(lambda:self.change_vel_cartesian(self.YVelSlider))
        self.ZVelSlider.sliderReleased.connect(lambda:self.change_vel_cartesian(self.ZVelSlider))
        self.RVelSlider.sliderReleased.connect(lambda:self.change_vel_cartesian(self.RVelSlider))

        # -----------------------------------------------------------------------------

        self.RailSVSlider.sliderReleased.connect(self.sliding_rail_SV_slider)
        self.RailSVSlider.valueChanged.connect(self.sliding_rail_current_pose)

        self.RailVelocitySlider.sliderReleased.connect(self.sliding_rail_params_slider)
        self.RailVelocitySlider.valueChanged.connect(self.sliding_rail_vel_display)

        self.RailAccelerationSlider.sliderReleased.connect(self.sliding_rail_params_slider)
        self.RailAccelerationSlider.valueChanged.connect(self.sliding_rail_acc_display)

        self.StopRailButton.clicked.connect(self.button_clicked_StopRail)




        # initial setup
        bot.set_jog_common_params(100, 1) # (vel_ratio, acc_ratio)
        bot.set_jog_joint_params([100, 100, 100, 100], [100, 100, 100, 100])
        bot.set_jog_coordinate_params([100, 100, 100, 100], [100, 100, 100, 100])




    def tcp_position_callback(self, msg):
        self.XPoseLCD.setText(str(round((msg.data[0])*1000, 3)))
        self.YPoseLCD.setText(str(round((msg.data[1])*1000, 3)))
        self.ZPoseLCD.setText(str(round((msg.data[2])*1000, 3)))
        self.RPoseLCD.setText(str(round((msg.data[3]), 3)))


    def joints_positions_callback(self, msg):
        self.dobot_current_joint_states = [math.degrees(msg.position[0]), math.degrees(msg.position[1]), math.degrees(msg.position[2]), math.degrees(msg.position[3])]
        self.JT1LCD.setText(str(round((math.degrees(msg.position[0])), 3)))
        self.JT2LCD.setText(str(round((math.degrees(msg.position[1])), 3)))
        self.JT3LCD.setText(str(round((math.degrees(msg.position[2])), 3)))
        self.JT4LCD.setText(str(round((math.degrees(msg.position[3])), 3)))

    def rail_pose_callback(self, msg):
        self.rail_pose = msg.data
        self.CurrentPositionRail.setText(str(int(self.rail_pose)))


    def framestate(self, b):
        
        if b.text() == "Base":
            if b.isChecked() == True:
                self.frame = "base"
                    
        if b.text() == "Joint":
            if b.isChecked() == True:
                self.frame = "joint"

    def JT_IDLE(self):
        if self.frame == "base":
            bot.set_jog_command(0, 0)
        elif self.frame == "joint":
            bot.set_jog_command(1, 0)

        
    def JT1_move(self, sign):
        if sign.text() == "+":
            if self.frame == "base":
                bot.set_jog_command(0, 1)
            elif self.frame == "joint":
                bot.set_jog_command(1, 1)
        elif sign.text() == "-":
            if self.frame == "base":
                bot.set_jog_command(0, 2)
            elif self.frame == "joint":
                bot.set_jog_command(1, 2)

    def JT2_move(self, sign):
        if sign.text() == "+":
            if self.frame == "base":
                bot.set_jog_command(0, 3)
            elif self.frame == "joint":
                bot.set_jog_command(1, 3)
        elif sign.text() == "-":
            if self.frame == "base":
                bot.set_jog_command(0, 4)
            elif self.frame == "joint":
                bot.set_jog_command(1, 4)

    def JT3_move(self, sign):
        if sign.text() == "+":
            if self.frame == "base":
                bot.set_jog_command(0, 5)
            elif self.frame == "joint":
                bot.set_jog_command(1, 5)
        elif sign.text() == "-":
            if self.frame == "base":
                bot.set_jog_command(0, 6)
            elif self.frame == "joint":
                bot.set_jog_command(1, 6)

    def JT4_move(self, sign):
        if sign.text() == "+":
            if self.frame == "base":
                bot.set_jog_command(0, 7)
            elif self.frame == "joint":
                bot.set_jog_command(1, 7)
        elif sign.text() == "-":
            if self.frame == "base":
                bot.set_jog_command(0, 8)
            elif self.frame == "joint":
                bot.set_jog_command(1, 8)

    def button_clicked_HomingButton(self):   
        msg = QMessageBox()
        msg.setIcon(QMessageBox.Information)

        # msg.setText("Information")
        msg.setInformativeText("The homing procedure is about to start. Wait until the arm stops moving and the led stops flashing blue and turns green.")
        msg.setWindowTitle("Homing procedure")
        msg.setStandardButtons(QMessageBox.Ok)
        msg.exec_()

        command = 'ros2 service call /dobot_homing_service dobot_msgs/srv/ExecuteHomingProcedure'
        subprocess.Popen(
                    command, universal_newlines=True, shell=True,
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()

    def button_clicked_EStopButton(self):
        bot.stop_queue(force=True) 

        command = 'ros2 service call /PTP_action/_action/cancel_goal action_msgs/srv/CancelGoal'
        subprocess.Popen(
                    command, universal_newlines=True, shell=True,
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()

        msg = QMessageBox()
        msg.setIcon(QMessageBox.Warning)

        msg.setText("Warning")
        msg.setInformativeText("The stop button has been pressed. Make sure nothing is in the robot's workspace before resuming operation.")
        msg.setWindowTitle("Motion stop")
        msg.setStandardButtons(QMessageBox.Ok)
        msg.exec_()


    def open_gripper(self):
        command = 'ros2 service call /dobot_gripper_service dobot_msgs/srv/GripperControl "{gripper_state: "open", keep_compressor_running: false}"'
        subprocess.Popen(
                    command, universal_newlines=True, shell=True,
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()

        self.send_gripper_state("opened")
 
    def close_gripper(self):
        command = 'ros2 service call /dobot_gripper_service dobot_msgs/srv/GripperControl "{gripper_state: "close", keep_compressor_running: false}"'
        subprocess.Popen(
                    command, universal_newlines=True, shell=True,
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()

        self.send_gripper_state("closed")


    def turn_on_suction_cup(self):
        command = 'ros2 service call /dobot_suction_cup_service dobot_msgs/srv/SuctionCupControl "{enable_suction: true}"'
        subprocess.Popen(
            command, universal_newlines=True, shell=True,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()

    def turn_off_suction_cup(self):
        command = 'ros2 service call /dobot_suction_cup_service dobot_msgs/srv/SuctionCupControl "{enable_suction: false}"'
        subprocess.Popen(
            command, universal_newlines=True, shell=True,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()

    def valuechange_joints(self, field):
        if field.objectName() == "JT1Vel":
            self.JT1Vel.setText(str(self.Joint1VelSlider.value()))
        elif field.objectName() == "JT2Vel":
            self.JT2Vel.setText(str(self.Joint2VelSlider.value()))
        elif field.objectName() == "JT3Vel":
            self.JT3Vel.setText(str(self.Joint3VelSlider.value()))
        elif field.objectName() == "JT4Vel":
            self.JT4Vel.setText(str(self.Joint4VelSlider.value()))


    def valuechange_cartesian(self, field):
        if field.objectName() == "XVel":
            self.XVel.setText(str(self.XVelSlider.value()))
        elif field.objectName() == "YVel":
            self.YVel.setText(str(self.YVelSlider.value()))
        elif field.objectName() == "ZVel":
            self.ZVel.setText(str(self.ZVelSlider.value()))
        elif field.objectName() == "RVel":
            self.RVel.setText(str(self.RVelSlider.value()))

    def change_vel_joints(self, slider):
        bot.set_jog_joint_params([int(self.Joint1VelSlider.value()), int(self.Joint2VelSlider.value()), int(self.Joint3VelSlider.value()), int(self.Joint4VelSlider.value())], [100, 100, 100, 100])


    def change_vel_cartesian(self, slider):
        bot.set_jog_coordinate_params([int(self.XVelSlider.value()), int(self.YVelSlider.value()), int(self.ZVelSlider.value()), int(self.RVelSlider.value())], [100, 100, 100, 100])


    def sliding_rail_disconnected(self):
        msg = QMessageBox()
        msg.setIcon(QMessageBox.Warning)

        msg.setText("Sliding rail is not connected")
        msg.setInformativeText("If you have actually placed the manipulator on the sliding rail, you must additionally set MAGICIAN_RAIL_IN_USE environment variable to 'true'.")
        msg.setWindowTitle("WARNING")
        msg.setStandardButtons(QMessageBox.Close)
        msg.exec_()

    def sliding_rail_SV_slider(self):
        if not self.RAIL_IN_USE:
            self.sliding_rail_disconnected()
            return

        bot.set_point_to_point_sliding_rail_command(4,self.dobot_current_joint_states[0], self.dobot_current_joint_states[1], self.dobot_current_joint_states[2], self.dobot_current_joint_states[3], int(self.RailSVSlider.value()))

    def sliding_rail_current_pose(self):
        self.RailSVDisplay.setText(str(self.RailSVSlider.value()))

    def sliding_rail_params_slider(self):
        if not self.RAIL_IN_USE:
            self.sliding_rail_disconnected()
            return
        bot.set_point_to_point_sliding_rail_params(int(self.RailVelocitySlider.value()) * 2, int(self.RailAccelerationSlider.value()) * 2) #BUG

    def sliding_rail_vel_display(self):
        self.RailVelLCD.setText(str(self.RailVelocitySlider.value()))


    def sliding_rail_acc_display(self):
        self.RailAccLCD.setText(str(self.RailAccelerationSlider.value()))

    def button_clicked_StopRail(self):
        if not self.RAIL_IN_USE:
            self.sliding_rail_disconnected()
            return
        bot.stop_queue(force=True) 
        bot.clear_queue()
        bot.start_queue()

        msg = QMessageBox()
        msg.setIcon(QMessageBox.Warning)

        msg.setText("Warning")
        msg.setInformativeText("The stop button has been pressed. Make sure nothing is in the robot's workspace before resuming operation.")
        msg.setWindowTitle("Motion stop")
        msg.setStandardButtons(QMessageBox.Ok)
        msg.exec_()

    # --------------------------------------------------------
    def send_gripper_state(self, state):
        msg = GripperStatus()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.status = state
        self.gripper_state_publ.publish(msg)


          
    def start(self):
        pass

    def shutdown_plugin(self):
        pass
