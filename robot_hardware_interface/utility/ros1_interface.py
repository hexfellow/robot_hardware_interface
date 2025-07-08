#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Hexfellow. All rights reserved.
# Author: Jecjune <jecjune@qq.com>
# Date  : 2025-07-08
################################################################

import json
from typing import Tuple, List
import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped

from .interface_base import InterfaceBase


class DataInterface(InterfaceBase):

    def __init__(self, name: str = "unknown"):
        super(DataInterface, self).__init__(name=name)

        ### ros node
        rospy.init_node(self._name, anonymous=True)
        self._rate_param["ros"] = rospy.get_param('~rate_ros', 300.0)
        self.__rate = rospy.Rate(self._rate_param["ros"])

        ### pamameter
        # declare parameters
        self._rate_param["state"] = rospy.get_param('~rate_state', 200.0)
        self.__frame_id = rospy.get_param('~frame_id', "base_link")
        self.__simple_mode = rospy.get_param('~simple_mode', True)
        self.__ws_url = rospy.get_param('~ws_url', "ws://127.0.0.1:8439")

        ### publisher
        self.__joint_state_pub = rospy.Publisher(
            'joint_state',
            JointState,
            queue_size=10,
        )
        self.__real_vel_pub = rospy.Publisher(
            'real_vel',
            TwistStamped,
            queue_size=10,
        )

        ### subscriber
        if self.__simple_mode:
            self.__cmd_vel_sub = rospy.Subscriber(
            'cmd_vel',
            TwistStamped,
            self.__cmd_vel_callback,
            )
            self.__cmd_vel_sub
        else:
            self.__joint_ctrl_sub = rospy.Subscriber(
                'joint_ctrl',
                JointState,
                self.__joint_ctrl_callback,
            )
            self.__joint_ctrl_sub

        ### finish log
        print(f"#### DataInterface init: {self._name} ####")

    def ok(self):
        return not rospy.is_shutdown()

    def shutdown(self):
        pass

    def sleep(self):
        self.__rate.sleep()

    def logd(self, msg, *args, **kwargs):
        rospy.logdebug(msg, *args, **kwargs)

    def logi(self, msg, *args, **kwargs):
        rospy.loginfo(msg, *args, **kwargs)

    def logw(self, msg, *args, **kwargs):
        rospy.logwarn(msg, *args, **kwargs)

    def loge(self, msg, *args, **kwargs):
        rospy.logerr(msg, *args, **kwargs)

    def logf(self, msg, *args, **kwargs):
        rospy.logfatal(msg, *args, **kwargs)

    def pub_motor_state(self, pos: List[float], vel: List[float], eff: List[float]):
        out = JointState()
        out.header.stamp = rospy.Time.now()
        out.name = [f"joint{i}" for i in range(len(pos))]
        out.position = pos
        out.velocity = vel
        out.effort = eff
        self.__joint_state_pub.publish(out)
    
    def pub_real_vel(self, x: float, y: float, yaw: float):
        out = TwistStamped()
        out.header.stamp = rospy.Time.now()
        out.header.frame_id = self.__frame_id
        out.twist.linear.x = x
        out.twist.linear.y = y
        out.twist.angular.z = yaw
        self.__real_vel_pub.publish(out)

    def __joint_ctrl_callback(self, msg: JointState):
        self._joint_ctrl_queue.put(msg)

    def __cmd_vel_callback(self, msg: TwistStamped):
        self._cmd_vel_queue.put(msg)

    def get_ws_url(self) -> str:
        return str(self.__ws_url)

    def has_cmd_vel(self):
        '''
        Check if there is a new cmd_vel command.
        '''
        with self._cmd_vel_lock:
            return not self._cmd_vel_queue.empty()

    def get_cmd_vel(self) -> Tuple[float, float, float]:
        '''
        Must be called after has_cmd_vel() is True, otherwise will block.
        '''
        with self._cmd_vel_lock:
            cmd =  self._cmd_vel_queue.get()
        return cmd.twist.linear.x, cmd.twist.linear.y, cmd.twist.angular.z

    def has_joint_ctrl(self):
        '''
        Check if there is a new joint_ctrl command.
        '''
        with self._joint_ctrl_lock:
            return not self._joint_ctrl_queue.empty()
    
    def get_joint_ctrl(self, command: str) -> List[float]:
        '''
        command: "position", "velocity", "effort"
        '''
        with self._joint_ctrl_lock:
            cmd =  self._joint_ctrl_queue.get()
        if command == "position":
            return cmd.position
        elif command == "velocity":
            return cmd.velocity
        elif command == "effort":
            return cmd.effort
        else:
            raise ValueError(f"Invalid command: {command}")

    def is_simple_ctl(self) -> bool:
        '''
        Simple control mode: only send cmd_vel.
        Otherwise: send joint_ctrl.
        '''
        return bool(self.__simple_mode)