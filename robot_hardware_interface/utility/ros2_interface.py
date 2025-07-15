#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Hexfellow. All rights reserved.
# Author: Jecjune <jecjune@qq.com>
# Date  : 2025-07-08
################################################################

import json
import threading
from typing import Tuple, List
import numpy as np

import rclpy
import rclpy.node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

from .interface_base import InterfaceBase

class DataInterface(InterfaceBase):

    def __init__(self, name: str = "unknown"):
        super(DataInterface, self).__init__(name=name)

        ### ros node
        rclpy.init()
        self.__node = rclpy.node.Node(name)
        self.__logger = self.__node.get_logger()
        self.__node.declare_parameter('rate_ros', 300.0)
        rate_value = self.__node.get_parameter('rate_ros').value
        self._rate_param["ros"] = float(rate_value if rate_value is not None else 300.0)
        self.__rate = self.__node.create_rate(self._rate_param["ros"])

        ### pamameter
        # declare parameters
        self.__node.declare_parameter('rate_state', 200.0)
        self._rate_param["state"] = self.__node.get_parameter('rate_state').value

        self.__node.declare_parameter('frame_id', "base_link")
        self.__frame_id = self.__node.get_parameter('frame_id').value

        self.__node.declare_parameter('simple_mode', True)
        self.__simple_mode = self.__node.get_parameter('simple_mode').value

        self.__node.declare_parameter('ws_url', "ws://127.0.0.1:8439")
        self.__ws_url = self.__node.get_parameter('ws_url').value

        ### publisher
        self.__motor_states_pub = self.__node.create_publisher(
            JointState,
            'motor_states',
            10,
        )
        self.__real_vel_pub = self.__node.create_publisher(
            TwistStamped,
            'real_vel',
            10,
        )
        self.__odom_pub = self.__node.create_publisher(
            Odometry,
            'odom',
            10,
        )

        ### subscriber
        self.__joint_ctrl_sub = self.__node.create_subscription(
            JointState,
            'joint_ctrl',
            self.__joint_ctrl_callback,
            10,
        )
        self.__cmd_vel_sub = self.__node.create_subscription(
            TwistStamped,
            'cmd_vel',
            self.__cmd_vel_callback,
            10,
        )

        self.__joint_ctrl_sub
        self.__cmd_vel_sub

        ### spin thread
        self.__spin_thread = threading.Thread(target=self.__spin)
        self.__spin_thread.start()

        ### finish log
        print(f"#### DataInterface init: {self._name} ####")

    def __spin(self):
        rclpy.spin(self.__node)

    def ok(self):
        return rclpy.ok()

    def shutdown(self):
        self.__node.destroy_node()
        rclpy.shutdown()
        self.__spin_thread.join()

    def sleep(self):
        self.__rate.sleep()

    def logd(self, msg, *args, **kwargs):
        self.__logger.debug(msg, *args, **kwargs)

    def logi(self, msg, *args, **kwargs):
        self.__logger.info(msg, *args, **kwargs)

    def logw(self, msg, *args, **kwargs):
        self.__logger.warning(msg, *args, **kwargs)

    def loge(self, msg, *args, **kwargs):
        self.__logger.error(msg, *args, **kwargs)

    def logf(self, msg, *args, **kwargs):
        self.__logger.fatal(msg, *args, **kwargs)

    def pub_motor_state(self, pos: List[float], vel: List[float], eff: List[float]):
        length = len(pos)
        msg = JointState()
        msg.header.stamp = self.__node.get_clock().now().to_msg()
        msg.name = [f"joint{i}" for i in range(length)]
        msg.position = pos
        msg.velocity = vel
        msg.effort = eff
        self.__motor_states_pub.publish(msg)

    def pub_real_vel(self, x: float, y: float, yaw: float):
        msg = TwistStamped()
        msg.header.stamp = self.__node.get_clock().now().to_msg()
        msg.header.frame_id = self.__frame_id
        msg.twist.linear.x = x
        msg.twist.linear.y = y
        msg.twist.angular.z = yaw
        self.__real_vel_pub.publish(msg)
    
    def pub_odom(self, pos_x: float, pos_y: float, pos_yaw: float, linear_x: float, linear_y: float, angular_z: float):
        out = Odometry()
        now = self.__node.get_clock().now()
        out.header.stamp = now.to_msg()
        out.header.frame_id = "odom"
        out.child_frame_id = self.__frame_id
        out.pose.pose.position.x = pos_x
        out.pose.pose.position.y = pos_y
        out.pose.pose.position.z = 0.0
        qz = np.sin(pos_yaw / 2.0)
        qw = np.cos(pos_yaw / 2.0)
        out.pose.pose.orientation.x = 0.0
        out.pose.pose.orientation.y = 0.0
        out.pose.pose.orientation.z = qz
        out.pose.pose.orientation.w = qw
        out.twist.twist.linear.x = linear_x
        out.twist.twist.linear.y = linear_y
        out.twist.twist.angular.z = angular_z
        self.__odom_pub.publish(out)

    def __joint_ctrl_callback(self, msg: JointState):
        self._joint_ctrl_queue.put(msg)

    def __cmd_vel_callback(self, msg: TwistStamped):
        with self._cmd_vel_lock:
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
        Only support one command at a time, so just throw away other commands.
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