#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Hexfellow. All rights reserved.
# Author: Jecjune <jecjune@qq.com>
# Date  : 2025-07-08
################################################################

import copy
import numpy as np

import os
import sys

scrpit_path = os.path.abspath(os.path.dirname(__file__))
sys.path.append(scrpit_path)
from utility import DataInterface
from hex_vehicle import PublicAPI as VehicleAPI

class ChassisTrans:

    def __init__(self):
        ### data interface
        self.__data_interface = DataInterface("chassis_trans")

        ### parameter
        self.__rate_param = self.__data_interface.get_rate_param()
        url = self.__data_interface.get_ws_url()

        # Init VehicleAPI
        self.__api = VehicleAPI(ws_url = url, control_hz = self.__rate_param["state"], control_mode = "speed")
        self.__velocity_interface = self.__api.vehicle

    def work(self):
        assert self.__velocity_interface is not None, "Velocity interface not initialized"
        try:
            motor_cnt = self.__velocity_interface.get_motor_cnt()
            cmd_vel = [0.0] * motor_cnt
            cmd_x = 0.1
            cmd_y = 0.0
            cmd_yaw = 0.0

            while self.__data_interface.ok() and not self.__api.is_api_exit():
                # get data
                if self.__velocity_interface.has_new_data:
                    pos = self.__velocity_interface.get_motor_position()
                    vel = self.__velocity_interface.get_motor_velocity()
                    eff = self.__velocity_interface.get_motor_torque()
                    x, y, yaw = self.__velocity_interface.get_vehicle_speed()

                    # pub data
                    self.__data_interface.pub_motor_state(pos, vel, eff)
                    self.__data_interface.pub_real_vel(x, y, yaw)

                error = self.__velocity_interface.get_motor_error()
                if error is not None and any(motor_error for motor_error in error):
                    self.__data_interface.loge(f"Motor error: {error}")

                # control sending
                if self.__data_interface.is_simple_ctl():
                    # simple control
                    if self.__data_interface.has_cmd_vel():
                        cmd_x, cmd_y, cmd_yaw = self.__data_interface.get_cmd_vel()
                        print("cmd_x: ", cmd_x, "cmd_y: ", cmd_y, "cmd_yaw: ", cmd_yaw)
                    self.__velocity_interface.set_target_vehicle_speed(cmd_x, cmd_y, cmd_yaw)

                else:
                    # complex control
                    if self.__data_interface.has_joint_ctrl():
                        cmd_vel = self.__data_interface.get_joint_ctrl("velocity")
                        print("cmd_vel: ", cmd_vel)
                    self.__velocity_interface.set_motor_velocity(cmd_vel)

                # sleep
                self.__data_interface.sleep()
        except KeyboardInterrupt:
            self.__data_interface.logi("Received Ctrl-C.")
        finally:
            self.__api.close()
        exit(0)

def main():
    chassis_trans = ChassisTrans()
    chassis_trans.work()


if __name__ == '__main__':
    main()
