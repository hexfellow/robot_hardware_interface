#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Hexfellow. All rights reserved.
# Author: Jecjune <jecjune@qq.com>
# Date  : 2025-07-08
################################################################

import os

ROS_VERSION = os.environ.get('ROS_VERSION')
if ROS_VERSION == '1':
    from .ros1_interface import DataInterface as DataInterface
elif ROS_VERSION == '2':
    from .ros2_interface import DataInterface as DataInterface
else:
    raise ValueError("ROS_VERSION is not set")


__all__ = [
    "DataInterface",
]
