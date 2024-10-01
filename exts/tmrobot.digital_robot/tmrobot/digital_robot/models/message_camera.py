####################################################################################################################
# TECHMAN ROBOT INC. ("Techman") CONFIDENTIAL                                                                      #
# Copyright (C) 2016~2024 Techman. All Rights Reserved.                                                            #
# This source code and any compilation or derivative thereof is proprietary and confidential to Techman.           #
# Under no circumstances may any portion of the source code or any modified version of the source code be          #
# distributed, disclosed, or otherwise made available to any third party without the express written consent of    #
# Techman.                                                                                                         #
####################################################################################################################
from dataclasses import dataclass
from enum import Enum


class CameraProperty(Enum):
    NONE = 0
    SHUTTER_TIME = 1
    GAIN = 2
    WHITE_BALANCE = 3
    FOCAL_LENGTH = 4


@dataclass
class ErrorMessage:
    API_NOT_FOUND = "API_NOT_FOUND"
    TMFLOW_IP_NOT_FOUND = "is not one of the available TMFlow IPs"


@dataclass
class MessageCamera:
    robot_ip: str = ""
    camera_sn: str = ""
    set_property: CameraProperty = CameraProperty.NONE
    value: float = 0.0
