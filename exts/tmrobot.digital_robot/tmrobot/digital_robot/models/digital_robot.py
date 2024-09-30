####################################################################################################################
# TECHMAN ROBOT INC. ("Techman") CONFIDENTIAL                                                                      #
# Copyright (C) 2016~2024 Techman. All Rights Reserved.                                                            #
# This source code and any compilation or derivative thereof is proprietary and confidential to Techman.           #
# Under no circumstances may any portion of the source code or any modified version of the source code be          #
# distributed, disclosed, or otherwise made available to any third party without the express written consent of    #
# Techman.                                                                                                         #
####################################################################################################################
from omni.isaac.core.robots.robot import Robot


class DigitalRobot:
    def __init__(self, ip: str, name: str, om_robot: Robot, rgb_annotator: any):
        self.ip = ip
        self.name = name
        self.model = ""
        self.om_robot = om_robot
        self._rgb_annotator = rgb_annotator

    # Setter methods
    def set_ip(self, ip: str):
        self.ip = ip

    def get_ip(self) -> str:
        return self.ip

    def set_name(self, name: str):
        self.name = name

    def get_name(self) -> str:
        return self.name

    def set_model(self, model: str):
        self.model = model

    def get_model(self) -> str:
        return self.model

    def get_articulation_controller(self) -> any:
        return self.om_robot.get_articulation_controller()

    def get_rgb(self):
        self._image = bytes(self._rgb_annotator.get_data())
        return self._image
