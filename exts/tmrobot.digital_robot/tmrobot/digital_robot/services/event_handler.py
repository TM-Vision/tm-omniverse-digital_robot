####################################################################################################################
# TECHMAN ROBOT INC. ("Techman") CONFIDENTIAL                                                                      #
# Copyright (C) 2016~2024 Techman. All Rights Reserved.                                                            #
# This source code and any compilation or derivative thereof is proprietary and confidential to Techman.           #
# Under no circumstances may any portion of the source code or any modified version of the source code be          #
# distributed, disclosed, or otherwise made available to any third party without the express written consent of    #
# Techman.                                                                                                         #
####################################################################################################################
import omni.kit.commands
from tmrobot.digital_robot.models.digital_camera import DigitalCamera
from tmrobot.digital_robot.models.message_camera import CameraProperty, MessageCamera


class EventHandler:
    def __init__(self, dg_cameras: dict[str, list[DigitalCamera]] = {}):
        self._dg_cameras = dg_cameras

    async def set_camera(self, message: MessageCamera):
        camera = self._get_camera_obj(message)
        print(f"camera: \n{camera}")
        print(f"message: \n{message}")

        if camera is not None:
            if message.set_property == CameraProperty.FOCAL_LENGTH:
                camera._om_camera.set_focal_length(message.value)
            if message.set_property == CameraProperty.GAIN:
                omni.kit.commands.execute(
                    "ChangeSetting",
                    path="/rtx/post/tonemap/filmIso",
                    value=message.value,
                )
            if message.set_property == CameraProperty.SHUTTER_TIME:
                omni.kit.commands.execute(
                    "ChangeSetting",
                    path="/rtx/post/tonemap/cameraShutter",
                    value=message.value,
                )
            if message.set_property == CameraProperty.WHITE_BALANCE:
                omni.kit.commands.execute(
                    "ChangeSetting",
                    path="/rtx/post/tonemap/whitepoint",
                    value=message.value,
                )

    def _get_camera_obj(self, message: MessageCamera):
        dg_cameras = self._dg_cameras.get(message.robot_ip)
        for camera in dg_cameras:
            if camera._serial_number == message.camera_sn:
                return camera
        return None
