####################################################################################################################
# TECHMAN ROBOT INC. ("Techman") CONFIDENTIAL                                                                      #
# Copyright (C) 2016~2024 Techman. All Rights Reserved.                                                            #
# This source code and any compilation or derivative thereof is proprietary and confidential to Techman.           #
# Under no circumstances may any portion of the source code or any modified version of the source code be          #
# distributed, disclosed, or otherwise made available to any third party without the express written consent of    #
# Techman.                                                                                                         #
####################################################################################################################
from dataclasses import dataclass
from omni.isaac.sensor import Camera
from enum import Enum


class CameraType(Enum):
    NONE = 0
    VIRTUAL_CAM = 1
    VIRTUAL_3DCAM = 2


class TriggerMode(Enum):
    Software_Trigger = 0
    Hardware_Trigger = 1


class ImageType(Enum):
    png = 0


class PixelFormat(Enum):
    RGB = 0
    MONO = 1


@dataclass
class CaptureSettings:
    current: float = 0
    maximum: float = 10000
    minimum: float = 0


class DigitalCamera:
    def __init__(self, om_camera: Camera, rgb_annotator: any):
        self._serial_number = ""
        self._prim_path = ""
        self._model = ""
        # cspell:disable
        self._image = (
            b"\x89PNG\r\n\x1a\n\x00\x00\x00\rIHDR\x00\x00\x00\x01\x00\x00\x00\x01"
            b"\x08\x02\x00\x00\x00\x90wS\xde\x00\x00\x00\nIDAT\x08\xd7c\xf8\x0f\x00"
            b"\x01\x01\x01\x00\x18\xdd\x8d\x18\x00\x00\x00\x00IEND\xaeB`\x82"
        )
        # cspell:enable
        self._image_type = ImageType.png.name
        self._pixel_format = "rgb8"
        self._image_width = 2592
        self._image_height = 1944
        self._shutter_time = 50.0  # Default Value in Isaac
        self._gain = 100.0  # Default Value in Isaac
        self._white_balance = (1.0, 1.0, 1.0)  # Default Value in Isaac
        self._om_camera = om_camera
        self._rgb_annotator = rgb_annotator
        self._focal_length: CaptureSettings = CaptureSettings()
        self._focus: CaptureSettings = CaptureSettings()

    def get_serial_number(self):
        return self._serial_number

    def set_serial_number(self, value):
        self._serial_number = value

    def get_prim_path(self):
        return self._prim_path

    def set_prim_path(self, value):
        self._prim_path = value

    def get_model(self):
        return self._model

    def set_model(self, value):
        self._model = value

    def get_image(self):
        return self._image

    def set_image(self, value):
        self._image = value

    def get_image_type(self):
        return self._image_type

    def set_image_type(self, value):
        self._image_type = value

    def get_pixel_format(self):
        return self._pixel_format

    def set_pixel_format(self, value):
        self._pixel_format = value

    def get_image_size(self):
        self._image_width = 2592
        self._image_height = 1944
        return (self._image_width, self._image_height)

    def set_image_size(self, value):
        self._image_width = value[0]
        self._image_height = value[1]

    def get_rgb(self):
        self._image = bytes(self._rgb_annotator.get_data())
        return self._image

    def get_shutter_time(self):
        return self._shutter_time

    def set_shutter_time(self, value: float):
        self._shutter_time = value

    def get_gain(self):
        return self._gain

    def set_gain(self, value: float):
        self._gain = value

    def get_white_balance(self):
        return self._white_balance

    def set_white_balance(self, value: float):
        self._white_balance = value

    def get_focus(self):
        self._focal_length.current = self._om_camera.get_focal_length()
        return self._focal_length

    def set_focus(self, value: float):
        self._focal_length.current = value
