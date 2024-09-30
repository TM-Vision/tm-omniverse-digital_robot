####################################################################################################################
# TECHMAN ROBOT INC. ("Techman") CONFIDENTIAL                                                                      #
# Copyright (C) 2016~2024 Techman. All Rights Reserved.                                                            #
# This source code and any compilation or derivative thereof is proprietary and confidential to Techman.           #
# Under no circumstances may any portion of the source code or any modified version of the source code be          #
# distributed, disclosed, or otherwise made available to any third party without the express written consent of    #
# Techman.                                                                                                         #
####################################################################################################################
import sys
import os
import grpc.aio
import asyncio
from google.protobuf import empty_pb2
from google.protobuf.json_format import MessageToJson
import inspect
import omni.replicator.core as rep  # type: ignore
import time
from concurrent.futures import ThreadPoolExecutor
from tmrobot.digital_robot.models.digital_camera import (
    DigitalCamera,
    CameraType,
    TriggerMode,
    ImageType,
    PixelFormat,
)
from tmrobot.digital_robot.models.message_camera import (
    CameraProperty,
    MessageCamera,
    ErrorMessage,
)


sys.path.append(os.path.join(os.path.dirname(__file__), "../modules"))
import tmrobot.digital_robot.modules.VirtualCameraAPI_pb2_grpc as VirtualCameraAPI_pb2_grpc  # noqa
import tmrobot.digital_robot.modules.VirtualCameraAPI_pb2 as VirtualCameraAPI  # noqa


class VirtualCameraServer(VirtualCameraAPI_pb2_grpc.VirtualCameraApiServicer):
    def __init__(
        self, set_queue: asyncio.Queue, dg_cameras: dict[str, list[DigitalCamera]]
    ):
        self._server = grpc.aio.server()
        self._set_queue: asyncio.Queue = set_queue
        self._dg_cameras: dict[str, list[DigitalCamera]] = dg_cameras
        self.stop_event = asyncio.Event()  # Used to signal server stop
        self.executor = ThreadPoolExecutor(max_workers=100)

    async def run_in_executor(self, func, *args):
        loop = asyncio.get_event_loop()
        return await loop.run_in_executor(self.executor, func, *args)

    async def start(self):
        print("Starting the virtual camera server...")

        VirtualCameraAPI_pb2_grpc.add_VirtualCameraApiServicer_to_server(
            self, self._server
        )
        self._server.add_insecure_port("[::]:9701")
        await self._server.start()
        print("Virtual camera server started.")
        # await self._server.wait_for_termination()
        await self.stop_event.wait()
        await self._server.stop(0.01)
        self.executor = ThreadPoolExecutor(max_workers=10)

    def stop(self):
        print("Received stop signal from extension.")
        self.stop_event.set()  # Signal to stop the server

    async def loadCameraList(self, request, context):
        client_ip = self._print_header(
            inspect.getframeinfo(inspect.currentframe()).function, context
        )
        self.device_list = []
        for dg_camera in self._dg_cameras[client_ip]:
            device = VirtualCameraAPI.CAMERA_ID(
                Type=CameraType.VIRTUAL_CAM.name,
                SerialNumber=dg_camera._serial_number,
                CameraName=dg_camera._serial_number,
                TriggerMode=TriggerMode.Software_Trigger.name,
            )
            self.device_list.append(device)

        print(f"request:\n{MessageToJson(request)}")
        camera_id_list = VirtualCameraAPI.loadCameraListResponse(
            CameraIDList=VirtualCameraAPI.CameraIDList(Devices=self.device_list)
        )
        print("response:\n" + MessageToJson(camera_id_list))
        return camera_id_list

    async def connectCamera(self, request, context):
        self._print_header(
            inspect.getframeinfo(inspect.currentframe()).function, context
        )
        print(f"request:\n{MessageToJson(request)}")
        context.set_code(grpc.StatusCode.NOT_FOUND)
        context.set_details(ErrorMessage.API_NOT_FOUND.name)
        print("response:\n" + MessageToJson(empty_pb2.Empty()))
        return empty_pb2.Empty()

    async def reconnectCamera(self, request, context):
        self._print_header(
            inspect.getframeinfo(inspect.currentframe()).function, context
        )
        print(f"request:\n{MessageToJson(request)}")
        context.set_code(grpc.StatusCode.NOT_FOUND)
        context.set_details(ErrorMessage.API_NOT_FOUND.name)
        print("response:\n" + MessageToJson(empty_pb2.Empty()))
        return empty_pb2.Empty()

    async def disconnectCamera(self, request, context):
        self._print_header(
            inspect.getframeinfo(inspect.currentframe()).function, context
        )
        print(f"request:\n{MessageToJson(request)}")
        context.set_code(grpc.StatusCode.NOT_FOUND)
        context.set_details(ErrorMessage.API_NOT_FOUND.name)
        print("response:\n" + MessageToJson(empty_pb2.Empty()))
        return empty_pb2.Empty()

    def get_camera_index_by_serial(self, serial_number, dg_cameras_client_ip):
        for index, dg_camera in enumerate(dg_cameras_client_ip):
            if dg_camera.get_serial_number() == serial_number:
                return index
        return 0

    async def getGrabImageData(self, request, context):
        # profiler = cProfile.Profile()
        # profiler.enable()
        start_time = time.time()
        client_ip = self._print_header(
            inspect.getframeinfo(inspect.currentframe()).function, context
        )
        print(f"request:\n{MessageToJson(request)}")
        dg_camera_index = self.get_camera_index_by_serial(
            request.SerialNumber, self._dg_cameras[client_ip]
        )
        r, g, b = self._dg_cameras[client_ip][dg_camera_index].get_white_balance()
        shutter_value = self._dg_cameras[client_ip][dg_camera_index].get_shutter_time()
        gain_value = self._dg_cameras[client_ip][dg_camera_index].get_gain()
        rep.settings.carb_settings("/rtx/post/tonemap/cameraShutter", shutter_value)
        rep.settings.carb_settings("/rtx/post/tonemap/filmIso", gain_value)
        rep.settings.carb_settings("/rtx/post/tonemap/whitepoint", [r, g, b])
        image_bytes = self._dg_cameras[client_ip][dg_camera_index].get_rgb()
        response = VirtualCameraAPI.getGrabImageDataResponse(
            ImageType=ImageType.png.name,
            PixelFormat=PixelFormat.RGB.name,
            EncodeString=image_bytes,
        )

        # print("response:\n" + MessageToJson(response))
        self._print_execution_time(start_time)
        # profiler.disable()
        # stats = pstats.Stats(profiler).sort_stats("cumtime")
        # stats.print_stats()
        return response

    async def getImageSize(self, request, context):
        client_ip = self._print_header(
            inspect.getframeinfo(inspect.currentframe()).function, context
        )
        print(f"request:\n{MessageToJson(request)}")
        dg_camera_index = self.get_camera_index_by_serial(
            request.SerialNumber, self._dg_cameras[client_ip]
        )

        image_size = self._dg_cameras[client_ip][dg_camera_index].get_image_size()
        image_width, image_height = image_size

        response = VirtualCameraAPI.getImageSizeResponse(
            ImageWidth=image_width, ImageHeight=image_height
        )

        print("response:\n" + MessageToJson(response))
        return response

    async def getShutterTime(self, request, context):
        client_ip = self._print_header(
            inspect.getframeinfo(inspect.currentframe()).function, context
        )
        print(f"request:\n{MessageToJson(request)}")
        dg_camera_index = self.get_camera_index_by_serial(
            request.SerialNumber, self._dg_cameras[client_ip]
        )
        current_value = self._dg_cameras[client_ip][dg_camera_index].get_shutter_time()

        response = VirtualCameraAPI.getShutterTimeResponse(
            ShutterTime=VirtualCameraAPI.CaptureSetting_value(
                CurrentValue=current_value, MinValue=1.0, MaxValue=5000.0
            )
        )

        print("response:\n" + MessageToJson(response))
        return response

    async def setShutterTimeAutoOnce(self, request, context):
        self._print_header(
            inspect.getframeinfo(inspect.currentframe()).function, context
        )
        print(f"request:\n{MessageToJson(request)}")

        # TODO: to be implemented

        response = empty_pb2.Empty()

        print("response:\n" + MessageToJson(response))
        return response

    async def setShutterTime(self, request, context):
        client_ip = self._print_header(
            inspect.getframeinfo(inspect.currentframe()).function, context
        )
        print(f"request:\n{MessageToJson(request)}")

        message = MessageCamera()
        message.robot_ip = client_ip
        message.camera_sn = request.SerialNumber
        message.set_property = CameraProperty.GAIN
        message.value = request.ShutterTimeValue
        dg_camera_index = self.get_camera_index_by_serial(
            request.SerialNumber, self._dg_cameras[client_ip]
        )
        self._dg_cameras[client_ip][dg_camera_index].set_shutter_time(message.value)

        self._set_queue.put(message)

        response = empty_pb2.Empty()

        print("response:\n" + MessageToJson(response))
        return response

    async def getGain(self, request, context):
        client_ip = self._print_header(
            inspect.getframeinfo(inspect.currentframe()).function, context
        )
        print(f"request:\n{MessageToJson(request)}")
        dg_camera_index = self.get_camera_index_by_serial(
            request.SerialNumber, self._dg_cameras[client_ip]
        )
        current_value = self._dg_cameras[client_ip][dg_camera_index].get_gain()

        response = VirtualCameraAPI.getGainResponse(
            Gain=VirtualCameraAPI.CaptureSetting_value(
                CurrentValue=current_value, MinValue=50.0, MaxValue=1600.0
            )
        )

        print("response:\n" + MessageToJson(response))
        return response

    async def setGain(self, request, context):
        client_ip = self._print_header(
            inspect.getframeinfo(inspect.currentframe()).function, context
        )
        print(f"request:\n{MessageToJson(request)}")

        message = MessageCamera()
        message.robot_ip = client_ip
        message.camera_sn = request.SerialNumber
        message.set_property = CameraProperty.GAIN
        message.value = request.GainValue
        dg_camera_index = self.get_camera_index_by_serial(
            request.SerialNumber, self._dg_cameras[client_ip]
        )
        self._dg_cameras[client_ip][dg_camera_index].set_gain(message.value)

        self._set_queue.put(message)
        response = empty_pb2.Empty()

        print("response:\n" + MessageToJson(response))
        return response

    async def setGainAutoOnce(self, request, context):
        client_ip = self._print_header(
            inspect.getframeinfo(inspect.currentframe()).function, context
        )
        print(f"request:\n{MessageToJson(request)}")

        message = MessageCamera()
        message.robot_ip = client_ip
        message.camera_sn = request.SerialNumber
        message.set_property = CameraProperty.GAIN
        dg_camera_index = self.get_camera_index_by_serial(
            request.SerialNumber, self._dg_cameras[client_ip]
        )
        self._dg_cameras[client_ip][dg_camera_index].set_gain(100.0)

        self._set_queue.put(message)

        response = empty_pb2.Empty()

        print("response:\n" + MessageToJson(response))
        return response

    async def getWhiteBalance(self, request, context):
        client_ip = self._print_header(
            inspect.getframeinfo(inspect.currentframe()).function, context
        )
        print(f"request:\n{MessageToJson(request)}")
        dg_camera_index = self.get_camera_index_by_serial(
            request.SerialNumber, self._dg_cameras[client_ip]
        )
        r, g, b = self._dg_cameras[client_ip][dg_camera_index].get_white_balance()

        response = VirtualCameraAPI.getWhiteBalanceResponse(
            WhiteBalance=VirtualCameraAPI.WhiteBalance(
                RedRatio=VirtualCameraAPI.CaptureSetting_value(
                    CurrentValue=r, MinValue=0.0, MaxValue=1.0
                ),
                GreenRatio=VirtualCameraAPI.CaptureSetting_value(
                    CurrentValue=g, MinValue=0.0, MaxValue=1.0
                ),
                BlueRatio=VirtualCameraAPI.CaptureSetting_value(
                    CurrentValue=b, MinValue=0.0, MaxValue=1.0
                ),
            )
        )

        print("response:\n" + MessageToJson(response))
        return response

    async def setWhiteBalance(self, request, context):
        client_ip = self._print_header(
            inspect.getframeinfo(inspect.currentframe()).function, context
        )
        print(f"request:\n{MessageToJson(request)}")

        message = MessageCamera()
        message.robot_ip = client_ip
        message.camera_sn = request.SerialNumber
        message.set_property = CameraProperty.WHITE_BALANCE
        message.value = (
            float(request.RedRatioValue),
            float(request.GreenRatioValue),
            float(request.BlueRatioValue),
        )
        dg_camera_index = self.get_camera_index_by_serial(
            request.SerialNumber, self._dg_cameras[client_ip]
        )
        self._dg_cameras[client_ip][dg_camera_index].set_white_balance(message.value)

        self._set_queue.put(message)

        response = empty_pb2.Empty()

        print("response:\n" + MessageToJson(response))
        return response

    async def setWhiteBalanceAutoOnce(self, request, context):
        client_ip = self._print_header(
            inspect.getframeinfo(inspect.currentframe()).function, context
        )
        print(f"request:\n{MessageToJson(request)}")

        message = MessageCamera()
        message.robot_ip = client_ip
        message.camera_sn = request.SerialNumber
        message.set_property = CameraProperty.WHITE_BALANCE
        dg_camera_index = self.get_camera_index_by_serial(
            request.SerialNumber, self._dg_cameras[client_ip]
        )
        self._dg_cameras[client_ip][dg_camera_index].set_white_balance((1.0, 1.0, 1.0))

        self._set_queue.put(message)

        response = empty_pb2.Empty()

        print("response:\n" + MessageToJson(response))
        return response

    async def getFocus(self, request, context):
        client_ip = self._print_header(
            inspect.getframeinfo(inspect.currentframe()).function, context
        )
        print(f"request:\n{MessageToJson(request)}")
        dg_camera_index = self.get_camera_index_by_serial(
            request.SerialNumber, self._dg_cameras[client_ip]
        )
        focal_length = self._dg_cameras[client_ip][dg_camera_index].get_focus()
        print(f"get_focal_length: {focal_length}")

        response = VirtualCameraAPI.getFocusResponse(
            Focus=VirtualCameraAPI.CaptureSetting_value(
                CurrentValue=int(focal_length.current * 10),
                MinValue=int(focal_length.minimum),
                MaxValue=int(focal_length.maximum),
            )
        )

        print("response:\n" + MessageToJson(response))
        return response

    async def setFocus(self, request, context):
        start_time = time.time()
        client_ip = self._print_header(
            inspect.getframeinfo(inspect.currentframe()).function, context
        )
        print(f"request:\n{MessageToJson(request)}")

        message = MessageCamera()
        message.robot_ip = client_ip
        message.camera_sn = request.SerialNumber
        message.set_property = CameraProperty.FOCAL_LENGTH
        dg_camera_index = self.get_camera_index_by_serial(
            request.SerialNumber, self._dg_cameras[client_ip]
        )

        self._dg_cameras[client_ip][dg_camera_index].set_focus, request.FocusValue

        message.value = request.FocusValue / 10

        self._set_queue.put_nowait(message)

        response = empty_pb2.Empty()

        print("response:\n" + MessageToJson(response))
        self._print_execution_time(start_time)
        return response

    async def setFocusAutoOnce(self, request, context):

        self._print_header(
            inspect.getframeinfo(inspect.currentframe()).function, context
        )
        print(f"request:\n{MessageToJson(request)}")

        context.set_code(grpc.StatusCode.NOT_FOUND)
        context.set_details(ErrorMessage.API_NOT_FOUND.name)

        print("response:\n" + MessageToJson(empty_pb2.Empty()))
        return empty_pb2.Empty()

    def _print_header(self, method, context):

        peer = context.peer()
        client_ip = peer.split(":")[1] if peer.startswith("ipv4:") else "127.0.0.1"

        print("==========================================")
        print(f"client: {client_ip}")
        print(f"method: {method}")

        return client_ip

    def _print_execution_time(self, start_time):
        time_elapsed = time.time() - start_time
        print(f"Execution time: {time_elapsed:.4f} seconds")
