from datetime import datetime
import glob
import os
import sys
import time
import grpc
from dotenv import load_dotenv
from grpc import RpcError
from google.protobuf.empty_pb2 import Empty
from google.protobuf.json_format import MessageToJson
import inspect
import numpy as np

sys.path.append(os.path.join(os.path.dirname(__file__), "../modules"))
import VirtualCameraAPI_pb2_grpc as VirtualCameraAPI_pb2_grpc
import VirtualCameraAPI_pb2 as VirtualCameraAPI
from PIL import Image

# CAMERA_RESOLUTION = (800, 600)
CAMERA_RESOLUTION = (2592, 1944)


class VirtualCameraClient:
    def __init__(self):
        env_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))), ".env"
        )
        load_dotenv(env_path)
        self.service_ip = os.getenv("DEV_IP")

        MAX_MESSAGE_LENGTH = 20 * 1024 * 1024
        self.channel = grpc.insecure_channel(
            f"{self.service_ip}:9701",
            options=[
                ("grpc.max_send_message_length", MAX_MESSAGE_LENGTH),
                ("grpc.max_receive_message_length", MAX_MESSAGE_LENGTH),
            ],
        )  # (f"{self.service_ip}:9701")
        self.stub = VirtualCameraAPI_pb2_grpc.VirtualCameraApiStub(self.channel)
        self._current_script_dir = os.path.dirname(os.path.abspath(__file__))

    def print_header(self, method):
        print("==========================================")
        print(f"method: {method}")

    def print_error(self, e: RpcError):
        print("response:")
        print(f"status: {e.code()}")
        print(f"message: {e.details()}")

    def format_message(self, message):
        return MessageToJson(
            message,
            preserving_proto_field_name=True,
            always_print_fields_with_no_presence=True,
        )

    def loadCameraList(self):
        self.print_header(inspect.getframeinfo(inspect.currentframe()).function)
        print(f"request:\n{self.format_message(Empty())}")

        try:
            response = self.stub.loadCameraList(Empty())
            print(f"response:\n{self.format_message(response)}")
        except RpcError as e:
            self.print_error(e)

    def connectCamera(self, serialNumber):
        self.print_header(inspect.getframeinfo(inspect.currentframe()).function)
        request = VirtualCameraAPI.CameraSerialNumberRequest(SerialNumber=serialNumber)
        print(f"request:\n{self.format_message(request)}")

        try:
            response = self.stub.connectCamera(request)
            print(f"response:\n{self.format_message(response)}")

        except RpcError as e:
            self.print_error(e)

    def reconnectCamera(self, serialNumber):
        self.print_header(inspect.getframeinfo(inspect.currentframe()).function)
        request = VirtualCameraAPI.CameraSerialNumberRequest(SerialNumber=serialNumber)
        print(f"request:\n{self.format_message(request)}")

        try:
            response = self.stub.reconnectCamera(request)
            print(f"response:\n{self.format_message(response)}")

        except RpcError as e:
            self.print_error(e)

    def disconnectCamera(self, serialNumber):
        self.print_header(inspect.getframeinfo(inspect.currentframe()).function)
        request = VirtualCameraAPI.CameraSerialNumberRequest(SerialNumber=serialNumber)
        print(f"request:\n{self.format_message(request)}")

        try:
            response = self.stub.disconnectCamera(request)
            print(f"response:\n{self.format_message(response)}")

        except RpcError as e:
            self.print_error(e)

    def getGrabImageData(self, serialNumber):
        self.print_header(inspect.getframeinfo(inspect.currentframe()).function)
        request = VirtualCameraAPI.CameraSerialNumberRequest(SerialNumber=serialNumber)
        print(f"request:\n{self.format_message(request)}")

        try:
            response = self.stub.getGrabImageData(request)
            # print(f"response:\n{self.format_message(response)}")

            # TEST: show Image
            image_bytes = response.EncodeString

            image = Image.frombytes(
                mode="RGBA", size=CAMERA_RESOLUTION, data=image_bytes
            )
            # image.show()

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            image_name = f"{serialNumber}_{timestamp}.png"
            image_path = os.path.join(self._current_script_dir, "tmp", image_name)
            print(f"image_path: {image_path}")
            image.save(image_path)

        except RpcError as e:
            self.print_error(e)

    def getImageBytes(self, serialNumber):
        self.print_header(inspect.getframeinfo(inspect.currentframe()).function)
        request = VirtualCameraAPI.CameraSerialNumberRequest(SerialNumber=serialNumber)
        print(f"request:\n{self.format_message(request)}")

        try:
            response = self.stub.getGrabImageData(request)
            # print(f"response:\n{self.format_message(response)}")

            # TEST: show Image
            image_bytes = response.EncodeString

            image = Image.frombytes(
                mode="RGBA", size=CAMERA_RESOLUTION, data=image_bytes
            )

            return image

        except RpcError as e:
            self.print_error(e)

    def getImageSize(self, serialNumber):
        self.print_header(inspect.getframeinfo(inspect.currentframe()).function)
        request = VirtualCameraAPI.CameraSerialNumberRequest(SerialNumber=serialNumber)
        print(f"request:\n{self.format_message(request)}")

        try:
            response = self.stub.getImageSize(request)
            print(f"response:\n{self.format_message(response)}")

        except RpcError as e:
            self.print_error(e)

    def getShutterTime(self, serialNumber):
        self.print_header(inspect.getframeinfo(inspect.currentframe()).function)
        request = VirtualCameraAPI.CameraSerialNumberRequest(SerialNumber=serialNumber)
        print(f"request:\n{self.format_message(request)}")

        try:
            response = self.stub.getShutterTime(request)
            print(f"response:\n{self.format_message(response)}")

        except RpcError as e:
            self.print_error(e)

    def setShutterTime(self, serialNumber, shutterTime):
        self.print_header(inspect.getframeinfo(inspect.currentframe()).function)
        request = VirtualCameraAPI.setShutterTimeRequest(
            SerialNumber=serialNumber, ShutterTimeValue=shutterTime
        )
        print(f"request:\n{self.format_message(request)}")

        try:
            response = self.stub.setShutterTime(request)
            print(f"response:\n{self.format_message(response)}")

        except RpcError as e:
            self.print_error(e)

    def setShutterTimeAutoOnce(self, serialNumber):
        self.print_header(inspect.getframeinfo(inspect.currentframe()).function)
        request = VirtualCameraAPI.CameraSerialNumberRequest(SerialNumber=serialNumber)
        print(f"request:\n{self.format_message(request)}")

        try:
            response = self.stub.setShutterTimeAutoOnce(request)
            print(f"response:\n{self.format_message(response)}")

        except RpcError as e:
            self.print_error(e)

    def getGain(self, serialNumber):
        self.print_header(inspect.getframeinfo(inspect.currentframe()).function)
        request = VirtualCameraAPI.CameraSerialNumberRequest(SerialNumber=serialNumber)
        print(f"request:\n{self.format_message(request)}")

        try:
            response = self.stub.getGain(request)
            print(f"response:\n{self.format_message(response)}")

        except RpcError as e:
            self.print_error(e)

    def setGain(self, serialNumber, gain):
        self.print_header(inspect.getframeinfo(inspect.currentframe()).function)
        request = VirtualCameraAPI.setGainRequest(
            SerialNumber=serialNumber, GainValue=gain
        )
        print(f"request:\n{self.format_message(request)}")

        try:
            response = self.stub.setGain(request)
            print(f"response:\n{self.format_message(response)}")

        except RpcError as e:
            self.print_error(e)

    def setGainAutoOnce(self, serialNumber):
        self.print_header(inspect.getframeinfo(inspect.currentframe()).function)
        request = VirtualCameraAPI.CameraSerialNumberRequest(SerialNumber=serialNumber)
        print(f"request:\n{self.format_message(request)}")

        try:
            response = self.stub.setGainAutoOnce(request)
            print(f"response:\n{self.format_message(response)}")

        except RpcError as e:
            self.print_error(e)

    def getWhiteBalance(self, serialNumber):
        self.print_header(inspect.getframeinfo(inspect.currentframe()).function)
        request = VirtualCameraAPI.CameraSerialNumberRequest(SerialNumber=serialNumber)
        print(f"request:\n{self.format_message(request)}")

        try:
            response = self.stub.getWhiteBalance(request)
            print(f"response:\n{self.format_message(response)}")

        except RpcError as e:
            self.print_error(e)

    def setWhiteBalance(self, serialNumber, wbRedRatio, wbGreenRatio, wbBlueRatio):
        self.print_header(inspect.getframeinfo(inspect.currentframe()).function)
        request = VirtualCameraAPI.setWhiteBalanceRequest(
            SerialNumber=serialNumber,
            RedRatioValue=wbRedRatio,
            GreenRatioValue=wbGreenRatio,
            BlueRatioValue=wbBlueRatio,
        )
        print(f"request:\n{self.format_message(request)}")

        try:
            response = self.stub.setWhiteBalance(request)
            print(f"response:\n{self.format_message(response)}")

        except RpcError as e:
            self.print_error(e)

    def setWhiteBalanceAutoOnce(self, serialNumber):
        self.print_header(inspect.getframeinfo(inspect.currentframe()).function)
        request = VirtualCameraAPI.CameraSerialNumberRequest(SerialNumber=serialNumber)
        print(f"request:\n{self.format_message(request)}")

        try:
            response = self.stub.setWhiteBalanceAutoOnce(request)
            print(f"response:\n{self.format_message(response)}")

        except RpcError as e:
            self.print_error(e)

    def getFocus(self, serialNumber):
        self.print_header(inspect.getframeinfo(inspect.currentframe()).function)
        request = VirtualCameraAPI.CameraSerialNumberRequest(SerialNumber=serialNumber)
        print(f"request:\n{self.format_message(request)}")

        try:
            response = self.stub.getFocus(request)
            print(f"response:\n{self.format_message(response)}")

        except RpcError as e:
            self.print_error(e)

    def setFocus(self, serialNumber, focus):
        self.print_header(inspect.getframeinfo(inspect.currentframe()).function)
        request = VirtualCameraAPI.setFocusRequest(
            SerialNumber=serialNumber, FocusValue=focus
        )
        print(f"request:\n{self.format_message(request)}")

        try:
            response = self.stub.setFocus(request)
            print(f"response:\n{self.format_message(response)}")

        except RpcError as e:
            self.print_error(e)

    def setFocusAutoOnce(self, serialNumber):
        self.print_header(inspect.getframeinfo(inspect.currentframe()).function)
        request = VirtualCameraAPI.CameraSerialNumberRequest(SerialNumber=serialNumber)
        print(f"request:\n{self.format_message(request)}")

        try:
            response = self.stub.setFocusAutoOnce(request)
            print(f"response:\n{self.format_message(response)}")

        except RpcError as e:
            self.print_error(e)

    def close(self):
        self.channel.close()


# Example usage:
if __name__ == "__main__":
    client = VirtualCameraClient()
    try:
        client.loadCameraList()
        # # client.connectCamera("world_camera")
        # # client.reconnectCamera("world_camera")
        # # client.disconnectCamera("world_camera")
        # client.getGrabImageData("world_camera")
        # client.getGrabImageData("world_camera")
        # client.getGrabImageData("world_camera")

        # client.setShutterTime("robot_01_tm12_camera", 365.0)
        # client.setWhiteBalance("robot_01_tm12_camera", 0.2, 0.3, 0.5)
        # # client.setFocus("vc_camera_02", 10)
        # client.getGrabImageData("robot_01_tm12_camera")
        # client.getGrabImageData("robot_01_tm12_camera")
        # client.getGrabImageData("robot_01_tm12_camera")

        # client.getImageSize("world_camera")
        # client.setShutterTime("world_camera", 365.0)
        # client.getShutterTime("world_camera")
        # client.setShutterTimeAutoOnce("world_camera")
        # client.getGain("world_camera")
        # client.setGain("world_camera", 20.0)
        # client.setGainAutoOnce("world_camera")
        # client.setWhiteBalance("world_camera", 0.2, 0.3, 0.5)
        # client.getWhiteBalance("world_camera")
        # client.setFocus("world_camera", 100)
        # client.setWhiteBalanceAutoOnce("world_camera")
        # client.setFocusAutoOnce("world_camera")
        # client.getFocus("world_camera")

        # for i in np.arange(0, 100, 0.1):
        #     client.setFocus("world_camera", i)
        # tmp_path = os.path.join(os.path.dirname(__file__), "tmp")
        # png_files = glob.glob(os.path.join(tmp_path, "*.png"))
        # for file_path in png_files:
        #     os.remove(file_path)

        # for i in np.arange(0, 100, 1):
        #     print(f"No.: {i}")
        #     client.getGrabImageData("robot_01_tm12_camera")

        # client.setFocus("world_camera", 1)
        # time.sleep(0.1)
        # client.setFocus("world_camera", 10)
        # time.sleep(0.1)
        # client.setFocus("world_camera", 50)
        # time.sleep(0.1)
        # client.setFocus("world_camera", 1)

        # time.sleep(3)
        # client.getFocus("world_camera")

    except RpcError as e:
        print(f"Connection failed: \n{e}")
    finally:
        client.close()
