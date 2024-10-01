import inspect
import os
import sys
import time
from datetime import datetime

import grpc
from dotenv import load_dotenv
from google.protobuf.empty_pb2 import Empty
from google.protobuf.json_format import MessageToJson
from grpc import RpcError
from PIL import Image

sys.path.append(os.path.join(os.path.dirname(__file__), "../modules"))
import VirtualCameraAPI_pb2 as VirtualCameraAPI  # type: ignore # noqa
import VirtualCameraAPI_pb2_grpc as VirtualCameraAPI_pb2_grpc  # type: ignore # noqa

# CAMERA_RESOLUTION = (800, 600)
CAMERA_RESOLUTION = (2592, 1944)


class VirtualCameraClient:
    def __init__(self):
        env_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))), ".env"
        )
        load_dotenv(env_path)
        self._server_ip = os.getenv("VIRTUAL_CAMERA_SERVER_IP")

        MAX_MESSAGE_LENGTH = 20 * 1024 * 1024
        self.channel = grpc.insecure_channel(
            f"{self._server_ip}:9701",
            options=[
                ("grpc.max_send_message_length", MAX_MESSAGE_LENGTH),
                ("grpc.max_receive_message_length", MAX_MESSAGE_LENGTH),
            ],
        )  # (f"{self._server_ip}:9701")
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
            start_time = time.time()
            response = self.stub.getGrabImageData(request)
            image_bytes = response.EncodeString
            image = Image.frombytes(
                mode="RGBA", size=CAMERA_RESOLUTION, data=image_bytes
            )
            end_time = time.time()
            time_elapsed = end_time - start_time
            print(f"Execution time: {time_elapsed:.4f} seconds")
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
    test_camera = "robot_01_tm12_camera"
    try:
        client.loadCameraList()
        client.connectCamera(test_camera)
        client.reconnectCamera(test_camera)
        client.disconnectCamera(test_camera)
        client.setShutterTime(test_camera, 365.0)
        client.setShutterTimeAutoOnce(test_camera)
        client.getShutterTime(test_camera)
        client.setWhiteBalance(test_camera, 0.2, 0.3, 0.5)
        client.setWhiteBalanceAutoOnce(test_camera)
        client.getWhiteBalance(test_camera)
        client.setGain(test_camera, 20.0)
        client.setGainAutoOnce(test_camera)
        client.getGain(test_camera)
        client.setFocus(test_camera, 100)
        client.setFocusAutoOnce(test_camera)
        client.getFocus(test_camera)
        client.getGrabImageData(test_camera)

    except RpcError as e:
        print(f"Connection failed: \n{e}")
    finally:
        client.close()
