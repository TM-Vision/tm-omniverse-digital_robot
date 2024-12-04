import os
from enum import Enum

from dotenv import load_dotenv
from tmrobot.digital_robot.models.digital_camera import Resolution


class RobotModel(Enum):
    NONE = 0
    TM5S = 1
    TM7S = 2
    TM12S = 3
    TM14S = 4
    TM25S = 5
    TM30S = 6


# Common Constants
EXTENSION_ROOT_PATH = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
load_dotenv(os.path.join(EXTENSION_ROOT_PATH, ".env"))  # MUST BE IN THE BEGINNING

EXTENSION_NAME = "TM Digital Robot"
FRAME_MESSAGE = "Message"
DEVELOPER_MODE = os.getenv("DEVELOPER_MODE", "false").lower() in ("true", "True")
VIRTUAL_CAMERA_SERVER_IP = os.getenv("VIRTUAL_CAMERA_SERVER_IP")
ROBOT_LIST: list[str] = ["Robot01", "Robot02", "Robot03", "Robot04"]
MAIN_SCENE_USD_NAME = "main_scene.usd"
SETTING_FILE_NAME = "settings.json"
SCENE_CAMERA_PATH = "/World/scene_camera"
LAST_SETTING_CACHE_FILE_NAME = ".last_settings_cache.txt"
EIH_RESOLUTION = (2592, 1944)


ROBOT_MODELS = [
    "",
    RobotModel.TM5S.name,
    RobotModel.TM7S.name,
    RobotModel.TM12S.name,
    RobotModel.TM14S.name,
    RobotModel.TM25S.name,
    RobotModel.TM30S.name,
]
RESOLUTION_OPTIONS = Resolution.get_all_options()
PORT_ETHERNET = 5891
PORT_ECHO_SERVER = 15568

# UI Constants
BUTTON_INITIAL = "initial"
BUTTON_START_SERVICE = "Start Service"
BUTTON_STOP_SERVICE = "Stop Service"
BUTTON_LOAD_SCENE = "Load Scene"
BUTTON_SAVE_SCENE = "Save Scene"
BUTTON_NEW_SCENE = "New Scene"
BUTTON_DISABLE_ALL = "Disable All"
BUTTON_ADD_ROBOT = "Add Robot"
BUTTON_LOAD_SETTING = "Load Setting"
BUTTON_SAVE_SETTING = "Save Setting"
BUTTON_CLEAR_SETTING = "Clear Setting"
BUTTON_REMOVE_ROBOT = "Remove Robot"
FIELD_SETTING_PATH = "Setting Path"
FIELD_USD_PATH = "USD Path"
FIELD_ACTIVATED = "Activated"
FIELD_MODEL = "Model"
FIELD_EXT01_RESOLUTION = "EXT01 Resolution"
FIELD_EXT01_WIDTH_LENGTH = "EXT01 Width,Length"
FIELD_EXT02_RESOLUTION = "EXT02 Resolution"
FIELD_EXT02_WIDTH_LENGTH = "EXT02 Width,Length"
FIELD_IP = "IP"
FIELD_CAMERA_ACTIVATED = "Camera Activated"
FIELD_ROBOT_PATH = "Robot Prim Path"
FIELD_EXT01_PATH = "EXT01 Prim Path"
FIELD_EXT02_PATH = "EXT02 Prim Path"
