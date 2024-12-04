import asyncio
import logging
import os
import shutil
from datetime import datetime
from typing import Callable

import numpy as np
import omni.ext
import omni.kit.commands
import omni.kit.viewport.utility as vp_utils
import omni.kit.window.file
import omni.ui as ui
import omni.usd
from omni.isaac.core.utils.prims import find_matching_prim_paths, get_prim_at_path
from omni.isaac.core.utils.stage import (
    add_reference_to_stage,
    clear_stage,
    close_stage,
    create_new_stage,
    get_current_stage,
    is_stage_loading,
    omni,
    open_stage,
    update_stage_async,
)
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.sensor import Camera
from omni.isaac.ui.element_wrappers import CollapsableFrame, ScrollingWindow
from omni.isaac.ui.menu import make_menu_item_description
from omni.isaac.ui.ui_utils import (
    btn_builder,
    cb_builder,
    dropdown_builder,
    get_style,
    multi_cb_builder,
    scrolling_frame_builder,
    setup_ui_headers,
    str_builder,
)
from omni.kit.menu.utils import MenuItemDescription, add_menu_items, remove_menu_items
from pxr import Gf, Sdf, Usd, UsdGeom
from tmrobot.digital_robot.models.digital_camera import Resolution
from tmrobot.digital_robot.models.digital_robot import RobotCamera
from tmrobot.digital_robot.models.setting import ExtensionSetting, RobotSetting
from tmrobot.digital_robot.ui import constants as const


class ExtensionUI:
    def __init__(
        self, ext_id: str, start_service_fn: Callable, stop_service_fn: Callable
    ):
        self._models = {}
        self._ext_id = ext_id
        self._is_on_load_scene = False
        self._start_service_fn = start_service_fn
        self._stop_service_fn = stop_service_fn
        self._models: dict[str, object] = {}  # UI models
        self._models_robots: dict[str, dict[str, object]] = {}
        self._models_robots_callbacks: dict[str, dict[str, object]] = {}
        self._extension_setting = ExtensionSetting()
        self._menu_items = []
        self._window = ScrollingWindow(
            title=const.EXTENSION_NAME,
            width=600,
            height=500,
            visible=False,
            dockPreference=ui.DockPreference.LEFT_BOTTOM,
        )
        self._window.set_visibility_changed_fn(self._on_window)
        self._build_menu_item()
        self._robot_settings_frame: dict[str, CollapsableFrame] = {}

    # def _build_menu_item(self):

    #     menu_items = [
    #         MenuItemDescription(
    #             name="TMRobot",
    #             sub_menu=[
    #                 make_menu_item_description(
    #                     self._ext_id,
    #                     const.EXTENSION_NAME,
    #                     self._menu_toggle,
    #                 )
    #             ],
    #         )
    #     ]
    #     add_menu_items(menu_items, "Isaac Utils")

    def _build_menu_item(self):

        self._menu_items = [
            MenuItemDescription(
                name=const.EXTENSION_NAME,
                onclick_fn=self._menu_toggle,
                # sub_menu=[
                #     # make_menu_item_description(
                #     #     self._ext_id,
                #     #     const.EXTENSION_NAME,
                #     #     self._menu_toggle,
                #     # )
                # ],
            )
        ]
        add_menu_items(self._menu_items, "TMRobot")

    def _menu_toggle(self):
        # self._window.visible = not self._window.visible
        self._window.visible = True
        return

        # if not self._window:
        #     self._on_window()
        #     self._window.visible = False
        # else:
        #     self._window.visible = not self._window.visible
        # return

    def _on_window(self, visible: bool):
        if visible:
            if not self._window:
                self._window = ui.Window(
                    title=const.EXTENSION_NAME,
                    width=200,
                    height=0,
                    visible=True,
                    dockPreference=ui.DockPreference.LEFT_BOTTOM,
                )

        with self._window.frame:
            with ui.VStack(spacing=5, height=0):
                title = const.EXTENSION_NAME
                doc_link = "https://www.tm-robot.com"
                overview = "TM Digital Robot"
                setup_ui_headers(self._ext_id, __file__, title, doc_link, overview)
                self._build_message_frame()
                self._build_scene_frame()
                self._build_communication_frame()

                for robot_name in const.ROBOT_LIST:
                    self._robot_settings_frame[robot_name] = (
                        self._build_robot_settings_frame(robot_name)
                    )

                self.change_action_mode(const.BUTTON_INITIAL)

    def _build_message_frame(self):
        frame = ui.CollapsableFrame(
            title=const.FRAME_MESSAGE,
            height=0,
            collapsed=False,
            style=get_style(),
            style_type_name_override="CollapsableFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )

        ui_settings = {
            const.FRAME_MESSAGE: {
                "type": "cb_scrolling_frame",
                "tooltip": "",
            }
        }

        with frame:
            with ui.VStack(spacing=5, height=0):
                self._models[const.FRAME_MESSAGE] = scrolling_frame_builder(
                    **(ui_settings[const.FRAME_MESSAGE])
                )

    def _build_scene_frame(self):
        frame = ui.CollapsableFrame(
            title="Scene",
            height=0,
            collapsed=False,
            style=get_style(),
            style_type_name_override="CollapsableFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )

        ui_settings = {
            const.FIELD_SETTING_PATH: {
                "label": const.FIELD_SETTING_PATH,
                "default_val": "",
                "use_folder_picker": True,
                "tooltip": "",
                # "on_clicked_fn": self._on_dummy_callable,
            },
            const.FIELD_USD_PATH: {
                "label": const.FIELD_USD_PATH,
                "default_val": "",
                "use_folder_picker": True,
                "tooltip": "",
                # "on_clicked_fn": self._on_dummy_callable,
            },
            const.BUTTON_LOAD_SCENE: {
                "label": "",
                "type": "button",
                "text": self._get_verb(const.BUTTON_LOAD_SCENE),
                "tooltip": "",
                "on_clicked_fn": self._on_load_scene,
            },
            const.BUTTON_SAVE_SCENE: {
                "label": "",
                "type": "button",
                "text": self._get_verb(const.BUTTON_SAVE_SCENE),
                "tooltip": "",
                "on_clicked_fn": self._on_save_scene,
            },
            # const.BUTTON_SAVE_SETTING: {
            #     "label": "",
            #     "type": "button",
            #     "text": const.BUTTON_SAVE_SETTING,
            #     "tooltip": "",
            #     "on_clicked_fn": self._on_save_setting,
            # },
            # const.BUTTON_LOAD_SETTING: {
            #     "label": "",
            #     "type": "button",
            #     "text": const.BUTTON_LOAD_SETTING,
            #     "tooltip": "",
            #     "on_clicked_fn": self._on_load_setting,
            # },
            const.BUTTON_NEW_SCENE: {
                "label": "",
                "type": "button",
                "text": self._get_verb(const.BUTTON_NEW_SCENE),
                "tooltip": "",
                "on_clicked_fn": self._on_new_scene,
            },
        }

        with frame:
            with ui.VStack(spacing=5, height=0):
                self._models_robots[const.FIELD_SETTING_PATH] = str_builder(
                    **(ui_settings[const.FIELD_SETTING_PATH])
                )
                self._models_robots[const.FIELD_USD_PATH] = str_builder(
                    **(ui_settings[const.FIELD_USD_PATH])
                )
                self._models[const.BUTTON_LOAD_SCENE] = btn_builder(
                    **(ui_settings[const.BUTTON_LOAD_SCENE])
                )
                self._models[const.BUTTON_SAVE_SCENE] = btn_builder(
                    **(ui_settings[const.BUTTON_SAVE_SCENE])
                )
                # self._models[BUTTON_SAVE_SETTING] = btn_builder(
                #     **(ui_settings[BUTTON_SAVE_SETTING])
                # )
                # self._models[BUTTON_LOAD_SETTING] = btn_builder(
                #     **(ui_settings[BUTTON_LOAD_SETTING])
                # )
                self._models[const.BUTTON_NEW_SCENE] = btn_builder(
                    **(ui_settings[const.BUTTON_NEW_SCENE])
                )

    def change_action_mode(self, mode):
        if mode == const.BUTTON_INITIAL:
            self._models[const.BUTTON_LOAD_SCENE].enabled = True
            self._models[const.BUTTON_SAVE_SCENE].enabled = False
            self._models[const.BUTTON_NEW_SCENE].enabled = False
            self._models[const.BUTTON_START_SERVICE].enabled = False
            self._models[const.BUTTON_STOP_SERVICE].enabled = False

        if mode == const.BUTTON_DISABLE_ALL:
            self._models[const.BUTTON_LOAD_SCENE].enabled = False
            self._models[const.BUTTON_START_SERVICE].enabled = False
            self._models[const.BUTTON_STOP_SERVICE].enabled = False

        if mode == const.BUTTON_LOAD_SCENE:
            self._models[const.BUTTON_LOAD_SCENE].enabled = True
            self._models[const.BUTTON_START_SERVICE].enabled = False
            self._models[const.BUTTON_STOP_SERVICE].enabled = False

        if mode == const.BUTTON_START_SERVICE:
            self._models[const.BUTTON_LOAD_SCENE].enabled = False
            self._models[const.BUTTON_SAVE_SCENE].enabled = True
            self._models[const.BUTTON_NEW_SCENE].enabled = True
            self._models[const.BUTTON_START_SERVICE].enabled = True
            self._models[const.BUTTON_STOP_SERVICE].enabled = False

        if mode == const.BUTTON_STOP_SERVICE:
            self._models[const.BUTTON_LOAD_SCENE].enabled = False
            self._models[const.BUTTON_SAVE_SCENE].enabled = False
            self._models[const.BUTTON_NEW_SCENE].enabled = False
            self._models[const.BUTTON_START_SERVICE].enabled = False
            self._models[const.BUTTON_STOP_SERVICE].enabled = True

    def _get_verb(self, text):
        return text.split()[0]

    def _on_load_scene(self):
        self.change_action_mode(const.BUTTON_DISABLE_ALL)

        async def on_load_scene_async():
            self._on_window(visible=True)

            last_setting_cache_file_path = (
                f"{const.EXTENSION_ROOT_PATH}/{const.LAST_SETTING_CACHE_FILE_NAME}"
            )

            # If .last_settings_cache.txt does not exist, create a new USD and setting file
            if not os.path.exists(last_setting_cache_file_path):
                print("Setting file does not exist")
                current_time = datetime.now().strftime(
                    "%Y%m%d_%H%M%S"
                )  # YYYYMMDD_HHMMSS
                working_dir = f"{const.EXTENSION_ROOT_PATH}/.workspace/{current_time}"
                print(f"Working Directory: {working_dir}")
                os.makedirs(working_dir)

                # Copy default world file to the working directory
                shutil.copytree(
                    f"{const.EXTENSION_ROOT_PATH}/assets/worlds/accessories",
                    f"{working_dir}/accessories",
                )
                shutil.copytree(
                    f"{const.EXTENSION_ROOT_PATH}/assets/worlds/SubUSDs",
                    f"{working_dir}/SubUSDs",
                )
                shutil.copyfile(
                    f"{const.EXTENSION_ROOT_PATH}/assets/worlds/default_v5.usd",
                    f"{working_dir}/{const.MAIN_SCENE_USD_NAME}",
                )

                self._on_save_setting(
                    new_setting_path=f"{working_dir}/{const.SETTING_FILE_NAME}",
                    new_usd_path=f"{working_dir}/{const.MAIN_SCENE_USD_NAME}",
                )
            self._is_on_load_scene = True
            setting = self._on_load_setting()
            self._is_on_load_scene = False

            open_stage(setting.usd_path)

            # Create a main scene camera
            Camera(
                prim_path=const.SCENE_CAMERA_PATH,
                resolution=const.EIH_RESOLUTION,
                # frequency=60,
            )

            set_camera_view(
                eye=np.array([7, 4, 3]),
                target=np.array([0, -0.5, 0.7]),
                camera_prim_path=const.SCENE_CAMERA_PATH,
            )

            vp_utils.get_active_viewport().set_active_camera(const.SCENE_CAMERA_PATH)

            self.change_action_mode(const.BUTTON_START_SERVICE)

        asyncio.ensure_future(on_load_scene_async())

    def _on_save_scene(self):

        try:
            self._on_save_setting()

            omni.kit.window.file.save(
                dialog_options=omni.kit.window.file.DialogOptions.HIDE
            )

        except Exception as e:
            print(f"Error: {e}")

        print("Scene saved")

    def _on_save_setting(self, new_setting_path="", new_usd_path=""):

        # Get the values from the UI
        if new_setting_path != "":
            setting_path = new_setting_path
        else:
            setting_path = self._models_robots[
                const.FIELD_SETTING_PATH
            ].get_value_as_string()

        if new_usd_path != "":
            usd_path = new_usd_path
        else:
            usd_path = self._models_robots[const.FIELD_USD_PATH].get_value_as_string()

        # Save last setting path
        if setting_path != "":
            with open(
                f"{const.EXTENSION_ROOT_PATH}/{const.LAST_SETTING_CACHE_FILE_NAME}", "w"
            ) as file:
                file.write(setting_path)

        # Create an instance of ExtensionSetting
        self._extension_setting.usd_path = usd_path
        self._extension_setting.setting_path = setting_path

        for robot_name in const.ROBOT_LIST:
            # fmt: off
            activated: ui.SimpleBoolModel = self._models_robots[robot_name][const.FIELD_ACTIVATED]
            ip: ui.SimpleStringModel = self._models_robots[robot_name][const.FIELD_IP]
            model: ui.AbstractItemModel = self._models_robots[robot_name][const.FIELD_MODEL]
            eih_activated_mdl: ui.SimpleBoolModel = self._models_robots[robot_name][RobotCamera.EIH.name]
            ext01_activated_mdl: ui.SimpleBoolModel = self._models_robots[robot_name][RobotCamera.EXT01.name]
            ext02_activated_mdl: ui.SimpleBoolModel = self._models_robots[robot_name][RobotCamera.EXT02.name]
            ext01_resolution_mdl: ui.AbstractItemModel = self._models_robots[robot_name][const.FIELD_EXT01_RESOLUTION]
            ext02_resolution_mdl: ui.AbstractItemModel = self._models_robots[robot_name][const.FIELD_EXT02_RESOLUTION]
            ext01_width_length_mdl: ui.SimpleStringModel = self._models_robots[robot_name][const.FIELD_EXT01_WIDTH_LENGTH] # noqa
            ext02_width_length_mdl: ui.SimpleStringModel = self._models_robots[robot_name][const.FIELD_EXT02_WIDTH_LENGTH] # noqa
            robot_prim_path_mdl: ui.SimpleStringModel = self._models_robots[robot_name][const.FIELD_ROBOT_PATH]
            ext01_prim_path_mdl: ui.SimpleStringModel = self._models_robots[robot_name][const.FIELD_EXT01_PATH]
            ext02_prim_path_mdl: ui.SimpleStringModel = self._models_robots[robot_name][const.FIELD_EXT02_PATH]

            activated = activated.as_bool
            ip = ip.get_value_as_string()
            model_index = model.get_item_value_model().get_value_as_int()
            model_name = const.ROBOT_MODELS[model_index]
            eih_activated = eih_activated_mdl.as_bool
            ext01_activated = ext01_activated_mdl.as_bool
            ext02_activated = ext02_activated_mdl.as_bool

            # Get the EXT01 resolution values
            ext01_index = ext01_resolution_mdl.get_item_value_model().get_value_as_int()
            ext01_resolution_key = const.RESOLUTION_OPTIONS[ext01_index]
            ext01_resolution_value: tuple = (0, 0)

            if ext01_resolution_key == "Custom":
                ext01_width_length_str = ext01_width_length_mdl.get_value_as_string()
                ext01_resolution_value = tuple(map(int, ext01_width_length_str.split(',')))
            else:
                ext01_resolution_value = Resolution.get_resolution_by_key(ext01_resolution_key)

            ext02_index = ext02_resolution_mdl.get_item_value_model().get_value_as_int()
            ext02_resolution_key = const.RESOLUTION_OPTIONS[ext02_index]
            ext02_resolution_value: tuple = (0, 0)

            # Get the EXT02 resolution values
            if ext02_resolution_key == "Custom":
                ext02_width_length_str = ext02_width_length_mdl.get_value_as_string()
                ext02_resolution_value = tuple(map(int, ext02_width_length_str.split(',')))
            else:
                ext02_resolution_value = Resolution.get_resolution_by_key(ext02_resolution_key)

            robot_prim_path = robot_prim_path_mdl.get_value_as_string()
            ext01_prim_path = ext01_prim_path_mdl.get_value_as_string()
            ext02_prim_path = ext02_prim_path_mdl.get_value_as_string()
            # fmt: on

            # Create a sample Robot01 Setting
            robot_setting = RobotSetting()
            robot_setting.activated = activated
            robot_setting.name = robot_name
            robot_setting.ip = ip
            robot_setting.model = model_name
            robot_setting.cameras_activated = {
                RobotCamera.EIH.name: eih_activated,
                RobotCamera.EXT01.name: ext01_activated,
                RobotCamera.EXT02.name: ext02_activated,
            }
            robot_setting.ext01_resolution_value = ext01_resolution_value
            robot_setting.ext02_resolution_value = ext02_resolution_value
            robot_setting.robot_prim_path = robot_prim_path
            robot_setting.ext01_prim_path = ext01_prim_path
            robot_setting.ext02_prim_path = ext02_prim_path
            self._extension_setting.robots_setting[robot_setting.name] = robot_setting

        # omni.kit.window.file.save(on_save_done: Optional[Callable[[bool, str], None]] = None,
        #                           exit=False,
        #                           dialog_options=DialogOptions.NONE)

        # omni.kit.window.file.save_stage_ui.StageSaveDialog._checkpoint_marks[""] = {}

        self._extension_setting.save_extension_setting_to_json(setting_path)
        print("Settings saved")

    def _on_load_setting(self):
        # To check if last setting file exists
        last_setting_cache_file_path = (
            f"{const.EXTENSION_ROOT_PATH}/{const.LAST_SETTING_CACHE_FILE_NAME}"
        )
        setting_file_path = ""
        with open(last_setting_cache_file_path, "r") as file:
            setting_file_path = file.readline()

        setting = self._extension_setting.load_extension_setting_from_json(
            setting_file_path
        )

        if len(self._models_robots) == 0:
            logging.error("Models are not initialized")
            return

        self._models_robots[const.FIELD_USD_PATH].set_value(setting.usd_path)
        self._models_robots[const.FIELD_SETTING_PATH].set_value(setting.setting_path)

        for robot_name in const.ROBOT_LIST:
            # fmt: off
            activated_mdl: ui.SimpleBoolModel = self._models_robots[robot_name][const.FIELD_ACTIVATED]
            model_mdl: ui.AbstractItemModel = self._models_robots[robot_name][const.FIELD_MODEL]
            ip_mdl: ui.SimpleStringModel = self._models_robots[robot_name][const.FIELD_IP]
            eih_activated_mdl: ui.SimpleBoolModel = self._models_robots[robot_name][RobotCamera.EIH.name]
            ext01_activated_mdl: ui.SimpleBoolModel = self._models_robots[robot_name][RobotCamera.EXT01.name]
            ext02_activated_mdl: ui.SimpleBoolModel = self._models_robots[robot_name][RobotCamera.EXT02.name]
            ext01_resolution_mdl: ui.AbstractItemModel = self._models_robots[robot_name][const.FIELD_EXT01_RESOLUTION]
            ext02_resolution_mdl: ui.AbstractItemModel = self._models_robots[robot_name][const.FIELD_EXT02_RESOLUTION]
            ext01_width_length_mdl: ui.SimpleStringModel = self._models_robots[robot_name][const.FIELD_EXT01_WIDTH_LENGTH] # noqa
            ext02_width_length_mdl: ui.SimpleStringModel = self._models_robots[robot_name][const.FIELD_EXT02_WIDTH_LENGTH] # noqa
            robot_prim_path_mdl: ui.SimpleStringModel = self._models_robots[robot_name][const.FIELD_ROBOT_PATH]
            ext01_prim_path_mdl: ui.SimpleStringModel = self._models_robots[robot_name][const.FIELD_EXT01_PATH]
            ext02_prim_path_mdl: ui.SimpleStringModel = self._models_robots[robot_name][const.FIELD_EXT02_PATH]

            # Handle EXT01 resolution values
            ext01_res = setting.robots_setting[robot_name].ext01_resolution_value
            ext01_resolution_key = Resolution.get_key_by_value(tuple(ext01_res))
            ext01_resolution_mdl.get_item_value_model().set_value(const.RESOLUTION_OPTIONS.index(ext01_resolution_key))
            ext01_wl = ""
            if ext01_res != [0, 0]:
                ext01_wl = f"{ext01_res[0]},{ext01_res[1]}"

            ext01_width_length_mdl.set_value(ext01_wl)

            # Handle EXT02 resolution values
            ext02_res = setting.robots_setting[robot_name].ext02_resolution_value
            ext02_resolution_key = Resolution.get_key_by_value(tuple(ext02_res))
            ext02_resolution_mdl.get_item_value_model().set_value(const.RESOLUTION_OPTIONS.index(ext02_resolution_key))
            ext02_wl = ""
            if ext02_res != [0, 0]:
                ext02_wl = f"{ext02_res[0]},{ext02_res[1]}"

            ext02_width_length_mdl.set_value(ext02_wl)

            activated_mdl.set_value(setting.robots_setting[robot_name].activated)
            model_mdl.get_item_value_model().set_value(const.ROBOT_MODELS.index(setting.robots_setting[robot_name].model)) # noqa
            ip_mdl.set_value(setting.robots_setting[robot_name].ip)
            eih_activated_mdl.set_value(setting.robots_setting[robot_name].cameras_activated[RobotCamera.EIH.name])
            ext01_activated_mdl.set_value(setting.robots_setting[robot_name].cameras_activated[RobotCamera.EXT01.name])
            ext02_activated_mdl.set_value(setting.robots_setting[robot_name].cameras_activated[RobotCamera.EXT02.name])
            robot_prim_path_mdl.set_value(setting.robots_setting[robot_name].robot_prim_path)
            ext01_prim_path_mdl.set_value(setting.robots_setting[robot_name].ext01_prim_path)
            ext02_prim_path_mdl.set_value(setting.robots_setting[robot_name].ext02_prim_path)

            # fmt: on
        return setting

    def clean_form(self):
        usd_path: ui.SimpleStringModel = self._models_robots[const.FIELD_USD_PATH]
        setting_path: ui.SimpleStringModel = self._models_robots[
            const.FIELD_SETTING_PATH
        ]
        usd_path.set_value("")
        setting_path.set_value("")

        self._extension_setting.usd_path = usd_path
        self._extension_setting.setting_path = setting_path

        for robot_name in const.ROBOT_LIST:
            # fmt: off
            activated: ui.SimpleBoolModel = self._models_robots[robot_name][const.FIELD_ACTIVATED]
            ip: ui.SimpleStringModel = self._models_robots[robot_name][const.FIELD_IP]
            model: ui.AbstractItemModel = self._models_robots[robot_name][const.FIELD_MODEL]
            eih_activated: ui.SimpleBoolModel = self._models_robots[robot_name][RobotCamera.EIH.name]
            ext01_activated: ui.SimpleBoolModel = self._models_robots[robot_name][RobotCamera.EXT01.name]
            ext02_activated: ui.SimpleBoolModel = self._models_robots[robot_name][RobotCamera.EXT02.name]
            ext01_resolution: ui.AbstractItemModel = self._models_robots[robot_name][const.FIELD_EXT01_RESOLUTION]
            ext01_width_length: ui.SimpleStringModel = self._models_robots[robot_name][const.FIELD_EXT01_WIDTH_LENGTH]
            ext02_resolution: ui.AbstractItemModel = self._models_robots[robot_name][const.FIELD_EXT02_RESOLUTION]
            ext02_width_length: ui.SimpleStringModel = self._models_robots[robot_name][const.FIELD_EXT02_WIDTH_LENGTH]
            robot_prim_path: ui.SimpleStringModel = self._models_robots[robot_name][const.FIELD_ROBOT_PATH]
            ext01_prim_path: ui.SimpleStringModel = self._models_robots[robot_name][const.FIELD_EXT01_PATH]
            ext02_prim_path: ui.SimpleStringModel = self._models_robots[robot_name][const.FIELD_EXT02_PATH]

            activated.set_value(False)
            ip.set_value("")
            model.get_item_value_model().set_value(0)
            eih_activated.set_value(False)
            ext01_activated.set_value(False)
            ext02_activated.set_value(False)
            ext01_resolution.get_item_value_model().set_value(0)
            ext01_width_length.set_value("")
            ext02_resolution.get_item_value_model().set_value(0)
            ext02_width_length.set_value("")
            robot_prim_path.set_value("")
            ext01_prim_path.set_value("")
            ext02_prim_path.set_value("")

    def _on_new_scene(self):
        # Delete last setting cache file
        last_setting_cache_file_path = (
            f"{const.EXTENSION_ROOT_PATH}/{const.LAST_SETTING_CACHE_FILE_NAME}"
        )
        if os.path.exists(last_setting_cache_file_path):
            os.remove(last_setting_cache_file_path)

        # Get the values from the UI and reset them
        self._is_on_load_scene = True
        self.clean_form()
        create_new_stage()
        self._is_on_load_scene = False
        self.change_action_mode(const.BUTTON_INITIAL)

    def _build_robot_settings_frame(self, robot_name: str):
        frame = ui.CollapsableFrame(
            title=f"{robot_name} Settings",
            height=0,
            collapsed=False,
            style=get_style(),
            style_type_name_override="CollapsableFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )

        self._models_robots[robot_name] = {}
        self._models_robots_callbacks[robot_name] = {}

        def __on_robot_activated(activated: bool):
            if self._is_on_load_scene:
                return
            print(f"{robot_name} Activated: {activated}")
            # robot_prim_path_mdl: ui.SimpleStringModel = self._models_robots[robot_name][
            #     const.FIELD_ROBOT_PATH
            # ]
            omni.kit.commands.execute(
                "ToggleActivePrims",
                stage_or_context=get_current_stage(),
                prim_paths=[f"/World/{robot_name}"],
                active=activated,
            )
            self._on_save_scene()

        def __on_model_selected(model: str):
            if self._is_on_load_scene or model == "":
                return
            print(f"{robot_name} Model Selected: {model}")

            # fmt: off
            robot_prim_path_mdl: ui.SimpleStringModel = self._models_robots[robot_name][const.FIELD_ROBOT_PATH]
            ext01_prim_path_mdl: ui.SimpleStringModel = self._models_robots[robot_name][const.FIELD_EXT01_PATH]
            ext02_prim_path_mdl: ui.SimpleStringModel = self._models_robots[robot_name][const.FIELD_EXT02_PATH]
            robot_activated: ui.SimpleBoolModel = self._models_robots[robot_name][const.FIELD_ACTIVATED]
            eih_activated: ui.SimpleBoolModel = self._models_robots[robot_name][RobotCamera.EIH.name]
            ext01_activated: ui.SimpleBoolModel = self._models_robots[robot_name][RobotCamera.EXT01.name]
            ext02_activated: ui.SimpleBoolModel = self._models_robots[robot_name][RobotCamera.EXT02.name]

            # fmt: on
            prim_name = model.lower()

            exist_robots = find_matching_prim_paths(f"/World/{robot_name}/*")
            robot_prim_path = f"/World/{robot_name}/{prim_name}"
            ext01_prim_path = f"/World/{robot_name}/ext01_2d_camera"
            ext02_prim_path = f"/World/{robot_name}/ext02_2d_camera"

            if len(exist_robots) == 0:
                self._create_xform(f"/World/{robot_name}")
                add_reference_to_stage(
                    usd_path=f"../../assets/robot_series/{prim_name}.usd",
                    prim_path=robot_prim_path,
                )
                robot_size = self._get_prim_size(robot_prim_path)

                eih_prim_path = f"{robot_prim_path}/body/tool0/eih_2d_camera"

                # fmt: on
                # Disable EIH Camera
                omni.kit.commands.execute(
                    "ToggleActivePrims",
                    stage_or_context=get_current_stage(),
                    prim_paths=[eih_prim_path],
                    active=False,
                )

                # Add EXT01 Camera
                add_reference_to_stage(
                    usd_path="../../assets/cameras/ext_2d_camera_v2.usd",
                    prim_path=ext01_prim_path,
                )
                ext_prim = get_prim_at_path(ext01_prim_path)
                ext_prim.GetPrim().GetAttribute("xformOp:translate").Set(
                    Gf.Vec3d(0.25, -0.8, robot_size[2])
                )
                ext_prim.GetPrim().GetAttribute("xformOp:rotateXYZ").Set(
                    Gf.Vec3d(0, 0, 180)
                )
                omni.kit.commands.execute(
                    "ToggleActivePrims",
                    stage_or_context=get_current_stage(),
                    prim_paths=[ext01_prim_path],
                    active=False,
                )

                # Add EXT02 Camera
                add_reference_to_stage(
                    usd_path="../../assets/cameras/ext_2d_camera_v2.usd",
                    prim_path=ext02_prim_path,
                )
                ext_prim = get_prim_at_path(ext02_prim_path)
                ext_prim.GetPrim().GetAttribute("xformOp:translate").Set(
                    Gf.Vec3d(-0.25, -0.8, robot_size[2])
                )
                ext_prim.GetPrim().GetAttribute("xformOp:rotateXYZ").Set(
                    Gf.Vec3d(0, 0, 180)
                )
                omni.kit.commands.execute(
                    "ToggleActivePrims",
                    stage_or_context=get_current_stage(),
                    prim_paths=[ext02_prim_path],
                    active=False,
                )

                # Create gripper
                gripper_prim_path = (
                    f"/World/{robot_name}/{model.lower()}/body/flange_link/gripper"
                )
                add_reference_to_stage(
                    usd_path="../../assets/grippers/epick_base_v1.usd",
                    prim_path=gripper_prim_path,
                )

                gripper_size = self._get_prim_size(gripper_prim_path)
                x, y, z = gripper_size
                print(f"Gripper Size (Meter): x={x}, y={y}, z={z}")

                omni.kit.commands.execute(
                    "ToggleActivePrims",
                    stage_or_context=get_current_stage(),
                    prim_paths=[gripper_prim_path],
                    active=True,
                )

                # Save settings
                robot_activated.set_value(True)
                eih_activated.set_value(False)
                ext01_activated.set_value(False)
                ext02_activated.set_value(False)
                robot_prim_path_mdl.set_value(robot_prim_path)
                ext01_prim_path_mdl.set_value(ext01_prim_path)
                ext02_prim_path_mdl.set_value(ext02_prim_path)
                # fmt: on

                self._on_save_scene()
            else:
                print(f"{exist_robots[0]} exists, you can't change to another robot")

        def __on_ip_input(ip: ui.SimpleStringModel):
            if self._is_on_load_scene:
                return

            print(f"{robot_name} IP: {ip.get_value_as_string()}")
            self._on_save_setting()

        def __on_eih_activated(activated: bool):
            if self._is_on_load_scene:
                return
            print(f"{robot_name} EIH Camera Activated:{activated}")
            robot_prim_path_mdl: ui.SimpleStringModel = self._models_robots[robot_name][
                const.FIELD_ROBOT_PATH
            ]
            eih_prim_path = (
                f"{robot_prim_path_mdl.get_value_as_string()}/body/tool0/eih_2d_camera"
            )

            omni.kit.commands.execute(
                "ToggleActivePrims",
                stage_or_context=get_current_stage(),
                prim_paths=[eih_prim_path],
                active=activated,
            )

            self._on_save_setting()

        def __on_ext01_activated(activated: bool):
            if self._is_on_load_scene:
                return

            print(f"{robot_name} EXT01 Activated: {activated}")
            ext01_prim_path_mdl: ui.SimpleStringModel = self._models_robots[robot_name][
                const.FIELD_EXT01_PATH
            ]

            omni.kit.commands.execute(
                "ToggleActivePrims",
                stage_or_context=get_current_stage(),
                prim_paths=[ext01_prim_path_mdl.get_value_as_string()],
                active=activated,
            )

            self._on_save_scene()

        def __on_ext02_activated(activated: bool):
            if self._is_on_load_scene:
                return
            print(f"{robot_name} EXT02 Activated:{activated}")
            ext01_prim_path_mdl: ui.SimpleStringModel = self._models_robots[robot_name][
                const.FIELD_EXT02_PATH
            ]

            omni.kit.commands.execute(
                "ToggleActivePrims",
                stage_or_context=get_current_stage(),
                prim_paths=[ext01_prim_path_mdl.get_value_as_string()],
                active=activated,
            )

            self._on_save_scene()

        def __on_ext01_resolution_selected(val: str):
            if self._is_on_load_scene:
                return

            print(f"{robot_name} Resolution Selected: {val}")
            resolution = ""
            if val != "":
                resolution_tuple = Resolution.get_resolution_by_key(val)
                resolution = f"{resolution_tuple[0]},{resolution_tuple[1]}"

            self._models_robots[robot_name][const.FIELD_EXT01_WIDTH_LENGTH].set_value(
                resolution
            )

            self._on_save_setting()

        def __on_ext02_resolution_selected(val: str):
            if self._is_on_load_scene:
                return
            print(f"{robot_name} Resolution Selected: {val}")
            resolution = ""
            if val != "":
                resolution_tuple = Resolution.get_resolution_by_key(val)
                resolution = f"{resolution_tuple[0]},{resolution_tuple[1]}"

            self._models_robots[robot_name][const.FIELD_EXT02_WIDTH_LENGTH].set_value(
                resolution
            )

            self._on_save_setting()

        def __on_remove_robot():
            if self._is_on_load_scene:
                return
            print(f"Remove Robot: {robot_name}")

            try:

                # omni.kit.commands.execute(
                #     "ToggleActivePrims",
                #     stage_or_context=get_current_stage(),
                #     prim_paths=[Sdf.Path(f"/World/{robot_name}")],
                #     active=False,
                # )

                omni.kit.commands.execute(
                    "DeletePrims",
                    paths=[f"/World/{robot_name}"],
                    destructive=False,
                )

            except Exception as e:
                print(f"Failed to delete prims for {robot_name}: {e}")

            # fmt: off
            activated: ui.SimpleBoolModel = self._models_robots[robot_name][const.FIELD_ACTIVATED]
            ip: ui.SimpleStringModel = self._models_robots[robot_name][const.FIELD_IP]
            model: ui.AbstractItemModel = self._models_robots[robot_name][const.FIELD_MODEL]
            eih_activated: ui.SimpleBoolModel = self._models_robots[robot_name][RobotCamera.EIH.name]
            ext01_activated: ui.SimpleBoolModel = self._models_robots[robot_name][RobotCamera.EXT01.name]
            ext02_activated: ui.SimpleBoolModel = self._models_robots[robot_name][RobotCamera.EXT02.name]
            ext01_resolution: ui.AbstractItemModel = self._models_robots[robot_name][const.FIELD_EXT01_RESOLUTION]
            ext01_width_length: ui.SimpleStringModel = self._models_robots[robot_name][const.FIELD_EXT01_WIDTH_LENGTH]
            ext02_resolution: ui.AbstractItemModel = self._models_robots[robot_name][const.FIELD_EXT02_RESOLUTION]
            ext02_width_length: ui.SimpleStringModel = self._models_robots[robot_name][const.FIELD_EXT02_WIDTH_LENGTH]
            robot_prim_path: ui.SimpleStringModel = self._models_robots[robot_name][const.FIELD_ROBOT_PATH]
            ext01_prim_path: ui.SimpleStringModel = self._models_robots[robot_name][const.FIELD_EXT01_PATH]
            ext02_prim_path: ui.SimpleStringModel = self._models_robots[robot_name][const.FIELD_EXT02_PATH]
            # fmt: on

            activated.set_value(False)
            ip.set_value("")
            model.get_item_value_model().set_value(0)
            eih_activated.set_value(False)
            ext01_activated.set_value(False)
            ext02_activated.set_value(False)
            ext01_resolution.get_item_value_model().set_value(0)
            ext01_width_length.set_value("")
            ext02_resolution.get_item_value_model().set_value(0)
            ext02_width_length.set_value("")
            robot_prim_path.set_value("")
            ext01_prim_path.set_value("")
            ext02_prim_path.set_value("")

            self._on_save_scene()

        ui_settings = {
            const.FIELD_ACTIVATED: {
                "label": self._get_verb(const.FIELD_ACTIVATED),
                "type": "checkbox",
                "default_val": False,
                "tooltip": "Enable/Disable the Robot",
                "on_clicked_fn": __on_robot_activated,
            },
            const.FIELD_MODEL: {
                "label": "Model",
                "type": "dropdown",
                "default_val": 0,
                "tooltip": "The Robot Model",
                "items": const.ROBOT_MODELS,
                "on_clicked_fn": __on_model_selected,
            },
            const.FIELD_IP: {
                "label": f"{self._remove_underline(const.FIELD_IP)}",
                "type": "stringfield",
                "default_val": "",
                "tooltip": "IP Address of the TM Simulator or TM Flow",
                "on_clicked_fn": __on_ip_input,
            },
            const.FIELD_CAMERA_ACTIVATED: {
                "label": f"{self._remove_underline(const.FIELD_CAMERA_ACTIVATED)}",
                "count": 3,
                "type": "multi_checkbox",
                "default_val": [False, False, False],
                "text": [
                    RobotCamera.EIH.name,
                    RobotCamera.EXT01.name,
                    RobotCamera.EXT02.name,
                ],
                "tooltip": [
                    "Enable/Disable the Cameras",
                    "Eye-in-Hand Camera",
                    "Eye-to-Hand Camera 01",
                    "Eye-to-Hand Camera 01",
                ],
                "on_clicked_fn": [
                    __on_eih_activated,
                    __on_ext01_activated,
                    __on_ext02_activated,
                ],
            },
            const.FIELD_EXT01_RESOLUTION: {
                "label": const.FIELD_EXT01_RESOLUTION,
                "type": "dropdown",
                "default_val": 0,
                "tooltip": "Select the a pre-defined resolution or select custom",
                "items": const.RESOLUTION_OPTIONS,
                "on_clicked_fn": __on_ext01_resolution_selected,
            },
            const.FIELD_EXT01_WIDTH_LENGTH: {
                "label": f"{self._remove_underline(const.FIELD_EXT01_WIDTH_LENGTH)}",
                "type": "stringfield",
                "default_val": "",
                "tooltip": "",
            },
            const.FIELD_EXT02_RESOLUTION: {
                "label": const.FIELD_EXT02_RESOLUTION,
                "type": "dropdown",
                "default_val": 0,
                "tooltip": "",
                "items": const.RESOLUTION_OPTIONS,
                "on_clicked_fn": __on_ext02_resolution_selected,
            },
            const.FIELD_EXT02_WIDTH_LENGTH: {
                "label": f"{self._remove_underline(const.FIELD_EXT02_WIDTH_LENGTH)}",
                "type": "stringfield",
                "default_val": "",
                "tooltip": "",
            },
            const.FIELD_ROBOT_PATH: {
                "label": const.FIELD_ROBOT_PATH,
                "type": "stringfield",
                "default_val": "",
                "tooltip": "",
            },
            const.FIELD_EXT01_PATH: {
                "label": const.FIELD_EXT01_PATH,
                "type": "stringfield",
                "default_val": "",
                "tooltip": "",
            },
            const.FIELD_EXT02_PATH: {
                "label": const.FIELD_EXT02_PATH,
                "type": "stringfield",
                "default_val": "",
                "tooltip": "",
            },
            const.BUTTON_REMOVE_ROBOT: {
                "label": "",
                "type": "button",
                "text": self._get_verb(const.BUTTON_REMOVE_ROBOT),
                "tooltip": "Add Tooltip Here",
                "on_clicked_fn": __on_remove_robot,
            },
        }

        with frame:
            with ui.VStack(spacing=5, height=0):
                self._models_robots[robot_name][const.FIELD_ACTIVATED] = cb_builder(
                    **(ui_settings[const.FIELD_ACTIVATED])
                )
                self._models_robots[robot_name][const.FIELD_MODEL] = dropdown_builder(
                    **(ui_settings[const.FIELD_MODEL])
                )
                self._models_robots[robot_name][const.FIELD_IP] = str_builder(
                    **(ui_settings[const.FIELD_IP])
                )
                options = multi_cb_builder(
                    **(ui_settings[const.FIELD_CAMERA_ACTIVATED])
                )

                # fmt: off
                for i in range(len(options)):
                    if ui_settings[const.FIELD_CAMERA_ACTIVATED]["text"][i] == RobotCamera.EIH.name:
                        self._models_robots[robot_name][RobotCamera.EIH.name] = options[i]

                    if ui_settings[const.FIELD_CAMERA_ACTIVATED]["text"][i] == RobotCamera.EXT01.name:
                        self._models_robots[robot_name][RobotCamera.EXT01.name] = options[i]

                    if ui_settings[const.FIELD_CAMERA_ACTIVATED]["text"][i] == RobotCamera.EXT02.name:
                        self._models_robots[robot_name][RobotCamera.EXT02.name] = options[i]

                # fmt: on

                # camera_name = ui_settings[const.FIELD_CAMERA_ACTIVATED]["text"][i]
                # label_name = ui_settings[const.FIELD_CAMERA_ACTIVATED]["label"]
                # self._models_robots[robot_name][
                #     label_name + "_" + camera_name
                # ] = option

                self._models_robots[robot_name][const.FIELD_EXT01_RESOLUTION] = (
                    dropdown_builder(**(ui_settings[const.FIELD_EXT01_RESOLUTION]))
                )
                self._models_robots[robot_name][const.FIELD_EXT01_WIDTH_LENGTH] = (
                    str_builder(**(ui_settings[const.FIELD_EXT01_WIDTH_LENGTH]))
                )
                self._models_robots[robot_name][const.FIELD_EXT02_RESOLUTION] = (
                    dropdown_builder(**(ui_settings[const.FIELD_EXT02_RESOLUTION]))
                )
                self._models_robots[robot_name][const.FIELD_EXT02_WIDTH_LENGTH] = (
                    str_builder(**(ui_settings[const.FIELD_EXT02_WIDTH_LENGTH]))
                )
                self._models_robots[robot_name][const.FIELD_ROBOT_PATH] = str_builder(
                    **(ui_settings[const.FIELD_ROBOT_PATH])
                )
                self._models_robots[robot_name][const.FIELD_EXT01_PATH] = str_builder(
                    **(ui_settings[const.FIELD_EXT01_PATH])
                )

                self._models_robots[robot_name][const.FIELD_EXT02_PATH] = str_builder(
                    **(ui_settings[const.FIELD_EXT02_PATH])
                )
                self._models_robots[robot_name][const.BUTTON_REMOVE_ROBOT] = (
                    btn_builder(**(ui_settings[const.BUTTON_REMOVE_ROBOT]))
                )

        return frame

    def _remove_underline(self, text):
        return text.replace("_", " ")

    def _build_communication_frame(self):
        frame = ui.CollapsableFrame(
            title="Communication",
            height=0,
            collapsed=False,
            style=get_style(),
            style_type_name_override="CollapsableFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )

        ui_settings = {
            f"{const.BUTTON_START_SERVICE}": {
                "label": const.BUTTON_START_SERVICE,
                "type": "button",
                "text": self._get_verb(const.BUTTON_START_SERVICE),
                "tooltip": "",
                "on_clicked_fn": self._start_service_fn,
            },
            f"{const.BUTTON_STOP_SERVICE}": {
                "label": const.BUTTON_STOP_SERVICE,
                "type": "button",
                "text": self._get_verb(const.BUTTON_STOP_SERVICE),
                "tooltip": "",
                "on_clicked_fn": self._stop_service_fn,
            },
        }

        with frame:
            with ui.VStack(spacing=5, height=0):
                self._models[const.BUTTON_START_SERVICE] = btn_builder(
                    **(ui_settings[const.BUTTON_START_SERVICE])
                )
                self._models[const.BUTTON_STOP_SERVICE] = btn_builder(
                    **(ui_settings[const.BUTTON_STOP_SERVICE])
                )

    def _create_xform(self, prim_path: str):
        omni.kit.commands.execute(
            "CreatePrimWithDefaultXform",
            prim_type="Xform",
            prim_path=prim_path,
            attributes={},
            select_new_prim=True,
        )

    def clear(self):
        remove_menu_items(menu=self._menu_items, name="TMRobot", can_rebuild_menus=True)
        self._menu_items = None
        self._models.clear()
        self._models_robots.clear()
        self._models_robots_callbacks.clear()
        self._window = None
        self._start_service_fn = None
        self._stop_service_fn = None

    def _get_prim_size(self, prim_path: str) -> Gf.Vec3d:
        stage = omni.usd.get_context().get_stage()
        bbox_cache = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(), includedPurposes=[UsdGeom.Tokens.default_]
        )
        bbox_cache.Clear()
        pallet_right_prim = stage.GetPrimAtPath(Sdf.Path(prim_path))
        prim_bbox = bbox_cache.ComputeWorldBound(pallet_right_prim)
        prim_range = prim_bbox.ComputeAlignedRange()
        prim_size: Gf.Vec3d = prim_range.GetSize()
        # x = prim_size[0]
        # y = prim_size[1]
        # z = prim_size[2]
        # print(f"Prim size: x={x}, y={y}, z={z}")

        return prim_size

    def enabled_robot_settings(self, enabled: bool):
        for robot_name in const.ROBOT_LIST:
            self._robot_settings_frame[robot_name].enabled = enabled
            # self._robot_settings_frame[robot_name].visible = enabled
            self._robot_settings_frame[robot_name].collapsed = not enabled
