####################################################################################################################
# TECHMAN ROBOT INC. ("Techman") CONFIDENTIAL                                                                      #
# Copyright (C) 2016~2024 Techman. All Rights Reserved.                                                            #
# This source code and any compilation or derivative thereof is proprietary and confidential to Techman.           #
# Under no circumstances may any portion of the source code or any modified version of the source code be          #
# distributed, disclosed, or otherwise made available to any third party without the express written consent of    #
# Techman.                                                                                                         #
####################################################################################################################
import asyncio
import gc
import math
import os
import queue
import random
import socket
import threading
import weakref
from typing import List

import numpy as np
import omni.ext
import omni.kit.commands
import omni.kit.viewport.utility as vp_utils
import omni.replicator.core as rep  # type: ignore
import omni.ui as ui
import omni.usd
from dotenv import load_dotenv
from omni.isaac.core.robots.robot import Robot
from omni.isaac.core.utils.stage import open_stage, update_stage_async
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.core.world.world import World
from omni.isaac.sensor import Camera
from omni.isaac.ui.element_wrappers import ScrollingWindow
from omni.isaac.ui.menu import make_menu_item_description
from omni.isaac.ui.ui_utils import btn_builder, get_style, setup_ui_headers
from omni.kit.menu.utils import MenuItemDescription, add_menu_items
from pxr import Sdf
from tmrobot.digital_robot.models.digital_camera import DigitalCamera
from tmrobot.digital_robot.models.digital_robot import DigitalRobot
from tmrobot.digital_robot.services.ethernet_master import EthernetMaster
from tmrobot.digital_robot.services.event_handler import EventHandler
from tmrobot.digital_robot.services.virtual_camera_server import VirtualCameraServer

current_script_dir = os.path.dirname(os.path.abspath(__file__))
env_path = os.path.join(current_script_dir, ".env")
load_dotenv(env_path)

VIRTUAL_CAMERA_SERVER_IP = os.getenv("VIRTUAL_CAMERA_SERVER_IP")
TMFLOW_01_IP = os.getenv("TMFLOW_01_IP")
DEVELOPER_MODE = os.getenv("DEVELOPER_MODE", "false").lower() in ("true", "True")
EXTENSION_NAME = "TM Digital Robot"
ACTION_INITIAL = "initial"
ACTION_START_SERVICE = "Start Service"
ACTION_STOP_SERVICE = "Stop Service"
ACTION_LOAD_WORLD = "Load World"
ACTION_DISABLE_ALL = "Disable All"
WORLD_CAMERA_PATH = "/World/world_camera"
CAMERA_RESOLUTION = (2592, 1944)
# CAMERA_RESOLUTION = (1280, 768)
# CAMERA_RESOLUTION = (800, 600)
SET_FOCAL_LENGTH = "Set Focal Length"
SET_IMG_SIZE = "Set Image Size"
GET_GRAB_IMG_DATA = "Get Grab Image Data"
SET_SHUTTER_TIME = "Set Shutter Time"
SET_GAIN = "Set Gain"
SET_WHITE_BALANCE = "Set White Balance"
PORT_ETHERNET = 5891


class TmrobotDigital_robotExtension(omni.ext.IExt):

    def on_startup(self, ext_id):

        print(f"DEVELOPER_MODE: {DEVELOPER_MODE}")
        print(f"TMFLOW_01_IP: {TMFLOW_01_IP}")
        print(f"VIRTUAL_CAMERA_SERVER_IP: {VIRTUAL_CAMERA_SERVER_IP}")

        self._world_settings = {
            "physics_dt": 1.0 / 1200.0,
            "rendering_dt": 1.0 / 60.0,
            "stage_units_in_meters": 1.0,
        }
        self._world = World(**self._world_settings)
        self._actions = {}
        self._dg_robots: List[DigitalRobot] = []
        self._dg_cameras: dict[str, list[DigitalCamera]] = {}  # key: tmflow ip
        self._set_queue = queue.Queue()
        self._motion_queue = queue.Queue()
        self._virtual_camera_thread: threading.Thread = None
        self._virtual_camera_server: VirtualCameraServer = None
        self._event_handler = EventHandler(self._dg_cameras)
        self._ext_id = ext_id
        self._tmflow_ips = [TMFLOW_01_IP]
        self._ethernet_masters: List[EthernetMaster] = None
        self._ethernet_master_threads: List[threading.Thread] = None

        self._menu_items = [
            MenuItemDescription(
                name="TMRobot",
                sub_menu=[
                    make_menu_item_description(
                        ext_id,
                        EXTENSION_NAME,
                        lambda a=weakref.proxy(self): a._menu_callback(),
                    )
                ],
            )
        ]
        add_menu_items(self._menu_items, "Isaac Utils")

        self._window = ScrollingWindow(
            title=EXTENSION_NAME,
            width=600,
            height=500,
            visible=False,
            dockPreference=ui.DockPreference.LEFT_BOTTOM,
        )
        self._window.set_visibility_changed_fn(self._on_window)

    def on_shutdown(self):

        if self._world.physics_callback_exists("sim_step"):
            self._world.remove_physics_callback("sim_step")

        try:
            self._virtual_camera_server.stop()
            self._stop_all_async_functions()

            for ethernet_master in self._ethernet_masters:
                ethernet_master.stop()

            for ethernet_master_thread in self._ethernet_master_threads:
                ethernet_master_thread.join()

        except Exception:
            print("Services stopped")

        omni.usd.get_context().new_stage()
        gc.collect()

    def _stop_all_async_functions(self):
        async def _stop_all_async_functions_async():
            tasks = [
                task
                for task in asyncio.all_tasks()
                if task is not asyncio.current_task()
            ]
            for task in tasks:
                task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)

        asyncio.ensure_future(_stop_all_async_functions_async())

    def _on_window(self, visible):
        if self._window.visible:
            if not self._window:
                self._window = ui.Window(
                    title=EXTENSION_NAME,
                    width=300,
                    height=0,
                    visible=True,
                    dockPreference=ui.DockPreference.LEFT_BOTTOM,
                )

        with self._window.frame:
            with ui.VStack(spacing=5, height=0):
                title = "TM Digital Robot"
                doc_link = "https://tobe.com"
                overview = "TM Digital Robot"
                setup_ui_headers(self._ext_id, __file__, title, doc_link, overview)
                self._build_world_control_frame()
                self._build_communication_frame()
                self._build_test_frame()

    def _build_communication_frame(self):
        world_control_frame = ui.CollapsableFrame(
            title="Communication",
            height=0,
            collapsed=False,
            style=get_style(),
            style_type_name_override="CollapsableFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )

        ui_settings = {
            f"{ACTION_START_SERVICE}": {
                "label": ACTION_START_SERVICE,
                "type": "button",
                "text": self._get_verb(ACTION_START_SERVICE),
                "tooltip": "",
                "on_clicked_fn": self._start_service,
            },
            f"{ACTION_STOP_SERVICE}": {
                "label": ACTION_STOP_SERVICE,
                "type": "button",
                "text": self._get_verb(ACTION_STOP_SERVICE),
                "tooltip": "",
                "on_clicked_fn": self._stop_service,
            },
        }

        # Robot Connection frame
        with world_control_frame:
            with ui.VStack(spacing=5, height=0):
                self._actions[ACTION_START_SERVICE] = btn_builder(
                    **(ui_settings[ACTION_START_SERVICE])
                )
                self._actions[ACTION_STOP_SERVICE] = btn_builder(
                    **(ui_settings[ACTION_STOP_SERVICE])
                )

    def _build_world_control_frame(self):
        world_control_frame = ui.CollapsableFrame(
            title="World Controls",
            height=0,
            collapsed=False,
            style=get_style(),
            style_type_name_override="CollapsableFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )

        ui_settings = {
            f"{ACTION_LOAD_WORLD}": {
                "label": ACTION_LOAD_WORLD,
                "type": "button",
                "text": self._get_verb(ACTION_LOAD_WORLD),
                "tooltip": "",
                "on_clicked_fn": self._load_world,
            }
        }

        with world_control_frame:
            with ui.VStack(spacing=5, height=0):
                self._actions[ACTION_LOAD_WORLD] = btn_builder(
                    **(ui_settings[ACTION_LOAD_WORLD])
                )

        self._change_action_mode(ACTION_INITIAL)

    def _build_test_frame(self):
        world_control_frame = ui.CollapsableFrame(
            title="Development Test",
            height=0,
            collapsed=False,
            style=get_style(),
            style_type_name_override="CollapsableFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )

        ui_settings = {
            f"{SET_FOCAL_LENGTH}": {
                "label": SET_FOCAL_LENGTH,
                "type": "button",
                "text": self._get_verb(SET_FOCAL_LENGTH),
                "tooltip": "",
                "on_clicked_fn": self._test_camera,
            },
            f"{SET_IMG_SIZE}": {
                "label": SET_IMG_SIZE,
                "type": "button",
                "text": self._get_verb(SET_IMG_SIZE),
                "tooltip": "",
                "on_clicked_fn": self._set_img_size,
            },
            f"{GET_GRAB_IMG_DATA}": {
                "label": GET_GRAB_IMG_DATA,
                "type": "button",
                "text": self._get_verb(GET_GRAB_IMG_DATA),
                "tooltip": "",
                "on_clicked_fn": self._get_grab_img_data,
            },
            f"{SET_SHUTTER_TIME}": {
                "label": SET_SHUTTER_TIME,
                "type": "button",
                "text": self._get_verb(SET_SHUTTER_TIME),
                "tooltip": "",
                "on_clicked_fn": self._set_shutter_time,
            },
            f"{SET_GAIN}": {
                "label": SET_GAIN,
                "type": "button",
                "text": self._get_verb(SET_GAIN),
                "tooltip": "",
                "on_clicked_fn": self._set_gain,
            },
            f"{SET_WHITE_BALANCE}": {
                "label": SET_WHITE_BALANCE,
                "type": "button",
                "text": self._get_verb(SET_WHITE_BALANCE),
                "tooltip": "",
                "on_clicked_fn": self._set_white_balance,
            },
        }

        with world_control_frame:
            with ui.VStack(spacing=5, height=0):
                self._actions[SET_FOCAL_LENGTH] = btn_builder(
                    **(ui_settings[SET_FOCAL_LENGTH])
                )
                self._actions[SET_IMG_SIZE] = btn_builder(**(ui_settings[SET_IMG_SIZE]))
                self._actions[GET_GRAB_IMG_DATA] = btn_builder(
                    **(ui_settings[GET_GRAB_IMG_DATA])
                )
                self._actions[SET_SHUTTER_TIME] = btn_builder(
                    **(ui_settings[SET_SHUTTER_TIME])
                )
                self._actions[SET_GAIN] = btn_builder(**(ui_settings[SET_GAIN]))
                self._actions[SET_WHITE_BALANCE] = btn_builder(
                    **(ui_settings[SET_WHITE_BALANCE])
                )

        self._change_action_mode(ACTION_LOAD_WORLD)

    def _change_action_mode(self, mode):
        if mode == ACTION_DISABLE_ALL:
            self._actions[ACTION_LOAD_WORLD].enabled = False
            self._actions[ACTION_START_SERVICE].enabled = False
            self._actions[ACTION_STOP_SERVICE].enabled = False

        if mode == ACTION_LOAD_WORLD:
            self._actions[ACTION_LOAD_WORLD].enabled = True
            self._actions[ACTION_START_SERVICE].enabled = False
            self._actions[ACTION_STOP_SERVICE].enabled = False

        if mode == ACTION_START_SERVICE:
            self._actions[ACTION_LOAD_WORLD].enabled = False
            self._actions[ACTION_START_SERVICE].enabled = True
            self._actions[ACTION_STOP_SERVICE].enabled = False

        if mode == ACTION_STOP_SERVICE:
            self._actions[ACTION_START_SERVICE].enabled = False
            self._actions[ACTION_STOP_SERVICE].enabled = True
            self._actions[ACTION_LOAD_WORLD].enabled = False
        pass

    def _get_verb(self, text):
        return text.split()[0]

    def _menu_callback(self):
        """
        Callback function for menu item, for displaying the extension window
        """
        self._window.visible = not self._window.visible
        return

    def _load_world(self):
        self._change_action_mode(ACTION_DISABLE_ALL)

        async def _load_world_async():

            current_file_path = os.path.abspath(__file__)
            current_directory = os.path.dirname(current_file_path)

            open_stage(f"{current_directory}/scenes/default/default_world.usd")
            omni.usd.get_context()

            self._dg_cameras[TMFLOW_01_IP] = []

            # Create a viewport camera
            world_camera = Camera(
                prim_path=WORLD_CAMERA_PATH,
                resolution=CAMERA_RESOLUTION,
                frequency=30,
            )

            set_camera_view(
                eye=np.array(
                    [2.4888289096734852, 2.3585157230177893, 2.0847313140453374]
                ),
                target=np.array([1.5, 1.5, 1.5]),
                camera_prim_path=WORLD_CAMERA_PATH,
            )

            rgb_annotator = rep.AnnotatorRegistry.get_annotator("rgb")
            self._replicator = rep.create.render_product(
                WORLD_CAMERA_PATH, resolution=CAMERA_RESOLUTION
            )
            rgb_annotator.attach(self._replicator)
            camera_serial_number = "world_camera"
            dg_camera = DigitalCamera(world_camera, rgb_annotator)
            dg_camera.set_serial_number(camera_serial_number)

            self._dg_cameras[TMFLOW_01_IP].append(dg_camera)

            # Create a digital robot
            robot_01_name = "robot_01"
            om_robot1 = Robot(name=robot_01_name, prim_path=f"/World/{robot_01_name}")
            dg_robot1 = DigitalRobot(
                TMFLOW_01_IP, robot_01_name, om_robot1, rgb_annotator
            )
            self._dg_robots.append(dg_robot1)
            self._world.scene.add(om_robot1)
            await self._world.initialize_simulation_context_async()
            await update_stage_async()

            # Create a eye-in-hand camera
            camera_serial_number = "robot_01_tm12_camera"
            robot_01_camera_01_path = f"/World/{robot_01_name}/tool0/Tm12_Camera"
            robot_01_camera_01 = Camera(
                prim_path=robot_01_camera_01_path,
                resolution=CAMERA_RESOLUTION,
                frequency=30,
            )

            robot_01_camera_01.initialize()  # Must be called before setting any properties
            robot_01_camera_01.set_clipping_range(0.01, 10000000.0)

            rgb_annotator = rep.AnnotatorRegistry.get_annotator("rgb")
            self._replicator = rep.create.render_product(
                robot_01_camera_01_path, resolution=CAMERA_RESOLUTION
            )
            rgb_annotator.attach(self._replicator)
            robot_01_camera_01 = DigitalCamera(world_camera, rgb_annotator)
            robot_01_camera_01.set_serial_number(camera_serial_number)

            self._dg_cameras[TMFLOW_01_IP].append(robot_01_camera_01)

            vp_utils.get_active_viewport().set_active_camera(WORLD_CAMERA_PATH)

            try:
                vp_utils.get_viewport_from_window_name("Viewport 2").set_active_camera(
                    robot_01_camera_01_path
                )
            except Exception:
                print("Viewport 2 is not open.")

            rep.orchestrator.run()

            self._change_action_mode(ACTION_START_SERVICE)

        asyncio.ensure_future(_load_world_async())

    def _is_service_on(self, ip, port):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            try:
                s.settimeout(1)  # Set the timeout value in seconds
                s.connect((ip, port))
                s.close()
                return True
            except socket.error as e:
                s.close()
                print(f"Server is not running. Exception: {e}")
                return False

    def _start_service(self):
        self._change_action_mode(ACTION_STOP_SERVICE)
        self._virtual_camera_server = VirtualCameraServer(
            self._set_queue, self._dg_cameras, DEVELOPER_MODE
        )
        # Check if ethernet slave is available
        for tmflow_ip in self._tmflow_ips:
            if not self._is_service_on(tmflow_ip, PORT_ETHERNET):
                print(f"Can't connect to Ethernet at {tmflow_ip}:{PORT_ETHERNET}")
                self._change_action_mode(ACTION_START_SERVICE)
                return

        # Create an async function to update camera settings from queue
        async def _set_cameras_async():
            while True:
                try:
                    message = self._set_queue.get_nowait()
                    await self._event_handler.set_camera(message)
                    await asyncio.sleep(0.001)
                except queue.Empty:
                    await asyncio.sleep(1)
                    # print("Waiting for message")

        asyncio.ensure_future(_set_cameras_async())

        # Create Virtual Camera Server
        asyncio.ensure_future(self._virtual_camera_server.start())

        # Create a thread for updating robot motion from ethernet master
        async def _ethernet_master_async():

            self._ethernet_masters = [EthernetMaster(ip) for ip in self._tmflow_ips]
            self._ethernet_master_threads = [
                threading.Thread(
                    target=ethernet_master.receive_data, args=(self._motion_queue,)
                )
                for ethernet_master in self._ethernet_masters
            ]

            for ethernet_master_thread in self._ethernet_master_threads:
                ethernet_master_thread.start()

        asyncio.ensure_future(_ethernet_master_async())

        # World: Ready to play
        async def _play_world_async():

            await self._world.reset_async()
            await self._world.pause_async()

            self._world.add_physics_callback(
                "sim_step", callback_fn=self._on_simulation_step
            )
            await self._world.play_async()

        asyncio.ensure_future(_play_world_async())

    def _on_simulation_step(self, step_size):

        try:
            # Attempt to get a message from the motion queue without blocking
            message = self._motion_queue.get_nowait()

            joint_radians = [math.radians(angle) for angle in message.joint_angle]
            # print(f"joint_radians: {joint_radians}")

            # if len(joint_radians) == 6:
            articulation_controller = self._dg_robots[0].get_articulation_controller()

            articulation_controller.apply_action(
                ArticulationAction(joint_positions=joint_radians)
            )
            # else:
            #     print("Error: Expected 6 joint angles, got {len(joint_radians)}")
        except queue.Empty:
            pass
            # print("Waiting for motion")
            # time.sleep(0.01)
        except Exception as e:
            print(f"An unexpected error occurred: {e}")

    def _stop_service(self):
        async def _stop_service_async():

            await self._world.stop_async()
            if self._world.physics_callback_exists("sim_step"):
                self._world.remove_physics_callback("sim_step")

            self._change_action_mode(ACTION_START_SERVICE)
            if hasattr(self, "_virtual_camera_server"):
                self._virtual_camera_server.stop()
            self._stop_all_async_functions()

            for ethernet_master in self._ethernet_masters:
                ethernet_master.stop()

            for ethernet_master_thread in self._ethernet_master_threads:
                ethernet_master_thread.join()

        asyncio.ensure_future(_stop_service_async())

    # For testing purposes
    def _test_camera(self):
        async def _test_camera_async():
            register_id = omni.kit.commands.register_callback(
                "ChangeProperty",
                omni.kit.commands.POST_DO_CALLBACK,
                self._post_processing,
            )

            for i in range(100):
                await asyncio.sleep(0.01)

                omni.kit.commands.execute(
                    "ChangeProperty",
                    prop_path=Sdf.Path("/World/robot_01/tool0/Tm12_Camera.focalLength"),
                    value=i,
                    prev=None,
                )

            omni.kit.commands.unregister_callback(register_id)

        asyncio.ensure_future(_test_camera_async())

    def _set_img_size(self, val=False):
        async def _set_img_size_async():
            print(f"CAMERA_RESOLUTION: {CAMERA_RESOLUTION}")

        asyncio.ensure_future(_set_img_size_async())

    def _get_grab_img_data(self, val=False):
        async def _get_grab_img_data_async():
            await asyncio.sleep(0.01)

            camera = Sdf.Path(WORLD_CAMERA_PATH)

            print(f"camera sdf: {camera}")
            rp = rep.create.render_product(
                camera, resolution=(CAMERA_RESOLUTION[0], CAMERA_RESOLUTION[1])
            )
            rgb = rep.AnnotatorRegistry.get_annotator("rgb")
            rgb.attach(rp)
            # await rep.orchestrator.step_async()
            rep.orchestrator.run()
            image_bytes = bytes(rgb.get_data())
            print(image_bytes)  # ((512, 1024, 4), uint8)

        asyncio.ensure_future(_get_grab_img_data_async())

    def _set_shutter_time(self, val=False):

        async def _set_shutter_time_async():
            await asyncio.sleep(0.01)
            value = random.randint(1, 150)  # 1-5000

            omni.kit.commands.execute(
                "ChangeSetting", path="/rtx/post/tonemap/cameraShutter", value=value
            )

        asyncio.ensure_future(_set_shutter_time_async())

    def _post_processing(self, val):
        print("callback_val: ", val)

    def _set_gain(self):
        async def _set_gain_async():

            register_id = omni.kit.commands.register_callback(
                "ChangeSetting",
                omni.kit.commands.POST_DO_CALLBACK,
                self._post_processing,
            )

            omni.kit.commands.execute(
                "ChangeSetting",
                path="/rtx/post/tonemap/filmIso",
                value=88,
                prev=None,
            )

            omni.kit.commands.unregister_callback(register_id)

        asyncio.ensure_future(_set_gain_async())

    def _set_white_balance(self, val=False):
        async def _set_white_balance_async():
            await asyncio.sleep(0.01)
            r, g, b = [random.randint(1, 10) / 10 for _ in range(3)]
            omni.kit.commands.execute(
                "ChangeSetting", path="/rtx/post/tonemap/whitepoint", value=[r, g, b]
            )
            # R: [0-1], G: [0-1] , B: [0-1]

        asyncio.ensure_future(_set_white_balance_async())
