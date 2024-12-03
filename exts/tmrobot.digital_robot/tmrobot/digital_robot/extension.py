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
import logging
import queue
import random
import socket
import threading
import traceback
from datetime import datetime, timezone
from typing import List

import carb
import numpy as np
import omni.ext
import omni.kit.commands
import omni.kit.viewport.utility as vp_utils
import omni.usd
import omni.usd.audio
from omni.isaac.core.utils.stage import (
    add_reference_to_stage,
    clear_stage,
    close_stage,
    create_new_stage,
    is_stage_loading,
    update_stage_async,
)
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.world.world import World
from omni.isaac.surface_gripper._surface_gripper import (
    Surface_Gripper,
    Surface_Gripper_Properties,
)
from pxr import Gf, Sdf, UsdGeom
from tmrobot.digital_robot.models.digital_camera import DigitalCamera
from tmrobot.digital_robot.models.digital_robot import DigitalRobot
from tmrobot.digital_robot.models.setting import ExtensionSetting, RobotSetting
from tmrobot.digital_robot.services.echo_client import EchoClient
from tmrobot.digital_robot.services.ethernet_master import EthernetData, EthernetMaster
from tmrobot.digital_robot.services.virtual_camera_server import VirtualCameraServer
from tmrobot.digital_robot.ui import constants as const
from tmrobot.digital_robot.ui.extension_ui import ExtensionUI

logger = logging.getLogger(__name__)

# Get the active viewport
viewport = vp_utils.get_active_viewport()
# viewport.set_texture_resolution((1920, 1080))
viewport.set_texture_resolution((1280, 720))


class TmrobotDigital_robotExtension(omni.ext.IExt):

    def initialize(self):
        # fmt: off
        carb.settings.get_settings().set("/rtx/pathtracing/maxSamplesPerLaunch", 892778)
        self._console(f"DEVELOPER_MODE: {const.DEVELOPER_MODE}")
        self._extension_setting = ExtensionSetting()
        self._models = {}
        self._virtual_camera_thread: threading.Thread = None
        self._virtual_camera_server: VirtualCameraServer = None
        self._dg_robots: dict[str, DigitalRobot] = {}
        self._dg_cameras: dict[str, dict[str, DigitalCamera]] = {}  # [tmflow ip][camera name]
        self._ethernet_masters: dict[str, EthernetMaster] = {}  # [robot name]
        self._ethernet_master_threads: dict[str, threading.Thread] = {}  # [robot name]
        self._motion_queue: queue.Queue = None
        self._robot_settings: List[RobotSetting] = []
        self._set_queue = queue.Queue()
        self._simulation_count = 0
        self._receive_count: dict[str, int] = {}
        self._receive_count[const.ROBOT_LIST[0]] = 0
        self._receive_count[const.ROBOT_LIST[1]] = 0
        self._receive_count[const.ROBOT_LIST[2]] = 0
        self._receive_count[const.ROBOT_LIST[3]] = 0
        self._fps_accumulated = 0
        self._gripper_state = 0
        self._gripper = None
        self._workpiece_id = 0
        self._world_settings = {
            "physics_dt": 1.0 / 60.0,
            "rendering_dt": 1.0 / 60.0,
            "stage_units_in_meters": 1.0,
        }
        self._world: World = World(**self._world_settings)
        # self._world: World = World()
        self._default_workpiece_position = Gf.Vec3d(0, 0.25, 0.5155)
        self._default_workpieces_prim_path = "/World/Accessories/Workpieces"
        # fmt: on

    def on_startup(self, ext_id):
        self._ext_id = ext_id
        self._ext_ui = ExtensionUI(
            self._ext_id, self._on_start_service, self._on_stop_service
        )

        self.initialize()

    def on_shutdown(self):

        if self._world.stage.GetPrimAtPath(Sdf.Path("/World")).IsValid():
            for robot in const.ROBOT_LIST:
                if self._world.scene.object_exists(robot):
                    self._world.scene.remove_object(robot)

        if len(self._ethernet_masters) != 0:
            for robot in self._robot_settings:
                self._ethernet_masters[robot.name].stop()
                self._ethernet_master_threads[robot.name].join(timeout=0)

        if self._world.physics_callback_exists("sim_step"):
            self._world.remove_physics_callback("sim_step")

        try:

            if self._virtual_camera_server is not None:
                self._virtual_camera_server.stop()
                self._stop_all_async_functions()

            self._console("Services stopped")
        except Exception as e:
            logger.error(f"Services stopped with error: {e}")

        for handler in logger.handlers[:]:
            logger.removeHandler(handler)
            handler.close()

        logger.handlers.clear()

        if self._world is not None:
            self._world.stop()
            self._world.clear_all_callbacks()
            self._current_tasks = None

        self._robot_settings = []
        self._ext_ui.clear()

        if is_stage_loading():
            clear_stage()
            close_stage()

        create_new_stage()
        gc.collect()
        return

    def _on_start_service(self):
        self.initialize()

        self._ext_ui.change_action_mode(const.BUTTON_STOP_SERVICE)
        self._ext_ui.enabled_robot_settings(False)

        self._ext_ui._on_save_scene()
        audio = omni.usd.audio.get_stage_audio_interface()
        audio._get_invalid_streamer_id()
        audio.stop_all_sounds()
        self._simulation_count = 0
        # self._motion_queue.queue.clear()

        if self._world.stage.GetPrimAtPath(
            Sdf.Path(self._default_workpieces_prim_path)
        ).IsValid():
            # Known issue: Get warning message below when remove prim
            # ... delegate.cpp -- Failed verification: ' prim '
            self._world.stage.RemovePrim(self._default_workpieces_prim_path)

        self._robot_settings = self._get_activated_robots_setting()
        # Check if TMFlow services are available
        for setting in self._robot_settings:
            self._console(f"Add Robot {setting.name} to the scene")

            if setting.ip == "":
                logger.error(f"Robot {setting.name} does not have an IP address")
                self._ext_ui.change_action_mode(const.BUTTON_START_SERVICE)
                return

            # Check if the status of TMflow Virtual Camera API is Activated
            echo_client = EchoClient(setting.ip)
            if not echo_client.connectTMFlow():
                logger.error(
                    f"Can't connect to {setting.name} TMFlow at {setting.ip}, "
                    "please check if the status of TMflow Virtual Camera API is Activated"
                )
                self._ext_ui.change_action_mode(const.BUTTON_START_SERVICE)
                return

            # Check if the status of TMflow Ethernet Slave is Activated
            if not self._is_service_on(setting.ip, const.PORT_ETHERNET):
                logger.error(
                    f"Can't connect to {setting.name} Ethernet at {setting.ip}:{const.PORT_ETHERNET}, "
                    "please check if the status of TMflow Ethernet Slave is Activated"
                )
                self._ext_ui.change_action_mode(const.BUTTON_START_SERVICE)
                return

            # Create Digital Robots
            try:
                self._dg_robots[setting.name] = DigitalRobot(setting, self._world.stage)
                self._dg_cameras[setting.ip] = {}

                # Create a Camera list
                camera_list = self._dg_robots[setting.name].get_activated_cameras()
                for camera in camera_list:
                    self._dg_cameras[setting.ip][camera.get_serial_number()] = camera

                if len(camera_list) == 0:
                    self._motion_queue = queue.Queue(maxsize=4)
                else:
                    self._motion_queue = queue.Queue(maxsize=1)

                if not self._world.scene.object_exists(setting.name):
                    self._world.scene.add(self._dg_robots[setting.name].get_robot())

            except Exception as e:
                logger.error(f"Failed to add robot {setting.name}: {e}")
                logger.error(traceback.format_exc())
                self._ext_ui.change_action_mode(const.BUTTON_START_SERVICE)
                return

            # Create Gripper
            if self._is_prim_exist(self._default_workpieces_prim_path):
                self._world.stage.RemovePrim(
                    Sdf.Path(self._default_workpieces_prim_path)
                )
                omni.kit.commands.execute(
                    "DeletePrims",
                    paths=[self._default_workpieces_prim_path],
                    destructive=False,
                )

            omni.kit.commands.execute(
                "CreatePrimWithDefaultXform",
                prim_type="Xform",
                prim_path=self._default_workpieces_prim_path,
                attributes={},
                select_new_prim=True,
            )

            # === Uncomment the code below to control the gripper ===
            # The example is only for the first robot Robot01
            if self._robot_settings[0].name == const.ROBOT_LIST[0]:
                sgp = Surface_Gripper_Properties()
                sgp.parentPath = f"/World/{self._robot_settings[0].name}/{self._robot_settings[0].model.lower()}/body/flange_link"  # noqa
                sgp.offset.p.x = 0
                sgp.offset.p.z = 0.337
                sgp.offset.r = [0.7071, 0, 0.7071, 0]
                sgp.gripThreshold = 0.005
                sgp.forceLimit = 1.0e2
                sgp.torqueLimit = 1.0e3
                sgp.bendAngle = np.pi / 4
                sgp.stiffness = 1.0e4
                sgp.damping = 1.0e3
                sgp.retryClose = True
                self._gripper = Surface_Gripper()
                self._gripper.initialize(sgp)

                if self._is_prim_exist("/World/Accessories/sugar_box"):
                    omni.kit.commands.execute(
                        "ToggleActivePrims",
                        stage_or_context=omni.usd.get_context().get_stage(),
                        prim_paths=[Sdf.Path("/World/Accessories/sugar_box")],
                        active=False,
                    )
                self._spawn_workpiece()

        # Play the world
        async def _play_world_async():
            await self._world.initialize_simulation_context_async()

            self._world.add_physics_callback(
                "sim_step", callback_fn=self._on_simulation_step
            )
            await self._world.reset_async()
            await update_stage_async()
            await self._world.play_async()

        # Create a ethernet master threads for updating robot motion from ethernet slave
        async def _ethernet_master_async():

            for robot in self._robot_settings:
                self._ethernet_masters[robot.name] = EthernetMaster(
                    robot.name, robot.ip
                )
                self._ethernet_master_threads[robot.name] = threading.Thread(
                    target=self._ethernet_masters[robot.name].receive_data,
                    args=(self._motion_queue,),
                )

                self._ethernet_master_threads[robot.name].start()

        # Create Virtual Camera gRPC Server
        self._virtual_camera_server = VirtualCameraServer(
            self._set_queue, self._dg_cameras
        )

        asyncio.ensure_future(self._virtual_camera_server.start())
        asyncio.ensure_future(_ethernet_master_async())
        asyncio.ensure_future(_play_world_async())
        omni.kit.commands.execute("SelectNone")
        self._ext_ui.change_action_mode(const.BUTTON_STOP_SERVICE)

    def _on_simulation_step(self, step_size):

        try:

            motion: EthernetData = self._motion_queue.get_nowait()
            self._dg_robots[motion.robot_name].apply_action(
                ArticulationAction(joint_positions=motion.joint_radian)
            )

            # === Uncomment the code below to control the gripper ===
            if motion.robot_name == const.ROBOT_LIST[0]:
                if self._gripper_state != motion.ctrl_do[0]:
                    self._gripper_state = motion.ctrl_do[0]
                    if self._gripper_state == 1:
                        self._gripper.close()
                        self._console("Gripper Close")

                    if self._gripper_state == 0:
                        self._gripper.open()
                        self._console("Gripper Open")

                        self._spawn_workpiece()

            # === Uncomment the code below to test the digital input and output ===
            # if self._simulation_count % 300 == 0:
            #     if motion.ctrl_di[3] == 0 and motion.end_di[3] == 0:
            #         self._ethernet_masters[motion.robot_name].set_ctrl_di(3, 1)
            #         self._ethernet_masters[motion.robot_name].set_end_di(3, 1)
            #     else:
            #         self._ethernet_masters[motion.robot_name].set_ctrl_di(3, 0)
            #         self._ethernet_masters[motion.robot_name].set_end_di(3, 0)

            # === Uncomment the code below to show the FPS ===
            # self._simulation_count += 1
            # self._fps_accumulated += viewport.fps
            # avg_fps = "{:.2f}".format(self._fps_accumulated / self._simulation_count)
            # if self._simulation_count % 100 == 0:
            #     self._console(f"Avg. FPS: {avg_fps} - {self._simulation_count}")

            # === Uncomment the code below to trace FPS ===
            # self._receive_count[motion.robot_name] = motion.receive_count
            # total_receive_count = sum(self._receive_count.values())
            # self._simulation_count += 1
            # print(
            #     f"rec/sim/qsize/fps :{total_receive_count}/{self._simulation_count}/{self._motion_queue.qsize()}/{viewport.fps:.2f}"  # noqa
            # )

        except queue.Empty:
            pass
        except Exception as e:
            logger.warning(f"Failed to update robot motion: {e}, {motion}")

    def _on_stop_service(self):
        async def _on_stop_service_async():
            self._ext_ui.change_action_mode(const.BUTTON_DISABLE_ALL)
            self._ext_ui.enabled_robot_settings(True)

            await self._world.stop_async()

            if self._world.stage.GetPrimAtPath(
                Sdf.Path(self._default_workpieces_prim_path)
            ).IsValid():
                # Known issue: Get warning message below when remove prim
                # ... delegate.cpp -- Failed verification: ' prim '
                self._world.stage.RemovePrim(self._default_workpieces_prim_path)

            for robot in self._robot_settings:
                self._ethernet_masters[robot.name].stop()
                self._world.scene.remove_object(robot.name)
                self._ethernet_master_threads[robot.name].join(timeout=0)
            #     self._dg_robots[setting.name].detach_all_annotators()

            if self._world.physics_callback_exists("sim_step"):
                self._world.remove_physics_callback("sim_step")

            if hasattr(self, "_virtual_camera_server"):
                self._virtual_camera_server.stop()
            self._stop_all_async_functions()
            self._ext_ui.change_action_mode(const.BUTTON_START_SERVICE)
            self._console("Services stopped")

            print(
                self._world.stage.GetPrimAtPath(
                    Sdf.Path(self._default_workpieces_prim_path)
                ).IsValid()
            )

        asyncio.ensure_future(_on_stop_service_async())

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

    def _console(self, message):
        current_time = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%S")

        print(f"{current_time} [Info] [tmrobot.digital_robot] {message}")
        logger.info(message)

    def _get_activated_robots_setting(self) -> List[RobotSetting]:
        active_robots = []
        for robot_name in const.ROBOT_LIST:
            setting = self._ext_ui._on_load_setting()
            if setting.robots_setting[robot_name].activated:
                active_robots.append(setting.robots_setting[robot_name])
        return active_robots

    def _is_service_on(self, ip, port):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            try:
                s.settimeout(1)  # Set the timeout value in seconds
                s.connect((ip, port))
                s.close()
                return True
            except socket.error as e:
                s.close()
                logger.error(f"{ip}:{port} is not available. exception: {e}")
                return False

    def _is_prim_exist(self, prim_path: str) -> bool:
        prim = self._world.stage.GetPrimAtPath(Sdf.Path(prim_path))
        return prim.IsValid()

    def _spawn_workpiece(self):

        workpieces_prim = self._world.stage.GetPrimAtPath(
            Sdf.Path("/World/Accessories/Workpieces")
        )
        spawn_position = self._default_workpiece_position
        for workpiece in workpieces_prim.GetChildren():
            path: str = workpiece.GetPath()
            wp_prim = self._world.stage.GetPrimAtPath(Sdf.Path(path))
            wp_xformable = UsdGeom.Xformable(wp_prim)
            wp_transform = wp_xformable.GetLocalTransformation()
            wp_translation = wp_transform.ExtractTranslation()
            x = round(wp_translation[0], 2)
            y = round(wp_translation[1], 2)
            # print(f"Workpiece {workpiece.GetName()} is at {wp_translation}")
            # print(f"Workpiece {x, y} is at {wp_translation}")
            if spawn_position[0] == x and spawn_position[1] == y:
                # print(f"Workpiece {workpiece.GetName()} is at the spawn position")
                return

        self._workpiece_id += 1
        # fmt: off
        workpiece_prim_path = f"/World/Accessories/Workpieces/workpiece_{self._workpiece_id}"
        # fmt: on

        absolute_path = f"{const.EXTENSION_ROOT_PATH}/assets/worlds/accessories/workpiece/004_sugar_box/004_sugar_box.usd" # noqa

        workpiece_prim = add_reference_to_stage(
            usd_path=absolute_path,
            prim_path=workpiece_prim_path,
        )

        # workpiece = self._world.stage.GetPrimAtPath(Sdf.Path(workpiece_prim_path))

        workpiece_xform = UsdGeom.XformCommonAPI(UsdGeom.Xformable(workpiece_prim))
        # workpiece_xform = UsdGeom.Xformable(workpiece_prim)
        workpiece_xform.SetTranslate(self._default_workpiece_position)
        workpiece_xform.SetScale((0.5, 0.5, 0.5))
        workpiece_xform.SetRotate(Gf.Vec3f(0, 0, random.uniform(0, 360)))

        # Can be removed
        # self._world.scene.add(
        #     XFormPrim(
        #         prim_path=workpiece_prim_path,
        #         name=f"workpiece_{self._workpiece_id}",
        #     )
        # )

        # isValidPathString = Sdf.Path.IsValidPathString(workpiece_prim_path)
        # print(isValidPathString)

        # Known issue: Get warning message below when set rigid body
        # path.cpp -- Ill-formed SdfPath <>: syntax error
        omni.kit.commands.execute(
            "SetRigidBody",
            path=Sdf.Path(workpiece_prim_path),
            approximationShape="convexHull",
            kinematic=False,
        )

        self._console(f"{workpiece_prim_path} is spawned")
