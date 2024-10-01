####################################################################################################################
# TECHMAN ROBOT INC. ("Techman") CONFIDENTIAL                                                                      #
# Copyright (C) 2016~2024 Techman. All Rights Reserved.                                                            #
# This source code and any compilation or derivative thereof is proprietary and confidential to Techman.           #
# Under no circumstances may any portion of the source code or any modified version of the source code be          #
# distributed, disclosed, or otherwise made available to any third party without the express written consent of    #
# Techman.                                                                                                         #
####################################################################################################################
import asyncio
import re
import socket


class EthernetData:
    def __init__(self, data_dict):
        # self.robot_link = data_dict.get('$TMSVR,317,3,2,Robot_Link', [0.0])
        self.joint_angle = data_dict.get("Joint_Angle", [0.0] * 6)
        self.ctrl_do = data_dict.get("Ctrl_DO", [0] * 16)
        self.ctrl_di = data_dict.get("Ctrl_DI", [0] * 16)
        self.ctrl_ao = data_dict.get("Ctrl_AO", [0.0] * 2)
        self.ctrl_ai = data_dict.get("Ctrl_AI", [0.0] * 2)
        self.end_do = data_dict.get("End_DO", [0] * 3)
        self.end_di = data_dict.get("End_DI", [0] * 3)
        self.end_ai = data_dict.get("End_AI", [0.0] * 2)


class EthernetMaster:
    def __init__(self, tmflow_ip):
        self.tmflow_ip = tmflow_ip
        self.port = 5891
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.buffer = ""
        self.running = True
        self.data = EthernetData({})
        self.start()

    def start(self):
        self.client.connect((self.tmflow_ip, self.port))

    def receive_data(self, motion_queue: asyncio.Queue):
        while self.running:
            data = self.client.recv(1024)
            if data:
                self.buffer += data.decode()
                if "$TMSVR" in self.buffer:
                    data_str = self.buffer.split("$TMSVR")
                    for data in data_str[1:]:
                        # timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                        # print(f"{timestamp}")
                        # print(f"Raw message: {data}")
                        ethernet_data = self.parse_data(f"$TMSVR{data}")
                        motion_queue.put(ethernet_data)

                    self.buffer = data_str[0]

    def parse_data(self, data):
        lines = data.split("\n")
        data_parsed = {}
        for line in lines:
            if "=" in line:
                key, value = line.split("=")
                value = re.findall(r"[-+]?\d*\.\d+|\d+", value)
                if "DI" in key or "DO" in key:
                    value = [int(v) for v in value]
                else:
                    value = [float(v) for v in value]
                data_parsed[key] = value

            # print(json.dumps(data_parsed, indent=2))

        try:
            self.data = EthernetData(data_parsed)
            return self.data  # Return the new data
        except Exception as e:
            print(f"Error parsing message: {e}")
            # return self.data  # Return the last known data

    def stop(self):
        self.running = False
        self.client.close()
