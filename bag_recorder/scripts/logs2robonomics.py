#!/usr/bin/env python3

import robonomicsinterface as RI
from bosdyn.mission.client import MissionClient
import bosdyn.client
import os
import time
# import psutil

class RobonomicsLogSender:
    def __init__(self):
        mnemonic = os.environ['ROBONOMICS_MNEMONIC_SEED']
        spot_password = os.environ['SPOT_PASSWORD']
        spot_username = os.environ['SPOT_USERNAME']
        sdk = bosdyn.client.create_standard_sdk('robonomics_sender', [MissionClient])
        robot = sdk.create_robot('192.168.50.3')
        robot.authenticate(spot_username, spot_password)
        self.state_client = robot.ensure_client('robot-state')
        self.interface = RI.RobonomicsInterface(seed=mnemonic)

    def spin(self):
        i = 0
        while True:
            i += 1
            data = self.state_client.get_robot_state()
            text = str(data.battery_states[0])
            extrinsic_hash = self.interface.record_datalog(text)
            print(f"Datalog created with extrinsic hash: {extrinsic_hash}")
            time.sleep(12)
            data = self.state_client.get_robot_state()
            if i >= len(data.system_fault_state.faults):
                i = 0
            if len(data.system_fault_state.faults) != 0:
                text = str(data.system_fault_state.faults[i])
                extrinsic_hash = self.interface.record_datalog(text)
                print(f"Datalog created with extrinsic hash: {extrinsic_hash}")
                time.sleep(12)


if __name__ == '__main__':
    RobonomicsLogSender().spin()
