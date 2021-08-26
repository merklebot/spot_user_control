#!/usr/bin/env python3

from substrateinterface import SubstrateInterface, Keypair
from bosdyn.mission.client import MissionClient
import bosdyn.client
import os
import time
# import psutil

class RobonomicsLogSender:
    def __init__(self):
        self.substrate = SubstrateInterface(
                    url="wss://kusama.rpc.robonomics.network",
                    ss58_format=32,
                    type_registry_preset="substrate-node-template",
                    type_registry={
                        "types": {
                            "Record": "Vec<u8>",
                            "<T as frame_system::Config>::AccountId": "AccountId",
                            "RingBufferItem": {
                                "type": "struct",
                                "type_mapping": [
                                    ["timestamp", "Compact<u64>"],
                                    ["payload", "Vec<u8>"],
                                ],
                            },
                        }
                    }
                )
        mnemonic = os.environ['ROBONOMICS_MNEMONIC_SEED']
        spot_password = os.environ['SPOT_PASSWORD']
        spot_username = os.environ['SPOT_USERNAME']
        sdk = bosdyn.client.create_standard_sdk('robonomics_sender', [MissionClient])
        robot = sdk.create_robot('192.168.50.3')
        robot.authenticate(spot_username, spot_password)
        self.state_client = robot.ensure_client('robot-state')
        self.keypair = Keypair.create_from_mnemonic(mnemonic, ss58_format=32)

    def write_datalog(self, data):
        call = self.substrate.compose_call(
            call_module="Datalog",
            call_function="record",
            call_params={
                'record': data
            }
        )
        extrinsic = self.substrate.create_signed_extrinsic(call=call, keypair=self.keypair)
        receipt = self.substrate.submit_extrinsic(extrinsic, wait_for_inclusion=True)
        print(f"Datalog created with extrinsic hash: {receipt.extrinsic_hash}")

    def spin(self):
        while True:
            data = self.state_client.get_robot_state()
            text = data.battery_states[0]
            self.write_datalog(text)
            time.sleep(12)

if __name__ == '__main__':
    RobonomicsLogSender().spin()