#!/usr/bin/env python3

from transitions import Machine
from user_control import UserControl
from blockchain import DataLogger, create_substrate_interface
import os

SUBSTRATE_MNEMONIC = os.environ["MNEMONIC"]
SUBSTRATE_URL = "wss://main.frontier.rpc.robonomics.network"

class Session:

    def __init__(self, session_id, user_email, key=None, lesson=None):
        self.session_id = session_id
        self.user_email = user_email
        self.key = key
        self.lesson = lesson
        self.uc = UserControl()
        states = [
            {"name": "before_start"},
            {"name": "started", "on_enter": ["create_user"], "on_exit": ["delete_user"]},
            {"name": "ipfs", "on_exit": ["send_to_ipfs"]},
            {"name": "blockchain", "on_exit": ["send_datalog"]},
            {"name": "finished"}
        ]
        initial_state = self.read_state()
        transitions = [
            {"trigger": "start_session", "source": "before_start", "dest": "started"},
            {"trigger": "send_to_ipfs", "source": "started", "dest": "ipfs"},
            {"trigger": "send_hash_to_blockchain", "source": "ipfs", "dest": "blockchain"},
            {"trigger": "finish_session", "source": "blockchain", "dest": "finished"}
        ]
        self.machine = Machine(model=self, states=states, transitions=transitions, initial=initial_state, after_state_change="change_state_in_database")

    def read_state(self) -> str:
        """
        Read current state from database

        Returns
        -------
        state: str
            name of current state for this session
        """
        return "before_start"

    def change_state_in_database(self):
        """
        Write current state to database
        """
        print(f"State: {self.state}")

    def create_user(self):
        """
        Create Spot and core users and allow moving
        """
        print(f"Create user: {self.state}")
        if self.key is None or self.lesson is None:
            raise CreateUserError("Key, lesson or email was not provided")
        passw = self.uc.create_user_pass()
        passw1 = self.uc.create_user_pass()
        user = "student"
        self.uc.add_user_core(user, passw1, key=self.key, lesson=self.lesson, email=self.user_email)
        self.uc.add_user_spot(user, passw)
        self.uc.estop_pub.publish("press allow")

    def delete_user(self):
        """
        Delete Spot and core users and forbid moving
        """
        user = "student"
        self.uc.delete_user_core(user)
        self.uc.delete_user_spot(user)
        self.uc.estop_pub.publish("press stop")

    def send_to_ipfs(self):
        """
        Compress logs and send them to IPFS
        Save hash to database
        """
        user = "student"
        self.uc.compress_files(f"/home/spot/{user}/")
        ipfs_hash = self.uc.pin_to_ipfs(f"/home/spot/{user}/")
        ### Save hash to database

    def send_datalog(self):
        """
        Read hash from database
        Send datalog with hash
        Write extrinsic hash to database
        """
        ### Read ipfs hash
        ipfs_hash = None
        ###

        substrate_interface = create_substrate_interface(SUBSTRATE_URL)
        datalogger = DataLogger(SUBSTRATE_MNEMONIC, substrate_interface)
        extrinsic_hash = datalogger.write(ipfs_hash)

        ### Save extrinsic hash to database

        print(f"DataLog Extrinsic Hash: {extrinsic_hash}")




