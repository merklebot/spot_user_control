#!/usr/bin/env python3

from transitions import Machine
import shelve
import robonomicsinterface as RI
from lease_control.session_data import SessionData
from lease_control.user_control import UserControl
import os

SUBSTRATE_MNEMONIC = os.environ["MNEMONIC"]

class Session:
    def __init__(self, session_id: int, user_email: str = None, key: str = None, lesson: int = None) -> None:
        self.session_id = str(session_id)
        self.extrinsic_hash = ""
        self.ipfs_hash = ""
        self.user_email = user_email
        self.key = key
        self.lesson = lesson
        self.session_data_path = "/home/spot/session_data"
        self.session_data = SessionData(ipfs_hash="", extrinsic_hash="", state="")
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
            {"trigger": "send_logs_to_ipfs", "source": "started", "dest": "ipfs"},
            {"trigger": "send_hash_to_blockchain", "source": "ipfs", "dest": "blockchain"},
            {"trigger": "finish_session", "source": "blockchain", "dest": "finished"}
        ]
        self.machine = Machine(model=self, states=states, transitions=transitions, initial=initial_state, after_state_change="change_state_in_database")
        self.machine.add_ordered_transitions(loop=False)

    def read_state(self) -> str:
        """
        Read current state from database

        Returns
        -------
        state: str
            name of current state for this session
        """
        with shelve.open(self.session_data_path) as db:
            if self.session_id in db:
                self.ipfs_hash = db[self.session_id].ipfs_hash
                self.extrinsic_hash = db[self.session_id].extrinsic_hash
                return db[self.session_id].state
            else:
                db[self.session_id] = self.session_data
                return "before_start"

    def change_state_in_database(self) -> None:
        """
        Write current state to database
        """
        print(f"State: {self.state}")
        with shelve.open(self.session_data_path) as db:
            data = db[self.session_id]
            data.state = self.state
            db[self.session_id] = data

    def create_user(self) -> None:
        """
        Create Spot and core users and allow moving
        """
        print(f"Create user: {self.state}")
        if self.key is None or self.lesson is None:
            raise CreateUserError("Key, lesson or email was not provided")
        passw = self.uc.create_user_pass()
        passw1 = self.uc.create_user_pass()
        user = "student"
        self.uc.add_user_core(user, passw1, keys=self.key, lesson=self.lesson, emails=self.user_email)
        self.uc.add_user_spot(user, passw)
        self.uc.estop_pub.publish("press allow")

    def delete_user(self) -> None:
        """
        Delete Spot and core users and forbid moving
        """
        print(f"Delete user: {self.state}")
        user = "student"
        self.uc.delete_user_core(user)
        self.uc.delete_user_spot(user)
        self.uc.estop_pub.publish("press stop")

    def send_to_ipfs(self) -> None:
        """
        Compress logs and send them to IPFS
        Save hash to database
        """
        print(f"Send to IPFS: {self.state}")
        user = "student"
        self.uc.compress_files(f"/home/spot/{user}/")
        self.ipfs_hash = self.uc.pin_to_ipfs(f"/home/spot/{user}/")
        ### Save hash to database
        with shelve.open(self.session_data_path) as db:
            data = db[self.session_id]
            data.ipfs_hash = self.ipfs_hash
            db[self.session_id] = data

    def send_datalog(self) -> None:
        """
        Read hash from database
        Send datalog with hash
        Write extrinsic hash to database
        """
        print(f"Send datalog: {self.state}")
        if self.ipfs_hash == "":
            with shelve.open(self.session_data_path) as db:
                self.ipfs_hash = db[self.session_id].ipfs_hash

        interface = RI.RobonomicsInterface(seed=SUBSTRATE_MNEMONIC)
        self.extrinsic_hash = interface.record_datalog(self.ipfs_hash)

        ### Save extrinsic hash to database
        with shelve.open(self.session_data_path) as db:
            data = db[self.session_id]
            data.extrinsic_hash = self.extrinsic_hash
            db[self.session_id] = data

        print(f"DataLog Extrinsic Hash: {self.extrinsic_hash}")
