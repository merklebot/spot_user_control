#!/usr/bin/env python3
import time
import bosdyn.client


sdk = bosdyn.client.create_standard_sdk('wait')
robot = sdk.create_robot('192.168.50.3')
id_client = robot.ensure_client('robot-id')

def connect(client):
    try:
        print("Try to connect")
        client.get_id()
    except Exception as e:
        print(f" Exception: {e}")
        print("Waiting for Spot")
        time.sleep(5)
        connect(client)

connect(id_client)