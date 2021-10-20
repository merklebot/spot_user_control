#!/usr/bin/env python3

import time
import bosdyn.client
from bosdyn.client.robot_command import RobotCommandClient, blocking_stand
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn.geometry import EulerZXY

def init_spot():
    sdk = bosdyn.client.create_standard_sdk('understanding-spot')
    robot = sdk.create_robot('192.168.50.3')
    robot.authenticate('alena', 'hdGn76CdkBv6CB3')
    lease_client = robot.ensure_client('lease')
    lease = lease_client.acquire()
    lease_keep_alive = bosdyn.client.lease.LeaseKeepAlive(lease_client)
    robot.time_sync.wait_for_sync()
    robot.power_on(timeout_sec=20)
    command_client = robot.ensure_client(RobotCommandClient.default_service_name)
    blocking_stand(command_client, timeout_sec=10)
    return command_client

def change_position(command_client, yaw, roll, pitch):
    footprint_R_body = EulerZXY(yaw=yaw, roll=roll, pitch=pitch)
    cmd = RobotCommandBuilder.stand_command(footprint_R_body=footprint_R_body)
    command_client.robot_command(cmd)

if __name__ == '__main__':
    command_client = init_spot()
    while True:
        print("Enter yaw, roll and pitch angles separated by commas (angles must be from -0.5 to 0.5)")
        print("For example: 0.3, 0.1, 0.2")
        line = input()
        line = line.split(',')
        yaw = line[0].strip()
        roll = line[1].strip()
        pitch = line[2].strip()
        print(f"yaw = {yaw}, roll = {roll}, pitch = {pitch}")
        change_position(command_client, yaw, roll, pitch)
        time.sleep(2)

