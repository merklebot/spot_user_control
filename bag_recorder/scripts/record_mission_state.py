#!/usr/bin/env python3
import rospy
import time
import bosdyn.client
import rosnode
import subprocess
from pinatapy import PinataPy
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import InteractiveMarkerUpdate
from bosdyn.mission.client import MissionClient
import os
import re


class MissionStateRecorder():
    def __init__(self):
        rospy.init_node(f"mission_state_recorder", anonymous=False)
        rospy.Subscriber("/start_lesson", String, self.start_lesson_callback)
        username = rospy.get_param("~username")
        password = rospy.get_param("~password")
        hostname = rospy.get_param("~hostname")
        sdk = bosdyn.client.create_standard_sdk('mission_state_recorging', [MissionClient])
        self.robot = sdk.create_robot(hostname)
        self.robot.authenticate(username, password)
        self.misson_client = self.robot.ensure_client('robot-mission')
        self.recording = False
        i = 0
        self.bag = None
        self.lesson_number = 0

    def start_lesson_callback(self, data):
        self.lesson_number = data.data

    def record(self):
        try:
            rospy.loginfo('Starting recorging mission status')
            date = time.strftime('%x')
            date = date.split('/')
            times = time.strftime('%X')
            times = times.split(':')
            file_name = f'/home/spot/{self.username}/mission_log_{date[0]}_{date[1]}_{date[2]}_{times[0]}:{times[1]}'
            mission_run = True
            f = open(file_name, "w")
            while mission_run:
                time.sleep(0.5)
                mission_run = (self.misson_client.get_state().status == 2)
                time.sleep(0.5)
            f.write(f"{self.misson_client.get_state()}\n")
            rospy.loginfo('Finished recording mission status')
            f.close()
            time.sleep(2)
        except Exception as e:
            rospy.loginfo(e)
        
    def spin(self):
        rospy.loginfo("Waiting fot student user")
        while True:
            if int(self.lesson_number) >= 4:
                with open('/etc/passwd', 'r') as f:
                    for line in f:
                        line = line.split(':')
                        if (re.match("student_[A-Z]", line[0]) is not None) and (line[0] != 'student_HSD'):
                            self.username = line[0]
                            #rospy.loginfo(f"Found user {line[0]}")
                            mission_run = (self.misson_client.get_state().status == 2)
                            if mission_run:
                                self.record()
                        else:
                            self.username = ''

MissionStateRecorder().spin()