#!/usr/bin/env python3
import rospy
import time
import bosdyn.client
import rosnode
import subprocess
from pinatapy import PinataPy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import InteractiveMarkerUpdate
import os
import re


class BagRecorder():
    def __init__(self):
        types = {'CameraInfo': CameraInfo, 'Image': Image, 'TFMessage': TFMessage, 'InteractiveMarkerUpdate': InteractiveMarkerUpdate}
        rospy.init_node(f"rosbag_recorder", anonymous=False)
        config_path = rospy.get_param("~config")
        print(config_path)
        username = rospy.get_param("~username")
        password = rospy.get_param("~password")
        hostname = rospy.get_param("~hostname")
        sdk = bosdyn.client.create_standard_sdk('understanding-spot')
        self.robot = sdk.create_robot(hostname)
        self.robot.authenticate(username, password)
        self.state_client = self.robot.ensure_client('robot-state')
        self.recording = False
        i = 0
        self.bag = None

    def record(self):
        try:
            rospy.loginfo('Starting recorging rosbag')
            date = time.strftime('%x')
            date = date.split('/')
            times = time.strftime('%X')
            times = times.split(':')
            file_name_full = f'/home/spot/rosbags/lesson_one_full_{date[0]}_{date[1]}_{date[2]}_{times[0]}:{times[1]}'
            self.command_full = ['rosbag', 'record', f'--output-name={file_name_full}', '/spot/depth/back/camera_info', '/spot/depth/back/image', '/spot/depth/frontleft/camera_info', '/spot/depth/frontleft/image', '/spot/depth/frontright/camera_info', '/spot/depth/frontright/image', '/spot/depth/left/camera_info', '/spot/depth/left/image', '/spot/depth/right/camera_info', '/spot/depth/right/image', '/tf', '/tf_static', '/twist_marker_server/update']
            files = os.listdir('/home/spot/')
            if f'{self.username}' not in files:
                os.mkdir(f'/home/spot/{self.username}')
            file_name = f'/home/spot/{self.username}/lesson_{date[0]}_{date[1]}_{date[2]}_{times[0]}:{times[1]}'
            self.command = ['rosbag', 'record', f'--output-name={file_name}', '/tf', '/tf_static']
            rosbag_proc = subprocess.Popen(self.command)
            rosbag_proc_full = subprocess.Popen(self.command_full)
            power_on = True
            while power_on:
                power_on = self.robot.is_powered_on()
                time.sleep(1)
                #power_on = False
            rosbag_proc.terminate()
            rosbag_proc_full.terminate()
            rospy.loginfo('Finished recording')
            time.sleep(2)
        except Exception as e:
            rospy.loginfo(e)
        
    def spin(self):
        rospy.loginfo("Waiting fot student user")
        while True:
            with open('/etc/passwd', 'r') as f:
                for line in f:
                    line = line.split(':')
                    if re.match("student_[A-Z]", line[0]) is not None:
                        self.username = line[0]
                        #rospy.loginfo(f"Found user {line[0]}")
                        power = self.robot.is_powered_on()
                        if power:
                            self.record()
                    else:
                        self.username = ''

BagRecorder().spin()
