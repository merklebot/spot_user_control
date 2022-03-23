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
import os
import re


class BagRecorder():
    def __init__(self):
        types = {'CameraInfo': CameraInfo, 'Image': Image, 'TFMessage': TFMessage, 'InteractiveMarkerUpdate': InteractiveMarkerUpdate}
        rospy.init_node(f"rosbag_recorder", anonymous=False)
        rospy.Subscriber("/start_lesson", String, self.start_lesson_callback)
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
        self.lesson_number = 0

    def record(self):
        try:
            rospy.loginfo('Starting recorging rosbag')
            date = time.strftime('%x')
            date = date.split('/')
            times = time.strftime('%X')
            times = times.split(':')
            if self.lesson_number == '3':
                file_name = f'/home/spot/rosbags/rosbag_log_full_{date[0]}_{date[1]}_{date[2]}_{times[0]}:{times[1]}'
                command = ['rosbag', 'record', f'--output-name={file_name}', '/spot/depth/back/camera_info', '/spot/depth/back/image', '/spot/depth/frontleft/camera_info', '/spot/depth/frontleft/image', '/spot/depth/frontright/camera_info', '/spot/depth/frontright/image', '/spot/depth/left/camera_info', '/spot/depth/left/image', '/spot/depth/right/camera_info', '/spot/depth/right/image', '/tf', '/tf_static', '/twist_marker_server/update']
            else:
                file_name = f'/home/spot/{self.username}/rosbag_log_{date[0]}_{date[1]}_{date[2]}_{times[0]}:{times[1]}'
                command = ['rosbag', 'record', f'--output-name={file_name}', '/tf', '/tf_static']
            rosbag_proc = subprocess.Popen(command)
            power_on = True
            while power_on:
                power_on = self.robot.is_powered_on()
                time.sleep(1)
                #power_on = False
            rosbag_proc.terminate()
            rospy.loginfo('Finished recording')
            time.sleep(2)
        except Exception as e:
            rospy.loginfo(e)
    
    def start_lesson_callback(self, data):
        self.lesson_number = data.data
        
    def spin(self):
        rospy.loginfo("Waiting fot student user")
        while True:
            time.sleep(0.5)
            with open('/etc/passwd', 'r') as f:
                for line in f:
                    line = line.split(':')
                    if re.match("student", line[0]) is not None:
                        self.username = line[0]
                        #rospy.loginfo(f"Found user {line[0]}")
                        power = self.robot.is_powered_on()
                        if power:
                            self.record()
                    else:
                        self.username = ''

BagRecorder().spin()
