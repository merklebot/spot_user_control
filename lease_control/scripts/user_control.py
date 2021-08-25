#!/usr/bin/env python3

import random
import time
from requests import Session
from selenium import webdriver
from selenium.webdriver.firefox.firefox_binary import FirefoxBinary
from selenium.webdriver import FirefoxOptions
import subprocess
from ast import literal_eval
from datetime import datetime, timedelta
from bosdyn.api import estop_pb2
import bosdyn.client
import os
import rospy
from std_msgs.msg import String
import zipfile
from pinatapy import PinataPy
import re
import smtplib, ssl
import imaplib
import email
from email.header import decode_header
import webbrowser
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
import lzma
from ast import literal_eval
from std_msgs.msg import String

class UserControl:
    def __init__(self):
        self.letters = [chr(i) for i in range(65, 91)]
        self.passw = [chr(i) for i in range(65, 91)] + [chr(i) for i in range(97, 123)] + [str(i) for i in range(10)]
        self.lease_time = timedelta(hours=1)
        self.path = os.path.realpath(__file__)[:-24] 
        rospy.init_node(f"user_control", anonymous=True)
        self.lesson_pub = rospy.Publisher("/start_lesson", String, queue_size=10, latch=True)
        self.hostname = '192.168.50.3'
        self.estop_pub = rospy.Publisher(f"/estop_core", String, queue_size=10)
        with open(f"{self.path}/config/config") as f:
            for line in f:
                line = line.split('/')
                self.username = line[0].strip()
                self.password = line[1].strip()
                pinata_pub = line[2].strip()
                pinata_secret = line[3].strip()
                self.mail_password = line[4].strip()
        self.pinata = PinataPy(pinata_pub, pinata_secret)
        rospy.loginfo("user_control ready")
    
    def create_user_pass(self):
        user = f'student_'
        for i in range(3):
            user += self.letters[random.randint(0, len(self.letters) - 1)]
        passw = ''
        for i in range(16):
            passw += self.passw[random.randint(0, len(self.passw) - 1)]
        return user, passw

    def delete_user_spot(self, username):
        opts = FirefoxOptions()
        opts.add_argument("--headless")
        browser = webdriver.Firefox(firefox_options=opts, executable_path='/home/spot/geckodriver')
        t = browser.get(f'https://{self.hostname}/users')
        browser.find_element_by_name("username").send_keys(self.username)
        browser.find_element_by_name("password").send_keys(self.password)
        browser.find_element_by_xpath("//button[1]").click()
        time.sleep(1)
        browser.find_element_by_link_text(username).click()
        browser.find_element_by_xpath("//button[@data-target='confirm-delete-modal']").click()
        browser.find_element_by_xpath("//button[@type='submit']").click()
        browser.quit()
        rospy.loginfo(f"Deleted spot user {username}")

    def add_user_spot(self, username, password):
        opts = FirefoxOptions()
        opts.add_argument("--headless")
        browser = webdriver.Firefox(firefox_options=opts, executable_path='/home/spot/geckodriver')
        t = browser.get(f'https://{self.hostname}/users/add')
        browser.find_element_by_name("username").send_keys(self.username)
        browser.find_element_by_name("password").send_keys(self.password)
        browser.find_element_by_xpath("//button[1]").click()
        browser.find_element_by_class_name("Form_input__2z3OC").send_keys(username)
        browser.find_element_by_xpath("//input[@type='password']").send_keys(password)
        browser.find_element_by_xpath("//button[@id='applyButton']").click()
        browser.quit()
        with open(f"/home/{username}/credentials", "w") as f:
            f.write(f"Username: {username}\n")
            f.write(f"Password: {password}")
        rospy.loginfo(f"Created spot user {username}")

    def add_user_core(self, username, password, metadata):    # metadata: {'key': '', 'lesson': , 'e-mail': ''}
        data = literal_eval(metadata)
        command = f"{self.path}/scripts/create_user_ubuntu.sh {username} {password}"
        create_user = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE)
        time.sleep(3)
        with open(f"/home/{username}/.ssh/authorized_keys", "a") as f:
            f.write(f"{data['key']}\n")
        self.create_lessons_task(username)
        os.mkdir(f"/home/spot/{username}")
        met_text = f"""
        Logs for Spot Education lesson №{metadata['lesson']}
        Link to the lesson: https://github.com/LoSk-p/robonomics-wiki/blob/master/docs/en/spot-lesson{metadata['lesson']}.md
        Lesson start data: {time.ctime()}
        Student e-mail: {metadata['e-mail']}
        """
        with open(f"/home/spot/{username}/metadata", "w") as met_f:
            met_f.write(met_text)
        self.lesson_pub.publish(str(metadata["lesson"]))
        rospy.loginfo(f"Created core user {username}")

    def delete_user_core(self, username):
        command = f"{self.path}/scripts/del_user_ubuntu.sh {username}"
        del_user = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE)
        rospy.loginfo(f"Deleted core user {username}")

    def compress_files(self, directory):
        files = os.listdir(directory)
        if directory[-1] != "/":
            directory += "/"
        for filename in files:
            with open(f"{directory}{filename}", "rb") as f:
                data = f.read()
            f_comressed = lzma.open(f"{directory}{filename}.xz", "wb")
            f_comressed.write(data)
            f_comressed.close()
            rospy.loginfo(f"{directory}{filename} compressed")
            os.remove(f"{directory}{filename}")


    def pin_to_ipfs(self, directory):
        time.sleep(3)
        self.compress_files(directory)
        res = self.pinata.pin_file_to_ipfs(directory)
        rospy.loginfo(f"Published to IPFS with hash: {res['IpfsHash']}")
        return res['IpfsHash']

    def send_email(self, receiver_email, text):
        port = 465  # For SSL
        smtp_server = "smtp.gmail.com"
        from_email = "spot@robonomics.network"
        sender_email = "spot.sdk.education@gmail.com"
        password = self.mail_password
        message = MIMEMultipart()
        message["Subject"] = "Spot Lesson"
        message["From"] = from_email
        message["To"] = receiver_email
        message.attach(MIMEText(text, 'plain'))
        context = ssl.create_default_context()
        with smtplib.SMTP_SSL("smtp.gmail.com", 465, context=context) as server:
            server.login(sender_email, password)
            server.sendmail(from_email, receiver_email, message.as_string())
    
    def create_lessons_task(self, user):
        os.mkdir(f"/home/{user}/lessons")
        with open(f"/home/{user}/lessons/lesson2", "w") as f:
            less2_x = [round(random.uniform(-0.5, 1), 1) for i in range(3)]
            less2_y = [round(random.uniform(-0.5, 1), 1) for i in range(3)]
            f.write("Dots:\n")
            for i in range(len(less2_x)):
                f.write(f"{{'x':{less2_x[i]}, 'y': {less2_y[i]}}}\n")

    def monitor_users(self):
        while not rospy.is_shutdown():
            try:
                time.sleep(0.3)
                with open('/etc/passwd', 'r') as f:
                    for line in f:
                        line = line.split(':')
                        if (re.match("student_[A-Z]", line[0]) is not None) and line[0] != 'student_HSD':
                            info = line[4].split('/')
                            time_created = info[1].split('.')
                            end_date = datetime(int(time_created[0]), int(time_created[1]), int(time_created[2]), int(time_created[3]), int(time_created[4]), int(time_created[5]))
                            end_date = end_date + self.lease_time
                            now_date = datetime.utcnow()
                            if now_date > end_date:
                                self.estop_pub.publish("press stop")
                                self.delete_user_spot(line[0])
                                self.delete_user_core(line[0])
                                rospy.loginfo(f"Deleted user {line[0]}")
                                time.sleep(10)
                                ipfs_hash = self.pin_to_ipfs(f"/home/spot/{line[0]}/")
                                # link = f"https://gateway.ipfs/ipfs/{ipfs_hash}"
                                # email_text = f"""
                                # You've passed Spot lesson
                                # Link to your rosbag log file:
                                # {link}
                                # """
                                # self.send_email(info[0], email_text)
                                # rospy.loginfo(f"Link {link} sent to {info[0]}")
                time.sleep(0.3)
            except Exception as e:
                rospy.loginfo(f"Exception: {e}")


if __name__ == '__main__':
    UserControl().monitor_users()