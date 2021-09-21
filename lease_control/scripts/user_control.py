#!/usr/bin/env python3

import random
import shutil
import time
from selenium import webdriver
from selenium.webdriver import FirefoxOptions
import subprocess
from ast import literal_eval
from datetime import datetime, timedelta
import os
import rospy
from std_msgs.msg import String
from pinatapy import PinataPy
import re
import lzma
from ast import literal_eval
from std_msgs.msg import String
import stat
from tenacity import retry, stop_after_attempt, wait_fixed
import shutil

def fail_pin_to_ipfs(retry_state):
        rospy.loginfo(f"Failed pin files to IPFS, retry_state: {retry_state}")

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
        self.pinata = PinataPy(pinata_pub, pinata_secret)
        rospy.loginfo("user_control ready")
    
    def create_user_pass(self):
        passw = ''
        for i in range(16):
            passw += self.passw[random.randint(0, len(self.passw) - 1)]
        return passw

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

    @retry(stop=stop_after_attempt(3))
    def add_user_spot(self, username, password):
        opts = FirefoxOptions()
        opts.add_argument("--headless")
        browser = webdriver.Firefox(firefox_options=opts, executable_path='/home/spot/geckodriver')
        t = browser.get(f'https://{self.hostname}/users/add')
        browser.find_element_by_name("username").send_keys(self.username)
        browser.find_element_by_name("password").send_keys(self.password)
        browser.find_element_by_xpath("//button[1]").click()
        time.sleep(1)
        browser.find_element_by_class_name("Form_input__2z3OC").send_keys(username)
        browser.find_element_by_xpath("//input[@type='password']").send_keys(password)
        browser.find_element_by_xpath("//button[@id='applyButton']").click()
        browser.quit()
        with open(f"/home/{username}/credentials", "w") as f:
            f.write(f"Username: {username}\n")
            f.write(f"Password: {password}")
        rospy.loginfo(f"Created spot user {username}")

    def add_user_core(self, username, password, metadata):    # metadata: {'key': [''], 'lesson': '', 'e-mail': ['']}
        data = literal_eval(metadata)
        command = f"{self.path}/scripts/create_user_ubuntu.sh {username} {password}"
        subprocess.Popen(command, shell=True, stdout=subprocess.PIPE)
        time.sleep(3)
        with open(f"/home/{username}/.ssh/authorized_keys", "a") as f:
            for key in data['key']:
                f.write(f"{key}\n")
        self.create_lessons_task(username)
        if os.path.exists(f"/home/spot/{username}"):
            shutil.rmtree(f"/home/spot/{username}")
        os.mkdir(f"/home/spot/{username}")
        os.chmod(f"/home/spot/{username}", stat.S_IRWXO)
        emails = ''
        for email in data['e-mail']:
            emails += f"{email}, "
        emails = emails[:-2]
        met_text = f"""
        Logs for Spot Education lesson №{data['lesson']}
        Link to the lesson: https://github.com/LoSk-p/robonomics-wiki/blob/master/docs/en/spot-lesson{data['lesson']}.md
        Lesson start data: {time.ctime()}
        Student e-mails: {emails}"""
        with open(f"/home/spot/{username}/metadata", "w") as met_f:
            met_f.write(met_text)
        self.lesson_pub.publish(data["lesson"])
        rospy.loginfo(f"Created core user {username}")

    def delete_user_core(self, username):
        files = os.listdir(f"/home/{username}/result")
        for file in files:
            shutil.copy2(f"/home/{username}/result/{file}", f"/home/spot/{username}/{file}")
        command = f"{self.path}/scripts/del_user_ubuntu.sh {username}"
        subprocess.Popen(command, shell=True, stdout=subprocess.PIPE)
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

    @retry(stop=stop_after_attempt(3), wait=wait_fixed(5), retry_error_callback=fail_pin_to_ipfs)
    def pin_to_ipfs(self, directory):
        time.sleep(3)
        try:
            res = self.pinata.pin_file_to_ipfs(directory)
            rospy.loginfo(f"Published to IPFS with hash: {res['IpfsHash']}")
            shutil.rmtree(directory)
            return res['IpfsHash']
        except Exception as e:
            rospy.loginfo(f"Can't pin files to IPFS with exception {e}. Retrying...")
            raise
    
    def create_lessons_task(self, user):
        os.mkdir(f"/home/{user}/lessons")
        with open(f"/home/{user}/lessons/lesson2", "w") as f:
            less2_x = [round(random.uniform(-0.5, 1), 1) for i in range(3)]
            less2_y = [round(random.uniform(-0.5, 1), 1) for i in range(3)]
            f.write("Dots:\n")
            for i in range(len(less2_x)):
                f.write(f"{{'x':{less2_x[i]}, 'y': {less2_y[i]}}}\n")
