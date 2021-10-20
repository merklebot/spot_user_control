#!/usr/bin/env python3

import rospy
import paho.mqtt.client as mqtt
from std_msgs.msg import String

rospy.init_node("estop_button")
estop_pub = rospy.Publisher("/estop_core", String, queue_size=10, latch=True)

def on_message(client, userdata, message):
    rospy.loginfo(message.payload.decode())
    estop_pub.publish(message.payload.decode())

client = mqtt.Client()
client.connect("200:7938:167:4f0a:e7f4:196c:6585:52ad", 1883, 60)
client.subscribe("estop")
client.on_message = on_message
client.loop_forever()
