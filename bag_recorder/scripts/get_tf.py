#!/usr/bin/env python3

import rospy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from google.protobuf.timestamp_pb2 import Timestamp
import bosdyn.client

# паблишить 20 раз в секунду 

def GetTFFromState(state):
    """Maps robot link state data from robot state proto to ROS TFMessage message

    Args:
        data: Robot State proto
    Returns:
        TFMessage message
    """
    tf_msg = TFMessage()

    for frame_name in state.kinematic_state.transforms_snapshot.child_to_parent_edge_map:
        if state.kinematic_state.transforms_snapshot.child_to_parent_edge_map.get(frame_name).parent_frame_name:
            transform = state.kinematic_state.transforms_snapshot.child_to_parent_edge_map.get(frame_name)
            new_tf = TransformStamped()
            local_time = state.kinematic_state.acquisition_timestamp
            new_tf.header.stamp = rospy.Time(local_time.seconds, local_time.nanos)
            new_tf.header.frame_id = transform.parent_frame_name
            new_tf.child_frame_id = frame_name
            new_tf.transform.translation.x = transform.parent_tform_child.position.x
            new_tf.transform.translation.y = transform.parent_tform_child.position.y
            new_tf.transform.translation.z = transform.parent_tform_child.position.z
            new_tf.transform.rotation.x = transform.parent_tform_child.rotation.x
            new_tf.transform.rotation.y = transform.parent_tform_child.rotation.y
            new_tf.transform.rotation.z = transform.parent_tform_child.rotation.z
            new_tf.transform.rotation.w = transform.parent_tform_child.rotation.w
            tf_msg.transforms.append(new_tf)

    return tf_msg

class PubTF():
    def __init__(self):
        rospy.init_node("get_tf", anonymous=False)
        username = rospy.get_param("~username")
        password = rospy.get_param("~password")
        hostname = rospy.get_param("~hostname")
        sdk = bosdyn.client.create_standard_sdk('understanding-spot')
        self.robot = sdk.create_robot(hostname)
        self.robot.authenticate(username, password)
        self.state_client = self.robot.ensure_client('robot-state')
        self.tf_pub = rospy.Publisher('tf', TFMessage, queue_size=10)

    def spin(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            state = self.state_client.get_status()
            state_tf = GetTFFromState(state)
            self.tf_pub.publish(state_tf)
            rate.sleep()

