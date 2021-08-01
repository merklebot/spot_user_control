#!/usr/bin/env python3

from bosdyn.api import estop_pb2
import bosdyn.client
import rospy
from std_msgs.msg import String

class EstopControl:
    def __init__(self):
        ### Create E-Stop
        rospy.init_node(f"estop_control", anonymous=False)
        username = rospy.get_param("~username")
        password = rospy.get_param("~password")
        hostname = rospy.get_param("~hostname")
        sdk = bosdyn.client.create_standard_sdk('understanding-spot')
        rospy.Subscriber(f"/estop_core", String, self.estop_callback)
        self.robot = sdk.create_robot(hostname)
        self.robot.authenticate(username, password)
        self.estop_client = self.robot.ensure_client('estop')
        self.create_estop()
    
    def estop_callback(self, data):
        if data.data == "press stop":
            self.estop_keep_alive.settle_then_cut()
            rospy.loginfo('Stopping Spot')
        elif data.data == "press allow":
            self.estop_keep_alive.allow()
            rospy.loginfo('Spot can move')

    def create_estop(self):
        new_config = estop_pb2.EstopConfig()
        new_config_endpoint = new_config.endpoints.add()
        estop_endpoint = bosdyn.client.estop.EstopEndpoint(client=self.estop_client, name='Core_estop', estop_timeout=9.0)
        new_config_endpoint.CopyFrom(estop_endpoint.to_proto())

        # estop_endpoint1 = bosdyn.client.estop.EstopEndpoint(client=self.estop_client, name='my_estop', estop_timeout=9.0, role="Student")
        # new_config_endpoint1 = new_config.endpoints.add()
        # new_config_endpoint1.CopyFrom(estop_endpoint1.to_proto())
        active_config = self.estop_client.get_config()
        active_config = self.estop_client.set_config(new_config, active_config.unique_id)
        estop_endpoint._unique_id = active_config.endpoints[0].unique_id
        estop_endpoint.register(active_config.unique_id)

        self.estop_keep_alive = bosdyn.client.estop.EstopKeepAlive(estop_endpoint)
        self.estop_keep_alive.allow()
        self.start_config = self.estop_client.get_config()

        rospy.loginfo(f'E-Stop created')

    def spin(self):
        while not rospy.is_shutdown():
            current_config = self.estop_client.get_config()
            if current_config != self.start_config:
                self.estop_keep_alive.shutdown()
                self.create_estop()

if __name__ == '__main__':
    EstopControl().spin()


