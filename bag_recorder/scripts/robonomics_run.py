#!/usr/bin/env python3

from substrateinterface import SubstrateInterface, Keypair
from bosdyn.mission.client import MissionClient
import bosdyn.client
import os
import time
from bosdyn.client.graph_nav import GraphNavClient
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder, blocking_stand
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.frame_helpers import get_odom_tform_body
from bosdyn.api import geometry_pb2
from bosdyn.api import power_pb2
from bosdyn.api import robot_state_pb2
from bosdyn.api.graph_nav import graph_nav_pb2
from bosdyn.api.graph_nav import map_pb2
from bosdyn.api.graph_nav import nav_pb2
import math

# import psutil

class RobonomicsRun:
    def __init__(self):
        self.substrate = SubstrateInterface(
            url="wss://main.frontier.rpc.robonomics.network",
            ss58_format=32,
            type_registry_preset="substrate-node-template",
            type_registry={
                "types": {
                    "Record": "Vec<u8>",
                    "Parameter": "Bool",
                    "LaunchParameter": "Bool",
                    "<T as frame_system::Config>::AccountId": "AccountId",
                    "RingBufferItem": {
                        "type": "struct",
                        "type_mapping": [
                            ["timestamp", "Compact<u64>"],
                            ["payload", "Vec<u8>"],
                        ],
                    },
                    "RingBufferIndex": {
                        "type": "struct",
                        "type_mapping": [
                            ["start", "Compact<u64>"],
                            ["end", "Compact<u64>"],
                        ],
                    }
                }
            },
        )
        mnemonic = os.environ['ROBONOMICS_MNEMONIC_SEED']
        spot_password = os.environ['SPOT_PASSWORD']
        spot_username = os.environ['SPOT_USERNAME']
        sdk = bosdyn.client.create_standard_sdk('robonomics_run', [MissionClient])
        self._robot = sdk.create_robot('192.168.50.3')
        self._robot.authenticate(spot_username, spot_password)
        self.keypair = Keypair.create_from_mnemonic(mnemonic, ss58_format=32)
        self._lease_client = self._robot.ensure_client(LeaseClient.default_service_name)
        self._robot_command_client = self._robot.ensure_client(RobotCommandClient.default_service_name)
        self._robot_state_client = self._robot.ensure_client(RobotStateClient.default_service_name)
        self._graph_nav_client = self._robot.ensure_client(GraphNavClient.default_service_name)
        self._estop_client = self._robot.ensure_client('estop')
        self._current_edges = dict()  #maps to_waypoint to list(from_waypoint)
        self._current_waypoint_snapshots = dict()  # maps id to waypoint snapshot
        self._current_edge_snapshots = dict()  # maps id to edge snapshot
        self._current_annotation_name_to_wp_id = dict()
        self.lease = self._lease_client.acquire()
        lease_keep_alive = LeaseKeepAlive(self._lease_client)

    def _upload_graph_and_snapshots(self, *args):
        """Upload the graph and snapshots to the robot."""
        print("Loading the graph from disk into local storage...")
        self._upload_filepath = "/home/spot/track"
        with open(self._upload_filepath + "/graph", "rb") as graph_file:
            # Load the graph from disk.
            data = graph_file.read()
            self._current_graph = map_pb2.Graph()
            self._current_graph.ParseFromString(data)
            print("Loaded graph has {} waypoints and {} edges".format(
                len(self._current_graph.waypoints), len(self._current_graph.edges)))
        for waypoint in self._current_graph.waypoints:
            # Load the waypoint snapshots from disk.
            with open(self._upload_filepath + "/waypoint_snapshots/{}".format(waypoint.snapshot_id),"rb") as snapshot_file:
                waypoint_snapshot = map_pb2.WaypointSnapshot()
                waypoint_snapshot.ParseFromString(snapshot_file.read())
                self._current_waypoint_snapshots[waypoint_snapshot.id] = waypoint_snapshot
        for edge in self._current_graph.edges:
            # Load the edge snapshots from disk.
            with open(self._upload_filepath + "/edge_snapshots/{}".format(edge.snapshot_id),
                      "rb") as snapshot_file:
                edge_snapshot = map_pb2.EdgeSnapshot()
                edge_snapshot.ParseFromString(snapshot_file.read())
                self._current_edge_snapshots[edge_snapshot.id] = edge_snapshot
        # Upload the graph to the robot.
        print("Uploading the graph and snapshots to the robot...")
        response = self._graph_nav_client.upload_graph(lease=self.lease.lease_proto,
                                                       graph=self._current_graph)
        # Upload the snapshots to the robot.
        for snapshot_id in response.unknown_waypoint_snapshot_ids:
            waypoint_snapshot = self._current_waypoint_snapshots[snapshot_id]
            self._graph_nav_client.upload_waypoint_snapshot(waypoint_snapshot)
            print("Uploaded {}".format(waypoint_snapshot.id))
        for snapshot_id in response.unknown_edge_snapshot_ids:
            edge_snapshot = self._current_edge_snapshots[snapshot_id]
            self._graph_nav_client.upload_edge_snapshot(edge_snapshot)
            print("Uploaded {}".format(edge_snapshot.id))

        # The upload is complete! Check that the robot is localized to the graph,
        # and it if is not, prompt the user to localize the robot before attempting
        # any navigation commands.
        localization_state = self._graph_nav_client.get_localization_state()
        if not localization_state.localization.waypoint_id:
            # The robot is not localized to the newly uploaded graph.
            print("\n")
            print("Upload complete! The robot is currently not localized to the map; please localize", \
                   "the robot using commands (2) or (3) before attempting a navigation command.")

    def id_to_short_code(self, id):
        """Convert a unique id to a 2 letter short code."""
        tokens = id.split('-')
        if len(tokens) > 2:
            return '%c%c' % (tokens[0][0], tokens[1][0])
        return None


    def pretty_print_waypoints(self, waypoint_id, waypoint_name, short_code_to_count, localization_id):
        short_code = self.id_to_short_code(waypoint_id)
        if short_code is None or short_code_to_count[short_code] != 1:
            short_code = '  '  # If the short code is not valid/unique, don't show it.

        print("%s Waypoint name: %s id: %s short code: %s" %
                ('->' if localization_id == waypoint_id else '  ',
                waypoint_name, waypoint_id, short_code))

    def update_waypoints_and_edges(self, graph, localization_id):
        """Update and print waypoint ids and edge ids."""
        name_to_id = dict()
        edges = dict()

        short_code_to_count = {}
        waypoint_to_timestamp = []
        for waypoint in graph.waypoints:
            # Determine the timestamp that this waypoint was created at.
            timestamp = -1.0
            try:
                timestamp = waypoint.annotations.creation_time.seconds + waypoint.annotations.creation_time.nanos / 1e9
            except:
                # Must be operating on an older graph nav map, since the creation_time is not
                # available within the waypoint annotations message.
                pass
            waypoint_to_timestamp.append((waypoint.id,
                                            timestamp,
                                            waypoint.annotations.name))

            # Determine how many waypoints have the same short code.
            short_code = self.id_to_short_code(waypoint.id)
            if short_code not in short_code_to_count:
                short_code_to_count[short_code] = 0
            short_code_to_count[short_code] += 1

            # Add the annotation name/id into the current dictionary.
            waypoint_name = waypoint.annotations.name
            if waypoint_name:
                if waypoint_name in name_to_id:
                    # Waypoint name is used for multiple different waypoints, so set the waypoint id
                    # in this dictionary to None to avoid confusion between two different waypoints.
                    name_to_id[waypoint_name] = None
                else:
                    # First time we have seen this waypoint annotation name. Add it into the dictionary
                    # with the respective waypoint unique id.
                    name_to_id[waypoint_name] = waypoint.id

        # Sort the set of waypoints by their creation timestamp. If the creation timestamp is unavailable,
        # fallback to sorting by annotation name.
        waypoint_to_timestamp = sorted(waypoint_to_timestamp, key= lambda x:(x[1], x[2]))

        # Print out the waypoints name, id, and short code in a ordered sorted by the timestamp from
        # when the waypoint was created.
        print('%d waypoints:' % len(graph.waypoints))
        for waypoint in waypoint_to_timestamp:
            self.pretty_print_waypoints(waypoint[0], waypoint[2], short_code_to_count, localization_id)

        for edge in graph.edges:
            if edge.id.to_waypoint in edges:
                if edge.id.from_waypoint not in edges[edge.id.to_waypoint]:
                    edges[edge.id.to_waypoint].append(edge.id.from_waypoint)
            else:
                edges[edge.id.to_waypoint] = [edge.id.from_waypoint]
            print("(Edge) from waypoint {} to waypoint {} (cost {})".format(
                edge.id.from_waypoint, edge.id.to_waypoint, edge.annotations.cost.value))

        return name_to_id, edges

    def find_unique_waypoint_id(self, short_code, graph, name_to_id):
        """Convert either a 2 letter short code or an annotation name into the associated unique id."""
        if len(short_code) != 2:
            # Not a short code, check if it is an annotation name (instead of the waypoint id).
            if short_code in name_to_id:
                # Short code is an waypoint's annotation name. Check if it is paired with a unique waypoint id.
                if name_to_id[short_code] is not None:
                    # Has an associated waypoint id!
                    return name_to_id[short_code]
                else:
                    print("The waypoint name %s is used for multiple different unique waypoints. Please use" + \
                            "the waypoint id." % (short_code))
                    return None
            # Also not an waypoint annotation name, so we will operate under the assumption that it is a
            # unique waypoint id.
            return short_code

    def _set_initial_localization_fiducial(self):
        """Trigger localization when near a fiducial."""
        robot_state = self._robot_state_client.get_robot_state()
        current_odom_tform_body = get_odom_tform_body(
            robot_state.kinematic_state.transforms_snapshot).to_proto()
        # Create an empty instance for initial localization since we are asking it to localize
        # based on the nearest fiducial.
        localization = nav_pb2.Localization()
        self._graph_nav_client.set_localization(initial_guess_localization=localization,
                                                ko_tform_body=current_odom_tform_body)
    
    def _set_initial_localization_waypoint(self, waypoint):
        """Trigger localization to a waypoint."""
        # Take the first argument as the localization waypoint.
        # if len(args) < 1:
        #     # If no waypoint id is given as input, then return without initializing.
        #     print("No waypoint specified to initialize to.")
        #     return
        destination_waypoint = self.find_unique_waypoint_id(
            waypoint, self._current_graph, self._current_annotation_name_to_wp_id)
        print(destination_waypoint)
        if not destination_waypoint:
            # Failed to find the unique waypoint id.
            return

        robot_state = self._robot_state_client.get_robot_state()
        current_odom_tform_body = get_odom_tform_body(
            robot_state.kinematic_state.transforms_snapshot).to_proto()
        # Create an initial localization to the specified waypoint as the identity.
        localization = nav_pb2.Localization()
        localization.waypoint_id = destination_waypoint
        localization.waypoint_tform_body.rotation.w = 1.0
        print(localization)
        self._graph_nav_client.set_localization(
            initial_guess_localization=localization,
            # It's hard to get the pose perfect, search +/-20 deg and +/-20cm (0.2m).
            max_distance = 0.2,
            max_yaw = 20.0 * math.pi / 180.0,
            fiducial_init=graph_nav_pb2.SetLocalizationRequest.FIDUCIAL_INIT_NO_FIDUCIAL,
            ko_tform_body=current_odom_tform_body)

    def get_waypoint_id(self, waypoint_number):
        graph = self._graph_nav_client.download_graph()
        for waypoint in graph.waypoints:
            if waypoint.annotations.name == f"waypoint_{waypoint_number}":
                dest_waypoint = waypoint
                break
        localization_id = self._graph_nav_client.get_localization_state().localization.waypoint_id
        # Update and print waypoints and edges
        self._current_annotation_name_to_wp_id, self._current_edges = self.update_waypoints_and_edges(self._current_graph, localization_id)
        destination_waypoint = self.find_unique_waypoint_id(
            dest_waypoint.id, self._current_graph, self._current_annotation_name_to_wp_id)
        return destination_waypoint
    
    def _check_success(self, command_id=-1):
        """Use a navigation command id to get feedback from the robot and sit when command succeeds."""
        if command_id == -1:
            # No command, so we have not status to check.
            return False
        status = self._graph_nav_client.navigation_feedback(command_id)
        if status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_REACHED_GOAL:
            # Successfully completed the navigation commands!
            return True
        elif status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_LOST:
            print("Robot got lost when navigating the route, the robot will now sit down.")
            return True
        elif status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_STUCK:
            print("Robot got stuck when navigating the route, the robot will now sit down.")
            return True
        elif status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_ROBOT_IMPAIRED:
            print("Robot is impaired.")
            return True
        else:
            # Navigation command is not complete yet.
            return False

    def route(self):
        time.sleep(1)
        while self._estop_client.get_status().stop_level != 4:
            time.sleep(1)
        self._upload_graph_and_snapshots()
        self._robot.power_on(timeout_sec=20)
        blocking_stand(self._robot_command_client, timeout_sec=10)
        self._set_initial_localization_fiducial()
        waypoint = self.get_waypoint_id(0)
        print(waypoint)
        self._set_initial_localization_waypoint(waypoint)
        waypoint = self.get_waypoint_id(37)
        print(waypoint)
        # waypoint = "bosky-pike-eA0JCh6PPUwc23.baGTTxQ=="
        is_finished = False
        nav_to_cmd_id = None
        while not is_finished:
            # Issue the navigation command about twice a second such that it is easy to terminate the
            # navigation command (with estop or killing the program).
            try:
                nav_to_cmd_id = self._graph_nav_client.navigate_to(waypoint, 1.0,
                                                                   leases=None,
                                                                   command_id=nav_to_cmd_id)
            except Exception as e:
                print("Error while navigating {}".format(e))
                break
            time.sleep(.5)  # Sleep for half a second to allow for command execution.
            # Poll the robot for feedback to determine if the navigation command is complete. Then sit
            # the robot down once it is finished.
            is_finished = self._check_success(nav_to_cmd_id)
        


    def subscription_handler(self, obj, update_nr, subscription_id):
        ch = self.substrate.get_chain_head()
        chain_events = self.substrate.get_events(ch)
        for ce in chain_events:
            # if ce.value["event_id"] == "NewLaunch":
            #     print(ce)
            #     print(self.keypair.ss58_address)
            if ce.value["event_id"] == "NewLaunch" and ce.params[1]["value"] == self.keypair.ss58_address \
                                            and ce.params[2]["value"] is True:  # yes/no
                print(f"\"ON\" launch command from employer.")
                self.route()
                # print("Route")


    def spin(self):
        print(f"Wait for payment")
        self.substrate.subscribe_block_headers(self.subscription_handler)

if __name__ == '__main__':
    RobonomicsRun().spin()