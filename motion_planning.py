import argparse
import time
from typing import Tuple

import msgpack
from enum import Enum, auto

import numpy as np

from planning_utils import a_star, heuristic, create_grid
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2],
                          self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        # TARGET_ALTITUDE = 5
        # SAFETY_DISTANCE = 5
        TARGET_ALTITUDE: int = 2
        SAFETY_DISTANCE: int = 6

        self.target_position[2] = TARGET_ALTITUDE

        map_file: str = 'colliders.csv'

        self.set_global_home(map_file)
        self.determine_local_position()

        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt(map_file, delimiter=',', dtype='Float64', skiprows=2)

        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        # Define starting point on the grid (this is just grid center)
        # grid_start = (-north_offset, -east_offset)

        # Set goal as some arbitrary position on the grid
        grid_goal = (-north_offset + 10, -east_offset + 10)

        grid_start: Tuple[float, float] = self.get_starting_location(north_offset=north_offset,
                                                                     east_offset=east_offset)

        goal_latitude: float = 37.793480
        goal_longitude: float = -122.396690
        grid_goal: Tuple[float, float] = self.get_goal_location(goal_latitude=goal_latitude,
                                                                goal_longitude=goal_longitude,
                                                                goal_altitude=TARGET_ALTITUDE,
                                                                north_offset=north_offset,
                                                                east_offset=east_offset)

        print(f"grid_goal value {grid[grid_goal[0], grid_goal[1]]}")
        obstacle_marker: int = 1
        if grid_goal == grid_start or grid[grid_goal[0], grid_goal[1]] == obstacle_marker:
            print("ERROR!!!!! The goal is not valid")

        # Run A* to find a path from start to goal
        # or move to a different search space such as a graph (not done here)
        print('Local Start and Goal: ', grid_start, grid_goal)
        path, _ = a_star(grid, heuristic, grid_start, grid_goal)
        # TODO: prune path to minimize number of waypoints
        # TODO (if you're feeling ambitious): Try a different approach altogether!

        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]
        # Set self.waypoints
        self.waypoints = waypoints
        print(f"self.waypoints {self.waypoints}")
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()

    def set_global_home(self, map_file: str):
        # DONE: read lat0, lon0 from colliders into floating point values
        with open(map_file) as file:
            first_line: str = file.readline()
            latitude_info: str
            longitude_info: str
            latitude_info, longitude_info = tuple(first_line.split(","))

            # These values are the center of the map (according to the Configuration Space Exercise.
            altitude: float = 0.0
            latitude: float = float(latitude_info.split()[1])
            longitude: float = float(longitude_info.split()[1])

            # DONE: set home position to (lon0, lat0, 0)
            self.set_home_position(longitude=longitude, latitude=latitude, altitude=altitude)
            print(f"home position->longitude {longitude} latitude {latitude} altitude {altitude}  ")

    def determine_local_position(self):
        north_local: float
        east_local: float
        down_local: float

        # DONE: retrieve current global position
        global_position: Tuple[float, float, float] = (self._longitude, self._latitude, self._altitude)
        print(f"global_position {global_position}")

        # DONE: convert to current local position using global_to_local()
        north_local, east_local, down_local = global_to_local(global_position, self.global_home)
        print(f"north_local {north_local} east_local {east_local} down_local {down_local}")

    def get_goal_location(self, goal_latitude: float, goal_longitude: float, goal_altitude: float,
                          north_offset: int, east_offset: int) -> Tuple[int, int]:

        # DONE: adapt to set goal as latitude / longitude position and convert
        print(f"goal_longitude {goal_longitude} goal_latitude {goal_latitude} goal_altitude {goal_altitude} ")

        global_position: Tuple[float, float, float] = (goal_longitude, goal_latitude, goal_altitude)

        north_goal, east_goal, _ = global_to_local(global_position, self.global_home)
        north_goal, east_goal = int(north_goal), int(east_goal)

        return north_goal - north_offset, east_goal - east_offset

    def get_starting_location(self, north_offset: int, east_offset: int) -> Tuple[int, int]:
        # DONE: convert start position to current position rather than map center

        north_coordinate_index: int = 0
        east_coordinate_index: int = 1

        grid_start: Tuple[int, int] = (int(self.local_position[north_coordinate_index]) - north_offset,
                                       int(self.local_position[east_coordinate_index]) - east_offset)

        return grid_start


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
