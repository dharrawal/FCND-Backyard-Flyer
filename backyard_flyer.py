import argparse
import time
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5


class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}

        self.currentWaypoint = -1
        self.takeOffAlt = 3.0

        # initial state
        self.flight_state = States.MANUAL

        # TODO: Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data
        """
        # compare with -self.takeOffAlt because relative altitude is -ve when drone climbs
        if self.flight_state == States.TAKEOFF and abs(self.local_position[2] - (-self.takeOffAlt)) < 0.1:
            self.calculate_box()
            self.currentWaypoint = 0
            self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if self.currentWaypoint < len(self.all_waypoints)-1:
                if abs(self.local_position[0] - self.target_position[0]) < 0.1    \
                and abs(self.local_position[1] - self.target_position[1]) < 0.1    \
                and abs(self.local_position[2] - (-self.target_position[2])) < 0.1:
                    self.currentWaypoint += 1
                    self.waypoint_transition()
            else:
                if abs(self.local_position[0] - self.target_position[0]) < 0.1    \
                and abs(self.local_position[1] - self.target_position[1]) < 0.1    \
                and abs(self.local_position[2] - (-self.target_position[2])) < 0.1:
                    self.landing_transition()
                    
        
        pass

    def velocity_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data
        """
        if self.flight_state == States.LANDING and abs(self.local_position[2]) < 0.1 and self.local_velocity[2] < 0.01:
            self.disarming_transition()
            
        pass

    def state_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.STATE` is received and self.armed and self.guided contain new data
        """

        if self.flight_state == States.MANUAL:
            self.arming_transition()
        elif self.flight_state == States.ARMING:
            self.takeoff_transition()
        elif self.flight_state == States.DISARMING:
            self.manual_transition()
            
        pass

    def calculate_box(self):
        self.all_waypoints = [(15, 0, 10, np.pi/2),
                              (15, 15, 10, 0),
                              (0, 15, 10, -np.pi/2),
                              (0, 0, 10, np.pi)]
        pass

    def arming_transition(self):
        """TODO: Fill out this method
        
        1. Take control of the drone
        2. Pass an arming command
        3. Set the home location to current position
        4. Transition to the ARMING state
        """
        self.take_control()
        self.arm()
        self.set_home_position(self.global_position[0], self.global_position[1], self.global_position[2])
        self.flight_state = States.ARMING
        
        print("arming transition")

    def takeoff_transition(self):
        """TODO: Fill out this method
        
        1. Set target_position altitude to 3.0m
        2. Command a takeoff to 3.0m
        3. Transition to the TAKEOFF state
        """
        self.takeoff(self.takeOffAlt)
        self.flight_state = States.TAKEOFF
        
        print("takeoff transition")

    def waypoint_transition(self):
        """TODO: Fill out this method
    
        1. Command the next waypoint position
        2. Transition to WAYPOINT state
        """
        self.target_position = self.all_waypoints[self.currentWaypoint]
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])
        self.flight_state = States.WAYPOINT

        print("waypoint transition")

    def landing_transition(self):
        """TODO: Fill out this method
        
        1. Command the drone to land
        2. Transition to the LANDING state
        """
        self.land()
        self.flight_state = States.LANDING

        print("landing transition")

    def disarming_transition(self):
        """TODO: Fill out this method
        
        1. Command the drone to disarm
        2. Transition to the DISARMING state
        """
        self.disarm()
        self.flight_state = States.DISARMING
        
        print("disarm transition")

    def manual_transition(self):
        """This method is provided
        
        1. Release control of the drone
        2. Stop the connection (and telemetry log)
        3. End the mission
        4. Transition to the MANUAL state
        """
        print("manual transition")

        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
        """This method is provided
        
        1. Open a log file
        2. Start the drone connection
        3. Close the log file
        """
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
