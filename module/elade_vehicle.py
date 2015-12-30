from dronekit import connect, LocationGlobalRelative
from dronekit.lib import VehicleMode
from pymavlink import mavutil
import time
import ConfigParser

from module.init_position import InitPosition

__author__ = 'francoischevalier'


class EladeVehicle(object):
    vehicle = None
    master = ''
    baudrate = 57600
    aircraft = ''
    is_guided_mode = 0
    connection_status = 0
    init_position_save_status = 0
    prearm_status = 0
    takeoff_status = 0
    posinit_status = 0
    followme_status = 0
    land_status = 0
    init_position = None

    def __init__(self, master, baudrate, aircraft):
        self.master = master
        self.baudrate = baudrate
        self.aircraft = aircraft
        self.observers = []

    def __init__(self):
        config = ConfigParser.RawConfigParser()
        config.read('./module/uav_vehicle_config.cfg')
        self.master = config.get('UavConnectionParam', 'master')
        self.baudrate = config.getint('UavConnectionParam', 'baudrate')
        self.aircraft = config.get('UavConnectionParam', 'aircraft')
        self.observers = []

    def register(self, observer):
        if not observer in self.observers:
            self.observers.append(observer)

    def notify_observer(self, *args, **kwargs):
        for observer in self.observers:
            observer.update(*args, **kwargs)

    def mode_change_callback(self, attr_name, value):
        if attr_name == 'mode':
            if value.name == "GUIDED":
                self.is_guided_mode = 1
                print "GUIDED MODE TRIGGERED"
            else:
                self.is_guided_mode = 0
                print "NO MORE GUIDED MODE"

    def handle_incomming_message(self, message):
        print "incoming message"
        commandSplited = message.split('/')
        if commandSplited[0] == "posinit":
            self.save_posinit(commandSplited[1])
        elif commandSplited[0] == "prearm":
            self.prearm_launch()
        elif commandSplited[0] == "takeoff":
            self.init_process()
        elif commandSplited[0] == "newposition":
            ned_data = commandSplited[1].split(':')
            self.followme_instruction(ned_data[0], ned_data[1], ned_data[2])
        elif commandSplited[0] == "land":
            self.land_and_disarm()

    def connect_uav(self):
        try:
            self.vehicle = connect('/dev/ttyAMA0', wait_ready=True, baud=57600)
            self.vehicle.add_attribute_listener('mode', self.mode_change_callback)
            self.connection_status = 1
        except:
            self.connection_status = 0

        self.notify_observer('connection', self.connection_status)
        print "connection done with status: ", self.connection_status

    def prearm_launch(self):
        try:
            if self.connection_status == 1:
                print "prearm start"
                if self.vehicle.mode.name == "INITIALISING":
                    print "Waiting for vehicle to initialise..."
                    time.sleep(1)

                print "Basic pre-arm checks"
                # Don't let the user try to arm until autopilot is ready
                while self.vehicle.gps_0.fix_type < 2:
                    print "Waiting for GPS...:", self.vehicle.gps_0.fix_type
                    time.sleep(1)

                while not self.vehicle.is_armable:
                    print " Waiting for vehicle to initialise..."
                    time.sleep(1)

                self.vehicle.mode = VehicleMode("GUIDED")
                print "vehicle mode now are ", self.vehicle.mode.name
                self.prearm_status = 1

        except:
            self.prearm_status = 0

        self.notify_observer('prearm', self.prearm_status)
        print "prearm done with status: ", self.prearm_status

    def init_process(self):
        self.arming_and_takeoff(self.init_position.alt)
        time.sleep(5)
        self.reach_init_position(self.init_position)
        # self.test_arming()

    def test_arming(self):
        try:
            if self.prearm_status == 1:
                print "Arming motors"
                # Copter should arm in GUIDED mode
                if self.vehicle.mode.name == "GUIDED":
                    print "start"
                    self.vehicle.armed = True

                    while not self.vehicle.armed:
                        print " Waiting for arming..."
                        time.sleep(1)

                    time.sleep(10)
                    self.vehicle.armed = False
        except:
            print "erreur test arming"

    def arming_and_takeoff(self, atargetaltitude):
        try:
            if self.prearm_status == 1:
                print "Arming motors"
                # Copter should arm in GUIDED mode
                if self.vehicle.mode.name == "GUIDED":

                    self.vehicle.armed = True

                    while not self.vehicle.armed:
                        print " Waiting for arming..."
                        time.sleep(1)

                    if self.vehicle.mode.name == "GUIDED":
                        print "Taking off!"
                        self.vehicle.simple_takeoff(atargetaltitude)  # Take off to target altitude

                        # Wait until the vehicle reaches a safe height before processing
                        while True:
                            print " Altitude: ", self.vehicle.location.global_relative_frame.alt
                            if self.vehicle.location.global_relative_frame.alt >= atargetaltitude * 0.95:
                                print "Reached target altitude"
                                break
                            time.sleep(1)

                    self.takeoff_status = 1
                else:
                    print "error bad mode"
                    self.takeoff_status = 0
        except:
            self.takeoff_status = 0

        self.notify_observer('takeoff', self.takeoff_status)
        print "takeoff done with status: ", self.takeoff_status

    def land_and_disarm(self):
        if self.is_guided_mode == 1:
            print("Setting LAND mode...")
            self.vehicle.mode = VehicleMode("LAND")

    def reach_init_position(self, gpsfix):
        try:
            if self.takeoff_status == 1 and self.vehicle.mode == "GUIDED":
                print "Go to initial position"
                print "Set default/target airspeed to 3"
                self.vehicle.airspeed = 3

                print "Going towards initial point for 15 seconds ..."
                initpos = LocationGlobalRelative(gpsfix.lat, gpsfix.lon)
                self.vehicle.simple_goto(initpos)
                time.sleep(15)
                self.condition_yaw(gpsfix.heading)
                self.posinit_status = 1
        except:
            self.posinit_status = 0

        self.notify_observer('posinit', self.posinit_status)
        print "initial position reached with status: ", self.posinit_status

    def followme_instruction(self, north, east, down):
        if self.is_guided_mode == 1:
            self.send_ned_velocity(north, east, down, 2)

    def save_posinit(self, posinit):
        try:
            pos_init_list = posinit.split(':')
            self.init_position = InitPosition(pos_init_list[0], pos_init_list[1], pos_init_list[2], pos_init_list[3])
            self.init_position_save_status = 1
        except:
            self.init_position_save_status = 0

        self.notify_observer('initpossave', self.init_position_save_status)
        print "initial position saved with status: ", self.init_position_save_status

    def condition_yaw(self, heading, relative=False):
        if relative:
            is_relative = 1  # yaw relative to direction of travel
        else:
            is_relative = 0  # yaw is an absolute angle
        # create the CONDITION_YAW command using command_long_encode()
        msg = self.vehicle.message_factory.command_long_encode(
                0, 0,  # target system, target component
                mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
                0,  # confirmation
                heading,  # param 1, yaw in degrees
                0,  # param 2, yaw speed deg/s
                1,  # param 3, direction -1 ccw, 1 cw
                is_relative,  # param 4, relative offset 1, absolute angle 0
                0, 0, 0)  # param 5 ~ 7 not used
        # send command to vehicle
        self.vehicle.send_mavlink(msg)

    def send_ned_velocity(self, velocity_x, velocity_y, velocity_z, duration):
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
                0,  # time_boot_ms (not used)
                0, 0,  # target system, target component
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
                0b0000111111000111,  # type_mask (only speeds enabled)
                0, 0, 0,  # x, y, z positions (not used)
                velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
                0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
                0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        # send command to vehicle on 1 Hz cycle
        for x in range(0, duration):
            if self.is_guided_mode == 1:
                self.vehicle.send_mavlink(msg)
            time.sleep(1)
