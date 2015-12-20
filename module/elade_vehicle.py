from dronekit import connect, LocationGlobalRelative
from dronekit.lib import VehicleMode
from pymavlink import mavutil
import json
import time
import ConfigParser

__author__ = 'francoischevalier'


class EladeVehicle(object):
    vehicle = None
    master = ''
    baudrate = 57600
    aircraft = ''
    connection_status = 0
    prearm_status = 0
    takeoff_status = 0
    posinit_status = 0
    followme_status = 0
    land_status = 0

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

    def handle_incomming_message(self, message):
        print "incoming message"

    def connect_uav(self):
        try:
            self.vehicle = connect(self.master)
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
                while not self.vehicle.is_armable:
                    print " Waiting for vehicle to initialise..."
                    time.sleep(1)

                while self.vehicle.gps_0.fix_type < 2:
                    print "Waiting for GPS...:", self.vehicle.gps_0.fix_type
                    time.sleep(1)

                self.vehicle.mode = VehicleMode("GUIDED")
                self.prearm_status = 1

        except:
            self.prearm_status = 0

        self.notify_observer('prearm', self.prearm_status)
        print "prearm done with status: ", self.prearm_status

    def arming_and_takeoff(self, atargetaltitude):
        try:
            if self.prearm_status == 1:
                print "Arming motors"
                # Copter should arm in GUIDED mode
                if self.vehicle.mode.name != "GUIDED":

                    self.vehicle.armed = True

                    while not self.vehicle.armed:
                        print " Waiting for arming..."
                        time.sleep(1)

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

    def reach_init_position(self, gpsfix):
        try:
            if self.takeoff_status == 1:
                print "Go to initial position"
                print "Set default/target airspeed to 3"
                self.vehicle.airspeed = 3

                print "Going towards first point for 30 seconds ..."
                initpos = LocationGlobalRelative(gpsfix.lat, gpsfix.lon)
                self.vehicle.simple_goto(initpos)
                time.sleep(10)
                self.posinit_status = 1
        except:
            self.posinit_status = 0

        self.notify_observer('posinit', self.posinit_status)
        print "initial position reached with status: ", self.posinit_status


    def followme_instruction(self, north, east, down):
        print "Not implemented"
