from dronekit import connect
from dronekit.lib import VehicleMode
from pymavlink import mavutil
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
            print "prearm start"
            if self.vehicle.mode.name == "INITIALISING":
                print "Waiting for vehicle to initialise"
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



