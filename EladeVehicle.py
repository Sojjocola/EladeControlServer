__author__ = 'francoischevalier'
from dronekit import connect
from dronekit.lib import VehicleMode
from pymavlink import mavutil
import time

class EladeVehicle(object):
    vehicle = None
    master = ''
    baudrate = 57600
    aircraft = ''

    def __init__(self, master, baudrate, aircraft):
        self.master = master
        self.baudrate = baudrate
        self.aircraft = aircraft

    def connectUav(self):
        self.vehicle = connect(self.master)

    def prearmLaunch(self):
        """
        Test code on real uav
        """
        print "Basic pre-arm checks"
        # Don't let the user try to fly while autopilot is booting
        if self.vehicle.mode.name == "INITIALISING":
            print "Waiting for vehicle to initialise"
            time.sleep(1)
        while self.vehicle.gps_0.fix_type < 2:
            print "Waiting for GPS...:", self.vehicle.gps_0.fix_type
            time.sleep(1)




