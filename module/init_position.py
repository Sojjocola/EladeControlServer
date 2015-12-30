__author__ = 'fchevalier12'


class InitPosition(object):
    lat = 0
    long = 0
    alt = 0
    heading = 0

    def __init__(self, alt, heading, lat, long):
        self.alt = alt
        self.heading = heading
        self.lat = lat
        self.long = long

