__author__ = 'fchevalier12'


class InitPosition(object):
    latitude = 0
    longitude = 0
    alt = 0
    heading = 0
    follower_orientation = 0

    def __init__(self, alt, heading, latitude, longitude, follower_orientation):
        self.alt = alt
        self.heading = heading
        self.latitude = latitude
        self.longitude = longitude
        self.follower_orientation = follower_orientation
