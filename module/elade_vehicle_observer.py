from abc import ABCMeta, abstractmethod
__author__ = 'fchevalier12'


class EladeVehicleObserver(object):
    __metaclass__ = ABCMeta

    @abstractmethod
    def update(self, *args, **kwargs):
        pass
