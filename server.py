import tornado.ioloop
import tornado.web
import tornado.websocket

from module import elade_vehicle
from module import elade_vehicle_observer

uavVehicle = None
config = None


class MainHandler(tornado.web.RequestHandler):
    def get(self):
        self.write("Hello, World")


class WSHandler(tornado.websocket.WebSocketHandler, elade_vehicle_observer.EladeVehicleObserver):
    def check_origin(self, origin):
        return True

    def open(self, *args, **kwargs):
        print("Connection Opened")
        uavVehicle.register(self)
        uavVehicle.connect_uav()

    def on_close(self):
        print("Connection closed")
        if uavVehicle.takeoff_status == 0:
            uavVehicle.vehicle.close()

    def on_message(self, message):
        if uavVehicle is not None:
            uavVehicle.handle_incomming_message(message)

    def update(self, *args, **kwargs):
        print('update triggered')
        self.write_message("{0}/{1}".format(args[0],args[1]))


application = tornado.web.Application([
    (r'/', MainHandler),
    (r'/ws', WSHandler),
])


if __name__ == '__main__':
    uavVehicle = elade_vehicle.EladeVehicle()
    application.listen(8888)
    tornado.ioloop.IOLoop.instance().start()

