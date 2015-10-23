__author__ = 'francoischevalier'
import tornado.ioloop
import tornado.web
import tornado.websocket
import ConfigParser
import EladeVehicle


uavVehicle = None
config = None


class MainHandler(tornado.web.RequestHandler):
    def get(self):
        self.write("Hello, World")


class WSHandler(tornado.websocket.WebSocketHandler):
    def check_origin(self, origin):
        return True

    def open(self, *args, **kwargs):
        print("Connection Opened")
        uavVehicle = EladeVehicle('/dev/cu.usbmodem1',115200,'MAV001')

    def on_close(self):
        print("Connection closed")

    def on_message(self, message):
        print(message)


application = tornado.web.Application([
    (r'/', MainHandler),
    (r'/ws', WSHandler),
])


if __name__ == '__main__':
    application.listen(8888)
    tornado.ioloop.IOLoop.instance().start()
