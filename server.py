__author__ = 'francoischevalier'
import tornado.ioloop
import tornado.web


class MainHandler(tornado.web.RequestHandler):
    def get(self):
        self.write("Hello, World")


application = tornado.web.Application([])