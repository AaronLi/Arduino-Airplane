import os
import webbrowser
import tornado.ioloop
import tornado.web
import tornado.websocket
import serial
import serial.tools.list_ports
import serial.serialutil
callbackHz = 30
readData = b""
ser = None
def connectSerial():
    global ser
    availablePorts = serial.tools.list_ports.comports()
    print("Ports:")
    for i in availablePorts:
        print("\t%s, %s" % (i.device, i.description))
    if len(availablePorts)>0:
        try:
            print("Attempting to connect to port", availablePorts[0].device)
            ser = serial.Serial(availablePorts[0].device, 115200)
            print("\tIsOpen:", ser.is_open, "port:", ser.name)
        except serial.serialutil.SerialException:
            print("\tPort unavailable")

class MainHandler(tornado.web.RequestHandler):
    def get(self, *args, **kwargs):
        self.render("index.html")
class WebSocketHandler(tornado.websocket.WebSocketHandler):
    def open(self, *args, **kwargs):
        print("Websocket opened",args,kwargs)
    def on_close(self):
        print("Websocket closed")

    def on_message(self, message):
        print("Message received",message)

def handleSerial():
    global readData
    if ser != None and ser.is_open:
        try:
            readData = ser.readline()
            print('dataIn:',readData)
        except serial.serialutil.SerialException:
            return
            print("Device has probably been disconnected")
            connectSerial()
    else:
        return
        #print("Serial port is not open")
        connectSerial()
connectSerial()
if __name__ == "__main__":
    tornado.ioloop.PeriodicCallback(handleSerial, 1000/callbackHz).start()
    application = tornado.web.Application([
        (r"/static/(.*)", tornado.web.StaticFileHandler, {'path': 'static'}),
        (r"/", MainHandler),
        (r"/ws",WebSocketHandler)
    ])
    application.listen(8888)
    print("Starting Server...")
    webbrowser.open('http://localhost:8888')
    tornado.ioloop.IOLoop.current().start()