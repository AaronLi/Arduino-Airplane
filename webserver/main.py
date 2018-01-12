import os
import webbrowser
import tornado.ioloop
import tornado.web
import tornado.websocket
import serial
import serial.tools.list_ports
import serial.serialutil
callbackHz = 20
readData = b""
ser = None
client = None#one client to 
def connectSerial():
    global ser
    availablePorts = serial.tools.list_ports.comports()
    print("Ports:")
    for i in availablePorts:
        print("\t%s, %s" % (i.device, i.description))
    if len(availablePorts)>0:
        try:
            print("Attempting to connect to port", availablePorts[0].device)
            ser = serial.Serial(availablePorts[0].device, 115200, timeout=1/callbackHz)
            print("\tIsOpen:", ser.is_open, "port:", ser.name)
        except serial.serialutil.SerialException:
            print("\tPort unavailable")

class MainHandler(tornado.web.RequestHandler):
    def get(self, *args, **kwargs):
        if client == None:
            print("accessed")
            self.render("index.html")
        else:
            print("client rejected")
            self.write("Already a client connected")

class WebSocketHandler(tornado.websocket.WebSocketHandler):
    def open(self, *args, **kwargs):
        global client
        print("Websocket opened",args,kwargs)
        client = self
    def on_close(self):
        global client
        print("Websocket closed")
        client = None
    def on_message(self, message):
        print("Message received: ",message)
        smessages = message.split()
        if smessages[0] == 'Position:':
            coords.append(smessage[1])

def handleSerial():
    global readData
    if ser != None and ser.is_open:
        try:
            readData = ser.readline()
            if len(readData) != 0:
                print('dataIn:',readData)
                if client != None:
                    client.write_message(readData)
        except serial.serialutil.SerialException:
            #return
            print("Device has probably been disconnected")
            connectSerial()
    else:
        #return
        #print("Serial port is not open")
        connectSerial()
connectSerial()


if __name__ == "__main__":
    tornado.ioloop.PeriodicCallback(handleSerial, 1000/callbackHz).start()
    application = tornado.web.Application([
        (r"/static/(.*)", tornado.web.StaticFileHandler, {'path': 'static'}),
        (r"/", MainHandler),
        (r"/ws",WebSocketHandler),
        ]
    )
    application.listen(8888)
    print("Starting Server...")
    webbrowser.open('http://localhost:8888')
    tornado.ioloop.IOLoop.current().start()
