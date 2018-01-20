import os
import webbrowser
import tornado.ioloop
import tornado.web
import tornado.websocket
import serial
import serial.tools.list_ports
import serial.serialutil
import airplane
callbackHz = 20
readData = b""
ser = None
client = None#one client to
coords = []
plane = None
def connectSerial():
    global ser, plane
    availablePorts = serial.tools.list_ports.comports()
    print("Ports:")
    for i in availablePorts:
        print("\t%s, %s" % (i.device, i.description))
    if len(availablePorts)>0:
        try:
            print("Attempting to connect to port", availablePorts[0].device)
            ser = serial.Serial(availablePorts[0].device, 115200, timeout=1/callbackHz)
            plane = airplane.Airplane(ser)
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
            self.write('''
Client already connected
<br/>
Think this is an error? <i> You might be right</i>
''')

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
            coords.append(smessages[1])
        #temp
        if message != 'Hello!':
            latLongOut = [float(i) for i in message.replace('(','').replace(')','').split(',')]
            print(latLongOut)
            plane.send_gps_coordinate(latLongOut[0],latLongOut[1])

def handleSerial():
    global readData
    if ser != None and ser.is_open:
        try:
            readData = ser.readline()
            try:
                readData = readData.decode('utf-8')
            except UnicodeDecodeError:
                pass
            if len(readData) != 0:
                print('serial:',readData.strip())
                if client != None and not str(readData)[0] in {"+", "|"}:
                    dataIn = str(list(readData[:-2])).replace('[','').replace(']','').replace(',','') #remove the \r\n
                    print('decoded data:',dataIn,'\n^ compare with hex being written to arduino ^')
                    client.write_message(dataIn) # bytes represented in decimal
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
