import serial as _sOut

class Airplane:
    """Used for sending GPS coordinates to the airplane"""
    def __init__(self, serialOut: _sOut.Serial):
        self.__serial = serialOut
    def send_gps_coordinate(self, latitude: float, longitude: float, ignoreSerialOpen = False):
        """latitude is dd.mmmmm longitude is ddd.mmmmm """
        if self.__serial.is_open or ignoreSerialOpen:
            mOut = []
            mOut.append(1<<3)
            latitudeInt = int((latitude+90)*100000)
            longitudeInt = int((longitude+180)*100000)
            mOut[0]+=latitudeInt>>22
            mOut.append((latitudeInt>>14)&255)
            mOut.append((latitudeInt>>6)&255)
            mOut.append((latitudeInt<<2)&252)
            mOut[3]+=(longitudeInt>>24)
            mOut.append((longitudeInt>>16)&255)
            mOut.append((longitudeInt>>8)&255)
            mOut.append((longitudeInt)&255)
            mOut.append(0)
            if(ignoreSerialOpen):
                for i in mOut:
                    bOut = bin(i)[2:]
                    print('0'*(8-len(bOut)),bOut,sep='')
            else:
                self.__serial.write(bytes(mOut))
            return len(mOut)
        else:
            return -1
def decode_gps_message(messageIn: list):
    messageType = messageIn[0]>>3
    latitude = (messageIn[0]&7)<<22
    latitude+= messageIn[1]<<14
    latitude+=messageIn[2]<<6
    latitude+=messageIn[3]>>2
    longitude = (messageIn[3]&3)<<24
    longitude += messageIn[4]<<16
    longitude += messageIn[5]<<8
    longitude += messageIn[6]
    return [messageType, (latitude-9000000)/100000, (longitude-18000000)/100000]
if __name__ == "__main__":
    a = Airplane(_sOut.Serial())
    a.send_gps_coordinate(12.34567,76.54321,True)