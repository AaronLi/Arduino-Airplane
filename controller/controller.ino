 //The current radio controller
#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS 4
#define RFM95_RST 3
#define RFM95_INT 2

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0
#define PACKET_SIZE 7
// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);
void setup() 
{
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  //while (!Serial);
  Serial.begin(115200);
  delay(100);

  Serial.println("Arduino LoRa TX Test!");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(500);
  Serial.println("Reset radio");
  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(5, false);
}

void loop()
{
    // Send a message to rf95_server
    uint8_t radiopacket[PACKET_SIZE];
    uint8_t messageType = 0;
    uint8_t rollValue = map(analogRead(0), 0, 1023, 0, 180);
    uint8_t pitchValue = map(analogRead(1),0,1023,0,180);
    uint8_t throttleValue = map(analogRead(2),0,1023,0,3);
    radiopacket[0] = (messageType<<3)+(throttleValue<<1)+(rollValue>>7);
    radiopacket[1] = (rollValue<<1)+(pitchValue>>7);
    radiopacket[2] = (pitchValue<<1);
    Serial.println(throttleValue);
    rf95.send((uint8_t *)radiopacket, PACKET_SIZE);
    if (rf95.available()){ 
    // Should be a reply message for us now   
    // Now wait for a reply
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf95.recv(buf, &len))
   {
      Serial.print("Got reply: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);    
    }
    else
    {
      Serial.println("Receive failed");
    }
  }
  rf95.waitPacketSent();
  //Serial.print(".");
}
