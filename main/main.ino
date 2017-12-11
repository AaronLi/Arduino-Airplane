
#include <RH_RF95.h>
#include <Adafruit_GPS.h>

#include <Adafruit_Sensor.h>

#include <Adafruit_LSM9DS1.h>

#define RFM95_RST     11   // "A"
#define RFM95_CS      10   // "B"
#define RFM95_INT     6    // "D"
#define RF95_FREQ 915.0
#define mySerial Serial1

Adafruit_GPS GPS(&mySerial);

RH_RF95 rf95(RFM95_CS, RFM95_INT);
void setupRadio(){
  // Setup LoRa radio
  // code from: https://learn.adafruit.com/adafruit-feather-m0-radio-with-lora-radio-module/using-the-rfm-9x-radio
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  while(!Serial);
  Serial.begin(9600);
  delay(100);
  Serial.println("Feather LoRa TX Test!");
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  while(!rf95.init()){
    Serial.println("LoRa radio init failed");
    while(1);
  }
  Serial.println("LoRa radio init OK!");
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  rf95.setTxPower(23, false);
}

void setup() {
  setupRadio();
}

int16_t packetnum = 0;

void loop() {
  // put your main code here, to run repeatedly:
 if (rf95.available())
  {
    // Should be a message for us now   
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    
    if (rf95.recv(buf, &len))
    {
      digitalWrite(LED, HIGH);
      RH_RF95::printBuffer("Received: ", buf, len);
      Serial.print("Got: ");
      Serial.println((char*)buf);
       Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);
      delay(10);
      // Send a reply
      delay(200); // may or may not be needed
      uint8_t data[] = "And hello back to you";
      rf95.send(data, sizeof(data));
      rf95.waitPacketSent();
      Serial.println("Sent a reply");
      digitalWrite(LED, LOW);
    }
    else
    {
      Serial.println("Receive failed");
    }
  }
}
