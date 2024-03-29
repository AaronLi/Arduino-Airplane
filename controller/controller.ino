#include <LiquidCrystal_I2C.h>

#include <Nunchuk.h>



//The current radio controller
#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS 8
#define RFM95_RST 9
#define RFM95_INT 7
//#define WAIT_FOR_SERIAL
// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0
#define PACKET_SIZE 7
#define DELAY_TIME 10
// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);
Nunchuk nck;
uint8_t noSerialCount = 0;
uint32_t serialTimer = millis();
int throttleValues = 3;
uint8_t throttle = 0;
boolean cPressed = false;
boolean zPressed = false;
LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
void setup()
{
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  nck.initialize();
  lcd.begin(16, 2);
  lcd.home();
  lcd.clear();
  lcd.noCursor();
#ifdef WAIT_FOR_SERIAL
  while (!Serial) {
    lcd.home();
    lcd.println("Awaiting SerialI");
    delay(200);
    lcd.home();
    lcd.println("Awaiting Serial*");
    delay(200);
  }
  lcd.home();
  lcd.clear();
#endif
  Serial.begin(115200);
  delay(100);

  Serial.println("+ Arduino LoRa TX Remote initialized!");
  lcd.print("Resetting radio");
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  //delay(500);
  Serial.println("| Reset radio");
  lcd.setCursor(0, 1);
  while (!rf95.init()) {
    Serial.println("| LoRa radio init failed");
    lcd.print("Failed");
    while (1);
  }
  lcd.print("Success!");
  Serial.println("| LoRa radio init OK!");
  //delay(500);
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  lcd.home();
  lcd.clear();
  lcd.print("Setting freq");
  //delay(500);
  lcd.setCursor(0, 1);
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("| setFrequency failed");
    lcd.print("Failed");
    while (1);
  }
  lcd.print("Success! ");
  lcd.print((int)RF95_FREQ);
  lcd.print("MHz");
  lcd.clear();
  lcd.print("Connecting Plane...");
  Serial.print("| Set Freq to: "); Serial.println(RF95_FREQ);
  //delay(500);
  rf95.setTxPower(23, false);
  lcd.clear();
}

void loop()
{
  // Send a message to rf95_server
  if (serialTimer > millis()) serialTimer = millis();
  if (millis() - serialTimer > 100) {
    serialTimer = millis();
    if (noSerialCount == DELAY_TIME - 1) {
      lcd.clear();
    }
    noSerialCount = min(noSerialCount + 1, DELAY_TIME);
  }
  if (Serial.available() > 0) {
    noSerialCount = 0;
    byte sIn[20];
    for (int i = 0; i < sizeof(sIn); i++) {
      sIn[i] = 0;
    }
    Serial.readBytesUntil(0, sIn, 20);
    Serial.flush();
    rf95.send(sIn, sizeof(sIn));
    rf95.waitPacketSent();
    lcd.home();
    lcd.clear();
    lcd.print("Serial mode");
    lcd.setCursor(0, 1);
    for (int i = 0; i < sizeof(sIn); i++) {
      if (sIn[i] == 0) {
        break;
      }
      Serial.print((char)sIn[i]);
      lcd.write(sIn[i]);
    }
    Serial.println();
  }
  else if (noSerialCount == DELAY_TIME) {
    nck.update();
    uint8_t radiopacket[PACKET_SIZE];
    uint8_t messageType = 0;
    uint8_t yawValue = map(nck.joystick_x(), 27, 229, 0, 180);//TODO: switch with roll
    uint8_t pitchValue = map(nck.joystick_y(), 27, 229, 0, 180);
    uint8_t rollValue = map(nck.x_acceleration(), 250, 750, 0, 180);
    //pitchValue = 180-pitchValue;
    if (nck.c_button()) {
      if (!cPressed) {
        cPressed = true;
        if (throttle < throttleValues) {
          throttle++;
        }
      }
    }
    else {
      cPressed = false;
    }
    if (nck.z_button()) {
      if (!zPressed) {
        zPressed = true;
        if (throttle > 0) {
          throttle--;
        }
      }
    }
    else {
      zPressed = false;
    }
    radiopacket[0] = (messageType << 3) + (throttle << 1) + (rollValue >> 7);
    radiopacket[1] = (rollValue << 1) + (pitchValue >> 7);
    radiopacket[2] = (pitchValue << 1) + (yawValue >> 7);
    radiopacket[3] = (yawValue << 1);
    lcd.home();
    lcd.print("                ");
    lcd.home();
    lcd.print("T");
    lcd.print(throttle);
    lcd.print(" X");
    lcd.print(rollValue);
    lcd.setCursor(7, 0);
    lcd.print(" Y");
    lcd.print(pitchValue);
    lcd.print(" Z");
    lcd.print(yawValue);
    rf95.send((uint8_t *)radiopacket, PACKET_SIZE);
    rf95.waitPacketSent();
    if (rf95.available()) {
      //head
      // Should be a reply message for us now
      // Now wait for a reply
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);
      if (rf95.recv(buf, &len))
      {
        Serial.println("Message Received!");
        long longitudeIn, latitudeIn;
        longitudeIn += ((buf[0] & 7) << 22) + (buf[1] << 14) + (buf[2] << 6) + buf[3] >> 2;
        latitudeIn += ((buf[3] & 3) << 24) + (buf[4] << 16) + (buf[5] << 8) + buf[6];
        longitudeIn -= 18000000;
        latitudeIn -= 9000000;
        lcd.setCursor(0, 1);
        for (int i = 0; i < len; i++) {
          lcd.print(buf[i]);
          //=======
          //      // Should be a reply message for us now
          //      // Now wait for a reply
          //      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
          //      uint8_t len = sizeof(buf);
          //      if (rf95.recv(buf, &len)){
          //        long longitudeIn, latitudeIn;
          //        longitudeIn += ((buf[0] & 7) << 22) + (buf[1] << 14) + (buf[2] << 6) + buf[3] >> 2;
          //        latitudeIn += ((buf[3] & 3) << 24) + (buf[4] << 16) + (buf[5] << 8) + buf[6];
          //        longitudeIn -= 18000000;
          //        latitudeIn -= 9000000;
          //        lcd.setCursor(1, 0);
          //        lcd.print("reply: ");
          //        lcd.print((char*)buf);
          //        Serial.print("Received: "); Serial.print(latitudeIn); Serial.print(" "); Serial.println(longitudeIn);
          //        longitudeIn+=((buf[0]&7)<<22)+(buf[1]<<14)+(buf[2]<<6)+buf[3]>>2;
          //        latitudeIn+=((buf[3]&3)<<24)+(buf[4]<<16)+(buf[5]<<8)+buf[6];
          //        longitudeIn-=18000000;
          //        latitudeIn-=9000000;
          //        lcd.setCursor(0,1);
          //        for(int i = 0;i<len;i++){
          //          lcd.print(buf[i]);
          //        }
          //        Serial.print(latitudeIn);Serial.print(", ");Serial.println(longitudeIn);
          //        //Serial.print("RSSI: ");
          //        //Serial.println(rf95.lastRssi(), DEC);
          //      }
          //      else{
          //        //Serial.println("Receive failed");
          //>>>>>>> origin/master
        }
      }
    }
    //Serial.print(".");
  }
}
