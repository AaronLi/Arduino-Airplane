#include <LiquidCrystal_I2C.h>

#include <Nunchuk.h>



//The current radio controller
#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS 8
#define RFM95_RST 9
#define RFM95_INT 7

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0
#define PACKET_SIZE 7
// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);
Nunchuk nck;
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
  //while (!Serial);
  Serial.begin(115200);
  lcd.begin(16, 2);
  lcd.home();
  lcd.clear();
  lcd.noCursor();
  delay(100);

  Serial.println("Arduino LoRa TX Remote!");
  lcd.print("Resetting radio");
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(500);
  Serial.println("Reset radio");
  lcd.setCursor(0, 1);
  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    lcd.print("Failed");
    while (1);
  }
  lcd.print("Success!");
  Serial.println("LoRa radio init OK!");
  delay(500);
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  lcd.home();
  lcd.clear();
  lcd.print("Setting freq");
  delay(500);
  lcd.setCursor(0, 1);
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    lcd.print("Failed");
    while (1);
  }
  lcd.print("Success! ");
  lcd.print((int)RF95_FREQ);
  lcd.print("MHz");
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  delay(500);
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
  lcd.clear();
}

void loop()
{
  // Send a message to rf95_server
  nck.update();
  uint8_t radiopacket[PACKET_SIZE];
  uint8_t messageType = 0;
  uint8_t rollValue = map(nck.joystick_x(), 27, 229, 0, 180);
  uint8_t pitchValue = map(nck.joystick_y(), 33, 225, 0, 180);
  pitchValue = 180 - pitchValue;
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
  radiopacket[2] = (pitchValue << 1);
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
  rf95.send((uint8_t *)radiopacket, PACKET_SIZE);
  if (rf95.available()) {
    // Should be a reply message for us now
    // Now wait for a reply
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf95.recv(buf, &len))
    {
      long longitudeIn, latitudeIn;
      longitudeIn += ((buf[0] & 7) << 22) + (buf[1] << 14) + (buf[2] << 6) + buf[3] >> 2;
      latitudeIn += ((buf[3] & 3) << 24) + (buf[4] << 16) + (buf[5] << 8) + buf[6];
      longitudeIn -= 18000000;
      latitudeIn -= 9000000;
      lcd.setCursor(0, 1);
      lcd.print("reply: ");
      lcd.print((char*)buf);
      Serial.print("Received: "); Serial.print(latitudeIn); Serial.print(" "); Serial.println(longitudeIn);
      //Serial.print("RSSI: ");
      //Serial.println(rf95.lastRssi(), DEC);
    }
    else
    {
      Serial.println("Receive failed");
    }
  }
  rf95.waitPacketSent();
  //Serial.print(".");
}
