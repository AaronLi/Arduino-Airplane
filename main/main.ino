#include <Adafruit_PWMServoDriver.h>

#include <Adafruit_Simple_AHRS.h>
#include <Adafruit_LSM9DS1.h>

#include <SPI.h>
#include <RH_RF95.h>
#include <Adafruit_GPS.h>

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

#define mySerial Serial1
#define SERVOMIN 90 //Should be pretty accurate
#define SERVOMAX 460


//<155 propeller braking?
//354 minimum speed
//460 maximum speed

RH_RF95 rf95(RFM95_CS, RFM95_INT);
Adafruit_GPS GPS(&mySerial);
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
Adafruit_Simple_AHRS ahrs(&lsm.getAccel(), &lsm.getMag());
Adafruit_PWMServoDriver pwmDriver = Adafruit_PWMServoDriver();
uint32_t timer = millis();
uint32_t senseTimer = millis();
uint32_t ledTimer = millis();
bool ledState = false;
int sensorHz = 10;

#define SHOW_ORIENTATION false
#define SHOW_GPS true
#define SHOW_RADIO false

#define LED 13




void writeServo(int channel,int angle){
  int writeAngle = map(angle,0,180,SERVOMIN,SERVOMAX);
  pwmDriver.setPWM(channel,0,writeAngle);
}




void setupSensor()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
}

void setup() 
{
  pinMode(LED, OUTPUT);     
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  //while (!Serial); // wait for serial if needed
  Serial.begin(115200);
  //~~~~~~~~~~~~~~~~~~~~~~~~START GPS~~~~~~~~~~~~~~~~~~~~
  GPS.begin(9600);
  mySerial.begin(9600);
  delay(100);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  //~~~~~~~~~~~~~START RADIO~~~~~~~~~~~~
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  rf95.setTxPower(23, false);
  //~~~~~~~~~~START SENSOR ARRAY~~~~~~~~~~~~~~
  if (!lsm.begin()){
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }
  Serial.println("Found LSM9DS1 9DOF");
  setupSensor();
  pwmDriver.begin();
  pwmDriver.setPWMFreq(50);
}

void loop()
{
   //~~~~~~~~~~~~~~~~~~~~~~~~~RADIO INPUT~~~~~~~~~~~~~~~~~~~~~
  if (rf95.available()){
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    
    if (rf95.recv(buf, &len))
    {
      writeServo(0,((int)buf[0])-32);
      writeServo(2,((int)buf[0])-32);
      writeServo(4,((int)buf[1])-32);
      if(SHOW_RADIO){
        Serial.println(rf95.lastRssi(), DEC);
      }
      uint8_t data[] = "dmfg";
      rf95.send(data, sizeof(data));
    }
    else
    {
      Serial.println("Receive failed");
    }
  }
  // ~~~~~~~~~~~~~~~~~~~~~~~~~GPS INPUT~~~~~~~~~~~~~~~~~~~~~~~~
  if (timer > millis())  timer = millis();
  char c = GPS.read();
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA())){   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
    }
    else{
      if (millis() - timer > 2000 && GPS.fix) { 
        timer = millis(); // reset the timer
        if(SHOW_GPS){
          Serial.print("\nTime: ");
          Serial.print(GPS.hour, DEC); Serial.print(':');
          Serial.print(GPS.minute, DEC); Serial.print(':');
          Serial.print(GPS.seconds, DEC); Serial.print('.');
          Serial.println(GPS.milliseconds);
          Serial.print("Date: ");
          Serial.print(GPS.day, DEC); Serial.print('/');
          Serial.print(GPS.month, DEC); Serial.print("/20");
          Serial.println(GPS.year, DEC);
          Serial.print("Fix: "); Serial.print((int)GPS.fix);
          Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
          Serial.print("Location: ");
          Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
          Serial.print(", "); 
          Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
          
          Serial.print("Speed (knots): "); Serial.println(GPS.speed);
          Serial.print("Angle: "); Serial.println(GPS.angle);
          Serial.print("Altitude: "); Serial.println(GPS.altitude);
          Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
          Serial.println();
        }
      }
    }
  }
  //~~~~~~~~~~~~~~~~~~~~SENSE ARRAY INPUT~~~~~~~~~~~~~~~~~~
  if (senseTimer > millis())  senseTimer = millis();
  if(millis()-senseTimer>1000/sensorHz){
    senseTimer = millis();
    sensors_vec_t orientation;
    if(ahrs.getOrientation(&orientation)&&SHOW_ORIENTATION){
    Serial.print("Roll: "); Serial.print(orientation.roll); Serial.print(" Pitch: ");Serial.print(orientation.pitch);Serial.print(" Yaw: ");Serial.println(orientation.heading);
    }
  }
  if(ledTimer>millis()) ledTimer = millis();
  if(millis()-ledTimer>1000){
    ledTimer = millis();
    digitalWrite(LED, ledState);
    ledState = !ledState;
  }
}
