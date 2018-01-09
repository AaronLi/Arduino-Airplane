#include <Adafruit_PWMServoDriver.h>

#include <Adafruit_Simple_AHRS.h>
#include <Adafruit_LSM9DS1.h>

#include <SPI.h>
#include <RH_RF95.h>
#include <Adafruit_GPS.h>

#define RF95_FREQ 915.0
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

#define mySerial Serial1
#define SERVOMIN 90 //Should be pretty accurate
#define SERVOMAX 460
#define AILERONMAX 317
#define AILERONMIN 265
#define ELEVATORMAX 176
#define ELEVATORMIN 95
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
uint32_t throttleTimer = millis();
uint32_t radioTimer = millis();
uint32_t safetyTimer = millis();
uint8_t sensor_status = 0;
uint8_t GPS_status = 0;
uint8_t radio_status = 0;

bool ledState = false;
int sensorHz = 10;
int motorSpeeds[] = {90,155,354,360}; // testing values for safety purposes
int throttleTarget = 0;
int currentThrottle = 0;
#define SHOW_ORIENTATION false
#define SHOW_GPS false
#define SHOW_RADIO false

#define LED 13
#define LEFT_AILERON 0
#define RIGHT_AILERON 2
#define ELEVATOR 15
#define THROTTLE 6


void writeServo(int channel,int angle){
  int writeAngle = map(angle,0,180,SERVOMIN,SERVOMAX);
  pwmDriver.setPWM(channel,0,writeAngle);
}
void writeAileron(int angle){
  int writeAngle = map(angle,0,180,AILERONMIN, AILERONMAX);
  pwmDriver.setPWM(LEFT_AILERON, 0, writeAngle);
  pwmDriver.setPWM(RIGHT_AILERON, 0, writeAngle);
}
void writeElevator(int angle){
  int writeAngle = map(angle,0,180,ELEVATORMIN, ELEVATORMAX);
  pwmDriver.setPWM(ELEVATOR,0,writeAngle);
}
void calibrateESC(){
  //Serial.println("Calibration started, setting max");
  pwmDriver.setPWM(THROTTLE,0,SERVOMAX); 
  delay(8000);
  //Serial.println("ESC should have max now, setting min");
  pwmDriver.setPWM(THROTTLE,0,SERVOMIN);
  delay(3000);
  //Serial.println("ESC should have min now");
  notifyUser();
  //Serial.println("Servo callibration complete! Please set throttle to 0");
  //while(analogRead(0)>0);
  //Wait for throttle to be set to 0
}

void notifyUser(){
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
  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
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
  delay(100);
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  rf95.setTxPower(23, false);
  //~~~~~~~~~~START SENSOR ARRAY~~~~~~~~~~~~~~
  Serial.println("Starting sensor array...");
  if (!lsm.begin()){
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }
  Serial.println("Found LSM9DS1 9DOF");
  setupSensor();
  pwmDriver.begin();
  pwmDriver.setPWMFreq(50);
  Serial.println("Calibrating ESC...");
  calibrateESC();
  safetyTimer = millis();
}

void loop()
{
   //~~~~~~~~~~~~~~~~~~~~~~~~~RADIO INPUT~~~~~~~~~~~~~~~~~~~~~
  if (rf95.available()){
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    
    if (rf95.recv(buf, &len))
    {
      uint8_t mType = buf[0]>>3;
      Serial.print("MessageType: ");
      Serial.print(mType);
      switch(mType){
        case 0: //will be changed once full program is outlined
          uint8_t throttleValue = (buf[0]&6)>>1;
          uint8_t rollValue = ((buf[0]&1)<<7)+(buf[1]>>1);
          uint8_t pitchValue = ((buf[1]&1)<<7)+(buf[2]>>1);
          uint8_t peripheral1Type = buf[2]&1;
          uint8_t peripheral2Type = (buf[3]&4)>>2;
          throttleTarget = motorSpeeds[throttleValue];
          writeAileron(rollValue);
          writeElevator(pitchValue);
          safetyTimer = millis();
          Serial.print(", Throttle: ");
          Serial.print(throttleValue);
          Serial.print(", PWM: ");
          Serial.print(motorSpeeds[throttleValue]);
          Serial.print(", Pitch: ");
          Serial.print(pitchValue);
          Serial.print(", Roll: ");
          Serial.println(rollValue);
      
          break;
      }
      Serial.println();
      if(SHOW_RADIO){
          Serial.println(rf95.lastRssi(), DEC);
      }
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
  if(throttleTimer>millis()) throttleTimer = millis();
  if(millis()-throttleTimer>5){
    throttleTimer = millis();
    if(currentThrottle<throttleTarget){
      currentThrottle++;
    }
    else if(currentThrottle>throttleTarget){
      currentThrottle--;
    }
  }
  pwmDriver.setPWM(THROTTLE, 0, currentThrottle);
  if(radioTimer>millis()) radioTimer = millis();
  if(millis()-radioTimer>1000){ // send radio message every second
    radioTimer = millis();
    //send gps coordinates and sensor status here
    uint8_t data[7];
    //if it's possible to find if the sensors are working then put that here
    sensor_status = 0;
    radio_status = 0;
    GPS_status = GPS.satellites>4;
    
    rf95.send(data, sizeof(data));
  }
  if(safetyTimer>millis()) safetyTimer = millis();
  if(millis()-safetyTimer>2000){ //if radio message hasn't been received for 2 seconds
    throttleTarget = 155; // set propeller to 0
    safetyTimer = millis();
  }
  //Serial.println(currentThrottle);
  if(ledTimer>millis()) ledTimer = millis();
  if(millis()-ledTimer>1000){//blink led to show program is running
    ledTimer = millis();
    digitalWrite(LED, ledState);
    ledState = !ledState;
  }
}
