#include <PID_v1.h>

#include <Adafruit_PWMServoDriver.h>

#include <Adafruit_Simple_AHRS.h>
#include <Adafruit_LSM9DS1.h>

#include <SPI.h>
#include <RH_RF95.h>
#include <Adafruit_GPS.h>



//Config
#define SHOW_ORIENTATION 0 // off for flight
#define SHOW_GPS 0 // off for flight
#define SHOW_RADIO 0 // off for flight
#define WAIT_FOR_SERIAL 0 // off for flight
#define WAIT_FOR_RADIO 1 // on for flight
#define CALIBRATE_ESC 1 // on for flight
#define PID_ON 0 // on for flight
#define DUMB_STABILIZE 1// off for flight

//IO setup
#define RF95_FREQ 915.0
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define mySerial Serial1

//Servo Limits
#define SERVOMIN 90 //Should be pretty accurate
#define SERVOMAX 460
#define AILERONMAX 277
#define AILERONMIN 195
#define ELEVATORMAX 166 // Actual max is 176
#define ELEVATORMIN 95

//Outputs
#define LED 13
#define LEFT_AILERON 0
#define RIGHT_AILERON 2
#define ELEVATOR 15
#define THROTTLE 6

//<155 propeller braking?
//354 minimum speed
//460 maximum speed
sensors_vec_t orientation;
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
uint32_t pidTimer = millis();
uint8_t sensor_status = 0;
uint8_t GPS_status = 0;
uint8_t radio_status = 0;
uint8_t noRollTime, noPitchTime;
bool ledState = false;
int sensorHz = 10;
int motorSpeeds[] = {90,155,354,360}; // testing values for safety purposes
int throttleTarget = 0;
int currentThrottle = 0;
uint8_t defaultPitch, defaultRoll; //values of the joystick when it isn't being touched
double rollValue, rollSetpoint, aileronValue; // used for roll based PID
double pitchValue, pitchSetpoint, elevatorValue;
#if PID_ON == 1 && DUMB_STABILIZE == 0
PID rollPID(&rollValue, &aileronValue, &rollSetpoint, 1.5, 2, 1, DIRECT);
PID pitchPID(&pitchValue, &elevatorValue, &pitchSetpoint, 1.5, 2, 1, DIRECT);
#endif

struct ManualRadioOut{
  uint8_t messageType;
  uint8_t throttleValue;
  uint8_t rollValue;
  uint8_t pitchValue;
  uint8_t peripheral1Type;
  uint8_t peripheral2Type;
};
ManualRadioOut lastManualMessage;
void writeServo(int channel,int angle){
  int writeAngle = map(angle,0,180,SERVOMIN,SERVOMAX);
  pwmDriver.setPWM(channel,0,writeAngle);
}
void writeAileron(int angle){
  int writeAngle = map(angle,0,180,AILERONMIN, AILERONMAX);
  pwmDriver.setPWM(LEFT_AILERON, 0, writeAngle+10);
  pwmDriver.setPWM(RIGHT_AILERON, 0, writeAngle);
}
void writeElevator(int angle){
  int writeAngle = map(angle,0,180,ELEVATORMIN, ELEVATORMAX);
  pwmDriver.setPWM(ELEVATOR,0,writeAngle);
}
ManualRadioOut readRadioMessage(uint8_t buf[]){
          uint8_t mType = buf[0]>>3;
          uint8_t throttleValue = (buf[0]&6)>>1;
          uint8_t rollValue = (double)((buf[0]&1)<<7)+(buf[1]>>1);
          uint8_t pitchValue = ((buf[1]&1)<<7)+(buf[2]>>1);
          uint8_t peripheral1Type = buf[2]&1;
          uint8_t peripheral2Type = (buf[3]&4)>>2;
          return ManualRadioOut{mType, throttleValue, rollValue, pitchValue, peripheral1Type, peripheral2Type};
}
void notifyUser(){
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




void setupSensor()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
}
void setupPID(){
  #if PID_ON == 1 && DUMB_STABILIZE == 0
  rollSetpoint = 0; // Straight up, replace as needed
  rollPID.SetOutputLimits(0,180);
  rollPID.SetMode(AUTOMATIC);
  
  pitchSetpoint = 0;
  pitchPID.SetOutputLimits(0,180);
  pitchPID.SetMode(AUTOMATIC);
  #endif
}
void setup() 
{
  pinMode(LED, OUTPUT);     
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  #if WAIT_FOR_SERIAL == 1
  while (!Serial); // wait for serial if needed
  #else
  delay(5000);
  #endif
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
  delay(100);
  if (!lsm.begin()){
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }
  Serial.println("Found LSM9DS1 9DOF");
  setupSensor();
  pwmDriver.begin();
  pwmDriver.setPWMFreq(50);
  setupPID();
  #if CALIBRATE_ESC == 1
  Serial.println("Calibrating ESC...");
  calibrateESC();
  Serial.println("ESC calibrated");
  #endif
  safetyTimer = millis();
  #if WAIT_FOR_RADIO == 1
  Serial.println("Finding rest position of controller...");
  while(!rf95.available()){
  }
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  rf95.recv(buf, &len);
  ManualRadioOut rMessage = readRadioMessage(buf);
  defaultPitch = rMessage.pitchValue;
  defaultRoll = rMessage.rollValue;
  #else
  defaultPitch = 90;
  defaultRoll = 90;
  aileronValue = 90;
  lastManualMessage = {0,0,90,90,0,0};
  #endif
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
      #if SHOW_RADIO == 1
      Serial.print("MessageType: ");
      Serial.print(mType);
      #endif
      switch(mType){
        case 0: //will be changed once full program is outlined
          lastManualMessage= readRadioMessage(buf);
          uint8_t throttleValue = lastManualMessage.throttleValue;
          aileronValue = (double)lastManualMessage.rollValue;
          elevatorValue = (double)lastManualMessage.pitchValue;
          uint8_t peripheral1Type = lastManualMessage.peripheral1Type;
          uint8_t peripheral2Type = lastManualMessage.peripheral2Type;
          throttleTarget = motorSpeeds[lastManualMessage.throttleValue];
          safetyTimer = millis();
          #if SHOW_RADIO == 1
          Serial.print(", Throttle: ");
          Serial.print(throttleValue);
          Serial.print(", PWM: ");
          Serial.print(motorSpeeds[throttleValue]);
          Serial.print(", Pitch: ");
          Serial.print(aileronValue);
          Serial.print(", Roll: ");
          Serial.println(elevatorValue);
          #endif
          break;
      }
      #if SHOW_RADIO == 1
      Serial.println();
      Serial.println(rf95.lastRssi(), DEC);
      #endif
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
        #if SHOW_GPS == 1
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
        #endif
      }
    }
  }
  //~~~~~~~~~~~~~~~~~~~~SENSE ARRAY INPUT~~~~~~~~~~~~~~~~~~
  ahrs.getOrientation(&orientation);
  rollValue = (orientation.roll+180)<180?180+orientation.roll:orientation.roll-180;//reverses the roll (sensor is upside down)
  pitchValue = orientation.pitch;
  if (senseTimer > millis())  senseTimer = millis();
  if(millis()-senseTimer>1000/sensorHz){
    senseTimer = millis();
    #if SHOW_ORIENTATION == 1
      Serial.print("Roll: "); Serial.print(rollValue); Serial.print(" Pitch: ");Serial.print(orientation.pitch);Serial.print(" Yaw: ");Serial.println(orientation.heading);
    #endif
  }
  //~~~~~~~~~~~~~~~~~~~~~~~~~FINE THROTTLE~~~~~~~~~~~~~~~~~~~~~~
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
  //~~~~~~~~~PLANE TO CONTROLLER COMMUNICATION~~~~~~~~
  if(radioTimer>millis()) radioTimer = millis();
  if(millis()-radioTimer>1000){ // send radio message every second
    radioTimer = millis();
    //send gps coordinates and sensor status here
    uint8_t data[7];
    //if it's possible to find if the sensors are working then put that here
    sensor_status = 0;
    radio_status = 1; // if you can read this then the radio should be working...
    GPS_status = GPS.satellites>4;
    Serial.print("GPS status: ");
    Serial.println(GPS_status);
    rf95.send(data, sizeof(data));
  }
//~~~~~~~~~~~~SAFE MOTOR OFF~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  if(safetyTimer>millis()) safetyTimer = millis();
  if(millis()-safetyTimer>2000){ //if radio message hasn't been received for 2 seconds
    throttleTarget = 155; // set propeller to 0
    Serial.println("No radio, stopping motor...");
    safetyTimer = millis();
  }
  //Serial.print(rollValue);Serial.print(" ");Serial.print(defaultRoll);Serial.print(" ");Serial.println(noRollTime);
  //Serial.println(lastManualMessage.pitchValue);
  Serial.println("Before PID");
  #if PID_ON == 1
  if(pidTimer>millis()) pidTimer = millis();
  if(millis()-pidTimer>100){
    pidTimer = millis();
    if(lastManualMessage.rollValue == defaultRoll){
      if(noRollTime<5){
        noRollTime++;
      }
      else{
        #if DUMB_STABILIZE == 1
        if(rollValue>5){
          writeAileron(70);
        }
        else if(rollValue<-5){
          writeAileron(110);
        }
        else{
          writeAileron(90);
        }
        #else
        if(rollPID.Compute()){
          /*Serial.print("Aileron value: ");
          Serial.println(aileronValue);*/
        }
        #endif
      }
    }
    else{
      noRollTime = 0;
    }
    if(lastManualMessage.pitchValue == defaultPitch){
      if(noPitchTime<5){
        noPitchTime++; 
      }
      else{
        //Serial.println("pitchPID");
        #if DUMB_STABILIZE == 1
        if(pitch>5){
          writeElevator(110);
        }
        else if(pitch<-5){
          writeElevator(70);
        }
        else{
          writeAileron(90);
        }
        #else
        if(pitchPID.Compute()){
        }
        #endif
      }
    }
    else{
      noPitchTime = 0; 
    }
  }
  #endif
  Serial.println("After PID");
  writeAileron((int)aileronValue);
  writeElevator((int)elevatorValue);
  //Serial.println(currentThrottle);
  if(ledTimer>millis()) ledTimer = millis();
  if(millis()-ledTimer>1000){//blink led to show program is running
    ledTimer = millis();
    digitalWrite(LED, ledState);
    ledState = !ledState;
  }
}
