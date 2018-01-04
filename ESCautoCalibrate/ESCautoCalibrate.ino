#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver servos = Adafruit_PWMServoDriver();
#define SERVOMIN 90 //Should be pretty accurate
#define SERVOMAX 460
#define SAFE_MODE false
void calibrateESC(){
  Serial.println("Calibration started, setting max");
  servos.setPWM(6,0,SERVOMAX); 
  delay(8000);
  Serial.println("ESC should have max now, setting min");
  servos.setPWM(6,0,SERVOMIN);
  delay(3000);
  Serial.println("ESC should have min now");
  notifyUser();
  Serial.println("Servo callibration complete! Please set throttle to 0");
  while(analogRead(0)>0);
}
void notifyUser(){
  Serial.println("Replace this with a hardware function, ESC is ready!");
}
void setup() {
  // put your setup code here, to run once:
  servos.begin();
  pinMode(13,INPUT_PULLUP);
  servos.setPWMFreq(50);
  Serial.begin(9600);
  //while(!Serial);
  Serial.println("Beginning ESC callibration");
  calibrateESC();

}

void loop() {
    int sVal = SERVOMIN;
    if(SAFE_MODE){
      sVal = map(analogRead(0),0,1023,155,356);
    }
    else{
      sVal = map(analogRead(0),0,1023,155,SERVOMAX);
    }
    Serial.println(sVal);
    servos.setPWM(6,0,sVal);
    //servos.setPWM(2,0,sVal);
  }
  // put your main code here, to run repeatedly:
