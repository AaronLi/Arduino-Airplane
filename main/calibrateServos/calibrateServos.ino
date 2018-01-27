#include <PID_v1.h>

#include <Adafruit_PWMServoDriver.h>




//Servo Limits
#define SERVOMIN 90 //Should be pretty accurate
#define SERVOMAX 460
#define AILERONMAX 277
#define AILERONMIN 195
#define ELEVATORMAX 175 // Actual max is 176
#define ELEVATORMIN 95
#define RUDDERMAX 175
#define RUDDERMIN 95

//Outputs
#define LED 13
#define LEFT_AILERON 0
#define RIGHT_AILERON 15
#define ELEVATOR 10
#define RUDDER 8
#define THROTTLE 6


void setup() 
{
  pwmDriver.begin();
  pwmDriver.setPWMFreq(50);
}

void loop()
{
  pwmDriver.setPWM(RUDDER,0,RUDDERMIN);
}
