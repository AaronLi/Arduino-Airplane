
#include <Nunchuk.h>





Nunchuk nck;
boolean cPressed = false;
boolean zPressed = false;
void setup()
{
  nck.initialize();
  Serial.begin(115200);
  delay(100);
}

void loop()
{
  if (nck.z_button()) {
    Serial.println("z")
  }
  Serial.print(nck.x_acceleration);
  Serial.print("     ");
  Serial.print(nck.x_acceleration);  
  Serial.print("     ");
  Serial.print(nck.x_acceleration);  
}
