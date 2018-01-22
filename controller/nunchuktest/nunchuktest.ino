
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
  nck.update();
  if (nck.z_button()) {
    if (!zPressed) {
      Serial.println("z");
    }
    else {
      zPressed = false;
    }
  }
  Serial.print(nck.x_acceleration());
  Serial.print("     ");
  Serial.print(nck.y_acceleration());
  Serial.print("     ");
  Serial.print(nck.z_acceleration());
}
