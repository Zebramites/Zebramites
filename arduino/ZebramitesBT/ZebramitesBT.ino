/**
 * <insert header comment here>
 */

#include <Alfredo_NoU2.h>

NoU_Motor *motors[6];

NoU_Servo *servos[4];

void setup() {

  for (int i = 0; i < 6; i++) {
    NoU_Motor *m = new NoU_Motor(i+1);
    motors[i] = m;
    motors[i]->setInverted(false);
  }

  for (int i = 0; i < 4; i++) {
    NoU_Servo *s = new NoU_Servo(i+1);
    servos[i] = s;
  }

  Serial.begin(115200);

  RSL::initialize();
  RSL::setState(RSL_ENABLED);
}

void loop() {
  char c = Serial.read();
  size_t index = Serial.readStringUntil(';').toInt();
  double command = Serial.readStringUntil(';').toDouble();
  switch (c) {
    case 'm':
      // motor
      motors[index]->set(command);
      break;
    case 's':
      // servo
      servos[index]->write(command);      
  }
  RSL::update();
  delay(1); // yeah 1000 Hz!
}