/**
 * <insert header comment here>
 */

#include <Alfredo_NoU2.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

NoU_Motor *motors[6];

NoU_Servo *servos[4];

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  for (int i = 0; i < 6; i++) {
    NoU_Motor *m = new NoU_Motor(i+1);
    motors[i] = m;
    motors[i]->setInverted(false);
  }

  for (int i = 0; i < 4; i++) {
    NoU_Servo *s = new NoU_Servo(i+1, 500, 2500);
    servos[i] = s;
  }

  Serial.begin(921600);

  RSL::initialize();
  RSL::setState(RSL_ENABLED);
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    size_t index = Serial.readStringUntil(';').toInt();
    double command = Serial.readStringUntil(';').toDouble();
    switch (c) {
      case 'm':
        // motor
        motors[index-1]->set(command);
        break;
      case 's':
        // servo
        servos[index-1]->write(command);      
    }
  }
  RSL::update();
  delay(1); // yeah 1000 Hz!
}