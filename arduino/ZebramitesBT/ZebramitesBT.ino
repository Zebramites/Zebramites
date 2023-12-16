/**
 * <insert header comment here>
 */

#include <Alfredo_NoU2.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <BluetoothSerial.h>

NoU_Motor *motors[6];

NoU_Servo *servos[4];
BluetoothSerial bluetooth;
bool HAS_RUN_AUTO = false;

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
  bluetooth.begin("Zebramites2023");
  bluetooth.setTimeout(5);
  RSL::initialize();
  RSL::setState(RSL_ENABLED);
}

void run_auto(double time_ms) {
  double command = -1.0;
  double invert = -1.0;
  motors[0]->set(command * invert);
  motors[1]->set(command * invert);
  motors[2]->set(command);
  motors[3]->set(command);
  delay(time_ms);
  command = 0.0;
  motors[0]->set(command * invert);
  motors[1]->set(command * invert);
  motors[2]->set(command);
  motors[3]->set(command);
}

void loop() {
  char c = bluetooth.read();
  if (c == 'm' || c == 's') {
    size_t index = bluetooth.readStringUntil(';').toInt();
    double command = bluetooth.readStringUntil(';').toDouble();
    switch (c) {
      case 'm':
        if (index == 32) {
          if (!HAS_RUN_AUTO) {
            run_auto(command * 10000);
            HAS_RUN_AUTO = true;
            break; // hmm... motors[31] throws an exception :)
          }
        }
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