
#include <Alfredo_NoU2.h>
#include <BluetoothSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

BluetoothSerial bluetooth;

unsigned long previousMillis = 0;
const unsigned long interval = 75; // Interval in milliseconds (1000 ms / 10 Hz = 100 ms)

double motor_1_speed;
NoU_Motor motor_1(1);
double motor_2_speed;
NoU_Motor motor_2(2);
double motor_3_speed;
NoU_Motor motor_3(3);
double motor_4_speed;
NoU_Motor motor_4(4);
NoU_Motor motor_5(4);
NoU_Motor motor_6(4);

bool SEND_IMU_DATA = false;


NoU_Servo rotate_base_arm(1, 500, 2500); // look at me reading the spec sheet to get the full 180, but wait, you can write -10 to the servos and it works??
double rotate_base_arm_angle = -5; // somehow can publish negatives???
NoU_Servo base_arm_joint(2, 500, 2500);
double base_arm_joint_angle = 90;
NoU_Servo stage_2_arm_joint(3, 500, 2500);
double stage_2_arm_joint_angle = 45;
NoU_Servo claw_joint(4, 500, 2500);
double claw_joint_angle = 170;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
sensors_event_t orientationData;
double x, y, z;


void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  rotate_base_arm.write(rotate_base_arm_angle);
  base_arm_joint.write(base_arm_joint_angle);
  stage_2_arm_joint.write(stage_2_arm_joint_angle);
  claw_joint.write(claw_joint_angle);
  bluetooth.begin("Zebramitesarecool");
  bluetooth.setTimeout(5);
  Serial.begin(115200);

  while (!Serial) delay(10);  // wait for serial port to open!    /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  Serial.println("Initalized");
}


void bluetoothPrintLineFast(const String& line) {

  unsigned int length = line.length();
  char buffer[length + 1];  // +1 for storing characters and newline
  
  unsigned int index = 0;
  for (unsigned int i = 0; i < length; i++) {
    if (line[i] != '\0') {
      buffer[index++] = line[i];
    }
  }
  
  buffer[index++] = '\n';  // Newline
  
  bluetooth.write(reinterpret_cast<uint8_t*>(buffer), index);
  //Serial.println("==========Wrote bluetooth=========");
  //Serial.println(line);
}


void bluetoothPrintLine(String line)
{
  
  unsigned l = line.length();
  for (int i = 0; i < l; i++)
  {
    if (line[i] != '\0')
      bluetooth.write(byte(line[i]));
   
  }
   bluetooth.write(10); // \n
}

void readAndSendRPY() {
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    x = orientationData.orientation.x;
    y = orientationData.orientation.y;
    z = orientationData.orientation.z;

    const String msg = "a" + String(x) + ";" + String(z) + "z";
    bluetoothPrintLineFast(msg); 
  
}

void loop() {


  // format: z<MOTOR ID>;<MOTOR POWER>;
  //Serial.println("Somewhat working");



  // Check if it's time to run the function
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval && SEND_IMU_DATA) {
    previousMillis = currentMillis;

    // Call your function here
    readAndSendRPY();
  }
  
  while(bluetooth.available() > 0 && bluetooth.read() == 'z' )
  {
    //Serial.println("data");

    while (true) {
 
      
      int motorId = bluetooth.readStringUntil(';').toInt();
      double speed = bluetooth.readStringUntil(';').toFloat();
      ////Serial.println(motorId);
      ////Serial.println(speed);
      
      if (motorId == -9) {
        if (speed == -9) {
          SEND_IMU_DATA = true;
        }
        else {
          SEND_IMU_DATA = false;
        }
        
        Serial.println("Found stop sequence!");
        break;
      }
      
      switch (motorId) {
        case 1:
          if (speed != motor_1_speed) {
            motor_1.set(speed);
            motor_1_speed = speed;
          }
          
          Serial.print("Setting motor 1 to ");
          Serial.println(speed); 
          break;
        case 2:
          if (speed != motor_2_speed) {
            motor_2.set(speed);
            motor_2_speed = speed;
          }
          Serial.print("Setting motor 2 to ");
          Serial.println(speed); 
          break;
        case 3:
         if (speed != motor_3_speed) {
            motor_3.set(speed);
            motor_3_speed = speed;
          }
          Serial.print("Setting motor 3 to ");
          Serial.println(speed); 
          break;
        case 4:
 
         if (speed != motor_4_speed) {
            motor_4.set(speed);
            motor_4_speed = speed;
          }          
          //Serial.print("Setting motor 4 to ");
          //Serial.println(speed); 
          break;
         
        case 5:
          Serial.println("Moving rotate_base_arm servo (A) to"); 
          Serial.println(speed);
          //rotate_base_arm.write(speed);
          rotate_base_arm_angle = speed;
          rotate_base_arm.write(rotate_base_arm_angle);
          break;
        case 6:
          //Serial.println("Moving base arm servo (B) to"); 
          //Serial.println(speed);
          //base_arm_joint.write(speed);
          base_arm_joint_angle = speed;
          base_arm_joint.write(base_arm_joint_angle);
          break;
        case 7:
          Serial.println("Moving stage 2 arm servo (C) to"); 
          Serial.println(speed);
          //stage_2_arm_joint.write(speed);
          stage_2_arm_joint_angle = speed;
          stage_2_arm_joint.write(stage_2_arm_joint_angle);
  
          break;
        case 8:
          Serial.println("Moving claw servo (D) to"); 
          Serial.println(speed);
          //claw_joint.write(speed);
          claw_joint_angle = speed;
          claw_joint.write(claw_joint_angle);
          break;
         
          
      }
      
    }
  
  }
  
 }
