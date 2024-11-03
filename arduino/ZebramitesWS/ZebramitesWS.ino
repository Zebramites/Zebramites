#include <WiFi.h>
#include <esp_wifi.h>  
#include <WebSocketsServer_Generic.h> // WebSockets_Generic
#include <Alfredo_NoU3.h>

// from 9DOF_IMU_Robot.ino
int interruptPinLSM6 = 48;
int interruptPinMMC5 = 47;

float acceleration_x, acceleration_y, acceleration_z;
float gyroscope_x, gyroscope_y, gyroscope_z;
float magnetometer_x, magnetometer_y, magnetometer_z;

volatile bool newDataAvailableLSM6 = true;
volatile bool newDataAvailableMMC5 = true;
volatile unsigned long long lastLSM6 = 0;
// end imu code

const char* ssid = "254hotspot";    // SSID for the AP
const char* password = "poofpoof";  // Password for the AP

WebSocketsServer webSocket = WebSocketsServer(9000);

// can change for nou2
constexpr int MAX_MOTORS = 8;
constexpr int MAX_SERVOS = 6;

NoU_Motor *motors[MAX_MOTORS];
NoU_Servo *servos[MAX_SERVOS];

void init_hardware() {
  for (int i = 0; i < MAX_MOTORS; i++) {
    NoU_Motor *m = new NoU_Motor(i+1);
    motors[i] = m;
    motors[i]->setInverted(false);
  }

  for (int i = 0; i < MAX_SERVOS; i++) {
    NoU_Servo *s = new NoU_Servo(i+1, 500, 2500);
    servos[i] = s;
  }
}

void init_wifi() {
  WiFi.hostname("esp32");
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  int txPower = WiFi.getTxPower();
  Serial.print("TX power: ");
  Serial.println(txPower);
  Serial.println("Starting ap");
  delay(2000);    
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);   
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  // debug
  IPAddress IP = WiFi.softAPIP();
  Serial.print("Access Point IP Address: ");
  Serial.println(IP);
}

void setup(){ 
  NoU3.begin(); // DO NOT FORGET LMAO
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  delay(2000);

  Serial.println("Initalizing motors/servos");
  init_hardware();

  Serial.println("Starting WiFi AP");
  init_wifi();

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("WebSocket server started");

  // from 9DOF_IMU_Robot.ino
  // Initialize LSM6
  if (LSM6.begin(Wire1) == false) {
    Serial.println("LSM6 did not respond - check your wiring.");
  }
  pinMode(interruptPinLSM6, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPinLSM6), interruptRoutineLSM6, RISING);
  LSM6.enableInterrupt();

  // Initialize MMC5
  if (MMC5.begin(Wire1) == false) {
    Serial.println("MMC5983MA did not respond - check your wiring.");
  }
  MMC5.softReset();
  MMC5.setFilterBandwidth(800);
  MMC5.setContinuousModeFrequency(100);
  MMC5.enableAutomaticSetReset();
  MMC5.enableContinuousMode();
  pinMode(interruptPinMMC5, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPinMMC5), interruptRoutineMMC5, RISING);
  MMC5.enableInterrupt();
  // end imu code
}

void loop() {
  // from 9DOF_IMU_Robot.ino
  // Check LSM6 for new data
  if (newDataAvailableLSM6) {
    newDataAvailableLSM6 = false;

    if (LSM6.accelerationAvailable()) {
      LSM6.readAcceleration(&acceleration_x, &acceleration_y, &acceleration_z); // g's
    }

    if (LSM6.gyroscopeAvailable()) {
      LSM6.readGyroscope(&gyroscope_x, &gyroscope_y, &gyroscope_z); // deg/s
    }
    // Serial.printf("%f %f %f\n", gyroscope_x, gyroscope_y, gyroscope_z);
  }

  // Check MMC5983MA for new data
  if (newDataAvailableMMC5) {
    newDataAvailableMMC5 = false;
    MMC5.clearMeasDoneInterrupt();

    MMC5.readAccelerometer(&magnetometer_x, &magnetometer_y, &magnetometer_z);  // Results in µT (microteslas).
  }
  // end imu code

  webSocket.loop(); // Handle WebSocket events
  delay(1);
}

void interruptRoutineLSM6() {
  newDataAvailableLSM6 = true;
  lastLSM6 = micros();
}

void interruptRoutineMMC5() {
  newDataAvailableMMC5 = true;
}

// Handle WebSocket events
void webSocketEvent(uint8_t clientNum, WStype_t type, uint8_t *payload, size_t length) {
  if (length < 1) {
    Serial.println("Len < 1");
    return;
  }
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("Client %u disconnected\n", clientNum);
      break;
    case WStype_CONNECTED: {
      IPAddress ip = webSocket.remoteIP(clientNum);
      Serial.printf("Client %u connected from %s\n", clientNum, ip.toString().c_str());
      break;
    }
    case WStype_TEXT: { 
      // Send response back to client
      parseMessage((char*)payload, clientNum);
      break;
    }
    case WStype_PONG:
      Serial.println("Recived pong");
      break;
  }
}

void parseMessage(char* message, uint8_t clientNum) {
  String msg = String(message);

  if (msg == "voltage?") {
    //Serial.print("Sent voltage of ");
    //Serial.println(NoU3.getBatteryVoltage());
    // char type[4] = "vol";
    // float data_values[2];
    // data_values[0] = *(float*)type;
    // data_values[1] = NoU3.getBatteryVoltage();
    // webSocket.sendBIN(clientNum, (uint8_t*)data_values, 2*sizeof(float));
    webSocket.sendTXT(clientNum, "v;" + String(NoU3.getBatteryVoltage()) + ";");
    return;
  }
  if (msg == "imu?") {
    // float data_values[10]; // ax ay az gx gy gz mx my mz
    // char type[4] = "imu";
    // data_values[0] = *(float*)type;
    // data_values[1] = acceleration_x;
    // data_values[2] = acceleration_y;
    // data_values[3] = acceleration_z;
    // data_values[4] = gyroscope_x;
    // data_values[5] = gyroscope_y;
    // data_values[6] = gyroscope_z;
    // data_values[7] = magnetometer_x;
    // data_values[8] = magnetometer_y;
    // data_values[9] = magnetometer_z;
    // webSocket.sendBIN(clientNum, (uint8_t*)data_values, 10*sizeof(float));
    webSocket.sendTXT(clientNum, "imut;" + String(lastLSM6) + ";ax;" + String(acceleration_x) + ";ay;" + String(acceleration_y) + ";az;" + String(acceleration_z) + ";vx;" + String(gyroscope_x) + ";vy;" + String(gyroscope_y) + ";vz;" + String(gyroscope_z) + ";mx;" + String(magnetometer_x) + ";my;" + String(magnetometer_y) + ";mz;" + String(magnetometer_z) + ";");
    // Serial.println("imut;" + String(lastLSM6) + ";ax;" + String(acceleration_x) + ";ay;" + String(acceleration_y) + ";az;" + String(acceleration_z) + ";vx;" + String(gyroscope_x) + ";vy;" + String(gyroscope_y) + ";vz;" + String(gyroscope_z) + ";mx;" + String(magnetometer_x) + ";my;" + String(magnetometer_y) + ";mz;" + String(magnetometer_z) + ";\n");
    return;
  }
  if (msg == "ping") {
    webSocket.sendTXT(clientNum, "pong");
    return;
  }
  Serial.print("Command recived: ");
  Serial.println(msg);
  // Split commands by semicolons
  int startIndex = 0;
  while (startIndex < msg.length()) {
    int firstSemicolon = msg.indexOf(';', startIndex);
    if (firstSemicolon == -1) break; // No more commands

    String command = msg.substring(startIndex, firstSemicolon);
    startIndex = firstSemicolon + 1; // Move past the semicolon

    // Find the value for this command
    int secondSemicolon = msg.indexOf(';', startIndex);
    if (secondSemicolon == -1) {
      secondSemicolon = msg.length(); // if we don't see another ; it means the command was just e.x s1;90 
    }
    
    String valueStr = msg.substring(startIndex, secondSemicolon);
    double value = valueStr.toDouble(); // Convert the value to double

    // Determine command type (e.g., m1 or m2)
    char commandType = command.charAt(0); // 'm' for motor, 's' for servo
    int index = command.substring(1).toInt(); // Get the index after 'm'

    switch (commandType) {
      case 'm':
        if (index >= 1 && index <= MAX_MOTORS) { 
          motors[index-1]->set(value); 
          Serial.printf("Motor %d set to: %f\n", index, value);
          //webSocket.sendTXT(clientNum, "Motor command received.");
        } else {
          webSocket.sendTXT(clientNum, "Invalid motor index.");
        }
        break;
      case 's':
        if (index >= 1 && index <= MAX_SERVOS) {
          servos[index-1]->write((int)value);
          Serial.printf("Servo %d set to: %d degrees\n", index, (int)value);
          //webSocket.sendTXT(clientNum, "Servo command received.");
        } else {
          webSocket.sendTXT(clientNum, "Invalid servo index.");
        }
        break;
      default:
        webSocket.sendTXT(clientNum, "Invalid command type.");
        break;
    }
    // Move past the value to continue processing commands
    startIndex = secondSemicolon + 1;
  }
  webSocket.sendTXT(clientNum, "Success");
}

