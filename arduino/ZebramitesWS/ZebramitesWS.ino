#include <WiFi.h>
#include <esp_wifi.h>  
#include <WebSocketsServer_Generic.h> // WebSockets_Generic
#include <Alfredo_NoU3.h>
#include <Wire.h>
#include "Adafruit_VL6180X.h"

// wifi 
const char* ssid = "254hotspot";    // SSID for the AP
const char* password = "poofpoof";  // Password for the AP
WebSocketsServer webSocket = WebSocketsServer(9000);
// end wifi

// distance sensor
constexpr int PIN_SDA_Q = 33;
constexpr int PIN_SDL_Q = 34;
Adafruit_VL6180X vl = Adafruit_VL6180X();
double previous_distance = 0; 
unsigned long last_distance_time = 0;
// end distance sensor


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
  Wire.setPins(PIN_SDA_Q, PIN_SDL_Q);
  if (! vl.begin()) {
    Serial.println("Failed to find distance sensor");
    while (1);
  }
}

void init_wifi() {
  WiFi.hostname("esp32");
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  int txPower = WiFi.getTxPower();
  Serial.print("TX power: ");
  Serial.println(txPower);
  Serial.println("Starting ap");
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  delay(2000);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  txPower = WiFi.getTxPower();
  Serial.print("TX power: ");
  Serial.println(txPower);
  // debug
  IPAddress IP = WiFi.softAPIP();
  Serial.print("Access Point IP Address: ");
  Serial.println(IP);
}

void setup(){ 
  NoU3.begin(); // DO NOT FORGET LMAO
  NoU3.beginIMUs();

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
}

void loop() {
  webSocket.loop(); // Handle WebSocket events
  delay(1);
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

void processError(uint8_t status) {
    // Some error occurred, print it out!
  if  ((status >= VL6180X_ERROR_SYSERR_1) && (status <= VL6180X_ERROR_SYSERR_5)) {
    Serial.println("System error");
  }
  else if (status == VL6180X_ERROR_ECEFAIL) {
    Serial.println("ECE failure");
  }
  else if (status == VL6180X_ERROR_NOCONVERGE) {
    Serial.println("No convergence");
  }
  else if (status == VL6180X_ERROR_RANGEIGNORE) {
    Serial.println("Ignoring range");
  }
  else if (status == VL6180X_ERROR_SNR) {
    Serial.println("Signal/Noise error");
  }
  else if (status == VL6180X_ERROR_RAWUFLOW) {
    Serial.println("Raw reading underflow");
  }
  else if (status == VL6180X_ERROR_RAWOFLOW) {
    Serial.println("Raw reading overflow");
  }
  else if (status == VL6180X_ERROR_RANGEUFLOW) {
    Serial.println("Range reading underflow");
  }
  else if (status == VL6180X_ERROR_RANGEOFLOW) {
    Serial.println("Range reading overflow");
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
  if (msg == "dist?") {
      if (!(millis() - last_distance_time > 100)) {
        webSocket.sendTXT(clientNum, "dist;" + String(previous_distance) + ";");
        return;
      }
      uint8_t range = vl.readRange();
      uint8_t status = vl.readRangeStatus();

      if (status == VL6180X_ERROR_NONE) {
        Serial.print("Range: "); Serial.println(range);
        webSocket.sendTXT(clientNum, "dist;" + String(range) + ";");
      }
      else {
        Serial.println("Error occured in reading distance");
        webSocket.sendTXT(clientNum, "d;900;");
      }
      last_distance_time = millis();
      previous_distance = range;
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
    webSocket.sendTXT(clientNum, "NOT IMPLMENTED!!!");
    //webSocket.sendTXT(clientNum, "imut;" + String(0) + ";ax;" + String(acceleration_x) + ";ay;" + String(acceleration_y) + ";az;" + String(acceleration_z) + ";vx;" + String(gyroscope_x) + ";vy;" + String(gyroscope_y) + ";vz;" + String(gyroscope_z) + ";mx;" + String(magnetometer_x) + ";my;" + String(magnetometer_y) + ";mz;" + String(magnetometer_z) + ";");
    // Serial.println("imut;" + String(lastLSM6) + ";ax;" + String(acceleration_x) + ";ay;" + String(acceleration_y) + ";az;" + String(acceleration_z) + ";vx;" + String(gyroscope_x) + ";vy;" + String(gyroscope_y) + ";vz;" + String(gyroscope_z) + ";mx;" + String(magnetometer_x) + ";my;" + String(magnetometer_y) + ";mz;" + String(magnetometer_z) + ";\n");
    return;
  }
  if (msg == "ping") {
    webSocket.sendTXT(clientNum, "pong");
    return;
  }
  // "dio?[pin number]" returns state of digital input
  if (msg.startsWith("dio?")) {
    long pin = msg.substring(4).toInt();
    if (pin < 0) {
      Serial.print("Invalid pin requested: ");
      Serial.println(pin);
      webSocket.sendTXT(clientNum, "Invalid pin requested.");
      return;
    }
    pinMode(pin, INPUT_PULLUP);
    webSocket.sendTXT(clientNum, "pin" + String(pin) + ";" + String(digitalRead(pin) ? "1;" : "0;"));
    return;
  }
  Serial.print("Command received: ");
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
      case 'd':
        if (index >= 0) {
          pinMode(index, OUTPUT);
          digitalWrite(index, (int)value);
          Serial.printf("DIO %d set to %d", index, (int)value);
        } else {
          webSocket.sendTXT(clientNum, "Invalid pin.");
        }
      default:
        webSocket.sendTXT(clientNum, "Invalid command type.");
        break;
    }
    // Move past the value to continue processing commands
    startIndex = secondSemicolon + 1;
  }
  webSocket.sendTXT(clientNum, "Success");
}

