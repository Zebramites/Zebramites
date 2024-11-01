#include <WiFi.h>
#include <esp_wifi.h>  
#include <WebSocketsServer_Generic.h>
#include <Alfredo_NoU3.h>

const char* ssid = "254hotspot";    // SSID for the AP
const char* password = "poofpoof";  // Password for the AP

WebSocketsServer webSocket = WebSocketsServer(9000); 
NoU_Servo testServo(1);
int MAX_MOTORS = 8;

void setup(){ 
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  delay(2000);
  Serial.println("Startup....");
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
  Serial.println("Poof!");

    // Get and print the IP address
  IPAddress IP = WiFi.softAPIP();
  Serial.print("Access Point IP Address: ");
  Serial.println(IP);
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("WebSocket server started");

}

void loop() {
  webSocket.loop(); // Handle WebSocket events
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
      Serial.print("Received text from client ");
      Serial.println((const char*)payload);

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
  Serial.print("Command recived: ");
  Serial.println(msg);
  if (msg == "ping") {
    webSocket.sendTXT(clientNum, "Pong");
    return;
  }
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
          // motors[index - 1]->set(value); // Set motor command eventually
          Serial.printf("Motor %d set to: %f\n", index, value);
          webSocket.sendTXT(clientNum, "Motor command received.");
        } else {
          webSocket.sendTXT(clientNum, "Invalid motor index.");
        }
        break;
      case 's':
        if (index >= 1 && index <= 180) { 
          testServo.write((int)value);
          Serial.printf("Servo %d set to: %d degrees\n", index, (int)value);
          webSocket.sendTXT(clientNum, "Servo command received.");
        } else {
          webSocket.sendTXT(clientNum, "Invalid servo angle. Must be between 0 and 180.");
        }
        break;
      default:
        webSocket.sendTXT(clientNum, "Invalid command type.");
        break;
    }

    // Move past the value to continue processing commands
    startIndex = secondSemicolon + 1;
  }
}

