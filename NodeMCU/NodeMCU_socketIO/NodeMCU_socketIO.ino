#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>

#include <ArduinoJson.h>

#include <WebSocketsClient.h>
#include <SocketIOclient.h>
#include <SoftwareSerial.h>

#include <Hash.h>

ESP8266WiFiMulti WiFiMulti;
SocketIOclient socketIO;

char *ssid1 = "Rishab's Galaxy M31";
char *pass1 = "ciqk4548";

char *ssid2 = "dugar 4g";
char *pass2 = "rishab0430";

char *ssid3 = "Flat";
char *pass3 = "87654321";

char path[] = "/";
char host[] = "https://cass-6o2w.onrender.com";

// String lockID = "633d5cfe87d66bc0cd4553ac";

SoftwareSerial arduino(D6, D5);

// void sendRes(res){
//   if(res == 0){
//       arduino.println("0");
//     }
//     else if(res == 1){
//       arduino.println("1");
//     }
// }

void socketIOEvent(socketIOmessageType_t type, uint8_t *payload, size_t length) {
  switch (type) {
    case sIOtype_DISCONNECT:
      Serial.printf("[IOc] Disconnected!\n");
      break;
    case sIOtype_CONNECT:
      Serial.printf("[IOc] Connected to url: %s\n", payload);

      // join default namespace (no auto join in Socket.IO V3)
      socketIO.send(sIOtype_CONNECT, "/");
      break;
    case sIOtype_EVENT:
      {
        // // Serial.printf("[IOc] get event: %s\n", payload);
        // DynamicJsonDocument doc(1024);
        // deserializeJson(doc, payload);

        // String eventName = doc[0];

        // Serial.print("PAYLOAD");
        // Serial.println(eventName);

        // if (eventName == "tag_denied") {
        //   Serial.println("NOT AUTHORIZED");
        //   // sendRes(0);
        //    arduino.println("0");

        // } else if (eventName == "tag_authorized") {
        //   Serial.println("AUTHORIZED");
        //   // sendRes(1);
        //    arduino.println("1");
        // }
        // break;
      }
    case sIOtype_ACK:
      Serial.printf("[IOc] get ack: %u\n", length);
      hexdump(payload, length);
      break;
    case sIOtype_ERROR:
      Serial.printf("[IOc] get error: %u\n", length);
      hexdump(payload, length);
      break;
    case sIOtype_BINARY_EVENT:
      Serial.printf("[IOc] get binary: %u\n", length);
      hexdump(payload, length);
      break;
    case sIOtype_BINARY_ACK:
      Serial.printf("[IOc] get binary ack: %u\n", length);
      hexdump(payload, length);
      break;
  }
}

void emitStatus(String type, String msg) {
  // creat JSON message for Socket.IO (event)
  DynamicJsonDocument doc(1024);
  JsonArray array = doc.to<JsonArray>();

  array.add("cass");

  JsonObject obj = array.createNestedObject();

  obj["type"] = type;
  obj["msg"] = msg;

  String output;

  serializeJson(doc, output);

  char payload[200];
  strcpy(payload, output.c_str());

  // Send event
  socketIO.sendEVENT(payload);

  // Print JSON for debugging
  Serial.print("OUTPUT: ");
  Serial.println(output);
}

void setup() {
  // Serial.begin(921600);
  Serial.begin(115200);
  arduino.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.setDebugOutput(false);
  // Serial.setDebugOutput(true);

  Serial.println();
  Serial.println();
  Serial.println();
  digitalWrite(LED_BUILTIN, LOW);
  for (uint8_t t = 4; t > 0; t--) {
    Serial.printf("[SETUP] BOOT WAIT %d...\n", t);
    Serial.flush();
    delay(1000);
  }
  digitalWrite(LED_BUILTIN, HIGH);
  WiFi.begin(ssid2, pass2);
  Serial.print("connecting.");


  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(125);
    digitalWrite(LED_BUILTIN, LOW);
    delay(125);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(125);
    digitalWrite(LED_BUILTIN, LOW);
    delay(125);
    Serial.print(".");
  }

  Serial.println("");
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("connected");

  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);

  // char* header = setAuth();


  // socketIO.setExtraHeaders(header);
  // event handler
  socketIO.onEvent(socketIOEvent);

  socketIO.beginSSL(host, 443, "/socket.io/?EIO=4");
}

void readStatus() {
  String status = arduino.readString();
  if(status.length() > 0){
    Serial.println(status); 
    String type = status.substring(0, 1);
    String msg= status.substring(2, status.length() - 1)    ;
    emitStatus(type, msg); 
    delay(1000);
    
  }

}

unsigned long messageTimestamp = 0;
void loop() {
  socketIO.loop();
  
  if(socketIO.isConnected()){
    readStatus();
  }
  readStatus();
}




