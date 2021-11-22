#include <ESP8266WiFi.h> 
#include <fastLED.h>
#include <Hash.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFiUdp.h>
#include <WiFiClient.h>
#include <WebSocketsClient.h>
#include <SocketIOclient.h>

#include <Hash.h>
#define CONNECTED 0
#define INIT 1
#define WAITING 3
#define DISCONNECTED 2
#define LED_COLORS_COUNT 72
#define DEFAULT_DELAY_BETWEEN_LEDS_MS 30
#define DATA_PIN 2
#define USE_SERIAL Serial

const char *ssid = "MASTER_5GHZ";
const char *pwd = "jesuislemotdepasse";
const int port=8082;
const char path[]="/";
int state=INIT;
char packet[255];

void handleCmd(JsonObject& json);

WiFiUDP udp;
WiFiClient client;
IPAddress subnet;
IPAddress broadcast;
//SocketIoClient webSocket;

void tryConnectTCP(const IPAddress &ip,const int port);
void udpSendPacket(char *packet, int packetSize);
void listenUDPInfo();
void loopClient();
IPAddress getBroadcast();
void sendJson(const JsonObject& json,char* cmd);
void parseCmd(uint8_t * payload, size_t length);
CRGB leds[LED_COLORS_COUNT];
void changeLedsColors(int color,int delayBetweenLedsMs);
void intToRGB(int color,int *rgb);
void socketIOEvent(socketIOmessageType_t type, uint8_t * payload, size_t length);

SocketIOclient socketIO;



void setup() {
   // Connect to wifi
   WiFi.begin(ssid, pwd);
   Serial.begin(9600);

   //  Wait esp8266 connected to wifi
   while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print('.');
   }
   
   subnet=WiFi.subnetMask();
   udp.begin(port);
   state=DISCONNECTED;
   FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, LED_COLORS_COUNT);
   for(int i=0;i<LED_COLORS_COUNT;i++){
     leds[i]=CRGB::Black;
     delay(30);
     FastLED.show();
   }
   }
void loop() {
  if (state==DISCONNECTED)
  {
    listenUDPInfo();
  }
  else
    socketIO.loop();
  
}
void listenUDPInfo(){
  const size_t CAPACITY = JSON_OBJECT_SIZE(1);
  StaticJsonDocument<CAPACITY> doc;
  JsonObject object = doc.to<JsonObject>();
  object["client"] = "ledcontroller";
  udp.beginPacket(getBroadcast(),port);
  serializeJson(doc,udp);
  udp.endPacket();

  int packetSize = udp.parsePacket();
  if (packetSize)
  {
    IPAddress remoteIp = udp.remoteIP();
    int len = udp.read(packet, 255);
    if (len > 0)
    {
      packet[len] = 0;
      StaticJsonDocument<CAPACITY> response;
      deserializeJson(response,packet);
      JsonObject object = response.as<JsonObject>();
      if(object.containsKey("server"))
      {
        tryConnectTCP(remoteIp,port);
        return;
      }
    }

  }
}
void udpSendPacket(char *packet, int packetSize){
  udp.beginPacket(getBroadcast(),port);
  udp.write(packet, packetSize);
  udp.endPacket();
}
IPAddress getBroadcast(){
  int b = WiFi.localIP().v4() | ( ~ subnet.v4() );
  return IPAddress(b); 
}
void tryConnectTCP(const IPAddress &ip,const int port){
   Serial.println("tryConnectTCP");
   socketIO.begin("192.168.1.37", 8082, "/socket.io/?EIO=4");
   socketIO.onEvent(socketIOEvent);
   state=WAITING;
}

void handleCmd(JsonObject& data){
    if(data.containsKey("cmd"))
    {
      String cmd=data["cmd"];
      if(cmd.compareTo("changeallcolor")==0){
      int color=data["color"];
      int delay=data.containsKey("delay")?data["delay"]:DEFAULT_DELAY_BETWEEN_LEDS_MS;
      changeLedsColors(color,delay);
      }else if(cmd.compareTo("shutalloffcolor")==0){
        int dely=data.containsKey("delay")?data["delay"]:DEFAULT_DELAY_BETWEEN_LEDS_MS;
        for(int i=0;i<LED_COLORS_COUNT;i++)
        {
          leds[i]=CRGB::Black;
          delay(dely);
          FastLED.show();

        }
      }

    }
}
void changeLedsColors(int color,int delayBetweenLedsMs){
   int rgb[]={0,0,0};
    intToRGB(color,rgb);
    Serial.println("change all color event");
  for(int i=0;i<LED_COLORS_COUNT;i++)
  {
    leds[i]=CRGB(rgb[0],rgb[1],rgb[2]);
    delay(delayBetweenLedsMs);
    FastLED.show();

  }
}
void intToRGB(int color,int *rgb){
  rgb[0]=(color>>16)&0xFF;
  rgb[1]=(color>>8)&0xFF;
  rgb[2]=color&0xFF;
};
void sendJson(const JsonObject& json,char* cmd){
      json["cmd"]=cmd;
      DynamicJsonDocument doc(1024);
      JsonArray array = doc.to<JsonArray>();
      array.add("deviceinformation");
      JsonObject param1 = array.createNestedObject();
      for (JsonPair kv : json) {
          param1[kv.key()] = kv.value();
      }
        // JSON to String (serializion)
      String output;
      serializeJson(doc, output);
      Serial.println(output);
        // Send event        
      socketIO.sendEVENT(output);

}
void loopClient(){
  if(client.connected())
  {
  
    while(client.connected())
    {
      if(client.available())
      {
        String line=client.readString();
        if(line.indexOf("{\"cmd\"")>=0)
        {
          Serial.println(line);
          StaticJsonDocument<512> doc;
          deserializeJson(doc,line);
          JsonObject object = doc.as<JsonObject>();
          handleCmd(object);
        }
      }
    }
  }
  else
  {
    state=DISCONNECTED;
  }
}

void socketIOEvent(socketIOmessageType_t type, uint8_t * payload, size_t length) {
    DynamicJsonDocument doc(1024);
  
    switch(type) {
        case sIOtype_DISCONNECT:
            state=DISCONNECTED;
            USE_SERIAL.printf("[IOc] Disconnected!\n");
            break;
        case sIOtype_CONNECT:
            USE_SERIAL.printf("[IOc] Connected to url: %s\n", payload);
            // join default namespace (no auto join in Socket.IO V3)
            socketIO.send(sIOtype_CONNECT, "/");
            state=CONNECTED;
            doc["client"] = "ledcontroller";
            doc["mac"]=WiFi.macAddress();
            doc["type"]="ledcontrol";
            doc["ip"]=WiFi.localIP().toString();
            doc["led"]=LED_COLORS_COUNT;
            doc["startAt"]=millis();
            doc["delayBetweenLedsMs"]=DEFAULT_DELAY_BETWEEN_LEDS_MS;
            sendJson(doc.as<JsonObject>(),"deviceinformation");
            break;
        case sIOtype_EVENT:
            parseCmd(payload,length); 
            break;
        case sIOtype_ACK:
            USE_SERIAL.printf("[IOc] get ack: %u\n", length);
            hexdump(payload, length);
            break;
        case sIOtype_ERROR:
            USE_SERIAL.printf("[IOc] get error: %u\n", length);
            hexdump(payload, length);
            break;
        case sIOtype_BINARY_EVENT:
            USE_SERIAL.printf("[IOc] get binary: %u\n", length);
            hexdump(payload, length);
            break;
        case sIOtype_BINARY_ACK:
            USE_SERIAL.printf("[IOc] get binary ack: %u\n", length);
            hexdump(payload, length);
            break;
    }
}
void parseCmd(uint8_t * payload, size_t length){
  StaticJsonDocument<1024> doc;
  auto str=String((char*)payload);
  deserializeJson(doc,str);
  JsonArray arr = doc.as<JsonArray>();
  JsonObject object = arr[1];
  String event=arr[0].as<String>();
  if(event.compareTo("cmd")==0)
  {
    handleCmd(object);
  }
}