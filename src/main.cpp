#include <ESP8266WiFi.h> 
#include <fastLED.h>
#include <Hash.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFiUdp.h>
#include <WiFiClient.h>
#define CONNECTED 0
#define INIT 1
#define DISCONNECTED 2
#define LED_COLORS_COUNT 72
#define DEFAULT_DELAY_BETWEEN_LEDS_MS 30
#define DATA_PIN 2
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

CRGB leds[LED_COLORS_COUNT];
void changeLedsColors(int color,int delayBetweenLedsMs);
void intToRGB(int color,int *rgb);

void setup() {
   // Connect to wifi
   WiFi.begin(ssid, pwd);
   Serial.begin(9600);

  Serial.println("Connecting to wifi");
   //  Wait esp8266 connected to wifi
   while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print('.');
   }
   
   subnet=WiFi.subnetMask();
   udp.begin(port);
   state=DISCONNECTED;
   FastLED.addLeds<WS2812B, DATA_PIN>(leds, LED_COLORS_COUNT);
   }
void loop() {
  if (state==DISCONNECTED)
  {
    listenUDPInfo();
  }
  else if (state==CONNECTED)
  {
    loopClient();
  }
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
  if(client.connect(ip,8083))
  {
    state=CONNECTED;
    DynamicJsonDocument doc(1024);
    doc["client"] = "ledcontroller";
    doc["mac"]=WiFi.macAddress();
    doc["ip"]=WiFi.localIP().toString();
    doc["led"]=LED_COLORS_COUNT;
    doc["startAt"]=millis();
    doc["delayBetweenLedsMs"]=DEFAULT_DELAY_BETWEEN_LEDS_MS;
    sendJson(doc.as<JsonObject>(),"deviceinformation");
  }
  else
  {
    //Serial.println("Connection failed");
  }
  

}

void handleCmd(JsonObject& data){
    if(data.containsKey("cmd"))
    {
      String cmd=data["cmd"];
      if(cmd.compareTo("changeLedsColors")==0){
      int color=data["color"];
      int delay=data.containsKey("delay")?data["delay"]:DEFAULT_DELAY_BETWEEN_LEDS_MS;
      changeLedsColors(color,delay);
      }
    }
}
void changeLedsColors(int color,int delayBetweenLedsMs){
   int rgb[]={0,0,0};
    intToRGB(color,rgb);
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
      serializeJson(json,client);      
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