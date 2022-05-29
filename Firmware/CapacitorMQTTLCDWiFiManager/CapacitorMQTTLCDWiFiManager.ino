
#include <FS.h>
//#include <SPIFFS.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson

#include <Wire.h>
#include <LiquidCrystal_PCF8574.h>
#include "PCF8574.h"

WiFiClient wifiClient;
PubSubClient client(wifiClient);


float pf;
int CuF=0;
int a = 0;
char mqtt_server[40];
char mqtt_port[6];
char MQTT_USER[34];
char MQTT_PASSWORD[34];

#define MQTT_SERIAL_PUBLISH_CH "/yusni/kirim/serial/"
#define MQTT_SERIAL_RECEIVER_CH "/yusni/terima/serial/"

#define MQTT_V_PUBLISH_CH "/yusni/data/V/"
#define MQTT_I_PUBLISH_CH "/yusni/data/I/"
#define MQTT_PF_PUBLISH_CH "/yusni/data/PF/"
#define MQTT_P_PUBLISH_CH "/yusni/data/W/"
#define MQTT_DEG_PUBLISH_CH "/yusni/data/SUDUT/"
#define MQTT_S_PUBLISH_CH "/yusni/data/VA/"
#define MQTT_Q_PUBLISH_CH "/yusni/data/VAR/"
#define MQTT_C_PUBLISH_CH "/yusni/data/C/"
#define MQTT_L_PUBLISH_CH "/yusni/data/L/"
#define MQTT_LL_PUBLISH_CH "/yusni/kirim/LAGLEAD/"
#define MQTT_CAKTIF_PUBLISH_CH "/yusni/data/caktif/"

#define CAP 0x20 // alamat i2c
#define LCD 0x27 // alamat i2c
#define C1uF P0
#define C2uF P1
#define C3uF P2
#define C4uF P3
#define C10uF P4
#define C15uF P5
#define C20uF P6
#define C20uF P7
#define C30uF D1
#define C40uF D2
#define sw    D3
#define sda   d4
#define scl   d5
#define dt    D6
#define clk    D7
LiquidCrystal_PCF8574 lcd(LCD); 
PCF8574 Cap(CAP);

//flag for saving data
bool shouldSaveConfig = true;
//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

int c[146]={
0b0000000000,0b0000000001,
0b0000000010,0b0000000100,0b0000001000,0b0000001001,0b0000001010,0b0000001100,0b0000001101,0b0000001110,
0b0000001111,0b0000010001,0b0000010010,0b0000010100,0b0000011000,0b0000100000,0b0000100001,0b0000100010,
0b0000100100,0b0000101000,0b0001000000,0b0001000001,0b0001000010,0b0001000100,0b0001001000,0b0001001001,
0b0001001010,0b0001001100,0b0001001101,0b0001001110,0b0100000000,0b0100000001,0b0100000010,0b0100000100,
0b0100001000,0b0100001001,0b0100001010,0b0100001100,0b0100001101,0b0100001110,0b1000000000,0b1000000001,
0b1000000010,0b1000000100,0b1000001000,0b1000001001,0b1000001010,0b1000001100,0b1000001101,0b1000001110,
0b1000010000,0b1000010001,0b1000010010,0b1000010100,0b1000011000,0b1000011001,0b1000011010,0b1000011100,
0b1000011101,0b1000011110,0b1010000000,0b1010000001,0b1010000010,0b1010000100,0b1010001000,0b1010001001,
0b1010001010,0b1010001100,0b1010001101,0b1010001110,0b1100000000,0b1100000001,0b1100000010,0b1100000100,
0b1100001000,0b1100001001,0b1100001010,0b1100001100,0b1100001101,0b1100001110,0b1011000000,0b1011000001,
0b1011000010,0b1011000100,0b1011001000,0b1011001001,0b1011001010,0b1011001100,0b1011001101,0b1011001110,
0b1110000000,0b1110000001,0b1110000010,0b1110000100,0b1110001000,0b1110001001,0b1110001010,0b1110001100,
0b1110001101,0b1110001110,0b1110010000,0b1110010001,0b1110010010,0b1110010100,0b1110011000,0b1110011001,
0b1110011010,0b1110011100,0b1110011101,0b1110011110,0b1111000000,0b1111000001,0b1111000010,0b1111000100,
0b1111001000,0b1111001001,0b1111001010,0b1111001100,0b1111001101,0b1111001110,0b1111010000,0b1111010001,
0b1111010010,0b1111010100,0b1111011000,0b1111011001,0b1111011010,0b1111011100,0b1111011101,0b1111011110,
0b1111100110,0b1111101010,0b1111101100,0b1111101101,0b1111101110,0b1111110000,0b1111110001,0b1111110010,
0b1111110100,0b1111111000,0b1111111001,0b1111111010,0b1111111100,0b1111111101,0b1111111110,0b1111111111};
int show;



void connectmqtt() {
  // Loop until we're reconnected
int i=5;
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
  lcd.clear();
    lcd.print("MQTT Connecting...");
    // Create a random client ID
    String clientId = "CapacitorBank-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(),MQTT_USER,MQTT_PASSWORD)) {
      Serial.print("success, rc=");
      Serial.print(client.state());
      Serial.println(" connected");
      lcd.clear();
      lcd.print("MQTT Connected");
      
    //Once connected, publish an announcement...
      client.publish("/ic/presence/cap/", "Kapasitor Bank Aktif");
      // ... and resubscribe
      client.subscribe(MQTT_C_PUBLISH_CH);
      client.subscribe(MQTT_SERIAL_RECEIVER_CH);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      Serial.print("restarting in ");
      lcd.clear();
      lcd.print("Connecting again....");
      lcd.setCursor(0,1);
      lcd.print("restarting in ");
      i=i-1;
      lcd.print(i); 
      Serial.println(i); 
      if(i==0){delay(1000); ESP.restart(); delay(3000);}
             
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}  
void autoconnectmqtt(){
if (client.state()!=0){//!client.connected()) {
    //WiFi.status() == 3
    Serial.println("MQTT disconnected");
    pasangkapasitor(0);
  lcd.clear();
    lcd.print("MQTT Terputus");
    client.setServer(mqtt_server, atoi(mqtt_port));
    client.setCallback(callback);
    connectmqtt();
  }
}


void callback(char* topic, byte *payload, unsigned int length) {
    Serial.println("-------pesan baru dari broker-----");
    Serial.print("channel:");
    Serial.println(topic);
    Serial.print("data:");  
    Serial.write(payload, length);
    Serial.println();
//if(strcmp(topic,"/yusni/data/pf/")){
//  payload[length]='\0';
//  String s= String((char*)payload);
//  pf=s.toFloat();
//      Serial.print("PF: ");  
//    Serial.print(pf);
//    Serial.println();}
if(strcmp(topic,"/yusni/data/c/")){
  payload[length]='\0';
  String s= String((char*)payload);
  CuF=s.toInt();
      Serial.print("C: ");  
    Serial.print(CuF);
    Serial.println();}
}
void pasangkapasitor(int nilai){
  
  //nilai 0~145
int n=c[nilai];
//  lcd.setCursor(0,1); 
//  lcd.print(n,BIN);

int np=n>>8;


//Serial.print(np,BIN);
//Serial.print(", ");

int nw;
if(np==0b00){
  nw=n-0b0000000000;
  digitalWrite(C40uF,LOW);
  digitalWrite(C30uF,LOW);}
if(np==0b01){
  nw=n-0b0100000000;
  digitalWrite(C40uF,LOW);
  digitalWrite(C30uF,HIGH);}
if(np==0b10){
  nw=n-0b1000000000;
  digitalWrite(C40uF,HIGH);
  digitalWrite(C30uF,LOW);}
if(np==0b11){
  nw=n-0b1100000000;
  digitalWrite(C40uF,HIGH);
  digitalWrite(C30uF,HIGH);}
//Serial.println(nw,BIN);
    Wire.begin(D4,D5);
Wire.beginTransmission(0x20);
Wire.write(nw);
Wire.endTransmission();
}


void setup() {
  int error;
  Serial.begin(115200);
  Serial.println();
  Serial.println("LCD...");
  while (! Serial);
  Serial.println("Dose: check for LCD");
  // See http://playground.arduino.cc/Main/I2cScanner
  Wire.begin(D4,D5);
  pasangkapasitor(0);
  Wire.beginTransmission(LCD);
  error = Wire.endTransmission();
  Serial.print("Error: ");
  Serial.print(error);

  if (error == 0) {
    Serial.println(": LCD found.");

  } else {
    Serial.println(": LCD not found.");
  } // if

  lcd.begin(16, 2); // initialize the lcd
  pinMode(sw,OUTPUT);
  pinMode(dt,OUTPUT);
  pinMode(clk,OUTPUT);
  pinMode(C30uF,OUTPUT);
  pinMode(C40uF,OUTPUT);
  
  Wire.begin(D4,D5);
  lcd.setBacklight(255);
  lcd.clear();
  lcd.setCursor(3,0); 
  lcd.print("M. Yusni M.");
  lcd.setCursor(0,1); 
  lcd.print("  Mekatronika  ");
  delay(2000);
  


  //read configuration from FS json
  Serial.println("mounting FS...");
  SPIFFS.begin();

  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println("\nparsed json");

          strcpy(mqtt_server, json["mqtt_server"]);
          strcpy(mqtt_port, json["mqtt_port"]);
          strcpy(MQTT_USER, json["MQTT_USER"]);
          strcpy(MQTT_PASSWORD, json["MQTT_PASSWORD"]);

        } else {
          Serial.println("failed to load json config");
        }
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }
  //end read



  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  WiFiManagerParameter custom_mqtt_server("mqtt_server", "mqtt_server", mqtt_server, 40);
  WiFiManagerParameter custom_mqtt_port("mqtt_port", "mqtt_port", mqtt_port, 6);
  WiFiManagerParameter custom_MQTT_USER("MQTT User", "MQTT User", MQTT_USER, 32);
  WiFiManagerParameter custom_MQTT_PASSWORD("MQTT Password", "MQTT Password", MQTT_PASSWORD, 32);

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //set static ip
  //wifiManager.setSTAStaticIPConfig(IPAddress(10,0,1,99), IPAddress(10,0,1,1), IPAddress(255,255,255,0));

  //add all your parameters here
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_MQTT_USER);
  wifiManager.addParameter(&custom_MQTT_PASSWORD);
  
    lcd.clear();
    lcd.print("AP Mode:");
  lcd.setCursor(0,1); 
    lcd.print("Capacitor Bank");
  
  //reset settings - for testing
  //wifiManager.resetSettings();

  //set minimu quality of signal so it ignores AP's under that quality
  //defaults to 8%
  //wifiManager.setMinimumSignalQuality();

  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  //wifiManager.setTimeout(120);

  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect("Capacitor Bank", "G42")) {
    Serial.println("failed to connect and hit timeout");
  
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.restart();
    delay(5000);
  }

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");
    lcd.clear();
    lcd.print("WiFi Connected");
    lcd.setCursor(0,1); 
    lcd.print("IP: ");
    lcd.print(WiFi.localIP());
    delay(1000);
  //read updated parameters
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());
  strcpy(MQTT_USER, custom_MQTT_USER.getValue());
  strcpy(MQTT_PASSWORD, custom_MQTT_PASSWORD.getValue());

  //save the custom parameters to FS
  if (shouldSaveConfig) {
    Serial.println("saving config");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["mqtt_server"] = mqtt_server;
    json["mqtt_port"] = mqtt_port;
    json["MQTT_USER"] = MQTT_USER;
    json["MQTT_PASSWORD"] = MQTT_PASSWORD;
    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    }
    json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();
    //end save
  }

  Serial.println("local ip");
  Serial.println(WiFi.localIP());
  Serial.println();
  Serial.print("mqtt_server: ");
  Serial.print(mqtt_server);
  Serial.println();
  Serial.print("mqtt_port: ");
  Serial.print(mqtt_port);
  Serial.println();
  Serial.print("MQTT User: ");
  Serial.print(MQTT_USER);
  Serial.println();
  Serial.print("MQTT Password: ");
  Serial.print(MQTT_PASSWORD);  
  Serial.println("%");

  //WiFi.disconnect(true); //erases store credentially
  //SPIFFS.format();  //erases stored values
  Serial.println("Done");
    delay(8);
  client.setServer(mqtt_server, atoi(mqtt_port));
  client.setCallback(callback);
  connectmqtt();  

}

void loop() {
  client.setCallback(callback);
  lcd.clear();
autoconnectmqtt();
   //uF
Serial.println(CuF);
if(CuF>0){
a++;
if(a<=0){a=0;}
if(a>145){a=145;}
pasangkapasitor(a);
lcd.setCursor(0,0); 
lcd.print("Diminta = ");
lcd.print(CuF);
lcd.print("uF");
lcd.setCursor(0,1); 
lcd.print("Aktif   = ");
lcd.print(a);
lcd.print("uF"); 
delay(500);
}
if(CuF<0){
a--;  
if(a<=0){a=0;}
if(a>145){a=145;}
pasangkapasitor(a);

lcd.setCursor(0,0); 
lcd.print("Diminta = ");
lcd.print(CuF);
lcd.print("uF");
lcd.setCursor(0,1); 
lcd.print("Aktif   = ");
lcd.print(a);
lcd.print("uF"); 
client.publish(MQTT_CAKTIF_PUBLISH_CH, String(a).c_str());
delay(500);
}

if(CuF==0){
a=a;
if(a!=0){
lcd.clear();
lcd.setCursor(0,0); 
lcd.print("PF Diperbaiki");
lcd.setCursor(0,1); 
lcd.print("Aktif   = ");
lcd.print(a);
lcd.print("uF"); 

client.publish(MQTT_CAKTIF_PUBLISH_CH, String(a).c_str());
}
if(a==0){
lcd.clear();
lcd.setCursor(0,0); 
lcd.print("      APFC      ");
lcd.setCursor(0,1); 
lcd.print(" Menunggu Beban ");

client.publish(MQTT_CAKTIF_PUBLISH_CH, String(a).c_str());

}

}

client.publish(MQTT_CAKTIF_PUBLISH_CH, String(a).c_str());
   
 // lcd.setCursor(0,0); 
 // lcd.print(a);
 // lcd.print(" uF");
  client.loop();
  delay(50);
  

///////////////////////////////////////////////////////////////////////

//Uncomment these lines of code if you want to reset the device 
//It could be linked to a physical reset button on the device and set
//to trigger the next 3 lines of code.
  //WiFi.disconnect(true); //erases store credentially
  //SPIFFS.format();  //erases stored values
  //ESP.restart();
///////////////////////////////////////////////////////////////////////

}


