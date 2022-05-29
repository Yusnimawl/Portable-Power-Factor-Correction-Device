

//include <FS.h>
#include "SPIFFS.h"
#include <WiFi.h>
#include <DNSServer.h>
//#include <ESP8266WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson
#include <PubSubClient.h>
#include "YusniVariables.h"
//#include "YusniADCCalibration.h"
//#include "YusniDualCore.h"
#include "YusniEnergyMath.h"
//#include "YusniCalculationCalibration.h"
//#include "YusniEnergyCalculationProcedure.h"
#include "YusniWiFiFunctions.h"
#include "YusniMQTT.h"

WiFiClient wifiClient;
PubSubClient client(wifiClient);

#define ADC_BITS    12
#define ADC_COUNTS  (1<<ADC_BITS)
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
byte biassource = 26;
byte biasread = 34;
byte Tegangan = 33;      
byte Arus = 32;

char mqtt_server[40];
char mqtt_port[6];
char MQTT_USER[34];
char MQTT_PASSWORD[34];


//Useful value variables
    double realPower,
      apparentPower,
      powerFactor,
      Vrms,
      Irms,
      anglerad,
      angledegree,
      reactivePower;
    //Set Voltage and current input pins
    unsigned int inPinV;
    unsigned int inPinI;
    unsigned int inPinOffset;
    //Calibration coefficients
    //These need to be set in order to obtain accurate results
    double VCAL;
    double ICAL;
    double PHASECAL;
    //--------------------------------------------------------------------------------------
    // Variable declaration for emon_calc procedure
    //--------------------------------------------------------------------------------------
    int sampleV;                        //sample_ holds the raw analog read value
    int sampleI;
    int sampleOffset;

    double lastFilteredV,filteredV;          //Filtered_ is the raw analog value minus the DC offset
    double filteredI;
    double offsetV;                          //Low-pass filter output
    double offsetI;                          //Low-pass filter output

    double phaseShiftedV;                             //Holds the calibrated phase shifted voltage.

    double sqV,sumV,sqI,sumI,instP,sumP;              //sq = squared, sum = Sum, inst = instantaneous

    int startV;                                       //Instantaneous voltage at start of sample window.

    boolean lastVCross, checkVCross;                  //Used to measure number of times threshold is crossed.
    
    int vnow,inow,vlast,laglead; //var zerocross: laglead 1 = LAG/induktif laglead 0 = LEAD /Capacitive 

  
//flag for saving data
bool shouldSaveConfig = true;

//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

TaskHandle_t Task0, Task1;

void iddle(){
  delay(1000);}

void bootcore(){
    xTaskCreatePinnedToCore(
    loopCore1,
    "core1",
    8192,
    NULL,
    1,
    &Task1,
    1);

delay(500);  

  xTaskCreatePinnedToCore(
    loopCore0,
    "core0",
    8192,
    NULL,
    1,
    &Task0,
    0);
delay(500);

}

double ReadVoltage(byte pin){
  double reading = analogRead(pin); // Reference voltage is 3v3 so maximum reading is 3v3 = 4095 in range 0 to 4095
  if(reading < 1 || reading > 4095) return 0;
  // return -0.000000000009824 * pow(reading,3) + 0.000000016557283 * pow(reading,2) + 0.000854596860691 * reading + 0.065440348345433;
  return -0.000000000000016 * pow(reading,4) + 0.000000000118171 * pow(reading,3)- 0.000000301211691 * pow(reading,2)+ 0.001109019271794 * reading + 0.034143524634089;

 /* ADC readings v voltage
 *  y = -0.000000000009824x3 + 0.000000016557283x2 + 0.000854596860691x + 0.065440348345433
 // Polynomial curve match, based on raw data thus:
 *   464     0.5
 *  1088     1.0
 *  1707     1.5
 *  2331     2.0
 *  2951     2.5 
 *  3775     3.0
 *  
 */
  
  } // Added an improved polynomial, use either, comment out as required

void offset(unsigned int _inPinOffset){
  inPinOffset = _inPinOffset;
}

//--------------------------------------------------------------------------------------
// Sets the pins to be used for voltage and current sensors
//--------------------------------------------------------------------------------------
void voltage(unsigned int _inPinV, double _VCAL, double _PHASECAL){
  inPinV = _inPinV;
  VCAL = _VCAL;
  PHASECAL = _PHASECAL;
/*  offsetV = ADC_COUNTS>>1;
 *   
 */
}

void current(unsigned int _inPinI, double _ICAL){
  inPinI = _inPinI;
  ICAL = _ICAL;
/*  offsetI = ADC_COUNTS>>1;
 *   
 */
}
//--------------------------------------------------------------------------------------
// emon_calc procedure
// Calculates realPower,apparentPower,powerFactor,Vrms,Irms,kWh increment
// From a sample window of the mains AC voltage and current.
// The Sample window length is defined by the number of half wavelengths or crossings we choose to measure.
//--------------------------------------------------------------------------------------
void calcVI(unsigned int crossings, unsigned int timeout){
  double res=3.3/4096;
  int SupplyVoltage=3300;
  //lag = 0;
  //lead = 0;

  unsigned int crossCount = 0;                             //Used to measure number of times threshold is crossed.
  unsigned int numberOfSamples = 0;                        //This is now incremented

  //-------------------------------------------------------------------------------------------------------------------------
  // 1) Waits for the waveform to be close to 'zero' (mid-scale adc) part in sin curve.
  //-------------------------------------------------------------------------------------------------------------------------
  boolean st=false;                                  //an indicator to exit the while loop

  unsigned long start = millis();    //millis()-start makes sure it doesnt get stuck in the loop if there is an error.

  while(st==false)                                   //the while loop...
  {
    //startV = analogRead(inPinV);                    //using the voltage waveform
    startV = ReadVoltage(inPinV)/res;                    //using the voltage waveform
    
    if ((startV < (ADC_COUNTS*0.65)) && (startV > (ADC_COUNTS*0.45)))//check its within range
    //Serial.println("tiada listrik");
    st=true;  
    if ((millis()-start)>timeout) st = true;
  }



  //-------------------------------------------------------------------------------------------------------------------------
  // 2) Main measurement loop
  //-------------------------------------------------------------------------------------------------------------------------
  start = millis();

  while ((crossCount < crossings) && ((millis()-start)<timeout))
  {
    numberOfSamples++;                       //Count number of times looped.
    lastFilteredV = filteredV;               //Used for delay/phase compensation

    //-----------------------------------------------------------------------------
    // A) Read in raw voltage and current samples
    //-----------------------------------------------------------------------------
//    sampleV = analogRead(inPinV);                 //Read in raw voltage signal
//    sampleI = analogRead(inPinI);                 //Read in raw current signal
//    sampleOffset = analogRead(inPinOffset);                 //Read in raw offset

    sampleV = ReadVoltage(inPinV)/res;                 //Read in raw voltage signal
    sampleI = ReadVoltage(inPinI)/res;                 //Read in raw current signal
    sampleOffset = ReadVoltage(inPinOffset)/res;                 //Read in raw offset

    //-----------------------------------------------------------------------------
    // B) Apply digital low pass filters to extract the 2.5 V or 1.65 V dc offset,
    //     then subtract this - signal is now centred on 0 counts.
    //-----------------------------------------------------------------------------
/*    offsetV = offsetV + ((sampleV-offsetV)/1024);
    filteredV = sampleV - offsetV;
    offsetI = offsetI + ((sampleI-offsetI)/1024);
    filteredI = sampleI - offsetI;

    offsetV = sampleOffset + ((sampleV-offsetV)/4096);
    filteredV = sampleV - offsetV;
    offsetI = sampleOffset + ((sampleI-offsetI)/4096);
    filteredI = sampleI - offsetI;
*/
//offsetV = sampleOffset + ((sampleV-offsetV)/4095);
offsetI = sampleOffset + ((sampleI-offsetI)/4095) ;    
filteredV = (sampleV+95.00) - sampleOffset;
filteredI = sampleI+27- sampleOffset;//-5.4;+0.36703 
//Serial.println(filteredI);

  //long hm = 0;



    //-----------------------------------------------------------------------------
    // C) Root-mean-square method voltage
    //-----------------------------------------------------------------------------
    sqV= filteredV * filteredV;                 //1) square voltage values
    sumV += sqV;                                //2) sum

    //-----------------------------------------------------------------------------
    // D) Root-mean-square method current
    //-----------------------------------------------------------------------------
    sqI = filteredI * filteredI;                //1) square current values
    sumI += sqI;                                //2) sum

    //-----------------------------------------------------------------------------
    // E) Phase calibration
    //-----------------------------------------------------------------------------
    phaseShiftedV = lastFilteredV + PHASECAL * (filteredV - lastFilteredV);
    //-----------------------------------------------------------------------------
    // F) Instantaneous power calc
    //-----------------------------------------------------------------------------
    instP = phaseShiftedV * filteredI;          //Instantaneous Power
    sumP +=instP;                               //Sum

    //-----------------------------------------------------------------------------
    // G) Find the number of times the voltage has crossed the initial voltage
    //    - every 2 crosses we will have sampled 1 wavelength
    //    - so this method allows us to sample an integer number of half wavelengths which increases accuracy
    //-----------------------------------------------------------------------------
    lastVCross = checkVCross;
    if (sampleV > startV) checkVCross = true;
                     else checkVCross = false;
    if (numberOfSamples==1) lastVCross = checkVCross;

    if (lastVCross != checkVCross) crossCount++;
    //Serial.print(phaseShiftedV);
    //Serial.print("\t");
    //Serial.println(filteredI);
  
  }  
  
    if(phaseShiftedV>0){//v+
      vnow=1;
    }
    if(phaseShiftedV<0){//v-
      vnow=0;
    }
    if(filteredI>0){//i+
      inow=1;
    }
    if(filteredI<0){//i-
      inow=0;
    }

    if(vlast==0&&vnow==1){
      if(inow==0){
    //LAG
    laglead=1;
      } else {
    //LEAD
    laglead=0;
    }
Serial.print("laglead= ");
Serial.println(laglead);
    }
vlast=vnow;
    
  /*  if(phaseShiftedV>0){//v+
      vnow=1;
    }
    if(phaseShiftedV<0){//v-
      vnow=0;
    }
if(filteredI>0){//
      //i+
      inow=1;
    }
    if(filteredI<0){//
      //i-
      inow=0;
    }
    
    if(vlast==0&&vnow==1){
      if(inow==1){
        lead=1; 
        lag=0; 
      //Serial.println("LEAD");
      } else {
        lag=1; 
      lead=0; 
      //Serial.println("LAG");}  
    }
    vlast=vnow;
*/
  //-------------------------------------------------------------------------------------------------------------------------
  // 3) Post loop calculations
  //-------------------------------------------------------------------------------------------------------------------------
  //Calculation of the root of the mean of the voltage and current squared (rms)
  //Calibration coefficients applied.

  double V_RATIO = VCAL *((SupplyVoltage/1000.0) / (ADC_COUNTS));
  Vrms = V_RATIO * sqrt(sumV / numberOfSamples);

  double I_RATIO = ICAL *((SupplyVoltage/1000.0) / (ADC_COUNTS));
  Irms = I_RATIO * sqrt(sumI / numberOfSamples);

  //Calculation power values
  realPower = V_RATIO * I_RATIO * sumP / numberOfSamples;
  apparentPower = Vrms * Irms;
  //Calculation power values test, hasil harus 
  //1732.050808 var 
  //pf = 0.5 
  //1.047197551 radian
  //59.97585975 derajat
  //realPower = 1000;
  //apparentPower = 2000;
  
  powerFactor=realPower / apparentPower;
  anglerad=acos(powerFactor);
  angledegree=(anglerad*360.00)/(PI*2.00);
  reactivePower=apparentPower*sin(anglerad);

  //Reset accumulators
  sumV = 0;
  sumI = 0;
  sumP = 0;

//--------------------------------------------------------------------------------------
}




void serialprint(){ 

  Serial.println();
  Serial.print("daya nyata  = ");
  Serial.print(realPower);
  Serial.println(" W");
  Serial.print("daya semu  = ");
  Serial.print(apparentPower);
  Serial.println(" VA");
  Serial.print("daya reaktif  = ");
  Serial.print(reactivePower);
  Serial.println(" VAR");
  Serial.print("tegangan  = ");
  Serial.print(Vrms);
  Serial.println(" V");
  Serial.print("arus  = ");
  Serial.print(Irms,10);
  Serial.println(" A");
  Serial.println(powerFactor);
  Serial.print(anglerad);
  Serial.println(" rad");
  Serial.print(angledegree);
  Serial.print(" deg");
  //if(lead==0&&lag==1&&powerFactor!=1){Serial.print(" lag");}
  //if(lead==1&&lag==0&&powerFactor!=1){Serial.print(" lead");}
  if(powerFactor>0.98){Serial.println(" unity");}
  //if(lead==1&&lag==1&&powerFactor!=1){Serial.print(" harmonics");}
  
  //if(lag==1&&lead==1){Serial.println(" harmonisa");}
  //lag=0;
  //lead=0;
  Serial.println();
  delay(1000);
}



//fungsi2 mqtt
void callback(char* topic, byte *payload, unsigned int length) {
    Serial.println("-------pesan baru dari broker-----");
    Serial.print("channel:");
    Serial.println(topic);
    Serial.print("data:");  
    Serial.write(payload, length);
    Serial.println();
}

void connectmqtt() {
  // Loop until we're reconnected
int i=5;
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ENERGY-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(),MQTT_USER,MQTT_PASSWORD)) {
      Serial.print("success, rc=");
      Serial.print(client.state());
      Serial.println(" connected");
      //Once connected, publish an announcement...
      client.publish("/ic/presence/energy/", "Energi Monitor Aktif");
      // ... and resubscribe
      client.subscribe(MQTT_SERIAL_RECEIVER_CH);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}  
void autoconnectmqtt(){
if (client.state()!=0){//!client.connected()) {
    //WiFi.status() == 3
    Serial.println("MQTT disconnected");
    client.setServer(mqtt_server, atoi(mqtt_port));
    client.setCallback(callback);
    connectmqtt();
  }
}
void publishll(char *data){
  client.publish(MQTT_LL_PUBLISH_CH, data);
}
void publishSerialData(char *serialData){
  client.publish(MQTT_SERIAL_PUBLISH_CH, serialData);
}
void kirimdatadariserial(){
     if (Serial.available() > 0) {
     char bfr[101];
     memset(bfr,0, 101);
     Serial.readBytesUntil( '\n',bfr,100);
     publishSerialData(bfr);
   }
}


void setup() {
  bootcore();
  Serial.begin(115200);
  Serial.setTimeout(500);// Set time out for 
  pinMode(biassource, OUTPUT);
  pinMode(biasread, INPUT);
  pinMode(Tegangan, INPUT);
  pinMode(Arus, INPUT);
  dacWrite(biassource, 127);
  offset(biasread);
  voltage(Tegangan, 257.7, 2);        // Voltage: input pin, calibration, phase_shift
  current(Arus, 12.85 );       // Current: input pin, calibration.

  Serial.println();



  //read configuration from FS json
  Serial.println("mounting FS...");
  SPIFFS.begin (true);

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
  if (!wifiManager.autoConnect("ESP32 Energy Monitor", "G42")) {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.restart();
    delay(5000);
  }

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");

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
iddle();  

///////////////////////////////////////////////////////////////////////

//Uncomment these lines of code if you want to reset the device 
//It could be linked to a physical reset button on the device and set
//to trigger the next 3 lines of code.
  //WiFi.disconnect(true); //erases store credentially
  //SPIFFS.format();  //erases stored values
  //ESP.restart();
///////////////////////////////////////////////////////////////////////

}


void loopCore0( void * parameter ){for (;;) {dacWrite(biassource, 127);
  
  calcVI(40,4000);
  //if(Irms<0.04){Irms=0;}
  serialprint();
}}

void loopCore1( void * parameter ){for (;;) {
  dacWrite(biassource, 127);
  //delay(10);
  
  long start = millis();
  
  
//autoconnectwifi();
  autoconnectmqtt();
  kirimdatadariserial();
  /*
   * kirim data ke broker
      realPower,
      apparentPower,
      powerFactor,
      Vrms,
      Irms,
      anglerad,
      angledegree,
      reactivePower,
      lag,
      lead;
  */
    int CuF = cariCuF(Vrms,reactivePower,50);
 //   int LH = cariLH(Vrms,reactivePower,50);
   
 //   client.publish(MQTT_L_PUBLISH_CH, String(LH).c_str());
    client.publish(MQTT_V_PUBLISH_CH, String(Vrms).c_str());
    client.publish(MQTT_I_PUBLISH_CH, String(Irms).c_str());
    client.publish(MQTT_P_PUBLISH_CH, String(realPower).c_str());
    client.publish(MQTT_S_PUBLISH_CH, String(apparentPower).c_str());
    client.publish(MQTT_Q_PUBLISH_CH, String(reactivePower).c_str());
    client.publish(MQTT_PF_PUBLISH_CH, String(powerFactor).c_str());
    client.publish(MQTT_DEG_PUBLISH_CH, String(angledegree).c_str());  
    
    if(laglead==1&&powerFactor<0.95){
      publishll("Arus Lagging");
      Serial.println("Arus Lagging");
      client.publish(MQTT_C_PUBLISH_CH, String(CuF).c_str());}
    if(laglead==0&&powerFactor<0.95){
      publishll("Arus Leading");
       client.publish(MQTT_C_PUBLISH_CH, String(CuF*(-1)).c_str());
      Serial.println("Arus Leading");}
    if(powerFactor>0.95){
      publishll("Sefasa");
       client.publish(MQTT_C_PUBLISH_CH, String(CuF*(0)).c_str());
      Serial.println("Sefasa");}
      
    
    client.loop();   
  
  Serial.print("calculation on Core ");
  Serial.print( xPortGetCoreID());
  Serial.print(" Time ");
  Serial.print(millis() - start);
  Serial.println(" ms");
  delay(1000);
    
}}
