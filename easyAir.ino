/*
  ccs811basic.ino - Demo sketch printing results of the CCS811 digital gas sensor for monitoring indoor air quality from ams.
  Created by Maarten Pennings 2017 Dec 11
*/

#include <Arduino.h>
#include <ESP8266WiFi.h>
// #include <WiFiClientSecure.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include "HTTPSRedirect.h"

#include <Adafruit_Sensor.h>
#include "config.h"

#include <Wire.h>    // I2C library
#include "ccs811.h"  // CCS811 library
//for LED status
#include <Ticker.h>


#define NUM_OF_READINGS 5




// Sensor vars
uint16_t eco2, etvoc, errstat, raw;
float eco2Array[5];
float etvocArray[5];
int currentArray = 0;

// Fake Multitask
unsigned long previousMillis = 0;
unsigned long previousMillisSheets = 0;
unsigned long currentMillis;

// Intervals
long intervalSensor = 250;
long intervalSheets = 5000;



#define PIN_WIRE_SDA (4)
#define PIN_WIRE_SCL (5)



static const uint8_t D0   = 16;
static const uint8_t D1   = 5;
static const uint8_t D2   = 4;
static const uint8_t D3   = 0;
static const uint8_t D4   = 2;
static const uint8_t D5   = 14;
static const uint8_t D6   = 12;
static const uint8_t D7   = 13;
static const uint8_t D8   = 15;
static const uint8_t D9   = 3;
static const uint8_t D10  = 1;

//RGB LED
int redpin = D5; // select the pin for the red LED
int bluepin = D6; // select the pin for the  blue LED
int greenpin = D7; // select the pin for the green LED
int valRGB;
int redValCO2, greenValCO2;

int co2High = 800;


// Ticker ticker;
WiFiManager wm;

// Wiring for ESP8266 NodeMCU boards: VDD to 3V3, GND to GND, SDA to D2, SCL to D1, nWAKE to D3 (or GND)
CCS811 ccs811(D3); // nWAKE on D3

#ifndef LED_BUILTIN
#endif

// Google Sheets setup (do not edit)
const char* host = "script.google.com";
const int httpsPort = 443;
const char* fingerprint = "";
String url = String("/macros/s/") + GScriptId + "/exec";
HTTPSRedirect* client = nullptr;


// Enter command (insert_row or append_row) and your Google Sheets sheet name (default is Sheet1):
String payload_base =  "{\"command\": \"insert_row\", \"sheet_name\": \"Sheet1\", \"values\": ";
String payload = "";

float average (float *array) {
  float sum = 0L ;
  for (int i = 0 ; i < NUM_OF_READINGS ; i++){
    sum += array [i] ;
  }
  return  ((float) sum) / NUM_OF_READINGS ;  // average will be fractional, so float may be appropriate.
}


void setup() {

  // Get LEDS running
  pinMode(redpin, OUTPUT);
  pinMode(bluepin, OUTPUT);
  pinMode(greenpin, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  

  Serial.begin(115200);
  Wire.begin(); 
  // setupWifi();
  wm.autoConnect("lekkerLuchtig");
  Serial.println(WiFi.status());
  if (WiFi.isConnected()) {
      digitalWrite(LED_BUILTIN, HIGH);
  }
  delay(1000);
  // // start ticker with 0.5 because we start in AP mode and try to connect
  // ticker.attach(0.6, tick);

  // Enable CCS811
  ccs811.set_i2cdelay(50); // Needed for ESP8266 because it doesn't handle I2C clock stretch correctly
  bool ok= ccs811.begin();
  if( !ok ) Serial.println("setup: CCS811 begin FAILED");

  // Print CCS811 versions
  Serial.print("setup: hardware    version: "); Serial.println(ccs811.hardware_version(),HEX);
  Serial.print("setup: bootloader  version: "); Serial.println(ccs811.bootloader_version(),HEX);
  Serial.print("setup: application version: "); Serial.println(ccs811.application_version(),HEX);
  
  // Start measuring
  ok= ccs811.start(CCS811_MODE_1SEC);
  delay(1000);

  // Use HTTPSRedirect class to create a new TLS connection
  client = new HTTPSRedirect(httpsPort);
  client->setInsecure();
  client->setPrintResponseBody(true);
  client->setContentTypeHeader("application/json");
  
  Serial.print("Connecting to ");
  Serial.println(host);

  // Try to connect for a maximum of 5 times
  bool flag = false;
  for (int i=0; i<5; i++){ 
    int retval = client->connect(host, httpsPort);
    if (retval == 1){
       flag = true;
       Serial.println("Connected");
       break;
    }
    else
      Serial.println("Connection failed. Retrying...");
  }
  if (!flag){
    Serial.print("Could not connect to server: ");
    Serial.println(host);
    return;
  }
  delete client;    // delete HTTPSRedirect object
  client = nullptr; // delete HTTPSRedirect object

}

// Subroutine for sending data to Google Sheets
void sendData(float valCO2, float valETVOC) {
  Serial.print("connecting to ");
  Serial.println(host);

  Serial.print("requesting URL: ");
  Serial.println(url);

  static bool flag = false;
  if (!flag){
    client = new HTTPSRedirect(httpsPort);
    client->setInsecure();
    flag = true;
    client->setPrintResponseBody(true);
    client->setContentTypeHeader("application/json");
  }
  if (client != nullptr){
    if (!client->connected()){
      client->connect(host, httpsPort);
    }
  }
  else{
    Serial.println("Error creating client object!");
  }


  payload = payload_base + "\"" + String(valCO2) + "," + String(valETVOC) + "," + String(deviceName) + "\"}";


  if(client->POST(url, host, payload)){ 
    // do stuff here if publish was successful
  }
  else{
    // do stuff here if publish was not successful
    Serial.println("Error while connecting");
  }


  } 

void updateSensor() {

  if (currentMillis - previousMillis >= intervalSensor){
    // Read
    ccs811.read(&eco2,&etvoc,&errstat,&raw); 
    redValCO2 = map(eco2,400,co2High, 0, 255);
    greenValCO2 = map(eco2,400,co2High, 255,0 );

    analogWrite(redpin,redValCO2);
    analogWrite(greenpin,greenValCO2);


    if(errstat==CCS811_ERRSTAT_OK ) { 
      Serial.print("CCS811: ");
      Serial.print("eco2=");  Serial.print(eco2);     Serial.print(" ppm  ");
      Serial.print("etvoc="); Serial.print(etvoc);    Serial.print(" ppb  ");
      Serial.println();

      // if (currentArray > 4){
      //   currentArray = 0;
      // }else{
      //   eco2Array[currentArray] = eco2;
      //   etvocArray[currentArray] = etvoc;
      //   currentArray++;
      // }
      
      // Serial.println(average(eco2Array));
      // Serial.println(eco2Array[1], eco2Array[2]);
      previousMillis = millis();

      if (currentMillis - previousMillisSheets >= intervalSheets){
      
      // float etvoc_avg = average(etvocArray);

      // sendData(eco2_avg, etvoc_avg); //--> Calls the sendData Subroutine      
      sendData(eco2, etvoc); //--> Calls the sendData Subroutine      
      previousMillisSheets = millis();

    }


    // } else if( errstat==CCS811_ERRSTAT_OK_NODATA ) {
    //   Serial.println("CCS811: waiting for (new) data");
    // } else if( errstat & CCS811_ERRSTAT_I2CFAIL ) { 
    //   Serial.println("CCS811: I2C error");
    // } else {
    //   Serial.print("CCS811: errstat="); Serial.print(errstat,HEX); 
    //   Serial.print("="); Serial.println( ccs811.errstat_str(errstat) ); 
    // }
    }
  }
   
}



void loop() 
{
  currentMillis = millis();
  updateSensor();

  // postSheet();
  // WifiHandler();
  // Wait
  // delay(1000); 
}

