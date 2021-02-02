/* LuminousThings - A color lamp controller for Alexa Home Automation
 * Copyright (C) 2021  David L Norris <dave@webaugur.com>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define ESPALEXA_ASYNC
#include <Espalexa.h>

#ifdef ARDUINO_ARCH_ESP32
#include <WiFi.h>
#include <AsyncTCP.h>
#else
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#endif
#include <ESPAsyncWebServer.h>
#include <SoftwareSerial.h>
#include <FastLED.h>

// WiFi Access Point Secrets
// Copy and Edit creds.example.h to creds.h
#include "creds.h"

/* Debugging */
#define DEBUG_SERIAL 1  // Print detailed command data to serial monitor

/* Hardware Interfacing */
#define PIN_PWR    0       // Node D3 - ATX Power Switch - Low On
#define PIN_OUT_1 13       // Node D7 - MOSFET 1 - 12VDC
#define PIN_OUT_2 12       // Node D6 - MOSFET 2 - 12VDC
#define PIN_OUT_3 14       // Node D5 - MOSFET 3 - 12VDC
#define PIN_OUT_4 16       // Node D0 - MOSFET 4 - 12VDC
#define PIN_SPI_1 5        // Node D1 - SPI GPIO Pin
#define PIN_SPI_2 4        // Node D2 - SPI GPIO Pin - @TODO CURRENTLY UNUSED
#define NUM_LEDS 10        // 10 x 3 LED segments
// #define LED_CHIPSET TM1803 // Radio shack's chipset (SKU 2760339)
#define LED_CHIPSET WS2811
// #define COLOR_ORDER GBR    // LED bit order is Green Blue Red.
#define COLOR_ORDER BRG

// Global Variables
CRGB leds[NUM_LEDS];
uint32_t g_color;
long n;
boolean wifiConnected = false;

// Power Management Hack
uint32_t portcount = 0;

// Prototype utility functions
boolean connectWifi();

// Prototype callback functions
void colorLightChanged(uint8_t brightness, uint32_t rgb);

// Initialize Singletons
Espalexa LuminousThings;
AsyncWebServer HttpService(80);

// Setup is called at bootup
void setup()
{  
  // On/Off/Dimmable 12VDC outputs
  pinMode(PIN_OUT_1, OUTPUT);
  pinMode(PIN_OUT_2, OUTPUT);
  pinMode(PIN_OUT_3, OUTPUT);
  pinMode(PIN_OUT_4, OUTPUT);
  pinMode(PIN_PWR, OUTPUT);

  // On/Off/Dim/RGB LED strips
  FastLED.addLeds<LED_CHIPSET, PIN_SPI_1, COLOR_ORDER>(leds, NUM_LEDS);
//  FastLED.addLeds<LED_CHIPSET, PIN_SPI_2, COLOR_ORDER>(leds, NUM_LEDS);

  // Serial Console baud rate
  Serial.begin(57600);
  
  // Initialise wifi connection
  wifiConnected = connectWifi();

  // If WiFi is connected
  if(wifiConnected){
    
    // Set "/" URL callback handler to send text response
    HttpService.on("/", HTTP_GET, [](AsyncWebServerRequest *HttpRequest){
      HttpRequest->send(200, "text/plain", "These are not the droids you are looking for.");
    });

    // Set Alexa Control callback function
    HttpService.onNotFound([](AsyncWebServerRequest *HttpRequest){
      // Unknown Request gets a 404 (srsly, it should be 410.)
      if (!LuminousThings.handleAlexaApiCall(HttpRequest))
      {
        HttpRequest->send(404, "text/plain", "Not found");
      }
    });


    // Power ATX
    digitalWrite(PIN_PWR, 0);
     
    // Devices Exposed by this System.
    LuminousThings.addDevice("Light Box 1", boxLight);
    LuminousThings.addDevice("Sky Light 1", skyLightOne);
//    LuminousThings.addDevice("Sky Light 2", skyLightOne);
//    LuminousThings.addDevice("Spot Light 1", spotLightOne);
//    LuminousThings.addDevice("Spot Light 2", spotLightTwo);
//    LuminousThings.addDevice("Spot Light 3", spotLightThree);
//    LuminousThings.addDevice("Spot Light 4", spotLightFour);

    // Start Alexa Service
    LuminousThings.begin(&HttpService);
    
  } else
  {
    // Couldn't connected WiFi on boot
    // @TODO Enter WiFi AP mode and present config UI
    while (1)
    {
      Serial.println("Cannot connect to WiFi. Please check data and reset the ESP.");
      delay(2500);
    }
  }
}

// This is the main loop
void loop()
{
  // Enter Alexa Service main loop
   LuminousThings.loop();

   // Wait a moment then loop back to beginning
   delay(1);
}

// Alexa "Light Box 1" callback function
void boxLight(uint8_t brightness, uint32_t rgb) {
  // Turn on the Main Power Supply if brightness <> 0
  if(brightness){
    digitalWrite(PIN_PWR, 0);
  }
  // Otherwise turn it off
  else {
    digitalWrite(PIN_PWR, 1);
  }
}

// Alexa "Change Light" callback function
void skyLightOne(uint8_t brightness, uint32_t rgb) {
  // Clear output buffer
  FastLED.clear();
  // set brightness level
  FastLED.setBrightness(brightness);

  // Queue the same command NUM_LEDS times
  for(int led = 0; led < NUM_LEDS; led++) {
      leds[led] = rgb; // Literally just pass the uint32_t in as we got it 
  }
  
  // Send the commands to the LEDs
  FastLED.show();

  // serial debug 
  if(DEBUG_SERIAL) {
    // These are broken out for debug
    uint8_t red   = (rgb >> 16) & 0xFF; 
    uint8_t green = (rgb >> 8) & 0xFF; 
    uint8_t blue  = rgb & 0xFF;
    
    // Tell Human what Alexa sent
    Serial.print("Sky Light 1; ");
    Serial.print("port count: ");
    Serial.print(portcount);
    Serial.print(", Brightness: ");
    Serial.print(brightness);
    Serial.print(", Red: ");
    Serial.print(red); //get red component
    Serial.print(", Green: ");
    Serial.print(green); //get green
    Serial.print(", Blue: ");
    Serial.println(blue); //get blue
  }
}

// Alexa "Spot Light 1" callback function
void spotLightOne(uint8_t brightness, uint32_t rgb) {
  // Brightness is 100%
  if( brightness == 255 ){
    analogWrite(PIN_OUT_1, 1023);
  }
  // Brightness is less than 100%
  else if( brightness ){
    analogWrite(PIN_OUT_1, (brightness * 4));
  }
  // Brightness is zero
  else { 

    analogWrite(PIN_OUT_1, 0);
  }

  // serial debug 
  if(DEBUG_SERIAL) {
    // Tell Human what Alexa sent
    Serial.print("Spot Light 1; ");
    Serial.print("Brightness: ");
    Serial.println(brightness);
  }
}

// Alexa "Spot Light 2" callback function
void spotLightTwo(uint8_t brightness, uint32_t rgb) {
  // Brightness is 100%
  if( brightness == 255 ){
    analogWrite(PIN_OUT_2, 1023);
  }
  // Brightness is less than 100%
  else if( brightness ){
    analogWrite(PIN_OUT_2, (brightness * 4));
  }
  // Brightness is zero
  else { 
    analogWrite(PIN_OUT_2, 0);
  }

  // serial debug 
  if(DEBUG_SERIAL) {
    // Tell Human what Alexa sent
    Serial.print("Spot Light 2; ");
    Serial.print("Brightness: ");
    Serial.println(brightness);
  }
}

// Alexa "Spot Light 3" callback function
void spotLightThree(uint8_t brightness, uint32_t rgb) {
  // Brightness is 100%
  if( brightness == 255 ){
    analogWrite(PIN_OUT_3, 1023);
  }
  // Brightness is less than 100%
  else if( brightness ){
    analogWrite(PIN_OUT_3, (brightness * 4));
  }
  // Brightness is zero
  else { 
    analogWrite(PIN_OUT_3, 0);
  }

  // serial debug 
  if(DEBUG_SERIAL) {
    // Tell Human what Alexa sent
    Serial.print("Spot Light 3; ");
    Serial.print("Brightness: ");
    Serial.println(brightness);
  }
}

// Alexa "Spot Light 4" callback function
void spotLightFour(uint8_t brightness, uint32_t rgb) {
  // Brightness is 100%
  if( brightness == 255 ){
    analogWrite(PIN_OUT_4, 1023);
  }
  // Brightness is less than 100%
  else if( brightness ){
    analogWrite(PIN_OUT_4, (brightness * 4));
  }
  // Brightness is zero
  else { 
    analogWrite(PIN_OUT_4, 0);
  }

  // serial debug 
  if(DEBUG_SERIAL) {
    // Tell Human what Alexa sent
    Serial.print("Spot Light 4; ");
    Serial.print("Brightness: ");
    Serial.println(brightness);
  }
}

// connect to wifi – returns true if successful or false if not
boolean connectWifi(){
  boolean state = true;
  int i = 0;
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");
  Serial.println("Connecting to WiFi");

  // Wait for connection
  Serial.print("Connecting...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (i > 20){
      state = false; break;
    }
    i++;
  }
  Serial.println("");
  if (state){
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  }
  else {
    Serial.println("Connection failed.");
  }
  delay(100);
  return state;
}
