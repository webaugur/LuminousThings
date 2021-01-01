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

/* Hardware Interfacing */
// Onboard LED
#define PIN_RED 12
#define PIN_GREEN 14
#define PIN_BLUE 16

// Outboard LED strip
#define NUM_LEDS 10        // 10 x 3 LED segments
#define DATA_PIN 4         // SPI GPIO Pin
#define LED_CHIPSET TM1803 // Radio shack's chipset (SKU 2760339)
#define COLOR_ORDER GBR    // LED bit order is Green Blue Red.

// Global Variables
CRGB leds[NUM_LEDS];
uint32_t g_color;
long n;
boolean wifiConnected = false;

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
  // Onboard RGB LED pin modes
  pinMode(PIN_RED, OUTPUT);
  pinMode(PIN_GREEN, OUTPUT);
  pinMode(PIN_BLUE, OUTPUT);

  // Outboard LED strip
  FastLED.addLeds<LED_CHIPSET, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS);

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

    // Devices Exposed by this System.
    LuminousThings.addDevice("Sky Light 1", colorLightChanged);

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

// Alexa "Change Light" callback function
void colorLightChanged(uint8_t brightness, uint32_t rgb) {
  uint8_t red   = (rgb >> 16) & 0xFF; 
  uint8_t green = (rgb >> 8) & 0xFF; 
  uint8_t blue  = rgb & 0xFF;

  /* Onboard RGB LED */
  // Brightness is non-zero
  if( brightness ){
    analogWrite(PIN_RED, red);
    analogWrite(PIN_GREEN, green);
    analogWrite(PIN_BLUE, blue);
  }
  // Brightness is zero
  else { 
    digitalWrite(PIN_RED, 0);
    digitalWrite(PIN_GREEN, 0);
    digitalWrite(PIN_BLUE, 0);
  }

  /* Outboard LED strip */
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
    
  // Tell Human what Alexa sent
  Serial.print("Brightness: ");
  Serial.print(brightness);
  Serial.print(", Red: ");
  Serial.print(red); //get red component
  Serial.print(", Green: ");
  Serial.print(green); //get green
  Serial.print(", Blue: ");
  Serial.println(blue); //get blue

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
