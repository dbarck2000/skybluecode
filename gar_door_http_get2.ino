// Garage Door HTTP Interface 
//
// Send HTTP GET - (use URI received from garage door controller)
//
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
#include <WiFiManager.h>
//#include <FS.h>   //Include File System Headers
//#include <ArduinoJson.h>

#define DEBUG
#include <serialdebug.h>
#include <debouncedswitch.h>

// delay after sending HTTP - this seams to improve reliability
//const unsigned long DELAY_AFTER_HTTP = 100;

// Pins
const byte WIFI_CONNECTED_LED = 0 ; // on if connected to WiFi (built in red LED on Adafruit HUZZAH ESP8266 breakout)
const byte RESET_CONFIG_BUTTON_PIN = 12 ; // if pressed 3 times within 3 seconds erase all WiFi and config data

//
// resetNetworkSettings and erace saved device IDs
//
void resetNetworkSettings() {
  WiFiManager wfm; 
  // Supress Debug information
  wfm.setDebugOutput(false);
  // Remove any previous WiFi settings
  wfm.resetSettings();
}

// Button - for resetting the board
DebouncedSwitch* pButton;

//
// setup
//
void setup() {
  Serial.begin (115200);
  pinMode(LED_BUILTIN,OUTPUT);
  pinMode(WIFI_CONNECTED_LED,OUTPUT);
  pinMode(RESET_CONFIG_BUTTON_PIN, INPUT_PULLUP);
  digitalWrite(WIFI_CONNECTED_LED, LOW);

  // Network settings reset button  
  pButton = new DebouncedSwitch(RESET_CONFIG_BUTTON_PIN, LOW); 

  // connect to WiFi, or get WiFi info from WiFiManager web page
  WiFiManager wfm; 
  // Supress Debug information
  wfm.setDebugOutput(false);
  
  // Remove any previous network settings (while in development)
  //resetNetworkSettings();

  // Attempt to connect, if can't connect setup an Access Point (GARDOOR_AP with PW: "password")
  if (!wfm.autoConnect("GARDOOR_AP", "password")) {
    // Did not connect, print error message
    DEBUG_PRINTLN("failed to connect and hit timeout");

    // Reset and try again
    ESP.restart();
    delay(1000);
  }

  DEBUG_PRINTLN("WiFi connected");
  DEBUG_PRINT("IP address: ");
  DEBUG_PRINTLN(WiFi.localIP());
  digitalWrite(WIFI_CONNECTED_LED,LOW);
}

unsigned long restButtonTimerStart = millis();
int resetButtonPressCount = 0;
void loop() {  
  if (pButton->triggered(millis())) {
    DEBUG_PRINTLN("button press");
    while(pButton->triggered(millis())) {
      // loop until button to be released
    }
    if (millis() - restButtonTimerStart > 3000) {
      restButtonTimerStart = millis();
      resetButtonPressCount = 1;
    } else {
      resetButtonPressCount++;
    }
    if (resetButtonPressCount > 2) {
      // the reset button was pressed 3 times within 3 seconds
      // erase the WiFi settings and reset the ESP
      resetButtonPressCount=0;
      resetNetworkSettings();
      delay(100);
      ESP.restart();
    }
  }

  if (Serial.available() > 0) {
    DEBUG_PRINTLN("Serial.available()");
    Serial.setTimeout(100);
    String strPayload = Serial.readStringUntil('\0');
    if (strPayload.length() > 1) {
      if(WiFi.status()== WL_CONNECTED){
        WiFiClient client;
        HTTPClient http;    
        DEBUG_PRINTLN(strPayload);
        http.begin(client, strPayload.c_str());
        // Send HTTP GET 
        int httpResponseCode = http.GET();
        //delay(DELAY_AFTER_HTTP);
        if (httpResponseCode > 0) {
          DEBUG_PRINT("HTTP Response code: ");
          DEBUG_PRINTLN(httpResponseCode);
          String payload = http.getString();
          DEBUG_PRINTLN(payload);
        }
        else {
          DEBUG_PRINT("Error code: ");
          DEBUG_PRINTLN(httpResponseCode);
        }
        // Free resources
        http.end();
      }
    } else {
      DEBUG_PRINTLN("!strPayload.length() > 1");
    }
  }
}
