// Garage Door HTTP Interface to Hubitat
//
// Update garage door status and running time to Hubitat via HTTP Get 
// Hard coded to update a Hubitat Device using the "Virtual Temperature Sensor" driver
// 
// Example uri: (using the Virtual Temperature Sensor driver to hold the status code, in this example the device id is 754, the status code is 11)
//    http://192.168.50.29/apps/api/427/devices/754/setTemperature/11?access_token=41727239-7999-4bc9-95d0-00024e02445b
// Example uri: (using the Virtual Temperature Sensor driver to hold the running time, in this example the device id is 755, the running time is 6)
//    http://192.168.50.29/apps/api/427/devices/755/setTemperature/6?access_token=41727239-7999-4bc9-95d0-00024e02445b
//
// 
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
#include <WiFiManager.h>
#include <FS.h>   //Include File System Headers
#include <ArduinoJson.h>

#define DEBUG
#include <serialdebug.h>
#include <debouncedswitch.h>

// Hubitat IP, access tokem, device ids
const int DEVICE_CONFIG_JSON_SIZE = 80; // the JSON for the Hubitat device ids 
const int SERIAL_JSON_SIZE = 80;        // the JSON received over the serial connection
const char* HUB_IP = "192.168.50.29";   // hub IP
const char* HUB_ACCESS_TOKEN = "41727239-7999-4bc9-95d0-00024e02445b"; // access code required by the Hubitat REST interface
const char* CONFIG_FILENAME = "/config.txt"; // store the device IDs in this file
char g_statusDeviceId[8], g_runningTimeDeviceId[8];  // The Hubitat device IDs 

// Pins
const byte WIFI_CONNECTED_LED = 0 ; // on if connected to WiFi (built in red LED on Adafruit HUZZAH ESP8266 breakout)
const byte RESET_CONFIG_BUTTON_PIN = 12 ; // if pressed 3 times within 3 seconds erase all WiFi and config data

// Button - for resetting the board
DebouncedSwitch* pButton;

//
// saveConfig
//
// save the device IDs (g_statusDeviceId, g_runningTimeDeviceId) to a file
//
bool saveConfig(const char* filename) {
  bool ret_val = false;
  //Initialize File System
  if(SPIFFS.begin()) {
    DEBUG_PPRINTLN("SPIFFS Initialize....ok");
    //Format File System
    if(SPIFFS.format()) {
      DEBUG_PRINTLN("File System Formated");
      // Create a JSON document
      StaticJsonDocument<DEVICE_CONFIG_JSON_SIZE> json_doc;
      json_doc["statusDeviceId"] = g_statusDeviceId;
      json_doc["runningTimeDeviceId"] = g_runningTimeDeviceId;
    
      //Create New File And Write Data to It
      //w=Write Open file for writing
      File f = SPIFFS.open(filename, "w");
      if (f) {
        //Write data to file
        DEBUG_PRINTLN("Writing Data to File");
        // Serialize JSON data to write to file
        serializeJson(json_doc, Serial);
        if (serializeJson(json_doc, f) == 0) {
          // Error writing file
          DEBUG_PRINTLN("Failed to write to file");
        }
        else {
          ret_val = true;
        }
        // Close file
        f.close();       
      }
      else {
          DEBUG_PRINTLN("file open failed");
      }
    }
    else {
      DEBUG_PRINTLN("File System Formatting Error");
    }     
  }
  else {
    DEBUG_PRINTLN("SPIFFS Initialization...failed");
  }
  return ret_val;
}

//
// loadConfig
//
// load the device IDs (g_statusDeviceId, g_runningTimeDeviceId) from a file
//
bool loadConfig(const char* filename) {
  bool ret_val = false;
  if(SPIFFS.begin()) {  
    File f = SPIFFS.open(filename, "r");
    if (f) {
      StaticJsonDocument<DEVICE_CONFIG_JSON_SIZE> json_doc;
      DeserializationError error = deserializeJson(json_doc, f);
      serializeJson(json_doc, Serial);
      if (error) {
        // Error loading JSON data
        DEBUG_PRINTLN("Failed to load json config");
      } 
      else {
        strcpy(g_statusDeviceId, json_doc["statusDeviceId"]);
        strcpy(g_runningTimeDeviceId, json_doc["runningTimeDeviceId"]);
        ret_val = true;
      }
    }
    else {    
      DEBUG_PRINTLN("file open failed");
    }
  }
  else {
    DEBUG_PRINTLN("SPIFFS.begin() failed");
  }
  return ret_val;
} 

//
// resetNetworkSettings and erace saved device IDs
//
void resetNetworkSettings() {
  WiFiManager wfm; 
  // Supress Debug information
  wfm.setDebugOutput(false);

  // Remove any previous WiFi settings
  wfm.resetSettings();
  
  // Remove config data
  strcpy(g_statusDeviceId, "");
  strcpy(g_runningTimeDeviceId, "");
  saveConfig(CONFIG_FILENAME);
}

//
// getURI
//
// construct the uri that will be used in the HTTP GET
//
String getURI(const char* deviceId, const String& value) {
  String strURI = "";
  strURI += "http://";
  strURI += HUB_IP;
  strURI += "/apps/api/427/devices/";
  strURI += deviceId;
  strURI += "/setTemperature/";
  strURI += value;
  strURI += "?access_token=";
  strURI += HUB_ACCESS_TOKEN;
  return strURI;
}

//
// updateDevice
//
// Update the garage door status, or garage door running time via HTTP call to Hubitat REST interface
//
// Example uri: (using the Virtual Temperature Sensor driver to hold the status code, in this example the device id is 754, the status code is 11)
//    http://192.168.50.29/apps/api/427/devices/754/setTemperature/11?access_token=41727239-7999-4bc9-95d0-00024e02445b
//
// Example uri: (using the Virtual Temperature Sensor driver to hold the running time (in seconds), in this example the device id is 755, the running time is 6)
//    http://192.168.50.29/apps/api/427/devices/755/setTemperature/6?access_token=41727239-7999-4bc9-95d0-00024e02445b
//
void updateDevice(const char* deviceId, const String& newValue) {
  String strURI = getURI(deviceId, newValue);
  DEBUG_PRINTLN(strURI);
  if(WiFi.status()== WL_CONNECTED){
    WiFiClient client;
    HTTPClient http;    
    http.begin(client, strURI.c_str());
    // Send HTTP GET request
    int httpResponseCode = http.GET();
    if (httpResponseCode>0) {
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
  else {
    DEBUG_PRINTLN("WiFi Disconnected");
  }
} 

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

  // ************ Step 1 - connect to WiFi, or get WiFi and config. info from WiFiManager web page
  WiFiManager wfm; 
  // Supress Debug information
  wfm.setDebugOutput(false);
  
  // Remove any previous network settings (while in development)
  //resetNetworkSettings();

  // Define WiFi Manager custom parameters (status device ID value, and running time ID value)
  WiFiManagerParameter wm_statusDeviceId("statusDeviceId", "Hubitat Status Device ID", "", 4);
  WiFiManagerParameter wm_runningTimeDeviceId("runningTimeDeviceId", "Hubitat Running Time Device ID", "", 4);
 
  // Add custom parameters to WiFi Manager
  wfm.addParameter(&wm_statusDeviceId);
  wfm.addParameter(&wm_runningTimeDeviceId);

  // Attempt to connect, if can't connect setup an Access Point (GARDOOR_AP)
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

  // Check if the user has entered values via WiFi MAnager
  if (strlen(wm_statusDeviceId.getValue()) > 1  ||
      strlen(wm_runningTimeDeviceId.getValue()) > 1 ) {
    // the user has entered new values, so save the new values
    
    DEBUG_PRINT("wm_statusDeviceId.getValue() = ");
    DEBUG_PRINTLN(wm_statusDeviceId.getValue());
    DEBUG_PRINT("wm_runningTimeDeviceId.getValue() = ");
    DEBUG_PRINTLN(wm_runningTimeDeviceId.getValue());
    DEBUG_PRINTLN("saving new drvice id values");

    strcpy(g_statusDeviceId, wm_statusDeviceId.getValue());
    strcpy(g_runningTimeDeviceId, wm_runningTimeDeviceId.getValue());
    
    if (saveConfig(CONFIG_FILENAME)) {
      DEBUG_PRINTLN("config saved");
    }
    else {
      DEBUG_PRINTLN("ERROR  - config NOT saved"); 
    }
  }

  // ************ Step 2 - get device id settings
  if (loadConfig(CONFIG_FILENAME)) {
    DEBUG_PRINTLN("Successfully read config values");

    // Print the config values
    DEBUG_PRINT("config values: ");
    DEBUG_PRINTLN(g_statusDeviceId);
    DEBUG_PRINTLN(g_runningTimeDeviceId);
  }
  else
  {
    DEBUG_PRINTLN("ERROR reading config values");
  }
}

unsigned long restButtonTimerStart = millis();
int resetButtonPressCount = 0;
StaticJsonDocument<SERIAL_JSON_SIZE> json_doc;

//unsigned long timeStart = millis();
//int seconds = 1;
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
      // erase the WiFi and MQTT settings and reset the ESP
      resetButtonPressCount=0;
      resetNetworkSettings();
      delay(100);
      ESP.restart();
    }
  }

  // read from the serial line comming from the Arduino Mega garage door controller, 
  // send the data to the Hubitat hub via HTTP GET
  // JSON:
  // {cmd:<command>, value:<value>}
  //  command will be "status" or "running_time"
  //  value is the value to update
  //
  
//if ((millis() - timeStart) > 500) {
//timeStart = millis();
//seconds++;
//String strPayload = "{cmd:\"running_time\",value:" + String(seconds) + "}";
//DEBUG_PRINTLN(strPayload);

  if (Serial.available() > 0) {
    DEBUG_PRINTLN("Serial.available()");
    Serial.setTimeout(100);
    String strPayload = Serial.readStringUntil('\0');
    if (strPayload.length() > 1) {
      // deserialize json
      DeserializationError error = deserializeJson(json_doc, strPayload.c_str());
      if (error) {
        // Error loading JSON data
        DEBUG_PRINTLN("Failed to load json config");
      } 
      else {
        // update the device value on the Hub via HTTP GET
        String cmd = json_doc["cmd"];
        String value = json_doc["value"];
        if (cmd == "status") {
          // update status on hub
          updateDevice(g_statusDeviceId, value);
        } else if (cmd == "running_time") {
          // update running time on hub        
          updateDevice(g_runningTimeDeviceId, value);
        }
      }
    } else {
      DEBUG_PRINTLN("!strPayload.length() > 1");
    }
  }
}
