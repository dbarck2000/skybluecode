// Garage Door MQTT Interface 
//
// Publish status data received from Arduino Mega garage door controller
// Respond to MQTT messages to Open garage door, Close garage door or Toggle garage door (similar to pressing the physical button)
//
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <WiFiManager.h>
#include <FS.h>   //Include File System Headers
#include <ArduinoJson.h>

//#define DEBUG
#include <serialdebug.h>
#include <debouncedswitch.h>

// MQTT connection values
const char* MQTT_CLIENT_ID = "main_gar_door";
const char* MQTT_SUBSCRIBE_TOPIC = "gar_door_cmd";
const char* MQTT_PUBLISH_TOPIC = "gar_door_status";
const char* MQTT_CONFIG_FILENAME = "/mqtt_config.txt";
const int MQTT_CONNECTION_JSON_SIZE = 200;
char mqttServerName[50], mqttServerPort[10], mqttUserid[20], mqttUserpw[20];

// Pins
const byte WIFI_CONNECTED_LED = 0 ; // on if connected to WiFi (built in red LED on Adafruit HUZZAH ESP8266 breakout)
const byte MQTT_CONNECTED_LED = 2 ; // on if connected to MQTT Broker (built in blue LED on Adafruit HUZZAH ESP8266 breakout)
const byte RESET_CONFIG_BUTTON_PIN = 12 ; // if pressed 3 times within 3 seconds erase all WiFi and MQTT data

//
// saveMQTTConfig
//
bool saveMQTTConfig(const char* filename, const char* mqttServerName, const char* mqttServerPort, const char* mqttUserID, const char* mqttUserPW) {
  bool ret_val = false;
  //Initialize File System
  if(SPIFFS.begin()) {
    DEBUG_PPRINTLN("SPIFFS Initialize....ok");
    //Format File System
    if(SPIFFS.format()) {
      DEBUG_PRINTLN("File System Formated");
      // Create a JSON document
      StaticJsonDocument<MQTT_CONNECTION_JSON_SIZE> json_doc;
      json_doc["mqtt_server_name"] = mqttServerName;
      json_doc["mqtt_server_port"] = mqttServerPort;
      json_doc["mqtt_userid"] = mqttUserID;
      json_doc["mqtt_userpw"] = mqttUserPW;
    
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
// loadMQTTConfig
//
bool loadMQTTConfig(const char* filename, char* mqttServerName, char* mqttServerPort, char* mqttUserid, char* mqttUserpw) {
  bool ret_val = false;
  if(SPIFFS.begin()) {  
    File f = SPIFFS.open(filename, "r");
    if (f) {
      StaticJsonDocument<MQTT_CONNECTION_JSON_SIZE> json_doc;
      
      DeserializationError error = deserializeJson(json_doc, f);
      serializeJson(json_doc, Serial);
      if (error) {
        // Error loading JSON data
        DEBUG_PRINTLN("Failed to load json config");
      } 
      else {
        strcpy(mqttServerName, json_doc["mqtt_server_name"]);
        strcpy(mqttServerPort, json_doc["mqtt_server_port"]);
        strcpy(mqttUserid, json_doc["mqtt_userid"]);
        strcpy(mqttUserpw, json_doc["mqtt_userpw"]);
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
// resetNetworkSettings
//
void resetNetworkSettings() {
  WiFiManager wfm; 
  // Supress Debug information
  wfm.setDebugOutput(false);

  // Remove any previous WiFi settings
  wfm.resetSettings();
  // Remove MQTT connection settings
  saveMQTTConfig(MQTT_CONFIG_FILENAME, "", "", "", "");
}

//
// SerialWriteString
//
void SerialWriteString(String strBuff) {
  for (int i=0; i < strBuff.length() + 1; i++) {
    Serial.write(strBuff.charAt(i));
    delay(1); // this appears to be required for timming issues
  }
}

//
// SerialWriteString
//
void SerialWriteString(const char* buff, const int buff_len) {
  for (int i=0; i <= buff_len; i++) {
    Serial.write(buff[i]);
    delay(1); // this appears to be required for timming issues
  }
}

//
// mqttCallback
// process mqtt messages received on the MQTT_SUBSCRIBE_TOPIC topic
// just pass them through over the serial line
//
void mqttCallback(const char *MQTT_SUBSCRIBE_TOPIC, byte *payload, int payload_length) {
  DEBUG_PRINTLN();
  DEBUG_PRINT("Message arrived on Topic:");
  DEBUG_PRINT(MQTT_SUBSCRIBE_TOPIC);

  // pass payload to the Arduino Mega controller via serial line
  payload[payload_length] = '\0';
  SerialWriteString((char*)payload, payload_length);
}

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
DebouncedSwitch* pButton;

//
// setup
//
void setup() {
  Serial.begin (115200);
  pinMode(LED_BUILTIN,OUTPUT);
  pinMode(WIFI_CONNECTED_LED,OUTPUT);
  pinMode(MQTT_CONNECTED_LED,OUTPUT);
  pinMode(RESET_CONFIG_BUTTON_PIN, INPUT_PULLUP);
  digitalWrite(WIFI_CONNECTED_LED, LOW);
  digitalWrite(MQTT_CONNECTED_LED, LOW);

  // Network settings reset button  
  pButton = new DebouncedSwitch(RESET_CONFIG_BUTTON_PIN, LOW); 

  // ************ Step 1 - connect to WiFi, or get WiFi and MQTT connection info from WiFiManager web page
  WiFiManager wfm; 
  // Supress Debug information
  wfm.setDebugOutput(false);
  
  // Remove any previous network settings (while in development)
  //resetNetworkSettings();

  // Define mqtt config value text boxs
  WiFiManagerParameter wm_mqtt_server_name("mqtt1", "MQTT Server Name", "", 50);
  WiFiManagerParameter wm_mqtt_server_port("mqtt2", "MQTT Server Port", "", 4);
  WiFiManagerParameter wm_mqtt_userid("mqtt3", "MQTT User ID", "", 20);
  WiFiManagerParameter wm_mqtt_userpw("mqtt4", "MQTT User PW", "", 20);
 
  // Add MQTT custom parameters to WiFi Manager
  wfm.addParameter(&wm_mqtt_server_name);
  wfm.addParameter(&wm_mqtt_server_port);
  wfm.addParameter(&wm_mqtt_userid);
  wfm.addParameter(&wm_mqtt_userpw);

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

  // Check if the user has entered values for the MQTT connection via WiFi MAnager
  if (strlen(wm_mqtt_server_name.getValue()) > 1  ||
      strlen(wm_mqtt_server_port.getValue()) > 1  ||
      strlen(wm_mqtt_userid.getValue()) > 1  ||
      strlen(wm_mqtt_userpw.getValue()) > 1 ) {
    // the user has entered new values, so save the new values
    if (saveMQTTConfig(MQTT_CONFIG_FILENAME, wm_mqtt_server_name.getValue(), wm_mqtt_server_port.getValue(), wm_mqtt_userid.getValue(), wm_mqtt_userpw.getValue())) {
      DEBUG_PRINTLN("MQTT config saved");
    }
    else {
      DEBUG_PRINTLN("ERROR  - MQTT config NOT saved");
    }
  }

  // ************ Step 2 - get MQTT broker connection settings
  if (loadMQTTConfig(MQTT_CONFIG_FILENAME, mqttServerName, mqttServerPort, mqttUserid, mqttUserpw)) {
    DEBUG_PRINTLN("Successfully read mqtt config values");

    // Print the mqtt config values
    DEBUG_PRINT("MQTT config values: ");
    DEBUG_PRINTLN(mqttServerName);
    DEBUG_PRINTLN(mqttServerPort);
    DEBUG_PRINTLN(mqttUserid);
    DEBUG_PRINTLN(mqttUserpw);
  
    // ************ Step 3 - connect to MQTT broker
    DEBUG_PRINTLN();
    mqttClient.setServer(mqttServerName, atoi(mqttServerPort));
    if(mqttClient.connect(MQTT_CLIENT_ID, mqttUserid, mqttUserpw)) {
      DEBUG_PRINT ("Connected to MQTT Broker");
      digitalWrite(MQTT_CONNECTED_LED,HIGH);
      
      // ************ Step 4 - set callback and subscribe to MQTT topic
      mqttClient.setCallback(mqttCallback);
      mqttClient.subscribe(MQTT_SUBSCRIBE_TOPIC);
    } else {
      DEBUG_PRINT("MQTT Broker connection failed");
      DEBUG_PRINT (mqttClient.state());
      delay(200);
    }
  }
  else
  {
    DEBUG_PRINTLN("ERROR reading mqtt config values");
  }
}

unsigned long restButtonTimerStart = millis();
int resetButtonPressCount = 0;
String strPayload;
void loop() {  
  if (pButton->triggered(millis())) {
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
  // then publish it to the MQTT_PUBLISH_TOPIC 
  if (Serial.available() > 0) {
    DEBUG_PRINTLN("Serial.available()");
    Serial.setTimeout(100);
    strPayload = Serial.readStringUntil('\0');
    if (strPayload.length() > 1) {
      if (mqttClient.connected()){
        DEBUG_PRINT("Sending payload: ");
        DEBUG_PRINT(strPayload);
        DEBUG_PRINT(" [");
        DEBUG_PRINT(strPayload.length());
        DEBUG_PRINTLN("]");  
        if (mqttClient.publish(MQTT_PUBLISH_TOPIC, (char*) strPayload.c_str())) {
          DEBUG_PRINTLN("Publish ok");
        }
        else {
          DEBUG_PRINTLN("Publish failed");
        }     
        strPayload = "";
      } else {
      DEBUG_PRINTLN("!mqttClient.connected()");
      }
    } else {
      DEBUG_PRINTLN("!strPayload.length() > 1");
    }
  }
  mqttClient.loop();
}
