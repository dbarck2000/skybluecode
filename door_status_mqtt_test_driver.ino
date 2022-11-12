#include <SoftwareSerial.h>
#include <ArduinoJson.h>
const int JSON_STATUS_SIZE = 150;
//
// JSON status doc example:
//  {"status_cd":5, "stop_cd":4, "descrip":"Stopped - Button pressed while opening"}
//
// JSON command doc examples:
//  {"cmd":"status"}  // publish the current status
//  {"cmd":"open"}    // open the door
//  {"cmd":"close"}   // close the door
//  {"cmd":"button"}  // same as pressing the physical button

SoftwareSerial mySerial(10, 11); // RX, TX

// Allocate the JSON document
StaticJsonDocument<JSON_STATUS_SIZE> json_status_doc;

// ***************** MODIFY GAR DOOR CODE - add the enum code - as below
//NOTE: Do not change the value of the emun code (example: DOOR_OPEN=1), the emun code is used by outside systems as a code to trigger actions
enum doorStatus {DOOR_NA=0, DOOR_OPEN=1, DOOR_OPENING=2, DOOR_CLOSED=3, DOOR_CLOSING=4, DOOR_STOPPED=5};
enum doorStopCause {DOOR_STOPPED_NA=0, DOOR_STOPPED_LIMIT_SW=1, DOOR_STOPPED_TIMEOUT=2, DOOR_STOPPED_OBSTRUCTION=3, DOOR_STOPPED_BUTTON_WHILE_OPENING=4, 
  DOOR_STOPPED_BUTTON_WHILE_CLOSING=5, DOOR_STOPPED_INIT=6, DOOR_STOPPED_ACTUATOR_ZERO_CURRENT=7, DOOR_STOPPED_ACTUATOR_EXCESS_CURRENT=8, 
  DOOR_STOPPED_REVERSED_ACTUATOR_EXCESS_CURRENT=8};
// ***************** END MODIFY GAR DOOR CODE 



// ***************** ADD THIS TO GAR DOOR CODE
//
// getStatusDescrip()
//
String getStatusDescrip(doorStatus currStatus) {  
  String descrip;
  switch (currStatus) {
    case DOOR_OPEN:
      descrip = "Open";
      break;
    case DOOR_OPENING:
      descrip = "Opening";
      break;
    case DOOR_CLOSED:
      descrip = "Closed";
      break;
    case DOOR_CLOSING:
      descrip = "Closing";
      break;
    case DOOR_STOPPED:
      descrip = "Stopped";
      break;
    default:
      descrip = "UNKNOWN";
      break;
  }
  return descrip; 
}

//
// getStopCauseDescrip()
//
String getStopCauseDescrip(doorStopCause stopCause) {  
  String descrip;
  switch (stopCause) {
    case DOOR_STOPPED_LIMIT_SW:
      descrip = "Limit Switch";
      break;
    case DOOR_STOPPED_TIMEOUT:
      descrip = "Timout";
      break;
    case DOOR_STOPPED_OBSTRUCTION:
      descrip = "Obstruction";
      break;
    case DOOR_STOPPED_BUTTON_WHILE_OPENING:
      descrip = "Button Pressed While Opening";
      break;
    case DOOR_STOPPED_BUTTON_WHILE_CLOSING:
      descrip = "Button Pressed While Closing";
      break;
    case DOOR_STOPPED_INIT:
      descrip = "Init";
      break;
    case DOOR_STOPPED_ACTUATOR_ZERO_CURRENT:
      descrip = "Zero Current";
      break;
    case DOOR_STOPPED_ACTUATOR_EXCESS_CURRENT:
      descrip = "Jammed (Excess Current)";
      break;
    case DOOR_STOPPED_NA:
      descrip = "NA";
      break;  
    default:
      descrip = "UNKNOWN";
      break;  
  }
  return descrip; 
}

//
// publishStatus()
//
// Output a JSON document to a serial port - to be read and published via ESP8266 module over WiFi MQTT
//
//  JSON format:
// {"status": <status code>, "stop_cause": <stop cause code>, "Descrip": <description>}
//
// status code and stop cause code are defined in the enumerated types of doorStatus and doorStopCause
//
void publishStatus(doorStatus currStatus, doorStopCause stopCause) {
  //doorStatus currStatus = getCurrStatus();
  //doorStopCause stopCause = getCurrStopCause();
  
  String descrip = getStatusDescrip(currStatus);
  if (currStatus == DOOR_STOPPED) {
    descrip += " - " + getStopCauseDescrip(stopCause);;
  }
 
  json_status_doc["status_cd"] = currStatus;
  json_status_doc["stop_cd"] = stopCause;
  json_status_doc["descrip"] = descrip;

  Serial.println();
  serializeJson(json_status_doc, Serial);
  Serial.println();
  serializeJson(json_status_doc, mySerial);
  Serial.println("=============================================");
 
  return; 
}
// ***************** END ADD THIS TO GAR DOOR CODE

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // ***************** MODIFY GAR DOOR CODE - open the second serial port - connected to the ESP8266
  mySerial.begin(115200); 
}

String payload;
void loop() {
  // ***************** MODIFY GAR DOOR CODE - 
  // call publishStatus every time the status changes
  // also call publishStatus if JSON of "cmd":"status" is received
  //
  publishStatus(DOOR_OPEN, DOOR_STOPPED_NA);
  delay(2000);
  publishStatus(DOOR_STOPPED, DOOR_STOPPED_BUTTON_WHILE_OPENING);
  delay(2000);
  
  // listen for input from ESP8266 over serial line...
  if (mySerial.available() > 0) {
    mySerial.setTimeout(100); 
    payload = mySerial.readStringUntil('\0');
    if (payload.length() > 1) {
      Serial.print("from ESP8266: ");
      Serial.println(payload);
  
      // parse and process JSON
      StaticJsonDocument<JSON_STATUS_SIZE> doc;
      DeserializationError error = deserializeJson(doc, payload);
      if (error) {
        // Error loading JSON data
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.f_str());
      } 
      else {
        String command = doc["cmd"];
        Serial.print("doc[\"cmd\"]: ");
        Serial.println(command);
        if (command == "status") {
          publishStatus(DOOR_STOPPED, DOOR_STOPPED_BUTTON_WHILE_OPENING);
        }
      }
    }
  }
  delay(2000);
}
