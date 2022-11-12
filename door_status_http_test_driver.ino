//
//
//
//
// Hubitat Maker AP interface
// Send Device Command (replace [Device ID] with actual subscribed device id and [Command] with a supported command.  Supports optional [Secondary value]
// http://192.168.50.29/apps/api/427/devices/[Device ID]/[Command]/[Secondary value]?access_token=41727239-7999-4bc9-95d0-00024e02445b
//
// Use Hubitat driver: "Virtual Temperature Sensor" to hold the value of the garage door status code 
//
// example (Hubitat Device ID=172, gar. door status code=12): 
// http://192.168.50.29/apps/api/427/devices/172/setTemperature/12?access_token=41727239-7999-4bc9-95d0-00024e02445b
//
//

/*
String http_req;
String deviceId = "172";
http_req = "http://192.168.50.29/apps/api/427/devices/";
http_req+= device_id;
http_req+= "/setTemperature/"
http_req+= String(externalStatusCode);
http_req+= "?access_token=41727239-7999-4bc9-95d0-00024e02445b";
172/setTemperature/12?access_token=41727239-7999-4bc9-95d0-00024e02445b
*/
//
//
#include <SoftwareSerial.h>
#include <ArduinoJson.h>
const int JSON_STATUS_SIZE = 150;
//
// JSON status doc example:
//  {cmd:"status", value:5}
//
// JSON runing time doc example:
//  {cmd:"running_time", value:11}

SoftwareSerial mySerial(10, 11); // RX, TX

// Allocate the JSON document
StaticJsonDocument<JSON_STATUS_SIZE> doc;

// ***************** MODIFY GAR DOOR CODE - add the enum code - as below
// 11/11/2022 - CORRECTIO - need need to change these eums - integer value of the enum is not usec
enum doorStatus {DOOR_NA=0, DOOR_OPEN=1, DOOR_OPENING=2, DOOR_CLOSED=3, DOOR_CLOSING=4, DOOR_STOPPED=5};
enum doorStopCause {DOOR_STOPPED_NA=0, DOOR_STOPPED_LIMIT_SW=1, DOOR_STOPPED_TIMEOUT=2, DOOR_STOPPED_OBSTRUCTION=3, DOOR_STOPPED_BUTTON_WHILE_OPENING=4, 
  DOOR_STOPPED_BUTTON_WHILE_CLOSING=5, DOOR_STOPPED_INIT=6, DOOR_STOPPED_ACTUATOR_ZERO_CURRENT=7, DOOR_STOPPED_ACTUATOR_EXCESS_CURRENT=8, 
  DOOR_STOPPED_REVERSED_ACTUATOR_EXCESS_CURRENT=8};
// ***************** END MODIFY GAR DOOR CODE 

int runSeconds = 0;


// ***************** ADD THIS TO GAR DOOR CODE
//
// getExternalStatusCode()
//
// NOTE: These codes must not change, and must be continious
// These codes are used as the index into the garage_status_list string in the Hubitat global variable
//
int getExternalStatusCode(doorStatus currStatus, doorStopCause stopCause) {  
  int externalCode;
  switch (currStatus) {
    case DOOR_OPEN:
      externalCode = 1;
      break;
    case DOOR_OPENING:
      externalCode = 2;
      break;
    case DOOR_CLOSED:
      externalCode = 3;
      break;
    case DOOR_CLOSING:
      externalCode = 4;
      break;
    case DOOR_STOPPED:
      switch (stopCause) {
        case DOOR_STOPPED_LIMIT_SW:
          externalCode = 5;
          break;
        case DOOR_STOPPED_TIMEOUT:
          externalCode = 5;
          break;
        case DOOR_STOPPED_OBSTRUCTION:
          externalCode = 6;
          break;
        case DOOR_STOPPED_BUTTON_WHILE_OPENING:
          externalCode = 7;
          break;
        case DOOR_STOPPED_BUTTON_WHILE_CLOSING:
          externalCode = 8;
          break;
        case DOOR_STOPPED_INIT:
          externalCode = 9;
          break;
        case DOOR_STOPPED_ACTUATOR_ZERO_CURRENT:
          externalCode = 10;
          break;
        case DOOR_STOPPED_ACTUATOR_EXCESS_CURRENT:
          externalCode = 11;
          break;
        case DOOR_STOPPED_NA:
          externalCode = 12;
          break;  
        default:
          externalCode = 0;
          break;  
      }
      break;
    default:
      externalCode = 0;
      break;
  }
  return externalCode; 
}

//
// sendStatusUpdate
//
void sendStatusUpdate(byte externalStatusCode) {

  String statusJson;
  statusJson = "{cmd:\"status\", value:";
  statusJson+= String(externalStatusCode);
  statusJson+= "}";
  
  Serial.println(statusJson);
  mySerial.write(statusJson.c_str());
}

//
// sendRuntimeUpdate
//
void sendRuntimeUpdate(int seconds) {

  String statusJson;
  statusJson = "{cmd:\"running_time\", value:";
  statusJson+= String(seconds);
  statusJson+= "}";
  
  Serial.println(statusJson);
  mySerial.write(statusJson.c_str());
}

//
// runDoorCycle
//
void runDoorCycle(doorStatus status, doorStopCause stopCause) {
  Serial.print("status:");
  Serial.print(status);
  Serial.print("  stop cause:");
  Serial.println(stopCause);
  sendStatusUpdate(getExternalStatusCode(status, stopCause));
  
  if (status == DOOR_OPENING || status == DOOR_CLOSING) {
    sendRuntimeUpdate(0);
    unsigned long runStart = millis();
    unsigned long timerStart = millis();
    while (millis() - runStart < 10000) {
      if (millis() - timerStart > 1000) {
        timerStart = millis();
        runSeconds = (millis() - runStart) / 1000;
        sendRuntimeUpdate(runSeconds);
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  mySerial.begin(115200); 
}

void loop() {
  runDoorCycle(DOOR_CLOSED, DOOR_STOPPED_NA);
  delay(2000);
  runDoorCycle(DOOR_OPENING, DOOR_STOPPED_NA);
  delay(2000);
  runDoorCycle(DOOR_OPEN, DOOR_STOPPED_NA);
  delay(2000);
  runDoorCycle(DOOR_CLOSING, DOOR_STOPPED_NA);
  delay(2000);
  runDoorCycle(DOOR_STOPPED, DOOR_STOPPED_OBSTRUCTION);
  delay(2000);
  runDoorCycle(DOOR_STOPPED, DOOR_STOPPED_ACTUATOR_EXCESS_CURRENT);
  delay(2000);
  runDoorCycle(DOOR_STOPPED, DOOR_STOPPED_ACTUATOR_EXCESS_CURRENT);
  delay(2000);

  
}
