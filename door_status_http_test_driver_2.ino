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

// The Virtual Garage Door Sensor has the ability to display:
//    a description
//    running time
//    motion active, motion inactive
//
// Example uri: 
//    Set description to "Opening":
//    http://192.168.50.29/apps/api/427/devices/757/setVariable/Opening?access_token=41727239-7999-4bc9-95d0-00024e02445b
//
//    Set running time to 7:
//    http://192.168.50.29/apps/api/427/devices/757/setRunningTime/7?access_token=41727239-7999-4bc9-95d0-00024e02445b
//
//    Set motion to active:
//    http://192.168.50.29/apps/api/427/devices/757/motionActive?access_token=41727239-7999-4bc9-95d0-00024e02445b
//
//    Set motion to inactive:
//    http://192.168.50.29/apps/api/427/devices/757/motionInactive?access_token=41727239-7999-4bc9-95d0-00024e02445b
//
//
#include <SoftwareSerial.h>
//#include <ArduinoJson.h>

SoftwareSerial mySerial(10, 11); // RX, TX

// ***************** TODO: ADD TO GARAGE DOOR CONTROLLER
const String URI_SUB1 = "http://192.168.50.29/apps/api/427/devices/757/";
const String URI_SUB2 = "?access_token=41727239-7999-4bc9-95d0-00024e02445b";
const unsigned long DELAY_AFTER_SERIAL_WRITE = 100; // need a delay after writting to SoftwareSerial port on Arduino Uno, might not need this when writting to Arduino Mega UART
// ***************** END TODO *********  

// ***************** TODO: 
//    Add codes for:
//      DOOR_STOPPED_EXCESS_CURRENT_WHILE_OPENING, 
//      DOOR_STOPPED_EXCESS_CURRENT_WHILE_OPENING
//
// ***************** END TODO *********  
enum doorStatus {DOOR_OPEN, DOOR_OPENING, DOOR_CLOSED, DOOR_CLOSING, DOOR_STOPPED};
enum doorStopCause {DOOR_STOPPED_LIMIT_SW, DOOR_STOPPED_TIMEOUT, DOOR_STOPPED_OBSTRUCTION, DOOR_STOPPED_BUTTON_WHILE_OPENING, 
  DOOR_STOPPED_BUTTON_WHILE_CLOSING, DOOR_STOPPED_INIT, DOOR_STOPPED_ACTUATOR_ZERO_CURRENT, DOOR_STOPPED_ACTUATOR_EXCESS_CURRENT, 
  DOOR_STOPPED_REVERSED_ACTUATOR_EXCESS_CURRENT, DOOR_STOPPED_NA};

// ***************** END MODIFY GAR DOOR CODE 

int runSeconds = 0;


// ***************** ADD THIS TO GAR DOOR CODE
//
// getExternalStatusDescrip()
//
// return description of door status, encode spaces as %20 for HTTP GET
//
// ***************** TODO: 
//  Add door ID to description of DOOR_STOPPED_EXCESS_CURRENT_WHILE_OPENING
//
String getExternalStatusDescrip(doorStatus currStatus, doorStopCause stopCause) {  
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
      switch (stopCause) {
        case DOOR_STOPPED_LIMIT_SW:
          descrip = "Stopped%20-%20Limit%20Switch";
          break;
        case DOOR_STOPPED_TIMEOUT:
          descrip = "Stopped%20-%20Timeout";
          break;
        case DOOR_STOPPED_OBSTRUCTION:
          descrip = "Stopped%20-%20Obstruction";
          break;
        case DOOR_STOPPED_BUTTON_WHILE_OPENING:
          descrip = "Stopped%20-%20Button%20While%20Opening";
          break;
        case DOOR_STOPPED_BUTTON_WHILE_CLOSING:
          descrip = "Stopped%20-%20Button%20While%20Closing";
          break;
        case DOOR_STOPPED_INIT:
          descrip = "Stopped%20-%20Init";
          break;
        case DOOR_STOPPED_ACTUATOR_ZERO_CURRENT:
          descrip = "Stopped%20-%20Zero%20Current";
          break;
        case DOOR_STOPPED_ACTUATOR_EXCESS_CURRENT:
          descrip = "Stopped%20-%20Excess%20Current%20(add%20door%20ID%20here)";
          break;
        case DOOR_STOPPED_NA:
          descrip = "Stopped%20-%20NA";
          break;  
        default:
          descrip = "Stopped%20-%20Unknown";
          break;  
      }
      break;
    default:
      descrip = "Unknown";
      break;
  }
  return descrip; 
}

//
// sendStatusDescrip
//
//    Example: Set description to "Opening":
//    http://192.168.50.29/apps/api/427/devices/757/setVariable/Opening?access_token=41727239-7999-4bc9-95d0-00024e02445b
void sendStatusDescrip(String descrip) { 
  String strUri = URI_SUB1 + "setVariable/" + descrip + URI_SUB2;
  Serial.println(strUri);
  mySerial.write(strUri.c_str());
  delay(DELAY_AFTER_SERIAL_WRITE);
}

//
// sendRunningTime
//
//    Example: Set running time to 7:
//    http://192.168.50.29/apps/api/427/devices/757/setRunningTime/7?access_token=41727239-7999-4bc9-95d0-00024e02445b
//
void sendRunningTime(int seconds) {
  String strUri = URI_SUB1 + "setRunningTime/" + String(seconds) + URI_SUB2;
  Serial.println(strUri);
  mySerial.write(strUri.c_str());
  delay(DELAY_AFTER_SERIAL_WRITE);
}

//
// sendMotionActive
//
//    Example: Set motion to active:
//    http://192.168.50.29/apps/api/427/devices/757/motionActive?access_token=41727239-7999-4bc9-95d0-00024e02445b
//
void sendMotionActive() {
  String strUri = URI_SUB1 + "motionActive" + URI_SUB2;
  Serial.println(strUri);
  mySerial.write(strUri.c_str());
  delay(DELAY_AFTER_SERIAL_WRITE);
}

//
// sendMotionInactive
//
//    Example: Set motion to inactive:
//    http://192.168.50.29/apps/api/427/devices/757/motionInactive?access_token=41727239-7999-4bc9-95d0-00024e02445b
//
void sendMotionInactive() {
  String strUri = URI_SUB1 + "motionInactive" + URI_SUB2;
  Serial.println(strUri);
  mySerial.write(strUri.c_str());
  delay(DELAY_AFTER_SERIAL_WRITE);
}

//
// runDoorCycle
//
void runDoorCycle(doorStatus status, doorStopCause stopCause) {
  String strPayload;
  String descrip = getExternalStatusDescrip(status, stopCause);
  
  sendStatusDescrip(descrip);
  if (status == DOOR_OPENING || status == DOOR_CLOSING) {
    sendMotionActive();

    unsigned long runStart = millis();
    unsigned long timerStart = millis();
    while (millis() - runStart < 8000) {
      if (millis() - timerStart > 1000) {
        timerStart = millis();
        runSeconds = (millis() - runStart) / 1000;
        sendRunningTime(runSeconds);
      }
    } 
  } 
  else {
    sendMotionInactive();
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  mySerial.begin(115200); 
}

String descrip;
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

}
