// Gar_Door_V2
//
// Created Dec 1, 2020
// Created by David Barck
//
// 08/15/2022 Added more debugging, coments, changed names of some objects
// 05/10/2021 Removed interrupt driven timer for blinking the "operate" button - this was just adding unnecessary complication 
//
// Garage Door Opener Controller
//
// The Micro Controller board is an Arduino Mega
// There is a separate motor driver board to drive the 2 linear actuators
// The actuators run at 12 molts
// The IR "beam break" sensor runs at 23.5 volts (voltage supplied via a boost voltage converted)
//
// Open and Close the bifold garage doors using a linear actuator attached to each door. Extending the actuator opens the door, retracting closes the door.
// The open or close operation is triggered via a momentary button, or a wireless remote control relay, or a wifi controlled relay
// Each door has open and closed limit switchs to indicate when the door is fully open, or fully closed.
// (each actuator also has internal limit switchs that stop the actuator if it becomes fully extended or fully retracted)
// There is an IR "beam break" sensor to detect if there is an obstruction in the door opening
// 
// While each door is opening or closing, check the appropriate limit switch . Stop the door when the limit switch is triggered.
// Stop both doors if:
//    - the button (or remote button) is pressed during door operation
//    - the IR obstruction sensor is triggered and the door is closing (no need to check for obstruction when the door is opening)
//    - excess current draw is detected (indicating that one or both doors are jammed)
//    - very low current is detected (indicating that the actuator has reached the limit of travel - this should never happen)
//    - the open or closing operation has been running for excess time
//
//#define DEBUG
#include <serialdebug.h>
#include <debouncedswitch.h>

// Global constants 
const unsigned long g_maxRuntimeMillis = 15000; // max run time before timing out
const int g_startRampDownMillis = 10000;       // elapsed opening or closing time before starting to ramp the actuator speed down;
const int g_rampSpeedChange = 1;               // number of steps to increment or decrement the actuator speed when ramping up or ramping down
const int g_startSpeed = 150;                  // initial actuator speed when opening or closing (0 to 255)
const int g_maxCurrent = 1500;                 // if current exceeds this value for an extended time, then stop 
const int g_minCurrent = 200;                  // if current falls below this value for an extended time, then consider the door open or closed due to the actuator internal limit switch hit     
const int g_maxMillisAtOverCurrent = 250;      // max time the current can exceed the max current
const int g_maxMillisAtUnderCurrent = 250;     // max time the current be under the min current before the door is considered open or closed
const int g_d1OpenStopDelayMillis = 0;         // milliseconds to delay after the open limit switch is triggered before stopping the actuator
const int g_d1ClosedStopDelayMillis = 0;       // milliseconds to delay after the closed limit switch is triggered before stopping the actuator
const int g_d2OpenStopDelayMillis = 0;         // milliseconds to delay after the open limit switch is triggered before stopping the actuator
const int g_d2ClosedStopDelayMillis = 0;       // milliseconds to delay after the closed limit switch is triggered before stopping the actuator

// enum types
enum actuatorStatus {ACTUATOR_EXTENDING, ACTUATOR_RETRACTING, ACTUATOR_STOPPED}; // EXTENING - opens the door, RETRACTING - closes the door
enum doorStatus {DOOR_OPEN, DOOR_OPENING, DOOR_CLOSED, DOOR_CLOSING, DOOR_STOPPED};
enum doorStopCause {DOOR_STOPPED_LIMIT_SW, DOOR_STOPPED_TIMEOUT, DOOR_STOPPED_OBSTRUCTION, DOOR_STOPPED_BUTTON_WHILE_OPENING, 
  DOOR_STOPPED_BUTTON_WHILE_CLOSING, DOOR_STOPPED_INIT, DOOR_STOPPED_ACTUATOR_ZERO_CURRENT, DOOR_STOPPED_ACTUATOR_EXCESS_CURRENT, DOOR_STOPPED_REVERSED_ACTUATOR_EXCESS_CURRENT, DOOR_STOPPED_NA};
enum doorMovingLEDMode {DM_ON, DM_OFF};
enum rampMode {RAMP_UP, RAMP_DOWN, RAMP_NONE};

// Global Pin assignments
// actuator dirver board pins //////////////////////////
// actuator 1
const byte g_m1FBPin = A2;    // current feedback input
const byte g_m1SFPin = 47;    // status flag input
const byte g_m1PWMD2Pin = 11; // PWM / Disable 2 - speed control
const byte g_m1In1Pin = 44;   // logic control of actuator OUT 1
const byte g_m1In2Pin = 45;   // logic control of actuator OUT 2
// actuator 2
const byte g_m2FBPin = A10;    // same as for actuator 1    
const byte g_m2SFPin = 50;      
const byte g_m2PWMD2Pin = 12;  
const byte g_m2In1Pin = 48;   
const byte g_m2In2Pin = 49;   
// this pin applies to both actuators
const byte g_enPin = 39;      // Enable input: when EN is LOW, the both actuator driver ICs are in a low-current sleep mode.
// end actuator driver board pins

// open-close-stop button //////////////////////////
const byte g_buttonPin = 33; // momentary button to indicate that both doors should open, close, or stop, triggered when signal goes HIGH

// obstruciton sensor     //////////////////////////
const byte g_obstructionSensorPin = 35;      // obstruction detected when signal goes low
const byte g_onOffObstructionSensorPin = 53; // turns obstruction sensor transmitter and receiver on or off (on = high, off = low)

// limit switches and approach sensors //////////////////////////
// door 1
const byte g_d1OpenLimitSwPin = 22;       // door 1 is open when signal goes LOW
const byte g_d1ClosedLimitSwPin = 23;     // door 1 is closed when signal goes LOW
const byte g_d1ApproachingOpenPin = 24;   // FUTURE USE: door is approaching open when signal goes momentarily LOW 
const byte g_d1ApproachingClosedPin = 25; // FUTURE USE: door is approaching closed when signal goes momentarily LOW 
//door 2
const byte g_d2OpenLimitSwPin = 28;       // door 2 is open when signal goes LOW
const byte g_d2ClosedLimitSwPin = 29;     // door 2 is closed when signal goes LOW
const byte g_d2ApproachingOpenPin = 30;   // FUTURE USE: door is approaching open when signal goes momentarily LOW 
const byte g_d2ApproachingClosedPin = 31; // FUTURE USE: door is approaching closed when signal goes momentarily LOW 

// LEDs
const byte g_excessCurrentFaultLEDPin = 3 ;  // light if doorStopCause is DOOR_STOPPED_ACTUATOR_EXCESS_CURRENT
const byte g_doorMovingLEDPin = 4;           // light when door is moving
const byte g_timeoutFaultLEDPin =  5;        // light if doorStopCause is DOOR_STOPPED_TIMEOUT
const byte g_obstructionFaultLEDPin =  6;    // light if doorStopCause is DOOR_STOPPED_OBSTRUCTION

// end Global pin assignments

///////////////////////////////////////////////////////////////////////////////////////////////
// Class Definitions
///////////////////////////////////////////////////////////////////////////////////////////////

//
// Actuator class 
//
class Actuator {
  public:
    Actuator(byte fBPin, byte sFPin, byte pWMD2Pin, byte in1Pin, byte in2Pin, byte g_enPin);
    void setSpeed(int speed);
    void extend();
    void extend(unsigned int speed);
    void retract();
    void retract(unsigned int speed);
    void stop(); 
    rampMode rampUp();
    rampMode rampDown();
    unsigned int getCurrent(); 
    bool isFault(); // Get fault reading.
    unsigned int getMaxSpeed() {return maxSpeed;};
  private:
    // arduino pins
    byte mFBPin;    
    byte mSFPin;    
    byte mPWMD2Pin;  
    byte mIn1Pin;   
    byte mIn2Pin;       
    byte mEnPin;  // this pin applys to both actuators, the same pin is used for both actuators

    actuatorStatus currStatus = ACTUATOR_STOPPED;
    const unsigned int maxSpeed = 255;
    unsigned int currSpeed = 0;
};
// end Actuator class prototype

//
// Actuator class implimentation
//
Actuator::Actuator(byte fBPin, byte sFPin, byte pWMD2Pin, byte in1Pin, byte in2Pin, byte g_enPin) {
  mFBPin = fBPin;
  mSFPin = sFPin;
  mPWMD2Pin = pWMD2Pin;
  mIn1Pin = in1Pin;
  mIn2Pin = in2Pin;
  mEnPin = g_enPin;
  digitalWrite(mEnPin, HIGH);    // enable input
}

//
// setSpeed
//
void Actuator::setSpeed(int speed) {
  if (speed > maxSpeed) {  // Max PWM dutycycle
    speed = maxSpeed;
  } else if (speed < 0) {
    speed = 0;
  }
  analogWrite(mPWMD2Pin, speed);  
  currSpeed = speed;
}

//
// extend
// extend the linear actuator
//
void Actuator::extend(){
  DEBUG_PRINTLN();
  extend(maxSpeed);
}

//
// extend
// extend the linear actuator
//
void Actuator::extend(unsigned int speed){
  setSpeed(speed);
  digitalWrite(mIn1Pin,HIGH);
  digitalWrite(mIn2Pin,LOW);
  currStatus = ACTUATOR_EXTENDING;
}

//
// retract
// retrct the linear actuator
//
void Actuator::retract(){
  DEBUG_PRINTLN();
  retract(maxSpeed);
}

//
// retract
// retrct the linear actuator
//
void Actuator::retract(unsigned int speed){
  setSpeed(speed);
  digitalWrite(mIn1Pin,LOW);
  digitalWrite(mIn2Pin,HIGH);
  currStatus = ACTUATOR_RETRACTING;
}

//
// stop
//
void Actuator::stop(){
  setSpeed(0);
  digitalWrite(mIn1Pin,LOW);
  digitalWrite(mIn2Pin,LOW);
  currStatus = ACTUATOR_STOPPED;
}

//
// rampUp
//
rampMode Actuator::rampUp() {
  if (currSpeed < maxSpeed) {
    setSpeed(currSpeed + g_rampSpeedChange);
    return RAMP_UP;
  } else {
    return RAMP_NONE;
  }
}

//
// rampDown
//
rampMode Actuator::rampDown() {
  if (currSpeed > 0) {
    setSpeed(currSpeed - g_rampSpeedChange);
    return RAMP_DOWN;
  } else {
    return RAMP_NONE;
  }
}

//
// getCurrent
// get the milliamps geing drawn by the actuator
//
unsigned int Actuator::getCurrent() {
  // 5V / 1024 ADC counts / 525 mV per A = 9 mA per count
  return analogRead(mFBPin) * 9;
}

//
// isFault
//
bool Actuator::isFault() {
  return digitalRead(mSFPin) == LOW;
}

//
// end class Actuator
//

//
// Door class 
//
class Door {
  public:
    Door(Actuator* actuatorPtr, byte openLimitSwPin, byte approachingOpenPin, byte closedLimitSwPin, byte approachingClosedPin, int openStopDelayMillis, int closedStopDelayMillis);
    void init();
    void open();
    void close();
    void stop(doorStatus status, doorStopCause stopCause);
    rampMode doorRampMode = RAMP_NONE;
    void processEvents();
    doorStatus getPrevStatus(){return prevStatus;};
    doorStatus getCurrStatus(){return currStatus;};
    doorStopCause getCurrStopCause() {return currStopCause;};
    Actuator* dActuatorPtr;
  private:
    byte dOpenLimitSwPin;
    byte dApproachingOpenPin;
    byte dClosedLimitSwPin;
    byte dApproachingClosedPin;
    DebouncedSwitch* dOpenLimitSwPtr; 
    DebouncedSwitch* dApproachingOpenPtr; 
    DebouncedSwitch* dClosedLimitSwPtr; 
    DebouncedSwitch* dApproachingClosedPtr;

    unsigned long startMillis = 0;
    bool isOverMaxCurrent = false;
    unsigned long overMaxCurrentStartMillis = millis();
    unsigned int overMaxCurrentTotalMillis = 0;
    bool isUnderMinCurrent = false;
    unsigned long underMinCurrentStartMillis = millis();
    unsigned int underMinCurrentTotalMillis = 0;
    
    doorStatus currStatus = DOOR_STOPPED;  
    doorStopCause currStopCause = DOOR_STOPPED_INIT;
    doorStatus prevStatus = DOOR_STOPPED;
    doorStopCause prevStopCause = DOOR_STOPPED_INIT;

    int dOpenStopDelayMillis;
    int dClosedStopDelayMillis;
    bool limitSwitchTriggered = false;                    // set to true as soon as the limit switch is triggered
    unsigned long limitSwitchTriggeredMillis;             // the time is milliseconds when the open or closed limit switch is triggered
    unsigned long getRuntimeMillis() {return millis() - startMillis;};
    void updateStatus(doorStatus status, doorStopCause stopCause);
    void debugPrintStopCause(doorStopCause stopCause);
    void debugPrintStatus(doorStatus status);
    bool isDoorFault();
    void setDoorFaultLED();
};
// end Door class prototype

//
// Door() 
//
Door::Door(Actuator* actuatorPtr, byte openLimitSwPin, byte approachingOpenPin, byte closedLimitSwPin, byte approachingClosedPin, int openStopDelayMillis, int closedStopDelayMillis) {
  dActuatorPtr = actuatorPtr;
  dOpenLimitSwPin = openLimitSwPin; 
  dApproachingOpenPin = approachingOpenPin; 
  dClosedLimitSwPin = closedLimitSwPin; 
  dApproachingClosedPin = approachingClosedPin; 
  dOpenStopDelayMillis = openStopDelayMillis;
  dClosedStopDelayMillis = closedStopDelayMillis;
}

//
// init
//
void Door::init() {
  // Door Limit Switches - are wired "Normally Closed" to ground - so the arduino pin is pulled low, when the limit switch is triggered it 
  // opens the circuit and the arduino pin will go high
  dOpenLimitSwPtr = new DebouncedSwitch(dOpenLimitSwPin, HIGH); 
  dApproachingOpenPtr = new DebouncedSwitch(dApproachingOpenPin, HIGH); 
  dClosedLimitSwPtr = new DebouncedSwitch(dClosedLimitSwPin, HIGH); 
  dApproachingClosedPtr = new DebouncedSwitch(dApproachingClosedPin, HIGH); 
  
  if (dOpenLimitSwPtr->triggered(millis())) {
    updateStatus(DOOR_OPEN, DOOR_STOPPED_LIMIT_SW);
  } else if (dClosedLimitSwPtr->triggered(millis())) {
    updateStatus(DOOR_CLOSED, DOOR_STOPPED_LIMIT_SW);
  } else {
    updateStatus(DOOR_STOPPED, DOOR_STOPPED_INIT);
  }
}

//
// open
//
void Door::open() {
  updateStatus(DOOR_OPENING, DOOR_STOPPED_NA);
  startMillis = millis(); // the door is starting to open or close, keep track of the starting time
  isOverMaxCurrent = false;
  overMaxCurrentTotalMillis = 0;
  isUnderMinCurrent = false;
  underMinCurrentTotalMillis = 0;

  doorRampMode = RAMP_UP;
  limitSwitchTriggered = false;
  dActuatorPtr->extend(g_startSpeed);
}

//
// close
//
void Door::close() {
  updateStatus(DOOR_CLOSING, DOOR_STOPPED_NA);
  startMillis = millis(); // the door is starting to open or close, keep track of the starting time
  isOverMaxCurrent = false;
  overMaxCurrentTotalMillis = 0;
  isUnderMinCurrent = false;
  underMinCurrentTotalMillis = 0;

  doorRampMode = RAMP_UP;
  limitSwitchTriggered = false;
  dActuatorPtr->retract(g_startSpeed);
}

//
// stop
//
void Door::stop(doorStatus status, doorStopCause stopCause) {
  updateStatus(status, stopCause);
  dActuatorPtr->stop();
  // turn off door moving LED
  doorRampMode = RAMP_NONE;
  if (isDoorFault()) {
    setDoorFaultLED();
  }
}

//
// updateStatus
//
void Door::updateStatus(doorStatus newStatus, doorStopCause newStopCause) {
  prevStatus = currStatus;
  prevStopCause = currStopCause;
  currStatus = newStatus;
  currStopCause = newStopCause;
    
  DEBUG_PRINT(" door prevStatus = ");
  debugPrintStatus(prevStatus);
  DEBUG_PRINT(" door prevStopCause = ");
  debugPrintStopCause(prevStopCause);
  DEBUG_PRINTLN();

  DEBUG_PRINT(" door currStatus = ");
  debugPrintStatus(currStatus);
  DEBUG_PRINT(" door currStopCause = ");
  debugPrintStopCause(currStopCause);
  DEBUG_PRINTLN();
}

//
// processEvents
//
void Door::processEvents() {
  if (currStatus == DOOR_OPENING || currStatus == DOOR_CLOSING) {
    unsigned int actuatorCurrent = dActuatorPtr->getCurrent();
    // Check for over max current
    if (actuatorCurrent > g_maxCurrent) {
      if (isOverMaxCurrent) {
        // the Current has already been over the max at least one time before this check
        overMaxCurrentTotalMillis = millis() - overMaxCurrentStartMillis;
      } else {
        // this is the first occurnace of the Current being over the max
        isOverMaxCurrent = true;
        overMaxCurrentStartMillis = millis();
        overMaxCurrentTotalMillis = 0;
      }
    } else {
      // the Current is below the max, so reset the over max flag and over max total milliseconds
      isOverMaxCurrent = false;
      overMaxCurrentTotalMillis = 0;
      // Check for under the min current
      if (actuatorCurrent < g_minCurrent) {
        if (isUnderMinCurrent) {
          // the Current has already been under the min at least one time before this check 
          underMinCurrentTotalMillis = millis() - underMinCurrentStartMillis;
        } else {
          // this is the first occurnace of the Current being under the min
          isUnderMinCurrent = true;
          underMinCurrentStartMillis = millis();
          underMinCurrentTotalMillis = 0;
        }
      } else {
        // the Current is above the min, so reset the under min flag and under min total milliseconds
        isUnderMinCurrent = false;
        underMinCurrentTotalMillis = 0;
      }
    }
    
    // Check for continious excessive current for more than the max allowed (250 milliseconds)
    if (overMaxCurrentTotalMillis > g_maxMillisAtOverCurrent) {
      DEBUG_PRINT(" overMaxCurrentTotalMillis = ");
      DEBUG_PRINT(overMaxCurrentTotalMillis);
      stop(DOOR_STOPPED, DOOR_STOPPED_ACTUATOR_EXCESS_CURRENT);
      return;
    }
    
    // Check for excessive running time - indicating that the door is jammed
    if (getRuntimeMillis() > g_maxRuntimeMillis) {
      stop(DOOR_STOPPED, DOOR_STOPPED_TIMEOUT);     
      return;
    }

    if (currStatus == DOOR_OPENING) {
      // Check for under the min current which indicates that the actuator internal limit switch has been triggered
      if (underMinCurrentTotalMillis > g_maxMillisAtUnderCurrent) {
        DEBUG_PRINT(" underMinCurrentTotalMillis = ");
        DEBUG_PRINT(underMinCurrentTotalMillis);
        stop(DOOR_OPEN, DOOR_STOPPED_ACTUATOR_ZERO_CURRENT);
        return;
      }
      // Check the door open limit switch
      if (dOpenLimitSwPtr->triggered(millis())) {
        if (!limitSwitchTriggered) {
          limitSwitchTriggered = true;
          limitSwitchTriggeredMillis = millis();
        }
        if (millis() >= limitSwitchTriggeredMillis + dOpenStopDelayMillis) {
          stop(DOOR_OPEN, DOOR_STOPPED_LIMIT_SW);
          return;
        } 
      }
    }

    if (currStatus == DOOR_CLOSING) {
      // Check for under the min current which indicates that the actuator internal limit switch has been triggered
      if (underMinCurrentTotalMillis > g_maxMillisAtUnderCurrent) {
        DEBUG_PRINT(" underMinCurrentTotalMillis = ");
        DEBUG_PRINT(underMinCurrentTotalMillis);
        stop(DOOR_CLOSED, DOOR_STOPPED_ACTUATOR_ZERO_CURRENT);
        return;
      }
      // Check the door closed limit switch
      if (dClosedLimitSwPtr->triggered(millis())) {
        if (!limitSwitchTriggered) {
            limitSwitchTriggered = true;
            limitSwitchTriggeredMillis = millis();
        }
        if (millis() >= limitSwitchTriggeredMillis + dOpenStopDelayMillis) {
          stop(DOOR_CLOSED, DOOR_STOPPED_LIMIT_SW);
          return;
        } 
      }
    }

    // ramp actuator speed up or down
    if (doorRampMode == RAMP_UP) {
      doorRampMode = dActuatorPtr->rampUp();
    } else if(doorRampMode == RAMP_DOWN) {
      doorRampMode = dActuatorPtr->rampDown();
    } else if (millis() - startMillis > g_startRampDownMillis) {
      doorRampMode = RAMP_DOWN;
    }
  }
}

//
// doorFault
//
bool Door::isDoorFault() {
  if (currStatus == DOOR_STOPPED) {
    switch (currStopCause) {
      case DOOR_STOPPED_TIMEOUT:
        return true;
      case DOOR_STOPPED_OBSTRUCTION:
        return true;
      case DOOR_STOPPED_ACTUATOR_EXCESS_CURRENT:
        return true;
    }
  }
  return false;  
}

//
// setDoorFaultLED
//
//
void Door::setDoorFaultLED() {
  if (currStatus == DOOR_STOPPED) {
    switch (currStopCause) {
      case DOOR_STOPPED_TIMEOUT:
        DEBUG_PPRINTLN("DOOR_STOPPED_TIMEOUT");
        digitalWrite(g_timeoutFaultLEDPin, HIGH);
        return;
      case DOOR_STOPPED_OBSTRUCTION:
        DEBUG_PPRINTLN("DOOR_STOPPED_OBSTRUCTION");
        digitalWrite(g_obstructionFaultLEDPin, HIGH);
        return;
      case DOOR_STOPPED_ACTUATOR_EXCESS_CURRENT:
        DEBUG_PPRINTLN("DOOR_STOPPED_ACTUATOR_EXCESS_CURRENT");
        digitalWrite(g_excessCurrentFaultLEDPin, HIGH);
        return;
    }
  }
}

//
// utility debug function to print the doorStopCause as a stinng
//
void Door::debugPrintStopCause(doorStopCause stopCause) {
  switch (stopCause) {
    case DOOR_STOPPED_LIMIT_SW:
      DEBUG_PRINT("DOOR_STOPPED_LIMIT_SW");
      break;
    case DOOR_STOPPED_TIMEOUT:
      DEBUG_PRINT("DOOR_STOPPED_TIMEOUT");
      break;
    case DOOR_STOPPED_OBSTRUCTION:
      DEBUG_PRINT("DOOR_STOPPED_OBSTRUCTION");
      break;
    case DOOR_STOPPED_BUTTON_WHILE_OPENING:
      DEBUG_PRINT("DOOR_STOPPED_BUTTON_WHILE_OPENING");
      break;
    case DOOR_STOPPED_BUTTON_WHILE_CLOSING:
      DEBUG_PRINT("DOOR_STOPPED_BUTTON_WHILE_CLOSING");
      break;
    case DOOR_STOPPED_INIT:
      DEBUG_PRINT("DOOR_STOPPED_INIT");
      break;
    case DOOR_STOPPED_ACTUATOR_ZERO_CURRENT:
      DEBUG_PRINT("DOOR_STOPPED_ACTUATOR_ZERO_CURRENT");
      break;
    case DOOR_STOPPED_ACTUATOR_EXCESS_CURRENT:
      DEBUG_PRINT("DOOR_STOPPED_ACTUATOR_EXCESS_CURRENT");
      break;
    case DOOR_STOPPED_NA:
      DEBUG_PRINT("DOOR_STOPPED_NA");
      break;
    default:
      DEBUG_PRINT("*** UNKOWN DOOR_STOP code *** (");
      DEBUG_PRINT(stopCause);
      DEBUG_PRINT(")");
  }
}

//
// utility debug function to print the doorStatus as a strinng
//
void Door::debugPrintStatus(doorStatus status) {
  switch (status) {
    case DOOR_OPEN:
      DEBUG_PRINT("DOOR_OPEN");
      break;
    case DOOR_OPENING:
      DEBUG_PRINT("DOOR_OPENING");
      break;
    case DOOR_CLOSED:
      DEBUG_PRINT("DOOR_CLOSED");
      break;
    case DOOR_CLOSING:
      DEBUG_PRINT("DOOR_CLOSING");
      break;
    case DOOR_STOPPED:
      DEBUG_PRINT("DOOR_STOPPED");
      break;
    default:
      DEBUG_PRINT("*** UNKOWN STATUS *** (");
      DEBUG_PRINT(status);
      DEBUG_PRINT(")");
      break;
  }
}

//
// end class Door
//

//
// DoorPair class 
//
class DoorPair {
  public:
    DoorPair(Door* door1Ptr, Door* door2Ptr);
    void init();
    void open();
    void close();
    void stop(doorStatus status, doorStopCause stopCause);
    void processEvents();
    doorStatus getCurrStatus();
    doorStatus getPrevStatus();
    doorStatus getD1PrevStatus();
    doorStatus getD2PrevStatus();
  private:    
    Door* door1Ptr;
    Door* door2Ptr;
    bool reverseProcessed = false; // used when the door motion needs to be reversed briefly after a fault
    void debugPrintStopCause(doorStopCause stopCause);
    void debugPrintStatus(doorStatus status);
    void clearFaultLEDs();
    void setDoorMovingLEDMode(doorMovingLEDMode mode);
};
// end DoorPair class prototype

//
// DoorPair
//
DoorPair::DoorPair(Door* d1Ptr, Door* d2Ptr) {
  door1Ptr = d1Ptr;
  door2Ptr = d2Ptr;
}

//
// init
//
void DoorPair::init() {
  door1Ptr->init();
  door2Ptr->init();
  setDoorMovingLEDMode(DM_OFF);
} 

//
// open
//
void DoorPair::open() {
  clearFaultLEDs();
  setDoorMovingLEDMode(DM_ON);
  reverseProcessed = false;
  door1Ptr->open();  
  door2Ptr->open();
}

//
// close
//
void DoorPair::close() {
  clearFaultLEDs();
  digitalWrite(g_onOffObstructionSensorPin, HIGH); // turn on the obstruction sensor
  DEBUG_PRINTLN("g_onOffObstructionSensorPin - 1, HIGH");

  setDoorMovingLEDMode(DM_ON);
  reverseProcessed = false;
  door1Ptr->close();  
  door2Ptr->close();
}

//
// stop
//
void DoorPair::stop(doorStatus status, doorStopCause stopCause) {
  setDoorMovingLEDMode(DM_OFF);
  door1Ptr->stop(status, stopCause);
  door2Ptr->stop(status, stopCause);
  digitalWrite(g_onOffObstructionSensorPin, LOW); // turn off the obstruction sensor
  DEBUG_PRINTLN("g_onOffObstructionSensorPin - 2, LOW");
}

//
// getCurrStatus
//
// determine the logical status of the door pair based the the status of each door
//  rule 1: door pair is not open or closed until both doors are open or closed
//  rule 2: door pair is opening or closing if either of the doors are opening or closing
//  rule 3: if one door is stopped, the other door should be stopped
//
doorStatus DoorPair::getCurrStatus() {
  doorStatus door1Status = door1Ptr->getCurrStatus();
  doorStatus door2Status = door2Ptr->getCurrStatus();
  doorStatus status = DOOR_STOPPED;

  if (door1Status == DOOR_OPEN && door2Status == DOOR_OPEN) {
     status = DOOR_OPEN;
  } else if (door1Status == DOOR_CLOSED && door2Status == DOOR_CLOSED) {
     status = DOOR_CLOSED;
  } else if (door1Status == DOOR_OPENING || door2Status == DOOR_OPENING) {
     status = DOOR_OPENING;
  } else if (door1Status == DOOR_CLOSING || door2Status == DOOR_CLOSING) {
     status = DOOR_CLOSING;
  } else if (door1Status == DOOR_STOPPED || door2Status == DOOR_STOPPED) {
     status = DOOR_STOPPED;
  } 
  if (status == DOOR_STOPPED) {
    if (door1Status != DOOR_STOPPED) {
      // error - this should never happed
      door1Ptr->stop(DOOR_STOPPED, door2Ptr->getCurrStopCause());
    }
    if (door2Status != DOOR_STOPPED) {
      // error - this should never happed
      door2Ptr->stop(DOOR_STOPPED, door1Ptr->getCurrStopCause());
    }
  }
  return status;
}

//
// getPrevStatus
//
// return the previous status of the door pair - 
// use door1 as the "master" previous status - (return the previous status of door 1 for the door pair prevoous status)
//
doorStatus DoorPair::getPrevStatus() {
  doorStatus door1PrevStatus = door1Ptr->getPrevStatus();
  return door1PrevStatus;
}

//
// getD1PrevStatus
//
// return the previous status of the door 
//
doorStatus DoorPair::getD1PrevStatus() {
  return door1Ptr->getPrevStatus();
}

//
// getD2PrevStatus
//
// return the previous status of the door 
//
doorStatus DoorPair::getD2PrevStatus() {
  return door2Ptr->getPrevStatus();
}

//
// processEvents
//
void DoorPair::processEvents() {
  door1Ptr->processEvents();
  door2Ptr->processEvents();

  doorStatus d1CurrStatus = door1Ptr->getCurrStatus();
  doorStatus d2CurrStatus = door2Ptr->getCurrStatus();

  bool doorsAreStopped = false;
  if (d1CurrStatus == DOOR_STOPPED) {
    if (d2CurrStatus != DOOR_STOPPED) {
      door2Ptr->stop(DOOR_STOPPED, door1Ptr->getCurrStopCause()); // use the door 1 stop cause for the door 2 stop cause
    } 
    doorsAreStopped = true;
  } else if (d2CurrStatus == DOOR_STOPPED) {
    door1Ptr->stop(DOOR_STOPPED, door2Ptr->getCurrStopCause()); // use the door 2 stop cause for the door 1 stop cause
    doorsAreStopped = true;
  }
  if (doorsAreStopped) {
    digitalWrite(g_onOffObstructionSensorPin, LOW); // turn off the obstruction sensor
    DEBUG_PRINTLN("g_onOffObstructionSensorPin - 3, LOW");
    setDoorMovingLEDMode(DM_OFF);
    if (door1Ptr->getCurrStopCause() == DOOR_STOPPED_ACTUATOR_EXCESS_CURRENT || door2Ptr->getCurrStopCause() == DOOR_STOPPED_ACTUATOR_EXCESS_CURRENT) {
      // one or both doors are drawing excessive current - indicating a jam
      if (!reverseProcessed) {
        // the doors are jammed, reverse the doors for a fraction of a second
        // don't update the door status when doing the reversing
        if (getPrevStatus() == DOOR_OPENING) {
          door1Ptr->dActuatorPtr->retract();
          door2Ptr->dActuatorPtr->retract();
          delay(250);
          door1Ptr->dActuatorPtr->stop();        
          door2Ptr->dActuatorPtr->stop();
          reverseProcessed = true;
        } else if (getPrevStatus() == DOOR_CLOSING) {
          door1Ptr->dActuatorPtr->extend();
          door2Ptr->dActuatorPtr->extend();
          delay(250);
          door1Ptr->dActuatorPtr->stop();
          door2Ptr->dActuatorPtr->stop();
          reverseProcessed = true;
        }
      }
    }
  } else if ((d1CurrStatus == DOOR_OPEN && d2CurrStatus == DOOR_OPEN) || (d1CurrStatus == DOOR_CLOSED && d2CurrStatus == DOOR_CLOSED)) {
    setDoorMovingLEDMode(DM_OFF);
  }
}

//
// clearFaultLEDs
//
void DoorPair:: clearFaultLEDs() {
  digitalWrite(g_obstructionFaultLEDPin, LOW);
  digitalWrite(g_excessCurrentFaultLEDPin, LOW);
  digitalWrite(g_timeoutFaultLEDPin, LOW);
}

// 
// setDoorMovingLEDMode()
//
void DoorPair::setDoorMovingLEDMode(doorMovingLEDMode mode) {
  if (mode == DM_OFF) {
    digitalWrite(g_doorMovingLEDPin, LOW);
  } else if (mode == DM_ON) {
    digitalWrite(g_doorMovingLEDPin, HIGH);
  }
}


//
// end class DoorPair
//

///////////////////////////////////////////////////////////////////////////////////////////////
// end Class Definitions
///////////////////////////////////////////////////////////////////////////////////////////////

// door 1
Actuator* actuator1Ptr;
Door* door1Ptr;
// door 2
Actuator* actuator2Ptr;
Door* door2Ptr;
// door pair
DoorPair* doorPairPtr;
// button and obstruction sensor
DebouncedSwitch* buttonPtr; 
DebouncedSwitch* obstructionSensorPtr; 

// setup
void setup()
{
  Serial.begin(115200);
  DEBUG_PRINTLN("Garage Door Opener");

  // pins for actuator driver board 
  pinMode(g_m1FBPin, INPUT);
  pinMode(g_m1SFPin, INPUT);
  pinMode(g_m1PWMD2Pin, OUTPUT);
  pinMode(g_m1In1Pin, OUTPUT);
  pinMode(g_m1In2Pin, OUTPUT);
  
  pinMode(g_m2FBPin, INPUT);
  pinMode(g_m2SFPin, INPUT);
  pinMode(g_m2PWMD2Pin, OUTPUT);
  pinMode(g_m2In1Pin, OUTPUT);
  pinMode(g_m2In2Pin, OUTPUT);

  pinMode(g_enPin, OUTPUT);

  // button and obstruction sensor pins
  pinMode(g_buttonPin, INPUT); // this pin needs a pull down resistor
  pinMode(g_obstructionSensorPin, INPUT_PULLUP);
  pinMode(g_onOffObstructionSensorPin, OUTPUT);
  digitalWrite(g_onOffObstructionSensorPin, LOW); // turn off the obstruction sensor
  DEBUG_PRINTLN("g_onOffObstructionSensorPin - 0, LOW");

  // limit switch and approaching limit pins
  // door 1
  pinMode(g_d1OpenLimitSwPin, INPUT_PULLUP);
  pinMode(g_d1ApproachingOpenPin, INPUT_PULLUP);
  pinMode(g_d1ClosedLimitSwPin, INPUT_PULLUP);
  pinMode(g_d1ApproachingClosedPin, INPUT_PULLUP);
  // door 2
  pinMode(g_d2OpenLimitSwPin, INPUT_PULLUP);
  pinMode(g_d2ApproachingOpenPin, INPUT_PULLUP);
  pinMode(g_d2ClosedLimitSwPin, INPUT_PULLUP);
  pinMode(g_d2ApproachingClosedPin, INPUT_PULLUP);

  // LEDs
  pinMode(g_doorMovingLEDPin, OUTPUT);
  pinMode(g_obstructionFaultLEDPin, OUTPUT);
  pinMode(g_excessCurrentFaultLEDPin, OUTPUT);
  pinMode(g_timeoutFaultLEDPin, OUTPUT);
  
  // create door 1 object
  actuator1Ptr = new Actuator(g_m1FBPin, g_m1SFPin, g_m1PWMD2Pin, g_m1In1Pin, g_m1In2Pin, g_enPin);
  door1Ptr = new Door(actuator1Ptr, g_d1OpenLimitSwPin, g_d1ApproachingOpenPin, g_d1ClosedLimitSwPin, g_d1ApproachingClosedPin, g_d1OpenStopDelayMillis, g_d1ClosedStopDelayMillis);
  
  // create door 2 object
  actuator2Ptr = new Actuator(g_m2FBPin, g_m2SFPin, g_m2PWMD2Pin, g_m2In1Pin, g_m2In2Pin, g_enPin);
  door2Ptr = new Door(actuator2Ptr, g_d2OpenLimitSwPin, g_d2ApproachingOpenPin, g_d2ClosedLimitSwPin, g_d2ApproachingClosedPin, g_d1OpenStopDelayMillis, g_d1ClosedStopDelayMillis);

  // create and initialize door pair object
  doorPairPtr = new DoorPair(door1Ptr, door2Ptr);
  doorPairPtr->init();

  // Button - the button pin is held low with a pull down resistor, when the button is pressed it completes a circuit and sends 5 volts to the arduino pin
  buttonPtr = new DebouncedSwitch(g_buttonPin, HIGH); 
  // IR Sensor / Obstruction Sensor - wired "Normally Closed" to ground - so the arduino pin is pulled low, when the IR sensor relay is triggered it 
  // opens the circuit and the arduino pin will go high
  obstructionSensorPtr = new DebouncedSwitch(g_obstructionSensorPin, HIGH); 

  //Flash the LEDs to indicate that the controller is ready
  for (int i=0; i < 3; i++) {
    digitalWrite(g_doorMovingLEDPin, HIGH);
    digitalWrite(g_obstructionFaultLEDPin, HIGH);
    digitalWrite(g_timeoutFaultLEDPin, HIGH);
    digitalWrite(g_excessCurrentFaultLEDPin, HIGH);
    delay(250);
    digitalWrite(g_doorMovingLEDPin, LOW);
    digitalWrite(g_obstructionFaultLEDPin, LOW);
    digitalWrite(g_timeoutFaultLEDPin, LOW);
    digitalWrite(g_excessCurrentFaultLEDPin, LOW);
    delay(250);
  }
}

// Main loop
bool buttonPressProcessed = false;
void loop()
{
  if (obstructionSensorPtr->triggered(millis()) && doorPairPtr->getCurrStatus() == DOOR_CLOSING) { 
    // only react to the obstruction sensor if the door is closing
    doorPairPtr->stop(DOOR_STOPPED, DOOR_STOPPED_OBSTRUCTION);
  } else { 
    if (buttonPtr->triggered(millis())) {
      // only respond to the button press once per button press, buttonPressProcessed will get reset to false when the button is released
      if (!buttonPressProcessed) {
        buttonPressProcessed = true;
        DEBUG_PRINTLN("button pushed - and not processed");
        doorStatus currStatus = doorPairPtr->getCurrStatus();
        DEBUG_PPRINT("doorPairPtr->getCurrStatus() = ");
        DEBUG_PRINTLN(currStatus);       
        if (currStatus == DOOR_OPEN) {
          DEBUG_PRINTLN("door is DOOR_OPEN");
          // start closing door 
          doorPairPtr->close();
        } else if (currStatus == DOOR_CLOSED) {
          DEBUG_PRINTLN("door is DOOR_CLOSED");
          // start opening door 
          doorPairPtr->open();
        } else if (currStatus == DOOR_OPENING) {
          DEBUG_PRINTLN("door is DOOR_OPENING");
          // stop doors 
          doorPairPtr->stop(DOOR_STOPPED, DOOR_STOPPED_BUTTON_WHILE_OPENING);
        } else if (currStatus == DOOR_CLOSING) {
          DEBUG_PRINTLN("door is DOOR_CLOSING");
          // stop doors
          doorPairPtr->stop(DOOR_STOPPED, DOOR_STOPPED_BUTTON_WHILE_CLOSING);
        } else if (currStatus == DOOR_STOPPED) {
          DEBUG_PRINTLN("door is DOOR_STOPPED");
          // if previous status was opening, then start closing the door, otherwise start opening the door 
          if (doorPairPtr->getPrevStatus() == DOOR_OPENING) {
            doorPairPtr->close();
          } else {
            if (doorPairPtr->getD1PrevStatus() == DOOR_OPENING || doorPairPtr->getD2PrevStatus() == DOOR_OPENING) {
              doorPairPtr->close();
            } else if (doorPairPtr->getD1PrevStatus() == DOOR_CLOSING || doorPairPtr->getD2PrevStatus() == DOOR_CLOSING) {
              doorPairPtr->open();
            } else {
              doorPairPtr->open();
            }
          }
        } 
      }
    } else {
      buttonPressProcessed = false;
    }
  }   
  delay(10);
  // check for limit switches, timeout, excess current, beam break and other events that need to be processed 
  doorPairPtr->processEvents(); 
}
