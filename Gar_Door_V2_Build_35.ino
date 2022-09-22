// Gar_Door_V2
//
// Created Dec 1, 2020
// Created by David Barck
//
// 09/18/2022 cleaned up some variable names,to moved functions from init() to constructors of Door and DoorOpening
// 09/15/2022 Removed pause and resume - the issue was resolved by moving the base of the east (door 1) actuator, reduced timeout from 15 to 13 seconds
// 09/14/2022 Added new stop cause: DOOR_AUTO_PAUSE_WHILE_CLOSING, added door::resumeClose
// 09/14/2022 Added pause and resume when closing - this fixes the issue of the east door not closing fully (don't know why - it just works)
// 08/29/2022 Added separate max current for opening and closing
// 08/15/2022 Added delay to allow for obstruction startup, added more debugging, coments, changed names of some objects
// 05/10/2021 Removed interrupt driven timer for blinking the "operate" button - this was just adding unnecessary complication 
//
// Garage Door Opener/Closer Controller
//
// Micro Controller board:
//   Arduino Mega
// Motor driver board:
//   Cytron 10A (?) 5-30V (?) Dual Channel DC Motor Driver, purchsed from robotshop.com
// Linear actuators:
//    Metal gear electric Linear actuator 12V, 700mm distance stroke, 90mm per second, 2.5A max, purchsed from Aliexpress
//   
// Photoelectric Obstruction sensor  
//    TOPENS TC102 Photo Eye Infrared Photocell Beam Sensor for Gate Openers (purchsed from Banggood? - also avail on Amazon)
//
// Class Diagram:
//    
//    DoorOpening 
//      ObstructionSensor
//      Door1
//        Actuator1
//      Door2
//        Actuator2
//
//
// Open and Close the bifold garage doors using a linear actuator attached to each door. Extending the actuator opens the door, retracting closes the door.
// The open or close operation is triggered via a momentary button, (a wireless remote control relay and a wifi controlled relay arre also wired to the button)
// Each door has open and closed limit switchs to indicate when the door is fully open, or fully closed.
// (each actuator also has internal limit switchs that stop the actuator if it becomes fully extended or fully retracted)
// There is an IR "beam break" sensor to detect if there is an obstruction in the door opening. The sensor runs at 12 volts and is switched on and off via a relay and arduino pin
// 
// While each door is opening or closing, check the appropriate limit switch . Stop the door when the limit switch is triggered.
// Stop BOTH doors if:
//    - the button (or remote button) is pressed during door operation
//    - the IR obstruction sensor is triggered and the door is closing (no need to check for obstruction when the door is opening)
//    - excess current draw is detected (indicating that one or both doors are jammed)
//    - very low current is detected (indicating that the actuator has reached the limit of travel - this should never happen)
//    - the open or closing operation has been running for excess time
//
#define DEBUG
#include <serialdebug.h>
#include <debouncedswitch.h>

// enum types
enum actuatorStatus {ACTUATOR_EXTENDING, ACTUATOR_RETRACTING, ACTUATOR_STOPPED}; // EXTENDING - opens the door, RETRACTING - closes the door
enum doorStatus {DOOR_OPEN, DOOR_OPENING, DOOR_CLOSED, DOOR_CLOSING, DOOR_STOPPED};
enum doorStopCause {DOOR_STOPPED_LIMIT_SW, DOOR_STOPPED_TIMEOUT, DOOR_STOPPED_OBSTRUCTION, DOOR_STOPPED_BUTTON_WHILE_OPENING, 
  DOOR_STOPPED_BUTTON_WHILE_CLOSING, DOOR_STOPPED_INIT, DOOR_STOPPED_ACTUATOR_ZERO_CURRENT, DOOR_STOPPED_ACTUATOR_EXCESS_CURRENT, 
  DOOR_STOPPED_REVERSED_ACTUATOR_EXCESS_CURRENT, DOOR_STOPPED_NA};
enum doorMovingLEDMode {DM_ON, DM_OFF};
enum rampMode {RAMP_UP, RAMP_DOWN, RAMP_NONE};
enum doorID {DOOR1, DOOR2};

// Global constants 
// Max and Min current
const int gc_maxCurrentOpening = 1800;          // if current exceeds this value for an extended time, then stop 
const int gc_maxCurrentClosing = 3000;          // if current exceeds this value for an extended time, then stop 
const int gc_minCurrent = 200;                  // if current falls below this value for an extended time, then consider the door open or closed due to the actuator internal limit switch hit     
const int gc_maxMillisAtOverCurrent = 250;      // max time the current can exceed the max current
const int gc_maxMillisAtUnderCurrent = 250;     // max time the current be under the min current before the door is considered open or closed
// Other global constants
const unsigned long gc_maxRuntimeMillis = 13000; // max run time before timing out
const int gc_startRampDownMillis = 13000;       // elapsed opening or closing time before starting to ramp the actuator speed down;
const int gc_rampSpeedChange = 1;               // number of steps to increment or decrement the actuator speed when ramping up or ramping down
const int gc_startSpeed = 150;                  // initial actuator speed when opening or closing (0 to 255)
const int gc_d1OpenStopDelayMillis = 0;         // milliseconds to delay after the open limit switch is triggered before stopping the actuator
const int gc_d1ClosedStopDelayMillis = 0;       // milliseconds to delay after the closed limit switch is triggered before stopping the actuator
const int gc_d2OpenStopDelayMillis = 0;         // milliseconds to delay after the open limit switch is triggered before stopping the actuator
const int gc_d2ClosedStopDelayMillis = 0;       // milliseconds to delay after the closed limit switch is triggered before stopping the actuator

// Global Pin assignments
// actuator dirver board pins //////////////////////////
// actuator 1 pins
const byte gc_m1FBPin = A2;    // current feedback input
const byte gc_m1SFPin = 47;    // status flag input
const byte gc_m1PWMD2Pin = 11; // PWM / Disable 2 - speed control
const byte gc_m1In1Pin = 44;   // logic control of actuator OUT 1
const byte gc_m1In2Pin = 45;   // logic control of actuator OUT 2
// actuator 2 pins
const byte gc_m2FBPin = A10;    // same as for actuator 1    
const byte gc_m2SFPin = 50;      
const byte gc_m2PWMD2Pin = 12;  
const byte gc_m2In1Pin = 48;   
const byte gc_m2In2Pin = 49;   
// this pin applies to both actuators
const byte gc_enPin = 39;      // Enable input: when EN is LOW, then both actuator driver ICs are in a low-current sleep mode.
// end actuator driver board pins

// open-close-stop button pin //////////////////////////
const byte gc_buttonPin = 33; // momentary button to indicate that both doors should open, close, or stop, triggered when signal goes HIGH (the wireless remote and WiFi relay are also wired to this pin)

// obstruciton sensor pins    //////////////////////////
const byte gc_obstructionSensorDetectPin = 35;               // obstruction detected when signal goes low
const byte gc_ObstructionSensorPowerPin = 52;                // turns obstruction sensor on or off (on = high, off = low)
const unsigned long gc_obstructionSensorStartupMillis = 250; // the time it takes for the obstruction sensor to startup

// limit switches and approach sensor pins //////////////////////////
// door 1
const byte gc_d1OpenLimitSwPin = 22;       // door 1 is open when signal goes LOW
const byte gc_d1ClosedLimitSwPin = 23;     // door 1 is closed when signal goes LOW
const byte gc_d1ApproachingOpenPin = 24;   // FUTURE USE: door is approaching open when signal goes momentarily LOW 
const byte gc_d1ApproachingClosedPin = 25; // FUTURE USE: door is approaching closed when signal goes momentarily LOW 
// door 2
const byte gc_d2OpenLimitSwPin = 28;       // door 2 is open when signal goes LOW
const byte gc_d2ClosedLimitSwPin = 29;     // door 2 is closed when signal goes LOW
const byte gc_d2ApproachingOpenPin = 30;   // FUTURE USE: door is approaching open when signal goes momentarily LOW 
const byte gc_d2ApproachingClosedPin = 31; // FUTURE USE: door is approaching closed when signal goes momentarily LOW 

// LEDs
const byte gc_excessCurrentFaultLEDPin = 3 ;  // light if doorStopCause is DOOR_STOPPED_ACTUATOR_EXCESS_CURRENT (due to one or both doors are jammed)
const byte gc_doorMovingLEDPin = 4;           // light when door is moving
const byte gc_timeoutFaultLEDPin =  5;        // light if doorStopCause is DOOR_STOPPED_TIMEOUT
const byte gc_obstructionFaultLEDPin =  6;    // light if doorStopCause is DOOR_STOPPED_OBSTRUCTION (IR "beam break" sensor is triggered)
 
// end Global pin assignments

///////////////////////////////////////////////////////////////////////////////////////////////
// Class Definitions
///////////////////////////////////////////////////////////////////////////////////////////////

//
// Obstruction Sensor class 
//
// The obstruction sensor is a photo electric IR sensor wired "Normally Closed" to ground - so the arduino pin is pulled low
// when the IR sensor relay is triggered it opens the circuit and the arduino pin will go high
//
class ObstructionSensor {
  public:
    ObstructionSensor();
    void turnOn();
    void turnOff();
    bool getIsOn() {return mIsOn;};
    bool isReady();
    bool obstructionDetected();
  private:
    unsigned long mObstSensStartTime = 0;
    bool mIsOn = false;
    DebouncedSwitch* mpObstSensSwitch; 
};
// end ObstructionSensor class definition

//
// ObstructionSensor class implimentation
//

//
// ObstructionSensor
//
ObstructionSensor::ObstructionSensor() {
  mpObstSensSwitch = new DebouncedSwitch(gc_obstructionSensorDetectPin, HIGH); 
}

//
// turnOn
//
void ObstructionSensor::turnOn() {
  digitalWrite(gc_ObstructionSensorPowerPin, HIGH); // turn on the obstruction sensor
  mIsOn = true;
  mObstSensStartTime = millis();
  DEBUG_PRINTLN("obstruction sensor is on");
}

//
// turnOff
//
void ObstructionSensor::turnOff() {
  digitalWrite(gc_ObstructionSensorPowerPin, LOW); // turn off the obstruction sensor
  mIsOn = false;
  DEBUG_PRINTLN("obstruction sensor is off");
}

//
// isReady
//
bool ObstructionSensor::isReady() {
  if (mIsOn) {
    if ((millis() - mObstSensStartTime) > gc_obstructionSensorStartupMillis) {
      return true;
    }
  }
  return false;
}

//
// obstructionDetected
//
bool ObstructionSensor::obstructionDetected() {
  if (isReady()) {
    return mpObstSensSwitch->triggered(millis());
  } else {
    return false;
  }
}

//
// End ObstructionSensor class implimentation
//


//
// Actuator class 
// Each door has one electricly driven linear actuator (extending opens the door, retracting closes the door)
//
class Actuator {
  public:
    Actuator(byte fBPin, byte sFPin, byte pWMD2Pin, byte in1Pin, byte in2Pin, byte enPin);
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
    unsigned int getMaxSpeed() {return mMaxSpeed;};
  private:
    // arduino pins
    byte mFBPin;    
    byte mSFPin;    
    byte mPWMD2Pin;  
    byte mIn1Pin;   
    byte mIn2Pin;       
    byte mEnPin;  // this pin applys to both actuators, the same pin is used for both actuators

    actuatorStatus mCurrActuatorStatus = ACTUATOR_STOPPED;
    const unsigned int mMaxSpeed = 255;
    unsigned int mCurrSpeed = 0;
};
// end Actuator class definition

//
// Actuator class implimentation
//
Actuator::Actuator(byte fBPin, byte sFPin, byte pWMD2Pin, byte in1Pin, byte in2Pin, byte enPin) {
  mFBPin = fBPin;
  mSFPin = sFPin;
  mPWMD2Pin = pWMD2Pin;
  mIn1Pin = in1Pin;
  mIn2Pin = in2Pin;
  mEnPin = enPin;
  digitalWrite(mEnPin, HIGH);    // enable input
}

//
// setSpeed
//
void Actuator::setSpeed(int speed) {
  if (speed > mMaxSpeed) {  // Max PWM dutycycle
    speed = mMaxSpeed;
  } else if (speed < 0) {
    speed = 0;
  }
  analogWrite(mPWMD2Pin, speed);  
  mCurrSpeed = speed;
}

//
// extend
// extend the linear actuator
//
void Actuator::extend(){
  DEBUG_PRINTLN();
  extend(mMaxSpeed);
}

//
// extend
// extend the linear actuator
//
void Actuator::extend(unsigned int speed){
  setSpeed(speed);
  digitalWrite(mIn1Pin,HIGH);
  digitalWrite(mIn2Pin,LOW);
  mCurrActuatorStatus = ACTUATOR_EXTENDING;
}

//
// retract
// retrct the linear actuator
//
void Actuator::retract(){
  DEBUG_PRINTLN();
  retract(mMaxSpeed);
}

//
// retract
// retrct the linear actuator
//
void Actuator::retract(unsigned int speed){
  setSpeed(speed);
  digitalWrite(mIn1Pin,LOW);
  digitalWrite(mIn2Pin,HIGH);
  mCurrActuatorStatus = ACTUATOR_RETRACTING;
}

//
// stop
//
void Actuator::stop(){
  setSpeed(0);
  digitalWrite(mIn1Pin,LOW);
  digitalWrite(mIn2Pin,LOW);
  mCurrActuatorStatus = ACTUATOR_STOPPED;
}

//
// rampUp
//
rampMode Actuator::rampUp() {
  if (mCurrSpeed < mMaxSpeed) {
    setSpeed(mCurrSpeed + gc_rampSpeedChange);
    return RAMP_UP;
  } else {
    return RAMP_NONE;
  }
}

//
// rampDown
//
rampMode Actuator::rampDown() {
  if (mCurrSpeed > 0) {
    setSpeed(mCurrSpeed - gc_rampSpeedChange);
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
// this class defines one of the bifold doors - there are two bifold doors in the door opening
//
class Door {
  public:
    Door(doorID ID, byte openLimitSwPin, byte approachingOpenPin, byte closedLimitSwPin, byte approachingClosedPin, int openStopDelayMillis, int closedStopDelayMillis, 
      byte fBPin, byte sFPin, byte pWMD2Pin, byte in1Pin, byte in2Pin, byte enPin);
    void open();
    void close();
    void stop(doorStatus status, doorStopCause stopCause);    
    void processEvents();
    doorStatus getPrevStatus(){return mPrevStatus;};
    doorStatus getCurrStatus(){return mCurrStatus;};
    doorStopCause getCurrStopCause() {return mCurrStopCause;};
    Actuator* get_pActuator() {return mpActuator;};
  private:
    Actuator* mpActuator;
    byte mOpenLimitSwPin;
    byte mApproachingOpenPin;
    byte mClosedLimitSwPin;
    byte mApproachingClosedPin;
    DebouncedSwitch* mpOpenLimitSw; 
    DebouncedSwitch* mpApproachingOpen; 
    DebouncedSwitch* mpClosedLimitSw; 
    DebouncedSwitch* mpApproachingClosed;
    rampMode mDoorRampMode = RAMP_NONE;
    doorID mDoorID;
    unsigned long mStartMillis = 0;
    bool mIsOverMaxCurrent = false;
    unsigned long mOverMaxCurrentStartMillis = millis();
    unsigned int mOverMaxCurrentTotalMillis = 0;
    bool mIsUnderMinCurrent = false;
    unsigned long mUnderMinCurrentStartMillis = millis();
    unsigned int mUnderMinCurrentTotalMillis = 0;
    unsigned long getRuntimeMillis() {return millis() - mStartMillis;};
    doorStatus mCurrStatus = DOOR_STOPPED;  
    doorStopCause mCurrStopCause = DOOR_STOPPED_INIT;
    doorStatus mPrevStatus = DOOR_STOPPED;
    doorStopCause mPrevStopCause = DOOR_STOPPED_INIT;

    int mOpenStopDelayMillis;
    int mClosedStopDelayMillis;
    bool mLimitSwitchTriggered = false;                    // set to true as soon as the limit switch is triggered
    unsigned long mLimitSwitchTriggeredMillis;             // the time is milliseconds when the open or closed limit switch is triggered

    void updateStatus(doorStatus status, doorStopCause stopCause);
    void debugPrintStopCause(doorStopCause stopCause);
    void debugPrintStatus(doorStatus status);
    void debugPrintDoorDescrip(doorID ID);
    void debugPrintCommaDelimitedInfo(unsigned int actuatorCurrent);

    bool isDoorFault();
    void setDoorFaultLED();
};
// end Door class definition

//
// Door() 
//
Door::Door(doorID ID, byte openLimitSwPin, byte approachingOpenPin, byte closedLimitSwPin, byte approachingClosedPin, int openStopDelayMillis, int closedStopDelayMillis, 
  byte fBPin, byte sFPin, byte pWMD2Pin, byte in1Pin, byte in2Pin, byte enPin) {
  mDoorID = ID;
  mOpenLimitSwPin = openLimitSwPin; 
  mApproachingOpenPin = approachingOpenPin; 
  mClosedLimitSwPin = closedLimitSwPin; 
  mApproachingClosedPin = approachingClosedPin; 
  mOpenStopDelayMillis = openStopDelayMillis;
  mClosedStopDelayMillis = closedStopDelayMillis;

  // Door Limit Switches - are wired "Normally Closed" to ground - so the arduino pin is pulled low, when the limit switch is triggered it 
  // opens the circuit and the arduino pin will go high
  mpOpenLimitSw = new DebouncedSwitch(mOpenLimitSwPin, HIGH); 
  mpApproachingOpen = new DebouncedSwitch(mApproachingOpenPin, HIGH); 
  mpClosedLimitSw = new DebouncedSwitch(mClosedLimitSwPin, HIGH); 
  mpApproachingClosed = new DebouncedSwitch(mApproachingClosedPin, HIGH); 
  
  if (mpOpenLimitSw->triggered(millis())) {
    updateStatus(DOOR_OPEN, DOOR_STOPPED_LIMIT_SW);
  } else if (mpClosedLimitSw->triggered(millis())) {
    updateStatus(DOOR_CLOSED, DOOR_STOPPED_LIMIT_SW);
  } else {
    updateStatus(DOOR_STOPPED, DOOR_STOPPED_INIT);
  }  

  // each door has an actuator that opens and closes the door
  mpActuator = new Actuator(fBPin, sFPin, pWMD2Pin, in1Pin, in2Pin, enPin);
}

//
// open - called once to start opening the door
//
void Door::open() {
  updateStatus(DOOR_OPENING, DOOR_STOPPED_NA);
  mStartMillis = millis(); // the door is starting to open or close, keep track of the starting time
  mIsOverMaxCurrent = false;
  mOverMaxCurrentTotalMillis = 0;
  mIsUnderMinCurrent = false;
  mUnderMinCurrentTotalMillis = 0;

  mDoorRampMode = RAMP_UP;
  mLimitSwitchTriggered = false;
  mpActuator->extend(gc_startSpeed);
}

//
// close - called once to start closing the door
//
void Door::close() {
  updateStatus(DOOR_CLOSING, DOOR_STOPPED_NA);
  mStartMillis = millis(); // the door is starting to open or close, keep track of the starting time
  mIsOverMaxCurrent = false;
  mOverMaxCurrentTotalMillis = 0;
  mIsUnderMinCurrent = false;
  mUnderMinCurrentTotalMillis = 0;
  mDoorRampMode = RAMP_UP;
  mLimitSwitchTriggered = false;
  mpActuator->retract(gc_startSpeed);
}

//
// stop - stops the door 
//
void Door::stop(doorStatus status, doorStopCause stopCause) {
  updateStatus(status, stopCause);
  mpActuator->stop();
  // turn off door moving LED
  mDoorRampMode = RAMP_NONE;
  if (isDoorFault()) {
    setDoorFaultLED();
  }
}

//
// updateStatus
//
void Door::updateStatus(doorStatus newStatus, doorStopCause newStopCause) {
  mPrevStatus = mCurrStatus;
  mPrevStopCause = mCurrStopCause;
  mCurrStatus = newStatus;
  mCurrStopCause = newStopCause;
    
  DEBUG_PRINT(" door mPrevStatus = ");
  debugPrintStatus(mPrevStatus);
  DEBUG_PRINT(" door mPrevStopCause = ");
  debugPrintStopCause(mPrevStopCause);
  DEBUG_PRINTLN();

  DEBUG_PRINT(" door mCurrStatus = ");
  debugPrintStatus(mCurrStatus);
  DEBUG_PRINT(" door mCurrStopCause = ");
  debugPrintStopCause(mCurrStopCause);
  DEBUG_PRINTLN();
}

//
// processEvents - called when the door is opening or closing, 
//    checks limits switches, over current, under current, timout, ramps the actuator speed up or down
//
void Door::processEvents() {  
  if (mCurrStatus == DOOR_OPENING || mCurrStatus == DOOR_CLOSING) {
    int maxCurrent;
    if (mCurrStatus == DOOR_OPENING) {
      maxCurrent = gc_maxCurrentOpening;
    } else {
      maxCurrent = gc_maxCurrentClosing;
    }

    unsigned int actuatorCurrent = mpActuator->getCurrent();
    debugPrintCommaDelimitedInfo(actuatorCurrent);
    
    // Check for over max current
    if (actuatorCurrent > maxCurrent) {
      if (mIsOverMaxCurrent) {
        // the Current has already been over the max at least one time before this check
        mOverMaxCurrentTotalMillis = millis() - mOverMaxCurrentStartMillis;
      } else {
        // this is the first occurnace of the Current being over the max
        mIsOverMaxCurrent = true;
        mOverMaxCurrentStartMillis = millis();
        mOverMaxCurrentTotalMillis = 0;
      }
    } else {
      // the Current is below the max, so reset the over max flag and over max total milliseconds
      mIsOverMaxCurrent = false;
      mOverMaxCurrentTotalMillis = 0;
      // Check for under the min current
      if (actuatorCurrent < gc_minCurrent) {
        if (mIsUnderMinCurrent) {
          // the Current has already been under the min at least one time before this check 
          mUnderMinCurrentTotalMillis = millis() - mUnderMinCurrentStartMillis;
        } else {
          // this is the first occurnace of the Current being under the min
          mIsUnderMinCurrent = true;
          mUnderMinCurrentStartMillis = millis();
          mUnderMinCurrentTotalMillis = 0;
        }
      } else {
        // the Current is above the min, so reset the under min flag and under min total milliseconds
        mIsUnderMinCurrent = false;
        mUnderMinCurrentTotalMillis = 0;
      }
    }
    
    // Check for continious excessive current for more than the max allowed
    if (mOverMaxCurrentTotalMillis > gc_maxMillisAtOverCurrent) {
      DEBUG_PRINT(" mOverMaxCurrentTotalMillis = ");
      DEBUG_PRINT(mOverMaxCurrentTotalMillis);
      stop(DOOR_STOPPED, DOOR_STOPPED_ACTUATOR_EXCESS_CURRENT);
      return;
    }
    
    // Check for excessive running time - indicating that the door is jammed
    if (getRuntimeMillis() > gc_maxRuntimeMillis) {
      stop(DOOR_STOPPED, DOOR_STOPPED_TIMEOUT);     
      return;
    }

    if (mCurrStatus == DOOR_OPENING) {
      // Check for under the min current which indicates that the actuator internal limit switch has been triggered
      if (mUnderMinCurrentTotalMillis > gc_maxMillisAtUnderCurrent) {
        DEBUG_PRINT(" mUnderMinCurrentTotalMillis = ");
        DEBUG_PRINT(mUnderMinCurrentTotalMillis);
        stop(DOOR_OPEN, DOOR_STOPPED_ACTUATOR_ZERO_CURRENT);
        return;
      }
      // Check the door open limit switch
      if (mpOpenLimitSw->triggered(millis())) {
        if (!mLimitSwitchTriggered) {
          mLimitSwitchTriggered = true;
          mLimitSwitchTriggeredMillis = millis();
        }
        if (millis() >= mLimitSwitchTriggeredMillis + mOpenStopDelayMillis) {
          stop(DOOR_OPEN, DOOR_STOPPED_LIMIT_SW);
          return;
        } 
      }
    }

    if (mCurrStatus == DOOR_CLOSING) {
      // Check for under the min current which indicates that the actuator internal limit switch has been triggered
      if (mUnderMinCurrentTotalMillis > gc_maxMillisAtUnderCurrent) {
        DEBUG_PRINT(" mUnderMinCurrentTotalMillis = ");
        DEBUG_PRINT(mUnderMinCurrentTotalMillis);
        stop(DOOR_CLOSED, DOOR_STOPPED_ACTUATOR_ZERO_CURRENT);
        return;
      }
      // Check the door closed limit switch
      if (mpClosedLimitSw->triggered(millis())) {
        if (!mLimitSwitchTriggered) {
            mLimitSwitchTriggered = true;
            mLimitSwitchTriggeredMillis = millis();
        }
        if (millis() >= mLimitSwitchTriggeredMillis + mOpenStopDelayMillis) {
          stop(DOOR_CLOSED, DOOR_STOPPED_LIMIT_SW);
          return;
        } 
      }
    }

    // ramp actuator speed up or down
    if (mDoorRampMode == RAMP_UP) {
      mDoorRampMode = mpActuator->rampUp();
    } else if(mDoorRampMode == RAMP_DOWN) {
      mDoorRampMode = mpActuator->rampDown();
    } else if (millis() - mStartMillis > gc_startRampDownMillis) {
      mDoorRampMode = RAMP_DOWN;
    }
  }
}

//
// doorFault
//
bool Door::isDoorFault() {
  if (mCurrStatus == DOOR_STOPPED) {
    switch (mCurrStopCause) {
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
  if (mCurrStatus == DOOR_STOPPED) {
    switch (mCurrStopCause) {
      case DOOR_STOPPED_TIMEOUT:
        DEBUG_PPRINTLN("DOOR_STOPPED_TIMEOUT");
        digitalWrite(gc_timeoutFaultLEDPin, HIGH);
        return;
      case DOOR_STOPPED_OBSTRUCTION:
        DEBUG_PPRINTLN("DOOR_STOPPED_OBSTRUCTION");
        digitalWrite(gc_obstructionFaultLEDPin, HIGH);
        return;
      case DOOR_STOPPED_ACTUATOR_EXCESS_CURRENT:
        DEBUG_PPRINTLN("DOOR_STOPPED_ACTUATOR_EXCESS_CURRENT");
        digitalWrite(gc_excessCurrentFaultLEDPin, HIGH);
        return;
    }
  }
}

//
// utility debug function to print the door ID description as a string
//
void Door::debugPrintDoorDescrip(doorID ID) {
  switch (ID) {
    case DOOR1:
      DEBUG_PRINT("D1");
      break;
    case DOOR2:
      DEBUG_PRINT("D2");
      break;
    default:
      DEBUG_PRINT("??");
      break;
  }
}

//
// utility debug function to print the doorStopCause as a strinng
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
// utility debug function to print the doorStatus as a string
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
// utility debug function to print: door ID, current status, actuator current, run time
//
void Door::debugPrintCommaDelimitedInfo(unsigned int actuatorCurrent) {
    debugPrintDoorDescrip(mDoorID);
    DEBUG_PRINT(", ");
    debugPrintStatus(mCurrStatus);
    DEBUG_PRINT(", ");
    DEBUG_PRINT(actuatorCurrent);
    DEBUG_PRINT(", ");
    DEBUG_PRINTLN(getRuntimeMillis());
}

//
// end class Door
//

//
// DoorOpening class 
//
// this class defines the door opening - the opening has 2 doors and the obstruction sensor
//
class DoorOpening {
  public:
    DoorOpening();
    void open();
    void close();
    void stop(doorStatus status, doorStopCause stopCause);
    void processEvents();
    doorStatus getCurrStatus();
    doorStatus getPrevStatus();
    doorStatus getD1PrevStatus();
    doorStatus getD2PrevStatus();
    bool obstructionDetected();
  private:
    Door* mpDoor1;
    Door* mpDoor2;
    ObstructionSensor* mpObstructionSensor;
    bool mReverseProcessed = false; // used when the door motion needs to be reversed briefly after a fault
    void clearFaultLEDs();
    void setDoorMovingLEDMode(doorMovingLEDMode mode);
};
// end DoorOpening class definition

//
// DoorOpening
//
DoorOpening::DoorOpening() {
  setDoorMovingLEDMode(DM_OFF);

  // Obstruction Sensor Object 
  mpObstructionSensor = new ObstructionSensor(); 
  
  // create door 1 object (door 1 is the east door)
  mpDoor1 = new Door(DOOR1, gc_d1OpenLimitSwPin, gc_d1ApproachingOpenPin, gc_d1ClosedLimitSwPin, gc_d1ApproachingClosedPin, gc_d1OpenStopDelayMillis, gc_d1ClosedStopDelayMillis, 
    gc_m1FBPin, gc_m1SFPin, gc_m1PWMD2Pin, gc_m1In1Pin, gc_m1In2Pin, gc_enPin);
  
  // create door 2 object (door 2 is the west door)
  mpDoor2 = new Door(DOOR2, gc_d2OpenLimitSwPin, gc_d2ApproachingOpenPin, gc_d2ClosedLimitSwPin, gc_d2ApproachingClosedPin, gc_d2OpenStopDelayMillis, gc_d2ClosedStopDelayMillis, 
    gc_m2FBPin, gc_m2SFPin, gc_m2PWMD2Pin, gc_m2In1Pin, gc_m2In2Pin, gc_enPin);
}

//
// open
//
void DoorOpening::open() {
  clearFaultLEDs();
  setDoorMovingLEDMode(DM_ON);
  mReverseProcessed = false;
  mpDoor1->open();  
  mpDoor2->open();
}

//
// close
//
void DoorOpening::close() {
  clearFaultLEDs();
  mpObstructionSensor->turnOn(); // turn on the obstruction sensor
  delay(100); // delay to allow the obstruction sensor to start
  setDoorMovingLEDMode(DM_ON);
  mReverseProcessed = false;
  
  mpDoor1->close();  
  mpDoor2->close();
}

//
// stop
//
void DoorOpening::stop(doorStatus status, doorStopCause stopCause) {
  setDoorMovingLEDMode(DM_OFF);
  mpDoor1->stop(status, stopCause);
  mpDoor2->stop(status, stopCause);
  mpObstructionSensor->turnOff(); // turn off the obstruction sensor
}

//
// getCurrStatus
//
// determine the logical status of the door opening based the the status of each door
//  rule 1: the door opening is not open or closed until both doors are open or closed
//  rule 2: the door opening is opening or closing if either of the doors are opening or closing
//  rule 3: if one door is stopped (due to exception), the other door should be stopped (exceptions - such as obstruction, over current, button press during opening or closing)
//
doorStatus DoorOpening::getCurrStatus() {
  doorStatus door1Status = mpDoor1->getCurrStatus();
  doorStatus door2Status = mpDoor2->getCurrStatus();
  doorStatus doorOpeningStatus = DOOR_STOPPED;

  if (door1Status == DOOR_OPEN && door2Status == DOOR_OPEN) {
     doorOpeningStatus = DOOR_OPEN;
  } else if (door1Status == DOOR_CLOSED && door2Status == DOOR_CLOSED) {
     doorOpeningStatus = DOOR_CLOSED;
  } else if (door1Status == DOOR_OPENING || door2Status == DOOR_OPENING) {
     doorOpeningStatus = DOOR_OPENING;
  } else if (door1Status == DOOR_CLOSING || door2Status == DOOR_CLOSING) {
     doorOpeningStatus = DOOR_CLOSING;
  } else if (door1Status == DOOR_STOPPED || door2Status == DOOR_STOPPED) {
     doorOpeningStatus = DOOR_STOPPED;
  } 
  if (doorOpeningStatus == DOOR_STOPPED) {
    if (door1Status != DOOR_STOPPED) {
      // error - this should never happen
      mpDoor1->stop(DOOR_STOPPED, mpDoor2->getCurrStopCause());
    }
    if (door2Status != DOOR_STOPPED) {
      // error - this should never happen
      mpDoor2->stop(DOOR_STOPPED, mpDoor1->getCurrStopCause());
    }
  }
  return doorOpeningStatus;
}

//
// getPrevStatus
//
// return the previous status of the door opening - 
// use door1 as the "master" previous status - (return the previous status of door 1 for the door opening prevoous status)
//
doorStatus DoorOpening::getPrevStatus() {
  doorStatus door1PrevStatus = mpDoor1->getPrevStatus();
  return door1PrevStatus;
}

//
// getD1PrevStatus
//
// return the previous status of the door 
//
doorStatus DoorOpening::getD1PrevStatus() {
  return mpDoor1->getPrevStatus();
}

//
// getD2PrevStatus
//
// return the previous status of the door 
//
doorStatus DoorOpening::getD2PrevStatus() {
  return mpDoor2->getPrevStatus();
}

//
// processEvents
//
void DoorOpening::processEvents() {
  mpDoor1->processEvents();
  mpDoor2->processEvents();

  doorStatus d1CurrStatus = mpDoor1->getCurrStatus();
  doorStatus d2CurrStatus = mpDoor2->getCurrStatus();

  bool doorsAreStopped = false;
  if (d1CurrStatus == DOOR_STOPPED) {
    if (d2CurrStatus != DOOR_STOPPED) {
      mpDoor2->stop(DOOR_STOPPED, mpDoor1->getCurrStopCause()); // use the door 1 stop cause for the door 2 stop cause
    } 
    doorsAreStopped = true;
  } else if (d2CurrStatus == DOOR_STOPPED) {
    mpDoor1->stop(DOOR_STOPPED, mpDoor2->getCurrStopCause()); // use the door 2 stop cause for the door 1 stop cause
    doorsAreStopped = true;
  }
  if (doorsAreStopped) {
    // NOTE A "STOPPED" status indicates that the doors stopped due to some exception - not from a normal full open or close operation
    if (mpObstructionSensor->getIsOn()) {
      DEBUG_PRINTLN("DoorOpening::processEvents - 1");
      mpObstructionSensor->turnOff(); // turn off the obstruciton sensor
    }
    setDoorMovingLEDMode(DM_OFF);
    if (mpDoor1->getCurrStopCause() == DOOR_STOPPED_ACTUATOR_EXCESS_CURRENT || mpDoor2->getCurrStopCause() == DOOR_STOPPED_ACTUATOR_EXCESS_CURRENT) {
      // one or both doors are drawing excessive current - indicating a jam
      if (!mReverseProcessed) {
        // the doors are jammed, reverse the doors for a fraction of a second
        // don't update the door status when doing the reversing
        if (getPrevStatus() == DOOR_OPENING) {
          mpDoor1->get_pActuator()->retract();
          mpDoor2->get_pActuator()->retract();
          delay(250);
          mpDoor1->get_pActuator()->stop();        
          mpDoor2->get_pActuator()->stop();
          mReverseProcessed = true;
        } else if (getPrevStatus() == DOOR_CLOSING) {
          mpDoor1->get_pActuator()->extend();
          mpDoor2->get_pActuator()->extend();
          delay(250);
          mpDoor1->get_pActuator()->stop();
          mpDoor2->get_pActuator()->stop();
          mReverseProcessed = true;
        }
      }
    }
  } else if ((d1CurrStatus == DOOR_OPEN && d2CurrStatus == DOOR_OPEN) || (d1CurrStatus == DOOR_CLOSED && d2CurrStatus == DOOR_CLOSED)) {
    if (mpObstructionSensor->getIsOn()) {
      mpObstructionSensor->turnOff(); // turn off the obstruction sensor
    }
    setDoorMovingLEDMode(DM_OFF);
 }
}

//
// clearFaultLEDs
//
void DoorOpening:: clearFaultLEDs() {
  digitalWrite(gc_obstructionFaultLEDPin, LOW);
  digitalWrite(gc_excessCurrentFaultLEDPin, LOW);
  digitalWrite(gc_timeoutFaultLEDPin, LOW);
}

// 
// setDoorMovingLEDMode()
//
void DoorOpening::setDoorMovingLEDMode(doorMovingLEDMode mode) {
  if (mode == DM_OFF) {
    digitalWrite(gc_doorMovingLEDPin, LOW);
  } else if (mode == DM_ON) {
    digitalWrite(gc_doorMovingLEDPin, HIGH);
  }
}

//
// obstructionDetected()
//
bool DoorOpening::obstructionDetected() {
  return mpObstructionSensor->obstructionDetected(); 
}


//
// end class DoorOpening
//

///////////////////////////////////////////////////////////////////////////////////////////////
// end Class Definitions
///////////////////////////////////////////////////////////////////////////////////////////////


// door opening pointer
DoorOpening* pDoorOpening;

// button pointer
DebouncedSwitch* pButton; 

// setup
void setup()
{
  Serial.begin(115200);
  DEBUG_PRINTLN("Garage Door Opener");

  // pins for actuator driver board - actuator 1
  pinMode(gc_m1FBPin, INPUT);
  pinMode(gc_m1SFPin, INPUT);
  pinMode(gc_m1PWMD2Pin, OUTPUT);
  pinMode(gc_m1In1Pin, OUTPUT);
  pinMode(gc_m1In2Pin, OUTPUT);
  // pins for actuator driver board - actuator 2  
  pinMode(gc_m2FBPin, INPUT);
  pinMode(gc_m2SFPin, INPUT);
  pinMode(gc_m2PWMD2Pin, OUTPUT);
  pinMode(gc_m2In1Pin, OUTPUT);
  pinMode(gc_m2In2Pin, OUTPUT);
  // pins for actuator driver board - print applies to both actuators
  pinMode(gc_enPin, OUTPUT);

  // button and obstruction sensor pins
  pinMode(gc_buttonPin, INPUT); // this pin needs a pull down resistor
  pinMode(gc_obstructionSensorDetectPin, INPUT_PULLUP);
  pinMode(gc_ObstructionSensorPowerPin, OUTPUT);
  digitalWrite(gc_ObstructionSensorPowerPin, LOW); // turn off the obstruction sensor
  DEBUG_PRINTLN("gc_ObstructionSensorPowerPin - 0, LOW");

  // limit switch and approaching limit pins
  // door 1
  pinMode(gc_d1OpenLimitSwPin, INPUT_PULLUP);
  pinMode(gc_d1ApproachingOpenPin, INPUT_PULLUP);
  pinMode(gc_d1ClosedLimitSwPin, INPUT_PULLUP);
  pinMode(gc_d1ApproachingClosedPin, INPUT_PULLUP);
  // door 2
  pinMode(gc_d2OpenLimitSwPin, INPUT_PULLUP);
  pinMode(gc_d2ApproachingOpenPin, INPUT_PULLUP);
  pinMode(gc_d2ClosedLimitSwPin, INPUT_PULLUP);
  pinMode(gc_d2ApproachingClosedPin, INPUT_PULLUP);

  // LEDs
  pinMode(gc_doorMovingLEDPin, OUTPUT);
  pinMode(gc_obstructionFaultLEDPin, OUTPUT);
  pinMode(gc_excessCurrentFaultLEDPin, OUTPUT);
  pinMode(gc_timeoutFaultLEDPin, OUTPUT);
  
  // create and initialize door opening object
  pDoorOpening = new DoorOpening();

  // Button  
  // (the button pin is held low with a pull down resistor, when the button is pressed it completes a circuit and sends 5 volts to the arduino pin)
  pButton = new DebouncedSwitch(gc_buttonPin, HIGH); 
  
  //Flash the LEDs to indicate that the controller is ready
  for (int i=0; i < 3; i++) {
    digitalWrite(gc_doorMovingLEDPin, HIGH);
    digitalWrite(gc_obstructionFaultLEDPin, HIGH);
    digitalWrite(gc_timeoutFaultLEDPin, HIGH);
    digitalWrite(gc_excessCurrentFaultLEDPin, HIGH);
    delay(200);
    digitalWrite(gc_doorMovingLEDPin, LOW);
    digitalWrite(gc_obstructionFaultLEDPin, LOW);
    digitalWrite(gc_timeoutFaultLEDPin, LOW);
    digitalWrite(gc_excessCurrentFaultLEDPin, LOW);
    delay(200);
  }
}

// Main loop
bool buttonPressProcessed = false;
void loop()
{
  if (pDoorOpening->getCurrStatus() == DOOR_CLOSING && pDoorOpening->obstructionDetected()) {
    pDoorOpening->stop(DOOR_STOPPED, DOOR_STOPPED_OBSTRUCTION);
  } else { 
    if (pButton->triggered(millis())) {
      // only respond to the button press once per button press, buttonPressProcessed will get reset to false when the button is released
      if (!buttonPressProcessed) {
        buttonPressProcessed = true;
        DEBUG_PRINTLN("button pushed - and not processed");
        doorStatus currStatus = pDoorOpening->getCurrStatus();
        DEBUG_PPRINT("pDoorOpening->getCurrStatus() = ");
        DEBUG_PRINTLN(currStatus);       
        if (currStatus == DOOR_OPEN) {
          DEBUG_PRINTLN("door is DOOR_OPEN");
          // start closing door 
          pDoorOpening->close();
        } else if (currStatus == DOOR_CLOSED) {
          DEBUG_PRINTLN("door is DOOR_CLOSED");
          // start opening door 
          pDoorOpening->open();
        } else if (currStatus == DOOR_OPENING) {
          DEBUG_PRINTLN("door is DOOR_OPENING");
          // stop doors 
          pDoorOpening->stop(DOOR_STOPPED, DOOR_STOPPED_BUTTON_WHILE_OPENING);
        } else if (currStatus == DOOR_CLOSING) {
          DEBUG_PRINTLN("door is DOOR_CLOSING");
          // stop doors
          pDoorOpening->stop(DOOR_STOPPED, DOOR_STOPPED_BUTTON_WHILE_CLOSING);
        } else if (currStatus == DOOR_STOPPED) {
          DEBUG_PRINTLN("door is DOOR_STOPPED");
          // if previous status was opening, then start closing the door, otherwise start opening the door 
          if (pDoorOpening->getPrevStatus() == DOOR_OPENING) {
            pDoorOpening->close();
          } else {
            if (pDoorOpening->getD1PrevStatus() == DOOR_OPENING || pDoorOpening->getD2PrevStatus() == DOOR_OPENING) {
              pDoorOpening->close();
            } else if (pDoorOpening->getD1PrevStatus() == DOOR_CLOSING || pDoorOpening->getD2PrevStatus() == DOOR_CLOSING) {
              pDoorOpening->open();
            } else {
              pDoorOpening->open();
            }
          }
        } 
      }
    } else {
      buttonPressProcessed = false;
    }
  }   
  delay(10);
  // check for limit switches, timeout, excess current, and other events that need to be processed 
  pDoorOpening->processEvents(); 
}
