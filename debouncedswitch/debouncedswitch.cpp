//
// DebouncedSwitch.cpp - Library for a debounced switch (button, reed switch, relay switch, etc...)
// The member function triggered(unsigned long nowMs) will return true when the pin state has remained at the target value (HIGH or LOW) for the debounced number of milliseconds 
// Created by David D. Barck January 20, 2020
//

#include "Arduino.h"
#include "debouncedswitch.h"

// constructor
// pin - the pin that the switch is wired to
// triggerValue (HIGH or LOW) is the value of the pin that signals that the switch is activated
// 
DebouncedSwitch::DebouncedSwitch(byte pin, uint8_t triggerValue) {
  _pin = pin;
  _triggerValue = triggerValue;
  _startDebounceMs = 0;
  _debounceMs = 50; // the number of milli seconds the pin state must stay at the same value to be considered stable  
  
  if (_triggerValue == HIGH) { 
    // The pin will go HIGH when the switch or button is activated - the pin will be pulled LOW when not triggered
    _pinState = LOW;
    _prevPinState = LOW;
  } else {
    // The pin will go LOW when the switch or button is activated  - the pin will be pulled HIGH when not triggered
    _pinState = HIGH;
    _prevPinState = HIGH;
  }
}

//return true if the switch or button is activated / triggered and stable (debounced)
bool DebouncedSwitch::triggered(unsigned long nowMs){
  bool retVal = false;
  uint8_t reading = digitalRead(_pin);
  if (reading == _prevPinState) {
    if ((nowMs - _startDebounceMs) > _debounceMs) {
      // whatever the reading is, it's been the same for longer than the debounce milliseconds

      // if the pin state has changed:
      if (reading != _pinState) {
        _pinState = reading;
      }
      retVal = reading == _triggerValue;
    }
  } else {
    //reading != _prevPinState
    //reset the debounce time
    _startDebounceMs = nowMs;
  }
  // save the reading, the next time through the loop, it'll be the _prevPinState
  _prevPinState = reading;
  return retVal;
}

// set the value of _debounceMs (default value is set in the constructor)
void DebouncedSwitch::setDebounceMs(int debounceMs) {
	_debounceMs = (unsigned int)debounceMs;
}