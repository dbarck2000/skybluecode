//
// DebouncedSwitch.h - Library for a debounced switch (button, reed switch, relay switch, etc...)
// The member function triggered(uint8_t nowMs) will return true when the pin state has remained at the target value (HIGH or LOW) for the debounced number of milliseconds 
// Created by David D. Barck January 20, 2020
//
#ifndef Arduino_h
#include "Arduino.h"
#endif

#ifndef debouncedswitch_h
#define debouncedswitch_h

class DebouncedSwitch {
  protected:
    byte _pin;                  	//hardware pin number
    uint8_t _triggerValue;         	//if the pin mode is INPUT_PULLUP, then the pin will read LOW when triggered, otherwise the pin will read HIGH when triggered
    uint8_t _pinState;            	//current pin state (HIGH or LOW)
    uint8_t _prevPinState;        	//previous pin state (HIGH or LOW)
    unsigned long _startDebounceMs;     //start of debounce time  
    unsigned int _debounceMs; 		//period before Switch latches (time before the Switch pin is considered stable in the new state)
  public:
    DebouncedSwitch(byte pin, uint8_t tiggerValue);   
    boolean triggered(unsigned long nowMs);
    void setDebounceMs(int debounceMs);
};
#endif
