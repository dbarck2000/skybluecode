//
// SerialDebug.h - Simple debugging utilities.
//

#ifndef SERIALDEBUG_H
#define SERIALDEBUG_H

#ifdef DEBUG
  #define DEBUG_PPRINT(...)  \
        Serial.print(__PRETTY_FUNCTION__); \
        Serial.print(' ');      \
        Serial.print(__VA_ARGS__)
  #define DEBUG_PRINT(...)  \
        Serial.print(__VA_ARGS__)
  #define DEBUG_PPRINTLN(...)  \
        Serial.print(__PRETTY_FUNCTION__); \
        Serial.print(' ');      \
        Serial.println(__VA_ARGS__)
  #define DEBUG_PRINTLN(...)  \
        Serial.println(__VA_ARGS__)
#else
  #define DEBUG_PPRINT(...)
  #define DEBUG_PRINT(...)
  #define DEBUG_PPRINTLN(...) 
  #define DEBUG_PRINTLN(...) 
#endif

#endif


