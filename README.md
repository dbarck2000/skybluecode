# skybluecode 
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

