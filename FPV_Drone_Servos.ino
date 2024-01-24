

//Useful class type functions
// Attach(int) : connect servo to arduino through Pin 'int'
//write (int) : write an angel to servo 
//read() : read the current angle of the servo


#include "Drone.h"

Drone myDrone (2,3,4,5,6,7,8,A5,A4,A3,A2); //declare a new drone and corresponding pins

void setup() 
{ 
  Serial.begin(9600);
  myDrone.DroneSetup(); //setup of drone 
}

void loop() 
{
  myDrone.getButtonInput();
  myDrone.flightMode();
}

