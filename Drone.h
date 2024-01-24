#ifndef Drone_h
#define Drone_h
#include <Servo.h>

class Drone
{
private: 
  Servo servo[4];      //array of servo objects
  int servoPin[4];     //array for digital servo pin nums
  int solenoidPin[2];  //array for solenoid pin nums
  int hallSensorPin[4];
  int parachutePin;     //varaible for parachute pin num
  int altimeterPin;     //variable for altimeter pin num
  int buttonPin = 2;    //variable for the switch command pin num

  int buttonState = 0;  //state of button command
  int servoState = 0;   //state of servo motors
  int hallSensorValue = 0;

public:
  Drone(int, int, int, int, int, int, int, int, int, int, int);
  void DroneSetup(); 
  void getButtonInput();
  void flightMode();
  void correctServoPosition(int, int);
  int readHallSensor(int);
  void sleepMode(int);
};

#endif /*Drone_h */