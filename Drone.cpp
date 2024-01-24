
//Done class implementation file to operate the servo motors
#include <Arduino.h>
#include "Drone.h"
#include <Servo.h>

Drone::Drone(int s1, int s2, int s3, int s4, int sol1, int sol2, int button, int h1, int h2, int h3, int h4)  //constructor
{
  //set servo pins
  servoPin[0] = s1;
  servoPin[1] = s2;
  servoPin[2] = s3;
  servoPin[3] = s4;

  //set solenoid pins
  solenoidPin[0] = sol1;
  solenoidPin[1] = sol2;

  //set btton pin
  buttonPin = button;

  //set hall sensor pins
  hallSensorPin[0] = h1;
  hallSensorPin[1] = h2;
  hallSensorPin[2] = h3;
  hallSensorPin[3] = h4;
}

void Drone::DroneSetup() 
{
  Serial.begin(9600);
  for(int i=0; i<4; i++)
  {
    pinMode(servoPin[i], OUTPUT);
    pinMode(hallSensorPin[i], INPUT);
  }
  
  for (int i = 0; i < 4; i++) {
    //link the servos to their respective pins
    servo[i].attach(servoPin[i]);
  }

  //setup the button and the solenoid
  pinMode(buttonPin, INPUT);
  pinMode(solenoidPin[0], OUTPUT);
  pinMode(solenoidPin[1], OUTPUT);

  // //ensure that the servo arms are in their 90 degree start position
  // for (int i = 0; i < 4; ++i) 
  // {
  //   servo[i].write(servo[i].read());  // Set servos to low-power state
  // }
  //Check hall sensors for each arm before attempting to move arms
  for (int i = 0; i < 4; ++i)
  {
    hallSensorValue = readHallSensor(i);

    Serial.print("Arm ");
    Serial.print(i);
    Serial.print(" Hall Sensor: ");
    Serial.println(hallSensorValue);

    // If the servo is supposed to be in closed position (servoState is LOW),
    // but the hall sensor reads HIGH (indicating open position), correct the servo position
    if (servoState == LOW && hallSensorValue == LOW)
    {
      Serial.println("Correcting Servo Position to Closed");
      correctServoPosition(i, 0);
      delay(300);
    }
    // If the servo is supposed to be in flight position (servoState is HIGH),
    // but the hall sensor reads LOW (indicating closed position), correct the servo position
    else if (servoState == HIGH && hallSensorValue == HIGH)
    {
      Serial.println("Correcting Servo Position to Open");
      correctServoPosition(i, 1);
      delay(300);
    }
  }

  // // servo[0].writeMicroseconds(1500);  // Set servos to low-power state
  // // servo[2].writeMicroseconds(1500);
  // servo[0].write(servo[2].read());
  // servo[2].write(servo[2].read());
  // delay(10);

  // digitalWrite(solenoidPin[0], HIGH);
  // delay(800);
  
  // servo[0].write(90.5);
  // servo[2].write(90.5);
  // delay(800);
  // digitalWrite(solenoidPin[0], LOW);
  // delay(800);

  // // servo[1].writeMicroseconds(1500);  // Set servos to low-power state
  // // servo[3].writeMicroseconds(1500);
  // servo[1].write(servo[1].read());
  // servo[3].write(servo[3].read());
  // delay(10);

  // digitalWrite(solenoidPin[1], HIGH);
  // delay(800);
   
  // servo[1].write(90.5);
  // servo[3].write(90.5);
  // delay(800);

  // digitalWrite(solenoidPin[1], LOW);
  // delay(800);
}


void Drone::getButtonInput() {
  //here we continously read the current input of the button/switch
  buttonState = digitalRead(buttonPin);
}


void Drone::flightMode()
{
  for (int i = 0; i < 4; ++i) 
  {
    servo[i].write(servo[i].read());  // Set servos to low-power state
  }
  //Check hall sensors for each arm before attempting to move arms
  for (int i = 0; i < 4; ++i)
  {
    hallSensorValue = readHallSensor(i);

    Serial.print("Arm ");
    Serial.print(i);
    Serial.print(" Hall Sensor: ");
    Serial.println(hallSensorValue);

    // If the servo is supposed to be in closed position (servoState is LOW),
    // but the hall sensor reads HIGH (indicating open position), correct the servo position
    if (servoState == LOW && hallSensorValue == LOW)
    {
      Serial.println("Correcting Servo Position to Closed");
      correctServoPosition(i, 0);
      delay(300);
    }
    // If the servo is supposed to be in flight position (servoState is HIGH),
    // but the hall sensor reads LOW (indicating closed position), correct the servo position
    else if (servoState == HIGH && hallSensorValue == HIGH)
    {
      Serial.println("Correcting Servo Position to Open");
      correctServoPosition(i, 1);
      delay(300);
    }
  }

  // Check the button state after checking hall sensors
  Serial.print("Button State: ");
  Serial.println(buttonState);

  if (buttonState == HIGH)
  {
    if (servoState == LOW) // Case when the drone is in the closed position and we want to open
    {
      //first group 
      Serial.println("Opening Servo Group 1");
      // servo[0].write(servo[0].read());
      // servo[2].write(servo[2].read());
      servo[0].writeMicroseconds(1500);
      servo[2].writeMicroseconds(1500);
      delay(10);

      digitalWrite(solenoidPin[0], HIGH);
      delay(800);

      //servoAttach(0,2);
      servo[0].write(135);
      servo[2].write(135);
      delay(800);

      digitalWrite(solenoidPin[0], LOW);
      delay(800);
      //servoDetach(0,2);

      //second group 
      Serial.println("Opening Servo Group 2");
      // servo[1].write(servo[1].read());
      // servo[3].write(servo[3].read());
      servo[1].writeMicroseconds(1500);
      servo[3].writeMicroseconds(1500);
      delay(10);

      digitalWrite(solenoidPin[1], HIGH);
      delay(800);

      //servoAttach(1,3);
      servo[1].write(45);
      servo[3].write(45);
      delay(800);

      digitalWrite(solenoidPin[1], LOW);
      delay(800);
      //servoDetach(1,3);

      servoState = HIGH; // Flag that the servos are now in the flight position
      delay(600);
    }
    else // Case when the drone is in open position and we want to close
    {
      Serial.println("Closing Servos Group 1");
      // servo[0].write(servo[0].read());
      // servo[2].write(servo[2].read());
      servo[0].writeMicroseconds(1500);
      servo[2].writeMicroseconds(1500);
      delay(10);

      digitalWrite(solenoidPin[0], HIGH);
      delay(800);

      //first group
      //servoAttach(0,2);
      servo[0].write(90);
      servo[2].write(90);
      delay(800);

      digitalWrite(solenoidPin[0], LOW);
      delay(800);
      //servoDetach(0,2);


      //second group 
      Serial.println("Closing Servos Group 2");
      // servo[1].write(servo[1].read());
      // servo[3].write(servo[3].read());
      servo[0].writeMicroseconds(1500);
      servo[2].writeMicroseconds(1500);
      delay(10);

      digitalWrite(solenoidPin[1], HIGH);
      delay(800);


      //servoAttach(1,3);
      servo[1].write(90);
      servo[3].write(90);
      delay(800);

      digitalWrite(solenoidPin[1], LOW);
      //servoDetach(1,3);
      servoState = LOW; // Flag that the servos are now in the closed position
      delay(1000);
    }

    buttonState = LOW;
  }
}


int Drone::readHallSensor(int armNum)
{
  // Replace this with your actual implementation to read the hall sensor for the specified arm
  // Return HIGH if the hall sensor detects the magnet (closed position), LOW otherwise
  // For testing purposes, assuming a digital hall sensor:
  return digitalRead(hallSensorPin[armNum]);
}

void Drone::correctServoPosition(int armNum, int errorCode)
{
  if(errorCode != -1)
  {
    int s = armNum % 2; // Determine the solenoid index based on the arm number

    servo[armNum].writeMicroseconds(1500);
    delay(10);
    digitalWrite(solenoidPin[s], HIGH); // Activate the corresponding solenoid
    delay(300);
    delay(50);

    if (errorCode == 0) // Error code for when arms need to be closed but are open
    {
      servo[armNum].write(90); //all servos have a closed position of 90 degrees
    }
    else if (errorCode == 1) // Error code for when arms need to be open but are closed
    {
      if (s == 0)
      {
        servo[armNum].write(135); //open arm 0 or 1 
      }
      else if (s == 1)
      {
        servo[armNum].write(45); //open arm 2 or 3
      }
    }

    delay(300);
    digitalWrite(solenoidPin[s], LOW); // Deactivate the solenoid
    delay(600);
    errorCode = -1;
  }
}

void Drone::sleepMode(int i)
{
  servo[i].writeMicroseconds(1500);
}

// void Drone::flightMode() {
//   if (buttonState == HIGH && servoState == LOW)  //this is the case when drone is in closed position
//   {
//     //command first group of servos and solenoids
//     digitalWrite(solenoidPin[0], HIGH);
//     delay(600);
//     servo[0].write(135);
//     servo[2].write(135);
//     delay(600);
//     digitalWrite(solenoidPin[0], LOW);
//     delay(600);

//     //command second group of servos and solenoids
//     digitalWrite(solenoidPin[1], HIGH);
//     delay(600);
//     servo[1].write(45);
//     servo[3].write(45);
//     delay(600);
//     digitalWrite(solenoidPin[1], LOW);
//     servoState = HIGH;  //this flags that the servos are now in flight position
//     delay(600);
//   }

//   else if (buttonState == HIGH && servoState == HIGH)  //case when drone is in flight position
//   {
//     //command first group of servos and solenoids
//     digitalWrite(solenoidPin[0], HIGH);
//     delay(600);
//     servo[0].write(90);
//     servo[2].write(90);
//     delay(600);
//     digitalWrite(solenoidPin[0], LOW);
//     delay(600);

//     //command second group of servos and solenoids
//     digitalWrite(solenoidPin[1], HIGH);
//     delay(600);
//     servo[1].write(90);
//     servo[3].write(90);
//     delay(600);
//     digitalWrite(solenoidPin[1], LOW);
//     servoState = LOW;  //flags that the servos are now in closed position
//     delay(600);
//   }

//   buttonState = LOW;
// }

////////////////////////////////////////////////////////////////////////////
// void Drone::flightMode()
// {
 
//   // Check hall sensors for each arm first before attempting to move arms
//   for (int i = 0; i < 1; ++i)
//   {
//     hallSensorValue = readHallSensor(i);

//     // If the servo is supposed to be in closed position (servoState is LOW),
//     // but the hall sensor reads HIGH (indicating open position), correct the servo position
//     if (servoState == LOW && hallSensorValue == LOW)
//     {
//       correctServoPosition(i,0);
//     }
//     // If the servo is supposed to be in flight position (servoState is HIGH),
//     // but the hall sensor reads LOW (indicating closed position), correct the servo position
//     else if (servoState == HIGH && hallSensorValue == HIGH)
//     {
//       correctServoPosition(i,1);
//     }

//     hallSensorValue = 0;
//   }
  
//   if(buttonState == HIGH)
//   {
//     if (servoState == LOW) // Case when the drone is in the closed position and we want to open
//     {
//       digitalWrite(solenoidPin[0], HIGH);
//       delay(600);
//       servo[0].write(135);
//       servo[2].write(135);
//       delay(600);
//       digitalWrite(solenoidPin[0], LOW);
//       delay(600);

//       digitalWrite(solenoidPin[1], HIGH);
//       delay(600);
//       servo[1].write(45);
//       servo[3].write(45);
//       delay(600);
//       digitalWrite(solenoidPin[1], LOW);

//       servoState = HIGH; // Flag that the servos are now in the flight position
//       delay(600);
//     }

//     else // Case when the drone is in open position and we want to close
//     {
//       digitalWrite(solenoidPin[0], HIGH);
//       delay(600);
//       servo[0].write(90);
//       servo[2].write(90);
//       delay(600);
//       digitalWrite(solenoidPin[0], LOW);
//       delay(600);

//       digitalWrite(solenoidPin[1], HIGH);
//       delay(600);
//       servo[1].write(90);
//       servo[3].write(90);
//       delay(600);
//       digitalWrite(solenoidPin[1], LOW);

//       servoState = LOW; // Flag that the servos are now in the closed position
//       delay(1000);
//     }

//     buttonState = LOW;
  
//   }  
// }





