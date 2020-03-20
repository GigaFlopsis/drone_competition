/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

String inString = "";    // string to hold input

int minVal = 0;
int maxVal = 180;

void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  myservo.write(90);              // tell servo to go to position in variable 'pos'
   Serial.begin(9600);

 while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // send an intro:
  Serial.println("\n\nString toInt():");
  Serial.println();
}

void loop() {
   // Read serial input:
  while (Serial.available() > 0) {
    int inChar = Serial.read();
    if (isDigit(inChar)) {
      // convert the incoming byte to a char and add it to the string:
      inString += (char)inChar;
    }
    // if you get a newline, print the string, then the string's value:
    if (inChar == '\n') {
      int value = inString.toInt();
      if (value == 0)
      {
      myservo.write(maxVal);              // tell servo to go to position in variable 'pos'  
      }
      if (value == 1)
      {
      myservo.write(minVal);              // tell servo to go to position in variable 'pos'  
      }
      
      Serial.print("Value:");
      Serial.println(inString.toInt());
      Serial.print("String: ");
      Serial.println(inString);
      
      // clear the string for new input:
      inString = "";
    }
  }
}
