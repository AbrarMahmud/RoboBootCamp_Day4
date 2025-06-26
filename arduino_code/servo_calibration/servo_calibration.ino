/* calibration
base_servo - 9
arm_servo - 10
end_eff - 11
*/

#include <Servo.h>

Servo base,arm,ee;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 90;    // variable to store the servo position(0-180)

void setup() {

  base.attach(9);  // attaches the servo on pin 9 to the servo object
  arm.attach(10);  // attaches the servo on pin 10 to the servo object
  ee.attach(11);  // attaches the servo on pin 11 to the servo object
  
  pos = 0;
  base.write(pos);              // tell servo to go to position in variable 'pos'
  delay(200);          
  arm.write(pos);              // tell servo to go to position in variable 'pos'
  delay(200);   
  ee.write(pos);              // tell servo to go to position in variable 'pos'
  delay(5000);   
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    base.write(pos);             
    delay(50);                       
  }
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    arm.write(pos);             
    delay(50);                       
  }
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    ee.write  (pos);             
    delay(10);                       
  }
  pos = 90;
}

void loop() {
    base.write(pos);              // tell servo to go to position in variable 'pos'
    delay(200);          
    arm.write(pos);              // tell servo to go to position in variable 'pos'
    delay(200);   
    ee.write(pos);              // tell servo to go to position in variable 'pos'
    delay(1000);   
}
