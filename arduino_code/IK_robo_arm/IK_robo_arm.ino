/* calibration
base_servo - 9
arm_servo - 10
end_eff - 11
*/

#include <Servo.h>

Servo base,arm,ee;  // create servo object to control a servo
float val =1;
float t_param = 4.0,increment=1.0;
unsigned long prev_millis = 0;
int pos = 90;    // variable to store the servo position(0-180)

void setup() {
 Serial.begin(9600);

  base.attach(9);  // attaches the servo on pin 9 to the servo object
  arm.attach(10);  // attaches the servo on pin 10 to the servo object
  ee.attach(11);  // attaches the servo on pin 11 to the servo object
  
  draw_pen(false);
  delay(200);
  base.write(pos);              // tell servo to go to position in variable 'pos'
  delay(200);          
  arm.write(pos);              // tell servo to go to position in variable 'pos'
  delay(200);   
  ee.write(pos);              // tell servo to go to position in variable 'pos'
  delay(1000);  
  // draw_pen(true);
  // delay(1000);
}


void loop() {

 
  float pos[3];
  float result_var[3]; // the end_eff direction is ignored here
  
  pathGeneration(pos,t_param);
  inverse_kine(pos[0],pos[1],pos[2],0,result_var);
  
  //delay(100); //NEVER use delay() in sketch
  
  joint1(result_var[0]*180/PI);
  joint2(result_var[1]*180/PI);
  joint3(result_var[2]*180/PI);

  
//  if(t_param>=5.3)
//    increment = -1;
//  else if(t_param<4)
//    increment = 1;
  if(millis()-prev_millis >5){
    prev_millis = millis();
    t_param= t_param+increment*0.2;  // best :0.038
  }

  // t_param= t_param+increment*0.2;  // best :0.038
  //delay(15);

}
void pathGeneration(float coordinate[],float t)
{     
      if(t==0.2)
        draw_pen(true);
      coordinate[0] = 2*cos(t)+20;
      coordinate[1] = 4*sin(2*t)+0;
      coordinate[2] = 0; 
  // coordinate[0] = 16;
  // coordinate[1] = 16;
  // coordinate[2] = 0;  
}

// Normalize angle between [-PI, PI]
float normalizeAngle(float theta) {
  while (theta > PI) theta -= TWO_PI;
  while (theta < -PI) theta += TWO_PI;
  return theta;
}

boolean withinLimits(float theta, float min, float max) {
  return theta >= min && theta <= max;
}

void inverse_kine(float x,float y,float z,float time_stamp,float t[])
{
//// End- effector orientation:

 float p11 = 1,p12 = 0,p13 = 0;
 float p21 = 0,p22 = 1,p23 = 0;
 float p31 = 0,p32 = 0,p33 = 1;
 
    
//2DOF planner robot (considering end-efector rotation)  
  //assumue end-effector rotating around Z0-axis
  //  |p11  p12  0  x|
  //  |p21  p22  0  y|
  //  |0    0    1  0|
  //  |0    0    0  1|
  float r1 = 10.5,r2 = 12 , r3 = 0.5;

 
  float dx = x - r3 * p11;
  float dy = y - r3 * p21;
  
  float ct2 = (dx * dx + dy * dy - r1 * r1 - r2 * r2) / (2 * r1 * r2);

  // Limit cosine to [-1, 1] range
  if (ct2 < -1 || ct2 > 1) {
    Serial.println("No solution: ct2 out of range.");
    return;
  }

  float st2_p = sqrt(1 - ct2 * ct2);
  float st2_m = -st2_p;

  // Limits (adjust as needed)
  float theta1_min = -PI/2, theta1_max = PI/2;
  float theta2_min = -PI/2, theta2_max = PI/2;
  float theta3_min = -PI/2, theta3_max = PI/2;

  float all_t[2][3];

  // First solution (elbow-down)
  all_t[0][1] = atan2(st2_p, ct2);  // t[1] = θ2
  all_t[0][0] = atan2(-r2 * st2_p, r1 + r2 * ct2) + atan2(dy, dx);  // t[0] = θ1
  all_t[0][2] = -(all_t[0][0] + all_t[0][1]) + atan2(p21, p11);     // t[2] = θ3

  // Second solution (elbow-up)
  all_t[1][1] = atan2(st2_m, ct2);  // t[1] = θ2
  all_t[1][0] = atan2(-r2 * st2_m, r1 + r2 * ct2) + atan2(dy, dx);  // t[0] = θ1
  all_t[1][2] = -(all_t[1][0] + all_t[1][1]) + atan2(p21, p11);     // t[2] = θ3
  // Print valid solutions
  for (int i = 0; i < 2; i++) {
    float t0 = normalizeAngle(all_t[i][0]);
    float t1 = normalizeAngle(all_t[i][1]);
    float t2 = normalizeAngle(all_t[i][2]);

    if (withinLimits(t0, theta1_min, theta1_max) &&
        withinLimits(t1, theta2_min, theta2_max) &&
        withinLimits(t2, theta3_min, theta3_max)) {
      if(i==0)  // repor only the first valid solution
      {
      t[0]=t0;t[1]=t1;t[2]=t2;
      }
      Serial.print("Valid solution ");
      Serial.print(i);
      Serial.println(":");

      Serial.print("t[0] = ");
      Serial.print(degrees(t0));
      Serial.println("°");

      Serial.print("t[1] = ");
      Serial.print(degrees(t1));
      Serial.println("°");

      Serial.print("t[2] = ");
      Serial.print(degrees(t2));
      Serial.println("°");


    }
    Serial.println("_______________________________________");
  }
  
 
//Print Output  
//      Serial.print("t[0]:");
//      Serial.print(t[0]*180/PI);
//      
//      Serial.print("    t[1]:");
//      Serial.print(t[1]*180/PI);
//      
//      Serial.print("    t[2]:");
//      Serial.print(t[2]*180/PI);
//      
//      Serial.print("    t[3]:");
//      Serial.print(t[3]*180/PI);
//      
//      Serial.print("    t[4]:");
//      Serial.print(t[4]*180/PI);
//      
//      Serial.print("    t[5]:");
//      Serial.print(t[5]*180/PI);
//      
//Serial.println("   ");
}












// Joint declaration::..............................::

void joint1(float theta1)
{
  // theta1 = 0* ----> PWM:80  
  // theta1 = 180* ----> PWM:480  
  
  //output 0 to 180
  //input 90 to -90
  //int pos = map(theta1,-90,90,0,180);
  int pos = theta1+90;
  base.write(pos);
  //delay(15);
}

void joint2(float theta2)
{
  // 0 ... -180 is taken from processing3 sketch
  
  //output 0 to 180
  //input -90 to +90
  //int pos = map(theta2,90,-90,0,180);
  int pos = -theta2+90; 
  
  arm.write(pos);
  // delay(50);
}

void joint3(float theta3)
{
  // 90 ... -90 is taken from processing3 sketch
  int pos = map(theta3,-90,90,0,180);
}
void draw_pen(bool draw_)
{
  if(draw_){
    ee.write(90);
    delay(50);
  }else{
    ee.write(20);
    delay(50);
  }
}