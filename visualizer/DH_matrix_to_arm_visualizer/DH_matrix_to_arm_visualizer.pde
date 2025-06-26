import peasy.*;
import processing.serial.*;
PeasyCam cam;
Serial myPort;
//constants
float cm = 40; //defining 40 pixels = 1cm

float t_param = 1.0,increment=1.0;

Joint j1,j2,j3,j4,j5,j6; //simulated
Joint j1_ard_code,j2_ard_code,j3_ard_code,j4_ard_code,j5_ard_code,j6_ard_code; //direct from arduino
String myText="";  //received arduino signal from COM_port
float[] list=new float[9]; //6-joint angles + 3-positional variable
boolean phy_arm_avail = false;

void setup()
{
  size(1000,1000,P3D);
  if(phy_arm_avail){
  myPort=new Serial(this, "COM7", 2000000);
  myPort.bufferUntil('\n');
  }
  
  cam =new PeasyCam(this,1500);
  cam.setMinimumDistance(5);
  cam.setMaximumDistance(8000);
  

  

  
// just dh-table
// represent any VARIABLE type data with 0
// Input parameters :  || Theta  ||  Alpha  ||  r  ||  d  ||


//6DOF - surgical ROBOT
//Explanation video___ https://www.youtube.com/watch?v=wDus2EKLg3s    
  //j1 = new Joint(0,PI/2,0*cm,2*cm,'R'); 
  //j2 = new Joint(0,0,15*cm,0*cm,'R');       
  //j3 = new Joint(0,PI/2,0*cm,0*cm,'R');       
  //j4 = new Joint(0,-PI/2,0*cm,20*cm,'R');  
  //j5 = new Joint(0,PI/2,0*cm,0*cm,'R'); 
  //j6 = new Joint(0,0,0*cm,4*cm,'E');

  //if(phy_arm_avail){
  //  j1_ard_code = new Joint(0,PI/2,0*cm,2*cm,'R'); 
  //  j2_ard_code = new Joint(0,0,15*cm,0*cm,'R');       
  //  j3_ard_code = new Joint(0,PI/2,0*cm,0*cm,'R');       
  //  j4_ard_code = new Joint(0,-PI/2,0*cm,20*cm,'R');  
  //  j5_ard_code = new Joint(0,PI/2,0*cm,0*cm,'R'); 
  //  j6_ard_code = new Joint(0,0,0*cm,4*cm,'E');
  //}
  
//3DOF X,Y,Z-arm 
  //j1 = new Joint(0,PI/2,0*cm,2*cm,'R');  
  //j2 = new Joint(0,0,8*cm,0*cm,'R'); 
  //j3 = new Joint(0,0,8*cm,0*cm,'E');  

//SCARA-type robot
  //j1 = new Joint(0,0,8*cm,0*cm,'R');  //j1 itself a rot-joint and rotate around base-frame
  //j2 = new Joint(0,PI,8*cm,0*cm,'P'); //j2 itself a primatic-joint but rotate around j1-frame
  //j3 = new Joint(0,0,0*cm,0*cm,'E');  //j3 is the end-effector frame but moves prismaticly w.r.t j2-farme

//2DOF planner robot
  j1 = new Joint(0,0,10.5*cm,0*cm,'R');  
  j2 = new Joint(0,0,12*cm,0*cm,'R'); 
  j3 = new Joint(0,0,0.2*cm,0*cm,'E');  
  
  //R = rotational joint
  //P = Prismatic joint
  //E = End-effector

} 
void serialEvent(Serial myPort)
{
  if (true)
  {
    myText=myPort.readStringUntil('\n');
    myText=trim(myText);
    list=float(split(myText,','));
    //if(list[1]<10){list[1]=10;}
    
  }
}

void draw()
{  
  directionalLight(250,250,250,500,500,-1000);
  background(100);
  float[] pos = new float[3];
  float[] result_var = new float[6];
  grid();

  pathGeneration(pos);
  inverse_kine(pos[0],pos[1],pos[2],0,result_var);


  
  if(phy_arm_avail){
    //Inputs positional var from arduino COM_port
    inverse_kine(list[6],list[7],list[8],0,result_var);

  }
  
  pushMatrix();
//joint udates

////6DOF - surgical ROBOT (simulated calculation)
//  j1.updateJoint(result_var[0],'A');   // variable_type,A = Angle 
//  j2.updateJoint(result_var[1],'A');   // variable_type,A = Angle
//  j3.updateJoint(result_var[2],'A');   // variable_type,A = Angle
//  j4.updateJoint(result_var[3],'A');   // variable_type,A = Angle 
//  j5.updateJoint(result_var[4],'A');   // variable_type,A = Angle
//  j6.updateJoint(result_var[5],'A');   // variable_type,A = Angle
  
//  if(phy_arm_avail){
//    // received signal from arduino
//    j1_ard_code.updateJoint(list[0],'A');   // variable_type,A = Angle 
//    j2_ard_code.updateJoint(list[1],'A');   // variable_type,A = Angle
//    j3_ard_code.updateJoint(list[2],'A');   // variable_type,A = Angle
//    j4_ard_code.updateJoint(list[3],'A');   // variable_type,A = Angle 
//    j5_ard_code.updateJoint(list[4],'A');   // variable_type,A = Angle
//    j6_ard_code.updateJoint(list[5],'A');   // variable_type,A = Angle
//  }
  
//3DOF X,Y,Z-arm 
  //j1.updateJoint(result_var[0],'A');   // variable_type,A = Angle 
  //j2.updateJoint(result_var[1],'A');   // variable_type,A = Angle
  //j3.updateJoint(result_var[2],'A');   // variable_type,A = Angle
  
//2DOF planner robot  
  j1.updateJoint(result_var[0],'A');   // variable_type,A = Angle 
  j2.updateJoint(result_var[1],'A');   // variable_type,A = Angle
  j3.updateJoint(result_var[2],'A');   // variable_type,A = Angle

//SCARA-type robot
  //j1.updateJoint(result_var[0],'A');   // variable_type,A = Angle 
  //j2.updateJoint(result_var[1],'A');   // variable_type,A = Angle
  //j3.updateJoint(result_var[2],'D');   // variable_type,D = distance
  popMatrix();

  //if(t_param>=5.2)
  //  increment = -1;
  //else if(t_param<4)
  //  increment = 1;
  
  t_param= t_param+increment*0.005;
}



void grid()
{
  strokeWeight(1);
  int i,j;
  for(i=-width/2;i<=width/2;i+=width/cm)
    line(i,-height/2,i,height/2);
  for(j=-height/2;j<=height/2 ; j+=height/cm)
    line(-width/2,j,width/2,j); 
  strokeWeight(2); 
  line(0,-height/2,0,height/2);
  line(-height/2,0,height/2,0);


  // Base Axis....generation

  strokeWeight(5);
  stroke(219, 0, 0);      //Red = X-axis
  line(0,0,0,width/2,0,0);
  stroke(1, 212, 86);     //Green = Y-axis
  line(0,0,0,0,-height/2,0);
  stroke(30, 35, 168);    //Blue = Z-axis
  line(0,0,0,0,0,width/2);
  
  stroke(200);
  strokeWeight(1);
}

void pathGeneration(float coordinate[])
{
  if(t_param >= 4*PI)
    t_param = 0.0;
  float t;
  noFill();
  beginShape();
  for(t=0.0;t<t_param;t += 0.01)
    {
    
      //coordinate[0] = 2*cos(t)+0;
      //coordinate[1] = 2*sin(t)+12;
      //coordinate[2] = 0;  
      coordinate[0] = 2*cos(t)+20;
      coordinate[1] = 4*sin(2*t)+0;
      coordinate[2] = 0; 
      curveVertex(coordinate[0]*cm,-1*coordinate[1]*cm,coordinate[2]*cm);
      
    }
  endShape();
  fill(200);
  
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

    
//2DOF planner robot (considering end-efector rotation)  
  //assumue end-effector rotating around Z0-axis
  //  |p11  p12  0  x|
  //  |p21  p22  0  y|
  //  |0    0    1  0|
  //  |0    0    0  1|
  float p11 = 1,p12 = 0,p13 = 0;
 float p21 = 0,p22 = 1,p23 = 0;
 float p31 = 0,p32 = 0,p33 = 1;
  float r1 = 10.5, r2 = 12, r3 = 0.5;
  
  float dx = x - r3 * p11;
  float dy = y - r3 * p21;
  
  float ct2 = (dx * dx + dy * dy - r1 * r1 - r2 * r2) / (2 * r1 * r2);

  // Limit cosine to [-1, 1] range
  if (ct2 < -1 || ct2 > 1) {
    println("No solution: ct2 out of range.");
    return;
  }

  float st2_p = sqrt(1 - ct2 * ct2);
  float st2_m = -st2_p;

  // Limits (adjust as needed)
  float theta1_min = -PI/2, theta1_max = PI/2;
  float theta2_min = -PI/2, theta2_max = PI/2;
  float theta3_min = -PI/2, theta3_max = PI/2;

  float[][] all_t = new float[2][3];

  // First solution (elbow-down)
  all_t[0][1] = atan2(st2_p, ct2); // t[1] = θ2
  all_t[0][0] = atan2(-r2 * st2_p, r1 + r2 * ct2) + atan2(dy, dx); // t[0] = θ1
  all_t[0][2] = -(all_t[0][0] + all_t[0][1]) + atan2(p21, p11); // t[2] = θ3

  // Second solution (elbow-up)
  all_t[1][1] = atan2(st2_m, ct2); // t[1] = θ2
  all_t[1][0] = atan2(-r2 * st2_m, r1 + r2 * ct2) + atan2(dy, dx); // t[0] = θ1
  all_t[1][2] = -(all_t[1][0] + all_t[1][1]) + atan2(p21, p11); // t[2] = θ3

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
      println("Valid solution " + i + ":");
      println("t[0] = " + degrees(t0) + "°");
      println("t[1] = " + degrees(t1) + "°");
      println("t[2] = " + degrees(t2) + "°");
    }
  }
  
//Print Output  
      print("used :t[0]:");
      print(t[0]*180/PI);
      
      print("    t[1]:");
      print(t[1]*180/PI);
      
      print("    t[2]:");
      print(t[2]*180/PI);
      
      //print("    t[3]:");
      //print(t[3]*180/PI);
      
      //print("    t[4]:");
      //print(t[4]*180/PI);
      
      //print("    t[5]:");
      //print(t[5]*180/PI);
      
      print("        t_param______");
      print(time_stamp);
println("   ");
}
