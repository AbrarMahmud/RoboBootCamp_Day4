class Joint
{
  float theta_,alpha_,r_,d_;
  char joint_type;
  Frame f = new Frame();
  
  Joint(float theta,float alpha, float r,float d,char type)
  {
    theta_=theta;
    alpha_=alpha;
    r_=r;
    d_=d;
    
    joint_type=type; // requare only for Drawing
    //R = rotational joint
    //P = Prismatic joint
    //E = End-effector
  }
  
  
  void drawJoint()
  {
     noStroke();
     if(joint_type == 'R')
        sphere(10);
     else if(joint_type == 'P')
        box(15,15,25);
     else
        sphere(5);
     
     f.updateFrame();
     stroke(200); 

       
  }
  void updateJoint(float var ,char varable_type) 
  {
    if(varable_type == 'A') //Angular variable(theta)
    {
      
      //strokeWeight(10);
      //line(0,0,0,r_,0,0);//r_n
      //translate(r_,0,d_);
      //line(0,0,0,0,0,-1*d_);//d_n
      //strokeWeight(1);
      //rotateZ(-1*(theta_+var));
      //rotateX(alpha_);
      
      //drawJoint();     
          
      rotateZ(-1*(theta_+var));
      strokeWeight(10);
      line(0,0,0,r_,0,0);//r_n
      translate(r_,0,d_);
      line(0,0,0,0,0,-1*d_);//d_n
      strokeWeight(1);
      rotateX(alpha_);
      
      drawJoint();    
   
                     
 
    }
    else if(varable_type == 'D') //Distance variable(d)
    {
      
      rotateX(alpha_);
      rotateZ(theta_);
      strokeWeight(10);
      line(0,0,0,r_,0,0);//r_n
      translate(r_,0,d_+var);
      line(0,0,0,0,0,-1*(d_+var));//d_n
      strokeWeight(1);
      
      drawJoint();
    }
  }
}
