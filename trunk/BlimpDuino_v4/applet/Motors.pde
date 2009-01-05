void Init_motors(void)
{
  pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(4,OUTPUT);
}

/**************************************************************
 * Sub-rutine to control right motor= motorRight(Speed, direction);
 ***************************************************************/
void motorRight(byte PWM, boolean dir)
{
  byte oldPWM;
  if(PWM==0) //If Speed = 0, shut down the motors
  {
    digitalWrite(3, LOW); 
    digitalWrite(2, LOW);
    static_friction_breaker_right=0; //And restart the state of the motor to indicate is stop.. 
  }  
  else{
    restart_right:
    
    PWM= constrain(PWM, minSpeed, maxSpeed); //Change highers values to the specified ranges
    
    if(static_friction_breaker_right == 0)//Verify if th static friction is 0, if the mean the the motor was stop so pulse harder
    {
     oldPWM=PWM; //Storing the original PWM value... 
     PWM=anti_static_friction_breaker; //This value is defined at the begining, pulsing the motor
    }  
    if(dir == true) //If direction is 1 or true or HIGH, the motor will go forward
    {
      digitalWrite(2, LOW);
      analogWrite(3, PWM); 
    }
    if(dir == false) //If direction is 0 or false or LOW, the motor will go back
    {
      digitalWrite(2, HIGH); 
      analogWrite(3, 250-PWM); //Trick to regulated speed in reverse direction
    }
   if(static_friction_breaker_right == 0)//The verify again it the motor was stoped, if yes
    {
     smart_delay(50); //Delay for 10 milliseconds
     static_friction_breaker_right=1; //and then change th state to 1, to indicate to the system that the motor is running
     PWM=oldPWM; //Restoring desired speed...
     goto restart_right; //Jumping to "restart_right:" defined above... 
    }  
  }
}  

/**************************************************************
 * Sub-rutine to control left motor
 ***************************************************************/
void motorLeft(byte PWM, boolean dir)
{
  byte oldPWM;
  if(PWM==0)//If Speed = 0, shut down the motor
  {
    digitalWrite(4, LOW);
    digitalWrite(5, LOW); 
    static_friction_breaker_left=0; //And restart the state of the motor to indicate is stop.. 
  }  
  else{
    restart_left:
    PWM= constrain(PWM, minSpeed, maxSpeed);//Change highers values to the specified ranges
    if(static_friction_breaker_left == 0)//Verify if th static friction is 0, if the mean the the motor was stop so pulse harder
    {
     oldPWM=PWM; //Storing the original PWM value...
     PWM=anti_static_friction_breaker; //This value is defined at the begining  
    }  
    if(dir == true)//If direction is 1 or true or HIGH, the motor will go forward
    {

      digitalWrite(4, LOW);
      analogWrite(5, PWM); 
    }
    if(dir == false)//If direction is 0 or false or LOW, the motor will go back
    {
      digitalWrite(4, HIGH); 
      analogWrite(5, 250-PWM);
    }
    if(static_friction_breaker_left == 0)
    {
     smart_delay(50);//Change highers values to the specified ranges
     static_friction_breaker_left=1; //and then change th state to 1, to indicate to the system that the motor is running
     PWM=oldPWM; //Restoring desired speed...
     goto restart_left; //Jumping to "restart_left:" defined above... 
    }  
  }
}
