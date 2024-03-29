    /**************************************************************
    * 
    ***************************************************************/
void Init_servo(void)//This part will configure the PWM to control the servo 100% by hardware, and not waste CPU time.. 
{   
    digitalWrite(10,LOW);
    pinMode(10,OUTPUT);
    /*Timer 1 settings for fast PWM. It's the same timer used to read the PWM of the remote control, so they can't be used at the same time*/
    TCCR1A =((1<<WGM11)|(1<<COM1B1));
    TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11);    
    OCR1B = 3000;
    ICR1 = 20000; 
}
    /**************************************************************
    * 
    ***************************************************************/
void pulse_servo(int angle)//Will convert the angle to the equivalent servo position... 
{
  constrain(angle,180,0);
  //angle=180-angle;//invert the vectoring
 OCR1B=((((long)angle*(long)(max16-min16))/180L)+(long)min16);// 180L is the same as 180, but the L means LONG.. So the compiler will take it as a LONG variable... you can also say "(long)2". 
}
     /**************************************************************
    * 
    ***************************************************************/


void test_servos(unsigned int time)//Routine to test the servo at the begining (max, min and then central position)
{
  pulse_servo(180);
  delay(time);

  pulse_servo(90);
  delay(time);

  pulse_servo(0);
  delay(time);  
}  
  
  
