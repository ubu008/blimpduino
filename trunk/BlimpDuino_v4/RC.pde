    /**************************************************************
    * Function to initialize the R/C mode.. 
    ***************************************************************/
void Init_RC(void)
{
  
  DDRB&= ~(0x03 << 6); //We change only bits 6 and 7 to 0 (input).
  
  /*Activating External interrupts for pins 6 and 7 of port B*/ 
  PCICR= 0x00|(1<<PCIE0); //Enable interrupt, read page 70 of Atmega168 datasheet to know more .
  PCMSK0= 0x00|(1<<PCINT7)|(1<<PCINT6); //Interrupt mask, read page 71 of datasheet to know more .
  
  /*Activating Timer2, we use it to count only useconds, is the same timer used to pulse the servo, can't be used at the same time*/
  TCCR1A=0x00;
  TCCR1B = 0x00 |(1 << CS11)|(1<<WGM12) ; // using prescaler 8 =), page 134
  OCR1A  = 10; //prescaler 8/8mhz= 1 us resolution, so the interrupt will be execute every 10us...                       
  TIMSK1 |=(1 << OCIE1A); //See page 136, Enabling timer interrupt  
  
  sei(); //Enabling all interrupts, this interrupt is at the end of the code. The functions are called "ISR(TIMER1_COMPA_vect)" and "ISR(PCINT0_vect)"
}
    /**************************************************************
    * Function to know if there is anything attached to the board.
    ***************************************************************/
byte RC_detector(void)
{
float average_ch1=0;

delay(1000); //wait a second for RC receivers that take a second to boot up
for(int c=0; c<=10; c++)
{
   average_ch1=(average_ch1*.50)+(ch1_position*.50);//Just average the input ch1
}

  if(average_ch1>1)//If the average is greater than 1, a RC receiver is attached to the board.. 
  {
  Serial.println ("Keep the sticks centered for a few seconds");  
  fun_LEDs(3, 15, 100); //Blink the LED to let you know when you need to keep the stick centered....   
  rc_ch1_central=ch1_position;//Then it stores the central position of the sticks
  rc_ch2_central=ch2_position;//You must keep the sticks centered during this part... 
  return 1; 
  }
return 0;
}
    /**************************************************************
    * Function to mix the thruster with two input channels..
    ***************************************************************/
    
void motor_mixing(int ch1, int ch2)
{
 int motorr=0;
 int motorl=0;
 
 if(abs(ch1_position-rc_ch1_central) >MAX_RC)//Auto adjusting MAX range of the Remote Control =)
 {
  MAX_RC=abs(ch1_position-rc_ch1_central); 
 }
  if(abs(ch2_position-rc_ch2_central) >MAX_RC)
 {
  MAX_RC=abs(ch2_position-rc_ch2_central); 
 }

   motorl=ch1-rc_ch1_central;
  if(motorl<0)motorl=0;

  motorr=-ch1+rc_ch1_central;
  if(motorr<0) motorr=0; 


  motorr=(motorr+(-ch2+rc_ch2_central));
  motorr=(motorr*(MAX_motor+40))/MAX_RC;
  
  motorl=(motorl+(-ch2+rc_ch2_central));
  motorl=(motorl*(MAX_motor+40))/MAX_RC;
  
  if(motorr <= -5)
  {
    motorRight(abs(motorr),REVERSE);
  }
  else{
    if(motorr >= 5)
    {
    motorRight(abs(motorr),FORW);
    }
    else
    {
     motorRight(0,0); 
    }
  }
/*************************************************/

    if(motorl <= -5)
  {
    motorLeft(abs(motorl),REVERSE);
  }
  else{
    if(motorl >= 5)
    {
    motorLeft(abs(motorl),FORW);
    }
    else
    {
     motorLeft(0,0); 
    }
  }

   
}
    /**************************************************************
    * Timer interrupt function
    ***************************************************************/
    
ISR(TIMER1_COMPA_vect)//This is a timer interrupt, executed every 10us 
{
  clock1++;//Just a counter that increments every 10us... 
  clock2++;
}

    /**************************************************************
    * External interrupt function 
    ***************************************************************/
   
ISR(PCINT0_vect)//This interrupt is executed every time the PIN 6 and 7 of portB changes state. 
{
  /****This part is for channel 1****/
  if((read_ch1)&&(ch1_state_flag==0)) //if read_ch1 goes high, and if the flag is equal to 0 
  {
    clock1=0; //restart the clock 
    ch1_state_flag=1;//and change the flag to avoid undesired restart of the clock1.. 
  }
  if((read_ch1==0)&&(ch1_state_flag==1))//check if the pin is low, and also check if the flag was activated...
  {
    ch1_position=clock1; //pass the value of clock1 to ch1_position... 
    ch1_state_flag=0; //Reset the flag.... and that's all... 
  }
  /****This part is for channel 2, the same as above***/
    if((read_ch2)&&(ch2_state_flag==0))
  {
    clock2=0;
    ch2_state_flag=2;
  }
  if((read_ch2==0)&&(ch2_state_flag==2))
  {
    ch2_position=clock2;
    ch2_state_flag=0;
  }
}
