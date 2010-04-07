/**************************************************************
 * 
 ***************************************************************/
void Init_IRs(void)
{
  pinMode(IR_FRONT, INPUT);  
  pinMode(IR_RIGHT, INPUT);
  pinMode(IR_BACK, INPUT);
  pinMode(IR_LEFT, INPUT); 
}
/**************************************************************
 * 
 ***************************************************************/
void read_irs(void)
{

  for(int i=0; i<=1000; i++) //Loop that will read the IR sensors 999 times
  {
    //IR_cnt[0] = (digitalRead(IR_FRONT));
     IR_cnt[0] += 0x01^(PINB & 0x01); //FRONT
     IR_cnt[1] += 0x01^((PIND & 0x40)>>6); //RIGHT
     IR_cnt[2] += 0x01^((PIND & 0x80)>>7); //BACK
     IR_cnt[3] += 0x01^((PINB & 0x02)>>1); //LEFT
  }
  
  /* Old routine, does the same but a lot less efficient   
  if(digitalRead(IR_FRONT) == LOW) //If the north IR sensor is currently low then 
  {
    IR_cnt[0]++; //Increase this value by one
  }  
  if(digitalRead(IR_RIGHT) == LOW)//If the east IR sensor is currently low then 
  {
    IR_cnt[1]++; //Increase this value by one
  } 
  if(digitalRead(IR_BACK) == LOW)//If the south IR sensor is currently low then 
  {
    IR_cnt[2]++; //Increase this value by one
  }  
  if(digitalRead(IR_LEFT) == LOW)//If the west IR sensor is currently low then 
  { 
    IR_cnt[3]++;//Increase this value by one
  } 
  */
}
/**************************************************************
 * 
 ***************************************************************/
byte analyse_irs(unsigned int refresh_rate)//Routine to know the direction of the beacon.. 
{
  if((millis() - timer_irs) > refresh_rate)
  {
     /*
     for(byte c=0; c<4; c++)
     {
      Serial.print((long)IR_cnt[c]); 
      Serial.print(","); 
     }
     Serial.println(""); */
    if((IR_cnt[0]+IR_cnt[1]+IR_cnt[2]+IR_cnt[3]) <= 5)//If the IR reading is leass than 10, that means no beacon present, soo.. 
    {
      off_leds; //We turn off all the LEDs
      ir_compare=5;  //We set ir_compare to 5 (mean nothing)
      system_state=1; //We set the autpilot to hold altitude only... 
    }
    else
    {
      system_state=2; //That means that system state is 2
      ir_compare=0;//Restart the variable to zero

      if(IR_cnt[1]>IR_cnt[0]) //If counter 1 (east IR sensor) is greater than the counter 0 (north ir sensor)
      {
        ir_compare|=0x01; //Set the bit 1 to high   (0b00000001)
      } //If not, leave it at zero (0b0000000). 

      if(IR_cnt[3]>IR_cnt[2]) //If counter 3 (west IR sensor) is greater than counter 2 (south ir sensor)..  
      {
        ir_compare|=0x02; // Set the second bit of the variable to HIGH. Example: 0b00000010
      } //If not leave it at zero.. 0b00000000

      if(IR_cnt[(ir_compare&0x01)] > IR_cnt[((ir_compare>>1)+2)])//Now compare the to the greatest counter. Either counter 0 or 1, vs. 2 or 3, 
      {
        ir_compare&=0x01;  //If yes, store the position of the > value, but eliminate the second bit and leave the first bit untouched.. 
      }
      else
      {
        ir_compare=((ir_compare>>1)+2);  //Eliminate the first bit and add two...
      }

      off_leds();//Shut off all the leds

        switch(ir_compare)//Turn on the LED pointing to the beacon. 
      {
      case 0:
        digitalWrite(led_north,HIGH);
        break; 
      case 1:
        digitalWrite(led_east,HIGH);
        break;
      case 2:
        digitalWrite(led_south,HIGH);
        break;
      case 3:
        digitalWrite(led_west,HIGH);
        break;   
      }

      IR_cnt[0]=0;//Restart counters
      IR_cnt[1]=0;
      IR_cnt[2]=0;
      IR_cnt[3]=0;
      timer_irs=millis();
    }

    return ir_compare;
  }
}
/**************************************************************
 * The smart delay will improve performace of the IR beacons. Instead of wasting CPU time 
 with a normal delay loop, this function will delay x time in miliseconds, using that time to capture more IR 
 packets from the beacon.. =) 
 ***************************************************************/
void smart_delay(unsigned int time)
{
  unsigned long start_time= millis();

  while((millis() - start_time) < time)
  {
    read_irs();  
  }   
}
