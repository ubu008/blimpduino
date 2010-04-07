/**************************************************************
 * 
 ***************************************************************/
void Init_LEDs(void)
{
  pinMode(led_north, OUTPUT);  
  pinMode(led_east, OUTPUT);
  pinMode(led_south, OUTPUT);
  pinMode(led_west, OUTPUT);

  fun_LEDs(0, led_cycles, led_delay);
  
  
}
/**************************************************************
 * 
 ***************************************************************/
void off_leds(void)//Shutdown all the LEDs. 
{
  digitalWrite(led_north,LOW);
  digitalWrite(led_east,LOW);
  digitalWrite(led_south,LOW);
  digitalWrite(led_west,LOW);
}
/**************************************************************
 * A function to choose and change the behavior, you can add your custom patterns  
 ***************************************************************/
void fun_LEDs(byte mode, byte cycles, unsigned int t_delay)
{
  switch(mode)
  {
  case 0:
    for(int x=0; x<=cycles; x++)//System to play with the LEDs
    {
      digitalWrite(led_north,LOW);
      digitalWrite(led_east,HIGH);
      delay(t_delay);
      digitalWrite(led_east,LOW);
      digitalWrite(led_south,HIGH);
      delay(t_delay);
      digitalWrite(led_south,LOW);
      digitalWrite(led_west,HIGH);
      delay(t_delay);
      digitalWrite(led_west,LOW);
      digitalWrite(led_north,HIGH);
      delay(t_delay);
    }
    digitalWrite(led_north,LOW);
    break; 
 /************************************************************/   
  case 1:
    for(int x=0; x<=cycles; x++)//System to play with the LEDs
    {
      digitalWrite(led_north,HIGH); //And I start blinking the LED like crazy..
      digitalWrite(led_east,HIGH);
      digitalWrite(led_south,HIGH);
      digitalWrite(led_west,HIGH);
      delay(t_delay);
      digitalWrite(led_north,LOW);
      digitalWrite(led_east,LOW);
      digitalWrite(led_south,LOW);
      digitalWrite(led_west,LOW);
      delay(t_delay);
    }
    break;
 /************************************************************/   
  case 2:
    for(int x=0; x<=cycles; x++)//System to play with the LEDs
    {
      digitalWrite(led_north,LOW);
      digitalWrite(led_south,LOW);
      digitalWrite(led_east,HIGH);
      digitalWrite(led_west,HIGH);
      delay(t_delay);
      digitalWrite(led_north,HIGH);
      digitalWrite(led_south,HIGH);
      digitalWrite(led_east,LOW);
      digitalWrite(led_west,LOW);
      delay(t_delay);

    }
    digitalWrite(led_east,LOW);
    digitalWrite(led_west,LOW);
    digitalWrite(led_north,LOW);
    digitalWrite(led_south,LOW);
    break;
/************************************************************/    
      case 3:
    for(int x=0; x<=cycles; x++)//This just blinks the north LED...
    {
      digitalWrite(led_north,HIGH);
      delay(t_delay);
      digitalWrite(led_north,LOW);
      delay(t_delay);
    }

    break;


  }
}
