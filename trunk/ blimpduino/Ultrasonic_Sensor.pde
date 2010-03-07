    /**************************************************************
    * 
    ***************************************************************/
void init_ultrasonic(void)
{
  #ifdef ping_mode == 0
  pinMode(ping_pw, INPUT);
  #endif
  pinMode(ping_rx, OUTPUT);
  digitalWrite(ping_rx, HIGH);//enabling ping sensor.
  //delay(1000);
} 
       /**************************************************************
    * 
    ***************************************************************/
unsigned int raw_ultrasonic(void) //Return the altitude exactly in the moment you request it... 
{
  unsigned int pulse_length=0; //declaring variable
  #ifdef ping_mode == 1
  static float average_pulse;
    //pulse_length=analogRead(2);
    average_pulse=(((float)average_pulse*.50)+(float)((float)analogRead(2)*.50));
    pulse_length=average_pulse;
  #else
    //digitalWrite(ping_rx, HIGH); 
    //delay(37);
    //smart_delay(40);
    //pulse_length=pulseIn(ping_pw,HIGH,200000); //pulseIn function to mesure the lenght of the pulse
    //digitalWrite(ping_rx, LOW); 
    //pulse_length=(pulse_length/147); 
  #endif
    
    return pulse_length;
}
         /**************************************************************
    * 
    ***************************************************************/
unsigned int pulse_ultrasonic(unsigned int refresh_rate)//Returns the altitude with specified refresh rate in milliseconds
{
  if((millis() - timer_ultrasonic) > refresh_rate)
  {
    altitude=raw_ultrasonic();
    timer_ultrasonic=millis();
  }  
  return altitude;
}  

