    /**************************************************************
    * 
    ***************************************************************/
void init_ultrasonic(void)
{
  digitalWrite(ping_rx, HIGH);
  digitalWrite(ping_rx, LOW); //Activating MCU internal pull-down resistor
  pinMode(ping_rx, OUTPUT); 
  pinMode(ping_pw, INPUT); 

  digitalWrite(ping_rx, HIGH);
  delay(1000);
} 
       /**************************************************************
    * 
    ***************************************************************/
unsigned int raw_ultrasonic(void) //Return the altitude exactly in the moment you request it... 
{
  unsigned int pulse_length=0; //declaring variable
    digitalWrite(ping_rx, HIGH); 
    //delay(37);
    smart_delay(40);
    pulse_length=pulseIn(ping_pw,HIGH,200000); //pulseIn function to mesure the lenght of the pulse
    digitalWrite(ping_rx, LOW); 
    pulse_length=(pulse_length/147); 
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

