#include <avr/interrupt.h>
#include <avr/io.h> 
/**************************************************************
 * By Jordi Muñoz ^ DIYdrones.com >> Chris Anderon 
 *Version 4.3 [Octuber/26/2008]
 *New:
 * -Added Manual mode function
 * -Fixed and improved the anti static friction of the motors 
 * -Added function "fun_LEDs()" with diferent LED patterns...
 * -Many, many bugs fixed... And still more =P
 * -And i don't remember what else.. 
 *Version 4.2.1
 * New:
 * -Added anti static friction function
 * -Support for PWM by hardware (to control servo)
 * -Better low battery panic state.. 
 * -And i don't remember.. 
 * -New smart_delay() function (it will capture packets when you are delaying, just to improve performance).. 
 * ToDo: 
 *
 *Simplify PID system for everyone =P
 *Add radio modem, and ground station... 
 ***************************************************************/
#define minSpeed 0 //min speed of diferential thrusters
#define maxSpeed 150 // max speed of diferential thrusters
#define FORW HIGH //defining FORWARD as HIGH, the same with above
#define REVERSE LOW
#define UP HIGH
#define DOWN LOW

#define ping_rx 15 //OK
#define ping_pw 16 //OK

#define led_north 12 //OK
#define led_east 17 //OK
#define led_south 11 //OK
#define led_west 13 //OK
#define led_delay 150 
#define led_cycles 2

#define IR_FRONT 8 //int0
#define IR_BACK 7 //
#define IR_RIGHT 6
#define IR_LEFT 9 //int1

#define max16 2100/16 //max position
#define min16 1000/16 //min position 

#define MAX_motor 45
#define MIN_motor -45

#define MAX_vect 135
#define MIN_vect 45

#define PID_dt 100 //The refresh rate in milliseconds of the PID system... 
#define PID_dt_seconds PID_dt/1000 

#define low_battery_limit 6000 //6.0volts

#define read_ch1 (PINB & (1 << 6))//Port B, pin 6 as channel 1;
#define read_ch2 (PINB & (1 << 7))//Port B, pin 7 as channel 2;


/**************************************************************
 * 
 ***************************************************************/

/* UltraSonic stuff   */
unsigned long timer_ultrasonic=0;
unsigned int altitude=0;
float average_altitude=0;

/*Infrared stuff*/
unsigned long timer_irs=0;
unsigned int IR_cnt[4];
byte ir_compare=0;

/*Servo stuff */
unsigned long timer_servo=0;
byte servo_position=0;

/*Motor Stuff */
byte static_friction_breaker_right=0;
byte static_friction_breaker_left=0;

/*System variables */
byte system_state=0;
float average_battery=1023;
float battery_voltage=6000;
byte last_state=5;
int startup_altitude=40;
unsigned long system_timer=0;

/*AutoPilot Stuff */
float kp=20.0;
float ki=5.9;
float kd=0.1;
float PID_I_altitude=0;
float PID_D_altitude=0;
float PID_average_altitude=0;
float PID_prev_error_altitude=0;
float PID_output_altitude=0;
float PID_error=0;
float PID_P=0;
//float PID_current_Point=0;;
unsigned long timerPID_altitude=0;

//RC mode stuff
unsigned int clock1=0;//Clock to count the pulse lenght
unsigned int clock2=0;
int ch1_position=0;  //This variable store the raw position of the channel 1
int ch2_position=0; //The same as above but for ch2. 
byte ch1_state_flag=0;//Something used to messure the pulse lenght... 
byte ch2_state_flag=0;
int rc_ch1_central; //The central position of the control
int rc_ch2_central;
int MAX_RC=20;   //Normally is 40, but is auto adjustable... 



float constrain_float(float value, float max, float min);

void setup()
{
  Serial.begin(19200);
  
  Serial.print("Starting System... ");
  Init_RC();
  Init_LEDs(); 
  Init_IRs();
  init_ultrasonic();   
  Init_motors();
  
  Serial.println("OK!!!");
  
  average_battery=analogRead(0);//refreshing..     
  Serial.print("Battery Analog: ");
  Serial.println((int)average_battery);

}
void loop()
{ 
  
  switch(system_state)//This is the system state switch.. 
  {
    /**************************************************************
     * 
     ***************************************************************/
  case 0:
    
    Serial.print("Reading startup altitude... ");
    //startup_altitude=raw_ultrasonic(); //Reading the startup altitude, same that the system will try to maintain
    Serial.println(startup_altitude);
    Serial.println("Checking RC connection...");
    if(RC_detector()==1)//RC_detector return 1 if it detects a reciver attached to the board. 
    {
      fun_LEDs(2, 5, 200); //Pulse the leds in a funny diferent way.. 
      Serial.println("RC detected, activating PWM settings...");
      system_state=3; 
    }
   else
   {
      Serial.println("No RC detected, starting normally...");
      Serial.println("Starting timer1 Fast PWM, to pulse vector servo...");
      Init_servo();//Starting servo services... i mean timer 1 setting =).. 
      Serial.println("Testing Vector Servo... ");
      test_servos(1000); //Module to test the vector servo
      system_state=1; //Now jump to system state 1.. 
   }
    delay(400);
    break;
    /**************************************************************
     * System State 1: This state is only to hold altitude
     ***************************************************************/
  case 1: //Hold altitude only (enters only when no beacon has been detected)
    read_irs();//Capture ir packets..
    analyse_irs(500);//after 500 millisecon will analyse the packets captured
    nav_pilot(1);//Indicate to the autopilot to be in the autopilot state 0, that means hold altitude only
    autopilot_state(); //Verfy the autopilot state, if it changes, it will restart the PID and adjust the variables, etc.
    system_refresh(250); //This is the system refresh loop, that runs every 250 miliseconds
    break;
    /**************************************************************
     *System State 2: This state is to hold altitude and navigate to the beacon
     ***************************************************************/
  case 2: //Navigation and hold altitude
    read_irs(); //Capture ir packets..
    analyse_irs(500); //after 500 millisecon will analyse the packets captured
    nav_pilot(0); //Indicate to the autopilot to be in the autopilot state 1 that means hold altitude and navigate
    autopilot_state(); //Verfy the autopilot state, if it changes, it will restart the PID and adjust the variables, etc.
    system_refresh(250); //This is the system refresh loop, that runs every 250 miliseconds
    break;
    /**************************************************************
     * System State 3: RC mode or manual mode.....
     ***************************************************************/
  case 3:
    motor_mixing(ch1_position, ch2_position);
    system_refresh(250); //This is the system refresh loop, that runs every 250 miliseconds
    break;
    /**************************************************************
     * System State 4: Will only enter if we are running out of battery, just to protect your "made in china" $11.99 lipo... 
     ***************************************************************/
  case 4:
    motorLeft(0, FORW); //So i shut down the motors... etc. 
    motorRight(0, FORW); 

    fun_LEDs(2, 5, 100);
    Serial.print("LOW BATERRY!!!: "); 
    Serial.println((int)battery_voltage);
    Battery_protection();

    break;
    /**************************************************************
     * 
     ***************************************************************/
  default:
    system_state=0;
    break;
  }
}//END OF THE MAIN LOOP HERE!!!!!!



/**************************************************************
 * Function to process system services, like battery level and print some parameters. 
 ***************************************************************/
void system_refresh(unsigned int refresh_rate)
{
  if(millis() - system_timer > refresh_rate) 
  {
    system_timer=millis();
    Battery_protection(); //Function locate bellow this to check the battery power... 

    Serial.print("Batt: "); //Prints importante values for debugging... 
    Serial.print((int)battery_voltage);
    Serial.print ("\t");
    Serial.print(" Alt: ");
    Serial.print((int)average_altitude);
    Serial.print (" [");
    Serial.print (startup_altitude);
    Serial.print ("]");
    Serial.print ("\t");
    Serial.print ("\r\n");

  }   
}
/**************************************************************
 * Function tu analyse the battery level and trigger the safe mode, (to void burning the lipo)
 ***************************************************************/
void Battery_protection(void)
{

  average_battery=(((float)average_battery*.99)+(float)((float)analogRead(0)*.01));//i called it dynamic average

  battery_voltage=(float)((float)average_battery*(float)4.887586)*(float)2;//converting values to millivolts.......


  if(battery_voltage < low_battery_limit)//If the battery voltage is less than the low_battery_limit, change the states\ and turn off the LED's... 
  {
    off_leds(); // This is no needed.. 
    system_state=4;

  }   
}

