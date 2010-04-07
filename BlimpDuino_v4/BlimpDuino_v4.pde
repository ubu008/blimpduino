#include <avr/interrupt.h>
#include <avr/io.h> 
/**************************************************************
 * By Jordi Munoz ^ DIYdrones.com >> Chris Anderson 
 *Beta 1.2 [April 2010]
 * ToDo: 
 *
 *Simplify PID system for everyone =P
 *Add radio modem, and ground station... 
 ***************************************************************/
#define minSpeed 0 //min speed of differential thrusters
#define maxSpeed 50 // max speed of differential thrusters
#define anti_static_friction_breaker 80 //This value is power that will be applied to the motor in the first second to break start it.. 
// (like the capacitor used to start big electric motors)

#define FORW HIGH //defining FORWARD as HIGH
#define REVERSE LOW
#define UP HIGH
#define DOWN LOW

#define ping_rx 15 //OK
#define ping_pw 16 //OK

#define led_north 12 //OK
#define led_east 17 //OK
#define led_south 11 //OK
#define led_west 13 //OK
#define led_delay 125 
#define led_cycles 1 

#define IR_FRONT 8 //define IR pins
#define IR_BACK 7 //
#define IR_RIGHT 6
#define IR_LEFT 9 //

#define max16 2000 //max position in useconds
#define min16 1000 //min position useconds

#define MAX_motor 45 //Maximun speed of the motors
#define MIN_motor -45

#define MAX_vect 135 //Maximun pitch of the vector in navigation mode... 
#define MIN_vect 45

//Motor power adjust for navigation mode

#define forward_motor_right 40 //The amount of thrust applied to the right motor when traveling forward, you need to adjust this value if the blimp doesn't fly straight 
#define forward_motor_left 40 //Left motor, same as above..

#define turn_motors 50 // The total thrust applied to the motor when you are making turns (right or left)... Try to put a low value to avoid blimp oscillations.. 

#define PID_dt 100 //The refresh rate in milliseconds of the PID system... 
#define PID_dt_seconds PID_dt/1000 

#define low_battery_limit 6000 //6.0volts

#define read_ch2 (PINB & (1 << 6))//Port B, pin 6 as channel 1;
#define read_ch1 (PINB & (1 << 7))//Port B, pin 7 as channel 2;

#define ping_mode 1 // 0= digital mode, 1=analog mode. 

//Altitude hold dead zone variables 
//remember we only use one motor at a time. One pushes up and the other pushes down 
#define up_dead_zone 30 // We use a greater value here because the propeller is less efficient in this direction
#define down_dead_zone 20

//PID gains Kp, Ki , Kd
//Holding altitude mode only gains (no navigation).
#define alt_Kp 3
#define alt_Ki .1
#define alt_Kd 200
//Navigation mode altitude gains.
#define nav_Kp 5
#define nav_Ki 0
#define nav_Kd 0

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
int startup_altitude=30;
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
unsigned int clock1=0;//Clock to count the pulse length
unsigned int clock2=0;
int ch1_position=0;  //This variable stores the raw position of channel 1
int ch2_position=0; //The same as above but for ch2. 
byte ch1_state_flag=0;//Something used to messure the pulse length... 
byte ch2_state_flag=0;
int rc_ch1_central; //The central position of the control
int rc_ch2_central;
int MAX_RC=20;   //Normally is 40, but is auto adjustable... 



float constrain_float(float value, float max, float min);

void setup()
{
  Serial.begin(38400);
  
  Serial.print("Starting System... ");
  Init_RC();
  Init_LEDs(); 
  Init_IRs();
  init_ultrasonic();   
  Init_motors();
  //test_servos(2000);
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
    startup_altitude=analogRead(2); //Reading the startup altitude, which the system will try to maintain
    Serial.println(startup_altitude);
    delay(1000); // wait one second
    Serial.println("Checking RC connection...");
    if(RC_detector()==1)//RC_detector return 1 if it detects a receiver attached to the board. 
    {
      fun_LEDs(2, 5, 200); //Pulse the leds in a funny diferent way.. 
      Serial.println("RC detected, activating PWM settings...");
      system_state=3; 
    }
   else
   {
      Serial.println("No RC detected, starting normally...");
      Serial.println("Starting timer1 Fast PWM, to pulse vector servo...");
      Init_servo();//Starting servo services...AKA timer 1 settings =).. 
      Serial.println("Testing Vector Servo... ");
      test_servos(1000); //Module to test the vector servo
      system_state=1; //Now jump to system state 1.. 
   }
    delay(400);
    //system_state=5;//temporal
    break;
    /**************************************************************
     * System State 1: This state is only to hold altitude
     ***************************************************************/
  case 1: //Hold altitude only (enters only when no beacon has been detected)
    read_irs();//Capture IR packets..
    analyse_irs(200);//after 500 millisecon will analyse the packets captured
    nav_pilot(1);// Tell the autopilot to be in the autopilot state 0, that means hold altitude only
    autopilot_state(); //Verfy the autopilot state, if it changes, it will restart the PID and adjust the variables, etc.
    system_refresh(150); //This is the system refresh loop, which runs every 250 miliseconds
    break;
    /**************************************************************
     *System State 2: This state is to hold altitude and navigate to the beacon
     ***************************************************************/
  case 2: //Navigation and hold altitude
    read_irs(); //Capture IR packets..
    analyse_irs(200); //after 500 millisecon will analyse the packets captured
    nav_pilot(0); // Tell the autopilot to be in the autopilot state 1. That means hold altitude and navigate
    autopilot_state(); //Verfy the autopilot state, if it changes, it will restart the PID and adjust the variables, etc.
    system_refresh(150); //This is the system refresh loop, which runs every 250 miliseconds
    break;
    /**************************************************************
     * System State 3: RC mode or manual mode.....
     ***************************************************************/
  case 3:
    motor_mixing(ch1_position, ch2_position);
    system_refresh(150); //This is the system refresh loop, which runs every 250 miliseconds
    break;
    /**************************************************************
     * System State 4: Will only enter if we are running out of battery, just to protect your "made in china" $11.99 lipo... 
     ***************************************************************/
  case 4:
    motorLeft(0, FORW); //So I shut down the motors... etc. 
    motorRight(0, FORW); 

    fun_LEDs(2, 5, 100);
    Serial.print("LOW BATTERY!: "); 
    Serial.println((int)battery_voltage);
    Battery_protection();

    break;

   case 5: //The test state
     read_irs();//Capture IR packets..
     analyse_irs(200);//after 500 milliseconds will analyse the packets captured
     delay(1);
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
    Serial.print((int)raw_ultrasonic());
    Serial.print (" [");
    Serial.print (startup_altitude);
    Serial.print ("]");
    Serial.print ("\t");
    Serial.print ("\r\n");

  }   
}
/**************************************************************
 * Function to analyse the battery level and trigger the safe mode, (to avoid killing the lipo battery)
 ***************************************************************/
void Battery_protection(void)
{

  average_battery=(((float)average_battery*.99)+(float)((float)analogRead(0)*.01));// I call it a dynamic average

  battery_voltage=(float)((float)average_battery*(float)4.887586)*(float)2;//converting values to millivolts.......


  if(battery_voltage < low_battery_limit)//If the battery voltage is less than the low_battery_limit, change the states and turn off the LED's... 
  {
    off_leds(); // This is not needed.. 
    system_state=4;

  }   
}

