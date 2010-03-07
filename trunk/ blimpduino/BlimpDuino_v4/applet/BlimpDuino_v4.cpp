#include <avr/interrupt.h>
#include <avr/io.h> 
/**************************************************************
 * By Jordi Mu\u00f1oz ^ DIYdrones.com >> Chris Anderson 
 *Beta 1.1 [Janury/01/09]
 *Solve some minor bugs..
 *Add more comments
 *Added more definitions to change settings of the system without going so deeply into the source code.
 *Beta 1.0 [Octuber/26/2008]
 *New:
 * -Added Manual mode function
 * -Fixed and improved the anti static friction of the motors 
 * -Added function "fun_LEDs()" with diferent LED patterns...
 * -Many, many bugs fixed... And still more =P
 * -And i don't remember what else.. 
 *Version Beta 0
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
#define maxSpeed 50 // max speed of diferential thrusters
#define anti_static_friction_breaker 80 //This value is power that will be applyed to the motor the first second to break start it.. 
//Is like the capacitor used to start big electric motors. ?

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

#define IR_FRONT 8 //definind IR pins
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

#define forward_motor_right 40 //The amound thrust applied to the motor when traveling forward, you need to adjust this values if the blimp is not well aligned when it flys
#define forward_motor_left 40 //motor left, same as above..

#define turn_motors 50 // The total thrust applied to the motor when you are making turns (right or left)... Try to put a low value, to void blimp oscillations.. 

#define PID_dt 100 //The refresh rate in milliseconds of the PID system... 
#define PID_dt_seconds PID_dt/1000 

#define low_battery_limit 6000 //6.0volts

#define read_ch2 (PINB & (1 << 6))//Port B, pin 6 as channel 1;
#define read_ch1 (PINB & (1 << 7))//Port B, pin 7 as channel 2;

#define ping_mode 1 // 0= digital mode, 1=analog mode. 

//Altitude hold dead zone variables 
//remember we only use one motor at the time, one is pushing up and the other pulling down only... 
#define up_dead_zone 30 //i had to put greater value because the propeller is less efficient in this direction
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
#include "WProgram.h"
void setup();
void loop();
void system_refresh(unsigned int refresh_rate);
void Battery_protection(void);
void nav_pilot(byte mode);
void autopilot_state(void);
float PID_altitude(float PID_set_Point, float PID_current_Point, int MAX_altitude, int MIN_altitude);
float constrain_float(float value, float max, float min);
void Init_IRs(void);
void read_irs(void);
byte analyse_irs(unsigned int refresh_rate);
void smart_delay(unsigned int time);
void Init_LEDs(void);
void off_leds(void);
void fun_LEDs(byte mode, byte cycles, unsigned int t_delay);
void Init_motors(void);
void motorRight(byte PWM, boolean dir);
void motorLeft(byte PWM, boolean dir);
void Init_RC(void);
byte RC_detector(void);
void motor_mixing(int ch1, int ch2);
void Init_servo(void);
void pulse_servo(int angle);
void test_servos(unsigned int time);
void init_ultrasonic(void);
unsigned int raw_ultrasonic(void);
unsigned int pulse_ultrasonic(unsigned int refresh_rate);
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
    startup_altitude=analogRead(2); //Reading the startup altitude, same that the system will try to maintain
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
      Init_servo();//Starting servo services... i mean timer 1 settings =).. 
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
    read_irs();//Capture ir packets..
    analyse_irs(200);//after 500 millisecon will analyse the packets captured
    nav_pilot(1);//Indicate to the autopilot to be in the autopilot state 0, that means hold altitude only
    autopilot_state(); //Verfy the autopilot state, if it changes, it will restart the PID and adjust the variables, etc.
    system_refresh(150); //This is the system refresh loop, that runs every 250 miliseconds
    break;
    /**************************************************************
     *System State 2: This state is to hold altitude and navigate to the beacon
     ***************************************************************/
  case 2: //Navigation and hold altitude
    read_irs(); //Capture ir packets..
    analyse_irs(200); //after 500 millisecon will analyse the packets captured
    nav_pilot(0); //Indicate to the autopilot to be in the autopilot state 1 that means hold altitude and navigate
    autopilot_state(); //Verfy the autopilot state, if it changes, it will restart the PID and adjust the variables, etc.
    system_refresh(150); //This is the system refresh loop, that runs every 250 miliseconds
    break;
    /**************************************************************
     * System State 3: RC mode or manual mode.....
     ***************************************************************/
  case 3:
    motor_mixing(ch1_position, ch2_position);
    system_refresh(150); //This is the system refresh loop, that runs every 250 miliseconds
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

   case 5: //The test state
     read_irs();//Capture ir packets..
     analyse_irs(200);//after 500 millisecon will analyse the packets captured
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


/**************************************************************
 * 
 ***************************************************************/
void nav_pilot(byte mode)
{
  int motor_speed=0;  

 if(millis() - timerPID_altitude > PID_dt) 
 {
  
  if(mode == 1)
  {
    //this is autipilot state 1, hold altitude only..
    
    motor_speed=PID_altitude(startup_altitude,raw_ultrasonic(), MAX_motor, MIN_motor);//Pulsing motor in function of the altitude
    
    if(motor_speed>=1)//this point will eliminate the dead zone of the motors,
    {
      motor_speed+=down_dead_zone; //push down
    }
    if(motor_speed<=-1)
    {
      motor_speed-=up_dead_zone; //push up, ive putted greater value because the propeller is less efficient in this direction
    }

    
    
    if(motor_speed >0) //Verify the direction we need to pulse the motors 
    {
      motorLeft(abs(0), REVERSE);
      motorRight(abs(motor_speed), REVERSE);   
    }
    else
    {
      motorLeft(abs(motor_speed), FORW);
      motorRight(abs(0), FORW);  
 
    }
  } 
  /**************************************************************
 * autopilot state 1, hold altitude and navigate...
 ***************************************************************/
  else//OHH beacon available.. 
  {     
    system_state=2;  //So we are in autopilot state 2 , hold altitude and navigate...

    pulse_servo(PID_altitude(startup_altitude,raw_ultrasonic(),MAX_vect,MIN_vect)); //Pulsing servo to hold altitude... 
    switch(ir_compare)//Pulse the motors in the correct way...  
    {
    case 0: //the beacon is to the  North
      motorLeft(forward_motor_left, FORW);
      motorRight(forward_motor_right, FORW);
      break; 
    case 1: //the beacon is to the East
      motorLeft(turn_motors, FORW);
      motorRight(0, FORW);
      break;
    case 2://the beacon is to the West
      motorLeft(0, FORW);
      motorRight(turn_motors, FORW);
      break;
    case 3://the beacon is to the South
      motorLeft(0, FORW);
      motorRight(turn_motors, FORW);
      break;
    } 
  }
   timerPID_altitude=millis();//Restarting the timer
}
}
/**************************************************************
 * 
 ***************************************************************/
void autopilot_state(void)
{

  if(system_state != last_state)//Check if the autopilot mode change
  {
    last_state=system_state;//restarting the status state

      //Here we are clearing the PID paramenters, just to void undesired behavior
    PID_I_altitude=0;
    PID_D_altitude=0;
    PID_prev_error_altitude=0;
    PID_output_altitude=0;
    PID_error=0;
    PID_P=0;
    switch(system_state)//System to setup the correct PID gains 
    {
    case 1://Autopilot mode 1 is hold altitude only
      pulse_servo(0); //Pointing motors to down
      //here im changing the PID constant values to hold altitude in function of the altitude unsing only motor power
      kp=alt_Kp;
      ki=alt_Ki;
      kd=alt_Kd;
      
      off_leds; //shutting down leds...
      break;

    case 2://Autopilot mode 2 is hold altitude and navigate... 
    // Here im changing the PID constants to hold altitude in function of the altitude using the vectoring
      kp=nav_Kp;
      ki=nav_Ki;
      kd=nav_Kd;
      break;

    }   
  }  

}

/****************************************************************************************
PID= P+I+D
 ***************************************************************/
 
 //PID_altitude(Desired altitude, current altitude, max output, min output)
float PID_altitude(float PID_set_Point, float PID_current_Point, int MAX_altitude, int MIN_altitude)
{

  //if (millis() - timerPID_altitude > PID_dt) {//Runs the module every x milisecond declared on PID_dt
    //Computes the error,
    average_altitude=(float)(average_altitude*0.5 + PID_current_Point*0.5); //The best way to average something, i do it to void glitches
    


    PID_error=average_altitude-PID_set_Point;//error = setpoint - actual_position

    /******************Proportional part****************/
    PID_P= kp*PID_error;

    /******************Integrator part****************/
    
    if((PID_output_altitude < MAX_altitude) && (PID_output_altitude > MIN_altitude))//This limit the output integrator, if not will increase till overflow
    {
     
      PID_I_altitude= PID_I_altitude+((float)((float)ki*(float)PID_error)*(float)PID_dt_seconds); // (1000 milliseconds / 1000) = 1 second
    }

    /******************Derivation part****************/

    PID_D_altitude= (float)((float)kd)*(((float)PID_error-(float)PID_prev_error_altitude)/(float)PID_dt_seconds);
   
   /******************output= P+I+D****************/
    //Plus all the PID results and limit the output... P+I+D and also constrain the output to the maximun and minimun value... 
    PID_output_altitude=(float)constrain_float((float)PID_P+(float)PID_I_altitude+(float)PID_D_altitude,MAX_altitude,MIN_altitude);//PID_P+PID_I+PID_D

    PID_prev_error_altitude=(float)PID_error; //Store the current error to use it letter (very important for derivated part)

  //}
  return (constrain_float(PID_output_altitude,MAX_altitude,MIN_altitude));
  //return (PID_output_altitude);  //To invert the result 
}




/**************************************************************
 * Special module to limit float values, is like contrain but this one is for variables IEEE 754 (Don't care is just floating point variables). =)  
 ***************************************************************/

float constrain_float(float value, float max, float min)
{
  if (value > max)
  {
    value=max;
  }
  if (value < min)
  {
    value=min;
  }
  return value;
}

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

  for(int i=0; i<=1000; i++) //Cicle that will read 9000 times the IR sensors
  {
    //IR_cnt[0] = (digitalRead(IR_FRONT));
     IR_cnt[0] += 0x01^(PINB & 0x01); //FRONT
     IR_cnt[1] += 0x01^((PIND & 0x40)>>6); //RIGHT
     IR_cnt[2] += 0x01^((PIND & 0x80)>>7); //BACK
     IR_cnt[3] += 0x01^((PINB & 0x02)>>1); //LEFT
  }
  
  /* Old rutine, do the same but a lot less efficient   
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
    if((IR_cnt[0]+IR_cnt[1]+IR_cnt[2]+IR_cnt[3]) <= 5)//If the ir reading is leass than 10, means no beacon preset, soo.. 
    {
      off_leds; //We turn off all the leds
      ir_compare=5;  //We put ir_compare to 5 (mean nothing)
      system_state=1; //We put the autpilot to hold altitude only... 
    }
    else
    {
      system_state=2; //That means that system state is 2
      ir_compare=0;//Restart the variable to cero

      if(IR_cnt[1]>IR_cnt[0]) //If the counter 1 (east IR sensor) is greater than the counter 0 (north ir sensor)
      {
        ir_compare|=0x01; //But the bit 1 in high   (0b00000001)
      } //If not leave it in cero (0b0000000). 

      if(IR_cnt[3]>IR_cnt[2]) //If the the counter 3 (west IR sensor) is greater than the counter 2 (south ir sensor)..  
      {
        ir_compare|=0x02; // Put the second bit of the variable in HIGH example: 0b00000010
      } //If not leave it in cero.. 0b00000000

      if(IR_cnt[(ir_compare&0x01)] > IR_cnt[((ir_compare>>1)+2)])//Now compare the to greatest counters either counter 0 or 1, vs. 2 or 3, 
      {
        ir_compare&=0x01;  //If yes, store the position of the > value, but eliminate the second bit and leave the first bit untouched.. 
      }
      else
      {
        ir_compare=((ir_compare>>1)+2);  //Eliminate the first bit and plus two...
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
 * The smart delay will improve performace of the IR beacons, instead wasting cpu time 
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
    for(int x=0; x<=cycles; x++)//System to play with the leds
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
    for(int x=0; x<=cycles; x++)//System to play with the leds
    {
      digitalWrite(led_north,HIGH); //And i start blinking the LED like crazy..
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
    for(int x=0; x<=cycles; x++)//System to play with the leds
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
    for(int x=0; x<=cycles; x++)//This just blinks the north led...
    {
      digitalWrite(led_north,HIGH);
      delay(t_delay);
      digitalWrite(led_north,LOW);
      delay(t_delay);
    }

    break;


  }
}

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
   if(static_friction_breaker_right == 0)//The verify again if the motor was stoped, if yes
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

    /**************************************************************
    * Function to init. the R/C mode.. 
    ***************************************************************/
void Init_RC(void)
{
  
  DDRB&= ~(0x03 << 6); //We change only bits 6 and 7 to 0 (input).
  
  /*Activating External interrupts for pins 6 and 7 of port B*/ 
  PCICR= 0x00|(1<<PCIE0); //Enable interrup, read page 70 of datasheet to know more .
  PCMSK0= 0x00|(1<<PCINT7)|(1<<PCINT6); //Interrupt mask, read page 71 of datasheet to know more .
  
  /*Activating Timer2, we use it to count only useconds, is the same timer used to pulse the servo, can't be used at the same time*/
  TCCR1A=0x00;
  TCCR1B = 0x00 |(1 << CS11)|(1<<WGM12) ; // using prescaler 8 =), page 134
  OCR1A  = 10; //prescaler 8/8mhz= 1 us resolution, so the interrupt will be execute every 10 us...                       
  TIMSK1 |=(1 << OCIE1A); //See page 136, Enabling timer interrupt  
  
  sei(); //Enabling all interrupts, this interrupt are at the end of the code, functions called "ISR(TIMER1_COMPA_vect)" and "ISR(PCINT0_vect)"
}
    /**************************************************************
    * Function to know if there is anything attached to the board.
    ***************************************************************/
byte RC_detector(void)
{
float average_ch1=0;

for(int c=0; c<=10; c++)
{
   average_ch1=(average_ch1*.50)+(ch1_position*.50);//Just average the input ch1
}

  if(average_ch1>1)//If the average is greater than 1, a RC receiver is attached to the board.. 
  {
  fun_LEDs(3, 15, 100); //Blink the led to let you know when you need to keep the stick centered....   
  rc_ch1_central=ch1_position;//Then it store the central position of the sticks
  rc_ch2_central=ch2_position;//You must maintain the stick from the control centered during this part... 
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
    
ISR(TIMER1_COMPA_vect)//This is a timer interrupts, executed every 10us 
{
  clock1++;//Just a counter, that increments every 10 us... 
  clock2++;
}

    /**************************************************************
    * External interrupt function 
    ***************************************************************/
   
ISR(PCINT0_vect)//This interrupt is executed every time the PIN 6 and 7 of portB change state. 
{
  /****This part is for channel 1****/
  if((read_ch1)&&(ch1_state_flag==0)) //if read_ch1 go to high, and if the flag is equal to 0 
  {
    clock1=0; //restart the clock 
    ch1_state_flag=1;//and change the flag to void undesired restart of the clock1.. 
  }
  if((read_ch1==0)&&(ch1_state_flag==1))//check if the pin is down, and also check if the flag was activated...
  {
    ch1_position=clock1; //pass the value of clock1 to ch1_position... 
    ch1_state_flag=0; //Reset the flag.... and thats all... 
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

    /**************************************************************
    * 
    ***************************************************************/
void Init_servo(void)//This part will configure the PWM to control the servo 100% bye hardware, and not waste CPU time.. 
{   
    digitalWrite(10,LOW);
    pinMode(10,OUTPUT);
    /*Timer 1 settings for fast PWM, is the same timer used to read the PWM of the remote control, can't be used at the same time*/
    TCCR1A =((1<<WGM11)|(1<<COM1B1));
    TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11);    
    OCR1B = 3000;
    ICR1 = 20000; 
}
    /**************************************************************
    * 
    ***************************************************************/
void pulse_servo(int angle)//Will convert the angle in the equivalent servo position... 
{
  constrain(angle,180,0);
  //angle=180-angle;//inverting the vectoring
 OCR1B=((((long)angle*(long)(max16-min16))/180L)+(long)min16);// 180L is the sames only 180, but the L means LONG.. So the compiler will take as a LONG variable... you can also say "(long)2". 
}
     /**************************************************************
    * 
    ***************************************************************/


void test_servos(unsigned int time)//Rutine to test the servo at the beggining (max, min and then central position)
{
  pulse_servo(180);
  delay(time);

  pulse_servo(90);
  delay(time);

  pulse_servo(0);
  delay(time);  
}  
  
  

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


int main(void)
{
	init();

	setup();
    
	for (;;)
		loop();
        
	return 0;
}

