/*  
 *  THIS CODE WAS WRITTEN FOR USE IN A HOME MADE VENTILATION DEVICE. 
 *  IT IS NOT TESTED FOR SAFETY AND NOT RECOMENDED FOR USE IN ANY COMERCIAL DEVICE.
 *  IT IS NOT APPROVED BY ANY REGULAOTRY AUTHORITY
 *  USE ONLY AT YOUR OWN RISK 
 */
 
// system configuration 
#define full_configuration 1               //  0 if the partial system - potentiometer on pulley, no potentiometers, ...

#define pressure_sensor_available 1

// options for display and debug
#define send_to_monitor 1    // 1 = send data to monitor  0 = dont
#define telemetry 0          // 1 = send telemtry fro debug

// UI
#define deltaUD 5   // define the value chnage per each button press

// clinical 
#define perc_of_lower_volume 65.0      // % of max press - defines lower volume
#define perc_of_lower_volume_display 40.0      // % of max press - defines lower volume
#define wait_time_after_resistance 3  // seconds
#define max_pres_disconnected 10      // if the max pressure during breathing cycle does not reach this value - pipe is disconnected
#define insp_pressure_default 40      // hold this pressure while breathing 
#define safety_pres_above_insp 10     // defines safety pressure as the inspirium pressure + this one
#define safety_pressure 70            // quicly pullnack arm when reaching this
#define speed_multiplier_reverse 2    // factor of speeed for releasing the pressure (runs motion in reverse at X this speed
#define smear_factor 0                // 0 to do all cycle in 2.5 seconds and wait for the rest 1 to "smear" the motion profile on the whole cycle time 


#if (full_configuration==0)  // Arm connected with strip or wire
  #define LCD_available 0 
  #define pres_pot_available 0        // 1 if the system has 3 potentiometer and can control the inspirium pressure 
  #define pin_SW3 7   // breath - On / Off / cal
  #define pin_TST 2   // test mode - not in use
  #define pin_LED_AMP 11   // amplitude LED
  #define pin_LED_FREQ 9   // frequency LED
  #define pin_LED_Fail 10  // FAIL and calib blue LED
  #define pin_USR 12  // User LED
  #define pin_FD 4    // freq Down
  #define pin_FU 5    // freq Up
  #define pin_AD 8    // Amp Down
  #define pin_AU 6    // Amp Up
  #define curr_sense 1 
  #define control_with_pot 0    // 1 = control with potentiometers  0 = with push buttons
  
  #define F 0.6       // motion control feed forward  
  #define KP 0.2      // motion control propportional gain 
  #define KI 2        // motion control integral gain 
  #define integral_limit 6  // limits the integral of error 
  #define f_reduction_up_val 0.65   // reduce feedforward by this factor when moving up 
#endif

#if (full_configuration==1) // Direct arm conection 
  #define LCD_available 1 
  #define pres_pot_available 1        // 1 if the system has 3 potentiometer and can control the inspirium pressure 
  #define pin_SW3 4         // breath - On / Off / cal
  #define pin_TST 2         // test mode - not in use
  #define pin_RST 5         // reset alarm - not in use
  #define pin_LED_AMP 13    // amplitude LED
  #define pin_LED_FREQ 13   // frequency LED
  #define pin_LED_Fail 10   // FAIL and calib blue LED
  #define pin_USR 9         // User LED

  #define pin_FD 13    // freq Down
  #define pin_FU 13    // freq Up
  #define pin_AD 13    // Amp Down
  #define pin_AU 13    // Amp Up
  #define curr_sense 0
  #define control_with_pot 1    // 1 = control with potentiometers  0 = with push buttons

  #define F 0.6       // motion control feed forward  
  #define KP 0.2      // motion control propportional gain 
  #define KI 2        // motion control integral gain 
 
//  #define F 6       // motion control feed forward  
//  #define KP 1.6    // motion control propportional gain 
//  #define KI 0      // motion control integral gain 
  #define integral_limit 6  // limits the integral of error 
  #define f_reduction_up_val 0.65    // reduce feedforward by this factor when moving up 

#endif

// Arduino pins alocation
#define pin_PWM 3

#define pin_POT 0   // analog pin of potentiometer
#define pin_CUR 1   // analog pin of current sense
#define pin_AMP 2   // analog pin of amplitude potentiometer control
#define pin_FRQ 3   // analog pin of amplitude frequency control
#define pin_PRE 6   // analog pin of pressure control

// Talon SR controller PWM settings ("angle" for Servo library) 
#define PWM_mid 93  // mid value for PWM 0 motion - higher pushes up
#define PWM_max 85
#define PWM_min -85
#define max_allowed_current 100 // 10 Amps

// motion control parameters
#define cycleTime 5        // milisec
#define alpha 0.95         // filter for current apatation - higher = stronger low pass filter
#define profile_length 500 // motion control profile length 
#define motion_control_allowed_error  30  // % of range

// motor and sensor definitions
#define invert_mot 1
#define invert_pot 0
#define min_address 4
#define max_address 8

#include <EEPROM.h>
#include <Servo.h> 
#include <Wire.h>
#include <SparkFun_MS5803_I2C.h> 
#include <LiquidCrystal_I2C.h>
#include "ArduinoUniqueID.h"

Servo motor;
LiquidCrystal_I2C lcd(0x27, 16, 2); // Set the LCD address to 0x27 for a 16 chars and 2 line display
MS5803 sparkfumPress(ADDRESS_HIGH);

// Motion profile parameters 
// pos byte 0...255  units: promiles of full range
// vel int 0...255  ZERO is at 128 , units: pos change per 0.2 sec
// profile data:  press 250 points (50%) relase 250 

byte pos[profile_length]={0,0,0,0,1,1,1,2,2,3,3,4,5,5,6,7,8,9,10,11,12,14,15,16,17,19,20,22,23,25,27,28,30,32,33,35,37,39,41,43,45,47,49,51,53,55,57,59,62,64,66,68,71,73,75,78,80,82,85,87,90,92,95,97,100,102,105,107,110,112,115,117,120,122,125,128,130,133,135,138,140,143,145,148,150,153,155,158,160,163,165,168,170,173,175,177,180,182,184,187,189,191,193,196,198,200,202,204,206,208,210,212,214,216,218,220,222,223,225,227,228,230,232,233,235,236,238,239,240,241,243,244,245,246,247,248,249,250,250,251,252,252,253,253,254,254,254,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,254,254,254,254,253,253,252,252,251,250,250,249,248,248,247,246,245,244,243,242,241,239,238,237,236,234,233,232,230,229,227,225,224,222,220,219,217,215,213,211,209,207,205,203,201,199,197,195,193,191,188,186,184,182,179,177,175,172,170,167,165,163,160,158,155,153,150,148,145,143,140,138,135,133,130,128,125,123,120,118,115,113,110,108,105,103,101,98,96,94,91,89,87,85,83,80,78,76,74,72,70,68,66,65,63,61,59,58,56,54,53,51,50,48,47,46,45,43,42,41,40,39,38,37,36,35,34,33,32,31,31,30,29,28,27,26,26,25,24,23,22,22,21,20,20,19,18,18,17,16,16,15,15,14,14,13,13,12,12,11,11,10,10,9,9,9,8,8,7,7,7,6,6,6,5,5,5,5,4,4,4,4,3,3,3,3,3,3,2,2,2,2,2,2,2,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
byte vel[profile_length]={129,129,130,130,131,131,132,133,133,134,135,135,136,136,137,138,138,138,139,140,140,141,141,141,142,142,143,143,144,144,145,145,145,146,146,146,147,147,147,148,148,148,149,149,149,150,150,150,150,151,151,151,151,151,152,152,152,152,152,152,153,153,153,153,153,153,153,153,153,153,153,153,153,153,153,153,153,153,153,153,153,153,153,153,153,153,153,153,153,153,152,152,152,152,152,152,151,151,151,151,151,150,150,150,150,149,149,149,148,148,148,147,147,147,146,146,146,145,145,145,144,144,143,143,142,142,141,141,141,140,140,139,138,138,138,137,136,136,135,135,134,133,133,132,131,131,130,130,129,129,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,127,127,126,126,126,125,125,124,124,123,123,122,122,121,121,120,120,120,119,118,118,118,117,117,116,116,115,115,115,114,114,113,113,113,112,112,111,111,111,110,110,109,109,109,108,108,108,107,107,107,107,106,106,106,105,105,105,105,105,104,104,104,104,104,104,103,103,103,103,103,103,103,103,103,103,103,103,103,103,103,103,103,103,103,103,104,104,104,104,104,105,105,105,105,105,106,106,106,107,107,107,108,108,108,109,109,109,110,110,111,111,112,112,113,113,114,114,114,115,116,116,116,117,117,118,118,118,118,118,119,119,119,119,119,119,119,120,120,120,120,120,120,120,121,121,121,121,121,121,121,122,122,122,122,122,122,122,123,123,123,123,123,123,123,124,124,124,124,124,124,124,124,124,125,125,125,125,125,125,125,125,125,126,126,126,126,126,126,126,126,126,126,126,126,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128};

byte FD,FU,AD,AU,FDFB,FUFB,ADFB,AUFB,SW3,SW3FB,TSTFB,run_profile,LED_status,USR_status,blueOn,calibrated=0, calibON, numBlinkAmp,numBlinkFreq;
int A_pot,prevA_pot, A_current, A_amplitude=80, prev_A_amplitude, A_freq;
int motorPWM,index=0, prev_index,i, wait_cycles,cycle_number, cycles_lost,index_last_motion;
unsigned int max_arm_pos=600, min_arm_pos=500;
float wanted_pos, wanted_vel_PWM, range, range_factor, profile_planned_vel, planned_vel, integral, error, f_reduction_up ;
unsigned long lastSent,lastBlink, lastIndex, lastUSRblink,last_TST_not_pressed,lastBlue,start_wait, last_sent_data, last_read_pres,start_disp_pres;
byte monitor_index=0, BPM=14,prev_BPM, in_wait, failure, send_beep, wanted_cycle_time, disconnected=0,high_pressure_detected=0, motion_failure=0, sent_LCD, hold_breath, safety_pressure_detected;
byte counter_ON,counter_OFF,SW3temp,insp_pressure,prev_insp_pressure, safety_pressure_counter, no_fail_counter,TST, counter_TST_OFF,counter_TST_ON,TSTtemp;
float pressure_baseline;
int pressure_abs,breath_cycle_time, max_pressure=100 , prev_max_pressure=100, min_pressure=999, prev_min_pressure=999, index_to_hold_breath;

void setup() {
  pinMode (pin_PWM,OUTPUT);
  pinMode (pin_FD,INPUT_PULLUP);
  pinMode (pin_FU,INPUT_PULLUP);
  pinMode (pin_AD,INPUT_PULLUP);
  pinMode (pin_AU,INPUT_PULLUP);
  pinMode (pin_SW3,INPUT_PULLUP);
  pinMode (pin_TST,INPUT_PULLUP);
  pinMode (pin_LED_AMP,OUTPUT);
  pinMode (pin_LED_FREQ,OUTPUT);
  pinMode (pin_LED_Fail,OUTPUT);
  pinMode (pin_USR,OUTPUT);

  motor.attach(pin_PWM); 
  Serial.begin (115200);
  Wire.begin();
  
  if (pressure_sensor_available) {
  sparkfumPress.reset();
  sparkfumPress.begin();
  pressure_baseline = sparkfumPress.getPressure(ADC_4096);
  }
  
  if (LCD_available){
    lcd.begin();  // initialize the LCD
    lcd.backlight();  // Turn on the blacklight and print a message.
  }
  
  for (i = 0; i < 2; i++) {UniqueIDdump(Serial);  delay(100); }  // for IAI monitor run for 100 cycles
  
  run_profile=0;
  EEPROM.get(min_address, min_arm_pos);   delay (100);
  EEPROM.get(max_address, max_arm_pos);  delay (100);
  if (min_arm_pos>=0 && min_arm_pos<1024 && max_arm_pos>=0 && max_arm_pos<1024) calibrated = 1;
  insp_pressure=insp_pressure_default;
}

void loop() 
{
  read_IO ();
  run_profile_func ();
  find_min_max_pressure();
  if (LCD_available) {if (index >= (profile_length-2)) if (sent_LCD==0) {sent_LCD=1; display_LCD();}
                      if (index==0) sent_LCD=0; }

  if (millis()-last_sent_data>7)
  { 
      if (telemetry) print_tele();
      if (send_to_monitor) send_data_to_monitor();
      last_sent_data=millis();
  }

  if (TST==0) last_TST_not_pressed=millis();
  if (millis()-last_TST_not_pressed>5000) calibrate_range();
  if (calibrated ==0) { run_profile=0;  calibrate_range(); }
}

void run_profile_func()
{
  if (run_profile)
  {
     if (millis()-lastIndex >= wanted_cycle_time)                        // do when cycle time was reached
     {
      cycles_lost = (millis()-lastIndex)/wanted_cycle_time-1;  
      cycles_lost=limit_int(cycles_lost,0,15);
      lastIndex=millis();  // last start of cycle time

      range = range_factor*(max_arm_pos - min_arm_pos);                 // range of movement in pot' readings
      wanted_pos = float(pos[index])*range/255 + min_arm_pos;           // wanted pos in pot clicks
      profile_planned_vel = (float(vel[index+1]) - 128.01)*range/255;   // in clicks per 0.2 second
      if (hold_breath==1 && safety_pressure_detected==0) 
        {   if (wanted_pos <= float (A_pot) || index ==0) hold_breath=0;
            planned_vel=0; 
            integral = 0;       
            wanted_pos = float (A_pot);                                 // hold current position
        }
      else 
        { 
          planned_vel= profile_planned_vel;
        }
      if (safety_pressure_detected) planned_vel=-speed_multiplier_reverse*planned_vel;          // to do the revese in case high pressure detected

      error = wanted_pos-float(A_pot);    
      if (100*abs(error)/(max_arm_pos - min_arm_pos)>motion_control_allowed_error && cycle_number>1) motion_failure=1;
         
      integral += error*float(wanted_cycle_time)/1000;
      if (integral> integral_limit) integral= integral_limit;
      if (integral<-integral_limit) integral=-integral_limit;
      if (index<20 || index ==200) integral=0;   // zero the integral accumulator at the beginning of cycle and movement up

      if (planned_vel<0) f_reduction_up = f_reduction_up_val; else f_reduction_up=1;  // reduce f for the movement up
  
      wanted_vel_PWM = F*planned_vel*f_reduction_up + KP*error+KI*integral;           // PID correction 
      wanted_vel_PWM = wanted_vel_PWM*float(cycleTime)/float(wanted_cycle_time);      // reduce speed for longer cycles 
 
      if (safety_pressure_detected) {index-=speed_multiplier_reverse*(1+cycles_lost);  }  // run in reverse if high pressure was detected
      if (index<0) { if(safety_pressure_detected==1) safety_pressure_counter+=1;          // count the number of cases reaching safety pressure
                     safety_pressure_detected=0; 
                     wait_cycles=200*wait_time_after_resistance; 
                     index=profile_length-2;                                               // set index to the point of waiting 
                    }    // stop the reverse when reching the cycle start point

      if (in_wait==0) index +=(1+cycles_lost);    // dont advance index while waiting at the end of cycle 
      if (index >= (profile_length-2))            // wait for the next cycle to begin in this point -> 2 points befoe the last cycle index
        {
        if (millis()-start_wait < breath_cycle_time) { index = profile_length-2; in_wait=1;   }    // still need to wait ...
        else {  index =0; cycle_number+=1; 
                start_wait=millis(); 
                in_wait=0; 
                send_beep=1; 
                high_pressure_detected=0;
             }            // time has come ... start from index = 0 
        }
      blink_user_led ();
    }
    calc_failure();
  }

else  // not running profile
  { wanted_vel_PWM=0;
    if (USR_status) 
      {if (millis()-lastUSRblink>10) {USR_status=0; lastUSRblink=millis(); LED_USR(0);}}
      else {if (millis()-lastUSRblink>490) {USR_status=1; lastUSRblink=millis(); LED_USR(1);}}
    cycle_number=0;
    delay (1);
  }
set_motor_PWM (wanted_vel_PWM);
}

int limit_int (int val,int low, int high)
 {
  int lim_val;
  lim_val=val;
  if (val<low) lim_val=low;
  if (val>high) lim_val=high;
  return (lim_val);
 }
 
void find_min_max_pressure()
{
  if (max_pressure<pressure_abs) max_pressure=pressure_abs;           // find the max pressure in cycle 
  if (min_pressure>pressure_abs) min_pressure=pressure_abs;           // find the min pressure in cycle 
  if (index== profile_length-3) { prev_min_pressure = min_pressure; prev_max_pressure = max_pressure; }   
  if (index== profile_length-2) { max_pressure=0;  min_pressure=999; }  
}

void blink_user_led()
{
  if (high_pressure_detected || safety_pressure_detected)   // blink LED fast
      { if (USR_status) 
        { if (millis()-lastUSRblink>20) {USR_status=0; lastUSRblink=millis(); LED_USR(0);}}
        else {if (millis()-lastUSRblink>80) {USR_status=1; lastUSRblink=millis(); LED_USR(1);}} }
  else     //  not in failure - blink LED once per cycle 
  { if (index>0.1*profile_length) LED_USR(0); else LED_USR(1); }  
}

void calc_failure()
{
  if (prev_max_pressure < max_pres_disconnected && cycle_number>2) disconnected=1; else disconnected=0; // tube was disconnected
  if (pressure_abs>insp_pressure && hold_breath==0 && profile_planned_vel>0) { high_pressure_detected=1; hold_breath=1; index_to_hold_breath=index; }   // high pressure detected 
  if (pressure_abs>safety_pressure && profile_planned_vel>0) safety_pressure_detected=1;
  if (pressure_abs>insp_pressure+safety_pres_above_insp && profile_planned_vel>0) safety_pressure_detected=1;
  if (index==0 && prev_index!=0 && failure==0 && safety_pressure_detected==0) no_fail_counter+=1;
  if (index==0)       failure =0;
  if (disconnected)   failure =1;
  if (safety_pressure_detected && safety_pressure_counter>=1) { failure=2; safety_pressure_counter=1; } 
  if (motion_failure) failure =3;
  if (disconnected==1 || motion_failure==1 || safety_pressure_detected==1) {blinkBlue (1); no_fail_counter=0;}  else {LED_FAIL(0); }
  if (no_fail_counter>=3) safety_pressure_counter=0;
  if (no_fail_counter>=100) no_fail_counter=100;
  prev_index= index;
}  

void display_text_calib (char *message)
{
  lcd.clear(); 
  lcd.setCursor(0, 0); lcd.print(message);  
  lcd.setCursor(0, 1); lcd.print("Then press Test");
}

void calibrate_range()   // used for calibaration of motion range
{ 
  byte progress;
  LED_USR(1);   calibON = 1; 
  while (TST==1) {read_IO ();     blinkBlue (2); }
  progress=0; TSTFB=0; delay(30);
  display_text_calib ("Move to Upper");
  while (progress==0)  // step 1 - calibrate top position
  {
    blinkBlue (2); read_IO (); delay(5);
    if (TST ==0 && TSTFB==1) progress=1;
    set_motor_PWM (0);
  }
  TSTFB=0;  delay(30);  progress=0;  LED_USR(0);
  read_IO (); min_arm_pos=A_pot;
  display_text_calib ("Move to Lower");
  while (progress==0)  // step 2 - calibrate bottom position
  {
    blinkBlue (4);  read_IO (); delay(5);
    if (TST ==0 && TSTFB==1) progress=1;
    set_motor_PWM (0);
  }
  TSTFB=0;  delay(30);   progress=0;   LED_USR(1);
  read_IO ();   max_arm_pos=A_pot; 
  display_text_calib ("Move to Safe");
  while (progress==0)   // step 3 - manual control for positioning back in safe location 
  {
    blinkBlue (8);  read_IO (); delay(5);
    if (TST ==0 && TSTFB==1) progress=1;
    set_motor_PWM (0);
  }

  EEPROM.put(min_address, min_arm_pos);  delay(200);
  EEPROM.put(max_address, max_arm_pos);  delay(200);
  SW3FB=0;
  last_TST_not_pressed=millis();
  run_profile=0;
  calibrated=1;
  calibON = 0; display_LCD();
}

void display_LCD()   // here function that sends data to LCD
{ 
  if (calibON==0) {
  lcd.clear();
  lcd.setCursor(0, 0);   lcd.print("BPM:");   lcd.print(byte(BPM));  
  lcd.print("  Dep:"); lcd.print(byte(A_amplitude));  lcd.print("%");
  lcd.setCursor(0, 1);  
  if (millis()- start_disp_pres<3000) { lcd.setCursor(0, 1); lcd.print("Insp. Press. :");  lcd.print(byte(insp_pressure));}
  else {lcd.print("Pmin:"); lcd.print(byte(prev_min_pressure)); lcd.print("  Pmax:"); lcd.print(byte(prev_max_pressure));}
  }   
}

void reset_failures()
{
  motion_failure=0;
  index_last_motion=index;
}

void set_motor_PWM (float wanted_vel_PWM)
{
  if (abs(A_pot-prevA_pot)>0 || abs(wanted_vel_PWM)<15) index_last_motion=index;
  //if (index-index_last_motion>10 || A_pot==0 || A_pot==1023) motion_failure=1;
  
  if (calibON==1 ) wanted_vel_PWM=read_motion_for_calib();  // allows manual motion during calibration
  if (invert_mot) wanted_vel_PWM=-wanted_vel_PWM;
  if (curr_sense) {if (A_current>max_allowed_current)   wanted_vel_PWM=0;}
  if (motion_failure==1 && calibON==0) wanted_vel_PWM=0;
  if (wanted_vel_PWM > 0) wanted_vel_PWM+=3;   // undo controller dead band
  if (wanted_vel_PWM < 0) wanted_vel_PWM-=3;   // undo controller dead band
  if (wanted_vel_PWM > PWM_max) wanted_vel_PWM= PWM_max;  // limit PWM
  if (wanted_vel_PWM < PWM_min) wanted_vel_PWM= PWM_min;  // limit PWM
  motorPWM = PWM_mid + int( wanted_vel_PWM );  
  motor.write(motorPWM);
}

int read_motion_for_calib()
{ int wanted_cal_PWM;
  if (control_with_pot)
    {
      if (A_freq>750) wanted_cal_PWM=(A_freq-750)/15;
      if (A_freq<250) wanted_cal_PWM=(A_freq-250)/15;
      if (A_freq>=250 && A_freq<=750) wanted_cal_PWM=0;
 //     Serial.println(wanted_cal_PWM);
    }
    else
    { wanted_cal_PWM=0;
      if (FD==1) wanted_cal_PWM = 8;  
      if (FU==1) wanted_cal_PWM = -8;   
      if (AD==1) wanted_cal_PWM = 16;
      if (AU==1) wanted_cal_PWM = -16;
    }
    return (wanted_cal_PWM);
}

void read_IO ()
{ FDFB=FD; FUFB=FU; ADFB=AD;  AUFB=AU;   SW3FB=SW3; TSTFB=TST;
  prev_A_amplitude=A_amplitude;
  prev_BPM=BPM;
  prevA_pot=A_pot;
  FD = (1-digitalRead  (pin_FD)); 
  FU = (1-digitalRead  (pin_FU));
  AD = (1-digitalRead  (pin_AD));
  AU = (1-digitalRead  (pin_AU));
  TSTtemp = (1-digitalRead  (pin_TST));
  SW3temp = (1-digitalRead (pin_SW3));
  if (SW3temp==1) {counter_ON+=1;  if (counter_ON>20)  {SW3=1; counter_ON=100; }} else counter_ON=0;
  if (SW3temp==0) {counter_OFF+=1; if (counter_OFF>20) {SW3=0; counter_OFF=100;}} else counter_OFF=0;
  if (TSTtemp==1) {counter_TST_ON+=1;  if (counter_TST_ON>20)  {TST=1; counter_TST_ON=100; }} else counter_TST_ON=0;
  if (TSTtemp==0) {counter_TST_OFF+=1; if (counter_TST_OFF>20) {TST=0; counter_TST_OFF=100;}} else counter_TST_OFF=0;
 
  A_pot= analogRead   (pin_POT);   if (invert_pot) A_pot=1023-A_pot;
  A_current= analogRead (pin_CUR)/8;  // in tenth Amps
  if(control_with_pot)
  {
  A_amplitude= perc_of_lower_volume_display + int(float(analogRead (pin_AMP))*(100-perc_of_lower_volume_display)/1023);
  A_amplitude=limit_int(A_amplitude,perc_of_lower_volume_display,100);
  A_freq= analogRead (pin_FRQ);
  BPM = 6+(A_freq-23)/55;
  breath_cycle_time = 60000/BPM;
  }
  else
  {
    if (TST==0) {
        if (FD==0 && FDFB==1) {BPM-=2; if(BPM<6) BPM=6; numBlinkFreq=2; cycle_number=0;} 
        if (FU==0 && FUFB==1) {BPM+=2; if(BPM>24) BPM=24; numBlinkFreq=2; cycle_number=0;}
        breath_cycle_time = 60000/BPM;
        if (AD==0 && ADFB==1) {A_amplitude-=deltaUD; numBlinkAmp=2; if (A_amplitude<perc_of_lower_volume_display) A_amplitude=perc_of_lower_volume_display; }
        if (AU==0 && AUFB==1) {A_amplitude+=deltaUD; numBlinkAmp=2; if (A_amplitude>100) A_amplitude=100;}
        }
     if (TST==1) {
        if (FD==0 && FDFB==1) {insp_pressure-=5; if(insp_pressure<30) insp_pressure=30; } 
        if (FU==0 && FUFB==1) {insp_pressure+=5; if(insp_pressure>70) insp_pressure=70;}
        if (AD==0 && ADFB==1) {insp_pressure-=5; if(insp_pressure<30) insp_pressure=30; } 
        if (AU==0 && AUFB==1) {insp_pressure+=5; if(insp_pressure>70) insp_pressure=70;}
        }
  }
  range_factor = perc_of_lower_volume+(A_amplitude-perc_of_lower_volume_display)*(100-perc_of_lower_volume)/(100-perc_of_lower_volume_display);
  range_factor = range_factor/100;
  if (range_factor>1) range_factor=1;  if (range_factor<0) range_factor=0; 
 
  if (pres_pot_available) insp_pressure= 10+analogRead (pin_PRE)/12;
  insp_pressure=limit_int(insp_pressure,30,70);
  if (abs(insp_pressure-prev_insp_pressure)>1) { prev_insp_pressure=insp_pressure; start_disp_pres=millis(); display_LCD(); }

  if (pressure_sensor_available)  
   {if (millis()-last_read_pres>100) 
     {
      last_read_pres = millis();  
      pressure_abs = int( sparkfumPress.getPressure(ADC_4096)-pressure_baseline);   // mbar
      if (pressure_abs<0) pressure_abs=0;
     }
   }
  if (prev_BPM != BPM || prev_A_amplitude !=A_amplitude)  display_LCD();
  if (SW3==0 && SW3FB==1)  // start /  stop breathing motion   
      {
        run_profile=1-run_profile; 
        start_wait=millis(); 
        integral=0; 
        reset_failures();
       } 
  
  wanted_cycle_time= cycleTime + int (float(breath_cycle_time-500*cycleTime)*float(smear_factor)/500);
  blink_LEDs ();
}

void send_data_to_monitor()
{ 
  if (monitor_index==0) Serial.println("A"); 
  if (monitor_index==1) Serial.println(byte(BPM)); 
  if (monitor_index==2) Serial.println(byte(A_amplitude)); 
  if (monitor_index==3) Serial.println(byte(pressure_abs)); 
  if (monitor_index==4) Serial.println(byte(failure)); 
  if (monitor_index==5) {if (send_beep) {Serial.println(byte(1)); send_beep=0;} else Serial.println(byte(0)); }
  if (monitor_index==6) Serial.println(byte(insp_pressure)); 
  monitor_index+=1; if (monitor_index==7) monitor_index=0;
}

void blink_LEDs()
{ 
  if (numBlinkAmp>0)
  {
    if (LED_status==0)  {if (millis()-lastBlink > 200) {LED_status =1; LED_AMP(1); analogWrite(pin_LED_Fail,50); lastBlink=millis();}}
    if (LED_status==1)  {if (millis()-lastBlink > 50) {LED_status =0; LED_AMP(0); lastBlink=millis(); analogWrite(pin_LED_Fail,0);    numBlinkAmp-=1;}}
  }
  if (numBlinkFreq>0)
  {
    if (LED_status==0)  {if (millis()-lastBlink > 200) {LED_status =1; LED_FREQ(1); analogWrite(pin_LED_Fail,127); lastBlink=millis();}}
    if (LED_status==1)  {if (millis()-lastBlink > 50) {LED_status =0; LED_FREQ(0); lastBlink=millis(); analogWrite(pin_LED_Fail,0);    numBlinkFreq-=1;}}
  }
}

void blinkBlue (int numb)
{
  if (blueOn==1) {if (millis()-lastBlue>20) {lastBlue=millis(); blueOn=0; LED_FAIL(0); }}   
  if (blueOn==0) {if (millis()-lastBlue>100*numb) {lastBlue=millis(); blueOn=1; LED_FAIL(1); }}   
}

void LED_FREQ(byte val)  {digitalWrite ( pin_LED_FREQ, val);}
void LED_AMP (byte val)  {digitalWrite ( pin_LED_AMP, val); }
void LED_FAIL (byte val)  {digitalWrite ( pin_LED_Fail, val);     }
void LED_USR (byte val)  {digitalWrite ( pin_USR, val);     }

void print_tele ()  // UNCOMMENT THE TELEMETRY NEEDED
{
//  Serial.print(" Fail (disc,motion,hiPres):"); Serial.print(disconnected); Serial.print(","); Serial.print(motion_failure); Serial.print(","); Serial.print(high_pressure_detected);
//  Serial.print(" CL:");  Serial.print(cycles_lost);  
//  Serial.print(" min,max:");  Serial.print(min_arm_pos); Serial.print(","); Serial.print(max_arm_pos);  
//  Serial.print(" WPWM :");  Serial.print(motorPWM); 
//  Serial.print(" integral:");  Serial.print(int(integral));  
  Serial.print(" Wa:");  Serial.print(int(wanted_pos));  
  Serial.print(" Ac:");  Serial.print(A_pot); 
//  Serial.print(" cur:");  Serial.print(A_current); 
//  Serial.print(" amp:");  Serial.print(A_amplitude); 
//  Serial.print(" freq:");  Serial.print(A_freq); 
//  Serial.print(" w cyc t:"); Serial.print(wanted_cycle_time);
// Serial.print(" P(mBar):"); Serial.println(pressure_abs);
//  Serial.print(" RF:");  Serial.print(range_factor); 
    Serial.println("");
}
