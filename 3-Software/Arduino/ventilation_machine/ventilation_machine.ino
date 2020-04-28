/*
THIS CODE WAS WRITTEN FOR TESTING AND RUNNING A HOME-MADE VENTILATION DEVICE.
IT IS NOT TESTED FOR SAFETY AND NOT APPROVED FOR USE IN ANY CLINICAL, MEDICAL OR COMERCIAL DEVICE.
IT IS NOT APPROVED BY ANY REGULATORY AUTHORITY.
USE ONLY AT YOUR OWN RISK.

To start calibrations:
First enter the maintenance setup menu by pressing the TEST button for 3 seconds.
Using the RATE potentiometer select the calibration required and press TEST to select.
Follow instructions on the screen.

For the Arm range calibration:
Use the Rate potentiometer to move the arm up/down.
*/

#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <SparkFun_MS5803_I2C.h>
#include <Wire.h>
#include "ArduinoUniqueID.h"

/// Version string: be sure to update this version number EVERY time you make any code change,
/// period, which isn't just a comment change! Document it in the Software Changelog in the main
/// AmboVent readme.
#define VERSION_STR "v0.1.0"

// TODO(ElectricRCAircraftGuy): get rid of nearly every single one of these #defines! They should
// be `constexpr` variables instead, with the sole exception of ones which are used for variable
// compilation, such as those which exclude certain portions of code based on hardware
// configuration. (Even this may not be true though, as using bool may still allow the compiler
// to optimize out those sections. Do some compilation testing when you get to it).

// System Configuration

/// Set to true for "full system" (the default), or to false for "partial system"--potentiometer
/// installed on pulley, no potentiometers... <---? TODO(@ElectricRCAircraftGuy): I request further
/// explanation from @nimrod46--please update this description to be more clear.
#define FULL_CONFIGURATION true
/// Set to true if you have installed an I2C pressure sensor
#define PRESSURE_SENSOR_AVAILABLE true
/// Set to true to send unique ID for 10 seconds at startup, false otherwise
#define CENTRAL_MONITOR_SYSTEM false

// Options for display and debug via serial communication port

/// Set to true to send data to the Serial Monitor, false otherwise
#define SEND_TO_MONITOR true
/// Set to true to send telemetry for debugging, false otherwise. See the end of this file for the
/// optional telemetry data to send (comment/uncomment selected lines as desired)
#define TELEMETRY false

// User Interface (UI) settings

/// Define the value change per button press, in percent, for the non-potentiometer version only
#define DELTA_COMPRESSION_PERCENT 5
/// IIR (Infinite Impulse Response) low-pass filter constant to filter the potentiometer values.
/// This is a floating point value that must be >= 0 and < 1.0. Make closer to 1.0 to increase
/// the strength of the low-pass filter, lowering the cutoff frequency and reducing the frequency
/// response, and closer to 0 to have the opposite effect.
#define POT_ALPHA 0.85

// Clinical settings

// TODO(@ElectricRCAircraftGuy): request help from @nimrod46: the pressure units are inconsistent!
// I need your help please to straighten in out. The Sparkfun `MS5803::getPressure()` function in
// "Libraries/SparkFun_MS5803-14BA_Breakout_Arduino_Library-master/src/SparkFun_MS5803_I2C.cpp" says
// it returns values in units of Pascals, yet many comments here are referring to units of cm H2O.
// 1 cm H2O = 98.0665 Pa.

/// Percent of max pressure; defines the lower volume
#define LOWER_VOLUME_PERCENT 50.0
/// Percent of max pressure; defines the lower volume to display when reaching the real lower volume
/// TODO(@ElectricRCAircraftGuy): I request further clarification from @nimrod46--please update
/// this--why is this different from LOWER_VOLUME_PERCENT?
#define LOWER_VOLUME_DISPLAY_PERCENT 33.0
/// Seconds to wait before re-attempting to push air after max pressure has been achieved
#define WAIT_TIME_AFTER_RESISTANCE_SEC 3
/// If the max pressure (in Pascals) during the breathing cycle does not reach this value it means
/// the pipe is disconnected
#define PRESSURE_MAX_DISCONNECTED_PA 10
/// Default inspiration (inhalation) pressure in Pascals; hold this pressure while breathing; the
/// value is changed if the inspiration pressure potentiometer is installed
#define PRESSURE_INSPIRATION_DEFAULT_PA 40
/// Defines the safety pressure (maximum safe pressure) as the inspiration pressure + this pressure,
/// in Pascals
#define PRESSURE_SAFETY_ABOVE_INSPIRATION_PA 10
/// Quickly pull back the arm when reaching this safety-critical pressure in cm H2O
#define PRESSURE_SAFETY_CMH2O 70
/// Speed multiplier for releasing the pressure (we will run the motion in reverse at a speed
/// equal to the normal forward speed x this speed multiplier)
#define SPEED_MULTIPLIER_REVERSE 2
/// Motion time in 100 millisecond counts; 35 = 3500 ms
#define MOTION_TIME_DEFAULT_100MS 35
/// Set to true to trigger a new breath in case of patient inhale during the PEEP plateau;
/// TODO(@ElectricRCAircraftGuy): what's a PEEP? Request clarity from @nimrod46--please fix this
#define PATIENT_TRIGGERED_BREATH true
/// TODO(@ElectricRCAircraftGuy): to @nimrod46: need help; units say cmH2O yet other units are
/// Pascals. See my TODO comment above. Also, this parameter needs a description.
#define PRESSURE_DELTA_PATIENT_INHALE_CMH2O 5
/// IIR (Infinite Impulse Response) low-pass filter constant to filter the pressure readings
/// during PEEP plateau.
/// TODO(@ElectricRCAircraftGuy): need help from @nimrod46. What's a PEEP plateau? Please update
/// this description.
/// This is a floating point value that must be >= 0 and < 1.0. Make closer to 1.0 to increase
/// the strength of the low-pass filter, lowering the cutoff frequency and reducing the frequency
/// response, and closer to 0 to have the opposite effect.
#define PRESSURE_ALPHA 0.98

// Full configuration: feedback pot on arm, potentiometers for User Interface.
// The system has 3 potentiometers and can control the inspiration (inhalation) pressure.
#if FULL_CONFIGURATION == true
#    define LCD_AVAILABLE true
#    define PIN_SW2 4  // breath - On / Off / cal
#    define pin_TST 2  // test mode - run one repiration cycle, enter and navigate the menu state
#    define pin_RST 5  // reset alarm - not in use
#    define pin_LED_RED 10    // Red LED - Indication for critical failures
#    define pin_LED_BLUE 12   // blue LED - not in use
#    define pin_LED_GREEN 11  // green LED -Indication for normal operation
#    define pin_LED_USR \
        9  // User LED - stady on when system is ON, blinking when in menu and clibrations
#    define pin_FD 13                // freq Down - not used when you have potentiometers
#    define pin_FU 13                // freq Up - not used when you have potentiometers
#    define pin_AD 13                // Amp Down - not used when you have potentiometers
#    define pin_AU 13                // Amp Up - not used when you have potentiometers
#    define curr_sense false         // o no current sensor
#    define control_with_pot true    // 1 = control with potentiometers  0 = with push buttons
#    define FF 10                    // motion control feed forward
#    define KP 3                     // motion control propportional gain
#    define KI 1                     // motion control integral gain
#    define integral_limit 5         // limits the integral of error
#    define f_reduction_up_val 0.80  // reduce feedforward by this factor when moving up
#else
// FULL_CONFIGURATION == false: no potentiometers for User Interface, feedback pot on pulley.
// The system can NOT control the inspiration (inhalation) pressure.
#    define LCD_AVAILABLE false
#    define PIN_SW2 7                // breath - On / Off / cal
#    define pin_TST 2                // test mode - not in use
#    define pin_LED_AMP 11           // amplitude LED
#    define pin_LED_FREQ 9           // frequency LED
#    define pin_LED_Fail 10          // FAIL and calib blue LED
#    define pin_LED_USR 12           // User LED
#    define pin_FD 4                 // freq Down
#    define pin_FU 5                 // freq Up
#    define pin_AD 8                 // Amp Down
#    define pin_AU 6                 // Amp Up
#    define curr_sense true          // 1- there is a curent sensor
#    define control_with_pot false   // 1 = control with potentiometers  0 = with push buttons
#    define FF 0.6                   // motion control feed forward
#    define KP 0.2                   // motion control propportional gain
#    define KI 2                     // motion control integral gain
#    define integral_limit 6         // limits the integral of error
#    define f_reduction_up_val 0.65  // reduce feedforward by this factor when moving up
#endif

// other Arduino pins alocation
#define pin_PWM 3  // digital pin that sends the PWM to the motor
#define pin_POT 0  // analog pin of motion feedback potentiometer
#define pin_CUR 1  // analog pin of current sense
#define pin_AMP 2  // analog pin of amplitude potentiometer control
#define pin_FRQ 3  // analog pin of rate potentiometer control
#define pin_PRE 6  // analog pin of pressure potentiometer control

// Talon SR or SPARK controller PWM settings ("angle" for Servo library)
#define PWM_mid 93  // was 93 -   mid value for PWM 0 motion - higher pushes up
#define PWM_max 85
#define PWM_min -85
#define max_allowed_current 100  // 100=10 Amps

// motion control parameters
#define cycleTime 10        // milisec
#define alpha 0.95          // filter for current apatation - higher = stronger low pass filter
#define profile_length 250  // motion control profile length
#define motion_control_allowed_error 1000  // % of range

// motor and sensor definitions
#define invert_mot true
#define invert_pot false

Servo motor;
// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);

#if PRESSURE_SENSOR_AVAILABLE == true
MS5803 sparkfumPress(ADDRESS_HIGH);
#endif

// Motion profile parameters
// pos byte 0...255  units: promiles of full range
// vel int 0...255  ZERO is at 128 , units: pos change per 0.2 sec
// profile data:  press 125 points (50%) relase 125

constexpr PROGMEM uint8_t pos[profile_length] = {
    0,   0,   1,   2,   4,   6,   8,   10,  13,  15,  18,  21,  25,  28,  31,  35,  38,  42,
    46,  50,  54,  57,  61,  66,  70,  74,  78,  82,  86,  91,  95,  99,  104, 108, 112, 117,
    121, 125, 130, 134, 138, 143, 147, 151, 156, 160, 164, 169, 173, 177, 181, 185, 189, 194,
    198, 201, 205, 209, 213, 217, 220, 224, 227, 230, 234, 237, 240, 242, 245, 247, 249, 251,
    253, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 254, 253, 252, 250, 248, 246, 244, 241, 238, 235, 232, 229, 225, 222, 218, 214, 210,
    206, 202, 198, 193, 189, 184, 180, 175, 171, 166, 162, 157, 152, 148, 143, 138, 134, 129,
    124, 120, 115, 111, 106, 102, 97,  93,  89,  84,  80,  76,  72,  68,  64,  61,  57,  54,
    50,  47,  44,  41,  38,  36,  33,  31,  29,  27,  25,  23,  22,  20,  19,  17,  16,  15,
    13,  12,  11,  10,  9,   8,   7,   6,   6,   5,   4,   3,   3,   2,   2,   1,   1,   1,
    0,   0,   0,   1,   1,   1,   1,   1,   1,   2,   2,   2,   2,   2,   2,   2,   2,   2,
    1,   1,   1,   1,   1,   1,   1,   1,   1,   0,   0,   0,   0,   0,   0,   0};
constexpr PROGMEM uint8_t vel[profile_length] = {
    129, 132, 134, 136, 137, 139, 140, 141, 142, 143, 143, 144, 144, 145, 146, 146, 146, 147,
    147, 147, 148, 148, 148, 148, 149, 149, 149, 149, 149, 149, 150, 150, 150, 150, 150, 150,
    150, 150, 150, 150, 150, 150, 150, 150, 150, 149, 149, 149, 149, 149, 149, 148, 148, 148,
    148, 147, 147, 147, 146, 146, 146, 145, 144, 144, 143, 143, 142, 141, 140, 139, 137, 136,
    134, 132, 129, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128,
    128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128,
    128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 127,
    125, 123, 121, 120, 119, 117, 116, 115, 114, 113, 112, 111, 111, 110, 109, 109, 108, 108,
    107, 107, 106, 106, 106, 106, 105, 105, 105, 105, 105, 105, 105, 105, 105, 105, 105, 105,
    105, 105, 105, 105, 106, 106, 106, 107, 107, 107, 108, 108, 109, 109, 110, 110, 111, 111,
    112, 113, 113, 114, 115, 116, 117, 118, 118, 119, 119, 120, 120, 120, 121, 121, 121, 122,
    122, 122, 123, 123, 123, 124, 124, 124, 124, 125, 125, 125, 125, 125, 126, 126, 126, 126,
    126, 127, 127, 127, 127, 127, 127, 127, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128,
    128, 128, 129, 129, 129, 129, 129, 129, 129, 129, 129, 128, 128, 128, 128, 128};

// TODO(ElectricRCAircraftGuy): global variables are bad. Meticulously go through every single
// one of these and move them into their appropriate scope. Also check their types, const-ness,
// and initial values. Need to see if they are even all used too. Before doing that, let's get
// clang-tidy working as a linter, as it can help identify that, and fix all compiler warnings,
// since some of those may catch this too.

uint8_t FD;
uint8_t FU;
uint8_t AD;
uint8_t AU;
uint8_t prev_FD;
uint8_t prev_FU;
uint8_t prev_AD;
uint8_t prev_AU;
uint8_t SW2;
uint8_t prev_SW2;
uint8_t prev_TST;
uint8_t RST;
uint8_t LED_status;
uint8_t USR_status;
uint8_t blueOn;
uint8_t calibON;
uint8_t numBlinkFreq;
uint8_t SW2_pressed;
uint8_t TST_pressed;
uint8_t menu_state;

bool calibrated = false;
bool is_led_red_on = false;
bool is_led_green_on = false;

uint8_t monitor_index = 0;
uint8_t BPM = 14;
uint8_t prev_BPM;
uint8_t in_wait;
uint8_t failure;
uint8_t send_beep;
uint8_t wanted_cycle_time;
uint8_t disconnected = 0;
uint8_t high_pressure_detected = 0;
uint8_t motion_failure = 0;
uint8_t sent_LCD, hold_breath;
uint8_t safety_pressure_detected;

uint8_t counter_ON;
uint8_t counter_OFF;
uint8_t SW2temp;
uint8_t insp_pressure;
uint8_t prev_insp_pressure;
uint8_t safety_pressure_counter;
uint8_t no_fail_counter;
uint8_t TST;
uint8_t counter_TST_OFF;
uint8_t counter_TST_ON;
uint8_t TSTtemp;

uint8_t patient_triggered_breath;
uint8_t motion_time;
uint8_t progress;

int16_t A_pot;
int16_t prev_A_pot;
int16_t A_current;
int16_t Compression_perc = 80;
int16_t prev_Compression_perc;
int16_t A_rate;
int16_t A_comp;
int16_t A_pres;

int16_t motorPWM;
int16_t index = 0;
int16_t prev_index;
int16_t i;
int16_t wait_cycles;
int16_t cycle_number;
int16_t cycles_lost;
int16_t index_last_motion;

int16_t pressure_abs;
int16_t breath_cycle_time;
int16_t max_pressure = 0;
int16_t prev_max_pressure = 0;
int16_t min_pressure = 100;
int16_t prev_min_pressure = 0;
int16_t index_to_hold_breath;
int16_t pressure_baseline;

int16_t comp_pot_low = 0;
int16_t comp_pot_high = 1023;
int16_t rate_pot_low = 0;
int16_t rate_pot_high = 1023;
int16_t pres_pot_low = 0;
int16_t pres_pot_high = 1023;

uint16_t max_arm_pos;
uint16_t min_arm_pos;

uint32_t lastSent;
uint32_t lastIndex;
uint32_t last_led_usr_blink_ms;
uint32_t last_led_red_blink_ms;
uint32_t last_led_green_blink_ms;
uint32_t last_led_yellow_blink_ms;
uint32_t last_TST_not_pressed;
uint32_t last_display_update_during_calib;
uint32_t start_wait;
uint32_t last_sent_data;
uint32_t last_read_pres;
uint32_t start_disp_pres;

float pot_rate;
float pot_pres;
float pot_comp;
float avg_pres;

float wanted_pos;
float wanted_vel_PWM;
float range;
float range_factor;
float profile_planned_vel;
float planned_vel;
float integral;
float error;
float prev_error;
float f_reduction_up;

enum main_states : uint8_t
{
    STBY_STATE,
    BREATH_STATE,
    MENU_STATE
};
enum main_states state;

/// @brief      Run once at startup
/// @note       This is called in `main()` in
///             "AmboVent/3-Software/Arduino/arduino_core/arduino/hardware/arduino/avr/cores/arduino/main.cpp
/// @param      None
/// @return     None
void setup()
{
    pinMode(pin_PWM, OUTPUT);
    pinMode(pin_FD, INPUT_PULLUP);
    pinMode(pin_FU, INPUT_PULLUP);
    pinMode(pin_AD, INPUT_PULLUP);
    pinMode(pin_AU, INPUT_PULLUP);
    pinMode(PIN_SW2, INPUT_PULLUP);
    pinMode(pin_TST, INPUT_PULLUP);
    pinMode(pin_LED_RED, OUTPUT);
    pinMode(pin_LED_GREEN, OUTPUT);
    // pinMode(pin_LED_BLUE, OUTPUT); //TODO: Return if needed
    pinMode(pin_LED_USR, OUTPUT);
    motor.attach(pin_PWM);

    Serial.begin(115200);
    Serial.print("\nAmboVent Version: ");
    Serial.println(VERSION_STR);
    Serial.println();

    Wire.begin();

#if PRESSURE_SENSOR_AVAILABLE == true
    sparkfumPress.reset();
    sparkfumPress.begin();
    pressure_baseline = int(sparkfumPress.getPressure(ADC_4096));
#endif

    if (LCD_AVAILABLE)
    {
        lcd.begin();      // initialize the LCD
        lcd.backlight();  // Turn on the blacklight and print a message.
        lcd.setCursor(0, 0);
        // TODO(ElectricRCAircraftGuy): need to print the version number on the LCD at startup too
        lcd.print("AmvoVent       ");
        lcd.setCursor(0, 1);
        lcd.print("1690.108       ");
    }

#if CENTRAL_MONITOR_SYSTEM == true
    // for IAI monitor run for 100 cycles
    for (i = 0; i < 100; i++)
    {
        UniqueIDdump(Serial);
        delay(100);
    }
#endif

    state = STBY_STATE;
    EEPROM.get(4, min_arm_pos);
    delay(20);
    EEPROM.get(8, max_arm_pos);
    delay(20);
    EEPROM.get(12, comp_pot_low);
    delay(20);
    EEPROM.get(16, comp_pot_high);
    delay(20);
    EEPROM.get(20, rate_pot_low);
    delay(20);
    EEPROM.get(24, rate_pot_high);
    delay(20);
    EEPROM.get(28, pres_pot_low);
    delay(20);
    EEPROM.get(32, pres_pot_high);
    delay(20);
    if (min_arm_pos >= 0 && min_arm_pos < 1024 && max_arm_pos >= 0 && max_arm_pos < 1024)
        calibrated = true;
    insp_pressure = PRESSURE_INSPIRATION_DEFAULT_PA;
    patient_triggered_breath = PATIENT_TRIGGERED_BREATH;
    motion_time = MOTION_TIME_DEFAULT_100MS;
    lcd.backlight();  // Turn on the blacklight and print a message.
    LED_USR(1);
}

/// @brief      Run repeatedly after setup()
/// @note       This is called in `main()` in
///             "AmboVent/3-Software/Arduino/arduino_core/arduino/hardware/arduino/avr/cores/arduino/main.cpp
/// @param      None
/// @return     None
void loop()
{
    read_IO();
    switch (state)
    {
    case STBY_STATE:  // standby
        standby_func();
        if (SW2_pressed && calibrated)  // start breathing motion
        {
            state = BREATH_STATE;
            initialize_breath();
        }
        if (TST == 0)
            last_TST_not_pressed = millis();
        if (millis() - last_TST_not_pressed > 3000)
        {
            while (TST == 1 || TST_pressed)
            {
                blink_user_led();
                read_IO();
            }  // wait for button release
            progress = 0;
            state = MENU_STATE;
        }
        break;

    case BREATH_STATE:  // run profile
        run_profile_func();
        if (SW2_pressed)
            state = STBY_STATE;  // stop breathing motion
        break;

    case MENU_STATE:  // maintanance menu
        display_menu();
        break;
    }

    if (millis() - last_sent_data > 20)
    {
        if (SEND_TO_MONITOR && !TELEMETRY)
            send_data_to_monitor();
        if (TELEMETRY)
            print_tele();
        last_sent_data = millis();
    }
}

void display_menu()
{
    blink_user_led();
    menu_state = map(pot_rate, 0, 1023, 0, 8);
    menu_state = constrain(menu_state, 0, 8);
    switch (menu_state)
    {
    case 1:  // calib pot
        display_text_2_lines("Calibrate Pots", "TEST to start");
        if (TST_pressed)
        {
            calibrate_pot_range();
            exit_menu();
        }
        break;

    case 2:  // calib pressure sensor
        display_text_2_lines("Calib pressure", "TEST to start");
        if (TST_pressed)
        {
            pressure_baseline = int(sparkfumPress.getPressure(ADC_4096));
            exit_menu();
        }
        break;

    case 3:  // move arm down once
        if (progress == 0)
        {
            display_text_2_lines("Press TEST to", "run one breath  ");
            if (TST_pressed)
            {
                initialize_breath();
                progress = 1;
            }
        }
        if (progress == 1)
        {
            run_profile_func();
            if (cycle_number > 0)
                exit_menu();
        }
        break;

    case 4:  // calib arm range of movement
        display_text_2_lines("Calibrate Arm", "TEST to start");
        if (TST_pressed)
        {
            calibrate_arm_range();
            exit_menu();
        }
        break;

    case 5:  // set motion profile total time
        display_text_2_lines("Set Motion Time", "TEST to start ");
        if (TST_pressed)
        {
            read_IO();
            while (TST_pressed == 0)
            {
                read_IO();
                motion_time = map(pot_rate, 0, 1023, 25, 50);
                motion_time = constrain(motion_time, 25, 50);
                if (millis() - last_display_update_during_calib > 100)
                {
                    last_display_update_during_calib = millis();
                    lcd.clear();
                    lcd.setCursor(0, 0);
                    lcd.print("Set Motion Time");
                    lcd.setCursor(0, 1);
                    lcd.print(int(100 * motion_time));
                    lcd.print(" mSec");
                }
            }
            delay(500);
            exit_menu();
        }
        break;

    case 6:  // toggle sync to patient
        if (patient_triggered_breath == 1)
            display_text_2_lines("Sync to patient", "ON  ");
        if (patient_triggered_breath == 0)
            display_text_2_lines("Sync to patient", "OFF  ");
        if (TST_pressed)
        {
            patient_triggered_breath = 1 - patient_triggered_breath;
            delay(110);  //  wait enough time that the display will be updated ..
            if (patient_triggered_breath == 1)
                display_text_2_lines("Sync to patient", "ON  ");
            if (patient_triggered_breath == 0)
                display_text_2_lines("Sync to patient", "OFF  ");
            delay(1000);
            exit_menu();
        }
        break;

    default:
        display_text_2_lines("Exit Menu", "Press TEST ");
        if (TST_pressed)
            exit_menu();
        break;
    }
}

void exit_menu()
{
    read_IO();
    last_TST_not_pressed = millis();
    state = STBY_STATE;
    index = 0;
    calibON = 0;
    display_LCD();
    progress = 0;
    LED_USR(1);
}

void run_profile_func()
{
    if (millis() - lastIndex >= wanted_cycle_time)  // do when cycle time was reached
    {
        cycles_lost = (millis() - lastIndex) / wanted_cycle_time - 1;
        cycles_lost = constrain(cycles_lost, 0, 15);
        lastIndex = millis();  // last start of cycle time
        calculate_wanted_pos_vel();

        if (safety_pressure_detected)
            index -= SPEED_MULTIPLIER_REVERSE
                     * (1 + cycles_lost);  // run in reverse if high pressure was detected
        if (index < 0)
        {
            if (safety_pressure_detected == 1)
                safety_pressure_counter += 1;  // count the number of cases reaching safety pressure
            safety_pressure_detected = 0;
            wait_cycles = 100 * WAIT_TIME_AFTER_RESISTANCE_SEC;
            index = profile_length - 2;  // set index to the point of waiting
        }                                // stop the reverse when reching the cycle start point

        if (in_wait == 0)
            index += (1 + cycles_lost);  //  advance index while not waiting at the end of cycle
        if (patient_triggered_breath
            == 1)  // detect drop in presure during the PEEP plateu and trigger breath based on this
        {
            if (in_wait == 1 || (index > profile_length / 2 && (A_pot < min_arm_pos + range / 18)))
            {
                if (avg_pres - pressure_abs > PRESSURE_DELTA_PATIENT_INHALE_CMH2O)
                    start_new_cycle();  // start new breath cycle if patient tries to inhale durint
                                        // the PEEP plateu
                avg_pres = avg_pres * PRESSURE_ALPHA
                           + (1 - PRESSURE_ALPHA)
                                 * float(pressure_abs);  // calculate the filtered pressure
            }
            else
            {
                avg_pres = pressure_abs;
            }  // initialize the filtered pressure
        }

        if (index >= (profile_length - 2))  // wait for the next cycle to begin in this point -> 2
                                            // points befoe the last cycle index
        {
            if (sent_LCD == 0)
            {
                sent_LCD = 1;
                display_LCD();  // update the display at the end of cycle
            }
            if (millis() - start_wait < breath_cycle_time)
            {
                index = profile_length - 2;
                in_wait = 1;  // still need to wait ...
            }
            else
                start_new_cycle();  // time has come ... start from index = 0
        }
    }
    calc_failure();
    set_motor_PWM(wanted_vel_PWM);
    find_min_max_pressure();
}

void calculate_wanted_pos_vel()
{
    uint8_t pos_from_profile, vel_from_profile;
    pos_from_profile = pgm_read_byte_near(pos + index);
    vel_from_profile = pgm_read_byte_near(vel + index + 1);

    range = range_factor * (max_arm_pos - min_arm_pos);  // range of movement in pot' readings
    wanted_pos = float(pos_from_profile) * range / 255 + min_arm_pos;  // wanted pos in pot clicks
    profile_planned_vel =
        (float(vel_from_profile) - 128.01) * range / 255;  // in clicks per 0.2 second

    planned_vel = profile_planned_vel;
    if (hold_breath == 1 && safety_pressure_detected == 0)
    {
        if (wanted_pos <= float(A_pot) || index == 0)
            hold_breath = 0;
        planned_vel = 0;
        integral = 0;
        wanted_pos = float(A_pot);  // hold current position
    }
    if (safety_pressure_detected)
        planned_vel = -SPEED_MULTIPLIER_REVERSE
                      * planned_vel;  // to do the revese in case high pressure detected
    prev_error = error;
    error = wanted_pos - float(A_pot);

    integral += error * float(wanted_cycle_time) / 1000;
    if (integral > integral_limit)
        integral = integral_limit;
    if (integral < -integral_limit)
        integral = -integral_limit;
    if (index < 2 || prev_error * error < 0)
        integral = 0;  // zero the integral accumulator at the beginning of cycle and movement up
    if (planned_vel < 0)
        f_reduction_up = f_reduction_up_val;
    else
        f_reduction_up = 1;  // reduce f for the movement up

    wanted_vel_PWM =
        FF * planned_vel * f_reduction_up + KP * error + KI * integral;  // PID correction
    wanted_vel_PWM = wanted_vel_PWM * float(cycleTime)
                     / float(wanted_cycle_time);  // reduce speed for longer cycles
}

void standby_func()  // not running profile
{
    if (TST_pressed)
    {
        initialize_breath();
        progress = 1;
    }
    if (progress == 1)
    {
        run_profile_func();
        if (cycle_number > 0)
            progress = 0;
    }
    else
    {
        wanted_vel_PWM = 0;  // dont move
        set_motor_PWM(wanted_vel_PWM);
    }
    delay(1);
}

void initialize_breath()
{
    cycle_number = 0;
    start_wait = millis();
    integral = 0;
    reset_failures();
    index = 0;
    in_wait = 0;
    high_pressure_detected = 0;
}

void start_new_cycle()
{
    index = 0;
    cycle_number += 1;
    start_wait = millis();
    in_wait = 0;
    send_beep = 1;
    sent_LCD = 0;
    high_pressure_detected = 0;
}

int range_pot(int val, int low, int high)
{
    int new_val;
    new_val = int(long(val - low) * long(1023) / (high - low));
    new_val = constrain(new_val, 0, 1023);
    return (new_val);
}

void find_min_max_pressure()
{
    if (max_pressure < pressure_abs)
        max_pressure = pressure_abs;  // find the max pressure in cycle
    if (min_pressure > pressure_abs)
        min_pressure = pressure_abs;  // find the min pressure in cycle
    if (index > profile_length - 10 && index < profile_length - 5)
    {
        prev_min_pressure = min_pressure;
        prev_max_pressure = max_pressure;
    }
    if (index >= profile_length - 5)
    {
        max_pressure = 0;
        min_pressure = 999;
    }
}

void calc_failure()
{
    if (100 * abs(error) / (max_arm_pos - min_arm_pos) > motion_control_allowed_error
        && cycle_number > 1)
        motion_failure = 1;

    if (prev_max_pressure < PRESSURE_MAX_DISCONNECTED_PA && cycle_number > 2)
        disconnected = 1;
    else
        disconnected = 0;  // tube was disconnected
    if (pressure_abs > insp_pressure && hold_breath == 0 && profile_planned_vel > 0)
    {
        high_pressure_detected = 1;
        hold_breath = 1;
        index_to_hold_breath = index;
    }  // high pressure detected
    if (pressure_abs > PRESSURE_SAFETY_CMH2O && profile_planned_vel > 0)
        safety_pressure_detected = 1;
    if (pressure_abs > insp_pressure + PRESSURE_SAFETY_ABOVE_INSPIRATION_PA
        && profile_planned_vel > 0)
        safety_pressure_detected = 1;
    if (index == 0 && prev_index != 0 && failure == 0 && safety_pressure_detected == 0)
        no_fail_counter += 1;
    if (index == 0)
        failure = 0;
    if (disconnected)
        failure = 1;
    if (safety_pressure_detected && safety_pressure_counter >= 1)
    {
        failure = 2;
        safety_pressure_counter = 1;
    }
    if (motion_failure)
        failure = 3;
    if (disconnected == 1 || motion_failure == 1 || safety_pressure_detected == 1)
    {
        no_fail_counter = 0;
    }
    if (no_fail_counter >= 3)
        safety_pressure_counter = 0;
    if (no_fail_counter >= 100)
        no_fail_counter = 100;
    prev_index = index;
}

void display_text_2_lines(char const *message1, char const *message2)
{
    if (millis() - last_display_update_during_calib > 100)
    {
        last_display_update_during_calib = millis();
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(message1);
        lcd.setCursor(0, 1);
        lcd.print(message2);
    }
}

void display_text_calib(char const *message)
{
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(message);
    lcd.setCursor(0, 1);
    lcd.print("Then press Test");
}

void display_pot_during_calib()
{
    if (millis() - last_display_update_during_calib > 100)
    {
        lcd.setCursor(13, 0);
        lcd.print(A_pot);
        lcd.print(" ");
        last_display_update_during_calib = millis();
    }
}

void calibrate_arm_range()  // used for calibaration of motion range
{
    calibON = 1;
    progress = 0;

    display_text_calib("Set Upper");
    while (progress == 0)
        internal_arm_calib_step();  // step 1 - calibrate top position
    progress = 0;
    min_arm_pos = A_pot;

    display_text_calib("Set Lower");
    while (progress == 0)
        internal_arm_calib_step();  // step 2 - calibrate bottom position
    progress = 0;
    max_arm_pos = A_pot;

    display_text_calib("Move to Safe");
    while (progress == 0)
        internal_arm_calib_step();  // step 3 - manual control for positioning back in safe location

    EEPROM.put(4, min_arm_pos);
    delay(200);
    EEPROM.put(8, max_arm_pos);
    delay(200);
    calibrated = true;
}

void internal_arm_calib_step()
{
    read_IO();
    if (TST_pressed)
        progress = 1;
    set_motor_PWM(0);
    display_pot_during_calib();
    delay(3);
    blink_user_led();
}

void calibrate_pot_range()  // used for calibaration of potentiometers
{
    calibON = 2;

    read_IO();
    display_text_calib("Pot to left pos");
    while (TST_pressed == 0)
    {
        blink_user_led();
        read_IO();  // step 1 - calibrate top position
    }
    comp_pot_low = analogRead(pin_AMP);
    rate_pot_low = analogRead(pin_FRQ);
    pres_pot_low = analogRead(pin_PRE);

    read_IO();
    display_text_calib("Pot to right pos");
    while (TST_pressed == 0)
    {
        blink_user_led();
        read_IO();  // step 2 - calibrate bottom position
    }
    comp_pot_high = analogRead(pin_AMP);
    rate_pot_high = analogRead(pin_FRQ);
    pres_pot_high = analogRead(pin_PRE);

    EEPROM.put(12, comp_pot_low);
    delay(100);
    EEPROM.put(16, comp_pot_high);
    delay(100);
    EEPROM.put(20, rate_pot_low);
    delay(100);
    EEPROM.put(24, rate_pot_high);
    delay(100);
    EEPROM.put(28, pres_pot_low);
    delay(100);
    EEPROM.put(32, pres_pot_high);
    delay(100);
}

void display_LCD()  // here function that sends data to LCD
{
    if (LCD_AVAILABLE)
    {
        if (calibON == 0 && state != MENU_STATE)
        {
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("BPM:");
            lcd.print((uint8_t)BPM);
            lcd.print("  Dep:");
            lcd.print((uint8_t)Compression_perc);
            lcd.print("%");
            lcd.setCursor(0, 1);
            if (failure == 0)
            {
                if (millis() - start_disp_pres < 2000)
                {
                    lcd.setCursor(0, 1);
                    lcd.print("Insp. Press. :");
                    lcd.print((uint8_t)insp_pressure);
                }
                else
                {
                    lcd.print("Pmin:");
                    lcd.print((uint8_t)prev_min_pressure);
                    lcd.print("  Pmax:");
                    lcd.print((uint8_t)prev_max_pressure);
                }
            }
            if (failure == 1)
                lcd.print("Pipe Disconnect");
            if (failure == 2)
                lcd.print("High Pressure");
            if (failure == 3)
                lcd.print("Motion Fail");
        }
    }
}

void reset_failures()
{
    motion_failure = 0;
    index_last_motion = index;
    failure = 0;
}

void set_motor_PWM(float wanted_vel_PWM)
{
    if (abs(A_pot - prev_A_pot) > 0 || abs(wanted_vel_PWM) < 15)
        index_last_motion = index;
    if (calibON == 1)
        wanted_vel_PWM = read_motion_for_calib();  // allows manual motion during calibration

    activate_rgb_led();

    if (invert_mot)
        wanted_vel_PWM = -wanted_vel_PWM;
    if (curr_sense)
    {
        if (A_current > max_allowed_current)
            wanted_vel_PWM = 0;
    }
    if (motion_failure == 1 && calibON == 0)
        wanted_vel_PWM = 0;
    if (wanted_vel_PWM > 0)
        wanted_vel_PWM += 3;  // undo controller dead band
    if (wanted_vel_PWM < 0)
        wanted_vel_PWM -= 3;  // undo controller dead band
    if (wanted_vel_PWM > PWM_max)
        wanted_vel_PWM = PWM_max;  // limit PWM
    if (wanted_vel_PWM < PWM_min)
        wanted_vel_PWM = PWM_min;  // limit PWM
    motorPWM = PWM_mid + int(wanted_vel_PWM);
    motor.write(motorPWM);
}

int read_motion_for_calib()
{
    int wanted_cal_PWM;
    if (control_with_pot)
    {
        if (pot_rate > 750)
            wanted_cal_PWM = (pot_rate - 750) / 15;
        if (pot_rate < 250)
            wanted_cal_PWM = (pot_rate - 250) / 15;
        if (pot_rate >= 250 && pot_rate <= 750)
            wanted_cal_PWM = 0;
        if (SW2 == 1)
            wanted_cal_PWM = -12;
        // if (RST==1) wanted_cal_PWM= 12;
    }
    else
    {
        wanted_cal_PWM = 0;
        if (FD == 1)
            wanted_cal_PWM = 8;
        if (FU == 1)
            wanted_cal_PWM = -8;
        if (AD == 1)
            wanted_cal_PWM = 16;
        if (AU == 1)
            wanted_cal_PWM = -16;
    }
    return (wanted_cal_PWM);
}

void store_prev_values()
{
    prev_FD = FD;
    prev_FU = FU;
    prev_AD = AD;
    prev_AU = AU;
    prev_SW2 = SW2;
    prev_TST = TST;
    prev_BPM = BPM;
    prev_A_pot = A_pot;
    prev_Compression_perc = Compression_perc;
}

void read_IO()
{
    store_prev_values();

    RST = (1 - digitalRead(pin_RST));
    TSTtemp = (1 - digitalRead(pin_TST));
    SW2temp = (1 - digitalRead(PIN_SW2));

    if (SW2temp == 1)
    {
        counter_ON += 1;
        if (counter_ON > 20)
        {
            SW2 = 1;
            counter_ON = 100;
        }
    }
    else
        counter_ON = 0;
    if (SW2temp == 0)
    {
        counter_OFF += 1;
        if (counter_OFF > 20)
        {
            SW2 = 0;
            counter_OFF = 100;
        }
    }
    else
        counter_OFF = 0;
    if (SW2 == 0 && prev_SW2 == 1)
        SW2_pressed = 1;
    else
        SW2_pressed = 0;

    if (TSTtemp == 1)
    {
        counter_TST_ON += 1;
        if (counter_TST_ON > 20)
        {
            TST = 1;
            counter_TST_ON = 100;
        }
    }
    else
        counter_TST_ON = 0;
    if (TSTtemp == 0)
    {
        counter_TST_OFF += 1;
        if (counter_TST_OFF > 20)
        {
            TST = 0;
            counter_TST_OFF = 100;
        }
    }
    else
        counter_TST_OFF = 0;
    if (TST == 0 && prev_TST == 1)
        TST_pressed = 1;
    else
        TST_pressed = 0;

    A_pot = analogRead(pin_POT);
    if (invert_pot)
        A_pot = 1023 - A_pot;
    A_current = analogRead(pin_CUR) / 8;  // in tenth Amps
    if (control_with_pot)
    {
        A_rate = analogRead(pin_FRQ);
        A_comp = analogRead(pin_AMP);
        A_pres = analogRead(pin_PRE);
        if (abs(pot_rate - A_rate) < 5)
            pot_rate = POT_ALPHA * pot_rate + (1 - POT_ALPHA) * A_rate;
        else
            pot_rate = A_rate;
        if (abs(pot_comp - A_comp) < 5)
            pot_comp = POT_ALPHA * pot_comp + (1 - POT_ALPHA) * A_comp;
        else
            pot_comp = A_comp;
        if (abs(pot_pres - A_pres) < 5)
            pot_pres = POT_ALPHA * pot_pres + (1 - POT_ALPHA) * A_pres;
        else
            pot_pres = A_pres;
        A_comp = range_pot(int(pot_comp), comp_pot_low, comp_pot_high);
        A_rate = range_pot(int(pot_rate), rate_pot_low, rate_pot_high);
        A_pres = range_pot(int(pot_pres), pres_pot_low, pres_pot_high);

        Compression_perc = LOWER_VOLUME_DISPLAY_PERCENT
                           + int(float(A_comp) * (100 - LOWER_VOLUME_DISPLAY_PERCENT) / 1023);
        Compression_perc = constrain(Compression_perc, LOWER_VOLUME_DISPLAY_PERCENT, 100);

        BPM = 6 + (A_rate - 23) / 55;           // 0 is 6 breaths per minute, 1023 is 24 BPM
        breath_cycle_time = 60000 / BPM + 100;  // in milisec

        insp_pressure = 30 + A_pres / 25;  // 0 is 30 mBar, 1023 is 70 mBar
        insp_pressure = constrain(insp_pressure, 30, 70);
        if (abs(insp_pressure - prev_insp_pressure) > 1)
        {
            prev_insp_pressure = insp_pressure;
            start_disp_pres = millis();
            display_LCD();
        }
    }
    else
    {
        FD = (1 - digitalRead(pin_FD));
        FU = (1 - digitalRead(pin_FU));
        AD = (1 - digitalRead(pin_AD));
        AU = (1 - digitalRead(pin_AU));
        if (TST == 0)
        {
            if (FD == 0 && prev_FD == 1)
            {
                BPM -= 2;
                if (BPM < 6)
                    BPM = 6;
                cycle_number = 0;
            }
            if (FU == 0 && prev_FU == 1)
            {
                BPM += 2;
                if (BPM > 24)
                    BPM = 24;
                cycle_number = 0;
            }
            breath_cycle_time = 60000 / BPM + 100;
            if (AD == 0 && prev_AD == 1)
            {
                Compression_perc -= DELTA_COMPRESSION_PERCENT;
                if (Compression_perc < LOWER_VOLUME_DISPLAY_PERCENT)
                    Compression_perc = LOWER_VOLUME_DISPLAY_PERCENT;
            }
            if (AU == 0 && prev_AU == 1)
            {
                Compression_perc += DELTA_COMPRESSION_PERCENT;
                if (Compression_perc > 100)
                    Compression_perc = 100;
            }
        }
        if (TST == 1)
        {
            if (FD == 0 && prev_FD == 1)
            {
                insp_pressure -= 5;
                if (insp_pressure < 30)
                    insp_pressure = 30;
            }
            if (FU == 0 && prev_FU == 1)
            {
                insp_pressure += 5;
                if (insp_pressure > 70)
                    insp_pressure = 70;
            }
            if (AD == 0 && prev_AD == 1)
            {
                insp_pressure -= 5;
                if (insp_pressure < 30)
                    insp_pressure = 30;
            }
            if (AU == 0 && prev_AU == 1)
            {
                insp_pressure += 5;
                if (insp_pressure > 70)
                    insp_pressure = 70;
            }
        }
    }
    if (is_starting_respiration())
    {
        range_factor = LOWER_VOLUME_PERCENT
                       + (Compression_perc - LOWER_VOLUME_DISPLAY_PERCENT)
                             * (100 - LOWER_VOLUME_PERCENT) / (100 - LOWER_VOLUME_DISPLAY_PERCENT);
        range_factor = range_factor / 100;
        if (range_factor > 1)
            range_factor = 1;
        if (range_factor < 0)
            range_factor = 0;
    }

#if (PRESSURE_SENSOR_AVAILABLE == 1)
    {
        if (millis() - last_read_pres > 100)
        {
            last_read_pres = millis();
            pressure_abs = int(sparkfumPress.getPressure(ADC_4096) - pressure_baseline);  // mbar
            if (pressure_abs < 0)
                pressure_abs = 0;
        }
    }
#endif

    if (prev_BPM != BPM || prev_Compression_perc != Compression_perc)
        display_LCD();
    wanted_cycle_time = int(100) * int(motion_time) / profile_length;
    if (wanted_cycle_time > breath_cycle_time / profile_length)
        wanted_cycle_time = breath_cycle_time / profile_length;
    if (wanted_cycle_time < cycleTime)
        wanted_cycle_time = cycleTime;
}

bool is_starting_respiration()
{
    return index == 0;
}

void activate_rgb_led()
{
    if (motion_failure)
    {
        LED_BLUE(0);
        LED_GREEN(0);
        LED_RED(1);
    }
    else if (safety_pressure_detected)
    {
        LED_GREEN(0);
        blink_red_led();
        LED_BLUE(0);
    }
    else if (wanted_vel_PWM > 0 && !calibON && planned_vel > 0)  // Arm is going down
    {
        if (disconnected || high_pressure_detected)
        {
            LED_BLUE(0);
            LED_RED(1);
            LED_GREEN(0);
        }
        else
        {
            LED_GREEN(1);
            LED_RED(0);
            LED_BLUE(0);
        }
    }
    else
    {
        LED_GREEN(0);
        LED_RED(0);
        LED_BLUE(0);
    }
}

void send_data_to_monitor()
{
    if (monitor_index == 0)
        Serial.println("A");
    if (monitor_index == 1)
        Serial.println((uint8_t)BPM);
    if (monitor_index == 2)
        Serial.println((uint8_t)Compression_perc);
    if (monitor_index == 3)
        Serial.println((uint8_t)pressure_abs);
    if (monitor_index == 4)
        Serial.println((uint8_t)failure);
    if (monitor_index == 5)
    {
        if (send_beep)
        {
            Serial.println((uint8_t)1);
            send_beep = 0;
        }
        else
            Serial.println((uint8_t)0);
    }
    if (monitor_index == 6)
        Serial.println((uint8_t)insp_pressure);
    monitor_index += 1;
    if (monitor_index == 7)
        monitor_index = 0;
}

void blink_user_led()
{
    if (USR_status)
    {
        if (millis() - last_led_usr_blink_ms > 10)
        {
            USR_status = 0;
            last_led_usr_blink_ms = millis();
            LED_USR(0);
        }
    }
    else
    {
        if (millis() - last_led_usr_blink_ms > 490)
        {
            USR_status = 1;
            last_led_usr_blink_ms = millis();
            LED_USR(1);
        }
    }
}

void blink_green_led()
{
    if (is_led_green_on)
    {
        if (millis() - last_led_green_blink_ms > 10)
        {
            is_led_green_on = false;
            last_led_green_blink_ms = millis();
            LED_GREEN(0);
        }
    }
    else
    {
        if (millis() - last_led_green_blink_ms > 490)
        {
            is_led_green_on = true;
            last_led_green_blink_ms = millis();
            LED_GREEN(1);
        }
    }
}

void blink_red_led()
{
    if (is_led_red_on)
    {
        if (millis() - last_led_red_blink_ms > 10)
        {
            is_led_red_on = false;
            last_led_red_blink_ms = millis();
            LED_RED(0);
        }
    }
    else
    {
        if (millis() - last_led_red_blink_ms > 490)
        {
            is_led_red_on = true;
            last_led_red_blink_ms = millis();
            LED_RED(1);
        }
    }
}

void LED_RED(uint8_t val)
{
    digitalWrite(pin_LED_RED, val);
}

void LED_GREEN(uint8_t val)
{
    digitalWrite(pin_LED_GREEN, val);
}

void LED_BLUE(uint8_t val)
{
    digitalWrite(pin_LED_BLUE, val);
}

void LED_PURPLE(uint8_t val)
{
    digitalWrite(pin_LED_BLUE, val);
    digitalWrite(pin_LED_RED, val);
}

void LED_YELLOW(uint8_t val)
{
    digitalWrite(pin_LED_GREEN, val);
    digitalWrite(pin_LED_RED, val);
}

void LED_USR(uint8_t val)
{
    digitalWrite(pin_LED_USR, val);
}

void print_tele()  // UNCOMMENT THE TELEMETRY NEEDED
{
    //  Serial.print(" Fail (disc,motion,hiPres):"); Serial.print(disconnected); Serial.print(",");
    //  Serial.print(motion_failure); Serial.print(","); Serial.print(high_pressure_detected);
    //  Serial.print(" CL:");  Serial.print(cycles_lost);
    //  Serial.print(" min,max:");  Serial.print(min_arm_pos); Serial.print(",");
    //  Serial.print(max_arm_pos); Serial.print(" WPWM :");  Serial.print(motorPWM); Serial.print("
    //  integral:");  Serial.print(int(integral));
    Serial.print(" Wa:");
    Serial.print(int(wanted_pos));
    Serial.print(" Ac:");
    Serial.print(A_pot);
    //  Serial.print(" cur:");  Serial.print(A_current);
    //  Serial.print(" amp:");  Serial.print(Compression_perc);
    //  Serial.print(" freq:");  Serial.print(A_rate);
    //  Serial.print(" w cyc t:"); Serial.print(wanted_cycle_time);
    //  Serial.print(" P :"); Serial.print(pressure_abs);
    //  Serial.print(" AvgP :"); Serial.print(int(avg_pres));
    //  Serial.print(" RF:");  Serial.print(range_factor);
    Serial.println("");
}
