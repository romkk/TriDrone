/*
 *  TriDrone for Arduino Pro Mini 16MHz/5V

 *  Setup How-To AVR GCC Toolchain on Mac OS X:
 *  Issue the following commands in Terminal.app (Homebrew required):
 *  $ brew tap larsimmisch/avr
 *  $ brew install avr-libc 
 *  $ brew install avrdude --with-usb

 *  The source code can be roughly divided in eleven parts:
 *  1  (startup)  : Initialization of hardware & variables & subs & startup procedure
 *  2  (gyro)     : Measurement of angular velocities via gyroscopes, PID control loop
 *  3  (acc)      : Measurement of accelerations (mainly gravitation)
 *  4  (mixer)    : Combination of sensor and stick inputs
 *  5  (send_mots): Sending out motor signals via I2C
 *  6  (led)      : Flashing of some leds
 *  7  (voltage)  : Measuring the voltage of the battery
 *  8  (failsave) : Monitoring receiver (RX) signal
 *  9  (receiver) : Reading RX's sum-signal
 *  10 (servo)    : Creation of servo PWM signal
 *  11 (GUI)      : Listen to serial commands from the GUI
 */

#include <avr/io.h> 
#include <stdint.h>         // for uint8_t data types and the like
#include <inttypes.h>
#include <math.h>           // ATTENTION! use -lm to link math.h
#include <util/delay.h>     // for _delay_ms command
#include "bits.h"           // set, reset and toggle bits
#include "i2c.h"

// ARDUINO PIN AND PORT CONFIGURATION (ATMega 328p) ////////////////////////////
#define PB_LED2      0
#define PB_LED3      1
#define PB_LED_GREEN 5          // green LED on Arduino Pro Mini
#define PD_LED1      7
#define PD_RECEIVER  3
#define PD_SCL       5
#define PD_SDA       4
#define PD_SERVO     6
// Receiver - > D3 = Portd.3 = Int1
//    Voltage meas = ADC[0]
//        Gyro Yaw = ADC[1]
//       Gyro Nick = ADC[2]
//       Gyro Roll = ADC[3]
//           X Acc = ADC[4]
//           Y Acc = ADC[5]


// PROTOTYPES //////////////////////////////////////////////////////////////////
void gyro();
void acc();
void mixer();
void sendMots();
void led();
void voltage();
void failsave();
void GUIconnection();

void getEeprom();
void testRC();


// DECLARATIONS ////////////////////////////////////////////////////////////////
// Global
uint16_t i;                  //counter var
float Lf;                    //stick sensitivity
float LfdynamicRoll;         //dynamic stick sensitvity (used for flying loopings)
float LfdynamicNick;         //dynamic stick sensitvity (used for flying loopings)
uint8_t motorsOn;            //self explanatory, right...?!
uint8_t state;               //Contains selected control loop state: 0 = Motors off, 1 = Acrobatic mode, 2 = Hover mode
uint8_t oldState ;           //Contains the state of the last cycle
int16_t lookupPosNick;
int16_t LookupPosRoll;

// Servo control
uint16_t servoTail;          //servo position, neutral = 62535, full left = 61535, full right = 63535
uint8_t sHigh;               //1 if servo pin is high, 0 if servo pin is low

// Read receiver (Rx)
uint16_t Empf[5];            //data from getreceiver interrupt
int16_t sEmpf[5] ;           //rescaled data
int16_t aEmpfh[5];           //higher limit for rx noise filter
int16_t aEmpfl[5];           //lower limit for rx noise filter
uint8_t channel;             //current channel to be read out
uint16_t rcOnCounter;        //counter that counts the amount of correct rx signals

// I2C
const int16_t M_h = 0x52;    //i2c address: motor at the back
const int16_t M_l = 0x54;    //i2c address: motor at the left
const int16_t M_r = 0x56;    //i2c address: motor at the right
int16_t ms_h_i;              //contains motor nominal value as integer
int16_t ms_r_i;              //contains motor nominal value as integer
int16_t ms_l_i;              //contains motor nominal value as integer
uint16_t Ms_h;               //converts motor nominal value
uint16_t Ms_r;               //converts motor nominal value
uint16_t Ms_l;               //converts motor nominal value

// Gyros
uint16_t RollInit;           //gyro offset
uint16_t NickInit;           //gyro offset
uint16_t YawInit;            //gyro offset

int16_t measRoll;            //angular velocity reading from gyros
float setPointRoll;          //Stick position
float errorRoll;             //Stick position - current position
float errorRollSum;          //Integral of the above
float MeasAngleRoll;         //the angle of the copter including acc signal (only in Hover mode)

int16_t measNick;            //angular velocity reading from gyros
float SetpointNick;          //Stick position
float errorNick;             //Stick position - current position
float errorNickSum;          //Integral of the above
float measAngleNick;         //the angle of the copter including acc signal (only in Hover mode)

float PSetRoll;              //P output including scaling (gain)
float ISetRoll;              //I output including scaling (gain)
float DSetRoll;              //D output including scaling (gain)

float PSetNick;              //P output including scaling (gain)
float ISetNick;              //I output including scaling (gain)
float DSetNick;              //D output including scaling (gain)

int16_t yawGyro;             //angular velocity reading from gyroscope
int16_t yawGyro_i;           //yaw gyro var
float yawGyroScale;          //scaled-down values
float yawGyroIScale;         //scaled-down values

float PSens;                 //roll/nick P control loop sensitivity
float ISens;                 //roll/nick I control loop sensitivity
float DSens;                 //roll/nick D control loop sensitivity
float yawPSens;              //yaw P sensitivity
float yawISens;              //yaw I sensitivity

uint8_t gyroIEnable;         //0 or 1: don't perform gyro integration when motors off

float errorNick_d[3];
float errorRoll_d[3];
float errorNickOld[3];
float errorRollOld[3];
uint8_t Looper;
float DSensAcro;

float DdSens;
float DdSetNick;
float DdSetRoll;
int16_t DdSetNick_int;
int16_t DdSetRoll_int;

// Acc
int16_t accX;                //accelerometer nick
int16_t accY;                //accelerometer roll
float accXTemp;              //used for complementary filtering
float accYTemp;              //used for complementary filtering

// Failsave
uint8_t Failure;             //increases when there are errors in receiver reading
uint16_t Meanrx[5];          //used for receiver noise filter

// Mixer
uint16_t Vorwahl;            //throttle idle up
int16_t yawDiff;             //yaw gyro var
int16_t Tailgas;             //increases throttle of rear motor if tilted

// Voltage check
uint16_t volt;               //reads voltage from adc
float voltAvg;               //used for filtering
uint16_t voltOld  = 1022;    //prepare the filter
uint16_t voltOld2 = 1023;    //prepare the filter
uint16_t voltOld3 = 1023;    //prepare the filter
uint8_t lowVoltage;          //will be 1 if voltage is low

// LEDs
uint16_t ledCount;           //used for bling bling
uint16_t Blinker;            //used for low voltage bling bling

// GUI connection
uint8_t A[9];
uint8_t ReadEmpfangen;
uint8_t ResetEmpfangen;
uint8_t sensorsEmpfangen;
uint8_t InputEmpfangen;
uint8_t Var[33];
uint8_t sensor[13];
uint8_t motorsEnable;
uint8_t rollGyroDir;
uint8_t NickGyroDir; 
uint8_t yawGyroDir; 
uint8_t xAccDir;
uint8_t yAccDir; 
float PSensAcro;
float ISensAcro;
float PSensHover;
float ISensHover;
float DSensHover;
float yawPSensEEP; 
float yawISensEEP;
float AccInfluence; 
float GyroInfluence;
int16_t xAccScale;
int16_t yAccScale;
float LfAcro; 
float LfHover; 
int16_t LfYaw; 
uint8_t LfBoost; 
uint16_t Idle_up; 
float voltageWarn; 
int16_t xAccOffset; 
int16_t yAccOffset; 
float measRollGUI; 
float measNickGUI; 
float yawGyroGUI; 
float xAccGUI; 
float yAccGUI; 
float MeasAngleRollGUI; 
float measAngleNickGUI; 
float sEmpfGUI[5]; 
float VoltageGUI; 
uint8_t throttleChannel; 
uint8_t nickChannel; 
uint8_t rollChannel; 
uint8_t yawChannel; 


// FUNCTIONS ///////////////////////////////////////////////////////////////////

// force _delay_ms to do one second delay, not only 262.14 ms (inaccurate)
void long_delay_ms(uint16_t ms)
{
  for (; ms > 0; ms--) _delay_ms(1);
}

//TODO replace this with a working solution
//BASCOM: Retrieves the analog value from the specified channel.
uint8_t getADC(uint8_t x)
{
  return 1;
}

// TODO replace this with a working solution
//BASCOM: Returns one(1) when a character is waiting in the hardware UART buffer.
uint8_t isCharWaiting()
{
  return 0;
}


void init()
{
   /*
   //===READ RX SETTINGS===
   Config Timer0 = Timer , Prescale = 256
   On Timer0 Detectrxpause      //timer overflow = pause in receiver's signal
   Config Int1 = Rising
   On Int1 Getreceiver          //rising edge -> measure time

   //===SERVO SETTINGS===
   Config Timer1 = Timer , Prescale = 8
   Timer1 = 62535
   On Timer1 Servoirq           //creates the servo PWM

   //===SERIAL IN SETTINGS===
   Config Serialout = Buffered , Size = 254
   Config Serialin = Buffered , Size = 254

   // ADC SETTINGS /////////////////////////////////////////////////////////////
   Config Adc = Single , Prescaler = Auto , Reference = Avcc       //avcc reference @ arduino =5V
   */
   DDRB   |= (1 << PB5);    ///PB5/digital 13 is an output

   ADCSRA |= ((1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0));    //Prescaler at 128 so we have an 125Khz clock source
   ADMUX  |= (1 << REFS0);
   ADMUX  &= ~(1 << REFS1);                //Avcc(+5v) as voltage reference
   ADCSRB &= ~((1 << ADTS2) | (1 << ADTS1) | (1 << ADTS0));    //ADC in free-running mode
   ADCSRA |= (1 << ADATE);                //Signal source, in this case is the free-running

   // PORT SETTINGS ////////////////////////////////////////////////////////////
   /*
   Config Scl = Portd.5         //for ESCs
   Config Sda = Portd.4         //for ESCs
   Config Pind.6 = Output       //servo
   Portd.6 = 0                  //servo
   Config Pind.7 = Output       //led 1
   Config Pinb.0 = Output       //led 2
   Config Pinb.1 = Output       //led 3
   Config Pinb.5 = Output       //Arduino LED_grn
   */
   DDRD  = 0b01110011;
   DDRB  = 0b00100011;
   PORTB = 0b00000000;

   // Alias definitions (doesn't work that easy in C) //////////////////////////
   // TODO possible solution via #define
   /*
   Led_1 Alias Portd.7
   Led_2 Alias Portb.0
   Led_3 Alias Portb.1
   Led_grn Alias Portb.5
   */

   // light up leds after reset
   PORTD = 0b10000011;
   // PORTD |= ((1 << PD_LED1) | (1 << PD_LED2) | (1 << PD_LED3));

   motorsOn    = 0;
   state       = 0;             //states: 0=GUI, 1=Acro, 2=Hover
   ledCount    = 1;
   servoTail   = 62535;         //neutral position for the servo
   gyroIEnable = 0;

   //TODO Clear Serialin
   //TODO Clear Serialout

   // Startup 2
   //TODO sleep(1);
   long_delay_ms(1000);
   //TODO Enable Timer0
   //TODO Enable Timer1
   //TODO Enable Interrupts
   //TODO Enable Int1

   //TODO Start Adc
   ADCSRA |= (1 << ADEN);                //Power up the ADC
   ADCSRA |= (1 << ADSC);                //Start converting
   
   //TODO I2cinit

   //TODO Ms_h = 0 : I2csend M_h , Ms_h //turn off the motors
   //TODO Ms_r = 0 : I2csend M_r , Ms_r
   //TODO Ms_l = 0 : I2csend M_l , Ms_l

   reset(&PORTB, PB_LED_GREEN); // Led_grn = 0
   _delay_ms(500.00);

   // turn off leds
   reset(&PORTD, PD_LED1);
   reset(&PORTB, PB_LED2);
   reset(&PORTB, PB_LED3);
}

// Load and process settings from EEPROM
void loadEeprom()
{
  for (i=1; i <= 6; i++) {
    toggle(&PORTB, PB_LED_GREEN);
    _delay_ms(50.00);
  }
  getEeprom();
  for (i=1; i <= 6; i++) {
    toggle(&PORTB, PB_LED_GREEN);
    _delay_ms(50.00);
  }
}

// RC turned on? ///////////////////////////////////////////////////////////////
void testRC()
{
testRCagain:
  // if motors are enabled from GUI, it is important to have stick in correct position
  // switch (channel5) and throttle stick must be zero in order to proceed
  if (motorsEnable == 1) { 
    // stick is not in correct position
    if (Empf[5] > 80 || Empf[5] < 50 || Empf[throttleChannel] > 80 || Empf[throttleChannel] < 50 ) {
      if (rcOnCounter > 0) {
        rcOnCounter--;
      } else {                  // stick is in correct position
        rcOnCounter++;
      }
    }

    if (rcOnCounter == 0) {     // stick is not in correct position
      toggle(&PORTD, PD_LED1);              // give signal to pilot
      toggle(&PORTB, PB_LED3);
      _delay_ms(75.00);
    }
    
    if (rcOnCounter < 500) {    // if we didn't get the correct stick position 500 times...
      _delay_ms(1.00);
      goto testRCagain;         // ...return and check again
    }
  }

  long_delay_ms(1000); // If we got the correct stick positions 500 times... proceed
  set(&PORTB, PB_LED_GREEN);    // Led_grn = 1
  reset(&PORTD, PD_LED1);
  reset(&PORTB, PB_LED3);
  // Measure Gyro Offset (don't move copter now!) ////////////////////////////////
  RollInit = 0;
  NickInit = 0;
  YawInit = 0;
  for (i = 1; i <= 100; i++) {                        //read and sum up gyro's 100 times
    RollInit = RollInit + getADC(3);
    NickInit = NickInit + getADC(2);
    YawInit = YawInit + getADC(1);
    volt = getADC(0);
    toggle(&PORTB, PB_LED_GREEN);
    if (i < 33){                       //bling bling
      toggle(&PORTD, PD_LED1);
    }
    if (i > 33 && i < 66){ 
      reset(&PORTD, PD_LED1);
      toggle(&PORTB, PB_LED2);
    }
    if (i > 66){
      reset(&PORTB, PB_LED2);
      toggle(&PORTB, PB_LED3);
    }
    _delay_ms(13.00);
  }
  reset(&PORTD, PD_LED1);
  reset(&PORTB, PB_LED2);
  reset(&PORTB, PB_LED3);
  RollInit = RollInit / 100;             //get the average of the last 100 readings
  NickInit = NickInit / 100;
  YawInit = YawInit / 100;

  //Prepare receiver noise filter..
  Meanrx[1] = 63;
  Meanrx[2] = 100;
  Meanrx[3] = 100;
  Meanrx[4] = 100;
  Meanrx[5] = 63;
}


// GYRO ////////////////////////////////////////////////////////////////////////
void gyro() 
{
   if (oldState != state) {         //compare state of last cycle with current state
      errorRoll = 0;                        //otherwise things would start mixing up...
      errorNick = 0;
      measRoll = 0;
      measNick = 0;
      errorRollSum = 0;
      errorNickSum = 0;
      DSetRoll = 0;
      DSetNick = 0;

      for (i=1; i<=3; i++) {
         errorRollOld[i] = 0;
         errorRoll_d[i] = 0;
         errorNickOld[i] = 0;
         errorNick_d[i] = 0;
      }

      if (state == 2) {
         MeasAngleRoll = accY;  // when switching from acro mode to hover mode
         measAngleNick = accX;  // start with a "close to reality" angle
      } else {
         MeasAngleRoll = 0;
         measAngleNick = 0;
      }
      yawGyro_i = 0;
      yawDiff = 0;
   }

   oldState = state;

   if (state == 1) {            // ACROBATIC MODE = Angular velocity control
      if (Looper < 2) {         // for angular acceleration measurement
         Looper++;              // acceleration will be calculated as the difference in velocity between
      } else {                  // loop n and loop n+2
         Looper = 0;
      }


      // Roll
      measRoll = getADC(3);                 //get gyro signal
      if (rollGyroDir == 0) {             //make gyro direction reversal possible
         measRoll = RollInit - measRoll;   //subtract offset
      } else if (rollGyroDir == 1) { 
         measRoll = measRoll - RollInit;   //subtract offset
      }
      //measRoll = measRoll - Rollstickvel
      setPointRoll = sEmpf[rollChannel] * LfdynamicRoll;       //sEmpf[2] is roll stick position
      errorRoll = measRoll - setPointRoll;       //calculate difference between angular velocity and stick position
      if (Looper == 0) {                     //this calculates the angular velocity (D-term in acro mode)
         errorRoll_d[1] = errorRoll - errorRollOld[1];
         errorRollOld[1] = errorRoll;
         DSetRoll = errorRoll_d[1] * DSens;
      } else if (Looper == 1) {
         errorRoll_d[2] = errorRoll - errorRollOld[2];
         errorRollOld[2] = errorRoll;
         DSetRoll = errorRoll_d[2] * DSens;
      } else if (Looper == 2) {
         errorRoll_d[3] = errorRoll - errorRollOld[3];
         errorRollOld[3] = errorRoll;
         DSetRoll = errorRoll_d[3] * DSens;
      }

      // clip here
      errorRollSum = errorRollSum + errorRoll;       //integrate the above
      if (errorRollSum > 10000) {
         errorRollSum = 10000;
      }
      if (errorRollSum < -10000) {
         errorRollSum = -10000;
      }
      PSetRoll = errorRoll * PSens;      //multiply with gain
      if (gyroIEnable == 0) {            //don't integrate when motors off
         errorRollSum = 0;
      }
      ISetRoll = errorRollSum * ISens;  //multiply with gain


      // Nick
      measNick = getADC(2);                 //see above
      if (NickGyroDir == 0) { 
         measNick = measNick - NickInit;
      } else if (NickGyroDir == 1) { 
         measNick = NickInit - measNick;
      }
      SetpointNick = sEmpf[nickChannel] * LfdynamicNick;
      errorNick = measNick - SetpointNick;


      if (Looper == 0) {                    //this calculates the angular velocity (D-term in acro mode)
         errorNick_d[1] = errorNick - errorNickOld[1];
         errorNickOld[1] = errorNick;
         DSetNick = errorNick_d[1] * DSens;
      } else if (Looper == 1) {
         errorNick_d[2] = errorNick - errorNickOld[2];
         errorNickOld[2] = errorNick;
         DSetNick = errorNick_d[2] * DSens;
      } else if (Looper == 2) {
         errorNick_d[3] = errorNick - errorNickOld[3];
         errorNickOld[3] = errorNick;
         DSetNick = errorNick_d[3] * DSens;
      }


      // clip here
      errorNickSum = errorNickSum + errorNick;
      if (errorNickSum > 10000) {
         errorNickSum = 10000;
      }
      if (errorNickSum < -10000) {
         errorNickSum = -10000;
      }
      PSetNick = errorNick * PSens;
      if (gyroIEnable == 0) { 
         errorNickSum = 0;
      }
      ISetNick = errorNickSum * ISens;
   } // end state 1

   if (state == 2 ||  state == 0) {    // HOVER MODE = Angle control
      if (Looper < 1) {                // for angular acceleration measurement
         Looper += 1;                  // acceleration will be calculated as the difference in velocity between
      } else {                         // loop n and loop n+2
         Looper = 0;
      }

      // Roll
      measRoll = getADC(3);               //get gyro signal
      if (rollGyroDir == 0) {             //make gyro direction reversal possible
         measRoll = RollInit - measRoll;  //subtract offset
      } else if (rollGyroDir == 1) {
         measRoll = measRoll - RollInit;
      }

      if (Looper == 0) {                   //this calculates the angular velocity (D-term in acro mode)
         errorRoll_d[1] = measRoll - errorRollOld[1];
         errorRollOld[1] = measRoll;
         DdSetRoll = errorRoll_d[1] * DdSens;
      } else if (Looper == 1) { 
         errorRoll_d[2] = measRoll - errorRollOld[2];
         errorRollOld[2] = measRoll;
         DdSetRoll = errorRoll_d[1] * DdSens;

      } else if (Looper == 2) { 
         errorRoll_d[3] = measRoll - errorRollOld[3];
         errorRollOld[3] = measRoll;
         DdSetRoll = errorRoll_d[1] * DdSens;
      }


      MeasAngleRoll = MeasAngleRoll + measRoll;       //integrate gyro signal

      /*
         this might require some further explanation:
         The gyroscopes can only measure differences in rotational speed. Integrated over a long time (e.g. 11 minutes of flight)
         the angle calculated from gyro's alone loses precision. Here, the acc comes into play: It always knows the angle of the
         copter, but it reacts pretty slowly. And it contains quite some noise. The following lines of code combine the fast
         signal of the gyroscopes and the absolute precision of the accelerometer. In the end, you get the best out of both worlds:
         */

      MeasAngleRoll = MeasAngleRoll * GyroInfluence;  //0.99 take 0.99 of gyro integral and 0.01 of acc...
      accYTemp = accY * AccInfluence;                 //0.01
      MeasAngleRoll = MeasAngleRoll + accYTemp;       //...and put these two together (complementary filtering)

      setPointRoll = sEmpf[rollChannel] * Lf;         //roll stick position * stick sensitivity
      errorRoll = MeasAngleRoll - setPointRoll;       //current angle minus desired angle (stick position)
      errorRollSum = errorRollSum + errorRoll;        //integral of an integral

      if (errorRollSum > 4000000)  errorRollSum = 4000000;  //integral clipping

      if (errorRollSum < -4000000) errorRollSum = -4000000;


      PSetRoll = errorRoll * PSens;      //multiply with gain
      if (gyroIEnable == 0) errorRollSum = 0;

      ISetRoll = errorRollSum * ISens;  //multiply with gain
      DSetRoll = measRoll * DSens;       //multiply with gain

      //--Nick--
      measNick = getADC(2);                 //see above
      if (NickGyroDir == 0) {
         measNick = measNick - NickInit;
      } else if (NickGyroDir == 1) {
         measNick = NickInit - measNick;
      }

      if (Looper == 0) {                     //this calculates the angular velocity (D-term in acro mode)
         errorNick_d[1] = measNick - errorNickOld[1];
         errorNickOld[1] = measNick;
         DdSetNick = errorNick_d[1] * DdSens;
      } else if (Looper == 1) {
         errorNick_d[2] = measNick - errorNickOld[2];
         errorNickOld[2] = measNick;
         DdSetNick = errorNick_d[1] * DdSens;
      } else if (Looper == 2) {
         errorNick_d[3] = measNick - errorNickOld[3];
         errorNickOld[3] = measNick;
         DdSetNick = errorNick_d[1] * DdSens;
      }

      measAngleNick = measAngleNick + measNick;
      measAngleNick = measAngleNick * GyroInfluence;       //0.99
      accXTemp = accX * AccInfluence;      //0.01
      measAngleNick = measAngleNick + accXTemp;
      SetpointNick = sEmpf[nickChannel] * Lf;
      errorNick = measAngleNick - SetpointNick;
      errorNickSum = errorNickSum + errorNick;

      if (errorNickSum > 4000000)  errorNickSum = 4000000;  //integral clipping
      if (errorNickSum < -4000000) errorNickSum = -4000000;

      PSetNick = errorNick * PSens;
      if (gyroIEnable == 0) errorNickSum = 0;
      ISetNick = errorNickSum * ISens;
      DSetNick = measNick * DSens;
   }

   //--Yaw--
   //new values will be assigned only with the speed that the servo supports.

   if (sHigh == 0) yawGyro = getADC(1);                    //get yaw rate from gyro
   if (yawGyroDir == 0) {
      yawGyro = yawGyro - YawInit;        //subtract offset
   } else {
      yawGyro = YawInit - yawGyro;
   }

   yawDiff = sEmpf[yawChannel] * LfYaw;   //yaw stick position

   if (yawGyroDir == 1) yawDiff = -yawDiff;

   yawDiff = yawDiff - yawGyro;          //stick position - current angular velocity

   yawGyro_i = yawGyro_i + yawDiff;      //integral of the above
   if (yawGyro_i > 32000) yawGyro_i = 32000;
   if (yawGyro_i < -32000) yawGyro_i = -32000;

   yawGyroScale = yawDiff * yawPSens;  //multiply with gain
   if (gyroIEnable == 0) {                //integrate only when motors on
      yawGyro_i = 0;
      yawGyroIScale = yawGyro_i * yawISens;       //multiply with gain
   }
}

// ACCELEROMETER ///////////////////////////////////////////////////////////////
void acc() 
{
  if (state != 1) {             //only used in HOVER MODE and when motors off
    accX = getADC(4);           //read nick "angle"
    accY = getADC(5);           //read roll "angle"
    if (xAccDir == 1) {         //make accX direction reversable
      accX = xAccOffset - accX; //515  these values are the ACC offsets. They were determined in flight (hovering in place)
    } else if (xAccDir == 0) {
      accX = accX - xAccOffset;
    }
    if (yAccDir == 1) {
      accY = yAccOffset - accY; //524
    } else if (yAccDir == 0) {
      accY = accY - yAccOffset;
    }
    accY *= yAccScale;          //120   make the amplitude if gyro_integrals and acc similar
    accX *= xAccScale;          //120   (necessary for successful complementary filtering)
  }
}

// MIXER ///////////////////////////////////////////////////////////////////////
void mixer() 
{
   //--Noise filter--
   //For I = 1 To 5
   for (i=1; i<=5; i++) {
      Meanrx[i] = Meanrx[i] * 3;
      Meanrx[i] = Meanrx[i] + Empf[i];       //"lowpass filter" of the RC signal
      Meanrx[i] = Meanrx[i]>>2;            // (=divide by 4)
      aEmpfh[i] = Meanrx[i] + 17;            //upper acceptable fluctuation
      aEmpfl[i] = Meanrx[i] - 17;            //lower acceptable fluctuation
      if (Empf[i] > aEmpfh[i] || Empf[i] < aEmpfl[i]) {        //compare allowed fluctuation with current rc reading
         Empf[i] = Meanrx[i];                //if fluctuation was too high -> replace with averaged value
         //TODO Print ("Error reading channel: %d\n", i);       //handy to check for RC problems
      }
   }

   //Empf(X) are filled in "getreceiver". They usually contain values ranging from 63 - 137.
   //Empf[throttleChannel] is the throttle stick. it will be rescaled differently.
   if (Empf[throttleChannel] > 61 && Empf[throttleChannel] < 139) {        //don't process values that can't be correct
      sEmpf[throttleChannel] -= 61;
      sEmpf[throttleChannel] *= 3;       //==> values ranging from 3 (stick at bottom) to 228 (full throttle)
   }
   //Now nick, roll, yaw and idle up switch

   if (Empf[nickChannel] > 61 && Empf[nickChannel] < 139) {        //don't process values that can't be correct
      sEmpf[nickChannel] -= 100;       //convert to values ranging from -37 to +37
   }

   if (Empf[rollChannel] > 61 && Empf[rollChannel] < 139) {       //don't process values that can't be correct
      sEmpf[rollChannel] -= 100;       //convert to values ranging from -37 to +37
   }

   if (Empf[yawChannel] > 61 && Empf[yawChannel] < 139) {       //don't process values that can't be correct
      sEmpf[yawChannel] -= 100;       //convert to values ranging from -37 to +37
   }
   if (Empf[5] > 61 && Empf[5] < 139) {   //don't process values that can't be correct
      sEmpf[5] -= 100;              //convert to values ranging from -37 to +37
   }

   //Dynamic LF: Makes the copter react non-linearly to nick. I use this for flying loopings. The more I pull, the faster(exponential) the copter will nick
   if (LfBoost == 1) { 
      if (state == 1) {                     //only when in acro mode
         LookupPosNick = abs(sEmpf[nickChannel]);       //make a variable that grows when stick is out of centre
         LookupPosNick -= 25;
         if (LookupPosNick < 0) LookupPosNick = 0;

         if (LookupPosNick >= 13) LookupPosNick = 13;

         if (LookupPosNick != 0) {
            LfdynamicNick = Lookup(lookupPosNick , Dta);       //look for a new sensitivity factor in a table
         } else {
            LfdynamicNick = Lf;
         }
         LookupPosRoll = abs(sEmpf[rollChannel]);       //make a variable that grows when stick is out of centre
         LookupPosRoll -= 26;
         if (LookupPosRoll < 0) LookupPosRoll = 0;

         if (LookupPosRoll >= 13) LookupPosRoll = 13;

         if (LookupPosRoll != 0) {
            LfdynamicRoll = Lookup(lookupPosRoll , Dta);       //look for a new sensitivity factor in a table
         } else {
            LfdynamicRoll = Lf;
         }
      }
   } else {
      LfdynamicRoll = Lf;
      LfdynamicNick = Lf;
   }

   //ACRO MODE (angular velocity control, ACC = off)
   if (motorsEnable == 1) {                 //only listen to receiver (and especially channel 5) when user enabled the motors in the GUI
      if (sEmpf[5] > -7 && sEmpf[5] < 7) {        //switch in middle = motors on; sEmpf[5] is the idle up switch
         Vorwahl = Idle_up;                   //minimum throttle
         gyroIEnable = 1;                   //start integrating the gyroscope signals
         Lf = LfAcro;                        //5.8                                                  'nick and roll sensitivity
         PSens = PSensAcro;                //0.45                                             'Gyro_P sensitivity roll and nick
         ISens = ISensAcro;                //0.001                                            'Gyro_I Sensitivity roll and nick
         DSens = DSensAcro;                //no D available in Acro mode (calculating it only yields noise)
         yawPSens = yawPSensEEP;         //Gyro_P Sensitivity yaw
         yawISens = yawISensEEP;         //Gyro_I Sensitivity yaw
         motorsOn = 1;
         state = 1;                           //flight mode: acrobatic
         //HOVER MODE (angle control, ACC = on)
      }
      if (sEmpf[5] >= 20 && sEmpf[5] < 39) {       //switch top = motors on
         Vorwahl = Idle_up;                   //minimum throttle
         gyroIEnable = 1;                   //start integrating the gyroscope signals
         Lf = LfHover;                       //450                                                  'nick and roll sensitivity (much bigger, because a different control loop is used)
         PSens = PSensHover;               //0.0025                                           'Gyro_P sensitivity roll and nick (corresponds to Gyro_I in acro mode)
         ISens = ISensHover;               //0.0000015                                        'Gyro_I sensitivity roll and nick (this is a double integral, thats why the value is so low)
         DSens = DSensHover;               //0.4                                              'Gyro_D sensitivity roll and nick (corresponds to Gyro_P in acro mode)
         yawPSens = yawPSensEEP;         //Gyro_P Sensitivity yaw
         yawISens = yawISensEEP;         //Gyro_I Sensitivity yaw
         motorsOn = 1;
         state = 2;                          //flight mode: hover
      }

      if (sEmpf[5] <= -20 && sEmpf[5] > -39) {        //switch bottom = motors off
         if (sEmpf[throttleChannel] < 50) {  //only turn motors off when throttle stick is also on bottom
            Vorwahl = 0;                        //no minimum throttle = motors off
            gyroIEnable = 0;                  //do not integrate gyros anymore
            motorsOn = 0;
            state = 0;                       //flight mode: off
            Lf = 0;                             //don't react to stick movements
            PSens = 0;                         //don't react to gyros
            ISens = 0;                         //don't react to gyros
            DSens = 0;                         //don't react to gyros
            yawPSens = 0;                     //don't react to gyros
            yawISens = 0;                     //don't react to gyros
         }
      }
   } else {                                 //if motors were not enabled in GUI or if EEprom emty: always stay in GUI mode
      Vorwahl = 0;                          //no minimum throttle = motors off
      gyroIEnable = 0;                    //do not integrate gyros anymore
      motorsOn = 0;
      state = 0;                            //flight mode: off
      Lf = 0;                               //don't react to stick movements
      PSens = 0;                           //don't react to gyros
      ISens = 0;                           //don't react to gyros
      DSens = 0;                           //don't react to gyros
      yawPSens = 0;                       //don't react to gyros
      yawISens = 0;                       //don't react to gyros
   }

   //-Mix components-
   if (motorsOn == 1) {                   //only when motors are running
      //-MOTOR 1- (front left)
      Ms_l_i = sEmpf[throttleChannel] + Vorwahl;       //throttle stick (3-228) + Vorwahl(20)
      if (Ms_l_i > 247) {                //throttle clipping
         Ms_l_i = 247;
      }
      Ms_l_i = Ms_l_i + PSetRoll;    //add the gyro measurements (note the differences in "+" and "-")
      Ms_l_i = Ms_l_i + PSetNick;
      Ms_l_i = Ms_l_i + ISetRoll;
      Ms_l_i = Ms_l_i + ISetNick;
      Ms_l_i = Ms_l_i + DSetRoll;
      Ms_l_i = Ms_l_i + DSetNick;
      if (state == 2) {                   //add DD term in hover mode
         Ms_l_i = Ms_l_i + DdSetRoll;
         Ms_l_i = Ms_l_i + DdSetNick;
      }

      //-MOTOR 2- (front right)
      Ms_r_i = sEmpf[throttleChannel] + Vorwahl;
      if (Ms_r_i > 247) Ms_r_i = 247;
      Ms_r_i -= PSetRoll;
      Ms_r_i += PSetNick;
      Ms_r_i -= ISetRoll;
      Ms_r_i += ISetNick;
      Ms_r_i -= DSetRoll;
      Ms_r_i += DSetNick;
      if (state == 2) {
         Ms_r_i -= DdSetRoll;
         Ms_r_i += DdSetNick;
         //}
         //-MOTOR 3-  (back)
         if (Ms_h_i > 247) Ms_h_i = 247;

         Ms_h_i = sEmpf[throttleChannel] + Vorwahl;
         Ms_h_i = Ms_h_i - PSetNick;
         Ms_h_i = Ms_h_i - ISetNick;
         Ms_h_i = Ms_h_i - DSetNick;
         if (state == 2) Ms_h_i -= DdSetNick;

         //when yawing very fast, throttle for the motor at the back is increased
         Tailgas = yawGyroScale + yawGyroIScale;       //if these values are big, the motor at the back was tilted
         Tailgas = Abs(tailgas) / 100;
         Ms_h_i += Tailgas;
   } else {                                    //motors off
      Ms_l_i = 0;
      Ms_r_i = 0;
      Ms_h_i = 0;
   }

   //-Servo control-
   //Servo neutral= 62535. First I add 32535, then I again add 30000 (=62535). This makes sense because I want
   //to protect "servoTail" from overflow.

   if (sHigh == 0) {                        //new values will be assigned only with the speed that the servo supports.
      servoTail = 32535 + yawGyroScale;       //add gyro_P
      servoTail = servoTail + yawGyroIScale;       //add gyro_I
      if (servoTail > 33534) servoTail = 33534;

      if (servoTail < 31536) servoTail = 31536;

      servoTail += 30000;         //add the remaining 30000 for neutral position
   }
}

// MOTORS I2C //////////////////////////////////////////////////////////////////
void sendMots() 
{
   if (Ms_l_i > 255) Ms_l_i = 255;

   if (motorsOn == 1 && Ms_l_i < Vorwahl) Ms_l_i = Vorwahl;

   if (Ms_l_i < 0) Ms_l_i = 0;

   if (Ms_r_i > 255) Ms_r_i = 255;

   if (motorsOn = 1 && Ms_r_i < Vorwahl) Ms_r_i = Vorwahl;

   if (Ms_r_i < 0) Ms_r_i = 0;

   if (Ms_h_i > 255) Ms_h_i = 255;

   if (motorsOn == 1 && Ms_h_i < Vorwahl) Ms_h_i = Vorwahl;

   if (Ms_h_i < 0) Ms_h_i = 0;

   Ms_l = Ms_l_i;                           //convert integer to word
   Ms_h = Ms_h_i;
   Ms_r = Ms_r_i;

   if (Failure < 15 && motorsEnable = 1) {      //if there are NO problems with the receiver and shrediquette was programmed: run motors
      //TODO I2csend(M_h, Ms_h);
      //TODO I2csend(M_r, Ms_r);
      //TODO I2csend(M_l, Ms_l);
   } else {                                    //if there are problems with the receiver: turn off motors
      //TODO I2csend(M_h, 0);
      //TODO I2csend(M_r, 0);
      //TODO I2csend(M_l, 0);
   }
}

// LEDs ////////////////////////////////////////////////////////////////////////
void led() 
{
  if (ledCount < 500) {
    ledCount++;
    if (lowVoltage == 0) {
      if (state > 0) {                    //flight modes 1 & 2 get a different LED signal
        if (ledCount == 130 || ledCount == 140 || ledCount == 230 || ledCount == 240)        //flash LEDs
          toggle(&PORTD, PD_LED1);
      }
      if (ledCount == 100 || ledCount == 110 || ledCount == 200 || ledCount == 210) { 
        toggle(&PORTB, PB_LED2);
      }
      if (ledCount == 160 || ledCount == 170 || ledCount == 260 || ledCount == 270) {
        toggle(&PORTB, PB_LED3);
      }
      if (ledCount == 470) {
        toggle(&PORTB, PB_LED_GREEN);
      }
    } else {                                //state = 0 ==> ready for GUI connection
      if (ledCount == 100 || ledCount == 120 || ledCount == 200 || ledCount == 220 || ledCount == 300 || ledCount == 320) {
        toggle(&PORTD, PD_LED1);
        toggle(&PORTB, PB_LED2);
        toggle(&PORTB, PB_LED3);
        toggle(&PORTB, PB_LED_GREEN);
      }
    }
    if (lowVoltage == 1) {                 //when battery is low, blink the LEDs in an annoying manner
      if (Blinker < 30) { 
        Blinker++;
      } else { 
        Blinker = 1;
        toggle(&PORTD, PD_LED1);
        toggle(&PORTB, PB_LED2);
        toggle(&PORTB, PB_LED3);
      }
      reset(&PORTB, PB_LED_GREEN);
    }

  } else {
    // toggle green led every 500 cycles (use this to determine the speed of the program)
    if (state > 0) {              // ledCount > 500
      set(&PORTB, PB_LED_GREEN);  // Set Led_grn
    } else {
      reset(&PORTB, PB_LED_GREEN);
    }
    ledCount = 1;
    reset(&PORTD, PD_LED1);
    reset(&PORTB, PB_LED2);
    reset(&PORTB, PB_LED3);
  }
}

// VOLTAGE /////////////////////////////////////////////////////////////////////
void voltage() 
{
  if (ledCount == 500) {        //we'll only check voltage every 500 cycles (==> about 1 Hz)
    volt = getADC(0);
    voltAvg = volt + voltOld;   //very unprofessional moving average filter
    voltAvg += voltOld2;
    voltAvg += voltOld3;
    voltAvg /= 4;
    voltOld = volt;
    voltOld2 = voltOld;
    voltOld3 = voltOld2;
    //ADC = 80.775 * voltage - 4.783 //experimentally determinded scale factor of the ADC
    // 10,50 V -> 843
    // 10,40 V -> 835
    // 10,30 V -> 827
    // 10,20 V -> 819
    // 10,10 V -> 811
    // 10,00 V -> 803
    // 9,90 V -> 795
    // 9,80 V -> 787
    // 9,70 V -> 779
    // 9,60 V -> 771
    // 9,50 V -> 763
    // 9,40 V -> 755
    // 9,30 V -> 746
    // 9,20 V -> 738
    if (voltAvg < voltageWarn)  //795 => 9.9V
      lowVoltage = 1;           //set low-voltage bit when voltage is low
  }
}

// FAILSAVE ////////////////////////////////////////////////////////////////////
void failsave()         //will be used to shut down motors if receiption is bad.
{
  // "channel" will be greater than 11 if something with the receiver went wrong
  if (channel >= 11 && Failure < 255)  Failure++;

  //"channel" will be lower than 11 if rx experiences NO problems
  if (channel < 11 && Failure > 0)     Failure--;
}

// GUI /////////////////////////////////////////////////////////////////////////
void GUIconnection()            //RS232 link to the GUI
{
  if (state == 0) {                        //don't listen to serial commands when motors running!
    //TODO replace Ischarwaiting with C equivalent
    if (isCharWaiting() > 0) { 
      //TODO Input A Noecho; // WTF
      ReadEmpfangen = Instr(a , "cr!");
      sensorsEmpfangen = Instr(a , "cs!");
      InputEmpfangen = Instr(a , "ci!");
      ResetEmpfangen = Instr(a , "reset!");
      if (ResetEmpfangen == 1) {
        ResetEmpfangen = 0;
        _delay_ms(100.00);
        //TODO Goto $3c00;                         //$3c00 -> m328p; $1c00 -> m168
      }
      if (ReadEmpfangen == 1) { 
        ReadEmpfangen = 0;
        _delay_ms(25.00);
        //TODO Print "s#p!";                       //tell pc that data will follow
        //TODO Printbin Var[1] ; 33;              //print the list of parameters that can be modified
      }
      if (sensorsEmpfangen == 1) {
        sensorsEmpfangen = 0;
        //TODO Print "ss!";                      //tell pc that sensor data will follow
        // rescale the readings from the sensors to a value ranging from 0 to 255
        // and put these values in an array "sensor(X)"
        measRollGUI = measRoll / 2;
        if (measRollGUI > 127) measRollGUI = 127;

        if (measRollGUI < -127) measRollGUI = -127;

        measRollGUI += 127;
        sensor[1] = measRollGUI;

        MeasAngleRollGUI = MeasAngleRoll / 200;
        if (MeasAngleRollGUI > 127) MeasAngleRollGUI = 127;

        if (MeasAngleRollGUI < -127) MeasAngleRollGUI = -127;

        MeasAngleRollGUI += 127;
        sensor[2] = MeasAngleRollGUI;

        yAccGUI = accY / 200;
        if (yAccGUI > 127) yAccGUI = 127;

        if (yAccGUI < -127) yAccGUI = -127;
        yAccGUI = yAccGUI + 127;
        sensor[3] = yAccGUI;

        measNickGUI = measNick / 2;
        if (measNickGUI > 127) measNickGUI = 127;

        if (measNickGUI < -127) measNickGUI = -127;

        measNickGUI = measNickGUI + 127;
        sensor[4] = measNickGUI;

        measAngleNickGUI = measAngleNick / 200;
        if (measAngleNickGUI > 127) measAngleNickGUI = 127;

        if (measAngleNickGUI < -127) measAngleNickGUI = -127;

        measAngleNickGUI = measAngleNickGUI + 127;
        sensor[5] = measAngleNickGUI;

        xAccGUI = accX / 200;
        if (xAccGUI > 127)  xAccGUI = 127;
        if (xAccGUI < -127) xAccGUI = -127;

        xAccGUI     += 127;
        sensor[6]    = xAccGUI;

        yawGyroGUI   = yawGyro / 2;
        if (yawGyroGUI > 127)  yawGyroGUI = 127;

        if (yawGyroGUI < -127) yawGyroGUI = -127;

        yawGyroGUI  += 127;
        sensor[7]    = yawGyroGUI;

        sEmpfGUI[1]  = sEmpf[throttleChannel] * 1.1;
        sensor[8]    = sEmpfGUI[1];

        sEmpfGUI[2]  = sEmpf[rollChannel] + 39;
        sEmpfGUI[2] *= 3.2;
        sensor[9]    = sEmpfGUI[2];

        sEmpfGUI[3]  = sEmpf[nickChannel] + 39;
        sEmpfGUI[3] *= 3.2;
        sensor[10]   = sEmpfGUI[3];

        sEmpfGUI[4]  = sEmpf[yawChannel] + 39;
        sEmpfGUI[4]  *= 3.2;
        sensor[11]   = sEmpfGUI[4];

        sEmpfGUI[5]  = sEmpf[5] + 39;
        sEmpfGUI[5] *= 3.2;
        sensor[12]   = sEmpfGUI[5];

        VoltageGUI   = voltAvg / 2;
        VoltageGUI  -= 256;
        sensor[13]   = VoltageGUI;

        //TODO Printbin sensor[1] ; 13;
      }
      if (InputEmpfangen == 1) {          //pc wants to transfer parameters
        InputEmpfangen = 0;
        //TODO Inputbin Var[1] , 33;            //read list of parameters
        //TODO Ms_h = 0 : I2csend M_h , Ms_h  //turn off the motors
        //TODO Ms_r = 0 : I2csend M_r , Ms_r
        //TODO Ms_l = 0 : I2csend M_l , Ms_l
        _delay_ms(10.00);
        for(i=1; i<=33; i++) {
          //TODO Writeeeprom Var[i] , i;        //save parameters to eeprom
        }
        _delay_ms(50.00);
        getEeprom();
      }
    }
  }
}

// READ AND PROCESS EEPROM /////////////////////////////////////////////////////
void getEeprom() 
{
  //TODO Ms_h = 0 : I2csend M_h , Ms_h;  //turn off the motors
  //TODO Ms_r = 0 : I2csend M_r , Ms_r;
  //TODO Ms_l = 0 : I2csend M_l , Ms_l;  //load and convert settings from eeprom
  for (i=1; i<=33; i++) {
    Readeeprom Var[i] , i;
  }

convertVars:
  motorsEnable    = Var[1];
  rollGyroDir     = Var[2];
  NickGyroDir     = Var[3];
  yawGyroDir      = Var[4];
  xAccDir         = Var[5];
  yAccDir         = Var[6];
  PSensAcro       = Var[7] / 255;
  ISensAcro       = Var[8] / 25500;
  PSensHover      = Var[9] / 25500;
  ISensHover      = Var[10] / 25500000;  //decrease factor to  12800000
  DSensHover      = Var[11] / 255;
  yawPSensEEP     = Var[12] / 21;
  yawISensEEP     = Var[13] / 2550;
  AccInfluence    = Var[14] / 3000;
  GyroInfluence   = 1 - AccInfluence;
  xAccScale       = Var[15];
  yAccScale       = Var[16];
  LfAcro          = Var[17] / 25.5;
  LfHover         = Var[18] * 4;
  LfYaw           = Var[19] / 17;
  LfBoost         = Var[20];
  Idle_up         = Var[21] / 5;
  voltageWarn     = Var[22] * 4;
  xAccOffset      = Var[23] + 384;
  yAccOffset      = Var[24] + 384;
  throttleChannel = Var[25];
  nickChannel     = Var[26];
  rollChannel     = Var[27];
  yawChannel      = Var[28];
  DSensAcro       = Var[29] / 50;
  DdSens          = Var[30] / 50;

  // If eeprom is empty, vars will be 255.
  // This can cause weird things. So just set all vars to 0.
  if (rollGyroDir > 100 || NickGyroDir > 100 || yawGyroDir > 100 ||
      xAccDir > 100 || yAccDir > 100 || LfBoost > 100 || throttleChannel > 4 ||
      rollChannel > 4 || nickChannel > 4 || yawChannel > 4) { 
    for (i=1; i <= 33; i++) {
      Var[i] = 0;
    }
    Var[25] = 1;
    Var[26] = 3;
    Var[27] = 2;
    Var[28] = 4;
    goto convertVars;           // and reinitialize the variables
  }
}

// SERVO CONTROL ///////////////////////////////////////////////////////////////
//TODO Servoirq:                // generate servo PWM pulse
void genServoPWM()
{
  if (sHigh == 0) {
    Timer1 = servoTail;
    set(&PORTD, PD_SERVO);      // Portd.6 = 1
    sHigh = 1;
  } else {
    reset(&PORTD, PD_SERVO);    // Portd.6 = 0
    sHigh = 0;
    //35535
    Timer1 = 30000;             //after a pulse, pause for 17ms (= 57Hz typically used for servos)
  }
  return;
}

// READ RX /////////////////////////////////////////////////////////////////////
//TODO Getreceiver:                    //falling edge detection
void getReceiver()
{
  if (channel > 0 || channel < 6) {    //fill empf(1-5)
    Empf[channel] = Timer0;
  }
  Timer0 = 6;                          //preload for 4.096 ms
  channel++;                           //if no falling edge was detected for a longer period, channel will increase above 11
  return;                              //that means that there are problems with the receiver
}

//TODO Detectrxpause:
void detectRXpause()
{
  //if (channel != 0) {
  //  Newvalsreceived = 1;
  //}
  channel = 0;
  return;
}

//TODO Dta:                            //table lookup for dynamic lf (increasing maneuvrability when stick is out of the centre)
//TODO Data 5.8! , 6.1! , 6.3! , 6.5! , 6.8! , 7.2! , 7.6! , 8.1! , 8.7! , 9.4! , 10.2! , 11! , 12! , 13.6!


int main()
{
   while (1) {
      acc();
      gyro();
      mixer();
      sendMots();
      led();
      voltage();
      failsave();
      GUIconnection();
   }
   return 0;
}
