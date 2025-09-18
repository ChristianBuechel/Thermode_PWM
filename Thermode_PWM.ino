
//***********************************************************************************
//*********** defines
//***********************************************************************************
// thermode v 3.5 (08/2025)
// thermode v 3.4 (06/2025)
// thermode v 3.3 (2025)
// thermode v 3.2 (2023-07-17)
// thermode v 3.1 (2023-06-22)
// thermode v 3.0 (2023-04-16)
// thermode v 2.8 (2023-04-12)

// thermode v 2.6 (2023-03-20)
// thermode v 2.5 (2021-05-13)
// original author: Christian Büchel
// modifications: Lea Kampermann (v1), Björn Horing (v2.0-v2.4, v3.1-v3.2), Christian Büchel (v2.5-v3.0; v3.3)

#include "Arduino.h"
#include "Thermode_PWM.h"
#include "SerialCommand.h"
#include "fastio.h"
#include <EEPROM.h>

// use C preproc to generate enums and char arrays with the same content
#define ERROR_CODES      \
  C(ERR_NULL)            \
  C(ERR_NO_PARAM)        \
  C(ERR_CMD_NOT_FOUND)   \
  C(ERR_CTC_BIN_WIDTH)   \
  C(ERR_CTC_PULSE_WIDTH) \
  C(ERR_CTC_NOT_INIT)    \
  C(ERR_CTC_FULL)        \
  C(ERR_CTC_EMPTY)       \
  C(ERR_SHOCK_RANGE)     \
  C(ERR_SHOCK_ISI)       \
  C(ERR_BUSY)            \
  C(ERR_DEBUG_RANGE)

#define C(x) x,
enum error_codes
{
  ERROR_CODES N_ERROR_CODES
}; // N_ERROR_CODES gets the number of elements in enum
#undef C
#define C(x) #x,
const char *error_str[] = {ERROR_CODES};
#undef C

// define error codes
#define OK_CODES  \
  C(OK_NULL)      \
  C(OK)           \
  C(OK_READY)     \
  C(OK_MOVE_SLOW) \
  C(OK_MOVE_PREC)

#define C(x) x,
enum ok_codes
{
  OK_CODES N_OK_CODES
}; // N_OK_CODES gets the number of elements in enum
#undef C
#define C(x) #x,
const char *ok_str[] = {OK_CODES};
#undef C

#define A 1 //
#define B 2 //

// TIMER1
//  OC1A = GPIO port PB5 = Arduino Digital Pin D11 Mega2560 --> DOWN blue
//  OC1B = GPIO port PB6 = Arduino Digital Pin D12 Mega2560 --> UP green
#define PB5 5 // pin 11
#define PB6 6 // pin 12

// TIMER3
// OC3A = GPIO port PE3 = Arduino	Digital pin D5 Mega2560  --> PWM shock
#define PE3 3 // pin 5 --> SHOCK

//could do a second shock (or thermode) channel using TIMER5: tbd

// no comments after the following port number #defines are allowed due to macro def !!!
#define ANALOG_PIN 14
#define UP_PIN 12
#define DOWN_PIN 11
#define START_PIN 7
#define LED_PIN 13
#define DIGI_PIN 5
#define PIN8_PIN 8
#define PIN9_PIN 9

#define PIN_PULSE 1 // duration in ms

#define CTC_MAX_N 2500
// max entries for complex time-course (2500 + 1 zero)
// given a max ctc_bin_ms of 500ms we can define ctc_data of 1000s

#define DIGIHI_US 100       // pulse dur in  µs
#define MIN_DIGI_ISI 1100   // in us
#define MAX_DIGI_STIM 30000 // why not?
#define MAX_DIGI_ISI 65000  // in us should allow freqeuncies down to 15 Hz

#define SCK 16E6        // clock at 16 MHz
#define PWMPRESCALER 64 // prescaler for PWM mode

#define MAX_OSP_TIME 4194240 // this is the longest in us the timer can do precisely. Then we switch to ms

// For prescaler = 8, 64, 256, 1024 use OSP_SET_AND_FIRE_LONG(cycles) instead. The "wait" time-waster makes it work!
// #define wait {delayMicroseconds(2);} // Un-comment this for prescaler = 8
// #define wait {delayMicroseconds(5);} // ...for prescaler = 64, make sure we get at least one clock
// #define wait {delayMicroseconds(17);} // ...for prescaler = 256

#define wait       \
  {                \
    _delay_us(65); \
  } // ...for prescaler = 1024

#define OSP_SET_AND_FIRE_LONG_A(cycles) \
  {                                     \
    uint16_t m = 0xffff - (cycles - 1); \
    OCR1A = m;                          \
    wait;                               \
    TCNT1 = m - 1;                      \
  } // for prescaler > 1
#define OSP_SET_AND_FIRE_LONG_B(cycles) \
  {                                     \
    uint16_t m = 0xffff - (cycles - 1); \
    OCR1B = m;                          \
    wait;                               \
    TCNT1 = m - 1;                      \
  }
#define OSP1_INPROGRESS() (TCNT1 > 0)
#define OSP3_INPROGRESS() (TCNT3 > 0)
#define OSP4_INPROGRESS() (TCNT4 > 0)

//***********************************************************************************
//*********** initialize global variables
//***********************************************************************************

const float SWversion = 3.5;

String last_cmd; // last cmd goes here

int32_t cps, prescaler;
uint8_t debug_mode;

uint16_t ctc_bin_ms; // 0 to 500; 500ms with 2500 CTC entries this allows 1250s (~20min) of waveform
volatile int32_t ctc_bin_ticks;
volatile int16_t ctc_data[CTC_MAX_N + 1]; // intwernally we have to add a zero pulse at the end
volatile uint16_t n_ctc, c_ctc;

volatile int32_t count_down_ms;

volatile bool busy_t, busy_d;

volatile uint16_t n_pulse, c_pulse;
int16_t pulse_ms0;
int32_t pulse_tick0;

// complex variables
SerialCommand s_cmd; // The demo SerialCommand object

//***********************************************************************************
//*********** Initialize
//***********************************************************************************

void setup()
{
  busy_d = false;
  busy_t = false;
  debug_mode = 0;
  SET_INPUT(ANALOG_PIN);
  OUT_WRITE(UP_PIN, LOW);    //
  OUT_WRITE(DOWN_PIN, LOW);  //
  OUT_WRITE(START_PIN, LOW); //
  OUT_WRITE(LED_PIN, LOW);   //
  OUT_WRITE(DIGI_PIN, LOW);  //
  OUT_WRITE(PIN8_PIN, LOW);  //
  OUT_WRITE(PIN9_PIN, LOW);  //

  TCCR1B = 0; // Halt counter by setting clock select bits to 0 (No clock source).
  TCCR1A = 0; // Halt counter by setting clock select bits to 0 (No clock source).
  // This keeps anything from happening while we get set up

  TCNT1 = 0x0000; // set counter to zero

  // kill Timer3
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3 = 0;

  // kill Timer4
  TCCR4A = 0;
  TCCR4B = 0;
  TCNT4 = 0;

  Serial.begin(115200);

  s_cmd.addCommand("VER", processVER);
  s_cmd.addCommand("DIAG", processDIAG);
  s_cmd.addCommand("MOVE", processMOVE);
  s_cmd.addCommand("START", processSTART);
  s_cmd.addCommand("SHOCK", processSHOCK);
  s_cmd.addCommand("PULSE8", processPULSE8);
  s_cmd.addCommand("PULSE9", processPULSE9);
  s_cmd.addCommand("GETTIME", processGETTIME);
  s_cmd.addCommand("DEBUG", processDEBUG);
  s_cmd.addCommand("HELP", processHELP);
  s_cmd.addCommand("INITCTC", processINITCTC);
  s_cmd.addCommand("LOADCTC", processLOADCTC);
  s_cmd.addCommand("QUERYCTC", processQUERYCTC);
  s_cmd.addCommand("EXECCTC", processEXECCTC);
  s_cmd.addCommand("FLUSHCTC", processFLUSHCTC);
  s_cmd.addCommand("STATUS", processSTATUS);
  s_cmd.addCommand("STATUS_D", processSTATUS_D);
  s_cmd.addCommand("STATUS_T", processSTATUS_T);
  s_cmd.addCommand("KILL", processKILL);
  s_cmd.addCommand("KILL_D", processKILL_D);
  s_cmd.addCommand("KILL_T", processKILL_T);
  s_cmd.addCommand("READVAS", processREADVAS);
  s_cmd.addCommand("SETID", processSETID);
  s_cmd.addCommand("GETID", processGETID);

  s_cmd.setDefaultHandler(unrecognized); // Handler for command that isn't matched  (says "What?")
}

//***********************************************************************************
//*********** Run
//***********************************************************************************

void loop()
{
  s_cmd.readSerial(); //  parse commands
}

//***********************************************************************************
//*********** Setup Parse handlers
//***********************************************************************************

void processSETID()
// allows to name Thermoino and store ID in EEPROM (set debug level to 2)
{
  char *arg;
  char Buffer[16] = {0};
  last_cmd = "SETID;";
  arg = s_cmd.next(); // Get the next argument from the SerialCommand object buffer
  if (arg != NULL)    // if there is more, take it
  {
    if (debug_mode > 1)
    {
      Serial.println(debug_mode);
      strncpy(Buffer, arg, sizeof(Buffer) - 1);
      Serial.println(Buffer);
      EEPROM.put(0, Buffer);
    }
    else
    {
      Serial.println(F("Not authorized !")); // debug level needs to be 2
    }
  }
}

void processGETID()
// returns name of Thermoino from EEPROM
{
  char *arg;
  char Buffer[16] = {0};
  last_cmd = "GETID;";
  EEPROM.get(0, Buffer);
  Serial.println(Buffer);
}

void processVER()
// returns the FW version of the Thermoino
{
  last_cmd = "VER;";
  Serial.println(SWversion);
}

void processDIAG()
// returns all available commands
{
  char *arg;
  last_cmd = "DIAG;";
  displayStatusSerial();
  Serial.print(F("Available commands:"));
  s_cmd.printCommands();
}

void processDEBUG()
//sets the debug level i.e. verbosity level 2 allows to set a new name
{
  char *arg;
  last_cmd = "DEBUG;";
  uint8_t New;
  arg = s_cmd.next(); // Get the next argument from the SerialCommand object buffer
  if (arg != NULL)    // if there is more, take it
  {
    New = atoi(arg);
    if (check_range(&debug_mode, New, (uint8_t)0, (uint8_t)2)) //setting DEBUG to 2 allows to change the ID in EEprom
      print_ok(OK);
    else
    {
      print_error(ERR_DEBUG_RANGE);
      return;
    }
  }
  else
  {
    print_error(ERR_NO_PARAM);
    return;
  }
}

void processMOVE()
//main thermode command moving temperature up (pos) or down (neg); times are in µs
{
  char *arg;
  int32_t us;
  last_cmd = "MOVE;";

  if ((busy_t) || (OSP1_INPROGRESS()) || (OSP4_INPROGRESS()))
  {
    print_error(ERR_BUSY);
    return;
  }

  arg = s_cmd.next(); // Get the next argument from the SerialCommand object buffer
  if (arg != NULL)    // As long as it exists, take it
  {
    last_cmd = last_cmd + arg;

    us = atol(arg);
    if (abs(us) < MAX_OSP_TIME)
    {
      ramp_temp_prec(us); 
      print_ok(OK_MOVE_PREC);
    }
    else
    {
      if (debug_mode > 0)
      {
        Serial.print(F("Pulse time longer than "));
        Serial.println(MAX_OSP_TIME);
        Serial.println(F("using ramp_temp"));
      }
      ramp_temp(us / 1000);
      print_ok(OK_MOVE_SLOW);
    }
  }
  else
  {
    print_error(ERR_NO_PARAM);
    return;
  }
}

void processPULSE8()
//simple command to create a pulse (PIN_PULSE duration: 1ms) on pin 8 
{
  last_cmd = "PULSE8;";
  // we are delibereatly not checking if stuff is going on see how this works
  OUT_WRITE(PIN8_PIN, HIGH); //
  delay(PIN_PULSE);          // wait
  OUT_WRITE(PIN8_PIN, LOW);  //
  print_ok(OK);
}

void processPULSE9()
//simple command to create a pulse (PIN_PULSE duration: 1ms) on pin 9 
{
  last_cmd = "PULSE9;";
  // we are delibereatly not checking if stuff is going on see how this works
  OUT_WRITE(PIN9_PIN, HIGH); //
  delay(PIN_PULSE);          // wait
  OUT_WRITE(PIN9_PIN, LOW);  //
  print_ok(OK);
}

void processSTART()
//simple command to create a pulse (40ms) to start thermode  
{
  last_cmd = "START;";
  if ((busy_t) || (OSP1_INPROGRESS()) || (OSP4_INPROGRESS()))
  {
    print_error(ERR_BUSY);
    return;
  }
  OUT_WRITE(START_PIN, HIGH); //
  delay(40);                  // wait
  OUT_WRITE(START_PIN, LOW);  //
  print_ok(OK);
}

void processSHOCK()
//create a train of shocks  
{
  char *arg;
  uint32_t New, n_stim, isi;
  last_cmd = "SHOCK;";
  if ((busy_d) || (OSP3_INPROGRESS()))
  {
    print_error(ERR_BUSY);
    return;
  }

  arg = s_cmd.next(); // Get the next argument from the SerialCommand object buffer
  if (arg != NULL)    // As long as it existed, take it
  {
    last_cmd = last_cmd + arg;
    New = atol(arg);

    if (!check_range(&n_stim, New, (uint32_t)0, (uint32_t)MAX_DIGI_STIM))
    {
      print_error(ERR_SHOCK_RANGE);
      return;
    }

    n_pulse = n_stim;
  }
  else
  {
    print_error(ERR_NO_PARAM);
    return;
  }
  arg = s_cmd.next(); // Get the next argument from the SerialCommand object buffer
  if (arg != NULL)
  {
    last_cmd = last_cmd + arg;
    New = atol(arg);
    if (!check_range(&isi, New, (uint32_t)MIN_DIGI_ISI, (uint32_t)MAX_DIGI_ISI))
    {
      print_error(ERR_SHOCK_ISI);
      return;
    }
  }
  else
  {
    isi = MIN_DIGI_ISI;
  }

  if (debug_mode > 0)
  {
    Serial.print(F("Using PWM Timer3 giving "));
    Serial.print(n_stim);
    Serial.print(F(" pulses, every "));
    Serial.print(isi);
    Serial.println(F(" us"));
  }

  // now prepare Timer3 for PWM
  cli();              // stop interrupts
  DDRE |= (1 << PE3); // all outputs go
  // DDRL |= (1 << PL3); for Timer5 and pin PL3 aka pin 46

  TCCR3B = 0;           // same for TCCR3B
  TIFR3 |= (1 << TOV3); // very important

  TIMSK3 = (1 << TOIE3); // interrupt when TCNT3 overflows

  TCCR3A = (1 << COM3A1) + (1 << COM3B1) + (1 << WGM31);
  TCCR3B = (1 << WGM33); // normal mode, phase correct PWM ... don't start yet!

  ICR3 = isi / 8; // ticks

  OCR3A = 12; // pulsewidth 8us * 12 = 96us

  TCNT3 = ICR3 - 1; // initialize counter value
  // TCNT3 = 0xFFFF; // initialize counter value
  busy_d = true;
  c_pulse = 0; // reset pulse counter
  print_ok(OK);
  TCCR3B |= (1 << CS30) | (1 << CS31); // Now start timer with prescaler 64 (res 8us, max dur = 0xFFFF x 8 = 524ms)

  sei(); // enable global interrupts
}

void processGETTIME()
//return Thermoino time in ms  
{
  last_cmd = "GETTIME;";
  Serial.println(millis());
}

void processHELP()
{
  char *arg;
  last_cmd = "HELP;";
  display_help();
  display_error_codes();
}

void processINITCTC()
//initialize CTC storage
{
  char *arg;
  uint16_t tmp;
  last_cmd = "INITCTC;";
  if ((busy_t) || (OSP1_INPROGRESS()) || (OSP4_INPROGRESS()))
  {
    print_error(ERR_BUSY);
    return;
  }

  arg = s_cmd.next(); // Get the next argument from the SerialCommand object buffer
  if (arg != NULL)    // As long as it existed, take it
  {
    last_cmd = last_cmd + arg;

    reset_ctc(); // ALWAYS reset all ctc_data vars (including flushing the loaded ctc_data itself) when initializing

    tmp = atoi(arg);

    if (check_range(&ctc_bin_ms, tmp, (uint16_t)1, (uint16_t)500))
      print_ok(OK);
    else
    {
      reset_ctc();
      print_error(ERR_CTC_BIN_WIDTH);
      return;
    }
  }
  else
  {
    print_error(ERR_NO_PARAM);
    return;
  }
}

void processLOADCTC()
// add an item to the CTC
{
  char *arg;
  int32_t move_ms, t_move_ms;

  last_cmd = "LOADCTC;";
  if ((busy_t) || (OSP1_INPROGRESS()) || (OSP4_INPROGRESS()))
  {
    print_error(ERR_BUSY);
    return;
  }

  if (ctc_bin_ms == 0) // not initialized
  {
    print_error(ERR_CTC_NOT_INIT);
    reset_ctc();
    return;
  }

  if (n_ctc == CTC_MAX_N) // full
  {
    print_error(ERR_CTC_FULL);
    reset_ctc();
    return;
  }
  // get segment duration
  arg = s_cmd.next(); // Get the next argument from the SerialCommand object buffer
  if (arg != NULL)    // As long as it existed, take it
  {
    last_cmd = last_cmd + arg;
    move_ms = atol(arg);
    if (check_range_abs(&t_move_ms, move_ms, (int32_t)0, (int32_t)ctc_bin_ms))
    {
      ctc_data[n_ctc] = t_move_ms;
      n_ctc++; // increment pulse counter CAVE, this is the number of pulses, not the index of the last pulse
      print_ok(OK);
    }
    else
    {
      reset_ctc();
      print_error(ERR_CTC_PULSE_WIDTH);
      return;
    }
  }
  else
  {
    print_error(ERR_NO_PARAM);
    return;
  }
}

void processQUERYCTC()
//return all entries from CTC storage
{
  char *arg;
  uint8_t query_lvl;
  last_cmd = "QUERYCTC;";
  if ((busy_t) || (OSP1_INPROGRESS()) || (OSP4_INPROGRESS()))
  {
    print_error(ERR_BUSY);
    return;
  }
  arg = s_cmd.next(); // Get the next argument from the SerialCommand object buffer
  if (arg != NULL)
    query_lvl = atoi(arg);
  else
    query_lvl = 1;

  if ((query_lvl == 1) && (ctc_bin_ms > 0) && (n_ctc > 0))
  {
    Serial.println(F("Status: READY"));
  }

  if ((query_lvl == 2) && (n_ctc > 0))
  {
    Serial.print(F("ctc_bin_ms: "));
    Serial.println(ctc_bin_ms);
    Serial.print(F("n_ctc: "));
    Serial.println(n_ctc);
  }

  if ((query_lvl == 3) && (n_ctc > 0)) // if required, also return more detailed info
  {
    for (uint16_t p = 0; p < n_ctc; p++)
    {
      Serial.print(p);
      Serial.print(F(" "));
      Serial.println(ctc_data[p]);
    }
  }
}

void processEXECCTC()
//start CTC
{
  last_cmd = "EXECCTC;";

  if ((busy_t) || (OSP1_INPROGRESS()) || (OSP4_INPROGRESS()))
  {
    print_error(ERR_BUSY);
    return;
  }

  if (ctc_bin_ms == 0) // not initialized
  {
    print_error(ERR_CTC_NOT_INIT);
    return;
  }

  if (n_ctc == 0) // no data
  {
    print_error(ERR_CTC_EMPTY);
    return;
  }

  c_ctc = 0;           // first entry
  ctc_data[n_ctc] = 0; // add a zero pulse to the end to circumvent cut-off in ISR
  n_ctc++;             // increment pulse counter

  cli();
  DDRB |= (1 << PB5) | (1 << PB6); // all outputs go

  TCCR1B = 0;            // Stop timer before configuring
  TIFR1 |= (1 << TOV1);  // very important as this is still set from EXECCTC (clear Timer/Counter1 Overflow Flag)
  TIMSK1 = (1 << TOIE1); // interrupt when TCNT1 overflows

  // we use phase correct PWM mode 10 (with ICR1 as TOP)
  TCCR1A = (1 << COM1A1) + (1 << COM1B1) + (1 << WGM11);
  TCCR1B = (1 << WGM13); // normal mode, phase correct PWM ... don't start yet!

  ctc_bin_ticks = SCK / 2 / PWMPRESCALER * (int32_t)ctc_bin_ms / 1000;
  ICR1 = ctc_bin_ticks + 1; // important to set this BEFORE OCR1A/B
                            // +1 hack to allow a pulse width equal ctc_bin_ms (actually ctc_bin_ms is 8 us longer)
  // get the first pulse
  pulse_ms0 = ctc_data[c_ctc];

  c_ctc++; // and increment counter, next ctc_data value will be set in interrupt
  pulse_tick0 = SCK / 2 / PWMPRESCALER * (int32_t)pulse_ms0 / 1000;
  if (pulse_tick0 > 0)
  {
    OCR1B = pulse_tick0;
    OCR1A = 0;
  }
  else
  {
    OCR1B = 0;
    OCR1A = -pulse_tick0;
  }

  TCNT1 = ICR1 - 1; // Should not be zero, because then we will miss first entry as ISR "eats up" ctc_data value
  // ICR1-1 means low latency as OCR1A/B will be updated at ICR1
  busy_t = true;
  // here we could also start the thermode
  TCCR1B |= (1 << CS10) | (1 << CS11); // Now start timer with prescaler 64 (max res 8us, max dur = 0xFFFF x 8 = 524ms)
  print_ok(OK);

  // alternative: prescaler 256 (max res 32us, max dur = 0xFFFF x 32 = 2s)

  sei();
}

void processFLUSHCTC()
//flush CTC storage
{
  last_cmd = "FLUSHCTC;";
  if ((busy_t) || (OSP1_INPROGRESS()) || (OSP4_INPROGRESS()))
  {
    print_error(ERR_BUSY);
    return; // then we're just not ready...
  }
  reset_ctc();
  print_ok(OK);
}

void processSTATUS()
//returns any activity (thermode or digitimer)
{
  last_cmd = "STATUS;";
  if ((busy_t) || (busy_d) || (OSP1_INPROGRESS()) || (OSP3_INPROGRESS()) || (OSP4_INPROGRESS()))
  {
    print_error(ERR_BUSY); // anything going on thermode or digitimer
  }
  else
  {
    print_ok(OK_READY);
  }
}

void processSTATUS_D()
//returns if digitimer is busy
{
  last_cmd = "STATUS_D;";
  if ((busy_d) || (OSP3_INPROGRESS()))
  {
    print_error(ERR_BUSY);
  }
  else
  {
    print_ok(OK_READY);
  }
}
void processSTATUS_T()
//returns if thermode is busy
{
  last_cmd = "STATUS_T;";
  if ((busy_t) || (OSP1_INPROGRESS()) || (OSP4_INPROGRESS()))
  {
    print_error(ERR_BUSY);
  }
  else
  {
    print_ok(OK_READY);
  }
}

void processKILL()
//stop any ongoing activty (thermode and digitimer)
{
  last_cmd = "KILL;";

  // TIMER4 stuff (slow move)
  TCCR4A = 0; // clear all Timer/PWM functionality
  TCCR4B = 0;
  TIMSK4 &= ~(1 << OCIE4A); // disable interrupt
  TCNT4 = 0;                // timer counter to 0

  // TIMER1 stuff (fast move)
  TCCR1A = 0; // clear all Timer/PWM functionality
  TCCR1B = 0;
  PORTB &= ~(1 << PB5);    // probably not necessary set PB5 low
  PORTB &= ~(1 << PB6);    // set PB6 low
  TCNT1 = 0;               //
  TIMSK1 &= ~(1 << TOIE1); // disable interrupt

  OUT_WRITE(DOWN_PIN, LOW); // set ports low
  OUT_WRITE(UP_PIN, LOW);   //
  OUT_WRITE(LED_PIN, LOW);  //

  busy_t = false;

  // TIMER3 stuff (digitimer)
  TCCR3A = 0; // clear all Timer/PWM functionality
  TCCR3B = 0;

  c_pulse = 0;
  TCNT3 = 0;                //
  TIMSK3 &= ~(1 << TOIE3);  // can we disable interrupt when TCNT1 overflows in ISR ??? -> YES
  OUT_WRITE(DIGI_PIN, LOW); //
  busy_d = false;

  print_ok(OK_READY);
}

void processKILL_D()
//stop ongoing activty of digitimer
{
  last_cmd = "KILL_D;";

  // TIMER3 stuff (digitimer)
  TCCR3A = 0; // clear all Timer/PWM functionality
  TCCR3B = 0;

  // PORTE &= ~(1 << PE3); // probably not necessary set PE3 low
  c_pulse = 0;
  TCNT3 = 0;                //
  TIMSK3 &= ~(1 << TOIE3);  // can we disable interrupt when TCNT1 overflows in ISR ??? -> YES
  OUT_WRITE(DIGI_PIN, LOW); //
  busy_d = false;
  print_ok(OK_READY);
}

void processKILL_T()
//stop ongoing activty of thermode
{
  last_cmd = "KILL_T;";

  // TIMER4 stuff (slow move)
  TCCR4A = 0;
  TCCR4B = 0;
  TIMSK4 &= ~(1 << OCIE4A); // can we disable interrupt  in ISR ??? -> YES
  TCNT4 = 0;                // does not help to do a MOVE after EXECCTCPWM...
                            // TIMER1 stuff (fast move)
  TCCR1A = 0;               // clear all Timer/PWM functionality
  TCCR1B = 0;
  PORTB &= ~(1 << PB5); // probably not necessary set PB5 low
  PORTB &= ~(1 << PB6); // set PB6 low
  c_ctc = 0;
  TCNT1 = 0;               // does not help to do a MOVE after EXECCTCPWM...
  TIMSK1 &= ~(1 << TOIE1); // can we disable interrupt when TCNT1 overflows in ISR ??? -> YES

  OUT_WRITE(DOWN_PIN, LOW); //
  OUT_WRITE(UP_PIN, LOW);   //
  OUT_WRITE(LED_PIN, LOW);  //

  busy_t = false;

  print_ok(OK_READY);
}

void processREADVAS()
//experimental setup for a potentiometer to get a VAS reading 
{
  last_cmd = "READVAS;";
  Serial.print(millis());
  Serial.print(F(" "));
  Serial.println(analogRead(ANALOG_PIN));
}

// This gets set as the default handler, and gets called when no other command matches.
void unrecognized(const char *command)
{
  last_cmd = "say what?"; // could print the whole command
  print_error(ERR_CMD_NOT_FOUND);
}

//***********************************************************************************
//*********** Interrupt Service Routines ********************************************
//***********************************************************************************
ISR(TIMER4_COMPA_vect) // for slow ramping up/down
{
  if (count_down_ms < 0)
    count_down_ms++;
  if (count_down_ms > 0)
    count_down_ms--;

  if (count_down_ms == 0) // now we are done
  {
    TCCR4A = 0;
    TCCR4B = 0;
    TIMSK4 &= ~(1 << OCIE4A); // can we disable interrupt  in ISR ??? -> YES
    OUT_WRITE(DOWN_PIN, LOW); //
    OUT_WRITE(UP_PIN, LOW);   //
    OUT_WRITE(LED_PIN, LOW);  //
    TCNT4 = 0;                // does not help to do a MOVE after EXECCTCPWM...
    busy_t = false;
  }
}

ISR(TIMER1_OVF_vect) // for CTC
{
  // where all the magic happens
  // ISR is called when counter has finished (after ctc_bin_ms)
  // we simply set the thresholds to fire the next pulse OCR1A for UP
  // OCR1B for DOWN

  if (c_ctc == n_ctc) // we are done
  {
    TCCR1A = 0; // clear all Timer/PWM functionality
    TCCR1B = 0;
    PORTB &= ~(1 << PB5); // probably not necessary set PB5 low
    PORTB &= ~(1 << PB6); // set PB6 low
    c_ctc = 0;
    TCNT1 = 0;               // does not help to do a MOVE after EXECCTCPWM...
    TIMSK1 &= ~(1 << TOIE1); // can we disable interrupt when TCNT1 overflows in ISR ??? -> YES
    busy_t = false;
    n_ctc--; // get rid of the null pulse (so we can add pulses with loadctc)
    return;
  }

  int16_t pulse_ms = ctc_data[c_ctc];
  c_ctc++;

  int32_t pulse_tick = SCK / 2 / PWMPRESCALER * (int32_t)pulse_ms / 1000;

  if (pulse_tick > 0)
  {
    OCR1B = pulse_tick;
    OCR1A = 0;
  }
  else
  {
    OCR1B = 0;
    OCR1A = -pulse_tick;
  }
}

ISR(TIMER3_OVF_vect) // for digitimer shocks
{
  // we simply count the number of pulses and then stop
  // ISR is called when counter has finished i.e. pulse has been generated

  if (c_pulse == n_pulse - 1) // almost done
    OCR3A = 0;                // create null pulse
  // as we fire ISR on TOP, we cut the last pulse in half therefore we do one more and set the last one to 0

  if (c_pulse == n_pulse) // done
  {
    TCCR3A = 0; // clear all Timer/PWM functionality
    TCCR3B = 0;

    // PORTE &= ~(1 << PE3); // probably not necessary set PE3 low
    c_pulse = 0;
    TCNT3 = 0;               //
    TIMSK3 &= ~(1 << TOIE3); // can we disable interrupt when TCNT1 overflows in ISR ??? -> YES
    busy_d = false;
  }
  c_pulse++;
}

//***********************************************************************************
//*********** helper functions
//***********************************************************************************

void print_error(int8_t error_code)
{
  char buffer[4]; //3 digits and a zero
  sprintf(buffer,"%03d",-error_code);
  Serial.print(buffer);
  if (debug_mode > 0)
  {
    Serial.print(" ");
    Serial.print(error_str[error_code]);
  }
  Serial.println();
}

void print_ok(int8_t ok_code)
{
  char buffer[4]; //3 digits and a zero
  sprintf(buffer,"%03d",ok_code);
  Serial.print(buffer);
  if (debug_mode > 0)
  {
    Serial.print(" ");
    Serial.print(ok_str[ok_code]);
  }
  Serial.println();
}
//***********************************************************************************
//*********** diagnostics, free RAM (i.e. between stack and heap)
//***********************************************************************************
int freeRam()

{
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

//***********************************************************************************
//*********** status display over serial port evoked bey serial command DIAG;
//***********************************************************************************
void displayStatusSerial()
{

  Serial.print(F("+++"));
  Serial.print(F(" V:"));
  Serial.print(SWversion, 1);
  Serial.print(F(" RAM:"));
  Serial.print(freeRam());
  Serial.println(F(" +++"));

  Serial.print(F("Debug level: "));
  Serial.println(debug_mode);

  Serial.print(F("Last serial command: "));
  Serial.println(last_cmd);
}

//***********************************************************************************
//*********** status help over serial port evoked by serial command HELP;
//***********************************************************************************
void display_help()
{
  Serial.print(F("Thermoino "));
  Serial.println(SWversion, 1);
  Serial.print(F("Name: "));
  processGETID();
  Serial.println(F("Only compatible with UseThermoinoPTB.m"));
  Serial.println(F("**************************************"));
  Serial.println(F("Available commands:"));
  Serial.println(F("-------------------"));
  Serial.println(F("VER           - Print version"));
  Serial.println(F("DIAG          - Get diagnostics"));
  Serial.println(F("MOVE;XX       - Move temp up/down for XX us"));
  Serial.println(F("START         - Send 40ms TTL pulse to start thermode"));
  Serial.println(F("SHOCK;nn(;yy) - Digitimer stimuli number (nn) @interval 1100us OR additionally specify interval between pulses (yy) in us (>1000) "));
  Serial.println(F("PULSE8        - Send 1ms TTL pulse to pin 8"));
  Serial.println(F("PULSE9        - Send 1ms TTL pulse to pin 9"));
  Serial.println(F("GETTIME       - Get Arduino time in ms"));
  Serial.println(F("DEBUG;XX      - Set debug state (0: OFF) (1: ON))"));
  Serial.println(F("HELP          - This command"));
  Serial.println(F("INITCTC;xx    - Initialize complex time courses (ctc_data) with cycle xx in ms (500 max)"));
  Serial.print(F("LOADCTC;xx    - add pulse to ctc_data queue (xx in ms) -xx means temperature decrease, max "));
  Serial.print(CTC_MAX_N);
  Serial.println(F(" items"));

  Serial.println(F("QUERYCTC(;yy) - status of the ctc_data queue (yy=3 to get all entries)"));
  Serial.println(F("EXECCTC       - execute ctc_data queue using precise PWM"));
  Serial.println(F("FLUSHCTC      - reset ctc and clear ctc_data queue"));
  Serial.println(F("STATUS        - check whether anything (thermode or digitimer) is running"));
  Serial.println(F("STATUS_D      - check whether digitimer is running"));
  Serial.println(F("STATUS_T      - check whether thermode is running"));
  Serial.println(F("KILL          - stop all activity (thermode and digitimer) "));
  Serial.println(F("KILL_D        - stop activity of digitimer"));
  Serial.println(F("KILL_T        - stop activity of thermode"));
  Serial.println(F("READVAS       - returns time in ms and the reading of the VAS potentiometer 0..1023"));
  Serial.println(F("SETID         - enter new name for thermoino, saved permanently in EEPROM"));
  Serial.println(F("GETID         - return name of thermoino from EEPROM"));
}

void display_error_codes()
{
  Serial.println(F("Error codes:"));
  for (int8_t i = 1; i < N_ERROR_CODES; i++) // skip first error code
  {
    Serial.print(-i);
    Serial.print(F(" "));
    Serial.println(error_str[i]);
  }

  Serial.println(F("OK codes:"));
  for (int8_t i = 1; i < N_OK_CODES; i++) // skip first ok code
  {
    Serial.print(i);
    Serial.print(F(" "));
    Serial.println(ok_str[i]);
  }
}
//***********************************************************************************
//*********** subfunction to reset ctc_data variables
//***********************************************************************************
void reset_ctc()
{
  ctc_bin_ms = 0;
  n_ctc = 0;
  c_ctc = 0;
  memset_volatile(ctc_data, 0, sizeof(ctc_data));
  // reset everything
}

//***********************************************************************************
//*********** Function to ramp up temperature
//***********************************************************************************

void ramp_temp(int32_t ms)
{
  if (debug_mode > 0)
  {
    Serial.print(F("ramp_temp: "));
    Serial.print(ms);
    Serial.println(F("ms"));
  }
  cli();
  count_down_ms = ms;
  TCCR1A = 0;
  TCCR1B = 0;

  TCCR4A = 0;
  TCCR4B = 0;
  TCCR4B |= (1 << WGM42);
  // set Output Compare Register to (250 - 1) ticks
  OCR4A = 0xF9;
  // TCNT4
  TCNT4 = 0;

  // TIMSK4
  // Set Timer Interrupt Mask Register to
  // Clear Timer on Compare channel A for timer 4
  TIFR4 |= (1 << OCF4A); // very important otherwise ISR will immediately be executed
  TIMSK4 |= (1 << OCIE4A);
  busy_t = true;
  if (ms < 0)
  {
    if (debug_mode > 0)
    {
      Serial.print(F("Ramping down:  "));
      Serial.println(-ms);
    }
    OUT_WRITE(LED_PIN, HIGH);  //
    OUT_WRITE(DOWN_PIN, HIGH); //
  }
  if (ms > 0)
  {
    if (debug_mode > 0)
    {
      Serial.print(F("Ramping up:  "));
      Serial.println(ms);
    }
    OUT_WRITE(LED_PIN, HIGH); //
    OUT_WRITE(UP_PIN, HIGH);  //
  }
  TCCR4B |= (1 << CS41) | (1 << CS40); // start clock
  sei();
}

//***********************************************************************************
//*********** Prepare timer 1
//***********************************************************************************

void osp_setup(uint8_t which, int32_t prescaler)
{
  TCCR1B = 0; // Halt counter by setting clock select bits to 0 (No clock source).
  // This keeps anything from happening while we get set up

  TCNT1 = 0; // Start counting at bottom.

  ICR1 = 0; // Set TOP to 0, Mode 14. This effectively keeps us from counting becuase the counter just keeps reseting back to 0.
  // We break out of this by manually setting the TCNT higher than 0, in which case it will count all the way up to MAX
  // and then overflow back to 0 and get locked up again.

  if (which == A)
  {
    OCR1A = 0xffff;
    TCCR1A = (1 << COM1A0) | (1 << COM1A1) | (1 << WGM11); // OC1A=Set on Match, clear on BOTTOM. Mode 14 Fast PWM. p.131
    // Set OC1A to output, pick your board- Uno vs 2560
    // DDRB = (1<<1);     // Set pin to output (Note that OC1A = GPIO port PB1 = Arduino Digital Pin D9 Uno)
    DDRB = (1 << 5); // Set pin to output (Note that OC1A = GPIO port PB5 = Arduino Digital Pin D11 Mega2560)
  }
  else if (which == B)
  {
    OCR1B = 0xffff;
    TCCR1A = (1 << COM1B0) | (1 << COM1B1) | (1 << WGM11); // OC1B=Set on Match, clear on BOTTOM. Mode 14 Fast PWM. p.131
    // Set OC1B to output, pick your board- Uno vs 2560
    // DDRB = (1<<2);     // Set pin to output (Note that OC1B = GPIO port PB2 = Arduino Digital Pin D10 Uno)
    DDRB = (1 << 6); // Set pin to output (Note that OC1B = GPIO port PB6 = Arduino Digital Pin D12 Mega2560)
  }

  else
  {
    Serial.println(F("ERROR only OC1A or OC1B supported"));
  }

  //   (using Chris Hahn's notation here)
  // Prescaler  Setup - Choose one of these, then choose a matching "wait" delay statement below.
  // TCCR1B = (1<<WGM12) | (1<<WGM13) | (1<<CS10); // Prescaler = 1; Start counting now. Max ~4mS
  // TCCR1B = (1<<WGM12) | (1<<WGM13) | (1<<CS11); // Prescaler = 8; Start counting now. Max ~32mS, starts in ~10uS or better
  // TCCR1B = (1<<WGM12) | (1<<WGM13) | (1<<CS10) | (1<<CS11); // Prescaler = 64; Start counting now. Max ~.26 sec, starts in ~20uS or better
  if (prescaler == 256)
  {
    TCCR1B = (1 << WGM12) | (1 << WGM13) | (1 << CS12); // Prescaler = 256; Start counting now. Max ~1.05 sec, starts in ~64uS or better
  }
  else if (prescaler == 1024)
  {
    TCCR1B = (1 << WGM12) | (1 << WGM13) | (1 << CS10) | (1 << CS12); // Prescaler = 1024; Start counting now. Max ~4 sec, starts in ~180uS or better
  }
  else
  {
    Serial.println(F("ERROR only 256 or 1024 supported"));
  }
}

//***********************************************************************************
//*********** Function to ramp up temperature
//***********************************************************************************

void ramp_temp_prec(int32_t o_us) // specifiy in us
{
  int32_t o_tic;
  // Serial.print(F("ramp_temp_prec: "));
  // Serial.print(o_us);
  // Serial.println(F("us"));

  if (abs(o_us) < 1048560) // we can use 256 as a prescaler
  {
    prescaler = 256; // we get up to ~4 s pulses
  }
  else
  {
    prescaler = 1024;
  }

  cps = SCK / prescaler; // tics per second
  // now convert us into tics
  o_tic = (int64_t)o_us * cps / 1000000; // convert from us to tics

  if (o_tic < 0)
  {
    o_tic = abs(o_tic);

    if (debug_mode > 0)
    {
      Serial.print(F("Precision-ramping down:  "));
      Serial.println(o_tic);
    }
    osp_setup(A, prescaler);
    OSP_SET_AND_FIRE_LONG_A(o_tic) // Use this for prescaler > 1!
  }
  else if (o_tic > 0)
  {
    if (debug_mode > 0)
    {
      Serial.print(F("Precision-ramping up:  "));
      Serial.println(o_tic);
    }
    osp_setup(B, prescaler);
    OSP_SET_AND_FIRE_LONG_B(o_tic) // Use this for prescaler > 1!
  }
}

void memset_volatile(volatile void *s, char c, size_t n)
{
  volatile char *p = s;
  while (n-- > 0)
  {
    *p++ = c;
  }
}
