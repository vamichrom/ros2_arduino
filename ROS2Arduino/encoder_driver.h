/* *************************************************************
   Encoder driver function definitions
   ************************************************************ */

#ifdef ARDUINO_ENC_COUNTER

/* Include pin change interrupt library */
#include <PinChangeInterrupt.h>

// Define the pins for each encoder
#define FL_ENCODER_PIN 12 // PCINT4
#define FR_ENCODER_PIN 8  // PCINT0
#define RR_ENCODER_PIN 2
#define RL_ENCODER_PIN 3

// Variables to store the pulse count for each encoder
volatile unsigned long pulseCountRR = 0L;
volatile unsigned long pulseCountRL = 0L;
volatile unsigned long pulseCountFL = 0L;
volatile unsigned long pulseCountFR = 0L;

// Interrupt service routines for each encoder
void pulseRR()
{
  if (reverseRR) pulseCountRR--;
  else pulseCountRR++;
}

void pulseRL()
{
  if (reverseRL) pulseCountRL--;
  else pulseCountRL++;
}

void pulseFR()
{
  if (reverseFR) pulseCountFR--;
  else pulseCountFR++;
}

void pulseFL()
{
  if (reverseFL) pulseCountFL--;
  else pulseCountFL++;
}
#endif

long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();
