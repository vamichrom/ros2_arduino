/* *************************************************************
   Encoder definitions
   ************************************************************ */

#ifdef USE_BASE

#ifdef ARDUINO_ENC_COUNTER

/* Wrap the encoder reading function */
long readEncoder(int i)
{
  switch (i)
  {
  case RL:
    return pulseCountRL;
    break;
  case RR:
    return pulseCountRR;
    break;
  case FR:
    return pulseCountFR;
    break;
  case FL:
    return pulseCountFL;
    break;
  }
}

/* Wrap the encoder reset function */
void resetEncoder(int i)
{
  switch (i)
  {
  case RL:
    pulseCountRL = 0L;
    break;
  case RR:
    pulseCountRR = 0L;
    break;
  case FR:
    pulseCountFR = 0L;
    break;
  case FL:
    pulseCountFL = 0L;
    break;
  }
}

#endif

/* Wrap the encoder reset function */
void resetEncoders()
{
  resetEncoder(FL);
  resetEncoder(FR);
  resetEncoder(RL);
  resetEncoder(RR);
}

#endif
