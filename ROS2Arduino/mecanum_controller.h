/* Functions and type-defs for PID control.

   Taken mostly from Mike Ferguson's ArbotiX code which lives at:

   http://vanadium-ros-pkg.googlecode.com/svn/trunk/arbotix/
*/

/* PID setpoint info For a Motor */
typedef struct
{
  double TargetTicksPerFrame; // target speed in ticks per frame
  long Encoder;               // encoder count
  long PrevEnc;               // last encoder count

  /*
   * Using previous input (PrevInput) instead of PrevError to avoid derivative kick,
   * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
   */
  int PrevInput; // last input
  // int PrevErr;                   // last error

  /*
   * Using integrated term (ITerm) instead of integrated error (Ierror),
   * to allow tuning changes,
   * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
   */
  // int Ierror;
  int ITerm; // integrated term

  long output; // last motor setting
} SetPointInfo;

// SetPointInfo flPID, frPID;

SetPointInfo frPID, flPID, rrPID, rlPID;

/* PID Parameters */
int Kp = 20;
int Kd = 12;
int Ki = 0;
int Ko = 50;

unsigned char moving = 0; // is the base in motion?

/*
 * Initialize PID variables to zero to prevent startup spikes
 * when turning PID on to start moving
 * In particular, assign both Encoder and PrevEnc the current encoder value
 * See http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
 * Note that the assumption here is that PID is only turned on
 * when going from stop to moving, that's why we can init everything on zero.
 */
void resetPID()
{
  flPID.TargetTicksPerFrame = 0.0;
  flPID.Encoder = readEncoder(FL);
  flPID.PrevEnc = flPID.Encoder;
  flPID.output = MIN_PWM;
  flPID.PrevInput = 0;
  flPID.ITerm = 0;

  frPID.TargetTicksPerFrame = 0.0;
  frPID.Encoder = readEncoder(FR);
  frPID.PrevEnc = frPID.Encoder;
  frPID.output = MIN_PWM;
  frPID.PrevInput = 0;
  frPID.ITerm = 0;

  rlPID.TargetTicksPerFrame = 0.0;
  rlPID.Encoder = readEncoder(RL);
  rlPID.PrevEnc = rlPID.Encoder;
  rlPID.output = MIN_PWM;
  rlPID.PrevInput = 0;
  rlPID.ITerm = 0;

  rrPID.TargetTicksPerFrame = 0.0;
  rrPID.Encoder = readEncoder(RR);
  rrPID.PrevEnc = rrPID.Encoder;
  rrPID.output = MIN_PWM;
  rrPID.PrevInput = 0;
  rrPID.ITerm = 0;
}

/* PID routine to compute the next motor commands */
void doPID(SetPointInfo *p)
{
  long Perror;
  long output;
  int input;

  // Perror = p->TargetTicksPerFrame - (p->Encoder - p->PrevEnc);
  input = fabs(p->Encoder - p->PrevEnc);
  Perror = p->TargetTicksPerFrame - input;

  /*
   * Avoid derivative kick and allow tuning changes,
   * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
   * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
   */
  // output = (Kp * Perror + Kd * (Perror - p->PrevErr) + Ki * p->Ierror) / Ko;
  //  p->PrevErr = Perror;
  output = (Kp * Perror - Kd * (input - p->PrevInput) + p->ITerm) / Ko;
  p->PrevEnc = p->Encoder;

  output += p->output;
  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates
  if (output >= MAX_PWM)
    output = MAX_PWM;
  else if (output <= -MAX_PWM)
    output = -MAX_PWM;
  else
    /*
     * allow turning changes, see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
     */
    p->ITerm += Ki * Perror;

  p->output = output;
  p->PrevInput = input;
}

/* Read the encoder values and call the PID routine */
void updatePID()
{
  /* Read the encoders */
  flPID.Encoder = readEncoder(FL);
  frPID.Encoder = readEncoder(FR);
  rlPID.Encoder = readEncoder(RL);
  rrPID.Encoder = readEncoder(RR);

  /* If we're not moving there is nothing more to do */
  if (!moving)
  {
    /*
     * Reset PIDs once, to prevent startup spikes,
     * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
     * PrevInput is considered a good proxy to detect
     * whether reset has already happened
     */
    if (flPID.PrevInput != 0 || frPID.PrevInput != 0 || rlPID.PrevInput != 0 || rrPID.PrevInput != 0)
      resetPID();
    return;
  }

  /* Compute PID update for each motor */
  doPID(&frPID);
  doPID(&flPID);
  doPID(&rlPID);
  doPID(&rrPID);

  /* Set the motor speeds accordingly */
  /* void setMotorSpeeds(int FLSpeed, int FRSpeed, int RLSpeed, int RRSpeed) */
  setMotorSpeeds(flPID.output, frPID.output, rlPID.output, rrPID.output); // very important to put correct arguments into correct places
}
