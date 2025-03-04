/***************************************************************
   Motor driver definitions
   *************************************************************/

// macro for setting certain bits
#define MASK(length, position) (~(0xffffffff << (length)) << (position))
#define SET_BITS(value, length, position, new_value) \
  ((value) & ~MASK(length, position)) | (((new_value) << (position)) & MASK(length, position))

#ifdef USE_BASE

#ifdef L298_MOTOR_DRIVER

// create a global shift register object
// parameters: <number of shift registers> (data pin, clock pin, latch pin)
ShiftRegister74HC595<1> sr(6, 4, 7);

uint8_t rotationDirections[] = {B01010101}; // { RL RR FR FL } all wheels forward

void initMotorController()
{
  // PWM pins to OUTPUT
  pinMode(FL_MOTOR_SPEED_PIN, OUTPUT);
  pinMode(FR_MOTOR_SPEED_PIN, OUTPUT);
  pinMode(RR_MOTOR_SPEED_PIN, OUTPUT);
  pinMode(RL_MOTOR_SPEED_PIN, OUTPUT);
}

// void setMotorSpeed(motor, speed)
void setMotorSpeed(int i, int spd)
{
  if (spd > 255)
    spd = 255;
  switch (i)
  {
  case FL:
    switch (reverseFL)
    {
    case 0:
      // Move FL forward at certain speed
      rotationDirections[0] = SET_BITS(rotationDirections[0], 2, 0, 1); // FL forward
      analogWrite(FL_MOTOR_SPEED_PIN, spd);                             // at spd speed
      sr.setAll(rotationDirections);
      break;
    case 1:
      // Move FL backward at certain speed
      rotationDirections[0] = SET_BITS(rotationDirections[0], 2, 0, 2); // FL backward
      analogWrite(FL_MOTOR_SPEED_PIN, spd);                             // at spd speed
      sr.setAll(rotationDirections);
      break;
    }
    break;
  case FR:
    switch (reverseFR)
    {
    case 0:
      // Move FR forward at certain speed
      rotationDirections[0] = SET_BITS(rotationDirections[0], 2, 2, 1); // FR forward
      analogWrite(FR_MOTOR_SPEED_PIN, spd);                             // at spd speed
      sr.setAll(rotationDirections);
      break;
    case 1:
      // Move FR backward at certain speed
      rotationDirections[0] = SET_BITS(rotationDirections[0], 2, 2, 2); // FR backward
      analogWrite(FR_MOTOR_SPEED_PIN, spd);                             // at spd speed
      sr.setAll(rotationDirections);
      break;
    }
    break;
  case RR:
    switch (reverseRR)
    {
    case 0:
      // Move RR forward at certain speed
      rotationDirections[0] = SET_BITS(rotationDirections[0], 2, 4, 1); // RR forward
      analogWrite(RR_MOTOR_SPEED_PIN, spd);                             // at spd speed
      sr.setAll(rotationDirections);
      break;
    case 1:
      // Move RR backward at certain speed
      rotationDirections[0] = SET_BITS(rotationDirections[0], 2, 4, 2); // RR backward
      analogWrite(RR_MOTOR_SPEED_PIN, spd);                             // at spd speed
      sr.setAll(rotationDirections);
      break;
    }
    break;
  case RL:
    switch (reverseRL)
    {
    case 0:
      // Move RL forward at certain speed
      rotationDirections[0] = SET_BITS(rotationDirections[0], 2, 6, 1); // RL forward
      analogWrite(RL_MOTOR_SPEED_PIN, spd);                             // at spd speed
      sr.setAll(rotationDirections);
      break;
    case 1:
      // Move RL backward at certain speed
      rotationDirections[0] = SET_BITS(rotationDirections[0], 2, 6, 2); // RL backward
      analogWrite(RL_MOTOR_SPEED_PIN, spd);                             // at spd speed
      sr.setAll(rotationDirections);
      break;
    }
    break;
  }
}

void setMotorSpeeds(int FLSpeed, int FRSpeed, int RLSpeed, int RRSpeed)
{
  setMotorSpeed(FR, FRSpeed);
  setMotorSpeed(FL, FLSpeed);
  setMotorSpeed(RR, RRSpeed);
  setMotorSpeed(RL, RLSpeed);
}

#endif
#endif
