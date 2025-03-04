/***************************************************************
   Motor driver function definitions
   *************************************************************/

#ifdef L298_MOTOR_DRIVER

/* Include shift register library */
#include <ShiftRegister74HC595.h>

// Prepare motor PWM pins
#define FL_MOTOR_SPEED_PIN 5
#define FR_MOTOR_SPEED_PIN 9
#define RR_MOTOR_SPEED_PIN 10
#define RL_MOTOR_SPEED_PIN 11

#endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int FLSpeed, int FRSpeed, int RLSpeed, int RRSpeed);
