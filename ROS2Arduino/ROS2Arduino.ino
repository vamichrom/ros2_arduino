/*********************************************************************
 *  ROSArduinoBridge

    Software License Agreement (BSD License)

    Copyright (c) 2012, Patrick Goebel.
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above
       copyright notice, this list of conditions and the following
       disclaimer in the documentation and/or other materials provided
       with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#define USE_BASE // Enable the base controller code

#define ARDUINO 100

/* Define the motor controller and encoder library you are using */
#ifdef USE_BASE
/* Encoders directly attached to Arduino board */
#define ARDUINO_ENC_COUNTER

/* L298 Motor driver*/
#define L298_MOTOR_DRIVER
#endif

/* Serial port baud rate */
#define BAUDRATE 115200

/* Maximum PWM signal */
#define MAX_PWM 255

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/* Include ABS function */
#include "math.h"

/* Include definition of serial commands */
#include "commands.h"

#ifdef USE_BASE
/* Motor driver function definitions */
#include "motor_driver.h"

/**
 *   Variables for controlling rotation direction of motors:
 *   0 - forward
 *   1 - backward
 *   These definitions should go before #include "encoder_driver.h"
 **/
unsigned char reverseFL = 0;
unsigned char reverseFR = 0;
unsigned char reverseRL = 0;
unsigned char reverseRR = 0;

/* Encoder driver function definitions */
#include "encoder_driver.h"

/* PID parameters and functions */
#include "mecanum_controller.h"

/* Run the PID loop at 1 times per second */
#define PID_RATE 2.50 // Hz

/* Convert the rate into an interval in milliseconds */
const int PID_INTERVAL = 1000 / PID_RATE;

/* Track the next time we make a PID calculation */
unsigned long nextPID = PID_INTERVAL;

/* Stop the robot if it hasn't received a movement command
 in this number of milliseconds */
#define AUTO_STOP_INTERVAL 2000
long lastMotorCommand = AUTO_STOP_INTERVAL;
#endif

/* Variable initialization */

// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
// and now also third and fourth arguments
char argv1[16];
char argv2[16];
char argv3[16];
char argv4[16];

// The arguments converted to integers
long arg1;
long arg2;
long arg3;
long arg4;

/* Clear the current command parameters */
void resetCommand()
{
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  memset(argv3, 0, sizeof(argv3));
  memset(argv4, 0, sizeof(argv4));
  arg1 = 0;
  arg2 = 0;
  arg3 = 0;
  arg4 = 0;
  arg = 0;
  index = 0;
}

/* Run a command.  Commands are defined in commands.h */
int runCommand()
{
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);
  arg3 = atoi(argv3);
  arg4 = atoi(argv4);

  switch (cmd)
  {
  case GET_BAUDRATE:
    Serial.println(BAUDRATE);
    break;

#ifdef USE_BASE
  case READ_ENCODERS:
    Serial.print(readEncoder(FL));
    Serial.print(" ");
    Serial.print(readEncoder(FR));
    Serial.print(" ");
    Serial.print(readEncoder(RL));
    Serial.print(" ");
    Serial.println(readEncoder(RR));
    break;
  case RESET_ENCODERS:
    resetEncoders();
    resetPID();
    Serial.println("OK");
    break;
  case MOTOR_SPEEDS:
    /* Reset the auto stop timer */
    lastMotorCommand = millis();
    /* arg1:FL arg2:FR arg3:RL arg4:RR */
    if (arg1 == 0 && arg2 == 0 && arg3 == 0 && arg4 == 0)
    {
      setMotorSpeeds(0, 0, 0, 0);
      resetPID();
      moving = 0;
    }
    else
      moving = 1;

    // arg1:FL
    if (arg1 < 0)
    {
      arg1 = -arg1;
      reverseFL = 1;
    }
    else
      reverseFL = 0;

    // arg2:FR
    if (arg2 < 0)
    {
      arg2 = -arg2;
      reverseFR = 1;
    }
    else
      reverseFR = 0;

    // arg3:RL
    if (arg3 < 0)
    {
      arg3 = -arg3;
      reverseRL = 1;
    }
    else
      reverseRL = 0;

    // arg4:RR
    if (arg4 < 0)
    {
      arg4 = -arg4;
      reverseRR = 1;
    }
    else
      reverseRR = 0;

    flPID.TargetTicksPerFrame = arg1;
    frPID.TargetTicksPerFrame = arg2;
    rlPID.TargetTicksPerFrame = arg3;
    rrPID.TargetTicksPerFrame = arg4;
    Serial.println("OK");
    break;
  case MOTOR_RAW_PWM:
    /* Reset the auto stop timer */
    lastMotorCommand = millis();
    resetPID();
    moving = 0; // Sneaky way to temporarily disable the PID
    
    // arg1:FL
    if (arg1 < 0)
    {
      arg1 = -arg1;
      reverseFL = 1;
    }
    else
      reverseFL = 0;

    // arg2:FR
    if (arg2 < 0)
    {
      arg2 = -arg2;
      reverseFR = 1;
    }
    else
      reverseFR = 0;

    // arg3:RL
    if (arg3 < 0)
    {
      arg3 = -arg3;
      reverseRL = 1;
    }
    else
      reverseRL = 0;

    // arg4:RR
    if (arg4 < 0)
    {
      arg4 = -arg4;
      reverseRR = 1;
    }
    else
      reverseRR = 0;
    
    setMotorSpeeds(arg1, arg2, arg3, arg4);
    Serial.println("OK");
    break;
  case UPDATE_PID:
    while ((str = strtok_r(p, ":", &p)) != '\0')
    {
      pid_args[i] = atoi(str);
      i++;
    }
    Kp = pid_args[0];
    Kd = pid_args[1];
    Ki = pid_args[2];
    Ko = pid_args[3];
    Serial.println("OK");
    break;
  case GET_PID:
    Serial.print("Kp:");
    Serial.print(Kp);
    Serial.print(";");
    Serial.print("Kd:");
    Serial.print(Kd);
    Serial.print(";");
    Serial.print("Ki:");
    Serial.print(Ki);
    Serial.print(";");
    Serial.print("Ko:");
    Serial.println(Ko);
    break;
#endif
  default:
    Serial.println("Invalid Command");
    break;
  }
}

/* Setup function--runs once at startup. */
void setup()
{
  Serial.begin(BAUDRATE);

// Initialize the motor controller if used */
#ifdef USE_BASE
#ifdef ARDUINO_ENC_COUNTER

  // Encoders' pins to INPUT
  pinMode(RL_ENCODER_PIN, INPUT_PULLUP);
  pinMode(FR_ENCODER_PIN, INPUT_PULLUP);
  pinMode(FL_ENCODER_PIN, INPUT_PULLUP);
  pinMode(RR_ENCODER_PIN, INPUT_PULLUP);

  attachPCINT(digitalPinToPCINT(RL_ENCODER_PIN), pulseRL, CHANGE);
  attachPCINT(digitalPinToPCINT(RR_ENCODER_PIN), pulseRR, CHANGE);
  attachPCINT(digitalPinToPCINT(FL_ENCODER_PIN), pulseFL, CHANGE);
  attachPCINT(digitalPinToPCINT(FR_ENCODER_PIN), pulseFR, CHANGE);

#endif
  initMotorController();
  resetPID();
#endif
}

/* Enter the main loop.  Read and parse input from the serial port
   and run any valid commands. Run a PID calculation at the target
   interval and check for auto-stop conditions.
*/
void loop()
{
  while (Serial.available() > 0)
  {
    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13)
    {
      if (arg == 1)
        argv1[index] = NULL;
      else if (arg == 2)
        argv2[index] = NULL;
      else if (arg == 3)
        argv3[index] = NULL;
      else if (arg == 4)
        argv4[index] = NULL;
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ')
    {
      // Step through the arguments
      if (arg == 0)
        arg = 1;
      else if (arg == 1)
      {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
      else if (arg == 2)
      {
        argv2[index] = NULL;
        arg = 3;
        index = 0;
      }
      else if (arg == 3)
      {
        argv3[index] = NULL;
        arg = 4;
        index = 0;
      }
      continue;
    }
    else
    {
      if (arg == 0)
      {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1)
      {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2)
      {
        argv2[index] = chr;
        index++;
      }
      else if (arg == 3)
      {
        argv3[index] = chr;
        index++;
      }
      else if (arg == 4)
      {
        argv4[index] = chr;
        index++;
      }
    }
  }

// If we are using base control, run a PID calculation at the appropriate intervals
#ifdef USE_BASE
  if (millis() > nextPID)
  {
    updatePID();
    nextPID += PID_INTERVAL;
  }

  // Check to see if we have exceeded the auto-stop interval
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL)
  {
    ;
    setMotorSpeeds(0, 0, 0, 0);
    moving = 0;
  }
#endif
}
