/* Define single-letter commands that will be sent by the PC over the
   serial link.
*/

#ifndef COMMANDS_H
#define COMMANDS_H

#define GET_BAUDRATE    'b'
#define READ_ENCODERS   'e'
#define MOTOR_SPEEDS    'm'
#define MOTOR_RAW_PWM   'o'
#define RESET_ENCODERS  'r'
#define UPDATE_PID      'u'
#define GET_PID         'v' // view PID values
#define FL              0   // just the number of motors
#define FR              1
#define RL              2
#define RR              3

#endif
