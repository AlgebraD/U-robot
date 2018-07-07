/* Define single-letter commands that will be sent by the PC over the
   serial link.
*/

#ifndef COMMANDS_H
#define COMMANDS_H

#define ANALOG_READ    'a'
#define GET_BAUDRATE   'b'
#define PIN_MODE       'c'
#define DIGITAL_READ   'd'
#define READ_ENCODERS  'e'
#define MOTOR_SPEEDS   'm'
#define PING           'p'
#define RESET_ENCODERS 'r'
#define SERVO_WRITE    's'
#define SERVO_READ     't'
#define UPDATE_PID     'u'
#define DIGITAL_WRITE  'w'
#define ANALOG_WRITE   'x'
#define READ_PIDIN    'i'
#define READ_PIDOUT   'f'
#define BEEP          'B' 
#define TEMP_HUMI     'T' //获取温湿度

#define ENC_TEST  'z'

#define A_WHEEL         0
#define B_WHEEL         1
#define C_WHEEL         2
#define FORWARDS    true
#define BACKWARDS   false

#endif


