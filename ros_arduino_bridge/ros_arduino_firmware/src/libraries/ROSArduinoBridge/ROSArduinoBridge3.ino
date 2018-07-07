/*********************************************************************
 *  ROSArduinoBridge
 
    A set of simple serial commands to control a differential drive
    robot and receive back sensor and odometry data. Default 
    configuration assumes use of an Arduino Mega + Pololu motor
    controller shield + Robogaia Mega Encoder shield.  Edit the
    readEncoder() and setMotorSpeed() wrapper functions if using 
    different motor controller or encoder method.

    Created for the Pi Robot Project: http://www.pirobot.org
    and the Home Brew Robotics Club (HBRC): http://hbrobotics.org
    
    Authors: Patrick Goebel, James Nugen

    Inspired and modeled after the ArbotiX driver by Michael Ferguson
    
    Software License Agreement (BSD License)

    Copyright (c) 2012, Patrick Goebel.
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditionse
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

#define USE_BASE      // Enable the base controller code
#define ARDUINO_ENC_COUNTER
#define USE_SERVOS  // Enable use of PWM servos as defined in servos.h
#define USE_BUZZER
#define USE_TEMP_HUMI

/* Serial port baud rate */
#define BAUDRATE     57600

/* Maximum PWM signal */
#define MAX_PWM        255

#include "definePins.h"
//#include <Scheduler.h>

#ifdef USE_TEMP_HUMI
#include "DHT.h"
DHT dht(TEMP_HUMI_PIN, DHT11);
#endif

#ifdef USE_BUZZER
  #include "YYJSBuzzer.h"
  YYJSBuzzer buzzer(BUZZER_PIN,0);
#endif

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

/* Include definition of serial commands */
#include "commands.h"

/* Sensor functions */
#include "sensors.h"

/* Include servo support if required */
#ifdef USE_SERVOS
//   #include <Servo.h>
//   #include "servos.h"
  #include "AdafruitServoDriver.h"
#endif

#ifdef USE_BASE
  /* Motor driver function definitions */
  #include "motor_driver.h"

  /* Encoder driver function definitions */
  #include "encoder_driver.h"

  /* PID parameters and functions */
//  #include "diff_controller.h"
  #include "omniwheel_controller.h"
  /* Run the PID loop at 30 times per second */
  #define PID_RATE           30     // Hz

  /* Convert the rate into an interval */
  const int PID_INTERVAL = 1000 / PID_RATE;
  
  /* Track the next time we make a PID calculation */
  unsigned long nextPID = PID_INTERVAL;

  /* Stop the robot if it hasn't received a movement command
   in this number of milliseconds */
  #define AUTO_STOP_INTERVAL 200
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
char argv1[48];
char argv2[48];
char argv3[48];

// The arguments converted to integers
long arg1;
long arg2;
long arg3;

/* Clear the current command parameters */
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  memset(argv3, 0, sizeof(argv3));
  arg1 = 0;
  arg2 = 0;
  arg3 = 0;
  arg = 0;
  index = 0;
}

/* Run a command.  Commands are defined in commands.h */
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[12];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);
  arg3 = atoi(argv3);
  
  switch(cmd) {
  case GET_BAUDRATE:
    Serial.println(BAUDRATE);
    break;
  case ANALOG_READ:
    Serial.println(analogRead(arg1));
    break;
  case DIGITAL_READ:
    Serial.println(digitalRead(arg1));
    break;
  case ANALOG_WRITE:
    analogWrite(arg1, arg2);
    Serial.println("OK"); 
    break;
  case DIGITAL_WRITE:
    if (arg2 == 0) digitalWrite(arg1, LOW);
    else if (arg2 == 1) digitalWrite(arg1, HIGH);
    Serial.println("OK"); 
    break;
  case PIN_MODE:
    if (arg2 == 0) pinMode(arg1, INPUT);
    else if (arg2 == 1) pinMode(arg1, OUTPUT);
    Serial.println("OK");
    break;
  case PING:
    Serial.println(Ultrasound(arg1,arg2));
    break;
#ifdef USE_SERVOS
  case SERVO_WRITE:
//    servos[arg1].setTargetPosition(arg2);
    setTargetPosition(arg1,arg2);
    Serial.println("OK");
    break;
  case SERVO_READ:
//    Serial.println(servos[arg1].getServo().read());
      Serial.println(readServoPosition(arg1));
    break;
#endif
    
#ifdef USE_BASE
  case READ_ENCODERS:
//    Serial.print(readEncoder(LEFT));
//    Serial.print(" ");
//    Serial.println(readEncoder(RIGHT));
    Serial.print(readEncoder(A_WHEEL));
    Serial.print(" ");
    Serial.print(readEncoder(B_WHEEL));
    Serial.print(" ");
    Serial.println(readEncoder(C_WHEEL));
    break;
   case RESET_ENCODERS:
    resetEncoders();
    resetPID();
    Serial.println("OK");
    break;
  case MOTOR_SPEEDS:
    /* Reset the auto stop timer */
    lastMotorCommand = millis();
    if (arg1 == 0 && arg2 == 0 && arg3 == 0) {
      setMotorSpeeds(0, 0, 0);
      resetPID();
      moving = 0;
    }else{
      moving = 1;
    }

      A_Wheel_PID.TargetTicksPerFrame = arg1;
      B_Wheel_PID.TargetTicksPerFrame = arg2;
      C_Wheel_PID.TargetTicksPerFrame = arg3;

//      setMotorSpeeds(arg1,arg2,arg3);

//    Serial.print("set speed "); 
//    Serial.print(arg1);
//    Serial.print(" ");
//    Serial.print(arg2);
//    Serial.print(" ");
//    Serial.println(arg3);
      Serial.println("OK");
    break;
  case UPDATE_PID:
    while ((str = strtok_r(p, ":", &p)) != '\0') {
       pid_args[i] = atoi(str);
       i++;
    }

    A_Wheel_Kp = pid_args[0];
    A_Wheel_Kd = pid_args[1];
    A_Wheel_Ki = pid_args[2];
    A_Wheel_Ko = pid_args[3];

    B_Wheel_Kp = pid_args[4];
    B_Wheel_Kd = pid_args[5];
    B_Wheel_Ki = pid_args[6];
    B_Wheel_Ko = pid_args[7];

    C_Wheel_Kp = pid_args[8];
    C_Wheel_Kd = pid_args[9];
    C_Wheel_Ki = pid_args[10];
    C_Wheel_Ko = pid_args[11];
    Serial.println("OK");
    break;
  case READ_PIDIN:
    Serial.print(readPidIn(A_WHEEL));
    Serial.print(" ");
    Serial.print(readPidIn(B_WHEEL));
    Serial.print(" ");
    Serial.println(readPidIn(C_WHEEL));
    break;
  case READ_PIDOUT:
    Serial.print(readPidOut(A_WHEEL));
    Serial.print(" ");
    Serial.print(readPidOut(B_WHEEL));
    Serial.print(" ");
    Serial.println(readPidOut(C_WHEEL));
    break;
#endif
  case BEEP:
    if(arg1 == -1)
      buzzer.stopBeep();
    else
      buzzer.startBeep(arg1);
    Serial.println("OK");
    break;
  case TEMP_HUMI:
#ifdef USE_TEMP_HUMI
  {
      // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    int h = dht.readHumidity();
    // Read temperature as Celsius (the default)
    int t = dht.readTemperature();
    Serial.print(t);
    Serial.print(" ");
    Serial.println(h);
  }
#endif
    break;
  default:
    Serial.println("Invalid Command");
    break;
  }
}

/* Setup function--runs once at startup. */
void setup() {
  Serial.begin(BAUDRATE);

// Initialize the motor controller if used */
#ifdef USE_BASE
  #ifdef ARDUINO_ENC_COUNTER
    initEncoders();
  #endif
  initMotorController();
#endif

/* Attach servos if used */
#ifdef USE_SERVOS
//    int i;
//    for (i = 0; i < N_SERVOS; i++) {
//      servos[i].initServo(
//          servoPins[i],
//          stepDelay[i],
//          servoInitPosition[i]);
//    }
  initServo();
  setTargetPosition(0,75);
  setTargetPosition(1,60);
#endif

#ifdef USE_BUZZER
//  Scheduler.startLoop(beepLoop);
#endif

#ifdef USE_TEMP_HUMI
  dht.begin();
#endif
}

/* Enter the main loop.  Read and parse input from the serial port
   and run any valid commands. Run a PID calculation at the target
   interval and check for auto-stop conditions.
*/
void loop() {
  while (Serial.available() > 0) {
    
    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      else if(arg == 3) argv3[index] = NULL;
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }else if (arg == 2)  {
        argv2[index] = NULL;
        arg = 3;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
      else if (arg == 3) {
        argv3[index] = chr;
        index++;
      }
    }
  }
  
// If we are using base control, run a PID calculation at the appropriate intervals
#ifdef USE_BASE
  if (millis() > nextPID) {
    updatePID();
    nextPID += PID_INTERVAL;
  }
  
  // Check to see if we have exceeded the auto-stop interval
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {;
    setMotorSpeeds(0, 0 ,0);
    moving = 0;
  }
#endif

// Sweep servos
#ifdef USE_SERVOS
//  int i;
//  for (i = 0; i < N_SERVOS; i++) {
//    servos[i].doSweep();
//  }
#endif

#ifdef USE_BUZZER
  buzzer.beep();
#endif


}

#ifdef USE_BUZZER
void beepLoop(){
    buzzer.beep();
}
#endif

