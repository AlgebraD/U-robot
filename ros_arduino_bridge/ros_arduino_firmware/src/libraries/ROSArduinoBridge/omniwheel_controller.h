/* Functions and type-defs for PID control.

   Taken mostly from Mike Ferguson's ArbotiX code which lives at:
   
   http://vanadium-ros-pkg.googlecode.com/svn/trunk/arbotix/
*/

#ifndef OMNIWHEEL_CONTROLLER
#define OMNIWHEEL_CONTROLLER

#include "commands.h"

/* PID setpoint info For a Motor */
typedef struct {
  double TargetTicksPerFrame;    // target speed in ticks per frame
  long Encoder;                  // encoder count
  long PrevEnc;                  // last encoder count

  /*
  * Using previous input (PrevInput) instead of PrevError to avoid derivative kick,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  */
  long PrevInput;                // last input
  long PrevErr;                   // last error

  /*
  * Using integrated term (ITerm) instead of integrated error (Ierror),
  * to allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  //int Ierror;
  long ITerm;                    //integrated term

  long output;                    // last motor setting
}
SetPointInfo;

//SetPointInfo leftPID, rightPID;
SetPointInfo A_Wheel_PID,B_Wheel_PID,C_Wheel_PID;

/* PID Parameters */
//int Kp = 20;
//int Kd = 12;
//int Ki = 0;
//int Ko = 50;

int A_Wheel_Kp = 10;
int A_Wheel_Kd = 16;
int A_Wheel_Ki = 0;
int A_Wheel_Ko = 5;

int B_Wheel_Kp = 10;
int B_Wheel_Kd = 16;
int B_Wheel_Ki = 0;
int B_Wheel_Ko = 5;

int C_Wheel_Kp = 10;
int C_Wheel_Kd = 16;
int C_Wheel_Ki = 0;
int C_Wheel_Ko = 5;
unsigned char moving = 0; // is the base in motion?

/*
* Initialize PID variables to zero to prevent startup spikes
* when turning PID on to start moving
* In particular, assign both Encoder and PrevEnc the current encoder value
* See http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
* Note that the assumption here is that PID is only turned on
* when going from stop to moving, that's why we can init everything on zero.
*/
void resetPID(){
//   leftPID.TargetTicksPerFrame = 0.0;
//   leftPID.Encoder = readEncoder(LEFT);
//   leftPID.PrevEnc = leftPID.Encoder;
//   leftPID.output = 0;
//   leftPID.PrevInput = 0;
//   leftPID.ITerm = 0;
//
//   rightPID.TargetTicksPerFrame = 0.0;
//   rightPID.Encoder = readEncoder(RIGHT);
//   rightPID.PrevEnc = rightPID.Encoder;
//   rightPID.output = 0;
//   rightPID.PrevInput = 0;
//   rightPID.ITerm = 0;

   A_Wheel_PID.TargetTicksPerFrame = 0.0;
   A_Wheel_PID.Encoder = readEncoder(A_WHEEL);
   A_Wheel_PID.PrevEnc = A_Wheel_PID.Encoder;
   A_Wheel_PID.output = 0;
   A_Wheel_PID.PrevInput = 0;
   A_Wheel_PID.ITerm = 0;

   B_Wheel_PID.TargetTicksPerFrame = 0.0;
   B_Wheel_PID.Encoder = readEncoder(B_WHEEL);
   B_Wheel_PID.PrevEnc = B_Wheel_PID.Encoder;
   B_Wheel_PID.output = 0;
   B_Wheel_PID.PrevInput = 0;
   B_Wheel_PID.ITerm = 0;

   C_Wheel_PID.TargetTicksPerFrame = 0.0;
   C_Wheel_PID.Encoder = readEncoder(C_WHEEL);
   C_Wheel_PID.PrevEnc = C_Wheel_PID.Encoder;
   C_Wheel_PID.output = 0;
   C_Wheel_PID.PrevInput = 0;
   C_Wheel_PID.ITerm = 0;
}

/* PID routine to compute the next motor commands */
//void doPID(SetPointInfo * p) {
//  long Perror;
//  long output;
//  int input;
//
//  //Perror = p->TargetTicksPerFrame - (p->Encoder - p->PrevEnc);
//  input = p->Encoder - p->PrevEnc;
//  Perror = p->TargetTicksPerFrame - input;
//
//
//  /*
//  * Avoid derivative kick and allow tuning changes,
//  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
//  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
//  */
//  //output = (Kp * Perror + Kd * (Perror - p->PrevErr) + Ki * p->Ierror) / Ko;
//  // p->PrevErr = Perror;
//  output = (Kp * Perror - Kd * (input - p->PrevInput) + p->ITerm) / Ko;
//  p->PrevEnc = p->Encoder;
//
//  output += p->output;
//  // Accumulate Integral error *or* Limit output.
//  // Stop accumulating when output saturates
//  if (output >= MAX_PWM)
//    output = MAX_PWM;
//  else if (output <= -MAX_PWM)
//    output = -MAX_PWM;
//  else
//  /*
//  * allow turning changes, see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
//  */
//    p->ITerm += Ki * Perror;
//
//  p->output = output;
//  p->PrevInput = input;
//}

//void doAWheelPid(SetPointInfo * p){
//  long Perror;
//  long output;
//  long input;
//
//  input = p->Encoder - p->PrevEnc;
//  Perror = p->TargetTicksPerFrame - input;
//  output = (A_Wheel_Kp * Perror - A_Wheel_Kd * (input - p->PrevInput) + p->ITerm) / A_Wheel_Ko;
//  p->PrevEnc = p->Encoder;
//
//  output += p->output;
//  // Accumulate Integral error *or* Limit output.
//  // Stop accumulating when output saturates
//  if( (A_WheelDirection == FORWARDS && output < 0) || (A_WheelDirection == BACKWARDS && output > 0) ) {
//    output = 0;
//  }else{
//    
//    if (output >= MAX_PWM)
//    output = MAX_PWM;
//  else if (output <= -MAX_PWM)
//    output = -MAX_PWM;
//  else
//    p->ITerm += A_Wheel_Ki * Perror;
//  
//  }
//  
//  
//
//  p->output = output;
//  p->PrevInput = input;
//  p->PrevErr = Perror;
//}
//
//void doBWheelPid(SetPointInfo * p){
//  long Perror;
//  long output;
//  long input;
//
//  input = p->Encoder - p->PrevEnc;
//  Perror = p->TargetTicksPerFrame - input;
//  output = (B_Wheel_Kp * Perror - B_Wheel_Kd * (input - p->PrevInput) + p->ITerm) / B_Wheel_Ko;
//  p->PrevEnc = p->Encoder;
//
//  output += p->output;
//  // Accumulate Integral error *or* Limit output.
//  // Stop accumulating when output saturates
//  if( (B_WheelDirection == FORWARDS && output < 0) || (B_WheelDirection == BACKWARDS && output > 0) ) {
//    output = 0;
//  }else{
//    
//    if (output >= MAX_PWM)
//    output = MAX_PWM;
//  else if (output <= -MAX_PWM)
//    output = -MAX_PWM;
//  else
//    p->ITerm += B_Wheel_Ki * Perror;
//  
//  }
//
//  p->output = output;
//  p->PrevInput = input;
//}
//
//void doCWheelPid(SetPointInfo * p){
//  long Perror;
//  long output;
//  long input;
//
//  input = p->Encoder - p->PrevEnc;
//  Perror = p->TargetTicksPerFrame - input;
//  output = (C_Wheel_Kp * Perror - C_Wheel_Kd * (input - p->PrevInput) + p->ITerm) / C_Wheel_Ko;
//  p->PrevEnc = p->Encoder;
//
//  output += p->output;
//  // Accumulate Integral error *or* Limit output.
//  // Stop accumulating when output saturates
//  if( (C_WheelDirection == FORWARDS && output < 0) || (C_WheelDirection == BACKWARDS && output > 0) ) {
//    output = 0;
//  }else{
//    
//    if (output >= MAX_PWM)
//    output = MAX_PWM;
//  else if (output <= -MAX_PWM)
//    output = -MAX_PWM;
//  else
//    p->ITerm += C_Wheel_Ki * Perror;
//  
//  }
//
//  p->output = output;
//  p->PrevInput = input;
//}


void doAWheelPid(SetPointInfo * p){
  long Perror;
  long output;
  int input;

  input = p->Encoder - p->PrevEnc;
  Perror = p->TargetTicksPerFrame - input;
  output = (A_Wheel_Kp * Perror - A_Wheel_Kd * (input - p->PrevInput) + p->ITerm) / A_Wheel_Ko ;
  p->PrevEnc = p->Encoder;

  output += p->output;
  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates
  if (output >= MAX_PWM)
    output = MAX_PWM;
  else if (output <= -MAX_PWM)
    output = -MAX_PWM;
  else
    p->ITerm += A_Wheel_Ki * Perror;

  p->output = output;
  p->PrevInput = input;
  p->PrevErr = Perror;
}

void doBWheelPid(SetPointInfo * p){
  long Perror;
  long output;
  int input;

  input = p->Encoder - p->PrevEnc;
  Perror = p->TargetTicksPerFrame - input;
  output = (B_Wheel_Kp * Perror - B_Wheel_Kd * (input - p->PrevInput) + p->ITerm) / B_Wheel_Ko ;
  p->PrevEnc = p->Encoder;

  output += p->output;
  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates
  if (output >= MAX_PWM)
    output = MAX_PWM;
  else if (output <= -MAX_PWM)
    output = -MAX_PWM;
  else
    p->ITerm += B_Wheel_Ki * Perror;


  p->output = output;
  p->PrevInput = input;
  p->PrevErr = Perror;
}

void doCWheelPid(SetPointInfo * p){
  long Perror;
  long output;
  int input;

  input = p->Encoder - p->PrevEnc;
  Perror = p->TargetTicksPerFrame - input;
  output = (C_Wheel_Kp * Perror - C_Wheel_Kd * (input - p->PrevInput) + p->ITerm) / C_Wheel_Ko ;
  p->PrevEnc = p->Encoder;

  output += p->output;
  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates
  if (output >= MAX_PWM)
    output = MAX_PWM;
  else if (output <= -MAX_PWM)
    output = -MAX_PWM;
  else
    p->ITerm += C_Wheel_Ki * Perror;


  p->output = output;
  p->PrevInput = input;
  p->PrevErr = Perror;
}

/* Read the encoder values and call the PID routine */
void updatePID() {
  /* Read the encoders */
  A_Wheel_PID.Encoder = readEncoder(A_WHEEL);
  B_Wheel_PID.Encoder = readEncoder(B_WHEEL);
  C_Wheel_PID.Encoder = readEncoder(C_WHEEL);
  /* If we're not moving there is nothing more to do */
  if (!moving){
    /*
    * Reset PIDs once, to prevent startup spikes,
    * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
    * PrevInput is considered a good proxy to detect
    * whether reset has already happened
    */
    if (A_Wheel_PID.PrevInput != 0 || B_Wheel_PID.PrevInput != 0 || C_Wheel_PID.PrevInput != 0) 
      resetPID();
    return;
  }

  /* Compute PID update for each motor */
  doAWheelPid(&A_Wheel_PID);
  doBWheelPid(&B_Wheel_PID);
  doCWheelPid(&C_Wheel_PID);

  /* Set the motor speeds accordingly */
  setMotorSpeeds(A_Wheel_PID.output, B_Wheel_PID.output,C_Wheel_PID.output);
}


long readPidIn(int wheel){
  
  long input = 0;
  switch(wheel){
    case A_WHEEL:
      input = A_Wheel_PID.PrevInput;
      break;
    case B_WHEEL:
      input = B_Wheel_PID.PrevInput;
      break;
    case C_WHEEL:
      input = C_Wheel_PID.PrevInput;
      break;
  }  
  return input;
  
}

long readPidOut(int wheel){
  long output = 0;
  switch(wheel){
    case A_WHEEL:
      output = A_Wheel_PID.output;
      break;
    case B_WHEEL:
      output = B_Wheel_PID.output;
      break;
    case C_WHEEL:
      output = C_Wheel_PID.output;
      break;
  }  
  return output;
}

#endif
