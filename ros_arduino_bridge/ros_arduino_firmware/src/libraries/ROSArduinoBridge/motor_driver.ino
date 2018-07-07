/***************************************************************
   Motor driver definitions
   
   Add a "#elif defined" block to this file to include support
   for a particular motor driver.  Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   *************************************************************/
#include "definePins.h"

#ifdef USE_BASE
   
#ifdef POLOLU_VNH5019
  /* Include the Pololu library */
  #include "DualVNH5019MotorShield.h"

  /* Create the motor driver object */
  DualVNH5019MotorShield drive;
  
  /* Wrap the motor driver initialization */
  void initMotorController() {
    drive.init();
  }

  /* Wrap the drive motor set speed function */
  void setMotorSpeed(int i, int spd) {
    if (i == LEFT) drive.setM1Speed(spd);
    else drive.setM2Speed(spd);
  }

  // A convenience function for setting both motor speeds
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }
#elif defined POLOLU_MC33926
  /* Include the Pololu library */
  #include "DualMC33926MotorShield.h"

  /* Create the motor driver object */
  DualMC33926MotorShield drive;
  
  /* Wrap the motor driver initialization */
  void initMotorController() {
    drive.init();
  }

  /* Wrap the drive motor set speed function */
  void setMotorSpeed(int i, int spd) {
    if (i == LEFT) drive.setM1Speed(spd);
    else drive.setM2Speed(spd);
  }

  // A convenience function for setting both motor speeds
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }
#else

//  #error A motor driver must be selected!
  
  /* Wrap the motor driver initialization */
  void initMotorController() {
//    drive.init();
      pinMode(A_WHEEL_IN1,OUTPUT);
      pinMode(A_WHEEL_IN2,OUTPUT);
      pinMode(A_WHEEL_PWM,OUTPUT);
      pinMode(B_WHEEL_IN1,OUTPUT);
      pinMode(B_WHEEL_IN2,OUTPUT);
      pinMode(B_WHEEL_PWM,OUTPUT);
      pinMode(C_WHEEL_IN1,OUTPUT);
      pinMode(C_WHEEL_IN2,OUTPUT);
      pinMode(C_WHEEL_PWM,OUTPUT);
  }

  /* Wrap the drive motor set speed function */
  void setMotorSpeed(int i, int spd) {

    if(spd > MAX_PWM)
      spd = MAX_PWM;
    else if(spd < -MAX_PWM)
      spd = -MAX_PWM;
      
    if (i == A_WHEEL){

//      A_WheelDirection = spd > 0 ? FORWARDS : BACKWARDS;
      
      if(spd > 0){
        A_WheelDirection = FORWARDS;
        analogWrite(A_WHEEL_PWM,spd);
        digitalWrite(A_WHEEL_IN1,HIGH);
        digitalWrite(A_WHEEL_IN2,LOW);
      }else if(spd < 0){
        A_WheelDirection = BACKWARDS;
        analogWrite(A_WHEEL_PWM,-spd);
        digitalWrite(A_WHEEL_IN1,LOW);
        digitalWrite(A_WHEEL_IN2,HIGH); 
      }else 
        analogWrite(A_WHEEL_PWM,0); 

    }else if(i == B_WHEEL){

//      B_WheelDirection = spd > 0 ? FORWARDS : BACKWARDS;
      
      if(spd > 0){
        B_WheelDirection = FORWARDS;
        analogWrite(B_WHEEL_PWM,spd);
        digitalWrite(B_WHEEL_IN1,HIGH);
        digitalWrite(B_WHEEL_IN2,LOW); 
      }else if(spd < 0){
        B_WheelDirection = BACKWARDS;
        analogWrite(B_WHEEL_PWM,-spd);
        digitalWrite(B_WHEEL_IN1,LOW);
        digitalWrite(B_WHEEL_IN2,HIGH); 
      }else 
        analogWrite(B_WHEEL_PWM,0); 


    }else if(i == C_WHEEL){

//      C_WheelDirection = spd > 0 ? FORWARDS : BACKWARDS;
      
      if(spd > 0){
        C_WheelDirection = FORWARDS;
        analogWrite(C_WHEEL_PWM,spd);
        digitalWrite(C_WHEEL_IN1,HIGH);
        digitalWrite(C_WHEEL_IN2,LOW); 
      }else if(spd < 0){
        C_WheelDirection = BACKWARDS;
        analogWrite(C_WHEEL_PWM,-spd);
        digitalWrite(C_WHEEL_IN1,LOW);
        digitalWrite(C_WHEEL_IN2,HIGH); 
      }else 
        analogWrite(C_WHEEL_PWM,0); 

    }
  }

  // A convenience function for setting both motor speeds
  void setMotorSpeeds(int A_WheelSpeed, int B_WheelSpeed, int C_WheelSpeed) {
    
//    if(A_WheelSpeed > 0)
//      A_WheelDirection = FORWARDS;
//    else if(A_WheelSpeed < 0)
//      A_WheelDirection = BACKWARDS;
//
//    if(B_WheelSpeed > 0)
//      B_WheelDirection = FORWARDS;
//    else if(B_WheelSpeed < 0)
//      B_WheelDirection = BACKWARDS;
//
//    if(C_WheelSpeed > 0)
//      C_WheelDirection = FORWARDS;
//    else if(C_WheelSpeed < 0)
//      C_WheelDirection = BACKWARDS;

    setMotorSpeed(A_WHEEL, A_WheelSpeed);
    setMotorSpeed(B_WHEEL, B_WheelSpeed);
    setMotorSpeed(C_WHEEL, C_WheelSpeed);
  }
  
#endif

#endif
