/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/
#ifndef MOTOR_DRIVER
#define MOTOR_DRIVER

bool A_WheelDirection;
bool B_WheelDirection;
bool C_WheelDirection;

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int A_WheelSpeed, int B_WheelSpeed, int C_WheelSpeed);

#endif
