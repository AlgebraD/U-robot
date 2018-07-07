#ifndef DEFINEPINS
#define DEFINEPINS

#ifdef ARDUINO_ENC_COUNTER
  //below can be changed, but should be PORTD pins; 
  //otherwise additional changes in the code are required
  #define A_WHEEL_ENC_PIN_A 2  //pin 2 -- interrupt 0
  #define A_WHEEL_ENC_PIN_B 3  //pin 3 -- interrupt 1

  #define B_WHEEL_ENC_PIN_A 21  //pin 21 -- interrupt 2
  #define B_WHEEL_ENC_PIN_B 20 //pin 20 -- interrupt 3
  
  #define C_WHEEL_ENC_PIN_A 19  //pin 19 -- interrupt 4
  #define C_WHEEL_ENC_PIN_B 18  //pin 18 -- interrupt 5

#endif

#ifdef USE_BASE
  #define A_WHEEL_IN1   5
  #define A_WHEEL_IN2   4
  #define A_WHEEL_PWM   6
  
  #define B_WHEEL_IN1   8
  #define B_WHEEL_IN2   7
  #define B_WHEEL_PWM   9
  
  #define C_WHEEL_IN1   11
  #define C_WHEEL_IN2   10
  #define C_WHEEL_PWM   12
#endif

  #define ULTRASONIC_BOTTOM_PIN 35  //底层超声波pin口
  #define ULTRASONIC_MIDDLE_PIN 37  //中间层超声波pin口
  #define BUZZER_PIN 39 //蜂鸣器pin口
  #define TEMP_HUMI_PIN 41  //温湿度传感器PIN口
#endif
