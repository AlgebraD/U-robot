/* *************************************************************
   Encoder definitions
   
   Add an "#ifdef" block to this file to include support for
   a particular encoder board or library. Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   ************************************************************ */
#include "definePins.h"
#include "motor_driver.h"

#ifdef USE_BASE

#ifdef ROBOGAIA
  /* The Robogaia Mega Encoder shield */
  #include "MegaEncoderCounter.h"

  /* Create the encoder shield object */
  MegaEncoderCounter encoders = MegaEncoderCounter(4); // Initializes the Mega Encoder Counter in the 4X Count mode
  
  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == LEFT) return encoders.YAxisGetCount();
    else return encoders.XAxisGetCount();
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT) return encoders.YAxisReset();
    else return encoders.XAxisReset();
  }
#elif defined(ARDUINO_ENC_COUNTER)
   
//  static const int8_t ENC_STATES [] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};  //encoder lookup table
//    
//  /* Interrupt routine for LEFT encoder, taking care of actual counting */
//  ISR (PCINT2_vect){
//  	static uint8_t enc_last=0;
//        
//	enc_last <<=2; //shift previous state two places
//	enc_last |= (PIND & (3 << 2)) >> 2; //read the current state into lowest 2 bits
//  
//  	left_enc_pos += ENC_STATES[(enc_last & 0x0f)];
//  }
//  
//  /* Interrupt routine for RIGHT encoder, taking care of actual counting */
//  ISR (PCINT1_vect){
//        static uint8_t enc_last=0;
//          	
//	enc_last <<=2; //shift previous state two places
//	enc_last |= (PINC & (3 << 4)) >> 4; //read the current state into lowest 2 bits
//  
//  	right_enc_pos += ENC_STATES[(enc_last & 0x0f)];
//  }


  volatile long a_enc_pos = 0L;
  volatile long b_enc_pos = 0L;
  volatile long c_enc_pos = 0L;
   
  //A轮A相中断
  void A_WheelEncoderA_ISR(){
//      int a = digitalRead(A_WHEEL_ENC_PIN_A);
//      int b = digitalRead(A_WHEEL_ENC_PIN_B);
//      if(a == b)//如果a、b相电位相同，反转，否则正转(A相比B相超前90度)
//        a_enc_pos--;  
//      else
//        a_enc_pos++;
//      Serial.println("A a:");
//      Serial.print(a);
//      Serial.print(" ");
//      Serial.println(b);
      if(A_WheelDirection == FORWARDS)
        a_enc_pos++;
      else
        a_enc_pos--;
    
      
  }

  //A轮B相中断
  void A_WheelEncoderB_ISR(){
//      int a = digitalRead(A_WHEEL_ENC_PIN_A);
//      int b = digitalRead(A_WHEEL_ENC_PIN_B);
//      if(a == b)//如果a、b相电位相同，正转，否则反转
//        a_enc_pos++;  
//      else
//        a_enc_pos--;
//        Serial.println("A b:");
//      Serial.print(a);
//      Serial.print(" ");
//      Serial.println(b);
      if(A_WheelDirection == FORWARDS)
        a_enc_pos++;
      else
        a_enc_pos--;
  }

  //B轮A相中断
  void B_WheelEncoderA_ISR(){
//      int a = digitalRead(B_WHEEL_ENC_PIN_A);
//      int b = digitalRead(B_WHEEL_ENC_PIN_B);
//      if(a == b)//如果a、b相电位相同，反转，否则正转
//        b_enc_pos--;  
//      else
//        b_enc_pos++;
//     Serial.println("B a:");
    if(B_WheelDirection == FORWARDS)
      b_enc_pos++;
    else
      b_enc_pos--;
  }

  //B轮B相中断
  void B_WheelEncoderB_ISR(){
//      int a = digitalRead(B_WHEEL_ENC_PIN_A);
//      int b = digitalRead(B_WHEEL_ENC_PIN_B);
//      if(a == b)//如果a、b相电位相同，正转，否则反转
//        b_enc_pos++;  
//      else
//        b_enc_pos--;
//      Serial.println("B b:");
      if(B_WheelDirection == FORWARDS)
        b_enc_pos++;
      else
        b_enc_pos--;

  }

  //C轮A相中断
  void C_WheelEncoderA_ISR(){
//      int a = digitalRead(C_WHEEL_ENC_PIN_A);
//      int b = digitalRead(C_WHEEL_ENC_PIN_B);
//      if(a == b)//如果a、b相电位相同，反转，否则正转
//        c_enc_pos--;  
//      else
//        c_enc_pos++;
//      Serial.println("C a:");
      if(C_WheelDirection == FORWARDS)
        c_enc_pos++;
      else
        c_enc_pos--;
  }

  //C轮B相中断
  void C_WheelEncoderB_ISR(){
//      int a = digitalRead(C_WHEEL_ENC_PIN_A);
//      int b = digitalRead(C_WHEEL_ENC_PIN_B);
//      if(a == b)//如果a、b相电位相同，正转，否则反转
//        c_enc_pos++;  
//      else
//        c_enc_pos--;
//      Serial.println("C b:");
      if(C_WheelDirection == FORWARDS)
        c_enc_pos++;
      else
        c_enc_pos--;
  }
    
  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == A_WHEEL)     return a_enc_pos;
    else if(i == B_WHEEL) return b_enc_pos;
    else if(i == C_WHEEL) return c_enc_pos;
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == A_WHEEL)     a_enc_pos = 0L;
    else if(i == B_WHEEL) b_enc_pos = 0L;
    else if(i == C_WHEEL) c_enc_pos = 0L;
  }
  
#else
  #error A encoder driver must be selected!
#endif

/* Wrap the encoder reset function */
void resetEncoders() {
  resetEncoder(A_WHEEL);
  resetEncoder(B_WHEEL);
  resetEncoder(C_WHEEL);
}

void initEncoders(){
  pinMode(A_WHEEL_ENC_PIN_A,INPUT);  
  pinMode(A_WHEEL_ENC_PIN_B,INPUT);
  pinMode(B_WHEEL_ENC_PIN_A,INPUT);
  pinMode(B_WHEEL_ENC_PIN_B,INPUT);
  pinMode(C_WHEEL_ENC_PIN_A,INPUT);
  pinMode(C_WHEEL_ENC_PIN_B,INPUT);

//  attachInterrupt(digitalPinToInterrupt(A_WHEEL_ENC_PIN_A),A_WheelEncoderA_ISR,RISING);
//  attachInterrupt(digitalPinToInterrupt(A_WHEEL_ENC_PIN_B),A_WheelEncoderB_ISR,RISING);
//  attachInterrupt(digitalPinToInterrupt(B_WHEEL_ENC_PIN_A),B_WheelEncoderA_ISR,RISING);
//  attachInterrupt(digitalPinToInterrupt(B_WHEEL_ENC_PIN_B),B_WheelEncoderB_ISR,RISING);
//  attachInterrupt(digitalPinToInterrupt(C_WHEEL_ENC_PIN_A),C_WheelEncoderA_ISR,RISING);
//  attachInterrupt(digitalPinToInterrupt(C_WHEEL_ENC_PIN_B),C_WheelEncoderB_ISR,RISING);

//  attachInterrupt(0,A_WheelEncoderA_ISR,RISING);
  attachInterrupt(1,A_WheelEncoderB_ISR,RISING);
//  attachInterrupt(2,B_WheelEncoderA_ISR,RISING);
  attachInterrupt(4,B_WheelEncoderB_ISR,RISING);
//  attachInterrupt(4,C_WheelEncoderA_ISR,RISING);
  attachInterrupt(5,C_WheelEncoderB_ISR,RISING);
}

#endif

