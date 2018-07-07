#include "YYJSBuzzer.h"
#include <assert.h>

/*
构造函数
  pin : 蜂鸣器连接的pin口
  beepType  : 蜂鸣器声音的种类
  initBeep  : 构建完成后是否自动开始蜂鸣，默认为否
*/
YYJSBuzzer::YYJSBuzzer(int pin,int beepType,bool initBeep){
  assert(beepType >= 0 && beepType < BEEP_TYPES_COUNT);
  pinMode(pin,OUTPUT);
  buzzerPin = pin;
  beepIndex = 0;
  this->beepType = beepType;
  enableBeep = initBeep;
 
  if(enableBeep){
      digitalWrite(buzzerPin,HIGH);
      curBeepState = true;
      nextUpdatePinTime = millis()+beep_intervals[beepType][beepIndex];
  }else{
      digitalWrite(buzzerPin,LOW);
      curBeepState = false; 
      nextUpdatePinTime = 0; 
  }
  
}

void YYJSBuzzer::startBeep(int beepType){
  if(beepType >= BEEP_TYPES_COUNT)
    return;
  this->beepType = beepType;
  startBeep();  
}

void YYJSBuzzer::startBeep(){
  enableBeep = true;
  curBeepState = true;
  beepIndex = 0;
  digitalWrite(buzzerPin,HIGH);
  nextUpdatePinTime = millis()+beep_intervals[beepType][beepIndex];
}

void YYJSBuzzer::stopBeep(){
  enableBeep = false;
  curBeepState = false;
  beepIndex = 0;
  digitalWrite(buzzerPin,LOW);
  nextUpdatePinTime = 0;
}

void YYJSBuzzer::beep(){
  
  if(!enableBeep || millis() < nextUpdatePinTime)
    return;

  digitalWrite(buzzerPin,curBeepState ? LOW : HIGH);
  curBeepState = !curBeepState;
  
  beepIndex++;
  if(beepIndex >= MAX_INTERVAL_NUM || beep_intervals[beepType][beepIndex] == 0)
      beepIndex = 0;

  nextUpdatePinTime = millis()+beep_intervals[beepType][beepIndex];
  char buf[50]={0};
//  sprintf(buf,"beep:%s,pin:%d",curBeepState ? "on":"off",buzzerPin);
//  Serial.println(buf);
  
}
