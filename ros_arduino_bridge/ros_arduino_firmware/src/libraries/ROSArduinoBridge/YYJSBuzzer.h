#ifndef YYJSBUZZER
#define YYJSBUZZER

#define BEEP_TYPES_COUNT  3
#define MAX_INTERVAL_NUM 10
int beep_intervals[BEEP_TYPES_COUNT][MAX_INTERVAL_NUM] = {{200,800},{200,100},{100,100,100,500}};//每一个beep数组的元素都是时间值，单位为毫秒，按“响、停、响、停……”的顺序排列，个数为偶数。beep函数会按照这里设定的值循环蜂鸣


class YYJSBuzzer{

private:
  int   buzzerPin;          //蜂鸣器pin口
  unsigned long   nextUpdatePinTime;  //下一次更新pin口电平的时间
  bool  curBeepState;      //当前是否正在蜂鸣,true为蜂鸣状态，也就是pin口为高电平，false为停止状态
  bool  enableBeep;        //是否启用蜂鸣
  int   beepIndex;         //beep_intervals第二维数组中的索引，表示当前正处理蜂鸣器的哪个时间段
  int   beepType;          //哪一个蜂鸣效果，beep_intervals的第一维数组索引
  
public:
  YYJSBuzzer(int pin,int beepType,bool initBeep=false);
  void startBeep(int beepType);
  void startBeep();
  void stopBeep();
  void beep();
  
};


#endif
