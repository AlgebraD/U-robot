#ifndef TEXTTOSOUND_H
#define TEXTTOSOUND_H


#include <iostream>
#include <time.h>
#include<curl/curl.h>

using namespace std;

#define ZERO_TTS_PARAM(param) memset(&param,0,sizeof(param))


//tts的参数
struct TTS_PARAM{

    int rdn;           //合成音频数字发音方式
    int volume;        //合成音频的音量
    int pitch;         //合成音频的音调
    int speed;         //合成音频对应的语速
    int sample_rate;   //合成音频采样率
    std::string voice_name;// 合成发音人
    std::string text_encoding; //合成文本编码格式

};

struct BAIDU_TOKEN_INFO{
    string  token;      //调用百度语音功能请求的token字符串
    time_t  expires;    //token的此时间前有效
    string  apiKey;     //token对应的应用apiKey
    string  secretKey;  //token对应的secrectKey
};


//指定使用哪一种tts
enum TTS_TYPE{
    TTS_BAIDU,  //百度TTS
    TTS_XUNFEI  //科大讯飞TTS
};

class TextToSound
{

private:
    TTS_TYPE    ttsType;            //tts的种类
    string      ttsSoundPath;       //tts转换后的文件路径名
    CURL        *curl;              //使用curl实现http请求
    string      httpResponseText;   //使用curl请求http的返回文本
    BAIDU_TOKEN_INFO baidu_token_info;

public:
    TextToSound();

    /**
     * @brief 设置要使用哪一种tts
     * @param ttsType
     */
    void setTypeOfTTS(TTS_TYPE ttsType);

    bool init();
    void uninit();


    /**
     * @brief 该函数执行具体的tts功能
     * @param text  要转换成语音的文本
     * @param param tts的参数，不同厂商提供的tts功能，需要的参数不一样
     * @return 返回文本转换成语音文件的全文件路径
     */
    string tts(string text,TTS_PARAM param);


    void tts_play(string text,TTS_PARAM param);


private:
    int tts_xunfei(const char* src_text, const char* des_path, const char* params);
    int tts_baidu(const char* src_text, const char* des_path, TTS_PARAM param);
};

#endif // TEXTTOSOUND_H
