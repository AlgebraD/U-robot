#include "texttosound.h"

#include "qtts.h"
#include "msp_cmn.h"
#include "msp_errors.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <ros/ros.h>
#include "json.hpp"
#include <sys/time.h>
//#include <unistd.h>

using json = nlohmann::json;

int curlHttpResponseWriter(char *data, size_t size, size_t nmemb,
                  std::string *writerData);

/* wav音频头部格式 */
typedef struct _wave_pcm_hdr
{
    char            riff[4];                // = "RIFF"
    int		size_8;                 // = FileSize - 8
    char            wave[4];                // = "WAVE"
    char            fmt[4];                 // = "fmt "
    int		fmt_size;		// = 下一个结构体的大小 : 16

    short int       format_tag;             // = PCM : 1
    short int       channels;               // = 通道数 : 1
    int		samples_per_sec;        // = 采样率 : 8000 | 6000 | 11025 | 16000
    int		avg_bytes_per_sec;      // = 每秒字节数 : samples_per_sec * bits_per_sample / 8
    short int       block_align;            // = 每采样点字节数 : wBitsPerSample / 8
    short int       bits_per_sample;        // = 量化比特数: 8 | 16

    char            data[4];                // = "data";
    int		data_size;              // = 纯数据长度 : FileSize - 44
} wave_pcm_hdr;

/* 默认wav音频头部数据 */
wave_pcm_hdr default_wav_hdr =
{
    { 'R', 'I', 'F', 'F' },
    0,
    {'W', 'A', 'V', 'E'},
    {'f', 'm', 't', ' '},
    16,
    1,
    1,
    16000,
    32000,
    2,
    16,
    {'d', 'a', 't', 'a'},
    0
};


TextToSound::TextToSound():curl(NULL)
{

}


void TextToSound::setTypeOfTTS(TTS_TYPE ttsType){
    this->ttsType = ttsType;
}

bool TextToSound::init(){

    if(ttsType == TTS_XUNFEI){

        /* 用户登录 */
        // const char* login_params = "appid = 59450ad8, work_dir = .";//登录参数,appid与msc库绑定,请勿随意改动 //youyou_robot
        // const char* login_params = "appid = 56ee43d0, work_dir = .";//树莓派
        const char* login_params = "appid = 59c9cf37, work_dir = .";//fala_api

        int ret = MSPLogin(NULL, NULL, login_params);//第一个参数是用户名，第二个参数是密码，第三个参数是登录参数，用户名和密码可在http://www.xfyun.cn注册获取
        if (MSP_SUCCESS != ret)
        {
            printf("MSPLogin failed, error code: %d.\n", ret);
            return false;
        }

    }else if(ttsType == TTS_BAIDU){

        //初始化设置curl网络请求库
        curl = curl_easy_init();        
        if(curl == NULL)
            return false;
        CURLcode code;
        code = curl_easy_setopt(curl, CURLOPT_ERRORBUFFER, &httpResponseText);
        if(code != CURLE_OK)
            return false;

        baidu_token_info.expires    = 0;    //初始化token信息
        baidu_token_info.apiKey     = "";
        baidu_token_info.secretKey  = "";
    }

    return true;
}

void TextToSound::uninit(){
    if(ttsType == TTS_XUNFEI){

        MSPLogout();

    }else if(ttsType == TTS_BAIDU){
        if(curl != NULL)
            curl_easy_cleanup(curl);
    }
}

void TextToSound::tts_play(string text,TTS_PARAM param){

    string soundPath = tts(text,param);
    if(soundPath.length() <= 0){
        printf("tts失败！");
        return;
    }

    string cmd;
    int status = 0;
    if(ttsType == TTS_XUNFEI){
        cmd = "play "+soundPath;
        status = system(cmd.data());
    }else{
        cmd = "play -tmp3 "+soundPath;
        status = system(cmd.data());
    }

    cmd = "rm "+soundPath;
    status = system(cmd.data());
}

string TextToSound::tts(std::string text,TTS_PARAM param){

    char fullPath[256];
    realpath("./",fullPath);
    struct timeval tv;
    gettimeofday(&tv,NULL);
    sprintf(fullPath,"%s/%d_%d_tts.wav",fullPath,tv.tv_sec,tv.tv_usec);
    // sprintf(fullPath,"%s/%ld_tts.wav",fullPath,(long)time(NULL));
    ttsSoundPath = string(fullPath);
    

    if(ttsType == TTS_XUNFEI){

        std::string voice_name      = param.voice_name.length() > 0 ? param.voice_name : "xiaoyan";
        std::string text_encoding   = param.text_encoding.length() > 0 ? param.text_encoding : "utf8";
        int sample_rate = param.sample_rate > 0 ? param.sample_rate : 16000;
        int speed   = param.speed > 0 ? param.speed : 50;
        int volume  = param.volume > 0 ? param.volume : 50;
        int pitch   = param.pitch > 0 ? param.pitch : 50;
        int rdn     = param.rdn > 0 ? param.rdn : 2;

        char paramStr[256];
        sprintf(paramStr,"voice_name = %s, \
                text_encoding = %s, \
                sample_rate = %d, \
                speed = %d, \
                volume = %d, \
                pitch = %d, \
                rdn = %d",voice_name.data(),text_encoding.data(),sample_rate,speed,volume,pitch,rdn);


        int ret = tts_xunfei(text.data(),ttsSoundPath.data(),paramStr);
        return ret == 0 ? ttsSoundPath : "";
    }else if(ttsType == TTS_BAIDU){
        int ret = tts_baidu(text.data(),ttsSoundPath.data(),param);
        return ret == 0 ? ttsSoundPath : "";
    }

    return "";
}



/********************* 科大讯飞文本合成 ******************/
int TextToSound::tts_xunfei(const char* src_text, const char* des_path, const char* params)
{
    int          ret          = -1;
    FILE*        fp           = NULL;
    const char*  sessionID    = NULL;
    unsigned int audio_len    = 0;
    wave_pcm_hdr wav_hdr      = default_wav_hdr;
    int          synth_status = MSP_TTS_FLAG_STILL_HAVE_DATA;

    if (NULL == src_text || NULL == des_path)
    {
        printf("params is error!\n");
        return ret;
    }
    fp = fopen(des_path, "wb");
    if (NULL == fp)
    {
        printf("open %s error.\n", des_path);
        return ret;
    }
    /* 开始合成 */
    sessionID = QTTSSessionBegin(params, &ret);
    if (MSP_SUCCESS != ret)
    {
        printf("QTTSSessionBegin failed, error code: %d.\n", ret);
        fclose(fp);
        return ret;
    }
    ret = QTTSTextPut(sessionID, src_text, (unsigned int)strlen(src_text), NULL);
    if (MSP_SUCCESS != ret)
    {
        printf("QTTSTextPut failed, error code: %d.\n",ret);
        QTTSSessionEnd(sessionID, "TextPutError");
        fclose(fp);
        return ret;
    }
    printf("正在合成 ...\n");
    fwrite(&wav_hdr, sizeof(wav_hdr) ,1, fp); //添加wav音频头，使用采样率为16000
    while (1)
    {
        /* 获取合成音频 */
        const void* data = QTTSAudioGet(sessionID, &audio_len, &synth_status, &ret);
        if (MSP_SUCCESS != ret)
            break;
        if (NULL != data)
        {
            fwrite(data, audio_len, 1, fp);
            wav_hdr.data_size += audio_len; //计算data_size大小
        }
        if (MSP_TTS_FLAG_DATA_END == synth_status)
            break;
        printf(">");
        usleep(150*1000); //防止频繁占用CPU
    }
    printf("\n");
    if (MSP_SUCCESS != ret)
    {
        printf("QTTSAudioGet failed, error code: %d.\n",ret);
        QTTSSessionEnd(sessionID, "AudioGetError");
        fclose(fp);
        return ret;
    }
    /* 修正wav文件头数据的大小 */
    wav_hdr.size_8 += wav_hdr.data_size + (sizeof(wav_hdr) - 8);

    /* 将修正过的数据写回文件头部,音频文件为wav格式 */
    fseek(fp, 4, 0);
    fwrite(&wav_hdr.size_8,sizeof(wav_hdr.size_8), 1, fp); //写入size_8的值
    fseek(fp, 40, 0); //将文件指针偏移到存储data_size值的位置
    fwrite(&wav_hdr.data_size,sizeof(wav_hdr.data_size), 1, fp); //写入data_size的值
    fclose(fp);
    fp = NULL;
    /* 合成完毕 */
    ret = QTTSSessionEnd(sessionID, "Normal");
    if (MSP_SUCCESS != ret)
    {
        printf("QTTSSessionEnd failed, error code: %d.\n",ret);
    }

    return ret;
}


int TextToSound::tts_baidu(const char* src_text, const char* des_path, TTS_PARAM param){
    if(curl == NULL)
        return -1;

    CURLcode code;
    //获取token
    char requestTokenUrl[512];
    sprintf(requestTokenUrl, "https://openapi.baidu.com/oauth/2.0/token?grant_type=client_credentials"
                             "&client_id=nA8xQaO5LzylmSV6Vb6DMerF"
                             "&client_secret=nyohimCTy08YkQe5PZWXEyZnO1RDkD5v");
    curl_easy_setopt(curl, CURLOPT_URL, requestTokenUrl);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &httpResponseText);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curlHttpResponseWriter);
    code = curl_easy_perform(curl);
    if(code != CURLE_OK){
        ROS_WARN("%s","获取百度语音接口token失败");
        return -1;
    }

    if(httpResponseText.length() > 0){
        json js = json::parse(httpResponseText);
        baidu_token_info.expires = time(NULL)+ js["expires_in"].get<time_t>();
        baidu_token_info.token  = js["access_token"];
        httpResponseText.clear();
    }

    //发送tts请求
    char *cuid = "cuid";
    int  per = param.voice_name.compare("women") == 0 ? 0 : 1;
    char requesTTSUrl[512];
    sprintf(requesTTSUrl, "http://tsn.baidu.com/text2audio?lan=zh&ctp=1"
                             "&tex=%s&tok=%s&cuid=%s&spd=%d&pit=%d&vol=%d&per=%d"
                            ,src_text,baidu_token_info.token.data(),cuid,
                            param.speed == 0 ? 5 : param.speed,
                            param.pitch == 0 ? 5 : param.pitch,
                            param.volume == 0 ? 5 : param.volume,
                            param.voice_name.compare("women") == 0 ? 0 : 1);

    FILE *ttsFile = fopen(ttsSoundPath.data(),"wb");
    if(ttsFile == NULL){
        ROS_WARN("%s","百度语音tts文件生成失败");
        return -1;
    }

    curl_easy_setopt(curl, CURLOPT_URL, requesTTSUrl);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, NULL);
    curl_easy_setopt(curl,CURLOPT_WRITEDATA,ttsFile);
    code = curl_easy_perform(curl);
    if(code != CURLE_OK){
        ROS_WARN("%s","百度语音tts失败");
        fclose(ttsFile);
        return -1;
    }

    fclose(ttsFile);
    return 0;
}


int curlHttpResponseWriter(char *data, size_t size, size_t nmemb,
                  std::string *writerData)
{
  if(writerData == NULL)
    return 0;
 
  writerData->append(data, size*nmemb);
 
  return size * nmemb;
}
