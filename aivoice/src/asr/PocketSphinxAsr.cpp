#include "PocketSphinxAsr.h"
#include <ros/ros.h>


using namespace ros;

static const arg_t cont_args_def[] = {
    POCKETSPHINX_OPTIONS,
    /* Argument file. */
    {"-argfile",
     ARG_STRING,
     NULL,
     "Argument file giving extra arguments."},
    {"-adcdev",
     ARG_STRING,
     NULL,
     "Name of audio device to use for input."},
    {"-infile",
     ARG_STRING,
     NULL,
     "Audio file to transcribe."},
    {"-inmic",
     ARG_BOOLEAN,
     "no",
     "Transcribe audio from microphone."},
    {"-time",
     ARG_BOOLEAN,
     "no",
     "Print word times in file transcription."},
    CMDLN_EMPTY_OPTION
};

void* asrThread(PocketSphinxAsr *asr);

PocketSphinxAsr::PocketSphinxAsr(string hmmDir,string lmPath,string dictPath):tid(-1)
{

    if(hmmDir.length() == 0 || lmPath.length() == 0 || dictPath.length() == 0){
        ROS_WARN("%s","PocketSphinxAsr构造函数参数错误");
        return;
    }
    init( hmmDir, lmPath, dictPath);
}

PocketSphinxAsr::~PocketSphinxAsr()
{
    if(ps != NULL)
        ps_free(ps);
    if(config != NULL)
        cmd_ln_free_r(config);

}

void PocketSphinxAsr::init(string hmmDir,string lmPath,string dictPath){

    // string modelDir = "/usr/local/share/pocketsphinx/model/zh/";
    // string hmmPath = modelDir + "hmm";
    // string lmPath2  = modelDir + "1271.lm";
    // string dictPath2 = modelDir + "1271.dic";
    const char *argv[] = {"","-hmm",hmmDir.c_str(),"-lm",lmPath.c_str(),"-dict",dictPath.c_str(),"-inmic","yes","-vad_threshold","3"};
    int argc = 11;
    config = cmd_ln_parse_r(NULL, cont_args_def, argc, (char**)argv, TRUE);
    // config = cmd_ln_init(NULL, cont_args_def, TRUE,
    //              "-hmm", hmmDir,
    //              "-lm", lmPath,
    //              "-dict", dictPath,
    //              "-inmic","yes",
    //              "-vad_threshold","3",
    //              NULL);
    ps_default_search_args(config);
    ps = ps_init(config);
    if (ps == NULL) {
        cmd_ln_free_r(config);
        ROS_WARN("pocketsphinx初始化失败：ps_init()！");
        return ;
    }
    
}



bool PocketSphinxAsr::beginListen(psAsrResultFunc resultCallback){

    if(isListening)
        return true;

    if(resultCallback != NULL)
        asrCallbackFunc = resultCallback;
    
    if(tid != -1){
        ROS_WARN("%s","pocketsphinx线程已经创建");
        return false;
    }

    stopThread = FALSE;
    int ret = pthread_create(&tid,NULL,(void* (*)(void*))asrThread,this);

    if(ret == 0)
        return true;
    else{
        ROS_ERROR("pocketsphinx线程创建失败，错误码：%d",ret);
        return false;
    }
}

void PocketSphinxAsr::stopListen(){

    if(!isListening)
        return;

    void *ret = NULL;
    stopThread = TRUE;
    // pthread_join(tid,&ret);
    // if(ret == (void*)1)
    //     ROS_INFO("%s","pocketsphinx asr线程关闭成功");
    // else
    //     ROS_WARN("%s","pocketsphinx asr线程关闭失败");
    tid = -1;
}

// void PocketSphinxAsr::getAsrResult(string asrStr){
//     ROS_DEBUG("asr识别结果：%s",asrStr.c_str());
//     cout<<"asr识别结果："<<asrStr.c_str()<<endl;
// }

void* asrThread(PocketSphinxAsr *asr){

    if(asr == NULL)
        return (void*)-4;

    ad_rec_t *ad;
    int16 adbuf[2048];
    uint8 utt_started, in_speech;
    int32 k;
    char const *hyp;

    if ((ad = ad_open_dev(NULL,16000)) == NULL){
        ROS_ERROR("%s","Failed to open audio device");
        return (void*)-1;
    }
    if (ad_start_rec(ad) < 0){
        ROS_ERROR("%s","Failed to start recording");
        return (void*)-2;
    }
    if (ps_start_utt(asr->ps) < 0){
        ROS_ERROR("%s","Failed to start utterance");
        return (void*)-3;
    }
    utt_started = FALSE;
    ROS_INFO("%s","Ready....");

    asr->isListening = true;

    while (!asr->stopThread) {
        
        if ((k = ad_read(ad, adbuf, 2048)) < 0)
            ROS_ERROR("%s","Failed to read audio");
        ps_process_raw(asr->ps, adbuf, k, FALSE, FALSE);
        in_speech = ps_get_in_speech(asr->ps);
        if (in_speech && !utt_started) {
            utt_started = TRUE;
            ROS_INFO("%s","PS Listening....");
        }
        if (!in_speech && utt_started) {
            /* speech -> silence transition, time to start new utterance  */
            ps_end_utt(asr->ps);
            hyp = ps_get_hyp(asr->ps, NULL );
            if (hyp != NULL) {

                asr->asrCallbackFunc(hyp);
            }

            if (ps_start_utt(asr->ps) < 0)
                ROS_ERROR("%s","Failed to start utterance");
            utt_started = FALSE;
            ROS_INFO("%s","PS Ready....");
        }
        usleep(100);
    }
    ps_end_utt(asr->ps);
    ad_close(ad);
    ROS_INFO("%s","PS asr识别线程退出");
    asr->isListening = false;
    return (void*)1;
}