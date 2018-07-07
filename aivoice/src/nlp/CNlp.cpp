#include "CNlp.h"
#include "json.hpp"

using json = nlohmann::json;

int curlHttpResponseWriter(char *data, size_t size, size_t nmemb,
                  std::string *writerData);

CNlp::CNlp()
{
    init();
}

CNlp::~CNlp()
{
    uninit();
}

int CNlp::init(){

    nlpType = NLP_TYPE_TULING;
    userId  = (long)time(NULL);

    //初始化设置curl网络请求库
    curl = curl_easy_init();        
    if(curl == NULL)
        return 0;
    CURLcode code;
    code = curl_easy_setopt(curl, CURLOPT_ERRORBUFFER, &httpResponseText);
    if(code != CURLE_OK)
         return 0;

    return 1;
}

int CNlp::uninit(){
    curl_easy_cleanup(curl);
    return 1;
}

string CNlp::process(string text){

    if(text.length() <= 0)
        return "";

    if(nlpType == NLP_TYPE_TULING)
        return processUseTuling(text);

}


string CNlp::processUseTuling(string text){

    char url[512];
    sprintf(url, "http://www.tuling123.com/openapi/api?userid=%ld"
                             "&key=de258109df06414680c5b3e85fd596bd"
                             "&info=%s",userId,text.data());
    curl_easy_setopt(curl, CURLOPT_URL, url);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &httpResponseText);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curlHttpResponseWriter);
    CURLcode code = curl_easy_perform(curl);
    if(code != CURLE_OK){
        printf("请求图灵接口失败->%s",url);
        return "";
    }

    if(httpResponseText.length() > 0){
        json js;
        try{
            js = json::parse(httpResponseText);
            httpResponseText.clear();
            int code = js["code"].get<int>();
            string text = js["text"];
            if( text.length() > 0)
                return text;
            else 
                return "";
        }catch(nlohmann::detail::parse_error err){
            char errMsg[512];
            sprintf(errMsg,"json解析失败 >> %s /n json>>%s",err.what(),js.dump().data());
            printf("%s",errMsg);
            
            return "";
        }
    }

    return "";
}

int curlHttpResponseWriter(char *data, size_t size, size_t nmemb,
                  std::string *writerData)
{
  if(writerData == NULL)
    return 0;
 
  writerData->append(data, size*nmemb);
 
  return size * nmemb;
}