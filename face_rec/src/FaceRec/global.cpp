#include "global.h"
#if defined(_WINDOWS)

#include <io.h>
#include <direct.h>

#elif defined(_LINUX)

#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <dirent.h>

#endif

#include <iostream>

CYYJSLogger logger;	//全局日志器

#if defined(_WINDOWS)
    string faceDataPath = "";
#elif   defined(_LINUX)
    string faceDataPath = "/usr/data/FaceData";
#endif

void convertToGrayImg(Mat& srcImg){
	
	if(srcImg.channels() == 4){
		cvtColor(srcImg,srcImg,CV_BGRA2GRAY);	 
	}else if(srcImg.channels() == 3){
		cvtColor(srcImg,srcImg,CV_BGR2GRAY);
	}

}


bool isFileOrDicExist(const char *path){
	
#if defined( _WINDOWS )
	long	hFile = 0;
    struct	_finddata_t fileInfo;
    string	pathName, exdName;

    if ((hFile = _findfirst(pathName.assign(path).c_str(), &fileInfo)) == -1) {
        return false;
    }else
		return true;
#elif defined (_LINUX)
    if(access(path,F_OK) != 0){
        return false;
    } else
        return true;
#endif
}


bool createDic(char *path){
#if defined( _WINDOWS )
    int flag =  _mkdir(path);
    if(flag != 0)
        return false;
    else
        return true;

#elif defined(_LINUX)
    int flag =  mkdir(path,0777);
    if(flag != 0)
        return false;
    else
        return true;
#endif
}


void listDir(const char *path,vector<String>& files){

#if defined( _WINDOWS )

	long	hFile = 0;
    struct	_finddata_t fileInfo;
    string	pathName, filePath;
	filePath.assign(path);

    if ((hFile = _findfirst(pathName.assign(path).append("\\*").c_str(), &fileInfo)) == -1) {
        return;
    }
    do {
		if((fileInfo.attrib & _A_SUBDIR) == 0){
			files.push_back(filePath+"\\"+fileInfo.name);
			cout << fileInfo.name<<endl;
		}
        //cout << fileInfo.name << (fileInfo.attrib&_A_SUBDIR? "[folder]":"[file]") << endl;
    } while (_findnext(hFile, &fileInfo) == 0);
    _findclose(hFile);


#elif defined(_LINUX)
    DIR              *pDir ;
    struct dirent    *ent  ;
    int               i=0  ;
    char              childpath[512];


    pDir=opendir(path);
    memset(childpath,0,sizeof(childpath));


    while((ent=readdir(pDir))!=NULL)
    {

            if(ent->d_type & DT_DIR)
            {

                    if(strcmp(ent->d_name,".")==0 || strcmp(ent->d_name,"..")==0)
                            continue;

                    sprintf(childpath,"%s/%s",path,ent->d_name);
                    printf("path:%s/n",childpath);

                    //listDir(childpath);

            }else{
                char              filePath[256];
                sprintf(filePath,"%s/%s",path,ent->d_name);
                files.push_back(String(filePath));
                cout<<ent->d_name<<endl;
            }
     }

#endif
}
