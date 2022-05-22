#ifndef GET_CAMERA_REFER_H
#define GET_CAMERA_REFER_H


#include <fstream>
#include <string>
#include <vector>
#include <iostream>

extern "C"
{
#include "LDWS_Interface.h"
#include "FCWSD_Interface.h"
}

using namespace std;


//视频最大帧数
#define MAX_FRAME_NUMBER 2000


//定义当前帧信息
typedef struct _FRAME_INFO
{
	int leftx, lefty;
	double leftSlope;

	int rightx, righty;
	double rightSlope;

}frameinfo;

void getCameraRefer(string sPathName, int* dx, int* dy, int* sy, int* deltx);

#endif // GET_CAMERA_REFER_H
