#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <iostream>
#include <fstream>
#include <string>

using namespace std;

#define DAY_OR_NIGHT 0
//#define DISP
#define LDWS
#define FCWSD
#define FCWST
//#define OBJVERF
//#define CNN
//#define OBJRLCT
#define my_record
//#define truth_value
#define SHOW_RESULT

#define PI 3.1415926

#ifdef CNN
#include "IdentifyNet.hpp"
#endif // CNN

#ifdef truth_value
#include "getCameraRefer.h"
#include "readTruthValue.h"
#endif // truth_value

#ifdef my_record
#include "saveResult.h"
#endif // my_record

#include <time.h>
#include <windows.h>
#include <io.h>
#include <direct.h>
#include <sys/timeb.h>
#define COST_TIME(function,endTime,startTime) \
	printf("%s function take time:%dms\n", function, ((endTime.time - startTime.time) * 1000 + (endTime.millitm - startTime.millitm)));

extern "C"
{

#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>

#include "DISP_Interface.h"
#include "3Dmap_Interface.h"
#include "LDWS_Interface.h"
#include "FCWSD_Interface.h"
#include "CMulitTrack.h"
#include "OBJVERF_Interface.h"
#include "vehicle_shadow.h"
#include "day_or_night.h"
#include "utils.h"
	//extern	float *m11, *m12, *m21, *m22;
}

objectSetsCar *pFCWSOutput = NULL;

cv::Mat frameDVR;
cv::Mat frameADAS, FCWSDrawframe;
cv::Mat warningPic;

int index;

int trackerObjNum = 0;

MuliTracker *pTrackOutput = NULL;

DayNightMode mode = DAY;

#ifdef CNN
static FClassifier identifyCar;
static int identifyFCWSTResult(const cv::Mat &src, const TrackedObjectList *trackedObjectsList, VerifyObjectList *verifyObjectOutput;)
{
	int result = 0;
	for (int loop = 0; loop < trackedObjectsList->objectCount; loop++)
	{
		WissenObjectRectTracked rect = trackedObjectsList->trackedObject[loop];
		cv::Mat roi = src(cv::Rect(rect.object.x, rect.object.y, rect.object.width, rect.object.height)).clone();
		result = identifyCar.identifyNet(roi);
		verifyObjectOutput->object[loop].nGroupID = rect.nGroupID;
		verifyObjectOutput->object[loop].nObjState = result;
	}
	verifyObjectOutput->objectCount = trackedObjectsList->objectCount;
	return 0;
}
static int identifyFCWSDTResult(const cv::Mat &src, objectSetsCar *pFCWSOutput)
{
	int result = 0;
	int index = 0;
	for (int loop = 0; loop < pFCWSOutput->nObjectNum; loop++)
	{
		WissenObjectRect object = pFCWSOutput->objects[loop];
		cv::Mat roi = src(cv::Rect(object.x, object.y, object.width, object.height)).clone();
		result = identifyCar.identifyNet(roi);
		if (result > 0)
		{
			pFCWSOutput->objects[index] = object;
			index++;
		}
	}
	pFCWSOutput->nObjectNum = index;
	return 0;
}
#endif // CNN

static int matWrite(const cv::Mat &img, const std::string &fileName)
{
	std::ofstream output;
	output.open(fileName.c_str(), std::ofstream::binary);
	if (!output.is_open())
	{
		std::cerr << "failed to open the file : " << fileName << std::endl;
		return 0;
	}
	for (int r = 0; r < img.rows; r++)
	{
		const uchar* data = img.ptr<uchar>(r);
		output.write(reinterpret_cast<const char*>(data), sizeof(uchar) * img.step[0]);
	}
	output.close();
	return 1;
}

static int readImage(const char *fileName, unsigned char *image)
{
	int count = 0;
	int size = 0;
	FILE *infile = NULL;
	infile = fopen(fileName, "rb");
	unsigned char buf[1024];

	if (infile == NULL)
	{
		//LOGI("Optical", "%s not exit\n", fileName);
		printf("%s, %s", fileName, "not exit\n");
		return -1;
	}
	while (1)
	{
		size = fread(buf, sizeof(unsigned char), 1024, infile);
		if (size == 0)
			break;
		memcpy(image + count, buf, size * sizeof(unsigned char));
		count += size;
	}

	fclose(infile);
	return count;
}

void logImageI(const char* fileName, const unsigned int *img, const int width, const int height)
{
	FILE *writeFile = fopen(fileName, "w");
	int i = 0;
	int j = 0;
	int offset = 0;
	char data[64];
	for (i = 0; i < height; i++)
	{
		for (j = 0; j < width; j++)
		{
			offset = sprintf(data, "%d ", img[j]);
			fwrite(data, sizeof(char), offset, writeFile);
		}
		fwrite("\n", sizeof(char), 1, writeFile);
		img += width;
	}
	fclose(writeFile);
}

void readUVData(const unsigned char *yuvImage, int width, int height, cv::Mat &yMat, cv::Mat &uMat, cv::Mat &vMat)
{
	const unsigned char *uData = yuvImage + width * height;
	const unsigned char *vData = uData + (width * height >> 2);
	for (int row = 0; row < height; row += 1)
	{
		for (int col = 0; col < width; col += 1)
		{
			yMat.at<uchar>(row, col) = *yuvImage;
			yuvImage++;
		}
	}
	for (int row = 0; row < height; row += 2)
	{
		for (int col = 0; col < width; col += 2)
		{
			uMat.at<uchar>(row, col) = *uData;
			uMat.at<uchar>(row, col + 1) = *uData;
			uMat.at<uchar>(row + 1, col) = *uData;
			uMat.at<uchar>(row + 1, col + 1) = *uData;

			vMat.at<uchar>(row, col) = *vData;
			vMat.at<uchar>(row, col + 1) = *vData;
			vMat.at<uchar>(row + 1, col) = *vData;
			vMat.at<uchar>(row + 1, col + 1) = *vData;

			uData++;
			vData++;
		}
	}
}

void mvDrawLD_Result(cv::Mat &Img, LDWS_InitGuid *pLDWSInit)
{
	int i, j, alarm_result;
	cv::Scalar color;
	LDWS_Output *pTest = NULL;


	LDWS_GetResult(&pTest); //pTest need to be free!!!

	//printf("L=%f, X0=%f \n",pTest->Param[0],pTest->Param[1]);
	char buffer[64];
	std::sprintf(buffer, "L=%0.1f,x=%0.1f,Vangle=%0.4f,Hangle=%0.4f,C0=%0.5f", pTest->Param[0], pTest->Param[1], pTest->Param[2] * 180 / PI, pTest->Param[3] * 180 / PI, pTest->Param[4] * 180 / PI);
	cv::putText(Img, buffer, cvPoint(10, 50), CV_FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);

	int vy = LDWS_GetVanishY();
	int Eu1, Ev1, Cx1, Cy1;
	LDWS_Get_inter_Pamer_W(&Eu1, &Ev1, &Cx1, &Cy1); //得到Donnees->Eu,Ev,Cx,Cy
	alarm_result = pTest->alarm_result;

	//for ( j = 0; j < pLDWSInit->NB_BANDES; j++ )
	//      {
	//	color = Scalar(255, 255, 0);
	//	for ( i = j * (pLDWSInit->NB_INTERVALLES); i < (j + 1) * pLDWSInit->NB_INTERVALLES-1; i++ )
	//	{
	//		line(Img,
	//			cvPoint(pLDWSInit->pCaPoint[i].x, pLDWSInit->pCaPoint[i].y),
	//			cvPoint(pLDWSInit->pCaPoint[i + 1].x, pLDWSInit->pCaPoint[i + 1].y), 
	//			color, 3);
	//	}
	//}

	for (i = 0; i < 6; i++)
	{
		line(Img,
			cvPoint(pLDWSInit->pBoundPoint[i].x, pLDWSInit->pBoundPoint[i].y),
			cvPoint(pLDWSInit->pBoundPoint[i + 1].x, pLDWSInit->pBoundPoint[i + 1].y),
			cv::Scalar(0, 255, 255), 3);
	}

	//#endif

	if (pTest->Route == 1 && pTest->Route_half == 0)
	{
		for (j = 0; j < 2; j++)
		{
			color = cv::Scalar(0, 255, 0);

			if ((j == 0) && (alarm_result == 1)) //左报警
			{
				color = cv::Scalar(0, 0, 255);
			}
			else if ((j == 1) && (alarm_result == 2))                       //右报警
			{
				color = cv::Scalar(0, 0, 255);  //右报警
			}

			if (pTest->Confidence_detection[0] <= pTest->Confidence - KEEP_FRAME)
			{
				color = cv::Scalar(255, 255, 0);
				break;
			}

			for (i = j * (pTest->NB_INTERVALLES); i < (j + 1) * pTest->NB_INTERVALLES; i++)
			{
				//double temp = (cos(pTest->Param[3]) / pTest->Z0) * 0.45 * (pTest->pCaPoint[i].y - Cy1) * 0.5;
				double temp = LDWS_GetXLengthofImage(0.15, pTest->pCaPoint[i].y) * 0.5;

				line(Img, cvPoint(pTest->pCaPoint[i].x - temp, pTest->pCaPoint[i].y), cvPoint(pTest->pCaPoint[i].x + temp, pTest->pCaPoint[i].y), color, 5);

				if ((i % pTest->NB_INTERVALLES) != 0)
				{
					//temp = (cos(pTest->Param[3]) / pTest->Z0) * 0.45 * ((pTest->pCaPoint[i].y + pTest->pCaPoint[i - 1].y) * 0.5 - Cy1) * 0.5;
					temp = LDWS_GetXLengthofImage(0.15, pTest->pCaPoint[i].y) * 0.5;

					line(Img, cvPoint((pTest->pCaPoint[i].x + pTest->pCaPoint[i - 1].x) * 0.5 - temp, (pTest->pCaPoint[i].y + pTest->pCaPoint[i - 1].y) * 0.5), cvPoint((pTest->pCaPoint[i].x + pTest->pCaPoint[i - 1].x) * 0.5 + temp, (pTest->pCaPoint[i].y + pTest->pCaPoint[i - 1].y) * 0.5), color, 5);
				}

				/*printf("L=%f,X0=%f, X=%f, Y=%f, Z=%f \n",pTest->Param[0],pTest->Param[1],pTest->p3DCaPoint[i].X,pTest->p3DCaPoint[i].Y,pTest->p3DCaPoint[i].Z);
				double Xdis;
				Xdis=LDWS_GetXofWorld_W(pTest->pCaPoint[i].x, pTest->pCaPoint[i].y);
				printf("Xdis=%f \n",Xdis);*/
				//if(j==0)
				//	printf("dist=%f \n",pTest->p3DCaPoint[i+pTest->NB_INTERVALLES].X-pTest->p3DCaPoint[i].X);
			}
		}

		color = cv::Scalar(0, 255, 0);



		// 左车道线
		if (pTest->Route == 1 && pTest->Confidence_detection[0] > pTest->Confidence - KEEP_FRAME && pTest->Route_L == 1 && pTest->Confidence_detection[4] > pTest->Confidence - KEEP_FRAME)
		{
			for (i = 2 * (pTest->NB_INTERVALLES); i < (2 + 1) * pTest->NB_INTERVALLES; i++)
			{
				//double temp = (cos(pTest->Param[3]) / pTest->Z0) * 0.45 * (pTest->pCaPoint[i].y - Cy1) * 0.5;
				double temp = LDWS_GetXLengthofImage(0.15, pTest->pCaPoint[i].y) * 0.5;

				line(Img, cvPoint(pTest->pCaPoint[i].x - temp, pTest->pCaPoint[i].y), cvPoint(pTest->pCaPoint[i].x + temp, pTest->pCaPoint[i].y), color, 5);
				if ((i % pTest->NB_INTERVALLES) != 0)
				{
					//temp = (cos(pTest->Param[3]) / pTest->Z0) * 0.45 * ((pTest->pCaPoint[i].y + pTest->pCaPoint[i - 1].y) * 0.5 - Cy1) * 0.5;
					temp = LDWS_GetXLengthofImage(0.15, pTest->pCaPoint[i].y) * 0.5;

					line(Img, cvPoint((pTest->pCaPoint[i].x + pTest->pCaPoint[i - 1].x) * 0.5 - temp, (pTest->pCaPoint[i].y + pTest->pCaPoint[i - 1].y) * 0.5), cvPoint((pTest->pCaPoint[i].x + pTest->pCaPoint[i - 1].x) * 0.5 + temp, (pTest->pCaPoint[i].y + pTest->pCaPoint[i - 1].y) * 0.5), color, 5);
				}
			}
		}

		// 右车道线
		if (pTest->Route == 1 && pTest->Confidence_detection[0] > pTest->Confidence - KEEP_FRAME && pTest->Route_R == 1 && pTest->Confidence_detection[5] > pTest->Confidence - KEEP_FRAME)
		{
			for (i = 3 * (pTest->NB_INTERVALLES); i < (3 + 1) * pTest->NB_INTERVALLES; i++)
			{
				//double temp = (cos(pTest->Param[3]) / pTest->Z0) * 0.45 * (pTest->pCaPoint[i].y - Cy1) * 0.5;
				double temp = LDWS_GetXLengthofImage(0.15, pTest->pCaPoint[i].y) * 0.5;

				line(Img, cvPoint(pTest->pCaPoint[i].x - temp, pTest->pCaPoint[i].y), cvPoint(pTest->pCaPoint[i].x + temp, pTest->pCaPoint[i].y), color, 5);
				if ((i % pTest->NB_INTERVALLES) != 0)
				{
					//temp = (cos(pTest->Param[3]) / pTest->Z0) * 0.45 * ((pTest->pCaPoint[i].y + pTest->pCaPoint[i - 1].y) * 0.5 - Cy1) * 0.5;
					temp = LDWS_GetXLengthofImage(0.15, pTest->pCaPoint[i].y) * 0.5;

					line(Img, cvPoint((pTest->pCaPoint[i].x + pTest->pCaPoint[i - 1].x) * 0.5 - temp, (pTest->pCaPoint[i].y + pTest->pCaPoint[i - 1].y) * 0.5), cvPoint((pTest->pCaPoint[i].x + pTest->pCaPoint[i - 1].x) * 0.5 + temp, (pTest->pCaPoint[i].y + pTest->pCaPoint[i - 1].y) * 0.5), color, 5);
				}
			}
		}

	}


	if (pTest->Route == 2 && (pTest->Confidence_detection[2] > pTest->Confidence - KEEP_FRAME))
	{

		color = cv::Scalar(0, 255, 0);

		if ((alarm_result == 1)) //左报警
		{
			color = cv::Scalar(0, 0, 255);
		}

		for (i = 0; i < pTest->NB_INTERVALLES; i++)
		{
			//double temp = (cos(pTest->Param[3]) / pTest->Z0) * 0.45 * (pTest->pCaPoint[i].y - Cy1) * 0.5;
			double temp = LDWS_GetXLengthofImage(0.15, pTest->pCaPoint[i].y) * 0.5;

			line(Img, cvPoint(pTest->pCaPoint[i].x - temp, pTest->pCaPoint[i].y), cvPoint(pTest->pCaPoint[i].x + temp, pTest->pCaPoint[i].y), color, 5);

			if ((i % pTest->NB_INTERVALLES) != 0)
			{
				//temp = (cos(pTest->Param[3]) / pTest->Z0) * 0.45 * ((pTest->pCaPoint[i].y + pTest->pCaPoint[i - 1].y) * 0.5 - Cy1) * 0.5;
				temp = LDWS_GetXLengthofImage(0.15, pTest->pCaPoint[i].y) * 0.5;

				line(Img, cvPoint((pTest->pCaPoint[i].x + pTest->pCaPoint[i - 1].x) * 0.5 - temp, (pTest->pCaPoint[i].y + pTest->pCaPoint[i - 1].y) * 0.5), cvPoint((pTest->pCaPoint[i].x + pTest->pCaPoint[i - 1].x) * 0.5 + temp, (pTest->pCaPoint[i].y + pTest->pCaPoint[i - 1].y) * 0.5), color, 5);
			}
		}
	}

	if (pTest->Route == 3 && (pTest->Confidence_detection[3] > pTest->Confidence - KEEP_FRAME))
	{

		color = cv::Scalar(0, 255, 0);

		if ((alarm_result == 2))                       //右报警
		{
			color = cv::Scalar(0, 0, 255);  //右报警
		}

		for (i = 1 * (pTest->NB_INTERVALLES); i < (1 + 1) * pTest->NB_INTERVALLES; i++)
		{
			//double temp = (cos(pTest->Param[3]) / pTest->Z0) * 0.45 * (pTest->pCaPoint[i].y - Cy1) * 0.5;
			double temp = LDWS_GetXLengthofImage(0.15, pTest->pCaPoint[i].y) * 0.5;

			line(Img, cvPoint(pTest->pCaPoint[i].x - temp, pTest->pCaPoint[i].y), cvPoint(pTest->pCaPoint[i].x + temp, pTest->pCaPoint[i].y), color, 5);

			if ((i % pTest->NB_INTERVALLES) != 0)
			{
				//temp = (cos(pTest->Param[3]) / pTest->Z0) * 0.45 * ((pTest->pCaPoint[i].y + pTest->pCaPoint[i - 1].y) * 0.5 - Cy1) * 0.5;
				temp = LDWS_GetXLengthofImage(0.15, pTest->pCaPoint[i].y) * 0.5;

				line(Img, cvPoint((pTest->pCaPoint[i].x + pTest->pCaPoint[i - 1].x) * 0.5 - temp, (pTest->pCaPoint[i].y + pTest->pCaPoint[i - 1].y) * 0.5), cvPoint((pTest->pCaPoint[i].x + pTest->pCaPoint[i - 1].x) * 0.5 + temp, (pTest->pCaPoint[i].y + pTest->pCaPoint[i - 1].y) * 0.5), color, 5);
			}

		}
	}

	if (pTest->Route_half == 1)
	{
		for (j = 0; j < 2; j++)
		{
			color = cv::Scalar(0, 255, 0);

			if ((j == 0) && (alarm_result == 1)) //左报警
			{
				color = cv::Scalar(0, 0, 255);
			}
			else if ((j == 1) && (alarm_result == 2))                       //右报警
			{
				color = cv::Scalar(0, 0, 255);  //右报警
			}

			if (pTest->Confidence_detection[1] <= pTest->Confidence - KEEP_FRAME)
			{
				color = cv::Scalar(255, 255, 0);
				break;
			}

			for (i = j * (pTest->NB_INTERVALLES) + 3; i < (j + 1) * pTest->NB_INTERVALLES; i++)
			{
				//double temp = (cos(pTest->Param[3]) / pTest->Z0) * 0.45 * (pTest->pCaPoint[i].y - Cy1) * 0.5;
				double temp = LDWS_GetXLengthofImage(0.15, pTest->pCaPoint[i].y) * 0.5;

				cv::line(Img, cvPoint(pTest->pCaPoint[i].x - temp, pTest->pCaPoint[i].y), cvPoint(pTest->pCaPoint[i].x + temp, pTest->pCaPoint[i].y), color, 5);

				if ((i % pTest->NB_INTERVALLES) != 0)
				{
					//temp = (cos(pTest->Param[3]) / pTest->Z0) * 0.45 * ((pTest->pCaPoint[i].y + pTest->pCaPoint[i - 1].y) * 0.5 - Cy1) * 0.5;
					temp = LDWS_GetXLengthofImage(0.15, pTest->pCaPoint[i].y) * 0.5;

					cv::line(Img, cvPoint((pTest->pCaPoint[i].x + pTest->pCaPoint[i - 1].x) * 0.5 - temp, (pTest->pCaPoint[i].y + pTest->pCaPoint[i - 1].y) * 0.5), cvPoint((pTest->pCaPoint[i].x + pTest->pCaPoint[i - 1].x) * 0.5 + temp, (pTest->pCaPoint[i].y + pTest->pCaPoint[i - 1].y) * 0.5), color, 5);
				}

				/*printf("L=%f,X0=%f, X=%f, Y=%f, Z=%f \n",pTest->Param[0],pTest->Param[1],pTest->p3DCaPoint[i].X,pTest->p3DCaPoint[i].Y,pTest->p3DCaPoint[i].Z);
				double Xdis;
				Xdis=LDWS_GetXofWorld_W(pTest->pCaPoint[i].x, pTest->pCaPoint[i].y);
				printf("Xdis=%f \n",Xdis);*/
				//if(j==0)
				//	printf("dist=%f \n",pTest->p3DCaPoint[i+pTest->NB_INTERVALLES].X-pTest->p3DCaPoint[i].X);
			}
		}
	}
}

void mvDrawDetector_Result(cv::Mat *img, int index)
{

	int i, j;
	static  int statrack = 0;

	//随机生成颜色
	static cv::Scalar Trackercol[512];

	if (!statrack)
	{
		for (i = 0; i < 512; i++)
		{
			Trackercol[i] = cv::Scalar(rand() % 255, rand() % 255, rand() % 255);
		}
	}
	statrack = 1;

	//FCWSD_GetResult(index, &pFCWSOutput);
	//FCWSD_RefineResult(pFCWSOutput, img->cols, img->rows);

	if (pFCWSOutput != NULL)
	{
		//绘制了目标所有的轨迹和中新轨迹
		for (j = 0; j < pFCWSOutput->nObjectNum; j++)
		{
			//绘制group外界矩形		
			cv::rectangle(*img, cvPoint(pFCWSOutput->objects[j].x, pFCWSOutput->objects[j].y), \
				cvPoint(pFCWSOutput->objects[j].x + pFCWSOutput->objects[j].width, \
					pFCWSOutput->objects[j].y + pFCWSOutput->objects[j].height), \
				cv::Scalar(0, 0, 255)/*Trackercol[j%512]*/, 2);

			//double carHeight = LDWS_GetImageY((pFCWSOutput->objects[j].objectRect.point.y + pFCWSOutput->objects[j].objectRect.size.height), pFCWSOutput->objects[j].objectRect.point.y);

			//double carWidth = LDWS_GetDetaXofWorld(pFCWSOutput->objects[j].objectRect.size.width, \
						//  				             pFCWSOutput->objects[j].objectRect.point.y+ pFCWSOutput->objects[j].objectRect.size.height);
			char buffer[64];
			sprintf(buffer, "%d", pFCWSOutput->objects[j].confidence);
			//sprintf(buffer,"%d",pFCWSOutput->objects[j].confidence);
			cv::putText(*img, buffer, cvPoint(pFCWSOutput->objects[j].x + pFCWSOutput->objects[j].width / 2, \
				MAX(0, pFCWSOutput->objects[j].y + pFCWSOutput->objects[j].width / 2)), \
				CV_FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0), 2);

			//	int heightNew = LDWS_GetCarY(pFCWSOutput->objects[j].objectRect.point.y+ pFCWSOutput->objects[j].objectRect.size.height, 1.8);
			//int yNew1 = pFCWSOutput->objects[j].objectRect.point.y + pFCWSOutput->objects[j].objectRect.size.height - heightNew;
			//int widthNew = pFCWSOutput->objects[j].objectRect.size.width * 1.8 / carWidth;
			//int xNew1 = 0.5 * ((2 * pFCWSOutput->objects[j].objectRect.point.x + pFCWSOutput->objects[j].objectRect.size.width) - widthNew);

			//Rect rectNew;
			//rectNew.x = xNew1;
			//rectNew.y = yNew1;
			//rectNew.width = widthNew;
			//rectNew.height = heightNew;

			//rectangle(*img,cvPoint(rectNew.x,rectNew.y),cvPoint(rectNew.x + rectNew.width,\
						//		rectNew.y + rectNew.height),Scalar(0,0,255) ,2 );

		}
	}

}

static int FCWSD_RefineResult(objectSetsCar *pFCWSOutput, int imgWidth, int imgheight)
{
	double carHeight = 0, carWidth = 0;
	int j, heightNew, yNew1, widthNew, xNew1;
	if (pFCWSOutput != NULL)
	{
		for (j = 0; j < pFCWSOutput->nObjectNum; j++)
		{
			carHeight = LDWS_GetImageY((pFCWSOutput->objects[j].y + pFCWSOutput->objects[j].height), pFCWSOutput->objects[j].y);

			carWidth = LDWS_GetDetaXofWorld(pFCWSOutput->objects[j].width, pFCWSOutput->objects[j].y + pFCWSOutput->objects[j].height);

			heightNew = LDWS_GetCarY(pFCWSOutput->objects[j].y + pFCWSOutput->objects[j].height, 1.8);
			yNew1 = pFCWSOutput->objects[j].y + pFCWSOutput->objects[j].height - heightNew;
			widthNew = pFCWSOutput->objects[j].width * 1.8 / carWidth;
			xNew1 = 0.5 * ((2 * pFCWSOutput->objects[j].x + pFCWSOutput->objects[j].width) - widthNew);

			if (xNew1 > 0 && yNew1 > 0 && (xNew1 + widthNew) < imgWidth && (yNew1 + heightNew) < imgheight)
			{
				pFCWSOutput->objects[j].x = xNew1;
				pFCWSOutput->objects[j].y = yNew1;
				pFCWSOutput->objects[j].width = MAX(widthNew, heightNew);
				pFCWSOutput->objects[j].height = pFCWSOutput->objects[j].width;
			}
			else
			{
				pFCWSOutput->objects[j].confidence = 0;
			}
		}
	}

	return(1);
}

void mvDrawTrack_Result(cv::Mat *img, float fzoom)
{

	int i, j;

	static cv::Mat g_InitMat;
	static WissenObjectRectTracked g_preRec;
	static  int statrack0 = 0;
	bool bGoroupExis = 0;

	static cv::Scalar Trackercol[512];


	//#ifdef ALL_TRACK_SHOW
	//		static int nFramSeq =0;
	//#endif

	if (!statrack0)
	{
		for (i = 0; i < 512; i++)
		{
#ifdef ALL_TRACK_SHOW
			Trackercol[i] = cv::Scalar(rand() % 255, rand() % 255, rand() % 255);
#else
			Trackercol[i] = cv::Scalar(0, 255, 0);
#endif			
		}
	}
	statrack0 = 1;

	//绘制了目标所有的轨迹和中新轨迹
	for (j = 0; j < pTrackOutput->nTrackeNum; j++)
	{

		trakobj pGroup = *(pTrackOutput->pTrackerset + j);

		//cv::Scalar col = Trackercol[pGroup.nId % 512];
		cv::Scalar  col = cv::Scalar(0, 0, 255);

		if (!pGroup.bTrue)
		{
			//col = cv::Scalar(0, 0, 0);

#ifndef ALL_TRACK_SHOW
			continue;
#else
#endif
		}
		else
		{

			if (pGroup.pMotion[(pGroup.nMotionLeng - 1) & 63].bInCollishionPath == 1)
			{
				col = cv::Scalar(0, 255, 0);
			}
			else if (pGroup.pMotion[(pGroup.nMotionLeng - 1) & 63].bInCollishionPath == 2)
			{
				col = cv::Scalar(0, 255, 255);
			}
			else if (pGroup.pMotion[(pGroup.nMotionLeng - 1) & 63].bInCollishionPath == 3)
			{
				col = cv::Scalar(255, 255, 0);
			}
			else if (pGroup.pMotion[(pGroup.nMotionLeng - 1) & 63].bInCollishionPath == 0)
			{
				col = cv::Scalar(255, 255, 255);
			}

		}

		pGroup.objRec.object.x *= fzoom;
		pGroup.objRec.object.y *= fzoom;
		pGroup.objRec.object.height *= fzoom;
		pGroup.objRec.object.width *= fzoom;

		//pGroup->objRec.y = pGroup->objRec.y + pGroup->objRec.height * 0.4;
		//pGroup->objRec.height = pGroup->objRec.height * 0.6;

		/**/
		cv::Mat src = *img;
		cv::Mat roiImg = src(cv::Rect(pGroup.objRec.object.x, pGroup.objRec.object.y, pGroup.objRec.object.width, pGroup.objRec.object.height));
		cv::Mat roiGray;
		cv::cvtColor(roiImg, roiGray, CV_BGR2GRAY);
		//findVehicleRect(roiGray.ptr(), groundThresh, pGroup->objRec.width, pGroup->objRec.height);

		cv::rectangle(*img, cvPoint(pGroup.objRec.object.x, pGroup.objRec.object.y), \
			cvPoint(pGroup.objRec.object.x + pGroup.objRec.object.width, pGroup.objRec.object.y + pGroup.objRec.object.height), \
			col, 2);


		cv::line(*img, cvPoint(pGroup.objRec.object.x, pGroup.objRec.object.y - 2), cvPoint(pGroup.objRec.object.x + 20, pGroup.objRec.object.y - 2), Trackercol[j % 512], 4);
		cv::line(*img, cvPoint(pGroup.objRec.object.x + pGroup.objRec.object.width - 20, pGroup.objRec.object.y - 2), cvPoint(pGroup.objRec.object.x + pGroup.objRec.object.width, pGroup.objRec.object.y - 2), \
			col, 4);
		cv::line(*img, cvPoint(pGroup.objRec.object.x - 2, pGroup.objRec.object.y), \
			cvPoint(pGroup.objRec.object.x - 2, pGroup.objRec.object.y + 20), \
			col, 4);
		cv::line(*img, cvPoint(pGroup.objRec.object.x - 2, pGroup.objRec.object.y - 20 + pGroup.objRec.object.height), \
			cvPoint(pGroup.objRec.object.x - 2, pGroup.objRec.object.y + pGroup.objRec.object.height), \
			col, 4);
		cv::line(*img, cvPoint(pGroup.objRec.object.x, pGroup.objRec.object.y - 2 + pGroup.objRec.object.height), \
			cvPoint(pGroup.objRec.object.x + 20, pGroup.objRec.object.y - 2 + pGroup.objRec.object.height), \
			col, 4);
		cv::line(*img, cvPoint(pGroup.objRec.object.x + pGroup.objRec.object.width - 20, pGroup.objRec.object.y - 2 + pGroup.objRec.object.height), \
			cvPoint(pGroup.objRec.object.x + pGroup.objRec.object.width, pGroup.objRec.object.y - 2 + pGroup.objRec.object.height), \
			col, 4);
		cv::line(*img, cvPoint(pGroup.objRec.object.x - 2 + pGroup.objRec.object.width, pGroup.objRec.object.y), \
			cvPoint(pGroup.objRec.object.x - 2 + pGroup.objRec.object.width, pGroup.objRec.object.y + 20), \
			col, 4);
		cv::line(*img, cvPoint(pGroup.objRec.object.x - 2 + pGroup.objRec.object.width, pGroup.objRec.object.y - 20 + pGroup.objRec.object.height), \
			cvPoint(pGroup.objRec.object.x - 2 + pGroup.objRec.object.width, pGroup.objRec.object.y + pGroup.objRec.object.height), \
			col, 4);

		double disX, disZ, disVh;
		char buffer[64];
		//LDWS_Get_Dist_xz(pGroup->objRec.x + pGroup->objRec.width*0.5, pGroup->objRec.y + pGroup->objRec.height,\
										//							&disX,&disZ,&disVh);
		//if(disZ>0)
		//{
		//
		//sprintf(buffer,"X:%.1f: Z:%.1f",disX,disZ);
		//putText(*img, buffer, cvPoint( pGroup->objRec.x + pGroup->objRec.width,pGroup->objRec.y + pGroup->objRec.height),\
										//	CV_FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,255),3 ); 
		//}

		sprintf(buffer, "Bin:%d, Len:%d,Conf:%d", pGroup.pMotion[(pGroup.nMotionLeng - 1) & 63].bInCollishionPath, pGroup.nTrackLen, pGroup.objRec.object.confidence);
		cv::putText(*img, buffer, cvPoint(pGroup.objRec.object.x + pGroup.objRec.object.width, pGroup.objRec.object.y + pGroup.objRec.object.height), \
			CV_FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 3);

		sprintf(buffer, "ID:%d", pGroup.nId);
		cv::putText(*img, buffer, cvPoint(pGroup.objRec.object.x + pGroup.objRec.object.width / 2, pGroup.objRec.object.y), \
			CV_FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 3);

		int x_left, x_right;
		int nLocalY = pGroup.objRec.object.y + pGroup.objRec.object.height;
		LDWS_GetXofY(nLocalY, &x_left, &x_right);

		//#ifdef ALL_TRACK_SHOW
		//	if(x_left < img->cols)
		//{
		//	circle(*img,Point(x_left,nLocalY),8,CV_RGB(255,0,0));
		//	circle(*img,Point(x_right,nLocalY),8,col);
		//}
		//#endif

		float nTTC_WARNING_TIME = 7;
		sprintf(buffer, "Bin:%d, TTC:%0.1f s", pGroup.pMotion[(pGroup.nMotionLeng - 1) & 63].bInCollishionPath, pGroup.fTTC);
		putText(*img, buffer, cvPoint(pGroup.objRec.object.x + pGroup.objRec.object.width / 2, MAX(0, pGroup.objRec.object.y - 8)), \
			CV_FONT_HERSHEY_SIMPLEX, 1, col, 2);
		if (pGroup.fTTC < 7 && pGroup.fTTC >0)
		{
			cv::Mat warningResize;
			cv::resize(warningPic, warningResize, cv::Size(pGroup.objRec.object.width*0.6, pGroup.objRec.object.height*0.6));

			cv::Mat imgRoi;
			imgRoi = *img;
			imgRoi = imgRoi(cv::Rect(pGroup.objRec.object.x + 0.2*pGroup.objRec.object.width, pGroup.objRec.object.y + 0.2*pGroup.objRec.object.height, 0.6*pGroup.objRec.object.width, 0.6*pGroup.objRec.object.height));
			double alpha = (pGroup.fTTC) / nTTC_WARNING_TIME;
			cv::addWeighted(imgRoi, alpha, warningResize, 1 - alpha, 0., imgRoi);
		}

	}
}

#ifndef DISP
int testLDAndFCW(char *videoName, char  *ldwsConfigFile, char* ldwsCustomCalibration, char  *fcwdConfigFile, char *identifyNet, char* identifyModel, int th)
{
	SYSTEMTIME currentTime;
	int uFramSeq = 0;

	cv::Mat frame, Resizeframe;
	cv::Mat yuv;
	cv::Mat uMat, resizeU;
	cv::Mat vMat, resizeV;

	cv::VideoCapture capture(videoName);
	if (!capture.isOpened())
	{
		printf("%s can not open!\n", videoName);
		return -1;
	}
	capture.set(CV_CAP_PROP_POS_FRAMES, uFramSeq);

	cv::Size sizeScale; //resized sacle for tracker
	float fzoomRate = 2.0f;
	struct timeb startTime, endTime;
	static int flg = 0;
	WissenImage GrayImg;  //data used for FCWSD
	FCWSDParam pParam;
	cv::Mat grayADASMat;
	cv::Mat ResizegrayTemp; // used for tracker
	IplImage *image_grayTemp; //data used for Tracker
	unsigned char * grayTemp = 0;

	LDWS_Output *pTest = NULL;

	int imageWidth = 1280;
	int imageHeight = 720;

#ifdef my_record
	cv::VideoWriter RecordAvi;
	bool bOpen = 0;
	FILE *writeFile = NULL;
	std::string result_file_name(videoName);
	std::string result_txt_file_name;
	result_file_name.insert((result_file_name.find_last_of('.')), "_smartEye");
	std::string s(result_file_name, (result_file_name.find_last_of('\\')));//将字符串str内"始于位置stridx"的部分当作字符串的初值 
	std::string savefromat(".avi");
	s.replace(s.find_last_of('.'), 4, savefromat);

	result_file_name = "result" + s;

	std::string savefromat1(".txt");
	s.replace(s.find_last_of('.'), 4, savefromat1);

	result_txt_file_name = "result" + s;

	std::cout << result_file_name.c_str() << std::endl;

	if ((_access("result", 0)) == -1) //文件不存在
	{
		_mkdir("result");
	}

#ifdef truth_value
	int allCount = 0;
	std::vector< std::vector<CarTruthValue> > videoTruthValues;
	std::vector< std::vector<CarTruthValue> > resultTruthValues;
	std::vector<CarTruthValue> truthValue;

	int Vx, Vy, sy, deltx;
	getCameraRefer(videoName, &Vx, &Vy, &sy, &deltx);

	readTruthValue(videoName, videoTruthValues);
	resultTruthValues.clear();
#endif // truth_value

	bOpen = RecordAvi.open(result_file_name.c_str(), cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), 25, cv::Size(imageWidth, imageHeight));
	writeFile = fopen(result_txt_file_name.c_str(), "w");
	bOpen = writeFile != NULL;
#endif

	/* LDWS初始化 */
	LDWS_AllocModel();
	LDWS_Init(ldwsConfigFile, ldwsCustomCalibration);

	//th = LDWS_GetFCWSD_th();

	LDWS_InitGuid *pLDWSInit = NULL;
	//#ifdef _LDWS_init_guid
	LDWS_Getinit(&pLDWSInit);  //pLDWSInit need to be free!!!
	int vanishY = LDWS_GetVanishY();

#ifdef FCWSD
	LDWS_GetResult(&pTest);
	pParam.srcROIYFactor = 0.3;
	pParam.scalingFactor = 1.2;
	pParam.eps = 0.2;
	pParam.srcWidth = imageWidth;
	pParam.srcHeight = imageHeight;
	pParam.fixedTaskNum = 5000;
	pParam.startMFactor = 1.0;
	pParam.useFixedTaskNum = 1;
	pParam.roi.x = 0;
	pParam.roi.y = pParam.srcROIYFactor * imageHeight;
	pParam.roi.width = imageWidth;
	pParam.roi.height = imageHeight - pParam.roi.y;

#if DAY_OR_NIGHT == 0
	FCWSD_Init(0, &pParam, pTest, NULL, fcwdConfigFile);
	pParam.startMFactor = 1.0;
	fcwdConfigFile = "//sh-ws-share-01/Lib/nugetpkgs/Mapa_model/day/Det_day_LBP_20180316_25_d2.txt";
	FCWSD_Init(1, &pParam, pTest, NULL, fcwdConfigFile);
	FCWSD_Init(2, &pParam, pTest, NULL, fcwdConfigFile);
	FCWSD_Init(3, &pParam, pTest, NULL, fcwdConfigFile);
#else
	FCWSD_Init(4, &pParam, pTest, NULL, fcwdConfigFile);
	pParam.startMFactor = 1.0;
	fcwdConfigFile = "\\\\sh-ws-share-01\\Lib\\nugetpkgs\\Mapa_model\\night\\Det_night_20180321_30D1.txt";
	FCWSD_Init(5, &pParam, pTest, NULL, fcwdConfigFile);
	FCWSD_Init(6, &pParam, pTest, NULL, fcwdConfigFile);
	FCWSD_Init(7, &pParam, pTest, NULL, fcwdConfigFile);
#endif

	WissenSize fcwsDetMinSize;
	WissenSize fcwsDetMaxSize;
	fcwsDetMinSize.width = 30 * pParam.startMFactor;
	fcwsDetMinSize.height = 30 * pParam.startMFactor;
	fcwsDetMaxSize.width = 720;
	fcwsDetMaxSize.height = 720;
	initShadowDetection(imageWidth, imageHeight, 0.5f);
#endif

#ifdef OBJVERF
	TrackedObjectList trackedObjectsList = { 0 };
	trackedObjectsList.objectCount = 0;
	trackedObjectsList.trackedObject = (WissenObjectRectTracked *)malloc(sizeof(WissenObjectRectTracked)* MAX_TRACK_MUM);
	VerifyObjectList verifyObjectOutput = { 0 };
	verifyObjectInit(&verifyObjectOutput);
#endif //OBJVERF
#ifdef CNN
	identifyCar.initClassifier(identifyNet, identifyModel);
#endif // CNN

	warningPic = cv::imread("WarningPic.jpg");

	grayADASMat = cv::Mat::zeros(imageHeight, imageWidth, CV_8UC1);
	uMat = cv::Mat::zeros(imageHeight, imageWidth, CV_8UC1);
	vMat = cv::Mat::zeros(imageHeight, imageWidth, CV_8UC1);

	long initTime = (long)clock();

	while (1)
	{
		capture >> frame;

		if (frame.empty())
		{
			printf(" Unload avi!!!!!!!!!!!\n");
			break;
		}
		printf("***********************frame:%d************************* \n", uFramSeq);

		cv::resize(frame, frame, cv::Size(imageWidth, imageHeight));
		frame.copyTo(frameADAS);
		//cv::cvtColor(frameADAS, grayADASMat, CV_BGR2GRAY); 
		cv::cvtColor(frameADAS, yuv, cv::COLOR_BGR2YUV_I420);
		readUVData(yuv.data, imageWidth, imageHeight, grayADASMat, uMat, vMat);

		//for FCWD data
		GrayImg.nWid = grayADASMat.cols;
		GrayImg.nHig = grayADASMat.rows;
		GrayImg.data = grayADASMat.data;

		printf("day or night: %d\n", mvDayOrNight(GrayImg.data, GrayImg.nWid, GrayImg.nHig));

#ifdef LDWS
		ftime(&startTime);
		LDWS_Tracker(GrayImg.data);
		ftime(&endTime);
		COST_TIME("LDWs_process", endTime, startTime);

		LDWS_GetResult(&pTest); //pTest need to be free!!!
		/*memcpy(pTest->pCaPoint, pLDWSInit->pCaPoint,
			sizeof(LDWS_Point)* pLDWSInit->NB_INTERVALLES
			* pLDWSInit->NB_BANDES);*/
		mvDrawLD_Result(frameADAS, pLDWSInit);
#endif

#ifdef FCWSD
		if (pFCWSOutput != NULL)
		{
			pFCWSOutput->nObjectNum = 0;
		}

		if (uFramSeq % 5 == 0)
		{
			ftime(&startTime);
			clock_t t1, t2;

			t1 = clock();
#if DAY_OR_NIGHT == 0
			FCWSD_Processor(0, &GrayImg, pTest, NULL, NULL, &fcwsDetMinSize, &fcwsDetMaxSize, th, 0);
#else
			FCWSD_Processor(4, &GrayImg, pTest, NULL, NULL, &fcwsDetMinSize, &fcwsDetMaxSize, th, 0);
#endif
			t2 = clock();

			printf("t1 is %f, t2 is %f, FCWs_process cost %f\n", (double)t1, (double)t2, (double)(t2 - t1));

			ftime(&endTime);
			//COST_TIME("FCWs_process",endTime,startTime) ;    

#ifdef OBJRLCT
			ftime(&startTime);
			detcorByRec(GrayImg, DAY_OR_NIGHT);
			ftime(&endTime);
			COST_TIME("detcorByRec cost:", endTime, startTime);
#endif

#if DAY_OR_NIGHT == 0
			FCWSD_GetResult(0, &pFCWSOutput);
#else
			FCWSD_GetResult(4, &pFCWSOutput);
#endif
			pFCWSOutput->frameNumber = uFramSeq;
#ifdef CNN
			identifyFCWSDTResult(grayADASMat, pFCWSOutput);
#endif // CNN

			mvDrawDetector_Result(&frameADAS, 0);
		}
#endif


#ifdef FCWST
		sizeScale.width = frameADAS.cols / fzoomRate;
		sizeScale.width = (sizeScale.width + 3) / 4 * 4;
		sizeScale.height = frameADAS.rows / fzoomRate;
		cv::resize(grayADASMat, ResizegrayTemp, sizeScale);
		//cv::resize(uMat, resizeU, sizeScale);
		//cv::resize(vMat, resizeV, sizeScale);
		image_grayTemp = &IplImage(ResizegrayTemp);
		cvGetRawData(image_grayTemp, &grayTemp);

		WissenSize sizeImg;
		sizeImg.width = ResizegrayTemp.cols;
		sizeImg.height = ResizegrayTemp.rows;
		FCW_TRACK_Init_adas_global_param(sizeImg);


#ifdef OBJVERF
		FCW_TRACK_DetcorBytrain(verifyObjectOutput);
#endif

		double carHeight = 0, carWidth = 0;
		int heightNew, yNew1, widthNew, xNew1;
		int imgWidth = GrayImg.nWid;
		int imgheight = GrayImg.nHig;
		int trackNum = 0;
		WissenObjectRectTracked objRec[128];

		if (pFCWSOutput != NULL)
		{
			for (int m = 0; m < pFCWSOutput->nObjectNum; m++)
			{
				if (pFCWSOutput->objects[m].confidence < 0)
					continue;


				objRec[trackNum].object.x = (int)(pFCWSOutput->objects[m].x / fzoomRate + 0.5);
				objRec[trackNum].object.y = (int)(pFCWSOutput->objects[m].y / fzoomRate + 0.5);
				objRec[trackNum].object.width = (int)(pFCWSOutput->objects[m].width / fzoomRate + 0.5);
				objRec[trackNum].object.height = (int)(pFCWSOutput->objects[m].height / fzoomRate + 0.5);
				objRec[trackNum].object.confidence = pFCWSOutput->objects[m].confidence;
				objRec[trackNum].nType = Car;

				trackNum++;
			}
		}

		PortInput InPutParam;
		CANInfo canData;
		canData.fSpeed = 60 / 3.6;
		InPutParam.pGrayfram = grayTemp;
		InPutParam.nFramSeq = uFramSeq;
		InPutParam.pOriGrayfram.data = GrayImg.data;
		InPutParam.pOriGrayfram.nWid = GrayImg.nWid;
		InPutParam.pOriGrayfram.nHig = GrayImg.nHig;
		InPutParam.fzoom = fzoomRate;
		InPutParam.dayOrNight = DAY_OR_NIGHT;

#ifdef INPUT_YUV
		//InPutParam.pOriGrayfram.putr = uMat.data;
		InPutParam.pOriGrayfram.pvtr = vMat.data;
#endif

		/*int shadowThreshold = computeShadow(&InPutParam.pOriGrayfram);
		cv::Mat shadow;
		cv::threshold(grayADASMat, shadow, shadowThreshold, 255, cv::THRESH_BINARY_INV);
		cv::imshow("shadow", shadow);
		InPutParam.pOriGrayfram.groundValue = shadowThreshold;*/

		//printf("******************************%d******************************\n",uFramSeq);

		//GetSystemTime(&currentTime);
		unsigned int milSec = (unsigned int)(clock());

		/*
		printf("time: %u/%u/%u %u:%u:%u:%u %d\n",
		currentTime.wYear,currentTime.wMonth,currentTime.wDay,
		currentTime.wHour,currentTime.wMinute,currentTime.wSecond,
		currentTime.wMilliseconds,currentTime.wDayOfWeek);
		*/
		InPutParam.objTime.wHour = 0; //currentTime.wHour;
		InPutParam.objTime.wMin = 0; //currentTime.wMinute;
		InPutParam.objTime.wSec = 0; //currentTime.wSecond;
		InPutParam.objTime.wMilSec = milSec; //currentTime.wMilliseconds;
		if (pFCWSOutput)
		{
			InPutParam.nRecNum = trackNum;
		}
		else
		{
			InPutParam.nRecNum = 0;
		}
		InPutParam.objRec = objRec;

		pFCWSOutput->nObjectNum = 0;

		ftime(&startTime);
		FCW_TRACK_MultieTrack(&InPutParam, &canData);
		ftime(&endTime);
		COST_TIME("FCMulTrack_process", endTime, startTime);

		printf("clock(): %ld\n", clock() - milSec);

		FCW_TRACK_GetResult(&pTrackOutput);
		trackerObjNum = pTrackOutput->nTrackeNum;
		mvDrawTrack_Result(&frameADAS, fzoomRate);
#endif

#ifdef CNN
		//identifyFCWSTResult(grayADASMat, objVerf, objVerfNum);
#endif //CNN

#ifdef OBJVERF
		int objVerfNum = 0;
		ftime(&startTime);
		if (pTrackOutput != NULL)
		{
			for (int m = 0; m < pTrackOutput->nTrackeNum; m++)
			{
				if (pTrackOutput->pTrackerset[m].bTrue == 1)
				{
					continue;
				}
				trackedObjectsList.trackedObject[objVerfNum].object.x = pTrackOutput->pTrackerset[m].objRec.object.x * fzoomRate;
				trackedObjectsList.trackedObject[objVerfNum].object.y = pTrackOutput->pTrackerset[m].objRec.object.y * fzoomRate;
				trackedObjectsList.trackedObject[objVerfNum].object.width = pTrackOutput->pTrackerset[m].objRec.object.width * fzoomRate;
				trackedObjectsList.trackedObject[objVerfNum].object.height = pTrackOutput->pTrackerset[m].objRec.object.height * fzoomRate;
				trackedObjectsList.trackedObject[objVerfNum].nGroupID = pTrackOutput->pTrackerset[m].nId;
				objVerfNum++;
			}
		}
		trackedObjectsList.objectCount = objVerfNum;
		verifyObjectDetcor(GrayImg, &trackedObjectsList, mode);
		getVerifyObjectResult(&verifyObjectOutput);
		ftime(&endTime);
		COST_TIME("verfObjByDetcor_process", endTime, startTime);
#endif

#ifdef my_record

#ifdef truth_value
		truthValue = truthValueFilter(&GrayImg, NULL, videoTruthValues[uFramSeq]);
		allCount += truthValue.size();
		resultTruthValues.push_back(truthValue);
#endif // truth_value

		if (bOpen)
		{
			RecordAvi.write(frameADAS);
#ifdef FCWST
			writeTrackerReault(writeFile, uFramSeq, fzoomRate, pTrackOutput, pTest);
#else ifdef FCWSD
			writeDetecctorReault(writeFile, uFramSeq, pFCWSOutput, pTest);
#endif
		}
#endif

#ifdef SHOW_RESULT
		cv::line(frameADAS, cv::Point(0, vanishY), cv::Point(imageWidth, vanishY), cv::Scalar(0, 0, 255), 2);
		//cv::line(frameADAS, cv::Point(0, 216), cv::Point(imageWidth, 216), cv::Scalar(0, 0, 252), 2);
		//cv::imshow("u", uMat);
		//cv::imshow("v", vMat);
		cv::imshow("ADAS1", frameADAS);
		//cv::imshow("frame", frame);
		if (cv::waitKey(1) == 27)
		{
			break;
		}
#endif // SHOW_RESULT
		uFramSeq += 1;
	}

#ifdef truth_value
	saveGtResultToTxt(videoName, allCount, resultTruthValues);
#endif // truth_value

#ifdef my_record
	RecordAvi.release();
	fclose(writeFile);
#endif

	FCW_TRACK_mvClearWholeGroup();
	FCW_TRACK_Unit_adas_global_param();
	freeShadowDetection();

#ifdef OBJVERF
	free(trackedObjectsList.trackedObject);
	trackedObjectsList.trackedObject = NULL;
	verifyObjectInit(&verifyObjectOutput);
#endif

#ifdef FCWSD
#if DAY_OR_NIGHT == 0
	FCWSD_Free(0);
	FCWSD_Free(1);
	FCWSD_Free(2);
	FCWSD_Free(3);
#else
	FCWSD_Free(4);
	FCWSD_Free(5);
	FCWSD_Free(6);
	FCWSD_Free(7);
#endif
	FCWSD_FreeResult(&pFCWSOutput);
#endif

#ifdef LDWS
	LDWS_Finalization();
	LDWS_FreeResult(&pTest);
	LDWS_Freeinit(&pLDWSInit);
#endif

	return 0;
}
#endif // DISP

#ifdef DISP
int testDisp(char *videoName, char  *ldwsConfigFile, char* ldwsCustomCalibration)
{
	cv::Mat frame;
	int nFrame = 0;
	char fileName[255] = { 0 };
	cv::Mat yimg1;
	cv::Mat yimg2;
	cv::VideoCapture capture;
	struct timeb startTime, endTime;
	capture.open(videoName);
	if (!capture.isOpened())
	{
		printf("%s can not open!\n", videoName);
		return -1;
	}
	capture.set(CV_CAP_PROP_POS_FRAMES, nFrame);

	RES_DISP resParams;
	TDMAP_TRACK_DISP res3DMap;
	FENCE_LIST drawFrences;

	LDWS_AllocModel();
	LDWS_Init(ldwsConfigFile, ldwsCustomCalibration);

#ifdef my_record
	std::string result_file_name(videoName);
	std::string result_txt_file_name;
	result_file_name.insert((result_file_name.find_last_of('.')), "_disp");
	std::string s(result_file_name, (result_file_name.find_last_of('\\')));//将字符串str内"始于位置stridx"的部分当作字符串的初值 
	std::string savefromat(".avi");
	s.replace(s.find_last_of('.'), 4, savefromat);

	result_file_name = "result" + s;

	std::string savefromat1(".txt");
	s.replace(s.find_last_of('.'), 4, savefromat1);

	result_txt_file_name = "result" + s;

	std::cout << result_file_name.c_str() << std::endl;

	if ((_access("result", 0)) == -1) //文件不存在
	{
		_mkdir("result");
	}

	cv::VideoWriter RecordAvi;
	bool bOpen = 0;
	FILE *writeFile = NULL;

	//bOpen = RecordAvi.open(result_file_name.c_str(), cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), 25, cv::Size(1280, 360));
	writeFile = fopen(result_txt_file_name.c_str(), "w");
	bOpen = writeFile != NULL;
#endif

	//cv::FileStorage fs_one("myMap1.yml", cv::FileStorage::READ); // Read *.yml file, including LUT and Q matrices.
	//cv::Mat RTF_Lx, RTF_Ly, RTF_Rx, RTF_Ry;
	//fs_one["m11"] >> RTF_Lx;
	//fs_one["m12"] >> RTF_Ly;
	//fs_one["m21"] >> RTF_Rx;
	//fs_one["m22"] >> RTF_Ry;
	//fs_one.release();

	//m11 = RTF_Lx.ptr<float>();
	//m12 = RTF_Ly.ptr<float>();
	//m21 = RTF_Rx.ptr<float>();
	//m22 = RTF_Ry.ptr<float>();

	//视差计算构造函数
#if 0
	setDispParams();
	Stereo_detect_init(-2.5, 1.45, 2.5, 150);//-5.8524
#else
	setDispParams();
	Stereo_detect_init();
#endif

	while (1)
	{
		/*capture >> frame;
		if (!frame.data )
		{
		printf(" Unload avi!!!!!!!!!!!\n");
		break;
		}*/
		if (nFrame > 200)
		{
			printf(" Unload avi!!!!!!!!!!!\n");
			break;
		}
		printf("***********************帧号:%d************************* \n", nFrame);

		cv::Rect rect1(0, 0, 960 * 2, 540 * 2);
		cv::Rect rect2(960 * 2, 0, 960 * 2, 540 * 2);
		//cv::Mat Org_L = frame(rect1);
		//cv::Mat Org_R = frame(rect2);

		/*cv::Mat input = cv::imread("cali1.bmp");
		cv::Mat Org_L = input(cv::Rect(0, 0, 1280, 720));
		cv::Mat Org_R = input(cv::Rect(1280, 0, 1280, 720));
		cv::cvtColor(Org_L, yimg1, CV_BGR2GRAY);
		cv::cvtColor(Org_R, yimg2, CV_BGR2GRAY);*/

		uchar *buff_l = new uchar[1280 * 720];
		uchar *buff_r = new uchar[1280 * 720];

		FILE *fid;
		sprintf_s(fileName, "capture\\%d_capture.yuv", nFrame);
		fid = fopen(fileName, "rb");
		fread(buff_l, 1, 1280 * 720, fid);
		fread(buff_r, 1, 1280 * 720, fid);
		fclose(fid);

		cv::Mat yimgl0(720, 1280, CV_8UC1, buff_l);
		cv::Mat yimgr0(720, 1280, CV_8UC1, buff_r);
		cv::Mat Org_L, Org_R;

		cvtColor(yimgl0, Org_L, CV_GRAY2BGR);
		cvtColor(yimgr0, Org_R, CV_GRAY2BGR);
		delete buff_l;
		delete buff_r;

		cv::cvtColor(Org_L, yimg1, CV_BGR2GRAY);
		cv::cvtColor(Org_R, yimg2, CV_BGR2GRAY);

		//matWrite(yimg1, "left0.dat");
		//matWrite(yimg2, "right0.dat");

		ftime(&startTime);

		setereoMatch(yimg1.ptr(), yimg2.ptr());
		RES_DISP resParams;
		getDispResult(&resParams);

		cv::Mat dispImg(resParams.height, resParams.width, CV_16S, resParams.disp);
		cv::Mat dispImg8U;
		dispImg.convertTo(dispImg8U, CV_8U, 0.5);
		cv::imshow("dispImg8U", dispImg8U);

		ftime(&endTime);
		COST_TIME("DISP_process", endTime, startTime);

		ftime(&startTime);
		//------------------------------------3DMap-----------------------------------------------
		TDMAP_TRACK_DISP res3DMap;
		FENCE_LIST drawFrences;
		Stereo_detect_show(resParams, SUBBITS_3);
		Stereo_detect_getresult(&res3DMap, &drawFrences, 5.0f);

		////////////////////////////////////////画短侧//////////////////////////////////////
		cv::Mat frameDVR_3DMap = Org_L.clone();

		//给检测到的物体按远近排序

		for (int j = 0; j < res3DMap.nTrackeNum - 1; j++)
		{
			for (int i = 0; i < res3DMap.nTrackeNum - 1 - j; i++)
			{
				TDMAP_TRACK_OBJ temp_obj;

				if (res3DMap.Trackerset[i].dist < res3DMap.Trackerset[i + 1].dist)
				{
					temp_obj = res3DMap.Trackerset[i];
					res3DMap.Trackerset[i] = res3DMap.Trackerset[i + 1];
					res3DMap.Trackerset[i + 1] = temp_obj;
				}
			}
		}
		int blue, green, red;
		int up, down, left, right;
		//给各个物体绘制灰色半透明区域以及四边直角框
		for (int i = 0; i < res3DMap.nTrackeNum; i++)
		{
			if (res3DMap.Trackerset[i].bTrue == 0 || res3DMap.Trackerset[i].bBackground == 1)
			{
				continue;
			}
			//需要跟踪的目标
			up = res3DMap.Trackerset[i].frame2D.up;
			down = res3DMap.Trackerset[i].frame2D.down;
			left = res3DMap.Trackerset[i].frame2D.left;
			right = res3DMap.Trackerset[i].frame2D.right;

			if (res3DMap.Trackerset[i].bTrue == 1)
			{
				blue = 170;
				green = 170;
				red = 200;
			}
			else if (res3DMap.Trackerset[i].bTrue == 2)
			{
				blue = 200;
				green = 170;
				red = 100;
			}
			//绘制灰色半透明区域
			for (int v = up + 1; v < down; v++)
			{
				for (int u = left + 1; u < right; u++)
				{
					if (v < 0 || v >= 720 || u < 0 || u > 1280)
						continue;

					cv::Vec3b &pt = frameDVR_3DMap.at<cv::Vec3b>(v, u);
					pt[0] = pt[0] * 0.5 + blue * 0.5;
					pt[1] = pt[1] * 0.5 + green * 0.5;
					pt[2] = pt[2] * 0.5 + red * 0.5;
				}
			}
			//上方半透明框 标注速度
			blue = 150;
			green = 150;
			red = 150;
			for (int v = up - 70; v < up - 10; v++)
			{
				for (int u = left + 1; u < left + 150; u++)
				{
					if (v < 0 || v >= 720 || u < 0 || u >= 1280)
						continue;

					cv::Vec3b &pt = frameDVR_3DMap.at<cv::Vec3b>(v, u);
					pt[0] = pt[0] * 0.3 + blue * 0.7;
					pt[1] = pt[1] * 0.3 + green * 0.7;
					pt[2] = pt[2] * 0.3 + red * 0.7;
				}
			}

			char message[50];
			sprintf(message, "%2.1fm", res3DMap.Trackerset[i].dist);
			putText(frameDVR_3DMap, message, cv::Point(left + 5, up - 28), CV_FONT_HERSHEY_COMPLEX, 1.3, cv::Scalar(255, 255, 255), 3);


			//绘制四角框
			//取得颜色
			int color = res3DMap.Trackerset[i].nColor;
			red = (color & 0x0000ff);
			green = (color & 0x00ff00) >> 8;
			blue = (color & 0xff0000) >> 16;

			int length = res3DMap.Trackerset[i].frame3D.biggest_disp;

			cv::line(frameDVR_3DMap, cv::Point2f(left, up), cv::Point2f(left + length, up), cv::Scalar(red, green, blue), 5, 8, 0);
			cv::line(frameDVR_3DMap, cv::Point2f(left, up), cv::Point2f(left, up + length), cv::Scalar(red, green, blue), 5, 8, 0);
			cv::line(frameDVR_3DMap, cv::Point2f(left, down), cv::Point2f(left + length, down), cv::Scalar(red, green, blue), 5, 8, 0);
			cv::line(frameDVR_3DMap, cv::Point2f(left, down), cv::Point2f(left, down - length), cv::Scalar(red, green, blue), 5, 8, 0);
			cv::line(frameDVR_3DMap, cv::Point2f(right, up), cv::Point2f(right - length, up), cv::Scalar(red, green, blue), 5, 8, 0);
			cv::line(frameDVR_3DMap, cv::Point2f(right, up), cv::Point2f(right, up + length), cv::Scalar(red, green, blue), 5, 8, 0);
			cv::line(frameDVR_3DMap, cv::Point2f(right, down), cv::Point2f(right - length, down), cv::Scalar(red, green, blue), 5, 8, 0);
			cv::line(frameDVR_3DMap, cv::Point2f(right, down), cv::Point2f(right, down - length), cv::Scalar(red, green, blue), 5, 8, 0);
		}


		//报警
		LDWS_InitGuid *pLDWSInit = NULL;
		LDWS_Getinit(&pLDWSInit);
		LDWS_Point ptVanish, ptOrg, ptTel;

		int CarWidth = 185;
		int CameraPosx = 210;
		int LeftDeviation = 20;
		int RoadWidth = 370;

		float32_t coeff = (CameraPosx - (CarWidth / 2 + LeftDeviation)) * 1.0f / RoadWidth;
		ptOrg.x = pLDWSInit->pBoundPoint[3].x + (pLDWSInit->pBoundPoint[2].x - pLDWSInit->pBoundPoint[3].x) * coeff;
		ptOrg.y = 720;

		ptTel.x = ptOrg.x + (pLDWSInit->pBoundPoint[2].x - pLDWSInit->pBoundPoint[3].x) * (CarWidth * 1.0f / RoadWidth);
		ptTel.y = 720;
		LDWS_GetVanishPointSet(&ptVanish);

		float32_t slpOrg = (ptVanish.x - ptOrg.x) * 1.0f / (ptVanish.y - ptOrg.y);
		float32_t slpTel = (ptVanish.x - ptTel.x) * 1.0f / (ptVanish.y - ptTel.y);

		for (int v = 0; v < 720; v++)
		{
			for (int u = 0; u < 1280; u++)
			{
				float32_t org = ptOrg.x + slpOrg * (v - ptOrg.y);
				float32_t tel = ptTel.x + slpTel * (v - ptTel.y);

				if (u > org && u < tel && v > ptVanish.y)
				{
					cv::Vec3b &pt = frameDVR_3DMap.at<cv::Vec3b>(v, u);
					pt[0] = 0;
					pt[1] = 0;
					pt[2] = 0;
				}
			}
		}


		//画短册
		for (int i = 0; i < drawFrences.nFenceNum; i++)
		{
			int x = drawFrences.frences[i].frame2D.left;
			int y = drawFrences.frences[i].frame2D.up;
			int width = drawFrences.frences[i].frame2D.right - drawFrences.frences[i].frame2D.left;
			int height = drawFrences.frences[i].frame2D.down - drawFrences.frences[i].frame2D.up;
			int color = drawFrences.frences[i].nColor;
			red = (color & 0x0000ff);
			green = (color & 0x00ff00) >> 8;
			blue = (color & 0xff0000) >> 16;
			cv::rectangle(frameDVR_3DMap, cv::Rect(x, y, width, height), cv::Scalar(red, green, blue), 1, 8, 0);
		}

		cv::Mat DVR_resize;
		resize(frameDVR_3DMap, DVR_resize, cv::Size(640, 360), 0, 0, CV_INTER_LINEAR);

		cv::Mat Video_frame(360, 1280, CV_8UC3, cv::Scalar(0));

		for (int u = 0; u < 640; u++)
		{
			for (int v = 0; v < 360; v++)
			{
				Video_frame.at<cv::Vec3b>(v, u) = DVR_resize.at<cv::Vec3b>(v, u);
			}
		}

		for (int u = 0; u < 320; u++)
		{
			for (int v = 0; v < 128; v++)
			{
				short value = (resParams.disp[v * 320 + u]);
				if (value > 255)
					value = 255;
				if (value < 0)
					value = 0;


				cv::Vec3b value_vec;
				value_vec[0] = value;
				value_vec[1] = value;
				value_vec[2] = value;
				Video_frame.at<cv::Vec3b>(v, u + 640) = value_vec;
			}
		}

		cv::imshow("DVR3DMap", Video_frame);
		nFrame++;
		cv::waitKey();
	}

	releaseDispBuff();
	Stereo_detect_release();


	LDWS_Finalization();

	_CrtDumpMemoryLeaks();

	return 0;
}
#endif // DISP

int main(int argc, char *argv[])
{
	char *ldwsConfigFile = "\\\\sh-ws-share-01\\Lib\\nugetpkgs\\Mapa_video\\day\\param_camera_MaPa.dat";
	char *ldwsCustomCalibration = "\\\\sh-ws-share-01\\Lib\\nugetpkgs\\Mapa_video\\day\\CarCalibration_MaPa.txt";
#ifdef DISP
	testDisp("disp.avi", ldwsConfigFile, ldwsCustomCalibration);
#else //FC LD


#if DAY_OR_NIGHT == 0
	int vehicleThreshold = 6;
	char *videoName = "C:\\Users\\wslipj\\Desktop\\Mapa_20180605145011_stream0_1_20_0.avi";
	char *fcwdConfigFile = "\\\\sh-ws-share-01\\Lib\\nugetpkgs\\Mapa_model\\day\\Det_day_LBP_20180131_30_d2.txt";
	char *identifyNet = "\\\\sh-ws-share-01\\Lib\\nugetpkgs\\Mapa_model\\car_deploy.prototxt";
	char* identifyModel = "\\\\sh-ws-share-01\\Lib\\nugetpkgs\\Mapa_model\\day\\fc-car_iter_400000_Day6.caffemodel";
#else
	int vehicleThreshold = 21;
	char *videoName = "\\\\sh-ws-share-01\\Lib\\nugetpkgs\\Mapa_video\\night\\Mapa_20180705110209_stream0_1_24_1.avi";
	char *fcwdConfigFile = "\\\\sh-ws-share-01\\Lib\\nugetpkgs\\Mapa_video\\night\\Det_Night20180704_30D2.txt";
	char *identifyNet = "\\\\sh-ws-share-01\\Lib\\nugetpkgs\\Mapa_model\\car_deploy.prototxt";
	char* identifyModel = "\\\\sh-ws-share-01\\Lib\\nugetpkgs\\Mapa_model\\night\\fc-car_night_iter_200000.caffemodel";
#endif

	//videoName = argv[1];
	/*vehicleThreshold = atoi(argv[2]);
	fcwdConfigFile = "Det_night_20180319_30D3.txt";
	ldwsConfigFile = "param_camera_HuaYang121.dat";
	ldwsCustomCalibration = "CarCalibration_test.txt";*/

	testLDAndFCW(videoName,
		ldwsConfigFile,
		ldwsCustomCalibration,
		fcwdConfigFile, identifyNet, identifyModel, vehicleThreshold);
#endif 
}