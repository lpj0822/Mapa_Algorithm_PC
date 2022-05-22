#include "OBJVERF_Interface.h"
#include <stdlib.h>
#include "utils.h"
#include "utility_function.h"

#define  MAX_PUBLIC_SPACE_SIZE  314572

//verify
static VerifyObjectList verifyObjectResult;

//resize
static int *resizeArrayX[2] = { NULL };
static int *resizeArrayY[2] = { NULL };

static void resizeObjectData(const WissenImage pOriGrayfram, const WissenObjectRect objRec, const int resizeWidth, const int resizeHeight, int *tempX, int *tempY, float* objData)
{
	const int imageWidth = pOriGrayfram.nWid;
	int row = 0;
	int col = 0;
	float factorWidth = (float)objRec.width / resizeWidth;
	float factorHeight = (float)objRec.height / resizeHeight;
	const unsigned char* grayImage = pOriGrayfram.data + imageWidth * objRec.y;
	const unsigned char* tempSrc = NULL;
	float* tempDst = NULL;
	for (col = 0; col < resizeWidth; col++)
	{
		tempX[col] = (int)(col * factorWidth);
	}
	for (row = 0; row < resizeHeight; row++)
	{
		tempY[row] = (int)(row * factorHeight);
	}
	for (row = 0; row < resizeHeight; row++)
	{
		tempSrc = grayImage + imageWidth * tempY[row] + objRec.x;
		tempDst = objData + resizeWidth * row;
		for (col = 0; col < resizeWidth; col++)
		{
			tempDst[col] = *(tempSrc + tempX[col]) - 99.0f;
		}
	}
}

void verifyObjectInit(VerifyObjectList *verifyObjectOutput)
{
	verifyObjectResult.objectCount = 0;
	verifyObjectResult.object = (VerifyObject *)malloc(sizeof(VerifyObject)* MAX_TRACK_MUM);
	if (verifyObjectOutput->object == NULL)
	{
		verifyObjectOutput->objectCount = 0;
		verifyObjectOutput->object = (VerifyObject *)malloc(sizeof(VerifyObject)* MAX_TRACK_MUM);
	}
}

/*
Function process:
+ verify the history target in m_globlparam[nId].m_pGroupSets
Fan-in :
+ mvSingledScaleTrack()
Fan-out:
+ FCWSD_Processor_ROI()
+ mvSureTrueGroup()
ATTENTION: __________
*/ 
void verifyObjectDetcor(const WissenImage pOriGrayfram, const TrackedObjectList *inputObject, DayNightMode mode)
{

	int  i;
	WissenObjectRect RioRec;
	unsigned char nDetCarNum = 0;
	WissenSize fcwsDetMinSize;
	WissenSize fcwsDetMaxSize;
	WissenRect roi;
	WissenImage imgRioTemp;
	WissenObjectRect dstRect = { 0, 0, pOriGrayfram.nWid - 1, pOriGrayfram.nHig - 1, 0};

	for (i = 0; i < inputObject->objectCount; i++)
	{
		RioRec = inputObject->trackedObject[i].object;

		if (RioRec.width * RioRec.height < MAX_PUBLIC_SPACE_SIZE)
		{
			
			RioRec.x -= (float)(RioRec.width * 0.2f);
			RioRec.y -= (float)(RioRec.height * 0.2f);
			RioRec.width += (float)(RioRec.width * 0.4f);
			RioRec.height += (float)(RioRec.height * 0.4f);

			limitObjectRectRange(dstRect, &RioRec);

			fcwsDetMinSize.width = (int)(inputObject->trackedObject[i].object.width
				/ 1.2f - 1);
			fcwsDetMinSize.height = (int)(inputObject->trackedObject[i].object.height
				/ 1.2f - 1);

			fcwsDetMaxSize.width = (int)(inputObject->trackedObject[i].object.width
				* 1.2f + 1);

			fcwsDetMaxSize.height = (int)(inputObject->trackedObject[i].object.height
				* 1.2f + 1);

			roi.x = RioRec.x;
			roi.y = RioRec.y;
			roi.width = RioRec.width;
			roi.height = RioRec.height;

			imgRioTemp.data = pOriGrayfram.data;
			imgRioTemp.nWid = inputObject->trackedObject[i].object.width;
			imgRioTemp.nHig = inputObject->trackedObject[i].object.height;

			if (mode == DAY)
			{
				nDetCarNum = FCWSD_Processor_ROI(1, 0, &imgRioTemp, &roi,
					&fcwsDetMinSize, &fcwsDetMaxSize, 1);
			}
			else if (mode == NIGHT)
			{
				nDetCarNum = FCWSD_Processor_ROI(5, 0, &imgRioTemp, &roi,
					&fcwsDetMinSize, &fcwsDetMaxSize, 1);
			}
		}
		else
		{
			nDetCarNum = 0;
		}

		verifyObjectResult.object[i].nGroupID = inputObject->trackedObject[i].nGroupID;
		verifyObjectResult.object[i].nObjState = nDetCarNum ? 1 : 0;
	}
	verifyObjectResult.objectCount = inputObject->objectCount;
}

void getVerifyObjectResult(VerifyObjectList *verifyObjectOutput)
{
	int index = 0;
	for (index = 0; index < verifyObjectResult.objectCount; index++)
	{
		verifyObjectOutput->object[index] = verifyObjectResult.object[index];
	}
	verifyObjectOutput->objectCount = verifyObjectResult.objectCount;
}

void verifyObjectRelease(VerifyObjectList *verifyObjectOutput)
{
	if (verifyObjectResult.object != NULL)
	{
		verifyObjectResult.objectCount = 0;
		free(verifyObjectResult.object);
		verifyObjectResult.object = NULL;
	}
	if (verifyObjectOutput->object != NULL)
	{
		verifyObjectOutput->objectCount = 0;
		free(verifyObjectOutput->object);
		verifyObjectOutput->object = NULL;
	}
}

void initResizeCnn(const int index, const int resizeWidth, const int resizeHeight)
{
	if (resizeArrayX[index] == NULL)
	{
		resizeArrayX[index] = (int*)malloc(sizeof(int)* resizeWidth);
	}
	if (resizeArrayY[index] == NULL)
	{
		resizeArrayY[index] = (int*)malloc(sizeof(int)* resizeHeight);
	}
}

void resizeCnnInputObject(const int index, const WissenImage *pOriGrayImg, const WissenObjectRect rect, const int resizeWidth, const int resizeHeight, float* objectData)
{
	resizeObjectData(*pOriGrayImg, rect, resizeWidth, resizeHeight, resizeArrayX[index], resizeArrayY[index], objectData);
}

void releaseResizeCnn(const int index, const int resizeWidth, const int resizeHeight)
{
	if (resizeArrayX[index] != NULL)
	{
		free(resizeArrayX[index]);
		resizeArrayX[index] = NULL;
	}
	if (resizeArrayY[index] != NULL)
	{
		free(resizeArrayY[index]);
		resizeArrayY[index] = NULL;
	}
}