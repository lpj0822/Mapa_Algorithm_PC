/**************************************************************************************************************
Copyright © Wissen Intelligent Sensing Technology Co., Ltd. 2011-2017. All rights reserved.
File name: FCWSD_Interface.c
Version: 1.0		Date: 2017-02-22		Author: Yanming Wang		ID: 1407930

Description:
	FCWS is the abbreviation of Forward Collision Warning System, FCWSD discribs the detection function of FCWS.
	The functions in this file are defined as the interface functions of vehicle detection.
	The following function types are included:
	+ FCWSD_Init(): The initialization of variables and structures to be used in FCWSD functions.
	+ FCWSD_Processor(): The main procedure of vehicle detection. Given the buff of input image and detection 
	                     parameters, the locations of vehicle will be given as the output.
	+ FCWSD_GetResult(): Get the result of detected vehicle.
	+ FCWSD_Free(): Free the memory space of variables.

Deviation: 'FCWSD_' is used as the prefix of vehicle detection interface functions

History:
	+ Version: 1.0		Date: 2017-02-22		Author: Yanming Wang	ID: 1407930
	  Modification: Coding Standards are added.
**************************************************************************************************************/

#include "FCWSD_Interface.h"
#include "utils.h"
#include "utility_function.h"
#include "vehicle_type.h"
#include "vehicle_det.h"
#include "vehicle_proposals.h"
#include <stdlib.h>

/* Global parameters for detector */
static FCWSDetectorGlobalPara pVehicleDetor[DETECT_HANDLE_NUM] = {0};
static FCWSDetectorROI		  g_DetectDefaultROI[DETECT_HANDLE_NUM][ROI_SEGMENT_NUM];

/*
Function process:
	+ Init variables and structures to be used in FCWSD functions
	Fan-in : 
	        + main()
	Fan-out:
	        + FCWD_InitVehicle()
	        + FCWD_InitVehicleDetprocess_Rio()
	ATTENTION: __________
*/
int FCWSD_Init(const int index, const FCWSDParam *pParam, const void *pLDWSOutput, const void *p3DMapOutput, const char *file)
{
	int ret = 0;
	WissenSize defaultMinObjectSize;
	WissenSize defaultMaxObjectSize;

	if (index < DETECT_HANDLE_NUM && index >= 0 
		&& pParam != NULL 
		&& pParam->startMFactor >= 1.0
		&& pParam->srcROIYFactor >= 0.0
		&& pParam->srcWidth != 0
		&& pParam->srcHeight != 0
		&& pParam->useFixedTaskNum >= 0
		&& pParam->useFixedTaskNum <= 1
		&& pParam->fixedTaskNum >=0)
	{
		if( pParam->useFixedTaskNum == 1 && pParam->fixedTaskNum ==0)
		{
			my_printf("Init failed, please check the pParam->fixedTaskNum! \n");
		    return 0;
		}
	
		pVehicleDetor[index].srcWidth			= pParam->srcWidth;
		pVehicleDetor[index].srcHeight			= pParam->srcHeight;
		pVehicleDetor[index].srcROIYFactor		= pParam->srcROIYFactor;
		pVehicleDetor[index].fixedTaskNum		= pParam->fixedTaskNum;
		pVehicleDetor[index].startMFactor		= pParam->startMFactor;
		pVehicleDetor[index].useFixedTaskNum	= pParam->useFixedTaskNum;
		pVehicleDetor[index].roi                = pParam->roi;

		pVehicleDetor[index].index = index;//detector index




#ifdef FCWS_DETECTOR_DEBUG
		pVehicleDetor[index].detframesUsed	= 1;
		pVehicleDetor[index].showFCWDebug	= 1;
#endif

		ret = FCWD_InitVehicle(&pVehicleDetor[index], pLDWSOutput, p3DMapOutput, file, pParam->scalingFactor, pParam->eps);

		pVehicleDetor[index].currentWindowSize  = pVehicleDetor[index].pdetorModel->l->data.featureWidth;

		if (ret == 0)
		{
			my_printf("Init failed, FCWD_InitVehicle, please check the param \n");
			return 0;
		}

		defaultMinObjectSize.width	= pVehicleDetor[index].pdetorModel->l->data.featureWidth;
		defaultMinObjectSize.height = pVehicleDetor[index].pdetorModel->l->data.featureHeight;
		pVehicleDetor[index].aspectRatio = (double) defaultMinObjectSize.width / defaultMinObjectSize.height;

		defaultMaxObjectSize.width	= (int) WS_MIN((int)(pVehicleDetor[index].aspectRatio * (1 - pParam->srcROIYFactor) * pParam->srcHeight),pParam->srcWidth);
		defaultMaxObjectSize.height = (int) ((1 - pParam->srcROIYFactor) * pParam->srcHeight);

		ret = FCWD_InitVehicleDetProcess_Rio(&pVehicleDetor[index], index, &g_DetectDefaultROI[index][0], \
			                                 pLDWSOutput, NULL, &defaultMinObjectSize, &defaultMaxObjectSize);

		if (ret == 0)
		{
			my_printf("Init failed, FCWD_InitVehicleDetProcess_Rio, please check the param \n");
			return 0;
		}

#ifdef USE_PROPOSALS
		if (index == 0)
		{
			ret = initProposals(pParam->srcWidth, pParam->srcHeight, pParam->srcROIYFactor);

			if (ret == 0)
			{
				my_printf("Init failed, initProposals, please check the param \n");
				return 0;
			}
		}
#endif
	}
	else
	{
		my_printf("Init failed, please check the param\n");
		return 0;
	}

	return 1;
}

/*
Function process:
	+ The main procedure of vehicle detection.
	Fan-in : 
	        + main()
	Fan-out:
	        + FCWD_VehicleDetProcess_Rio()
	ATTENTION: __________
*/
int FCWSD_Processor(const int index, const WissenImage *pOriGrayImg, const void *pLDWSOutput, const void *p3DMapOutput,
	WissenRect *roi, const WissenSize *minObjectSize, const WissenSize *maxObjectSize, const int groupThreshold, const int maxRT)
{
	int ret;
	if(roi != NULL)
	{
		/* roi should inside of pVehicleDetor[index].roi */
		roi->x = WS_MAX(pVehicleDetor[index].roi.x, roi->x);
		roi->y = WS_MAX(pVehicleDetor[index].roi.y, roi->y);
		roi->width = WS_MIN(pVehicleDetor[index].roi.x + pVehicleDetor[index].roi.width, 
							  roi->x + roi->width) - roi->x;
		roi->height = WS_MIN(pVehicleDetor[index].roi.y + pVehicleDetor[index].roi.height,
							  roi->y + roi->height) - roi->y;
	}

	ret = FCWD_VehicleDetProcess_Rio(&pVehicleDetor[index], &g_DetectDefaultROI[index][0], pOriGrayImg, pLDWSOutput, NULL, roi, minObjectSize, maxObjectSize, groupThreshold, maxRT);

	return ret;
}

int  FCWSD_Processor_ROI(const int index, const int VerFlg, const WissenImage *pGrayImg, WissenRect *roi, \
	const WissenSize *minObjectSize, const WissenSize *maxObjectSize, const int groupThreshold)
{
	int ret;
	if(roi != NULL)
	{
		/* roi should inside of pVehicleDetor[index].roi */
		roi->x = WS_MAX(pVehicleDetor[index].roi.x, roi->x);
		roi->y = WS_MAX(pVehicleDetor[index].roi.y, roi->y);
		roi->width = WS_MIN(pVehicleDetor[index].roi.x + pVehicleDetor[index].roi.width,
							  roi->x + roi->width) - roi->x;
		roi->height = WS_MIN(pVehicleDetor[index].roi.y + pVehicleDetor[index].roi.height,
							  roi->y + roi->height) - roi->y;
	}

	ret = FCW_DETCOR_Vehicle_Rio(&pVehicleDetor[index], &g_DetectDefaultROI[index][0], pGrayImg,minObjectSize,maxObjectSize,roi,groupThreshold, VerFlg);
	return ret;
}

/*
Function process:
+ Do the re-det around the pSrcRect and get the new location as pDetRec
Fan-in :
+ mvReLoaclDetRec()
Fan-out:
+ N/A
ATTENTION: __________
*/
void detcorByRec(const WissenImage GrayImg, const unsigned char dayOrNight)
{

	int i;
	WissenObjectRect pSrcRect;
	WissenObjectRect OriRioRec;
	int nExtend_x, nExtend_y;
	WissenSize fcwsDetMinSize;
	WissenSize fcwsDetMaxSize;
	WissenRect roi;
	WissenObjectRect dstRect = { 0, 0, GrayImg.nWid - 1, GrayImg.nHig - 1, 0};

	/* get the global det result */
	objectSetsCar fcwsdInput;
	if (dayOrNight == 0)
	{
		fcwsdInput = pVehicleDetor[0].objSets;
	}
	else if (dayOrNight == 1)
	{
		fcwsdInput = pVehicleDetor[4].objSets;
	}

	for (i = 0; i < fcwsdInput.nObjectNum; ++i)
	{
		pSrcRect = fcwsdInput.objects[i];

		if (pSrcRect.width > 0)
		{
			nExtend_x = (int)(pSrcRect.width * 0.2f);
			nExtend_y = (int)(pSrcRect.height * 0.2f);
			OriRioRec.x = (pSrcRect.x - nExtend_x);
			OriRioRec.y = (pSrcRect.y - nExtend_y);
			OriRioRec.width = (pSrcRect.width + (nExtend_x << 1));
			OriRioRec.height = (pSrcRect.height + (nExtend_y << 1));

			limitObjectRectRange(dstRect, &OriRioRec);

			fcwsDetMinSize.width = (int)(pSrcRect.width
				/ 1.2f - 1);
			fcwsDetMinSize.height = (int)(pSrcRect.height
				/ 1.2f - 1);

			fcwsDetMaxSize.width = (int)(pSrcRect.width
				* 1.2f + 1);
			fcwsDetMaxSize.height = (int)(pSrcRect.height
				* 1.2f + 1);

			roi.x = OriRioRec.x;
			roi.y = OriRioRec.y;
			roi.width = OriRioRec.width;
			roi.height = OriRioRec.height;

			if (dayOrNight == 0)
			{
				FCWSD_Processor_ROI(2, 1, &GrayImg, &roi, &fcwsDetMinSize,
					&fcwsDetMaxSize, 1);
				if (1 == pVehicleDetor[2].objSets.nObjectNum)
				{
					pVehicleDetor[0].objSets.objects[i].x = pVehicleDetor[2].objSets.objects[0].x;
					pVehicleDetor[0].objSets.objects[i].y = pVehicleDetor[2].objSets.objects[0].y;
					pVehicleDetor[0].objSets.objects[i].width = pVehicleDetor[2].objSets.objects[0].width;
					pVehicleDetor[0].objSets.objects[i].height = pVehicleDetor[2].objSets.objects[0].height;
				}
				else
				{
					pVehicleDetor[0].objSets.objects[i].confidence = -255;
				}
			}
			else if (dayOrNight == 1)
			{
				FCWSD_Processor_ROI(6, 1, &GrayImg, &roi, &fcwsDetMinSize,
					&fcwsDetMaxSize, 1);
				if (1 == pVehicleDetor[6].objSets.nObjectNum)
				{
					pVehicleDetor[4].objSets.objects[i].x = pVehicleDetor[6].objSets.objects[0].x;
					pVehicleDetor[4].objSets.objects[i].y = pVehicleDetor[6].objSets.objects[0].y;
					pVehicleDetor[4].objSets.objects[i].width = pVehicleDetor[6].objSets.objects[0].width;
					pVehicleDetor[4].objSets.objects[i].height = pVehicleDetor[6].objSets.objects[0].height;
				}
				else
				{
					pVehicleDetor[4].objSets.objects[i].confidence = -255;
				}
			}	

			
		}
	}
	
}

/*
Function process:
	+ Get the result of detected vehicle.
	Fan-in : 
	        + main()
	Fan-out:
	        + FCWD_GetDetResult()
	ATTENTION: __________
*/
int FCWSD_GetResult(const int index, objectSetsCar **pFCWSDOutput)
{
	int ret;

	ret = FCWD_GetDetResult(&pVehicleDetor[index], pFCWSDOutput);
	
	return ret;
}

void getDetectObject(const int index, objectSetsCar **pFCWSOutput)
{
	*pFCWSOutput = &pVehicleDetor[index].objSets;
}

int FCWSD_RefineResult(objectSetsCar *pFCWSOutput, int imgWidth, int imgheight)
{
	double carHeight = 0, carWidth = 0;
	int j, heightNew, yNew1, widthNew, xNew1;
	if (pFCWSOutput != NULL)
	{
		for (j = 0 ;j < pFCWSOutput->nObjectNum; j++)
		{
			carHeight = LDWS_GetImageY((pFCWSOutput->objects[j].y + pFCWSOutput->objects[j].height), pFCWSOutput->objects[j].y);

			carWidth = LDWS_GetDetaXofWorld(pFCWSOutput->objects[j].width, pFCWSOutput->objects[j].y+ pFCWSOutput->objects[j].height);
			
   			heightNew = LDWS_GetCarY(pFCWSOutput->objects[j].y+ pFCWSOutput->objects[j].height, 1.8);
			yNew1 = pFCWSOutput->objects[j].y + pFCWSOutput->objects[j].height - heightNew;
			widthNew = pFCWSOutput->objects[j].width * 1.8 / carWidth;
			xNew1 = 0.5 * ((2 * pFCWSOutput->objects[j].x + pFCWSOutput->objects[j].width) - widthNew);

			if(xNew1>0 && yNew1>0 && (xNew1 + widthNew)< imgWidth && (yNew1 + heightNew)< imgheight)
			{
				pFCWSOutput->objects[j].x = xNew1;
				pFCWSOutput->objects[j].y = yNew1;
				pFCWSOutput->objects[j].width = WS_MAX(widthNew,heightNew);
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

/*
Function process:
	+ Free the result of detected vehicle.
	Fan-in : 
	        + main()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
int FCWSD_FreeResult(objectSetsCar **pFCWSDOutput)
{
	if ((*pFCWSDOutput)->objects != NULL)
	{
		my_free((*pFCWSDOutput)->objects);
		(*pFCWSDOutput)->objects = NULL;
	}
	my_free(*pFCWSDOutput);
	*pFCWSDOutput = NULL;


    return 0;
}

/*
Function process:
	+ Free the memory space of variables.
	Fan-in : 
	        + main()
	Fan-out:
	        + FCWD_UnitVehicle()
	ATTENTION: __________
*/
int FCWSD_Free(const int index)
{
	int ret;

	ret = FCWD_UnitVehicle(&pVehicleDetor[index], &g_DetectDefaultROI[index][0]);

#ifdef USE_PROPOSALS
	if (index == 0)
	{
		ret = freeProposals();
	}
#endif // USE_PROPOSALS

	return ret;
}

