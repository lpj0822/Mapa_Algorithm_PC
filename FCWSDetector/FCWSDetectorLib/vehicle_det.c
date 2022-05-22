/**************************************************************************************************************
Copyright 漏 Wissen Intelligent Sensing Technology Co., Ltd. 2011-2017. All rights reserved.
File name: vehicle_det.h
Version: 1.0		Date: 2017-02-22		Author: Yanming Wang		ID: 1407930

Description:
	The functions in this file are the realization of vehicle detection.
	The following function types are included:
	+ FCWD_InitVehicle(): The initialization of variables and structures to be used in FCWSD functions.
	+ FCWD_UnitVehicle(): Free the memory space of variables.

	+ FCWD_InitVehicleDetprocess_Rio():Init tasks for detector
	+ FCWD_VehicleDetprocess_Rio(): Main realization for vehicle detection
	+ FCWD_GetDetResult(): Get the detector results

	static function:
	+ initTaskWindowsCar(): Assign the scanning windows (tasks) for chained list
	+ lbpDetectWindowsCar(): Detect scanning windows for one single scaling factor
	+ getResizeIntegralImage(): Calculate the integral image
	+ userGetTime(): Return system time (ms)
	+ lbpGroupWindowsCar(): Merge overlapped detect rectangles
	+ freeTaskWindowsCar(): Free the memory of single scale tasks chained list
	+ freeVehicleDetProcess_Rio(): Free the memory of multi-scale detector chained list

Deviation: 'FCWSD_' is used as the prefix of vehicle detection functions in FCWS

History:
	+ Version: 1.0		Date: 2017-02-22		Author: Yanming Wang	ID: 1407930
	  Modification: Coding Standards are added.
**************************************************************************************************************/

#include "utils.h"
#include "vehicle_type.h"
#include "vehicle_det.h"
#include "lbp_detect.h"
#include "group_rect.h"
#include "vehicle_proposals.h"

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#ifdef USE_GPU
	//#include "GPULbpDetect.h"
     #include "gpu.h"
#endif

nrowCol arrRowCol[15 * 1024];
/*
I/O:	    Name		    Type	                       Size			  	                      Content
					    								  
[in/out]	p		        FCWSDetectorMultiscaleList*	   sizeof(FCWSDetectorMultiscaleList*)	  chained list for Multiscale tasks.
[in]	    step		    const int	                   4-Byte                                 step for scanning window.
[in]	    minObjectSize	const WissenSize*		       sizeof(WissenSize*)                     Minimum size for detector.
					    								  
[out]	returned value  int          4-Byte	              If 0, initialized failed.

Realized function:
	+ Assign the scanning windows (tasks) for chained list.
*/
static int	initTaskWindowsCar(FCWSDetectorMultiscaleList *p, const int step, const WissenSize *minObjectSize, const int sizeLimit);

/*
I/O:	 Name		    Type	       Size			  	    Content
		 			    			  					    
[in/out] l		        lbpCar*	       sizeof(lbpCar*)	    Structural for detector
[in]     img            unsigned int*           sizeof(unsigned int*)         integral image
[in]     factor         float      sizeof(float)    Scaling factor for the detect window
[in]     roiFlg         unsigned char  sizeof(uchar)        if roiFlg ==1, detector is used for ROI region
					    								    
[out]	returned value  int            4-Byte	            Number of detected objects.

Realized function:
	+ Detect scanning windows for one single scaling factor.
*/
static unsigned int  lbpDetectWindowsCar( lbpCar *l, unsigned int *img, const float factor, const unsigned char roiFlg, const nrowCol *arrRowColDet);

/*
I/O:	  Name		    Type	                    Size			  	                  Content
		 			    			  					    
[in]      pOriGrayImg	WissenImage*	                sizeof(WissenImage*)   	              origin gray image
[in/out]  iData         unsigned int*                        sizeof(unsigned int*)                          integral image
[in]      detROI        WissenRect*                  sizeof(WissenRect*)                    ROI region for calculate integral image
[in]      p             FCWSDetectorMultiscaleList  sizeof(FCWSDetectorMultiscaleList*)   chained list for Multiscale tasks.
					    								    
[out]	returned value  int                         4-Byte	                              if 0, function failed.

Realized function:
	+ Calculate the integral image.
*/
static unsigned int	getResizeIntegralImage(const WissenImage *pOriGrayImg, unsigned int *iData, const WissenRect *detROI, const FCWSDetectorMultiscaleList *p);

/*
I/O:	  Name		    Type	                    Size			  	                  Content
		 			    			  					    					    								    
[out]	returned value  double                      8-Byte	                              Return system time.

Realized function:
	+ Return system time (ms).
*/
static double userGetTime(void);

/*
I/O:	   Name		        Type	                 Size			  	      Content

[in]       pVehicleDetor    FCWSDetectorGlobalPara*                           Global parameter for detector
[in]       l		        lbpCar*	                 sizeof(lbpCar*)          LBP structure for vehicle detector 
[in/out]   pobjSets		    objectSetsCar*	         sizeof(objectSetsCar*)	  The detected objects
[in]       roiStartPoint	WissenPoint*	             sizeof(WissenPoint*)	  Offset of the detected ROI region
[in]       groupThreshold	int	                     4-Byte	                  Threshold for a rect group being a cluster
					     			  					  
[out]	   returned value   int                      4-Byte	                  Number of detected objects.

Realized function:
	+ Merge overlapped detect rectangles.
*/
static int lbpGroupWindowsCar(FCWSDetectorGlobalPara *pVehicleDetor, lbpCar *l, objectSetsCar *pobjSets, WissenPoint *roiStartPoint, int groupThreshold);

static void object_detector_lbp_group_window_car_ROI(int index, FCWSDetectorGlobalPara *pVehicleDetor, lbpCar *l, objectSetsCar *pobjsets, WissenPoint *RioStartPoint, int group_threshold);

/*
I/O:	    Name		Type	                      Size			  	                   Content
					   						  
[in/out]	p		    FCWSDetectorMultiscaleList*	  sizeof(FCWSDetectorMultiscaleList*)  chained list for single scale tasks.
					   						  
[out]	returned value  int                           4-Byte	                            If 0, free failed.

Realized function:
	+ Free the memory of single scale tasks chained list.
*/
static int freeTaskWindowsCar(FCWSDetectorMultiscaleList *p);

/*
I/O:	    Name		    Type	               Size			  	            Content
					    								  
[in/out]	p		        FCWSDetectorROI*	   sizeof(FCWSDetectorROI*)	    chained list for multi-scale tasks.
					    								  
[out]	returned value      int                    4-Byte	                    If 0, free failed.

Realized function:
	+ Free the memory of multi-scale detector chained list.
*/
static int freeVehicleDetProcess_Rio(FCWSDetectorROI *p);



/*
Function process:
	+ Detect scanning windows for one single scaling factor.
	Fan-in :
	        + FCWD_InitVehicleDetProcess_Rio()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
static int initTaskWindowsCar(FCWSDetectorMultiscaleList *p, const int step, const WissenSize *minObjectSize, const int sizeLimit)
{
	int x, y,taskNum = 0;
	float vehicleHeight = 0;

	LBPTaskCarList *t1 = NULL;
	LBPTaskCarList *t2 = NULL;

	/* free in freeTaskWindowsCar */
	p->pLBPTCLhead = (LBPTaskCarList *) my_malloc(sizeof(LBPTaskCarList));
	memset(p->pLBPTCLhead, 0, sizeof(LBPTaskCarList));

	if (p->pLBPTCLhead == NULL)
	{
		return 0;
	}

	/* free at the end of this function */
	t1 = (LBPTaskCarList *) my_malloc(sizeof(LBPTaskCarList));
	memset(t1, 0, sizeof(LBPTaskCarList));

	if (t1 == NULL)
	{ 
		return 0;
	}
	else 
	{
		t2 = t1;
		p->pLBPTCLhead->next = NULL; 
	}

	for ( y = 0; y < p->roiRec.height - minObjectSize->height - 1; y += step)
	{
		for ( x = 0; x < p->roiRec.width - minObjectSize->width - 1 ;x += step) 
		{	
#ifdef USE_LDWS
			if(sizeLimit)
			{
				vehicleHeight = LDWS_GetDetaXofWorld(minObjectSize->height * p->factor, (y + minObjectSize->height) * p->factor + p->roiRec.y);

				if (vehicleHeight < 1 || vehicleHeight > 4)
					continue;
			}
#endif // USE_LDWS
			t1->task.x = x;
			t1->task.y = y;

			taskNum++;
			if (taskNum == 1)
			{
				p->pLBPTCLhead = t1;
				t2->next	   = NULL;
			}
			else
			{
				t2->next = t1;
			}
			t2 = t1;
			t1 = (LBPTaskCarList *)malloc(sizeof(LBPTaskCarList));
			memset(t1, 0, sizeof(LBPTaskCarList));

		}
	}

	if(taskNum == 0)
	{
		free(p->pLBPTCLhead);
		p->pLBPTCLhead = NULL;
	}

	t2->next = NULL;

	if(t1 != NULL)
	    free(t1);
	t1 = NULL; 
	
//#ifdef FCWS_DETECTOR_DEBUG

	p->ntask = taskNum;
//#endif

	return 1;

 }

/*
Function process:
	+ Assign the scanning windows (tasks) for chained list
	Fan-in :
	        + FCWD_VehicleDetProcess_Rio()
	Fan-out:
	        + lbpDetectCar()
	ATTENTION: __________
*/
static unsigned int lbpDetectWindowsCar( lbpCar *l, unsigned int *img, const float factor, const unsigned char roiFlg, const nrowCol *arrRowColDet)
{
#ifdef USE_GPU
	gpu_speed(l, img, factor);
#else

	int t = 0;
	int found = 0;
	WissenObjectRect rectCar; /* detector result */

	if (roiFlg == 0)
	{
#ifdef OPENMP
#pragma omp parallel for private(found) schedule(guided) num_threads(2)
#endif // OPENMP
		for (t = 0; t < l->ntaskNum; t++)
		{
#if ACC_DETECTOR
			int arrNrow = l->ptasks[t].x + l->ptasks[t].y * l->width;
#else
			int arrNrow = 0;
#endif

			found = lbpDetectCar(l, img, l->ptasks[t].x, l->ptasks[t].y, arrNrow, arrRowColDet);
			if (found)
			{
#ifdef OPENMP
#pragma omp critical
#endif // OPENMP
				{
					rectCar.x = (int)(l->ptasks[t].x * factor);
					rectCar.y = (int)(l->ptasks[t].y * factor);
					rectCar.width = (int)(l->data.featureWidth * factor);
					rectCar.height = (int)(l->data.featureHeight * factor);
					l->pdetectedRect[l->ndetdNum++] = rectCar;
				}
			}
		}
	}
	else if (roiFlg == 1) /* for ROI object detect, Openmp is not used */
	{
		for (t = 0; t < l->ntaskNum; t++)
		{
#if ACC_DETECTOR
			int arrNrow = l->ptasks[t].x + l->ptasks[t].y * l->width;
#else
			int arrNrow = 0;
#endif
			found = lbpDetectCar(l, img, l->ptasks[t].x, l->ptasks[t].y, arrNrow,arrRowColDet);
			if (found) 
			{
				{
					rectCar.x = (int)(l->ptasks[t].x * factor);
					rectCar.y = (int)(l->ptasks[t].y * factor);
					rectCar.width = (int)(l->data.featureWidth * factor);
					rectCar.height = (int)(l->data.featureHeight * factor);
					l->pdetectedRect[l->ndetdNum++] = rectCar;
				}
			}
		} 
	}
#endif
	return l->ndetdNum;

}

/*
Function process:
	+ Calculate the integral image.
	Fan-in :
	        + FCWD_VehicleDetProcess_Rio()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
static unsigned int getResizeIntegralImage(const WissenImage *pOriGrayImg, unsigned int *iData, const WissenRect *detROI, const FCWSDetectorMultiscaleList *p)
{
	int i, j;
	unsigned int rs; 
	unsigned char *PSrc  = NULL;
	unsigned int		*PData = NULL; 

	//PSrc = pOriGrayImg->data + p->arr_y[0];
	PSrc = pOriGrayImg->data + p->arr_y[detROI->y];// according to original image (0,0)
	PData = iData + detROI->y * p->roiRec.width;// according to p->roiRec (p->roiRec.point.x, p->roiRec.point.y)
	rs = 0;
	for (i = detROI->x ;i < detROI->width + detROI->x; i++)
	{
        rs		 +=  PSrc[p->arr_x[i]];
        PData[i] = rs;
	}

	for ( j = detROI->y + 1 ; j < detROI->height + detROI->y; j++)
	{
		PSrc = pOriGrayImg->data + p->arr_y[j];

		PData = iData + (j - 1) * p->roiRec.width;

		rs = 0;

		for (i = detROI->x; i < detROI->width + detROI->x; i++)
		{
            rs		+= PSrc[p->arr_x[i]];
            PData[i + p->roiRec.width] = rs + PData[i];
		}
	}

	return 1;
}

/*
Function process:
	+ Return system time (ms).
	Fan-in :
	        + FCWD_VehicleDetProcess_Rio()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
static double userGetTime(void) 
{
	return 0;
}

/*
Function process:
	+ Merge overlapped detect rectangles.
	Fan-in :
	        + getGroupedRectanglesCar()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
static int lbpGroupWindowsCar(FCWSDetectorGlobalPara *pVehicleDetor, lbpCar *l, objectSetsCar *pobjSets, WissenPoint *roiStartPoint, int groupThreshold)
{
	int i;
	int index = 0;
	unsigned char dayOrNight = 0;
	if (pVehicleDetor->index >= 4)
	{
		dayOrNight = 1;
	}
	/* merge overlapped rectangles */
	getGroupedRectanglesCar(pVehicleDetor, l->pdetectedRect, &l->ndetdNum, groupThreshold, l->para.eps);
	
	if (dayOrNight == 0)
	{
		for (i = 0; i < l->ndetdNum; i++)
		{
			pobjSets->objects[i].x = l->pdetectedRect[i].x + roiStartPoint->x;
			pobjSets->objects[i].y = l->pdetectedRect[i].y + roiStartPoint->y;
			pobjSets->objects[i].width = l->pdetectedRect[i].width;
			pobjSets->objects[i].height = l->pdetectedRect[i].height;
			pobjSets->objects[i].confidence = l->pdetectedRect[i].confidence;
		}
		pobjSets->nObjectNum = l->ndetdNum;
	}
	else if (dayOrNight == 1)
	{
		for (i = 0, index = 0; i < l->ndetdNum; i++)
		{
			if (l->pdetectedRect[i].width > 140)
			{
				if (l->pdetectedRect[i].confidence > groupThreshold * 2)
				{
					pobjSets->objects[index].x = l->pdetectedRect[i].x + roiStartPoint->x;
					pobjSets->objects[index].y = l->pdetectedRect[i].y + roiStartPoint->y;
					pobjSets->objects[index].width = l->pdetectedRect[i].width;
					pobjSets->objects[index].height = l->pdetectedRect[i].height;
					pobjSets->objects[index].confidence = l->pdetectedRect[i].confidence;
					index++;
				}
			}
			else
			{
				pobjSets->objects[index].x = l->pdetectedRect[i].x + roiStartPoint->x;
				pobjSets->objects[index].y = l->pdetectedRect[i].y + roiStartPoint->y;
				pobjSets->objects[index].width = l->pdetectedRect[i].width;
				pobjSets->objects[index].height = l->pdetectedRect[i].height;
				pobjSets->objects[index].confidence = l->pdetectedRect[i].confidence;
				index++;
			}

		}
		pobjSets->nObjectNum = index;
	}
	
	l->ndetdNum = 0;

	return pobjSets->nObjectNum;
}

static void object_detector_lbp_group_window_car_ROI(int index, FCWSDetectorGlobalPara *pVehicleDetor, lbpCar *l, objectSetsCar *pobjsets, WissenPoint *RioStartPoint, int group_threshold)
{
	int i = 0, maxId = 0, maxConf = -1;
	WissenObjectRect r;
	//lbpRectCar r, r1;
	int Weight;
	float s; 
	if (index == 1)/* location */
	{
		/* merge overlapped rectangles */
		getGroupedRectanglesCar(pVehicleDetor, l->pdetectedRect, &l->ndetdNum, group_threshold, l->para.eps*2);
	
		for ( i = 0; i < l->ndetdNum; i++)
		{
			if (l->pdetectedRect[i].confidence > maxConf)
			{
				maxConf = l->pdetectedRect[i].confidence;
				maxId = i;
			}			
		}

		pobjsets->objects[0].x = l->pdetectedRect[maxId].x + RioStartPoint->x;
		pobjsets->objects[0].y = l->pdetectedRect[maxId].y + RioStartPoint->y;
		pobjsets->objects[0].width = l->pdetectedRect[maxId].width;
		pobjsets->objects[0].height = l->pdetectedRect[maxId].height;
		pobjsets->objects[0].confidence = l->pdetectedRect[maxId].confidence;

		pobjsets->nObjectNum = 1;

		l->ndetdNum = 0;
	}
	else
	{
		r.x = 0;
		r.y = 0;
		r.width = 0;
		r.height = 0;
		Weight = 0;
		for ( i = 0; i < l->ndetdNum; i++)
		{
			r.x += l->pdetectedRect[i].x;
			r.y += l->pdetectedRect[i].y;
			r.width += l->pdetectedRect[i].width;
			r.height += l->pdetectedRect[i].height;
			Weight ++;
		}
		s= 1.f / Weight;
		pobjsets->objects[0].x = WS_MyRound(r.x*s) + RioStartPoint->x;
		pobjsets->objects[0].y = WS_MyRound(r.y*s) + RioStartPoint->y;
		pobjsets->objects[0].width = WS_MyRound(r.width*s);
		pobjsets->objects[0].height = WS_MyRound(r.height*s);
		pobjsets->objects[0].confidence	= l->ndetdNum;
		pobjsets->nObjectNum = 1;

		l->ndetdNum = 0;
	}
}

/*
Function process:
	+ Free the memory of single scale tasks chained list..
	Fan-in :
	        + FCWD_UnitVehicle()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
static int freeTaskWindowsCar(FCWSDetectorMultiscaleList *p)
{
	LBPTaskCarList *node, *temp;
	FCWSDetectorMultiscaleList *nodeFCWS,*tempFCWS;

	if (!p)
	{
		return 0;
	}

	nodeFCWS = p;

	while(nodeFCWS != NULL)
	{
		tempFCWS = nodeFCWS;

	    node = tempFCWS->pLBPTCLhead;
	
		while (node != NULL)
		{
			temp = node;
			node = node->next;
			my_free(temp);
		}

	    tempFCWS->pLBPTCLhead = NULL;

		my_free(tempFCWS->arr_x);
		tempFCWS->arr_x = NULL;

		my_free(tempFCWS->arr_y);
		tempFCWS->arr_y = NULL;

		my_free(tempFCWS->arrRowCol);
		tempFCWS->arrRowCol = NULL;
		nodeFCWS = nodeFCWS->next;
	}

	p->next = NULL;

	return 1;
}

/*
Function process:
	+ Free the memory of multi-scale tasks chained list..
	Fan-in :
	        + FCWD_UnitVehicle()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
static int freeVehicleDetProcess_Rio(FCWSDetectorROI *p)
{
	FCWSDetectorMultiscaleList *node, *temp;

	if (!p)
	{
		return 0;
	}

	node = p->pDMLhead;
	
	while (node != NULL)
	{
		temp = node;
		node = node->next;
		my_free(temp);
	}

	p->pDMLhead = NULL;

	return 1;
}


/*
Function process:
	+ Init variables and structures to be used in FCWSD functions
	Fan-in :
	        + FCWSD_Init()
	Fan-out:
	        + loadLbpDataCar()
	ATTENTION: __________
*/
int FCWD_InitVehicle(FCWSDetectorGlobalPara *pVehicleDetor, const void *pLDWSOutput,const void *p3DMapOutput, const char *file, const double scalingFactor, const double eps)
{
	unsigned char *pTr = NULL;
	int		mallcoSize = 0;
	int		bLoadSuccess = 0;

#ifdef USE_LDWS
	LDWS_Output *pLDWS = NULL;
	if(pLDWSOutput != NULL)
	{
	    pLDWS	 = (LDWS_Output *)pLDWSOutput;
	}
#endif

	mallcoSize = MAX_PUBLIC_BUFFER					               /* pbulicBuff */
				+ sizeof(objectDetCar)			                   /* object_det */
				+ sizeof(lbpCar)					               /* object_det->l */
				+ MAX_IMAGE_WIDTH *  MAX_IMAGE_HIGHT * sizeof(unsigned int) /* object_det->integral_img */
				+MAX_TASKS_NUM * sizeof(WissenPoint)			   /* object_det->l->ptasks */
				+MAX_DETECT_NUM * sizeof(WissenObjectRect)			   /* object_det->l->pdetected_r */
				+MAX_DETECT_NUM * sizeof(WissenObjectRect)			   /* ObjectSets_car.objects */
				+ (MAX_TREE_NODE<<1) * sizeof(int)				   /* PARENT + RANK */
				+ (MAX_TREE_NODE) * sizeof(int);				   /* labels */
 
#ifdef USE_LDWS	
	if (pLDWS != NULL)
	{
		mallcoSize += pLDWS->NB_INTERVALLES * pLDWS->NB_BANDES * sizeof(LDWS_Point) << 1;
	}
#endif


	mallcoSize += (1024 << 3) ; 

	pVehicleDetor->pAlloc = (unsigned char*) my_malloc(mallcoSize);
	memset(pVehicleDetor->pAlloc, 0, mallcoSize);

	if (pVehicleDetor->pAlloc == NULL) 
	{
		my_printf("FCWSD Malloc Error.\n");
	}

	pTr = pVehicleDetor->pAlloc;

	pVehicleDetor->pbulicBuff = pTr;
	pTr = ALIGN_16BYTE(pVehicleDetor->pbulicBuff + MAX_PUBLIC_BUFFER);

	pVehicleDetor->pdetorModel = (objectDetCar *)pTr;
	pTr = ALIGN_16BYTE(pVehicleDetor->pdetorModel + 1); 

	pVehicleDetor->pdetorModel->l = (lbpCar *)pTr;
	pTr = ALIGN_16BYTE(pVehicleDetor->pdetorModel->l + 1); 

	pVehicleDetor->pdetorModel->integralImg = (unsigned int *)pTr;
	pTr = ALIGN_16BYTE(pVehicleDetor->pdetorModel->integralImg + MAX_IMAGE_WIDTH * MAX_IMAGE_HIGHT); 

	pVehicleDetor->pdetorModel->l->ptasks = (WissenPoint *)pTr;
	pTr = ALIGN_16BYTE(pVehicleDetor->pdetorModel->l->ptasks + MAX_TASKS_NUM); 
	
	pVehicleDetor->pdetorModel->l->pdetectedRect = (WissenObjectRect *)pTr;
	pTr = ALIGN_16BYTE(pVehicleDetor->pdetorModel->l->pdetectedRect + MAX_DETECT_NUM); 

	pVehicleDetor->objSets.objects = (WissenObjectRect *)pTr;
	pTr = ALIGN_16BYTE(pVehicleDetor->objSets.objects  + MAX_DETECT_NUM);

	pVehicleDetor->pTreeNode = (int *)pTr;
	pTr = ALIGN_16BYTE(pVehicleDetor->pTreeNode + (MAX_TREE_NODE << 1)); 

	pVehicleDetor->pLabNode = (int *)pTr;
	pTr = ALIGN_16BYTE(pVehicleDetor->pLabNode + MAX_TREE_NODE);

#ifdef USE_LDWS
	if (pLDWS != NULL)
	{
		pVehicleDetor->roiPoint = (LDWS_Point *)pTr; 
		pTr = ALIGN_16BYTE(pVehicleDetor->roiPoint + pLDWS->NB_INTERVALLES * pLDWS->NB_BANDES);		

		pVehicleDetor->roiPointCopy = (LDWS_Point *)pTr; 
		pTr = ALIGN_16BYTE(pVehicleDetor->roiPointCopy + pLDWS->NB_INTERVALLES * pLDWS->NB_BANDES);				
	}
#endif
	
	pVehicleDetor->pdetorModel->l->ntaskNum = 0;
	pVehicleDetor->pdetorModel->l->ndetdNum = 0;
	pVehicleDetor->objSets.nObjectNum		= 0;

	pVehicleDetor->pdetorModel->l->para.scalingFactor		= scalingFactor;
	pVehicleDetor->pdetorModel->l->para.eps					= eps;
	
	bLoadSuccess = loadLbpDataCar(pVehicleDetor->pdetorModel->l, file);

	pVehicleDetor->pdetorModel->l->para.minSizeWidth		= pVehicleDetor->pdetorModel->l->data.featureWidth;
	pVehicleDetor->initFlg = 0;

  
	return bLoadSuccess;
}

/*
Function process:
	+ Init chained list for Multiscale tasks
	Fan-in :
	        + FCWSD_Init()
	Fan-out:
	        + initTaskWindowsCar()
	ATTENTION: In caculation of integral image, Width is supposed to be larger then height
*/
int FCWD_InitVehicleDetProcess_Rio(FCWSDetectorGlobalPara *pVehicleDetor, const int index, 
								   FCWSDetectorROI	*pDetectotDefaultROI, const void *pLDWSOutput, 
								   const void *p3DMapOutput, const WissenSize *minObjectSize,
								   const WissenSize	*maxObjectSize)
{
	int j, k, step, iScaleCnt, returnVal;

	float factor, fSum, startFactor, scaleFactor;
	
	WissenSize windowSize;
	
	WissenRect roiRec;

	int fstepHight;

	FCWSDetectorMultiscaleList *p1 = NULL;
	FCWSDetectorMultiscaleList *p2 = NULL;

#ifdef FCWS_DETECTOR_DEBUG
	int totalTaskNum;
#endif

	/* Vanish line is set to be a constant value */
	pVehicleDetor->vanishY = pVehicleDetor->srcROIYFactor * MAX_IMAGE_HIGHT;

	//if(index == 0)
	   fstepHight = (MAX_IMAGE_HIGHT - pVehicleDetor->vanishY) * ROI_SEGMENT_FACTOR;
	//else
    //   fstepHight = (MAX_IMAGE_HIGHT) * ROI_SEGMENT_FACTOR;

	for ( j = 0; j < ROI_SEGMENT_NUM; ++j )
	{
		//if(index == 0)
		    pDetectotDefaultROI[j].posInfor = (int)( pVehicleDetor->vanishY + (j + 1) * fstepHight - 1);
		//else
		//	pDetectotDefaultROI[j].posInfor = (int)((j + 1) * fstepHight - 1);

		//LOGI("In %d Section, %d is end at %d \n", index, j, pDetectotDefaultROI[j].posInfor);

		if ( j == ROI_SEGMENT_NUM - 1)
		{
			iScaleCnt = 0;

#ifdef FCWS_DETECTOR_DEBUG

			totalTaskNum = 0;
#endif
			/* free at the end of this function */
			p1 = (FCWSDetectorMultiscaleList *)malloc(sizeof(FCWSDetectorMultiscaleList));
			if (p1 == NULL)
			{ 
				return 0;
			} 
			else 
			{
				p2 = p1;
				pDetectotDefaultROI[j].pDMLhead = NULL; 
			}

			//if(index == 0)
			{
				scaleFactor = pVehicleDetor->pdetorModel->l->para.scalingFactor;//*1.2;
			    startFactor = pVehicleDetor->startMFactor;
			}
			//else
			//{
			//	scaleFactor = 1.2;
			//	startFactor = pVehicleDetor->startMFactor;
			//}

			for ( factor = startFactor; ; factor *=scaleFactor )
			{
				windowSize.height = (int)( minObjectSize->height * factor );
				windowSize.width  = (int)(pVehicleDetor->aspectRatio * windowSize.height);

				if ( windowSize.width > maxObjectSize->width || windowSize.height > maxObjectSize->height ||\
					 windowSize.height > fstepHight*ROI_SEGMENT_NUM || windowSize.width > pVehicleDetor->roi.width ||
					 windowSize.height > pVehicleDetor->roi.height)
				{
					break;
				}

				roiRec.x = pVehicleDetor->roi.x;
				roiRec.y = pVehicleDetor->roi.y;
				roiRec.width	= (int)(pVehicleDetor->roi.width / factor);
				roiRec.height	= (int)(pVehicleDetor->roi.height / factor);

				step = factor > 4. ? 1 : 2;//1 : 2;

				p1->windowSize = windowSize;
				p1->factor	   = factor;
				p1->roiRec	   = roiRec;

				/* free at freeTaskWindowsCar */
				p1->arr_y = (int *) my_malloc(sizeof(int) * roiRec.height);
				memset(p1->arr_y, 0, sizeof(int) * roiRec.height);
				p1->arr_x = (int *) my_malloc(sizeof(int) * roiRec.width);
				memset(p1->arr_x, 0, sizeof(int) * roiRec.width);

				/* ATTENTION: In caculation of integral image, Width is supposed to be larger then height */
				fSum = -factor;
				for (k = 0 ; k < roiRec.height; ++k)
				{
					fSum += factor;
					p1->arr_x[k] = (int)(fSum + roiRec.x );
					p1->arr_y[k] = (int)(fSum + roiRec.y ) * MAX_IMAGE_WIDTH;
				}
				for (; k < roiRec.width; k++)
				{
					fSum += factor;
					p1->arr_x[k] = (int)(fSum + roiRec.x );
				}

				//if(index == 0)
				{
					if(pLDWSOutput != NULL)
					{
				        returnVal = initTaskWindowsCar( p1, step, minObjectSize, 1);
					}
					else
					{
						returnVal = initTaskWindowsCar( p1, step, minObjectSize, 0);
					}

				   if(returnVal == 0)
					   return 0;

				}
				//else
				//{
				//	p1->pLBPTCLhead = NULL;
				//}

				/*acculate part*/
				p1->arrRowCol = NULL;
#if ACC_DETECTOR
				if (1)
				{
					int s, w, n, t = 0;
					lbpCar *l;
					l = pVehicleDetor->pdetorModel->l;
					p1->arrRowCol = (nrowCol *)malloc(sizeof(nrowCol) * l->data.totalWeakNum);
					for (s = 0; s < l->data.stagesNum; s++) 
					{
						for (w = 0; w < l->data.s[s].weakClassifiersNum; w++) 
						{
							const WissenObjectRect *r = l->data.r;
							const weakClassifierCar *c = &l->data.s[s].classifiers[w];

							for (n = 0; n < l->data.leafNum - 1; n++)
							{
								p1->arrRowCol[t].nrow[n] = r[c->rectIdx[n]].x + r[c->rectIdx[n]].y * p1->roiRec.width;

								p1->arrRowCol[t].ncol[n] =  r[c->rectIdx[n]].height * p1->roiRec.width;

								p1->arrRowCol[t].ncolAdd[n] =  r[c->rectIdx[n]].width;	
							}
							t++;
						}
					}
				}
#endif    
	
				iScaleCnt++;

#ifdef FCWS_DETECTOR_DEBUG

				totalTaskNum += p1->ntask;
				printf("ScaleLevel=%d, width=%d, height=%d, LevelTaskNum=%d \n", iScaleCnt, windowSize.width, windowSize.height, p1->ntask);
#endif
				if (iScaleCnt == 1)
				{
					pDetectotDefaultROI[j].pDMLhead = p1;
					p2->next = NULL;
				}
				else
				{
					p2->next = p1;
				}
				p2 = p1; 

				/* free at the end of this function */
				p1 = (FCWSDetectorMultiscaleList *) my_malloc(
						sizeof(FCWSDetectorMultiscaleList));
						
				memset(p1, 0, sizeof(FCWSDetectorMultiscaleList));

			}
			p2->next = NULL;
			my_free(p1);
			p1 = NULL;

#ifdef FCWS_DETECTOR_DEBUG
			printf("In all of %d Scales,totalTaskNum = %d \n", iScaleCnt,totalTaskNum);
#endif

			return 1;
		}
	}

    return 0;
}

/*
Function process:
	+ The main function for multiScale vehicle detection, can be used for ROI region detection.
	Fan-in :
	        + FCWSD_Processor()
	Fan-out:
	        + getResizeIntegralImager()
			+ lbpDetectWindowsCar()
			+ lbpGroupWindowsCar()
	ATTENTION: In caculation of integral image, Width is supposed to be larger then height
*/
int	FCWD_VehicleDetProcess_Rio(FCWSDetectorGlobalPara *pVehicleDetor, 
							   FCWSDetectorROI		  *pDetectotDefaultROI, 
							   const WissenImage			  *pOriGrayImg,
							   const void				  *pLDWSOutput, 
							   const void				  *p3DMapOutput,
							   const WissenRect			  *roi,
							   const WissenSize			  *minObjectSize,
							   const WissenSize			  *maxObjectSize,
							   const int				  groupThreshold, 
							   const int				  maxRT)
{
	int taskNum = 0;
	int sectionIndex = 0;
	int roiFlg = 0;
	int flgPosInfo		= 0;
	int totalTaskNum	= 0;
	int detNum =0;
	int detROIHeight = 0;
	//int minx = 10000;
	//int maxx = -1;
	//int miny = 10000;
	//int maxy = -1;

	WissenPoint	roiStartPoint = { 0 };
	WissenRect	detROI = { 0 };

#ifdef USE_LDWS
	int posInfor[ROI_SEGMENT_NUM];
#endif

	FCWSDetectorMultiscaleList	*p  = NULL;
	LBPTaskCarList				*t1 = NULL;

	float tStart = 0;
	float tEnd = 0;

#ifdef USE_PROPOSALS
	unsigned char dayOrNight = 0;
#endif

#ifdef USE_LDWS
	/* Vanishing line position in scale image */
	int vyScale = 0;
	/* end line position in scale image */
	int endScale = pVehicleDetor->srcHeight;
#endif

#ifdef USE_LDWS

	int i;
	LDWS_Output *pLDWS	 = (LDWS_Output *)pLDWSOutput;

	if (roi != NULL)
	{
		pLDWS = NULL;
	}
#endif

#ifdef USE_PROPOSALS
	if (pVehicleDetor->index >= 4)
	{
		dayOrNight = 1;
	}
#endif // USE_PROPOSALS


	if (roi != NULL)
	{
		roiFlg = 1;
	}

	if (maxRT != 0)
	{
		tStart = userGetTime();
	}

	pVehicleDetor->objSets.nObjectNum = 0;

	p = pDetectotDefaultROI[ROI_SEGMENT_NUM - 1].pDMLhead;

	if (pVehicleDetor->useFixedTaskNum)
	{
		while (p != NULL && p->windowSize.width < pVehicleDetor->currentWindowSize)
		{
			p = p->next;
		}

		if(p == NULL)
		    p = pDetectotDefaultROI[ROI_SEGMENT_NUM - 1].pDMLhead;
	}

	if (p != NULL)
	{
		roiStartPoint.x = p->roiRec.x;
		roiStartPoint.y = p->roiRec.y;
	}

#ifdef USE_LDWS

	/*  LDWS--> Providing the ROI, if there is no result, provided the initial value */
	if (pLDWS != NULL )
	{
		int laneWidth = 0;

		for ( i = 0; i < pLDWS->NB_INTERVALLES; i++ )
		{
			pVehicleDetor->roiPoint[i].y = pLDWS->pCaPoint[i].y - roiStartPoint.y;
			if (pVehicleDetor->roiPoint[i].y < 0)
			{
				pVehicleDetor->roiPoint[i].y = 0;
			}
			pVehicleDetor->roiPoint[i + pLDWS->NB_INTERVALLES].y = pVehicleDetor->roiPoint[i].y;

			/* Extended the ROI region for vehicle detection */
			laneWidth = (pLDWS->pCaPoint[i + pLDWS->NB_INTERVALLES].x - pLDWS->pCaPoint[i].x + 1) >> 3;// ;// << 1;
			
			pVehicleDetor->roiPoint[i + pLDWS->NB_INTERVALLES].x = pLDWS->pCaPoint[i + pLDWS->NB_INTERVALLES].x + laneWidth - 1 - roiStartPoint.x;
			pVehicleDetor->roiPoint[i].x						 = pLDWS->pCaPoint[i].x - laneWidth + 1 - roiStartPoint.x;
			
			if (pVehicleDetor->roiPoint[i + pLDWS->NB_INTERVALLES].x >= pOriGrayImg->nWid)
			{
				pVehicleDetor->roiPoint[i + pLDWS->NB_INTERVALLES].x = pOriGrayImg->nWid - 1;
			}
			else if (pVehicleDetor->roiPoint[i + pLDWS->NB_INTERVALLES].x < 0)
			{
				pVehicleDetor->roiPoint[i + pLDWS->NB_INTERVALLES].x = 0;
			}

			if (pVehicleDetor->roiPoint[i].x  >= pOriGrayImg->nWid)
			{
				pVehicleDetor->roiPoint[i].x = pOriGrayImg->nWid - 1;
			}
			else if (pVehicleDetor->roiPoint[i].x < 0)
			{
				pVehicleDetor->roiPoint[i].x = 0;
			}
//			printf("roiPoint[%d].x= %d;  roiPoint[%d].y= %d \n",i,pVehicleDetor->roiPoint[i].x,i,pVehicleDetor->roiPoint[i].y);
//			printf("roiPoint[%d].x= %d;  roiPoint[%d].y= %d \n",i + pLDWS->NB_INTERVALLES,pVehicleDetor->roiPoint[i + pLDWS->NB_INTERVALLES].x,i + pLDWS->NB_INTERVALLES,pVehicleDetor->roiPoint[i + pLDWS->NB_INTERVALLES].y);

 		}

		for( i = 0; i < ROI_SEGMENT_NUM; i++ )
		{
			//posInfor[i] = LDWS_GetCarY( pDetectotDefaultROI[i].posInfor, 1.5);/* if the car is 1.5m and at images position y, calculate the image height */
			posInfor[i] = LDWS_GetXLengthofImage( 1.5, pDetectotDefaultROI[i].posInfor);
			//LOGI("posInfor[%d] = %d \n",i,posInfor[i]);			
		}

	}
#endif

#ifdef USE_PROPOSALS
	computeProposals(dayOrNight, pOriGrayImg);
#endif // USE_PROPOSALS

	/* main function for chain list */
	while (p != NULL)
	{
		sectionIndex = -1;

#ifdef USE_LDWS

		if (pLDWS != NULL )
		{
			for(i = 0; i < ROI_SEGMENT_NUM; i++)
			{
				if( posInfor[i] > (int)(p->windowSize.width * 1.2) && \
					((int)((pDetectotDefaultROI[i].posInfor - p->roiRec.y)/ p->factor) >  (pVehicleDetor->pdetorModel->l->data.featureHeight+1)))
				{
					sectionIndex = i;
					break;
				}
			}  

			for ( i = 0; i < pLDWS->NB_INTERVALLES; i++ )
			{

				pVehicleDetor->roiPointCopy[i].x = (int)((pVehicleDetor->roiPoint[i].x - p->windowSize.width + 1) / p->factor); 
				if (pVehicleDetor->roiPointCopy[i].x < 0)
				{
					pVehicleDetor->roiPointCopy[i].x = 0;
				}
				
				pVehicleDetor->roiPointCopy[i + pLDWS->NB_INTERVALLES].x = (int)((pVehicleDetor->roiPoint[i + pLDWS->NB_INTERVALLES].x) / p->factor);
				
				if (pVehicleDetor->roiPointCopy[i + pLDWS->NB_INTERVALLES].x >= pOriGrayImg->nWid)
				{
					pVehicleDetor->roiPointCopy[i + pLDWS->NB_INTERVALLES].x = pOriGrayImg->nWid - 1;
				}
				
				pVehicleDetor->roiPointCopy[i].y = (int)((pVehicleDetor->roiPoint[i].y) / p->factor);
				
				if (pVehicleDetor->roiPointCopy[i].y < 0)
				{
					pVehicleDetor->roiPointCopy[i].y = 0;
				}
				
				pVehicleDetor->roiPointCopy[i + pLDWS->NB_INTERVALLES].y = pVehicleDetor->roiPointCopy[i].y;
				//my_printf("roiPointCopy[%d].x= %d;  roiPointCopy[%d].y= %d \n",i,pVehicleDetor->roiPointCopy[i].x,i,pVehicleDetor->roiPointCopy[i].y);
			    //my_printf("roiPointCopy[%d].x= %d;  roiPointCopy[%d].y= %d \n",i + pLDWS->NB_INTERVALLES,pVehicleDetor->roiPointCopy[i + pLDWS->NB_INTERVALLES].x,i + pLDWS->NB_INTERVALLES,pVehicleDetor->roiPointCopy[i + pLDWS->NB_INTERVALLES].y);

			}

			vyScale = ( pOriGrayImg->nHig/2 - roiStartPoint.y) / p->factor;//pLDWS->vy
			endScale = (pLDWS->pCaPoint[pLDWS->NB_INTERVALLES-1].y - roiStartPoint.y)/ p->factor;
		}

#endif

		if (sectionIndex < 0)
		{
			flgPosInfo	  = 1;
			sectionIndex  = ROI_SEGMENT_NUM - 1;
		}
		else
		{
			flgPosInfo = 0;
		}
		
		detROIHeight = WS_MIN((pDetectotDefaultROI[sectionIndex].posInfor - p->roiRec.y + 1) / p->factor,p->roiRec.height);

		if( ((int)(p->roiRec.width * p->factor) < minObjectSize->width
			|| (int)(p->roiRec.height * p->factor) < minObjectSize->height ) && flgPosInfo == 1 )
		{
			pVehicleDetor->currentWindowSize = 0;
			break;
		}

		if (roi != NULL)
		{
			if (roi->width < p->windowSize.width || roi->height < p->windowSize.height)
			{
				pVehicleDetor->currentWindowSize = 0;
				break;
			}

			if (p->windowSize.width > maxObjectSize->width || p->windowSize.height > maxObjectSize->height)
			{
				pVehicleDetor->currentWindowSize = 0;
				break;
			}
			if (p->windowSize.width < minObjectSize->width || p->windowSize.height < minObjectSize->height)
			{
				p = p->next;
				continue;
			}
			
			/* roi is for original image, according to 1920*1080 image size */
			detROI.width  = WS_MIN((int)(roi->width / p->factor), p->roiRec.width);
			detROI.height = WS_MIN((int)(roi->height / p->factor),detROIHeight);

			detROI.x = (int)((roi->x - p->roiRec.x) / p->factor);
			if (detROI.x < 0)
			{
				detROI.x = 0;
			}
			detROI.y = (int)((roi->y - p->roiRec.y) / p->factor);
			if (detROI.y < 0)
			{  
				detROI.y = 0;
			}
		}
		else
		{
			detROI.width  = p->roiRec.width;
			detROI.height = detROIHeight;
			detROI.x = 0;
			detROI.y = 0;
		}

		pVehicleDetor->pdetorModel->width		= p->roiRec.width;
		pVehicleDetor->pdetorModel->height		= detROIHeight;
		pVehicleDetor->pdetorModel->l->width	= p->roiRec.width;
		pVehicleDetor->pdetorModel->l->height	= detROIHeight;

		/* chain list for tasks */
		t1 = p->pLBPTCLhead;
		taskNum = 0;

#ifdef USE_LDWS

		if (pLDWS != NULL )
		{
			while (t1 != NULL)
			{
				if(t1->task.y + pVehicleDetor->pdetorModel->l->data.featureHeight > detROI.y + detROIHeight)
					break;
				if (t1->task.y > detROI.y 
				 && t1->task.y + pVehicleDetor->pdetorModel->l->data.featureHeight < detROI.y + detROIHeight)
				{
					for (i = 0; i < pLDWS->NB_INTERVALLES; ++i)
					{
						if (t1->task.y + (pVehicleDetor->pdetorModel->l->data.featureHeight)< pVehicleDetor->roiPointCopy[i].y)
						{
							if (t1->task.x > pVehicleDetor->roiPointCopy[i].x && t1->task.x < pVehicleDetor->roiPointCopy[i + pLDWS->NB_INTERVALLES].x)
							{
								//if(t1->task.y + pVehicleDetor->pdetorModel->l->data.featureHeight > vyScale-15
								//	&& t1->task.y + pVehicleDetor->pdetorModel->l->data.featureHeight < endScale+15)
								{
#ifdef USE_PROPOSALS
									if (filterCarTask(dayOrNight, &t1->task, pVehicleDetor, p->factor) == 1)
#endif
										pVehicleDetor->pdetorModel->l->ptasks[taskNum++] = t1->task;
								}
							}
							break;
						}
						else
						{
							if(i == pLDWS->NB_INTERVALLES-1)
							{
								if (t1->task.y + (pVehicleDetor->pdetorModel->l->data.featureHeight>>1)< pVehicleDetor->roiPointCopy[i].y)
								{
									if (t1->task.x > pVehicleDetor->roiPointCopy[i].x && t1->task.x < pVehicleDetor->roiPointCopy[i + pLDWS->NB_INTERVALLES].x)
									{
										//if(t1->task.y + pVehicleDetor->pdetorModel->l->data.featureHeight > vyScale-15)
										{
#ifdef USE_PROPOSALS
											if (filterCarTask(dayOrNight, &t1->task, pVehicleDetor, p->factor) == 1)
#endif
												pVehicleDetor->pdetorModel->l->ptasks[taskNum++] = t1->task;
										}
									}
									break;
								}
							}
						}
					}
				}
				t1 = t1->next;
			}
		}
		else
		{
			if (roi != NULL)
			{
				while (t1 != NULL)
				{
					if (t1->task.x > detROI.x 
					 && t1->task.y > detROI.y
					 && t1->task.x < detROI.x + detROI.width  - pVehicleDetor->pdetorModel->l->data.featureWidth
					 && t1->task.y < detROI.y + detROI.height - pVehicleDetor->pdetorModel->l->data.featureHeight)
					{
						pVehicleDetor->pdetorModel->l->ptasks[taskNum++] = t1->task;
					}
					t1 = t1->next;
				}
			}
			else
			{
				while (t1 != NULL)
				{
					pVehicleDetor->pdetorModel->l->ptasks[taskNum++] = t1->task;
					t1 = t1->next;
				}
			}
		}
#else

		if (roi != NULL)
		{
			while (t1 != NULL)
			{
				if (t1->task.x >= detROI.point.x
				 && t1->task.y >= detROI.point.y
				 && t1->task.x <= detROI.point.x + detROI.size.width  - pVehicleDetor->pdetorModel->l->data.featureWidth
				 && t1->task.y <= detROI.point.y + detROI.size.height - pVehicleDetor->pdetorModel->l->data.featureHeight)
				{
					pVehicleDetor->pdetorModel->l->ptasks[taskNum++] = t1->task;
				}
				t1 = t1->next;
			}
		}
		else
		{
			while (t1 != NULL)
			{
				pVehicleDetor->pdetorModel->l->ptasks[taskNum++] = t1->task;
				t1 = t1->next;
			}
		}
#endif

		pVehicleDetor->pdetorModel->l->ntaskNum = taskNum;
		
#ifdef FCWS_DETECTOR_DEBUG

		/*if (roi == NULL && pVehicleDetor->showFCWDebug == 1)
		{
		my_printf("D: DetetWindowSize = %d, DetectSection = %d, DetcWinsize = %d, vySacle = %d, endSacle = %d, taskNum = %d \n", p->windowSize.height, sectionIndex, p->roiRec.size.height, vyScale, endScale, taskNum);
		} else*/
		//	else if (pVehicleDetor->showFCWDebug == 1)
		{
			my_printf("T: DetetWindowSize = %d, DetectSection = %d : taskNum = %d \n", p->windowSize.height, sectionIndex, taskNum);

		}
#endif
		totalTaskNum += taskNum;

		getResizeIntegralImage(pOriGrayImg, pVehicleDetor->pdetorModel->integralImg, &detROI, p);

		lbpDetectWindowsCar(pVehicleDetor->pdetorModel->l, pVehicleDetor->pdetorModel->integralImg, p->factor, roiFlg, p->arrRowCol);

#ifdef FCWS_DETECTOR_DEBUG
		my_printf(" l->detNum = %d \n",pVehicleDetor->pdetorModel->l->ndetdNum);
#endif

		if (maxRT != 0)
		{
			tEnd = userGetTime();
			if (tEnd > tStart + maxRT)
			{
				/* object size in last detected size */
				int temp = (int)(p->windowSize.width)*1.1;// / pVehicleDetor->pdetorModel->l->para.scalingFactor);

				if (temp > pVehicleDetor->currentWindowSize)
				{
					pVehicleDetor->currentWindowSize = temp - 1;
				}
				else
				{
					pVehicleDetor->currentWindowSize = p->windowSize.width + 1;
				}

				//my_printf("FCWS Detector timeout,and exit!\n");

				p = p->next;
				if (pVehicleDetor->useFixedTaskNum && p == NULL)
				{
					pVehicleDetor->currentWindowSize = 0;

#ifdef FCWS_DETECTOR_DEBUG

					if (pVehicleDetor->showFCWDebug == 1)
					{
						//my_printf("totalTaskNum = %d \n", totalTaskNum);
					}
					//my_printf("Finish all Scales in %d Detected Frames! \n",	pVehicleDetor->detframesUsed);

					pVehicleDetor->detframesUsed = 1;
#endif
				}

				break;
			}
		}

		if (totalTaskNum > pVehicleDetor->fixedTaskNum && pVehicleDetor->useFixedTaskNum )
		{
			int temp = (int)(p->windowSize.width)*1.1;// / pVehicleDetor->pdetorModel->l->para.scalingFactor);

#ifdef FCWS_DETECTOR_DEBUG

			if (pVehicleDetor->showFCWDebug)
			{
				//my_printf("totalTaskNum = %d \n",totalTaskNum);
			}
#endif
			if (temp > pVehicleDetor->currentWindowSize)
			{
			    pVehicleDetor->currentWindowSize = temp - 1;
			}
			else
			{
				pVehicleDetor->currentWindowSize = p->windowSize.width + 1;
			}

#ifdef FCWS_DETECTOR_DEBUG

			pVehicleDetor->detframesUsed++;
#endif
			p = p->next;
			if (pVehicleDetor->useFixedTaskNum && p == NULL)
			{
				pVehicleDetor->currentWindowSize = 0;

#ifdef FCWS_DETECTOR_DEBUG

				if (pVehicleDetor->showFCWDebug == 1)
				 {
					//my_printf("totalTaskNum = %d \n", totalTaskNum);
				}
				//my_printf("Finish all Scales in %d Detected Frames! \n", pVehicleDetor->detframesUsed);

				pVehicleDetor->detframesUsed = 1;
#endif
			}
			break;
		}

		p = p->next;

		if (pVehicleDetor->useFixedTaskNum && p == NULL)
		{
			pVehicleDetor->currentWindowSize = 0;

#ifdef FCWS_DETECTOR_DEBUG

			if (pVehicleDetor->showFCWDebug == 1)
			{
				//my_printf("totalTaskNum = %d \n", totalTaskNum);
			}
			//my_printf("Finish all Scales in %d Detected Frames! \n", pVehicleDetor->detframesUsed);

			pVehicleDetor->detframesUsed = 1;
#endif
	/*		if (totalTaskNum < (int)(0.3 * pVehicleDetor->fixedTaskNum))
			{
				p = pDetectotDefaultROI[ROI_SEGMENT_NUM - 1].pDMLhead;
			}
	*/
		}
	}

	if (pVehicleDetor->pdetorModel->l->ndetdNum)
	{
		/* merge overlapped rectangles */
		detNum = lbpGroupWindowsCar(pVehicleDetor, pVehicleDetor->pdetorModel->l, &pVehicleDetor->objSets, &roiStartPoint, groupThreshold);
	}

#ifdef FCWS_DETECTOR_DEBUG
	my_printf("final object count: %d \n", detNum);
#endif

	return detNum;
}

int	FCW_DETCOR_Vehicle_Rio(FCWSDetectorGlobalPara *pVehicleDetor, 
						   FCWSDetectorROI		*pDetectotDefaultROI, 
						   const WissenImage			  *pGrayImg,
						   const WissenSize			  *minObjectSize,
						   const WissenSize			  *maxObjectSize,
						   const WissenRect			  *roi,
						   const int				  group_threshold, 
						   const int				  index)
{	
    int taskNum = 0;
	int roi_flg = 1;
	int totalTaskNum	= 0;
	int x, y, step;
	WissenPoint task;
	int ScaleNum = 0;

	WissenPoint	RioStartPoint = { 0 };
	WissenRect	detROI = { 0 };

	FCWSDetectorMultiscaleList	*p  = NULL;

	pVehicleDetor->objSets.nObjectNum = 0;

	p = pDetectotDefaultROI[ROI_SEGMENT_NUM - 1].pDMLhead;
	if (p != NULL)
	{
		RioStartPoint.x = p->roiRec.x;
		RioStartPoint.y = p->roiRec.y;
	}

	while (p != NULL)
	{
		p->roiRec.height = (pDetectotDefaultROI[ROI_SEGMENT_NUM - 1].posInfor - p->roiRec.y + 1) / p->factor;

		if( ((int)(p->roiRec.width * p->factor) < minObjectSize->width
			|| (int)(p->roiRec.height * p->factor) < minObjectSize->height ))
		{
			break;
		}

		if (roi != NULL)
		{
			if (roi->width < p->windowSize.width || roi->height < p->windowSize.height)
			{
				break;
			}

			if (p->windowSize.width > maxObjectSize->width || p->windowSize.height > maxObjectSize->height)
			{
				break;
			}
			if (p->windowSize.width < minObjectSize->width || p->windowSize.height < minObjectSize->height)
			{
				p = p->next;
				continue;
			}


			/* roi is for original image, according to 1920*1080 image size */
			detROI.width  = WS_MIN((int)(roi->width / p->factor), p->roiRec.width);;
			detROI.height = WS_MIN((int)(roi->height / p->factor), p->roiRec.height);
			detROI.x = (int)((roi->x - p->roiRec.x) / p->factor);/* according to p->roiRec.point.x */
			if (detROI.x < 0)
			{
				detROI.x = 0;
			}
			detROI.y = (int)((roi->y - p->roiRec.y) / p->factor);
			if (detROI.y < 0)
			{  
				detROI.y = 0;
			}
			if(detROI.y + detROI.height > p->roiRec.height)
			{
					detROI.height = p->roiRec.height - detROI.y;
			}
		}
		pVehicleDetor->pdetorModel->width		= p->roiRec.width;
		pVehicleDetor->pdetorModel->height		= p->roiRec.height;
		pVehicleDetor->pdetorModel->l->width	= p->roiRec.width;
		pVehicleDetor->pdetorModel->l->height	= p->roiRec.height;

		//t1 = p->pLBPTCLhead;
		taskNum = 0;

		if (roi != NULL)
		{
			//while (t1 != NULL)
			//{
			//	if (t1->task.x > detROI.point.x
			//	 && t1->task.y > detROI.point.y
			//	 && t1->task.x < detROI.point.x + detROI.size.width  - pVehicleDetor->pdetorModel->l->data.feature_width
			//	 && t1->task.y < detROI.point.y + detROI.size.height - pVehicleDetor->pdetorModel->l->data.feature_height)
			//	{
			//		/* 閲囩敤LDW鐨勮溅閬撶嚎淇℃伅锛孡DW鏃犺緭鍑猴紝濡傛灉鏈塺oi鍒欒缃畆oi鍖哄煙 */
			//		pVehicleDetor->pdetorModel->l->ptasks[taskNum++] = t1->task;
			//	}
			//	t1 = t1->next;
			//}
			//if( p->factor > 4 )
			//    step = 2;
			//else
			//	step = 1;
			if( index ==1)
			   step = (int) ((detROI.width  - pVehicleDetor->pdetorModel->l->data.featureWidth)>>3);
			else
			   step = (int) ((detROI.width  - pVehicleDetor->pdetorModel->l->data.featureWidth)>>3);

			if( step < 1)
				step = 1;

			//step = 2;

			for (x = detROI.x; x<detROI.x + detROI.width  - pVehicleDetor->pdetorModel->l->data.featureWidth; x += step)
			{
				for(y = detROI.y; y<detROI.y + detROI.height  - pVehicleDetor->pdetorModel->l->data.featureHeight; y += step)
				{
					task.x = x;
					task.y = y;
					pVehicleDetor->pdetorModel->l->ptasks[taskNum++] = task;
				}
			}
		}

		pVehicleDetor->pdetorModel->l->ntaskNum = taskNum;
		
#ifdef FCWS_DETECTOR_DEBUG

		//LOGI("p ROI: x %d, y %d, W %d, H %d \n",p->roiRec.point.x, p->roiRec.point.y, p->roiRec.size.width, p->roiRec.size.height);
		//LOGI("Det ROI: x %d, y %d, W %d, H %d \n",detROI.point.x, detROI.point.y, detROI.size.width, detROI.size.height);
		if (roi == NULL && pVehicleDetor->showFCWDebug == 1)
		{
			//my_printf("D: DetetWindowSize = %d, taskNum = %d \n",p->windowSize.height,taskNum);
		}
		else
		{
			//my_printf("T: DetetWindowSize = %d, taskNum = %d, MinSIze(%d, %d), MaxSIze(%d, %d)\n",p->windowSize.height,taskNum, minObjectSize->height, minObjectSize->width,maxObjectSize->height,maxObjectSize->width);

		}
#endif
		totalTaskNum += taskNum;
		
		if(taskNum!=0)
		{
			getResizeIntegralImage(pGrayImg, pVehicleDetor->pdetorModel->integralImg, &detROI, p);
		 
			lbpDetectWindowsCar(pVehicleDetor->pdetorModel->l, pVehicleDetor->pdetorModel->integralImg, p->factor, roi_flg, p->arrRowCol);
		}		
		ScaleNum++;
									
		//if( index == 0 )
		//{
		//	if(ScaleNum > 0)
		//		break;
		//}
		//else
		//{
		//	if(ScaleNum > 1)
		//		break;
		//}

		
		//if(pVehicleDetor->pdetorModel->l->ndetdNum > 5)
		//	break;


		p = p->next;
		
	}

	//printf("index=%d, group_threshold=%d, ScaleNum = %d , TotalTaskNum = %d, l->detNum = %d ", index, group_threshold,ScaleNum, totalTaskNum,pVehicleDetor->pdetorModel->l->ndetdNum);

	if (pVehicleDetor->pdetorModel->l->ndetdNum)
	{
		/* merge overlapped rectangles */
		object_detector_lbp_group_window_car_ROI(index,pVehicleDetor, pVehicleDetor->pdetorModel->l,&pVehicleDetor->objSets, &RioStartPoint, group_threshold);
	}

	//printf("objsets.nObjectNum = %d \n", pVehicleDetor->objSets.nObjectNum);
	//printf("Local Detection\n");
	
	return (pVehicleDetor->objSets.nObjectNum);


}

/*
Function process:
	+ Get the detector results.
	Fan-in :
	        + FCWSD_GetResult()
	Fan-out: N/A

	ATTENTION: 
*/
int FCWD_GetDetResult(const FCWSDetectorGlobalPara *pVehicleDetor, objectSetsCar **pFCWSDOutput)
{
	int i = 0;

	if (*pFCWSDOutput == NULL)
	{
		/* free in ???? */
		*pFCWSDOutput = (objectSetsCar *)malloc(sizeof(objectSetsCar));
	}
	else
	{
		if ((*pFCWSDOutput)->objects != NULL)
		{
			free((*pFCWSDOutput)->objects);
			(*pFCWSDOutput)->objects = NULL;
		}
	}

	(*pFCWSDOutput)->nObjectNum = pVehicleDetor->objSets.nObjectNum;

	/* free in the begin of this function */
	(*pFCWSDOutput)->objects = (WissenObjectRect *)malloc(sizeof(WissenObjectRect)* pVehicleDetor->objSets.nObjectNum);

	for (i = 0; i < pVehicleDetor->objSets.nObjectNum; ++i)
	{
		(*pFCWSDOutput)->objects[i] = pVehicleDetor->objSets.objects[i];
	}

	(*pFCWSDOutput)->frameNumber = 0;

	return pVehicleDetor->objSets.nObjectNum;
}

/*
Function process:
	+ Free the memory space of variables.
	Fan-in :
	        + FCWSD_Free()
	Fan-out: N/A

	ATTENTION: 
*/
int FCWD_UnitVehicle(FCWSDetectorGlobalPara *pVehicleDetor, FCWSDetectorROI *pDetectotDefaultROI)
{
	int j, ret;

	for( j = 0; j < ROI_SEGMENT_NUM; ++j )
	{
 		ret = freeTaskWindowsCar(pDetectotDefaultROI[j].pDMLhead);

		ret = freeVehicleDetProcess_Rio(&pDetectotDefaultROI[j]);

	}

	if (pVehicleDetor->pAlloc != NULL)
	{
		free(pVehicleDetor->pAlloc);
		pVehicleDetor->pAlloc = NULL;
	}

	return 1;
}
