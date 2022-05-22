/**************************************************************************************************************
Copyright © Wissen Intelligent Sensing Technology Co., Ltd. 2011-2017. All rights reserved.
File name: vehicle_type.h
Version: 1.0		Date: 2017-02-22		Author: Yanming Wang		ID: 1407930

Description:
	This header file defines the structures and macros used in vehicle detection.
	
Deviation: N/A

History:
	+ Version: 1.0		Date: 2017-02-22		Author: Yanming Wang	ID: 1407930
	  Modification: Coding Standards are added.
**************************************************************************************************************/

#ifndef VEHICLE_TYPE_H
#define VEHICLE_TYPE_H

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "FCWSD_Interface.h"

/* Used for debug */
//#define FCWS_DETECTOR_DEBUG

/* If defined, use the result of LDWS for vehicle detection */
#define USE_LDWS

#define USE_PROPOSALS

#define ACC_DETECTOR 1

/* If defined, use GPU to speed up vehicle detection */
//#define USE_GPU

/* If defined, use openmp to speed up vehicle detection */
#define OPENMP

/* If defined, use NEON to speed up vehicle detection */
//#define NEON

#ifdef FCWS_DETECTOR_DEBUG
	#ifdef WIN32
		#define WIN32_FCWS_DETECTOR_DEBUG
	#else
		#define ANDROID_FCWS_DETECTOR_DEBUG
	#endif
#endif

#ifdef WIN32
	#undef NEON
	#undef NE10
#endif

#ifdef USE_LDWS
	#include <LDWS_Interface.h>
#endif

#ifdef USE_GPU
#ifdef WIN32
	#define FCWS_GPU_KERNEL_BFILE_PATH ("fcw.bin")
	#define FCWS_GPU_KERNEL_CFILE_PATH ("fcw.txt")
#elif defined __arm__
	#define FCWS_GPU_KERNEL_BFILE_PATH ("/data/fcw.bin")
	#define FCWS_GPU_KERNEL_CFILE_PATH ("/data/fcw.txt")
#endif
#endif

#ifdef NE10
	#include "NE10.h"
#endif

#ifdef NEON
	#include <arm_neon.h>
#endif

#define MAX_IMAGE_WIDTH		(pVehicleDetor->srcWidth)
#define MAX_IMAGE_HIGHT		(pVehicleDetor->srcHeight)

#define MAX_TASKS_NUM		((MAX_IMAGE_WIDTH * MAX_IMAGE_HIGHT) >> 2)
//#define MAX_TASKS_NUM		(15000)
#define MAX_DETECT_NUM		(MAX_TASKS_NUM >> 3)
#define MAX_PUBLIC_BUFFER	((MAX_DETECT_NUM << 3) + 102400)
#define MAX_TREE_NODE		(MAX_DETECT_NUM)

/* The minimum size of width can be detected, decided by detector */
//#define MIN_DET_SIZE_WIDTH	(20)
/* The minimum size of height can be detected, decided by detector */
//#define MIN_DET_SIZE_HEIGHT	(20)

#define ALIGN_16BYTE(x)		((unsigned char*)(((unsigned long long)(x) + 15) >> 4 << 4))

/* Used in GPU speed up */
#define LOCKA_X_NUM			(16)
#define LOCKA_Y_NUM			(16)

/* Num of segments of detected region */
#define ROI_SEGMENT_NUM		(10)
#define ROI_SEGMENT_FACTOR	(0.1f) /* 1 / ROI_SEGMENT_NUM */

typedef struct WeakClassifierCar
{
    int*		lbpmap; /* Loading need to be singed */
	int*		rectIdx;
	float*	posiblity;
    int*	    leafIdx;
}weakClassifierCar;

typedef struct LbpParaCar 
{
	float	scalingFactor;
	float	eps;
	int			minSizeWidth;
}lbpParaCar;

typedef struct StageCar 
{
    weakClassifierCar   *classifiers;
    float			stageThreshold;
	int					weakClassifiersNum;
}stageCar;

typedef struct LbpDataCar
{
    stageCar		        *s;
	WissenObjectRect	    *r;
    int				        featureWidth;
    int				        featureHeight;
	int                     maxDepth;
	int                     leafNum;
    int				        stagesNum;
    int				        rectsNum;
	int                     totalWeakNum;
}lbpDataCar;

typedef struct lbpCar 
{
	lbpDataCar	     data;
    lbpParaCar	     para;
	WissenObjectRect *pdetectedRect; /* Must use new/delete instead of malloc/free because of this */
	WissenPoint	     *ptasks;		/* Task to scan all size and positions  */

    int				 width;
    int				 height;
	int				 ndetdNum;
	int				 ntaskNum;
}lbpCar;

typedef struct ObjectDetCar 
{
	lbpCar  *l;
    unsigned int		*integralImg;
    int		width;
    int		height;
}objectDetCar;

typedef struct DetectorGlobalPara
{
	objectSetsCar	objSets;

	unsigned char			*pAlloc;

#ifdef USE_GPU
	unsigned char			*pGpuBuff;
#endif
	unsigned char			*pbulicBuff;

	int				*pTreeNode;  /* for grouped windows */
	int				*pLabNode;   /* for grouped windows */

	objectDetCar	*pdetorModel;


	int				vanishY;            /* Vanishing line Y coordinate */

	int				srcWidth;
	int				srcHeight;
	int				fixedTaskNum;		/* Max num of tasks to be processed if useFixedTaskNum is set */
	int				useFixedTaskNum;
	int				currentWindowSize;	/* if useFixedTaskNum is set, detector works in this scale size  */
	float		srcROIYFactor;
	float		startMFactor;
	float		aspectRatio;

#ifdef FCWS_DETECTOR_DEBUG
	int		detframesUsed;				/* if useFixedTaskNum is set, how many frames can detect all the scales */
	int		showFCWDebug;				/* show debug result */
#endif

#ifdef USE_LDWS
	LDWS_Point		*roiPoint;
	LDWS_Point		*roiPointCopy;
	int				posInfor[ROI_SEGMENT_NUM];
#endif
	WissenRect       roi;
	unsigned char			initFlg;
	unsigned char         index;
}FCWSDetectorGlobalPara;

/* chained list for tasks */
typedef struct LbpTaskCarNode
{
	WissenPoint				task;
	struct LbpTaskCarNode	*next;
}LBPTaskCarList;

typedef struct NROWCOL
{
	int nrow[3];
	int ncol[3];
	int ncolAdd[3];
}nrowCol;

/* chained list for Multiscale tasks */
typedef struct DetectorMultiscaleNode
{
	WissenRect	roiRec;
	WissenSize	windowSize;
	float	factor;
	int			*arr_y; /* for integral image calculation */
	int			*arr_x;

	nrowCol     *arrRowCol;

//#ifdef FCWS_DETECTOR_DEBUG
	int           ntask;
//#endif

	LBPTaskCarList					*pLBPTCLhead;
	struct DetectorMultiscaleNode   *next;
}FCWSDetectorMultiscaleList;



typedef struct FCWSDetectorROI
{
	int	     posInfor;
	FCWSDetectorMultiscaleList *pDMLhead;
}FCWSDetectorROI;

#ifdef USE_GPU

#define MAX_DET_NUM			(10240)
#define WEAK_CLASSIFER_NUM	(512)
#define MAX_NUM_STAGES		(64)
#define MAX_NUM_RECS		(40960)

typedef struct reduce_stage
{
	int		num_weak_classifiers;
	float	stage_threshold;
}reduce_stage;

typedef struct reduce_lbp_data 
{
	int num_stages;
	int num_rects;
	int feature_width;
	int feature_height;
}reduce_lbp_data;

typedef struct reduce_lbp 
{
	int width;
	int height;

	reduce_lbp_data data;
	lbp_para_car	para;
}reduce_lbp;
#endif

#endif
