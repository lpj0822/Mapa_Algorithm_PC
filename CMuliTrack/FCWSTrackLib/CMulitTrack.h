/**************************************************************************************************************
Copyright © Wissen Intelligent Sensing Technology Co., Ltd. 2011-2017. All rights reserved.
File name: CMulitTrack.h
Version: 1.0		Date: 2017-06-05		Author: Yanming Wang		ID: 1047930
Version: 2.0		Date: 2018-05-08		Author: Li Wan		        ID: 1047931

Description:	
	The functions in this file are defined as the realized of Multi-target tracking.
	The following function types are included:
	+ FCW_TRACK_Init_adas_global_param(): malloc the memory for Tracking.
	+ FCW_TRACK_MultieTrack(): The main realize of multi-tracking.
    + FCW_TRACK_GetResult():  Get the multi-tracking result.
	+ FCW_TRACK_mvClearWholeGroup(): Clear m_globlparam[nId].m_pGroupSets
	+ FCW_TRACK_Unit_adas_global_param(): Free memory.
	+ FCW_TRACK_SimpleTrack(): Do the sample tracking based on temple tracking. if tracked, return 1; else return 0;
	
ATTENTION:

Deviation:

History:
	+ Version: 1.0		Date: 2017-06-05		Author: Yanming Wang	ID: 1047930
	  Modification: Coding Standards are added.
	+ Version: 2.0		Date: 2018-05-08		Author: Li Wan		    ID: 1047931
**************************************************************************************************************/

#ifndef CMULITTRACK_H
#define CMULITTRACK_H

#include "common.h"
#include "OBJVERF_Interface.h"

#define DETCOR_STAR
#define TRACK_DEBUG

#define scale_shink_1_id_W  40
#define scale_shink_2_id_W  120
#define inner_global_param_num 16

#define scale_shink_1_id 0  
#define scale_shink_2_id 1 
#define scale_shink_4_id 2 


#ifdef TRACK_DEBUG
	#ifdef WIN32

		#define _WIN32_TRACK_DEBUG_
	#else

		#define _ANDROID_TRACK_DEBUG_
	#endif
#endif

typedef struct CAN_INFO
{
	float  fSpeed;
	float  fPretendingAngle;
}CANInfo;

typedef struct PORT_INPUT
{
	long long nFramSeq;
	float  fzoom;
	unsigned char *pGrayfram;
	WissenObjectRectTracked *objRec;
	int  nRecNum;
	WissenSize imgSize;
	WissenSystime objTime;
	//imgage pOriGrayfram2;
	WissenImage pOriGrayfram;
	unsigned char groundValue;
	unsigned char dayOrNight;
}PortInput;

typedef struct HISTREC
{
	int  *pGroupSiz;
	int nSizNum;
}histrec;

typedef struct TRACK_POINT
{
	long long nFramseq;
	short nMatchStatus;
	WissenSystime nTime;
	WissenPoint point;
} TrackPoint;


typedef struct MOTION
{

	WissenObjectRectTracked groupRec;

	float   x_fdis;
	float  x_fSpeed;
	float  x_fAccelSpeed;

	float   z_fdis;
	float  z_fAccelSpeed;

	float  delVanish;
	float  fTTC;
	float  dTTC;


	WissenSystime ts;
	unsigned char  bInCollishionPath;

} Motion;



typedef struct TRACK_OBJ
{
	long long nFramSeq;
	long long nId;
	WissenObjectRectTracked objRec;
	int  DisToBottom;
	TrackPoint *pCenTrajecy;
	Motion  *pMotion;
	int  nMotionLeng;
	float  fTTC;
	float  dTTC;
	int  nTrackLen;
	unsigned char bTrue;
}trakobj;


typedef struct MULI_TRACKER
{
	trakobj *pTrackerset;
	int  nTrackeNum;
}MuliTracker;

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in]	    ImgSize		          WissenSize		     Size of input image.

Realized function:
    + malloc the memory for Tracking
	+ ATTENTION: ImgSize is the size of resized img (1/2 of orig image)
*/
void FCW_TRACK_Init_adas_global_param(WissenSize ImgSize);

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in]	    pInPutParam		      PortInput*		  input stract of multi-tracking.

Realized function:
    + The main realize of multi-tracking
*/
void FCW_TRACK_MultieTrack(PortInput *pInPutParam, CANInfo *CANData);

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in]	    pTrackOutput		  MuliTracker**		  output stract of multi-tracking.

Realized function:
    + Get the multi-tracking result
*/
void FCW_TRACK_GetResult(MuliTracker **pTrackOutput);

/*
Realized function:
    + Clear m_globlparam[nId].m_pGroupSets
*/
void FCW_TRACK_mvClearWholeGroup(void);

/*
Realized function:
    + Free memory
*/
void FCW_TRACK_Unit_adas_global_param(void);

/*
I/O:	    Name		        Type	     	  Content
					    						  
[in]	    SrcRec		        const AdasRect	  Target rect in last frame.
[in]	    pSrcImg		        WissenImage*		      image date in last frame.
[in]	    pDstImg		        const WissenImage*	  image date in this frame.
[in]	    MotionVec		    Wissen16SPoint	      offset of SrcRec in this frame.
[in/out]	    pDstRec		        AdasRect*    	  Target rect in this frame.

Realized function:
    + Do the sample tracking based on temple tracking. if tracked, return 1; else return 0;
*/
int FCW_TRACK_SimpleTrack(const WissenObjectRectTracked SrcRec, WissenImage *pSrcImg, const WissenImage *pDstImg, Wissen16SPoint MotionVec, \
	WissenObjectRectTracked *pDstRec);


void  FCW_TRACK_DetcorBytrain(VerifyObjectList verifyObjectOutput);

#endif
