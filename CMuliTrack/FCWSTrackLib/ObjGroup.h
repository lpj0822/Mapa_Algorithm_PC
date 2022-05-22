/**************************************************************************************************************
Copyright © Wissen Intelligent Sensing Technology Co., Ltd. 2011-2017. All rights reserved.
File name: ObjGroup.h
Version: 1.0		Date: 2017-06-05		Author: Yanming Wang		ID: 1047930

Description:	
	The functions in this file are defined as the operate for tracking groups.
	The following function types are included:
	+ mvClearGroupIfo(): clear Group information.
	+ mvMidGroupRec(): Do the midle filter for pGroup->rtContour based on pGroup->histoyRec.
    + mvScaleVal(): Caculate the scale factor of pGroup.
	+ mvDelUnReasonTrack(): judge if the tracked point is suitable;
	+ mvEstablInitVote(): Init vote for traject that inside pGroup->rtContour.
	+ mvclearInitState(): make pGroup->pObjtr->bInitTracked=0, if pGroup->pObjtr in Rec.
	+ mvUpdataGroupCenTraj(): update pGroup->Centr by pGroup->rtContour.
	+ mvclearProcesState(): make pGroup->pObjtr->bProcessTracked=0, if pGroup->pObjtr inside of Rec.
		
ATTENTION:

Deviation:

History:
	+ Version: 1.0		Date: 2017-06-05		Author: Yanming Wang	ID: 1047930
	  Modification: Coding Standards are added.
**************************************************************************************************************/

#ifndef OBJGROUP_H
#define OBJGROUP_H

#include "track_type.h"

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in/out]	pGroup		          obj_group*		  tracking Target group
[in]	    bCleaTrajecy		  int8_t		      if 1 clear traject;

Realized function:
    + clear Group information;
*/
void mvClearGroupIfo(obj_group*pGroup, unsigned char bCleaTrajecy);//bCleaTrajecy =1

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in/out]	pGroup		          obj_group*		  tracking Target group

Realized function:
    + Do the midle filter for pGroup->rtContour based on pGroup->histoyRec
*/
void  mvMidGroupRec(obj_group*pGroup);

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in]	    pGroup		          obj_group*		  tracking Target group
[in/out]    pSclarSpace           float*          scale ratio of different points
[in/out]    fscale                float*          middle point of pSclarSpace
[in]        votestyle             VoteSort            vote style
[in]        nId                   int                 tracking image sacle factor

[out]       returned              unsigned char             if can get suitable sacle factor return 1; else return 0

Realized function:
    + Caculate the scale factor of pGroup;
*/
unsigned char mvScaleVal(obj_group*pGroup, float *pSclarSpace, float *fscale, VoteSort votestyle, int nId);

void mvElimiNateInitTrack(obj_group*pGroup); 

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in/out]	pGroup		          obj_group*		  tracking Target group

Realized function:
    + judge if the tracked point is suitable;
*/
void mvDelUnReasonTrack(obj_group*pGroup); 

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in/out]	pGroup		          obj_group*		  tracking Target group
[in]        fShirkRat             float           if >0, do shirking for pGroup->InitContour
[in]        nId                   int                 sacle factor
[in]        nFramSeq              short             frame Num

Realized function:
    + Init vote for traject that inside pGroup->rtContour;
*/
void mvEstablInitVote(obj_group*pGroup, float fShirkRat, int nId, long long nFramSeq);//float fShirkRat = 0.15f

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in/out]	pGroup		          obj_group*		  tracking Target group
[in]        Rec                   AdasRect            rect reion

Realized function:
    + make pGroup->pObjtr->bInitTracked=0, if pGroup->pObjtr in Rec
*/
void mvclearInitState(obj_group*pGroup, WissenObjectRectTracked Rec);

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in/out]	pGroup		          obj_group*		  tracking Target group

Realized function:
    + update pGroup->Centr by pGroup->rtContour;
*/
void mvUpdataGroupCenTraj(obj_group*pGroup);

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in/out]	pGroup		          obj_group*		  tracking Target group
[in]        Rec                   AdasRect            rect reion

Realized function:
    + make pGroup->pObjtr->bProcessTracked=0, if pGroup->pObjtr inside of Rec
*/
void mvclearProcesState(obj_group*pGroup, WissenObjectRectTracked Rec);

void mvSetTrajLenthOne(obj_group*pGroup);

#endif
