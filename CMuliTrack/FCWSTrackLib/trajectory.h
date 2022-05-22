/**************************************************************************************************************
Copyright © Wissen Intelligent Sensing Technology Co., Ltd. 2011-2017. All rights reserved.
File name: trajectory.h
Version: 1.0		Date: 2017-06-05		Author: Yanming Wang		ID: 1047930

Description:	
	The functions in this file are defined as the operate of trajects of object.
	The following function types are included:
	+ mvPredictTrack(): Predict the Trajecy point (pTrajecy->ptPredict) in current frame based on the history trajecy points (pTrajecy->pTrackPoint).
	+ mvClearTrajIfo(): Clear the trajecy of groups.
    + mvMatchTrackPoint(): Predict the Trajecy point (pTrajecy->ptPredict) in current frame based on the history trajecy points (pTrajecy->pTrackPoint).
	+ mvInitVote(): caculate the InitVote (offset with the center point) of traject point based on InitRect
	+ mvProcessVote(): caculate the InitVote (offset with the center point) of traject point based on InitRect
	+ mvTrajecyVote(): caculate the center vote point of pTrajecy
			
ATTENTION:

Deviation:

History:
	+ Version: 1.0		Date: 2017-06-05		Author: Yanming Wang	ID: 1047930
	  Modification: Coding Standards are added.
**************************************************************************************************************/

#ifndef TRAJECTORY_H
#define  TRAJECTORY_H

#include "track_type.h"

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in/out]	pTrajecy		      trajecy*		      trajecy struct.
[in]	    size		          const WissenSize	      Max predict region (image size)
[in]	    nFramSeq		      const long long		  frame num.

Realized function:
    + Predict the Trajecy point (pTrajecy->ptPredict) in current frame based on the history trajecy points (pTrajecy->pTrackPoint);
*/
void mvPredictTrack(trajecy *pTrajecy, const WissenSize size, const long long nFramSeq);

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in/out]	pTrajecy		      trajecy*		      trajecy struct.

Realized function:
    + Clear the trajecy of groups.
*/
void mvClearTrajIfo(trajecy *pTrajecy);

void mvCopyTrajecy(trajecy *pDstTrajecy,trajecy *pSrcTrajecy);

/*
I/O:	    Name		          Type	     		     Content
					    						  
[in]	    pTrajecy		      const trajecy*		 trajecy struct.
[in]	    pfeature		      const unsigned char*	     input fast points features
[in]	    pFastCorner		      AdasCorner*	         input fast points corner
[in]	    FastCornerNum		  const int	             input fast points Num
[in/out]	pCornerPos		      int*                   output matched fast point ID
[in/out]	pCornerTrackDis		  int*                   output matched fast point distance
[in]	    nId		              int	                 Scale factor

[out]	    returned		      int		             if find matched point return 1; else return 0.

Realized function:
    + Predict the Trajecy point (pTrajecy->ptPredict) in current frame based on the history trajecy points (pTrajecy->pTrackPoint);
*/
int mvMatchTrackPoint(const trajecy *pTrack, const unsigned char *pfeature, \
	                   AdasCorner *pFastCorner,const int FastCornerNum,\
	                  int *pCornerPos, int *pCornerTrackDis,int nId);

int mvMatchProcessPoint(const trajecy *pTrack, WissenObjectRectTracked ProcessContour, const unsigned char *pfeature, \
	AdasCorner *pFastCorner,const int FastCornerNum,\
	int *pCornerPos, int *pCornerTrackDis);

/*
I/O:	    Name		          Type	     		     Content
					    						  
[in/out]	pTrajecy		      trajecy*		         trajecy struct.
[in]	    initRec		          const AdasRect	     targect rect
[in]	    VotePointIndex		  int	                 index of vote point

Realized function:
    + caculate the InitVote (offset with the center point) of traject point based on InitRect
*/
void mvInitVote(trajecy *pTrajecy, const WissenObjectRectTracked initRec, int VotePointIndex);

/*
I/O:	    Name		          Type	     		     Content
					    						  
[in/out]	pTrajecy		      trajecy*		         trajecy struct.
[in]	    ProcessRec		      const AdasRect	     process targect rect

Realized function:
    + caculate the InitVote (offset with the center point) of traject point based on InitRect
*/
void mvProcessVote(trajecy *pTrajecy, const WissenObjectRectTracked ProcessRec);

/*
I/O:	    Name		          Type	     		     Content
					    						  
[in]	    pTrajecy		      trajecy*		         trajecy struct.
[in]	    fscale		          float	                 sacle ratio of pTrajecy
[in]	    VoteStyle		      VoteSort	             vote style

Realized function:
    + caculate the center vote point of pTrajecy
*/
Wissen16SPoint mvTrajecyVote(trajecy *pTrajecy, float fscale, VoteSort VoteStyle);

int  mvDisPowToIndexPoin(trajecy *pTrajecy,int poinIndex);

WissenPoint mvMovecToIndexPoin( trajecy *pTrajecy,int poinIndex );

void mvCorrectProcessVote( trajecy *pTrajecy, const Wissen16SPoint voteCorrectVal );

/*
I/O:	    Name		    Type	     		  Content

[in]        ptPredict	    const WissenPoint*	   input predicted point.
[in]	    object		const WissenObjectRect target size in last frame.
[in]	    nMatchLeng		const int		       length of track, pTrajec->nTrackLen.

[out]	    returned		WissenObjectRect	   predicted the possible region of target targect.

Realized function:
+ predict the possible region of target targect based on points and target size in last frame.
*/
WissenObjectRect preditObjectRegion(const WissenPoint *ptPredict, const WissenObjectRect object, const int nMatchLeng);


#endif
