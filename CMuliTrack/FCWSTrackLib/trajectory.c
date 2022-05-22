/**************************************************************************************************************
Copyright 漏 Wissen Intelligent Sensing Technology Co., Ltd. 2011-2017. All rights reserved.
File name: trajectory.cpp
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

#include "utils.h"
#include "utility_function.h"
#include "surf_feature.h"
#include "trajectory.h"
#include "declare.h"

#ifdef SHOW_CORNER_MATCH
#include "CMulitTrack.h"
extern adas_fcw_inner_global_param  m_globlparam[16];
#endif


/*
Function process:
	+ Clear the trajecy of groups
	Fan-in : 
	        + 
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
void mvClearTrajIfo( trajecy *pTrajecy )
{
	pTrajecy->nEstTimes = 0;
	pTrajecy->nMapGroupId = -1;
	pTrajecy->nTrackLen = 0;
	pTrajecy->nTrackId = -1;
	pTrajecy->bInitTracked = 0;
	pTrajecy->bProcessTracked = 0;
	pTrajecy->bOrInitVoteTracked = 0;

#ifdef SHOW_RESULT
	pTrajecy->col = Scalar(rand()%255,rand()%255,rand()%255);
#endif
	
}

/*
Function process:
	+ Predict the Trajecy point (pTrajecy->ptPredict) in current frame based on the history trajecy points (pTrajecy->pTrackPoint)
	Fan-in : 
	        + mvGoupPreditRec()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
void mvPredictTrack(trajecy *pTrajecy, const WissenSize size, const long long nFramSeq)
{
	int dx;
	int dy;
	int nTrackLen;
	long long nDiffram;

	nTrackLen = pTrajecy->nTrackLen;

	if (1 == nTrackLen)
	{
		pTrajecy->ptPredict.x = pTrajecy->pTrackPoint[0].point.x;
	    pTrajecy->ptPredict.y = pTrajecy->pTrackPoint[0].point.y;
	}
	else if (2 == nTrackLen)
	{
		dx = pTrajecy->pTrackPoint[1].point.x - pTrajecy->pTrackPoint[0].point.x;
		dy = pTrajecy->pTrackPoint[1].point.y - pTrajecy->pTrackPoint[0].point.y;
		pTrajecy->ptPredict.x = pTrajecy->pTrackPoint[1].point.x + dx;
		pTrajecy->ptPredict.y = pTrajecy->pTrackPoint[1].point.y + dy;
	}
	else
	{
		dx = (int)((pTrajecy->pTrackPoint[(nTrackLen - 1)&MAX_CORNER_OF_TRACK_BIT].point.x - pTrajecy->pTrackPoint[(nTrackLen - 3)&MAX_CORNER_OF_TRACK_BIT].point.x)) >> 1;
		dy = (int)((pTrajecy->pTrackPoint[(nTrackLen - 1)&MAX_CORNER_OF_TRACK_BIT].point.y - pTrajecy->pTrackPoint[(nTrackLen - 3)&MAX_CORNER_OF_TRACK_BIT].point.y)) >> 1;
		nDiffram = (pTrajecy->pTrackPoint[(nTrackLen - 1)&MAX_CORNER_OF_TRACK_BIT].nFramseq - pTrajecy->pTrackPoint[(nTrackLen - 3)& MAX_CORNER_OF_TRACK_BIT].nFramseq)>>1;
		
		if (1 == nDiffram)
		{
			if ( 1 == (nFramSeq - pTrajecy->pTrackPoint[(nTrackLen - 1)&MAX_CORNER_OF_TRACK_BIT].nFramseq) )
			{
				pTrajecy->ptPredict.x = pTrajecy->pTrackPoint[(nTrackLen - 1)&MAX_CORNER_OF_TRACK_BIT].point.x + dx;
				pTrajecy->ptPredict.y = pTrajecy->pTrackPoint[(nTrackLen - 1)&MAX_CORNER_OF_TRACK_BIT].point.y + dy;
			}
			else
			{
				pTrajecy->ptPredict.x = pTrajecy->pTrackPoint[(nTrackLen - 1)&MAX_CORNER_OF_TRACK_BIT].point.x
					+ dx * (int)(nFramSeq - pTrajecy->pTrackPoint[(nTrackLen - 1)&MAX_CORNER_OF_TRACK_BIT].nFramseq);
				pTrajecy->ptPredict.y = pTrajecy->pTrackPoint[(nTrackLen - 1)&MAX_CORNER_OF_TRACK_BIT].point.y \
					+ dy * (int)(nFramSeq - pTrajecy->pTrackPoint[(nTrackLen - 1)&MAX_CORNER_OF_TRACK_BIT].nFramseq);
			}		
		}
		else
		{
			if ( 1 == (nFramSeq - pTrajecy->pTrackPoint[(nTrackLen - 1)&MAX_CORNER_OF_TRACK_BIT].nFramseq) )
			{
				pTrajecy->ptPredict.x = pTrajecy->pTrackPoint[(nTrackLen - 1)&MAX_CORNER_OF_TRACK_BIT].point.x \
					+ (int)(dx/ (nDiffram + 1e-10));
				pTrajecy->ptPredict.y = pTrajecy->pTrackPoint[(nTrackLen - 1)&MAX_CORNER_OF_TRACK_BIT].point.y \
					+ (int)(dy/(nDiffram + 1e-10)) ;
			}
			else
			{
				pTrajecy->ptPredict.x = pTrajecy->pTrackPoint[(nTrackLen - 1)&MAX_CORNER_OF_TRACK_BIT].point.x \
					+ (int)(dx * (nFramSeq - pTrajecy->pTrackPoint[(nTrackLen - 1)&MAX_CORNER_OF_TRACK_BIT].nFramseq)/(nDiffram + 1e-10));
				pTrajecy->ptPredict.y = pTrajecy->pTrackPoint[(nTrackLen - 1)&MAX_CORNER_OF_TRACK_BIT].point.y \
					+ (int)(dy * (nFramSeq - pTrajecy->pTrackPoint[(nTrackLen - 1)&MAX_CORNER_OF_TRACK_BIT].nFramseq)/(nDiffram + 1e-10));
			}	
		}	
	}

	if (pTrajecy->ptPredict.x < 0
		|| pTrajecy->ptPredict.x >= size.width
		|| pTrajecy->ptPredict.y < 0
		|| pTrajecy->ptPredict.y >= size.height)
	{
		pTrajecy->ptPredict.x = pTrajecy->pTrackPoint[(nTrackLen - 1)&MAX_CORNER_OF_TRACK_BIT].point.x;
		pTrajecy->ptPredict.y = pTrajecy->pTrackPoint[(nTrackLen - 1)&MAX_CORNER_OF_TRACK_BIT].point.y;
		
	}
}


int mvMatchProcessPoint(const trajecy *pTrack, WissenObjectRectTracked ProcessContour, const unsigned char *pfeature, \
	AdasCorner *pFastCorner,const int FastCornerNum,\
	int *pCornerPos, int *pCornerTrackDis)
{
	int i;
	int dist;
	int dist1;
	int dist2;
	int nDistThresh;
	int nMatchPos1;
	int nMatchPos2;
	WissenObjectRectTracked procesRelatRec;
	//unsigned char nNearNum = 0 ;
	AdasCorner *pCorner = 0;
	const int nExtend = WS_MAX(3,ProcessContour.object.width/7);

	dist1 = 1000000;
	dist2 = 1000000;
	nDistThresh = 24000;
	nMatchPos1 = -1;
	nMatchPos2 = -1;

	procesRelatRec.object.x = (pTrack->ProcessPoint.x - ProcessContour.object.x - nExtend);
	procesRelatRec.object.y = (pTrack->ProcessPoint.y - ProcessContour.object.y - nExtend);
	procesRelatRec.object.width = nExtend<<1;
	procesRelatRec.object.height = nExtend<<1;

	for (i = 0 ; i < FastCornerNum;i++)
	{

		pCorner = pFastCorner + i;

		if (!isPointInRect(&pCorner->point, &procesRelatRec.object))
		{
			continue;
		}

		//dist = mvSurfDist(pTrack->processfeature, pfeature + SURF_DESC_DIMENTION * i);
		dist = mvSurfDist(pTrack->processfeature, pfeature +   (i<<6) );

		if (dist < dist1)
		{
			dist2 = dist1;
			dist1 = dist;
			nMatchPos2 = nMatchPos1;
			nMatchPos1 = i;
		}
		else if (dist < dist2)
		{
			dist2 = dist;
			nMatchPos2 = i;
		}
	}

	if (-1 == nMatchPos2 && -1 == nMatchPos1)
	{
		
		return 0;
	}
	else if (-1 == nMatchPos2)
	{
		if (dist1 < nDistThresh)
		{
			*pCornerPos = nMatchPos1;
			*pCornerTrackDis = dist1;
			return 1;
		}
		else
		{
			
			return 0;
		}
	}
	else
	{
		if (dist1 < nDistThresh)
		{
			if (dist1 < dist2 * 0.7f )  //0.6f 0.7f 0.81
			{
				*pCornerPos = nMatchPos1;
				*pCornerTrackDis = dist1;
				return 1;
			}
			else
			{
				

				return 0;
			}
		}
		else
		{
			
			return 0;
		}
	}

}

/*
Function process:
	+ Match the trajecy in the fast points, return the matched point id and matched distance; If matched point found return 1, else return 0
	Fan-in : 
	        + mvTrajectorymatch()
	Fan-out:
	        + mvSurfDist()
	ATTENTION: __________
*/
int mvMatchTrackPoint( const trajecy *pTrack, const unsigned char *pfeature, AdasCorner *pFastCorner, const int FastCornerNum,\
	                   int *pCornerPos, int *pCornerTrackDis, int nId)
{

	int i,j;
	int dist;
	int dist1;
	int dist2;
	int nDistThresh;
	int nMatchPos1;
	int nMatchPos2;
	
	int nNearTrackPredit[32];//const int nMaxNear = 32;
	unsigned char nNearNum = 0 ;
	AdasCorner *pCorner = 0;
	const int nNearTrd = 5;
	dist1 = 10000000;
	dist2 = 10000000;
	nDistThresh = 24001;//24001
	
	nMatchPos1 = -1;
	nMatchPos2 = -1;


#ifdef SHOW_CORNER_MATCH
   
	AdasRect Rio;
	IplImage * CombineMatchImg  = 0;
	CvScalar col;

	if (nId !=0)
	{
	
	Rio = adasrect(0,0,m_globlparam[nId].m_ImgWidth, m_globlparam[nId].m_ImgHeight);

	IplImage * TrajecyImg = GetImg(m_globlparam[nId].m_preGrayData, m_globlparam[nId].m_ImgWidth,\
		 m_globlparam[nId].m_ImgHeight>>nId, Rio );

	IplImage * Trajecycolimg = cvCreateImage(cvSize(m_globlparam[nId].m_ImgWidth,m_globlparam[nId].m_ImgHeight),8,3);
 	cvMerge(TrajecyImg, TrajecyImg, TrajecyImg, NULL, Trajecycolimg);
	cvReleaseImage(&TrajecyImg);

	col=  CV_RGB(0,0,255);;
	if(pTrack->bInitTracked)
	{
		col = CV_RGB(255,0,0);
	}
	else if(pTrack->bProcessTracked)
	{
		col = CV_RGB(0,255,0);
	}

	CvPoint point;
	x = pTrack->pTrackPoint[(pTrack->nTrackLen-1)&MAX_CORNER_OF_TRACK_BIT].x;
	y = pTrack->pTrackPoint[(pTrack->nTrackLen-1)&MAX_CORNER_OF_TRACK_BIT].y;
	cvCircle(Trajecycolimg,point,2,col);

	IplImage * Matchcolimg = cvCreateImage(cvSize(m_globlparam[nId].m_ImgWidth,m_globlparam[nId].m_ImgHeight),8,3);
	cvMerge(MatchImg,MatchImg,MatchImg,NULL,Matchcolimg);
	cvReleaseImage(&MatchImg);

	cvSetImageROI(CombineMatchImg,cvRect(0,0,m_globlparam[nId].m_ImgWidth,m_globlparam[nId].m_ImgHeight));
	cvCopy(Trajecycolimg,CombineMatchImg);
	cvResetImageROI(CombineMatchImg);
	cvReleaseImage(&Trajecycolimg);

	cvSetImageROI(CombineMatchImg,cvRect(m_globlparam[nId].m_ImgWidth,0,m_globlparam[nId].m_ImgWidth,\
		m_globlparam[nId].m_ImgHeight));
	cvCopy(Matchcolimg,CombineMatchImg);
	cvResetImageROI(CombineMatchImg);
	cvReleaseImage(&Matchcolimg);

	for (i = 0 ; i < FastCornerNum;i++)
	{

		pCorner = pFastCorner + i;

		if( !isPointInRect(&pCorner->point,&pTrack->pRectPredict.object) )
		{
			continue;
		}

		//dist = mvSurfDist(pTrack->pfeature, pfeature + SURF_DESC_DIMENTION * i);

#ifdef SHOW_CORNER_MATCH
		cvCircle(CombineMatchImg, cvPoint(pCorner->x + m_globlparam[nId].m_ImgWidth, pCorner->y), 2 ,CV_RGB(255,255,255));
#endif
	}

	cvShowImage("CombineMatchImg",CombineMatchImg);


	for (i = 0 ; i < FastCornerNum;i++)
	{
		
		pCorner = pFastCorner + i;

		if( !isPointInRect(&pCorner->point, &pTrack->pRectPredict.object) )
		{
			continue;
		}
		IplImage * Img = cvCloneImage(CombineMatchImg);
		dist = mvSurfDist(pTrack->pfeature, pfeature + SURF_DESC_DIMENTION * i);
		
		char buffer[64];
		sprintf(buffer,"%d",dist);
		CvFont font;  
			
        cvInitFont(&font,CV_FONT_HERSHEY_COMPLEX, 0.5, 0.5, 1, 2, 8); 

        cvPutText(Img,buffer, cvPoint(50,50), &font, CV_RGB(255,0,0));      

#ifdef SHOW_CORNER_MATCH
   if (nId !=0)
   {
		cvCircle(Img,cvPoint(pCorner->x + (m_globlparam[nId].m_ImgWidth) ,pCorner->y),2,CV_RGB(255,255,0));
   }
#endif
		cvShowImage("every_corner_dis",Img);
		cvWaitKey(0);
		cvReleaseImage(&Img);
	}
		
	 cvWaitKey(0);
	 }
#endif
	
	for (i = 0 ; i < FastCornerNum;i++)
	{

		pCorner = pFastCorner + i;

		if (!isPointInRect(&pCorner->point, &pTrack->pRectPredict.object))
		{
			continue;
		}

#ifdef SHOW_CORNER_MATCH
		if (nId !=0)
		{
		 cvCircle(CombineMatchImg,cvPoint(pCorner->x + (m_globlparam[nId].m_ImgWidth),pCorner->y),2,CV_RGB(255,255,255));
		}
#endif

		if (WS_ABS(pCorner->point.x - pTrack->ptPredict.x) + WS_ABS(pCorner->point.y - pTrack->ptPredict.y) < nNearTrd)
		{
			nNearTrackPredit[nNearNum++] = i;
		}

		//dist = mvSurfDist(pTrack->pfeature, pfeature + SURF_DESC_DIMENTION * i);
		dist = mvSurfDist(pTrack->pfeature, pfeature +  (i<<6));

		if (dist < dist1)
		{
			dist2 = dist1;
			dist1 = dist;
			nMatchPos2 = nMatchPos1;
			nMatchPos1 = i;
		}
		else if (dist < dist2)
		{
			dist2 = dist;
			nMatchPos2 = i;
		}
	}

#ifdef SHOW_CORNER_MATCH
	if (nId !=0)
	{
		my_printf("MAX_Index:%d,SECD_maxIndex:%d,Dist1:%d,Dist2:%d,\n",nMatchPos1,nMatchPos2,dist1,dist2);
		my_printf("[%d,%d],\n",pFastCorner[nMatchPos1].x - pFastCorner[nMatchPos2].x ,pFastCorner[nMatchPos1].y - pFastCorner[nMatchPos2].y);
		my_printf("鏈�浉浼兼瘮渚�%0.3f\n",dist1*1.0f/dist2);}
#endif

	if (-1 == nMatchPos2 && -1 == nMatchPos1)
	{
		for (j = 0 ; j < nNearNum;  j++)
		{
			pCorner = pFastCorner + nNearTrackPredit[j];
			if (!pCorner->State.nMacthNum )
			{
				pCorner->State.nMacthNum = 2;
			}
			
		}
#ifdef SHOW_CORNER_MATCH
		if (nId !=0)
		{
	       cvShowImage("NO_match",CombineMatchImg);
	       cvWaitKey(0);
	       cvReleaseImage(&CombineMatchImg);
		}
#endif
		return 0;
	}
	else if (-1 == nMatchPos2)
	{
		if (dist1 < nDistThresh)
		{
			*pCornerPos = nMatchPos1;
			*pCornerTrackDis = dist1;

#ifdef SHOW_CORNER_MATCH	
			if (nId !=0)
			{
	  cvCircle(CombineMatchImg,cvPoint(pFastCorner[nMatchPos1].x + (m_globlparam[nId].m_ImgWidth),pFastCorner[nMatchPos1].y),\
		  2,col);
	  cvShowImage("match_1",CombineMatchImg);
	   cvWaitKey(0);
	   cvReleaseImage(&CombineMatchImg);}
#endif
			return 1;
		}
		else
		{
			for (j = 0 ; j < nNearNum;  j++)
			{
				pCorner = pFastCorner + nNearTrackPredit[j];
				if (!pCorner->State.nMacthNum )
				{
					pCorner->State.nMacthNum = 2;
				}
			}
			return 0;
		}
	}
	else
	{
		if (dist1 < nDistThresh)
		{
#ifdef SHOW_CORNER_MATCH		
	  my_printf("dist1:%d,dist2:%d,rate:%0.3f\n",dist1,dist2,dist1*1.0f/dist2 );
#endif
			if (dist1 < dist2 * 0.8f )  //0.6f 0.7f 0.81
			{
				*pCornerPos = nMatchPos1;
				*pCornerTrackDis = dist1;

#ifdef SHOW_CORNER_MATCH	
				if (nId !=0)
				{
	  cvCircle(CombineMatchImg,cvPoint(pFastCorner[nMatchPos1].x + (m_globlparam[nId].m_ImgWidth),pFastCorner[nMatchPos1].y),\
 		  2,col);
	  cvShowImage("match_12",CombineMatchImg);
	   cvWaitKey(0);
	   cvReleaseImage(&CombineMatchImg);}
#endif
				return 1;
			}
			else
			{
				for (j = 0 ; j < nNearNum;  j++)
				{
					pCorner = pFastCorner + nNearTrackPredit[j];
					if (!pCorner->State.nMacthNum )
					{
						pCorner->State.nMacthNum = 2;
					}
				}
#ifdef SHOW_CORNER_MATCH	
				if (nId !=0)
				{
     cvCircle(CombineMatchImg,cvPoint(pFastCorner[nMatchPos1].x + (m_globlparam[nId].m_ImgWidth),pFastCorner[nMatchPos1].y),\
		  2,CV_RGB(255,255,0));
	  cvShowImage("match_Rate_failedbyrate",CombineMatchImg);
	  cvWaitKey(0);
	  cvReleaseImage(&CombineMatchImg);}
#endif
				return 0;
			}
		}
		else
		{
			for (j = 0 ; j < nNearNum;  j++)
			{
				pCorner = pFastCorner + nNearTrackPredit[j];
				if (!pCorner->State.nMacthNum )
				{
					pCorner->State.nMacthNum = 2;
				}
			}
			return 0;
		}
	}

}

void mvCopyTrajecy( trajecy *pDstTrajecy,trajecy *pSrcTrajecy )
{
	memcpy(pDstTrajecy->pTrackPoint,pSrcTrajecy->pTrackPoint,sizeof(TrackPoint) * WS_MIN(MAX_CORNER_OF_TRACK ,pSrcTrajecy->nTrackLen));
	pDstTrajecy->nTrackLen = pSrcTrajecy->nTrackLen;

	memcpy(pDstTrajecy->pfeature,pSrcTrajecy->pfeature,SURF_DESC_DIMENTION);
	memcpy(pDstTrajecy->processfeature,pSrcTrajecy->processfeature,SURF_DESC_DIMENTION);
	
	pDstTrajecy->nTrackId = pSrcTrajecy->nTrackId;
	pDstTrajecy->nEstTimes= pSrcTrajecy->nEstTimes;
	pDstTrajecy->ptPredict = pSrcTrajecy->ptPredict;
	pDstTrajecy->pRectPredict = pSrcTrajecy->pRectPredict;
	pDstTrajecy->nMapGroupId = pSrcTrajecy->nMapGroupId;

	pDstTrajecy->bInitTracked = pSrcTrajecy->bInitTracked;
	pDstTrajecy->InitVote = pSrcTrajecy->InitVote;
	pDstTrajecy->InitPoint  = pSrcTrajecy->InitPoint;

	pDstTrajecy->bProcessTracked = pSrcTrajecy->bProcessTracked;
	pDstTrajecy->ProcessVote =  pSrcTrajecy->ProcessVote;
	pDstTrajecy->ProcessPoint  = pSrcTrajecy->ProcessPoint;

	pDstTrajecy->OrInitVote = pSrcTrajecy->OrInitVote;
	pDstTrajecy->bOrInitVoteTracked = pSrcTrajecy->bOrInitVoteTracked;
	pDstTrajecy->OrinitPoint = pSrcTrajecy->OrinitPoint;


	
	
	
}

/*
Function process:
	+ caculate the InitVote (offset with the center point) of traject point based on InitRect
	Fan-in : 
	        + mvEstablInitVote()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
void mvInitVote(trajecy *pTrajecy, const WissenObjectRectTracked initRec, int VotePointIndex)
{
	TrackPoint *pVotePoin = pTrajecy->pTrackPoint + VotePointIndex;

	pTrajecy->InitVote.x = pVotePoin->point.x - (initRec.object.x + (initRec.object.width >> 1));
	pTrajecy->InitVote.y = pVotePoin->point.y - (initRec.object.y + (initRec.object.height >> 1));
	pTrajecy->InitPoint =   pVotePoin->point;

}

/*
Function process:
	+ caculate the ProcessVote (offset with the center point) of traject point based on ProcessRec
	Fan-in : 
	        + mvPreditGroup()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
void mvProcessVote(trajecy *pTrajecy, const WissenObjectRectTracked ProcessRec)
{
	TrackPoint *pVotePoin = pTrajecy->pTrackPoint + ( (pTrajecy->nTrackLen - 1)&MAX_CORNER_OF_TRACK_BIT ) ;
	
	pTrajecy->ProcessVote.x = pVotePoin->point.x - (ProcessRec.object.x + (ProcessRec.object.width >> 1));
	pTrajecy->ProcessVote.y = pVotePoin->point.y - (ProcessRec.object.y + (ProcessRec.object.height >> 1));
	pTrajecy->ProcessPoint =  pVotePoin->point;
}

void mvCorrectProcessVote(trajecy *pTrajecy, const Wissen16SPoint voteCorrectVal)
{
	pTrajecy->ProcessVote.x += voteCorrectVal.x;
	pTrajecy->ProcessVote.y += voteCorrectVal.y;

}

/*
Function process:
	+ caculate the center vote point of pTrajecy
	Fan-in : 
	        + mvInitConsensVote()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
Wissen16SPoint mvTrajecyVote(trajecy *pTrajecy, float fscale, VoteSort VoteStyle)
{
	Wissen16SPoint CenVote;
	TrackPoint *point = 0;

	if (  InitVote == VoteStyle)
	{
 		point = pTrajecy->pTrackPoint + ( (pTrajecy->nTrackLen - 1) & MAX_CORNER_OF_TRACK_BIT );
		CenVote.x = point->point.x - (short)(fscale * pTrajecy->InitVote.x);
		CenVote.y = point->point.y - (short)(fscale * pTrajecy->InitVote.y);
	}
	else if ( ProcessVote == VoteStyle)
	{
	   point = pTrajecy->pTrackPoint + ( (pTrajecy->nTrackLen - 1) & MAX_CORNER_OF_TRACK_BIT );
	   CenVote.x = point->point.x - (int)(fscale * pTrajecy->ProcessVote.x);
	   CenVote.y = point->point.y - (int)(fscale * pTrajecy->ProcessVote.y);

	}else if (OrinitVote == VoteStyle)
	{
		point = pTrajecy->pTrackPoint + ( (pTrajecy->nTrackLen - 1) & MAX_CORNER_OF_TRACK_BIT );
		CenVote.x = point->point.x - (int)(fscale * pTrajecy->OrInitVote.x);
		CenVote.y = point->point.y - (int)(fscale * pTrajecy->OrInitVote.y);
	}
	

	return CenVote;
}

int mvDisPowToIndexPoin( trajecy *pTrajecy,int poinIndex )
{
	WissenPoint *pLastpoint = 0;
	WissenPoint *pIndexPoint = 0;
	TrackPoint *TrajecPoin = 0;
	int nDis;

	TrajecPoin  = pTrajecy->pTrackPoint + ( (pTrajecy->nTrackLen -1)&MAX_CORNER_OF_TRACK_BIT );
    pLastpoint = &(TrajecPoin->point);

	TrajecPoin  = (pTrajecy->pTrackPoint + poinIndex);
	pIndexPoint = &(TrajecPoin->point);

	nDis = mvDisPow(pLastpoint,pIndexPoint);

	return nDis;
}

WissenPoint mvMovecToIndexPoin(trajecy *pTrajecy, int poinIndex)
{
	WissenPoint *pLastpoint = 0;
	WissenPoint *pIndexPoint = 0;
	TrackPoint *TrajecPoin = 0;
	WissenPoint MoVec;
	

	TrajecPoin  = pTrajecy->pTrackPoint + ((pTrajecy->nTrackLen -1)&MAX_CORNER_OF_TRACK_BIT);
	pLastpoint = &(TrajecPoin->point);

	TrajecPoin  = (pTrajecy->pTrackPoint + poinIndex);
	pIndexPoint = &(TrajecPoin->point);

	MoVec.x = pIndexPoint->x -  pLastpoint->x;
	MoVec.y = pIndexPoint->y -  pLastpoint->y;
	return MoVec;
}

/*
I/O:	    Name		    Type	     		  Content

[in]        ptPredict	    const WissenPoint*	   input predicted point.
[in]	    object		const WissenObjectRect target size in last frame.
[in]	    nMatchLeng		const int		       length of track, pTrajec->nTrackLen.

[out]	    returned		WissenObjectRect	   predicted the possible region of target targect.

Realized function:
+ predict the possible region of target targect based on points and target size in last frame.
*/
WissenObjectRect preditObjectRegion(const WissenPoint *ptPredict, const WissenObjectRect object, const int nMatchLeng)
{
	int nExtandX;
	int nExtandY;
	WissenObjectRect result;
	const int nMinExTend = 8;
	int nMaxExTend = 15;

	if (nMatchLeng < 3) {
		nMaxExTend = 20;
	}

	nExtandX = object.width >> 2;
	nExtandY = object.height >> 2;

	if (nExtandX < nMinExTend) 
	{
		nExtandX = nMinExTend;
	}

	nExtandX = WS_MIN(nExtandX, nMaxExTend);

	if (nExtandY < nMinExTend) 
	{
		nExtandY = nMinExTend;
	}

	nExtandY = WS_MIN(nExtandY, nMaxExTend);

	result.x = ptPredict->x - (nExtandX >> 1);
	result.width = nExtandX;

	result.y = ptPredict->y - (nExtandY >> 1);
	result.height = nExtandY;

	return result;

}
