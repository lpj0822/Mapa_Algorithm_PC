#ifndef TRACK_TYPE_H
#define TRACK_TYPE_H

#include "declare.h"
#include "CMulitTrack.h"
#include "kalman_filter.h"
#include "TTCkalmanfilter.h"

typedef enum VoteSort
{
	ProcessVote = 0,
	InitVote = 1,
	OrinitVote
}VoteSort;

typedef struct CAR_BOTTOM
{
	long long nDetBottomFam;
	int  nDetYPos;
	int  nDistoBottom;

	short nDetBottomNum;
}CarBot;

typedef struct TRAGECY
{
	WissenObjectRectTracked pRectPredict;

	TrackPoint *pTrackPoint;
	unsigned char *pfeature;
	unsigned char *processfeature;
	int nTrackId;
	int nMapGroupId;

	short nTrackLen;
	short nEstTimes;
	WissenPoint ptPredict;
	WissenPoint  InitVote;
	WissenPoint  OrInitVote;
	WissenPoint  ProcessVote;
	WissenPoint InitPoint;
	WissenPoint ProcessPoint;
	WissenPoint OrinitPoint;

	unsigned char bInitTracked;
	unsigned char bOrInitVoteTracked;
	unsigned char bProcessTracked;

#ifdef SHOW_RESULT
	Scalar col;
#endif

}trajecy;

typedef struct OBj_GROUP
{
	long long nFramseq;
	long long nPreProssVotFram;
	long long nPerLastDetFramseq;
	long long updataOriFramSeq;
	long long updataFrambyCar;
	WissenObjectRectTracked rtContour;
	WissenObjectRectTracked InitContour;
	WissenObjectRectTracked OriInitContour;
	WissenObjectRectTracked ProcessContour;
	WissenObjectRectTracked PreditRec;
	trajecy Centr;
	CarBot   CarBottom;

	int nGroupId;
	int  nLastupdataCarWidth;
	int  ntrajecyNum;
	int  nCurrentCarBottom;
	int  nMotionLeng;
	int nBottomWidth[GROUP_BOTTOM_NUM];
	int nBottomNum;
	int nUpdataIndex;
	objtype ntype;
	kalman_rect_state KalmState;
	TTCKalmanState sysKalmState;
	Motion  *pMotion;
	unsigned char *pOrInitGray;
	trajecy *pObjtr;
	histrec histoyRec;
	WissenImage UpdatedImg[UPDATA_IMAGE_NUM];
	WissenImage Templat;

	WissenSystime    nTime;

	unsigned char nDetState[GROUP_DETER_STATE_NUM];
	unsigned char nStateNum;
	unsigned char nTruelyObj;
	unsigned char SerpreditNum;
	short  nMinCarWid;

	long long nDetDiscardFam;
	int  nDisCardNum;

#ifdef SHOW_RESULT
	Scalar Groupcol;
	unsigned char binitvote;
#endif

}obj_group;

typedef struct ADASFCWINNERGLOBALPARA
{
	unsigned char *m_pAlloc;


	WissenPoint* m_pXYCorners;
	WissenPoint* m_pXYNoMax;
	int * m_pRowStart;
	int * m_pScore;
	AdasCorner *m_pCornerPass;
	int m_nCornerPassNum;
	int m_nCornerThresh;
	int m_nInitSuccess;
	AdasCorner *m_pFastCorner;
	int m_nFastCornerNum;
	unsigned char *m_pfeature;

	WissenObjectRectTracked *m_PNewRec;
	int m_NewRecNum;

	unsigned char *m_preGrayData;
	unsigned char *m_pGrayData;
	int  m_ImgWidth;
	int  m_ImgHeight;
	WissenSize m_ImgSize;

	unsigned char *m_pMask;

	obj_group *m_pGroupSets;

	int m_GroupId;
	PortInput *pOriInPutParam;
	int * m_ndx;
	int * m_ndy;
	float *m_fsclare; 
	int *m_PublacSape;
	int *m_extern_Space;
	unsigned char *m_pGroupIndex;
	unsigned char m_GroupIndexNum;
	PortInput scaleInput;

	int nscope_start_y;
	int nscope_end_y;
	int nscope_start_x;
	int nscope_end_x;

}adas_fcw_inner_global_param;


#endif //TRACK_TYPE_H
