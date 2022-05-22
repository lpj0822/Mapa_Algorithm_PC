#include "CMulitTrack.h"

#include "tracking_utility.h"
#include "image_process.h"
#include "trajectory.h"
#include "ObjGroup.h"
#include "vehicle_taillight.h"

//#define RET_DS   //switch for if reserve discard target for 3 frame
#define NEW_DET_PORT
//#define OBJVERF //switch for verify target
//#define OBJRLCT  //switch for relocation
//#ifdef TIME_TEST  //switch for cost time print

#include "utils.h"
#include "LDWS_Interface.h"
#include "sort_algorithm.h"
#include "utility_function.h"
#include "corner_detection.h"
#include "surf_feature.h"
#include "OBJVERF_Interface.h"

#ifdef  DETCOR_STAR

#ifdef  NEW_DET_PORT
#include <FCWSD_Interface.h>
static objectSetsCar *g_pDetobjsets;
#endif

#endif

static adas_fcw_inner_global_param m_globlparam[16];
static MuliTracker g_MuliTracker[2];
static int g_OutIndex = -1;
static LDWS_Point g_pLDWSVPoint;
static LDWS_InitGuid *pLDWSInit = NULL;
static DarkTailLightGlobalPara gl_tailnightpara = { 0 };

/*
I/O:	    Name		    Type	     		  Content

[in/out]	pGroup		    obj_group*	          target group
[in]	    nId		        int		              scale factor.

Realized function:
+  update pgroup->pOrInitGray based on pgroup->InitContour
*/
static void mvSetOriInitGray(obj_group *pgroup, int nId)
{
	int j;
	unsigned char *pDst = 0;
	unsigned char *pSrc = 0;
	int nRow = 0;
	WissenObjectRect tempRect = { 0, 0, m_globlparam[nId].m_ImgWidth, m_globlparam[nId].m_ImgHeight };

	limitObjectRectRange(tempRect, &pgroup->InitContour.object);

	if (Car
		== pgroup->ntype&& WS_MAX(pgroup->InitContour.object.width, pgroup->InitContour.object.height) >= MAX_TEMPLAT_TRACK_SIZE) {
		pgroup->InitContour.object.width = 0;
		pgroup->InitContour.object.height = 0;
		return;
	}

	for (j = pgroup->InitContour.object.y;
		j < pgroup->InitContour.object.y + pgroup->InitContour.object.height; j++) {
		pDst = pgroup->pOrInitGray + pgroup->InitContour.object.width * nRow;
		pSrc = m_globlparam[nId].m_pGrayData + m_globlparam[nId].m_ImgWidth * j
			+ pgroup->InitContour.object.x;
		memcpy(pDst, pSrc, pgroup->InitContour.object.width);
		nRow++;
	}

}

/*
I/O:	    Name		    Type	     		  Content

[in/out]	pGroup		    obj_group*	          target group
[in]	    nId		        int		              scale factor.

Realized function:
+  add traject to pgroup->pObjtr by the matched fast points
*/
static void mvAddTrajecToGroup(obj_group *pgroup, int nId)
{

	int m;
	int nInRecNum = 0;
	AdasCorner *pCorner = 0;
	trajecy *pTrajecy = 0;
	WissenObjectRectTracked shrinkRec;


	shrinkRec.object.x = pgroup->ProcessContour.object.x
		+ (int)(pgroup->ProcessContour.object.width * 0.15f);
	shrinkRec.object.y = pgroup->ProcessContour.object.y
		+ (int)(pgroup->ProcessContour.object.height * 0.15f);
	shrinkRec.object.width = (int)(pgroup->ProcessContour.object.width * 0.7f);
	shrinkRec.object.height = (int)(pgroup->ProcessContour.object.height * 0.7f);

	for (m = 0; m < m_globlparam[nId].m_nFastCornerNum; m++) {

		pCorner = m_globlparam[nId].m_pFastCorner + m;

		if (isPointInRect(&pCorner->point, &shrinkRec.object)
			&& !pCorner->State.nMacthNum) {
			if (pgroup->ntrajecyNum >= MAX_TRACKS_NUM_OF_GROUP) {
				break;
			}

			pTrajecy = pgroup->pObjtr + pgroup->ntrajecyNum;

			pTrajecy->nMapGroupId = m_globlparam[nId].m_GroupId;
			pTrajecy->nEstTimes = 0;

			pTrajecy->pTrackPoint->point = pCorner->point;
			pTrajecy->pTrackPoint->nFramseq = m_globlparam[nId].scaleInput.nFramSeq;
			pTrajecy->pTrackPoint->nTime = m_globlparam[nId].scaleInput.objTime;
			pTrajecy->pTrackPoint->nMatchStatus = 1;
			pTrajecy->nTrackLen++;

			//memcpy(pTrajecy->pfeature,m_globlparam.m_pfeature + m * SURF_DESC_DIMENTION,SURF_DESC_DIMENTION);
			memcpy(pTrajecy->pfeature, m_globlparam[nId].m_pfeature + (m << 6),
				SURF_DESC_DIMENTION);

			pgroup->ntrajecyNum++;
		}
		else if (isPointInRect(&pCorner->point, &pgroup->rtContour.object)
			&& !pCorner->State.nMacthNum) {
			if (nInRecNum < MAX_PUBLIC_SPACE_SIZE) {
				m_globlparam[nId].m_PublacSape[nInRecNum++] = m;
			}

		}
	}

	for (m = 0; m < nInRecNum; m++) {

		if (pgroup->ntrajecyNum >= MAX_TRACKS_NUM_OF_GROUP) {
			break;
		}

		pCorner = m_globlparam[nId].m_pFastCorner
			+ m_globlparam[nId].m_PublacSape[m];

		pTrajecy = pgroup->pObjtr + pgroup->ntrajecyNum;

		pTrajecy->nMapGroupId = m_globlparam[nId].m_GroupId;
		pTrajecy->nEstTimes = 0;

		pTrajecy->pTrackPoint->point = pCorner->point;
		pTrajecy->pTrackPoint->nFramseq = m_globlparam[nId].scaleInput.nFramSeq;
		pTrajecy->pTrackPoint->nTime = m_globlparam[nId].scaleInput.objTime;
		pTrajecy->pTrackPoint->nMatchStatus = 1;
		pTrajecy->bProcessTracked = 0;
		pTrajecy->bInitTracked = 0;
		pTrajecy->nTrackLen++;

		memcpy(pTrajecy->pfeature,
			m_globlparam[nId].m_pfeature + m * SURF_DESC_DIMENTION,
			SURF_DESC_DIMENTION);

		pgroup->ntrajecyNum++;

	}

}

/*
I/O:	    Name		    Type	     		  Content

[in/out]	RioImg		    WissenImage*	              ROI image buffer
[in]	    RioRec		    AdasRect	          ROI rect.
[in]	    bPresent		unsigned char	              if 1 get ROI of m_globlparam[nId].m_pGrayData，else get from m_globlparam[nId].m_preGrayData.
[in]	    nId		        int		              scale factor.

Realized function:
+  get the ROI image buffer.
*/
static void mvSelcImg(WissenImage * RioImg, WissenObjectRectTracked RioRec, unsigned char bPresent, int nId)
{
	int j;
	unsigned char *pDstr = 0;
	unsigned char *pSrctr = 0;

	RioImg->nWid = RioRec.object.width;
	RioImg->nHig = RioRec.object.height;

	if (bPresent) {
		for (j = RioRec.object.y; j < RioRec.object.y + RioRec.object.height; j++) {
			pDstr = RioImg->data + RioImg->nWid * (j - RioRec.object.y);

			pSrctr = m_globlparam[nId].m_pGrayData
				+ m_globlparam[nId].m_ImgWidth * j;

			memcpy(pDstr, pSrctr + RioRec.object.x, RioRec.object.width);

		}
	}
	else {
		for (j = RioRec.object.y; j < RioRec.object.y + RioRec.object.height; j++) {
			pDstr = RioImg->data + RioImg->nWid * (j - RioRec.object.y);

			pSrctr = m_globlparam[nId].m_preGrayData
				+ m_globlparam[nId].m_ImgWidth * j;

			memcpy(pDstr, pSrctr + RioRec.object.x, RioRec.object.width);

		}

	}

}


/*
Function process:
+ Do the re-det around the pGroup->rtContour and get the new location as pDetRec
Fan-in :
+ mvPreditGroup()
Fan-out:
+ FCWSD_Processor_ROI()
ATTENTION: __________
*/
static unsigned char mvDetcorBytrainByGroup(const PortInput *pInPutParam,
	obj_group * pGroup, WissenObjectRectTracked *pDetRec, unsigned char bSync, int nId)
{
	WissenObjectRectTracked OriRioRec;
	WissenObjectRect tempRect = { 0, 0, m_globlparam[nId].m_ImgWidth - 1, m_globlparam[nId].m_ImgHeight - 1 };
	WissenObjectRect tempRect1 = { 0, 0, m_globlparam[nId].m_ImgWidth, m_globlparam[nId].m_ImgHeight };
	int i;
	int nDistance;
	int nExtend_x, nExtend_y;

#ifdef NEW_DET_PORT
	WissenSize fcwsDetMinSize;
	WissenSize fcwsDetMaxSize;
	WissenImage imgROITemp;
	WissenRect roi;
#endif

	if (Car != pGroup->ntype || !pGroup->nTruelyObj) {
		return 0;
	}

	if (pGroup->nMinCarWid > 0) {

#ifndef NEW_DET_PORT
		nExtend = nExtend * 0.4f;
		OriRioRec.x = (pGroup->rtContour.x - nExtend);
		OriRioRec.y = (pGroup->rtContour.y - nExtend);
		OriRioRec.width = (pGroup->rtContour.width + (nExtend << 1));
		OriRioRec.height = (pGroup->rtContour.height + (nExtend << 1));

		mvScopeImg(&OriRioRec, adasrect(0, 0, m_globlparam.m_ImgWidth - 1, m_globlparam.m_ImgHeight - 1));

		imgRio.data = (unsigned char*)m_globlparam.m_PublacSape;

		mvSelcInputImg(pInPutParam, &imgRio, OriRioRec);
#endif

#ifdef NEW_DET_PORT

		nExtend_x = (int)(pGroup->rtContour.object.width * 0.2f);
		nExtend_y = (int)(pGroup->rtContour.object.height * 0.2f);
		OriRioRec.object.x = (pGroup->rtContour.object.x - nExtend_x);
		OriRioRec.object.y = (pGroup->rtContour.object.y - nExtend_y);
		OriRioRec.object.width = (pGroup->rtContour.object.width + (nExtend_x << 1));
		OriRioRec.object.height = (pGroup->rtContour.object.height + (nExtend_y << 1));

		limitObjectRectRange(tempRect, &OriRioRec.object);

		imgROITemp.data = pInPutParam->pOriGrayfram.data;
		imgROITemp.nWid = (int)(m_globlparam[nId].m_ImgWidth * m_globlparam[nId].scaleInput.fzoom);
		imgROITemp.nHig = (int)(m_globlparam[nId].m_ImgHeight * m_globlparam[nId].scaleInput.fzoom);

		fcwsDetMinSize.width = (int)(pGroup->rtContour.object.width
			* m_globlparam[nId].scaleInput.fzoom / 1.2f - 1);
		fcwsDetMinSize.height = (int)(pGroup->rtContour.object.height
			* m_globlparam[nId].scaleInput.fzoom / 1.2f - 1);

		fcwsDetMaxSize.width = (int)(pGroup->rtContour.object.width
			* m_globlparam[nId].scaleInput.fzoom * 1.2f + 1);
		fcwsDetMaxSize.height = (int)(pGroup->rtContour.object.height
			* m_globlparam[nId].scaleInput.fzoom * 1.2f + 1);

		roi.x = (int)(OriRioRec.object.x * m_globlparam[nId].scaleInput.fzoom);
		roi.y = (int)(OriRioRec.object.y * m_globlparam[nId].scaleInput.fzoom);
		roi.width = (int)(OriRioRec.object.width * m_globlparam[nId].scaleInput.fzoom);
		roi.height = (int)(OriRioRec.object.height * m_globlparam[nId].scaleInput.fzoom);

		if (m_globlparam[nId].pOriInPutParam->dayOrNight == 0)
		{
			FCWSD_Processor_ROI(3, 0, &imgROITemp, &roi,
				&fcwsDetMinSize, &fcwsDetMaxSize, 1);
			FCWSD_GetResult(3, &g_pDetobjsets);
		}
		else if (m_globlparam[nId].pOriInPutParam->dayOrNight == 1)
		{
			FCWSD_Processor_ROI(7, 0, &imgROITemp, &roi,
				&fcwsDetMinSize, &fcwsDetMaxSize, 1);
			FCWSD_GetResult(7, &g_pDetobjsets);
		}

		for (i = 0; i < g_pDetobjsets->nObjectNum; ++i) {
			g_pDetobjsets->objects[i].x = (int)(g_pDetobjsets->objects[i].x / m_globlparam[nId].scaleInput.fzoom);
			g_pDetobjsets->objects[i].y = (int)(g_pDetobjsets->objects[i].y / m_globlparam[nId].scaleInput.fzoom);
			g_pDetobjsets->objects[i].width =
				(int)(g_pDetobjsets->objects[i].width / m_globlparam[nId].scaleInput.fzoom);
			g_pDetobjsets->objects[i].height =
				(int)(g_pDetobjsets->objects[i].height / m_globlparam[nId].scaleInput.fzoom);
		}
#else

		FCW_DETCOR_VehicleDetprocess_Rio(&imgRio, nn, WissenSize((int)(pGroup->rtContour.width * 0.6f), (int)(pGroup->rtContour.width * 0.6f)),
			WissenSize((int)(OriRioRec.width), (int)(OriRioRec.width)), 0);
#endif


		if (1 == g_pDetobjsets->nObjectNum) {

#ifdef  NEW_DET_PORT

			pDetRec->object.x = (g_pDetobjsets->objects[0].x);
			pDetRec->object.y = (g_pDetobjsets->objects[0].y);
			pDetRec->object.width = (g_pDetobjsets->objects[0].width);
			pDetRec->object.height =
				(g_pDetobjsets->objects[0].height);

#else
			pDetRec->x = (g_pDetobjsets->objects[0].x + OriRioRec.x);
			pDetRec->y = (g_pDetobjsets->objects[0].y + OriRioRec.y);
			pDetRec->width = (g_pDetobjsets->objects[0].width);
			pDetRec->height = (g_pDetobjsets->objects[0].height);
#endif

			limitObjectRectRange(tempRect1, &pDetRec->object);

			g_pDetobjsets->nObjectNum = 0;
			pDetRec->nType = Car;

			pDetRec->object.confidence = pGroup->rtContour.object.confidence;

			if (bSync) {
				nDistance =
					WS_ABS(pDetRec->object.x + (pDetRec->object.width >> 1) - (pGroup->rtContour.object.x + (pGroup->rtContour.object.width >> 1)))
					+ \
					WS_ABS(pDetRec->object.y + (pDetRec->object.height >> 1) - (pGroup->rtContour.object.y + (pGroup->rtContour.object.height >> 1)));

				if (nDistance > 0.3f * pGroup->rtContour.object.width) {
					return 0;
				}
			}
			return 1;
		}
		return 0;
	}

	return 0;
}

/*
Function process:
+ Update pGroup by detected result
Fan-in :
+ mvUpdataByDetctor()
Fan-out:
+ mvSetOriInitGray()
+ mvDelUnReasonTrack()
+ mvAddTrajecToGroup()
+ mvEstablInitVote()
+ mvSelcImg()
ATTENTION: __________
*/
static void mvUpdataGroupByDet(obj_group *pGroup, WissenObjectRectTracked *pDetrec, int nId) 
{
	WissenObjectRectTracked ShrinkTempRec;
	float fzoom = (float)GROUP_TEMPLATE_SHINK_RATE;
	int nValidCar = 0;
	WissenObjectRect tempRect = { 0, 0, m_globlparam[nId].m_ImgWidth, m_globlparam[nId].m_ImgHeight };

	unsigned char bFilte = (isRectOverlapped(&pGroup->rtContour.object, &pDetrec->object, 0.7f)
		|| isRectOverlapped(&pDetrec->object, &pGroup->rtContour.object, 0.7f));

	if (bFilte) {
		//
		pDetrec->object.x = (pDetrec->object.x + (pDetrec->object.width >> 1))
			- (pGroup->rtContour.object.width >> 1);
		pDetrec->object.y = (pDetrec->object.y + (pDetrec->object.height >> 1))
			- (pGroup->rtContour.object.height >> 1);
		pDetrec->object.width = pGroup->rtContour.object.width;
		pDetrec->object.height = pGroup->rtContour.object.height;
		nValidCar = 1;
	}

	limitObjectRectRange(tempRect, &pDetrec->object);

	pGroup->rtContour = *pDetrec;
	pGroup->InitContour = *pDetrec;

	if (nValidCar) {
		mvSetOriInitGray(pGroup, nId);
	}

	mvDelUnReasonTrack(pGroup);

	mvAddTrajecToGroup(pGroup, nId);

	mvEstablInitVote(pGroup, 0, nId, m_globlparam[nId].scaleInput.nFramSeq);

	pGroup->OriInitContour = *pDetrec;
	pGroup->histoyRec.nSizNum = 0;
	pGroup->histoyRec.pGroupSiz[0] = pDetrec->object.width;
	pGroup->histoyRec.pGroupSiz[MAX_HISRORY_GROUP_REC_NUM] = pDetrec->object.height;
	pGroup->histoyRec.nSizNum++;
	pGroup->updataFrambyCar = m_globlparam[nId].scaleInput.nFramSeq;
	pGroup->SerpreditNum = 0;
	pGroup->nMinCarWid = WS_MIN(pGroup->nMinCarWid, pGroup->rtContour.object.width);
	pGroup->CarBottom.nDetBottomNum = 0;

	mvSetOriInitGray(pGroup, nId);

	if (WS_MAX(pGroup->rtContour.object.width, pGroup->rtContour.object.height)
		< MAX_TEMPLAT_NCC_TRACK_SIZE) {
		mvShinkRect(pGroup->rtContour.object, &ShrinkTempRec.object, fzoom);

		mvSelcImg(&pGroup->Templat, ShrinkTempRec, 1, nId);
	}

}

/*
I/O:	    Name		          Type	     		  Content

[in]	    pInPutParam		      const PortInput*	  input stract of multi-tracking.
[in]	    nId		              const int		      Scale index.

Realized function:
+ Calculate the ground line of every target
*/
static void mvGroundLineDet(const PortInput *pInPutParam, const int nId)
{
	int i;
	obj_group *pGroup = 0;
	unsigned char *ptr;
	unsigned char localGroundValue = -1, groundValue = 70;// pInPutParam->pOriGrayfram.groundValue;
	float coeff = 0.1;
	int deltCols = 0;
	WissenObjectRectTracked rltRect;
	int groundHist[256] = { 0 };

	for (i = 0; i < MAX_LAYER_OBJ_GROUP_NUMS; i++)
	{
		unsigned short hist[400] = { 0 };
		int x, y, z, objHeight, groundRow, rows, temp0, maxGrayValue;
		pGroup = m_globlparam[nId].m_pGroupSets + i;
		if (-1 == pGroup->nGroupId || 1 != pGroup->nTruelyObj)
		{
			continue;
		}

		// local ground value
		objHeight = WS_MIN(6 * pGroup->rtContour.object.height / 4, m_globlparam[nId].m_ImgHeight - pGroup->rtContour.object.y);
		ptr = m_globlparam[nId].m_pGrayData + (pGroup->rtContour.object.y + objHeight - 1) * m_globlparam[nId].m_ImgWidth;
		for (y = 0; y < objHeight * 1 / 6; y++)
		{
			for (x = pGroup->rtContour.object.x; x < pGroup->rtContour.object.x + pGroup->rtContour.object.width; x++)
			{
				z = ptr[x];
				groundHist[z]++;

			}
			ptr -= m_globlparam[nId].m_ImgWidth;
		}

		maxGrayValue = -256;
		for (y = 0; y < 256; y++)
		{
			if (groundHist[y] > maxGrayValue)
			{
				maxGrayValue = groundHist[y];
				localGroundValue = y;
			}
		}
		//cv::Mat src2(720, 1280, CV_8UC1, pInPutParam->pOriGrayfram.ptr);
		//cv::line(src2, cv::Point(0, pLDWSInit->pBoundPoint[2].y), cv::Point(1280, pLDWSInit->pBoundPoint[2].y), cv::Scalar(0, 0, 255), 2);
		//cv::rectangle(src2, cv::Point(pGroup->rtContour.x << nId, pGroup->rtContour.y << nId), cv::Point( ((pGroup->rtContour.x + pGroup->rtContour.width) << nId), ((pGroup->rtContour.y + pGroup->rtContour.height) << nId) ), cv::Scalar(0, 0, 255), 2);
		//namedWindow("src2", CV_WINDOW_NORMAL);
		//imshow("src2", src2);

		LDWS_Getinit(&pLDWSInit);
		if (((pGroup->rtContour.object.y + objHeight - 1) << nId) < pLDWSInit->pBoundPoint[2].y && localGroundValue <= groundValue)
		{
			groundValue = localGroundValue;
		}

		//can use the diff of gray value and the diff of rows as weight
		//objHeight = WS_MIN(5 * pGroup->rtContour.height / 4, m_globlparam[nId].m_ImgHeight - pGroup->rtContour.y);
		ptr = m_globlparam[nId].m_pGrayData + (pGroup->rtContour.object.y + 5 * objHeight / 6) * m_globlparam[nId].m_ImgWidth;
		temp0 = 0;
		rows = 0;
		for (y = 0; y < 3 * objHeight / 6; y++)
		{
			for (x = pGroup->rtContour.object.x; x < pGroup->rtContour.object.x + pGroup->rtContour.object.width; x++)
			{
				if (ptr[x] < groundValue * 0.9)
				{
					hist[rows]++;
				}
			}
			ptr -= m_globlparam[nId].m_ImgWidth;
			temp0 += hist[rows];
			hist[rows] = temp0;
			rows++;
		}

		// calculate the ground value segmentation line
		z = 0;
		groundRow = -1;
		for (y = 5 * objHeight / 6; y > 3 * objHeight / 6; y--)
		{
			if (hist[z] > temp0 * coeff)
			{
				groundRow = y;
				break;
			}
			z++;
		}

		//can use the diff of gray value and the diff of rows as weight
		//objHeight = WS_MIN(5 * pGroup->rtContour.height / 4, m_globlparam[nId].m_ImgHeight - pGroup->rtContour.y);
		//ptr = m_globlparam[nId].m_pGrayData + (pGroup->rtContour.y + objHeight - 1) * m_globlparam[nId].m_ImgWidth;
		//temp1 = 0;
		//rows = 0;
		//memset(hist, 0, sizeof(unsigned short)* 400);
		//for (y = 0; y < objHeight * 2 / 5; y++)
		//{
		//	for (x = pGroup->rtContour.x; x < pGroup->rtContour.x + pGroup->rtContour.width; x++)
		//	{
		//		if (ptr[x] < groundValue * 0.5)
		//		{
		//			hist[rows]++;
		//		}
		//	}
		//	ptr -= m_globlparam[nId].m_ImgWidth;
		//	temp1 += hist[rows];
		//	hist[rows] = temp1;
		//	rows++;
		//}

		//// calculate the ground value segmentation line
		//z = 0;
		//groundRow2 = -1;
		//for (y = objHeight - 1; y > objHeight * 2 / 5 - 1; y--)
		//{
		//	if (hist[z] > temp1 * coeff)
		//	{
		//		groundRow2 = y;
		//		break;
		//	}
		//	z++;
		//}

		// updata the target rect
		rltRect = pGroup->rtContour;
		//if ( WS_ABS(temp0 - temp1) )
		//groundRow = groundRow2;

		if ((WS_ABS(groundRow - pGroup->rtContour.object.height) > pGroup->rtContour.object.height / 4))
		{
			WissenObjectRectTracked DetRect;
#ifdef DETCOR_STAR	
			unsigned char bDetCarByRio = mvDetcorBytrainByGroup(pInPutParam, pGroup,
				&DetRect, 1, nId);
#else
			bDetCarByRio = 0;
#endif
			if (!bDetCarByRio)
			{
				my_printf(
					"***pgroup ID: %d is not Tracked and not bottom shadow detected!\n",
					pGroup->nGroupId);
				mvClearGroupIfo(pGroup, 1);
			}
			else
			{
				pGroup->nDisCardNum = 0;
				mvUpdataGroupByDet(pGroup, &DetRect, nId);
				mvUpdataGroupCenTraj(pGroup);
			}
		}
		else
		{
			//			if (pGroup->sysKalmState.x[5] > 3 || pGroup->sysKalmState.x[5] < 0.5)
			//			{
			//				AdasRect DetRect;
			//#ifdef DETCOR_STAR	
			//				unsigned char bDetCarByRio = mvDetcorBytrainByGroup(pInPutParam, pGroup,
			//					&DetRect, 1, nId);
			//#else
			//				bDetCarByRio = 0;
			//#endif
			//				if (!bDetCarByRio)
			//				{
			//					my_printf(
			//						"***pgroup ID: %d is not Tracked and not bottom shadow detected!\n",
			//						pGroup->nGroupId);
			//					mvClearGroupIfo(pGroup, 1);
			//				}
			//				else
			//				{
			//					pGroup->nDisCardNum = 0;
			//					mvUpdataGroupByDet(pGroup, &DetRect, nId);
			//					mvUpdataGroupCenTraj(pGroup);
			//				}
			//			}
			//			else
			//			{
			//groundRow = WS_MIN(groundRow, m_globlparam[nId].m_ImgHeight - pGroup->rtContour.y);
			//groundRow = (groundRow + pGroup->rtContour.height) / 2;
			//deltCols = (groundRow - pGroup->rtContour.width) / 2;
			//rltRect.height = groundRow;
			//rltRect.x -= WS_MIN(deltCols, pGroup->rtContour.x);
			//rltRect.width += WS_MIN(deltCols, m_globlparam[nId].m_ImgWidth - pGroup->rtContour.x) * 2;

			groundRow = (groundRow + pGroup->rtContour.object.height) / 2;
			deltCols = groundRow - pGroup->rtContour.object.height;
			rltRect.object.y = WS_MAX(rltRect.object.y, deltCols);
			rltRect.object.height = WS_MIN(pGroup->rtContour.object.height + 2 * deltCols, m_globlparam[nId].m_ImgHeight - rltRect.object.y);
			rltRect.object.x = WS_MAX(rltRect.object.x, deltCols);
			rltRect.object.width = WS_MIN(pGroup->rtContour.object.width + 2 * deltCols, m_globlparam[nId].m_ImgWidth - rltRect.object.x);
			pGroup->rtContour = rltRect;
		}
	}
}

/*
Function process:
+ extract the corner points in mask region, the result saved in m_globlparam.m_nCornerPassNum and m_globlparam.m_nFastCornerNum
*/
static void mvCornerDetct(const WissenImage *srcImage, const unsigned char *pMask,
	const int Barrier, const int nId) {
	WissenPoint *corners = 0;
	WissenPoint *corners_nonmax = 0;
	WissenRect imageRoi;
	int num_corners = 0;
	int num_nonmax = 0;
	int i;

	m_globlparam[nId].m_nCornerPassNum = 0;
	m_globlparam[nId].m_nFastCornerNum = 0;

#ifdef TIME_TEST
	double ts = cvGetTickCount();
#endif

#ifdef MV_ADAS_USE_FAST_STATIC

#ifdef  FAST_OAST9_16
	imageRoi.x = m_globlparam[nId].nscope_start_x;
	imageRoi.y = m_globlparam[nId].nscope_start_y;
	imageRoi.width = m_globlparam[nId].nscope_end_x - m_globlparam[nId].nscope_start_x;
	imageRoi.height = m_globlparam[nId].nscope_end_y - m_globlparam[nId].nscope_start_y;
	fastCornerDetect9_16(srcImage, pMask, imageRoi, Barrier, MAX_XY_CORNER, &num_corners, m_globlparam[nId].m_pXYCorners);
#else
	corners = fast_corner_detect_9(pGray, width, height, Barrier, &num_corners, m_globlparam.m_pXYCorners);

#endif
	corners = m_globlparam[nId].m_pXYCorners;

#else
	corners = fast_corner_detect_9(pGray, width, height, Barrier, &num_corners, 0);

#endif

#ifdef TIME_TEST
	my_printf("fast_corner_detect_9:%0.3fms\n", (cvGetTickCount() - ts) / cvGetTickFrequency() / 1000.0f);
	ts = cvGetTickCount();
#endif

#ifdef MV_ADAS_USE_FAST_STATIC
	corners_nonmax = fast_nonmax(srcImage, corners, num_corners,
		Barrier, &num_nonmax, m_globlparam[nId].m_pRowStart,
		m_globlparam[nId].m_pScore, m_globlparam[nId].m_pXYNoMax);
#else
	corners_nonmax = fast_nonmax(pGray, width, height, corners, num_corners, Barrier, &num_nonmax, 0, 0, 0);
#endif

#ifdef TIME_TEST
	my_printf("fast_nonmax:%0.3fms\n", (cvGetTickCount() - ts) / cvGetTickFrequency() / 1000.0f);
	ts = cvGetTickCount();
#endif

	if (num_nonmax < MAX_CORNERS_PER_FRAME)
	{
		for (i = 0; i < num_nonmax; ++i)
		{
			m_globlparam[nId].m_pFastCorner[m_globlparam[nId].m_nFastCornerNum].point = corners_nonmax[i];
			m_globlparam[nId].m_pFastCorner[m_globlparam[nId].m_nFastCornerNum].State.nMacthNum =
				0;
			m_globlparam[nId].m_pFastCorner[m_globlparam[nId].m_nFastCornerNum++].State.nMatchDis =
				0;
		}

		return;
	}

	for (i = 0; i < num_nonmax; ++i)
	{
		if (m_globlparam[nId].m_nCornerPassNum >= MAX_CORNERS_OF_NONMAX - 1) 
		{
			break;
		}
		m_globlparam[nId].m_pCornerPass[m_globlparam[nId].m_nCornerPassNum].point = corners_nonmax[i];

		m_globlparam[nId].m_pCornerPass[m_globlparam[nId].m_nCornerPassNum].State.nMacthNum =
			0;
		m_globlparam[nId].m_pCornerPass[m_globlparam[nId].m_nCornerPassNum++].State.nMatchDis =
			0;

	}

#ifndef MV_ADAS_USE_FAST_STATIC
	my_free(corners_nonmax);
	my_free(corners);
#endif

	CalculateHarrisResponse(srcImage, m_globlparam[nId].m_pCornerPass, m_globlparam[nId].m_nCornerPassNum);

#ifdef TIME_TEST
	my_printf("CalculateHarrisResponse:%0.3fms\n", (cvGetTickCount() - ts) / cvGetTickFrequency() / 1000.0f);
	ts = cvGetTickCount();
#endif

	CornerResponseRestrain(m_globlparam[nId].m_pCornerPass,
		m_globlparam[nId].m_pFastCorner, m_globlparam[nId].m_nCornerPassNum,
		&m_globlparam[nId].m_nFastCornerNum, MAX_CORNERS_PER_FRAME);

#ifdef TIME_TEST
	my_printf("CornerResponseRestrain:%0.3fms\n", (cvGetTickCount() - ts) / cvGetTickFrequency() / 1000.0f);
	ts = cvGetTickCount();
#endif

}

/*
 I/O:	    Name		    Type	     		  Content

 [in]    	pInPutParam	    const PortInput*	  input stract of multi-tracking.
 [in]	    nId		        const int		      Scale index.

 Realized function:
 + Get the tracking tragets for single-scale tracking, Given the value of m_globlparam[nId].scaleInput.
 */
static void mvGetScaleInPut(const PortInput *pInPutParam, const int nId);

/*
 I/O:	    Name		    Type	     		  Content

 [in]	    nId		        const int		      Scale index.

 Realized function:
 + find m_globlparam[nId].m_pGroupSets->nGroupId != -1; change m_globlparam[nId].m_GroupIndexNum
 and m_globlparam[nId].m_pGroupIndex
 */
static void mvGetCurrenGroupIndex(const int nId);

/*
 I/O:	    Name		    Type	     		  Content

 [in]        pInPutParam	    PortInput*	          input stract of multi-tracking.
 [in]	    nId		        const int		      Scale index.

 Realized function:
 + Add new tracking target based on the detected result. Update m_globlparam.m_PNewRec
 */
static void mvAddNewObjRec(const int nId);

/*
 I/O:	    Name		    Type	     		  Content

 [in]	    nId		        const int		      Scale index.

 Realized function:
 + update pgroup->PreditRec
 */
static void mvGoupPreditRec(const int nId);

/*
 I/O:	    Name		    Type	     		  Content

 [in]        pInPutParam	    PortInput*	          input stract of multi-tracking.
 [in]	    nId		        const int		      Scale index.

 Realized function:
 + update m_globlparam[nId].m_pMask based on pInPutParam->objRec and m_globlparam[nId].m_pGroupSets
 */
static void mvGenerateRioGrayByMask(const int nId);


/*
 I/O:	    Name		    Type	     		  Content

 [in]	    nId		        const int		      Scale index.

 Realized function:
 + match the traject point with the fast points and update m_globlparam[nId].m_pGroupSets->pObjtr->pTrackPoint
 */
static void mvTrajectorymatch(int nId);

/*
 I/O:	    Name		    Type	     		  Content

 [in]        RioRec	        AdasRect	          ROI region of target
 [in/out]	pCarimg		    WissenImage*		          output car image buffer
 [in]	    nId		        const int		      Scale index.

 Realized function:
 + Get the car temple img based on ROI region of RioRec
 */
static void mvSetUpdataCarImg(WissenObjectRectTracked RioRec, WissenImage *pCarimg, int nId);

/*
 I/O:	    Name		    Type	     		  Content

 [in]	    pGroup		    const obj_group*	  target group
 [in]	    pLapCarRec		AdasRect*		      detected car region.
 [in]	    nId		        int		              scale factor.

 [out]       returned        int                   if 1 use the detected rsult to update group; else, return 0

 Realized function:
 +  match the detect region in pGroup if matched return 1; else return 0
 */
static int mvMatchDetRec(const obj_group *pGroup, WissenObjectRectTracked *pLapCarRec, int nId);

/*
 I/O:	    Name		    Type	     		  Content

 [in/out]	pGroup		    obj_group*	          target group
 [in]	    RioRec		    AdasRect		      detected rect.
 [in]	    nId		        int		              scale factor.

 Realized function:
 +  update the temple of target(pGroup->UpdatedImg) based on the detected region
 */
static void mvUpdataNormalCarImg(obj_group * pGroup, WissenObjectRectTracked RioRec, int nId);

/*
 I/O:	    Name		    Type	     		  Content

 [in/out]	pGroup		    obj_group*	          target group
 [in]	    pInPutParam		const PortInput*	  detetcted result.
 [in]	    nId		        int		              scale factor.

 Realized function:
 + Update pGroup by detected result
 */
static unsigned char mvUpdataByDetctor(obj_group *pgroup, int nId);

/*
 I/O:	    Name		       Type	     		  Content

 [in]	    SelfGroupIndex	   unsigned char	          input group index
 [in/out]	OccluedRec		   AdasRect *         overlopped region.
 [in/out]	pFindInxdex		   int*		          overlopped group index.
 [in]	    nId		           int		          scale factor.

 [out]       returned           unsigned char            if there is overlopped return 1; else return 0

 Realized function:
 + decide if there is overlapped in differnet scales
 */
static unsigned char mvOccludedbyGroup(unsigned char SelfGroupIndex, WissenObjectRectTracked *OccluedRec,
		int *pFindInxdex, int nId);

/*
 I/O:	    Name		    Type	     		  Content

 [in/out]	pGroup		    obj_group*	          target group
 [in/out]	MatchResultRec	AdasRect*	          matched result
 [in]	    nId		        int		              scale factor.
 [in/out]	pfMatchScore	float*	          matching score

 [out]	    returned    	unsigned char	              if 1 find the matched rect; else return 0

 Realized function:
 + do the temple matching in pgroup->rtContour by pgroup->Templat
 */
static unsigned char mvMatchByGroupTemplate(obj_group *pgroup,
	WissenObjectRectTracked *MatchResultRec, int nId, float *pfMatchScore);

/*
 I/O:	    Name		    Type	     		  Content

 [in/out]	pGroup		    obj_group*	          target group
 [in]	    bUpdata	        unsigned char	              if 1 using the filter
 [in]	    nId		        int		              scale factor.

 Realized function:
 + Do the midle filter for pGroup->rtContour based on pGroup->histoyRec
 */
static void mvGroupFilter(obj_group *pgroup, unsigned char bUpdata, int nId);

/*    
 I/O:	    Name		        Type	     		  Content

 [in/out]	pGroup		        obj_group*	          target group
 [in]	    MidVoteCen	        Wissen16SPoint	          middle vote center
 [in]	    pVoteTrajecIndex	int *	              vote traject index
 [in]	    nVoteNum	        int 	              vote traject Num
 [in]	    nDisTrd	            int 	              traject vote bias threshold
 [in]	    nId		            int		              scale factor.

 Realized function:
 + return the Num of traject that has similar vote with MidVoteCen
 */
static int mvDelFarCen(obj_group *pGroup, Wissen16SPoint MidVoteCen,
		int * pVoteTrajecIndex, int nVoteNum, int nDisTrd, int nId);

/*    
 I/O:	    Name		        Type	     		  Content

 [in/out]	pGroup		        obj_group*	          target group
 [in]	    nId		            int		              scale factor.

 [out]	    returned		    unsigned char		          the similar tarject Num.

 Realized function:
 + caculate group sacles and location based on pgroup->InitContour and pTrajecy->InitPoint, update pgroup->rtContour
 */
static unsigned char mvInitConsensVote(obj_group *pgroup, int nId);

/*    
 I/O:	    Name		        Type	     		  Content

 [in/out]	pGroup		        obj_group*	          target group
 [in]	    nId		            int		              scale factor.

 Realized function:
 + update pGroup->ProcessContour，pGroup->nPreProssVotFram，pTrajecy->bProcessTracked，pTrajecy->ProcessVote，pTrajecy->processfeature
 */
static void mvUpdataProcessVote(obj_group *pGroup, int nId);

/*    
 I/O:	    Name		        Type	     		  Content

 [in]	    pGroup		        obj_group*	          target group
 [in/out]	pResImg		        unsigned char*	          output image buffer
 [in]	    nId		            int		              scale factor.

 Realized function:
 + copy and resize the image of pgroup->OriInitContour to pResImg, pResImg has the same size of pgroup->rtContour
 */
static void mvOriObjResizeToDstImg(obj_group *pgroup, unsigned char *pResImg, int nId);

/*    
 I/O:	    Name		        Type	     		  Content

 [in/out]	pgroup		        obj_group*	          Tracking target group
 [in/out]	pMatchRec		    AdasRect*	          temple matched result
 [in]	    nId		            int		              scale factor.
 [in]	    fMatchScore		    float*		      matched score.

 [out]	    returned		    unsigned char		          if 1 matching succees,else return 0.

 Realized function:
 + Do the temple matching for group if vote tarcking failed
 */
static unsigned char mvTempleMatchByOri(obj_group *pgroup, WissenObjectRectTracked *pMatchRec,
		int nId, float *fMatchScore);

/*    
 I/O:	    Name		        Type	     		  Content

 [in/out]	pGroup		        obj_group*	          target group
 [in]	    nId		            int		              scale factor.

 [out]	    returned		    unsigned char		          the similar tarject Num.

 Realized function:
 + caculate group sacles and location based on pTrajecy->ProcessPoint and pTrajecy->InitPoint, update pgroup->rtContour
 */
static unsigned char mvProcessConsensVote(obj_group *pgroup, int nId);

/*    
 I/O:	    Name		        Type	     		  Content

 [in]	    pInPutParam		    const PortInput*      target group
 [in]	    nId		            int		              scale factor.

 Realized function:
 + Tracking the groups in m_globlparam[].m_pGroupSets
 */
static void mvPreditGroup(const PortInput *pInPutParam, int nId);

/*    
 I/O:	    Name		        Type	     		  Content

 [in]	    nId		            int		              scale factor.

 Realized function:
 + add new group to m_globlparam.m_pGroupSets from m_globlparam[nId].m_PNewRec
 */
static void mvGroupGenerate(int nId);

/*    
 I/O:	    Name		        Type	     		  Content

 [in]	    nId		            int		              scale factor.

 Realized function:
 + Do the Kalman filter for pGroup->rtContour
 */
static void mvKalmanFiter(int nId);

/*
Realized function:
+ do the fushion for neighbor scale oobject
*/
static void neighborScaleObjectFusion(const int scale1, const int scale2, obj_group *scale1ObjectList, obj_group *scale2ObjectList)
{
	int i, j;
	obj_group *pgroup, *pComparedgroup;
	WissenObjectRectTracked srcRec;
	unsigned char bFilte;
	WissenObjectRectTracked tempRect = { 0 };
	tempRect.object.x = 0;
	tempRect.object.y = 0;
	tempRect.object.width = UPDATA_IMAGE_SIZE;
	tempRect.object.height = UPDATA_IMAGE_SIZE;

	for (i = 0; i < MAX_LAYER_OBJ_GROUP_NUMS; i++)
	{
		pgroup = scale1ObjectList + i;
		if (-1 == pgroup->nGroupId)
			continue;

		srcRec.object.x = pgroup->rtContour.object.x >> scale2;
		srcRec.object.y = pgroup->rtContour.object.y >> scale2;
		srcRec.object.width = pgroup->rtContour.object.width >> scale2;
		srcRec.object.height = pgroup->rtContour.object.height >> scale2;
		srcRec.object.confidence = pgroup->rtContour.object.confidence;

		for (j = 0; j < MAX_LAYER_OBJ_GROUP_NUMS; j++)
		{
			pComparedgroup = scale2ObjectList + j;
			if (-1 == pComparedgroup->nGroupId)
				continue;

			bFilte = (isRectOverlapped(&srcRec.object, &pComparedgroup->rtContour.object, 0.7f)
				|| isRectOverlapped(&pComparedgroup->rtContour.object, &srcRec.object, 0.7f));
			if (!bFilte)
				continue;

			if (pgroup->nTruelyObj && pComparedgroup->nTruelyObj)
			{
				if (pgroup->nPerLastDetFramseq == m_globlparam[scale1].scaleInput.nFramSeq)
				{
					if (srcRec.object.confidence < pComparedgroup->rtContour.object.confidence)
					{
						mvClearGroupIfo(pgroup, 1);
					}
					else
					{
						// decide the two target if are the same
						int nDisTance = WS_ABS(pComparedgroup->rtContour.object.x + (pComparedgroup->rtContour.object.width >> 1) - \
							srcRec.object.x - (srcRec.object.width >> 1)) + \
							WS_ABS(pComparedgroup->rtContour.object.y + (pComparedgroup->rtContour.object.height >> 1) - \
							srcRec.object.y - (srcRec.object.height >> 1));

						if (WS_ABS(srcRec.object.width - pComparedgroup->rtContour.object.width) < 0.2 * srcRec.object.width || nDisTance < 0.4f * srcRec.object.width)
						{
							pComparedgroup->rtContour.object.confidence = pgroup->rtContour.object.confidence;
							mvClearGroupIfo(pgroup, 1);
						}
						else
						{
							WissenImage Carimg;
							float fMatchScore;
							float fAvgScore = 0.0f;
							WissenObjectRectTracked MatchRec;
							int noff = UPDATA_IMAGE_SIZE * UPDATA_IMAGE_SIZE;
							Carimg.nWid = UPDATA_IMAGE_SIZE;
							Carimg.nHig = UPDATA_IMAGE_SIZE;
							Carimg.data = (unsigned char*)m_globlparam[scale1].m_PublacSape;

							mvSetUpdataCarImg(pgroup->rtContour, &Carimg, scale1);
							for (j = 0; j < WS_MIN(pComparedgroup->nUpdataIndex, UPDATA_IMAGE_NUM); j++)
							{
								mvTemplatMatch(pComparedgroup->UpdatedImg[j], Carimg,
									tempRect, &MatchRec, noff, 0,
									m_globlparam[scale2].m_PublacSape, &fMatchScore);
								fAvgScore += fMatchScore;
							}
							fAvgScore /=
								WS_MIN(pComparedgroup->nUpdataIndex, UPDATA_IMAGE_NUM);

							if (fAvgScore > 0.5f)
							{
								pComparedgroup->rtContour.object.confidence = pgroup->rtContour.object.confidence;
								mvClearGroupIfo(pgroup, 1);
							}
							else
							{
								mvClearGroupIfo(pComparedgroup, 1);
							}
						}
					}

				}
				else if (pComparedgroup->nPerLastDetFramseq == m_globlparam[scale2].scaleInput.nFramSeq)
				{
					if (srcRec.object.confidence > pComparedgroup->rtContour.object.confidence)
					{
						mvClearGroupIfo(pComparedgroup, 1);
					}
					else
					{
						// decide the two target if are the same
						int nDisTance = WS_ABS(pComparedgroup->rtContour.object.x + (pComparedgroup->rtContour.object.width >> 1) - \
							srcRec.object.x - (srcRec.object.width >> 1)) + \
							WS_ABS(pComparedgroup->rtContour.object.y + (pComparedgroup->rtContour.object.height >> 1) - \
							srcRec.object.y - (srcRec.object.height >> 1));

						if (WS_ABS(srcRec.object.width - pComparedgroup->rtContour.object.width) < 0.2 * srcRec.object.width || nDisTance < 0.4f * srcRec.object.width)
						{
							pgroup->rtContour.object.confidence = pComparedgroup->rtContour.object.confidence;
							mvClearGroupIfo(pComparedgroup, 1);
						}
						else
						{
							WissenImage Carimg;
							float fMatchScore;
							float fAvgScore = 0.0f;
							WissenObjectRectTracked MatchRec;
							int noff = UPDATA_IMAGE_SIZE * UPDATA_IMAGE_SIZE;
							Carimg.nWid = UPDATA_IMAGE_SIZE;
							Carimg.nHig = UPDATA_IMAGE_SIZE;
							Carimg.data = (unsigned char*)m_globlparam[scale2].m_PublacSape;

							mvSetUpdataCarImg(pComparedgroup->rtContour, &Carimg, scale2);
							for (j = 0; j < WS_MIN(pgroup->nUpdataIndex, UPDATA_IMAGE_NUM); j++)
							{
								mvTemplatMatch(pgroup->UpdatedImg[j], Carimg,
									tempRect, &MatchRec, noff, 0,
									m_globlparam[scale1].m_PublacSape, &fMatchScore);
								fAvgScore += fMatchScore;
							}
							fAvgScore /=
								WS_MIN(pgroup->nUpdataIndex, UPDATA_IMAGE_NUM);

							if (fAvgScore > 0.5f)
							{
								pgroup->rtContour.object.confidence = pComparedgroup->rtContour.object.confidence;
								mvClearGroupIfo(pComparedgroup, 1);
							}
							else
							{
								mvClearGroupIfo(pgroup, 1);
							}
						}
					}
				}
				else
				{
					if (pComparedgroup->rtContour.object.width > srcRec.object.width)
					{
						mvClearGroupIfo(pgroup, 1);
					}
					else
					{
						mvClearGroupIfo(pComparedgroup, 1);
					}
				}

			}
			else  if (pgroup->nTruelyObj && !pComparedgroup->nTruelyObj)
			{
				mvClearGroupIfo(pComparedgroup, 1);
			}
			else if (!pgroup->nTruelyObj && pComparedgroup->nTruelyObj)
			{
				mvClearGroupIfo(pgroup, 1);
			}
		}
	}
}

/*
 I/O:	    Name		    Type	     		  Content

 [in/out]	pInPutParam	    PortInput*		      input stract of multi-tracking.
 [in]	    nId	            const int		      Scale index.

 Realized function:
 + Do the single-sacle tracking for pInPutParam in sacle index of nId.
 */
static void mvSingledScaleTrack(PortInput *pInPutParam, const int nId);

/*    
 I/O:	    Name		        Type	     		  Content

 [in/out]	pGroup		        obj_group*	          target group
 [in]	    nId		            int		              scale factor.
 [in]	    fZDis		        float		      Z distance of group.
 [in]	    fXDis		        float		      X distance of group.
 [in]	    fDelVanish		    float	          vanish line of group.

 Realized function:
 + set the value of pgroup->pMotion
 */
static void mvSetGroupMotion(obj_group *pgroup, int nId, float fZDis,
		float fXDis, float fDelVanish);

/*    
 Realized function:
 + set the value of pgroup->pMotion of each sacle in m_globlparam[nId].m_pGroupSets
 */
static void mvMotionState(void);

/*    
 I/O:	    Name		        Type	     		  Content

 [in/out]	pGroup		        obj_group*	          target group
 [in]	    nId		            int		              scale factor.

 Realized function:
 + Caculate the TTC and collison path for pgroup
 */
static void mvGroupTimeToCollison(CANInfo *CANData, obj_group *pgroup, int nId)
{

	int nLengTrd = 20;
	Motion *pNearestMotion = 0;
	Motion *pfarestMotion = 0;
	float dLmda, wLmda, lmda;
	float fTimeSub;
	double W_HostCar_Lef, W_HostCar_Rig;
	double W_Group_Lef, W_Group_Rig;
	WissenSystime SystimeSub;
	double tempRight = 0.0;
	double tempLeft = 0.0;

	int Eu, Ev, Cx, Cy;
	LDWS_Get_inter_Pamer_N(&Eu, &Ev, &Cx, &Cy);

	/* get the Nearest group */
	pNearestMotion = pgroup->pMotion
		+ ((pgroup->nMotionLeng - 1) & MAX_MOTION_BIT);
	pNearestMotion->bInCollishionPath = 0;
	pNearestMotion->fTTC = TTC_MAX;
	pNearestMotion->dTTC = TTC_MAX;
	if (!pgroup->nTruelyObj)
	{
		return;
	}

	//calculate the Collishion Path
	LDWS_GetVanishPointSet(&g_pLDWSVPoint);

	//printf("maxVPoint(x,y):%d,%d\n",(g_pLDWSVPoint).x,(g_pLDWSVPoint).y);
	W_HostCar_Lef = LDWS_GetXofWorld((g_pLDWSVPoint).x,
		(pgroup->rtContour.object.y + pgroup->rtContour.object.height)
		<< nId) - HALF_CAR_WIDTH;

	W_HostCar_Rig = W_HostCar_Lef + (HALF_CAR_WIDTH * 2);

	W_Group_Lef = LDWS_GetXofWorld(pgroup->rtContour.object.x << nId,
		(pgroup->rtContour.object.y + pgroup->rtContour.object.height) << nId);

	W_Group_Rig = LDWS_GetXofWorld(
		(pgroup->rtContour.object.x + pgroup->rtContour.object.width) << nId,
		(pgroup->rtContour.object.y + pgroup->rtContour.object.height) << nId);



	if (WS_ABS(W_HostCar_Lef)<50 && WS_ABS(W_HostCar_Rig)<50 && WS_ABS(W_Group_Lef) < 50 && WS_ABS(W_Group_Rig)< 50)
	{
		tempRight = WS_MAX(W_Group_Rig, W_HostCar_Rig);
		tempLeft = WS_MIN(W_HostCar_Lef, W_Group_Lef);
		//my_printf("tempRight %f tempLeft %f\n", tempRight, tempLeft);
		if (tempRight - tempLeft < (HALF_CAR_WIDTH * 2 + W_Group_Rig - W_Group_Lef)) {
			pNearestMotion->bInCollishionPath = 1;
			//my_printf("bin 1\n");
			//return;
		}
		else if (W_Group_Rig < W_HostCar_Lef) {
			pNearestMotion->bInCollishionPath = 2;
			//my_printf("bin 2\n");
		}
		else if (W_Group_Lef > W_HostCar_Rig) {
			pNearestMotion->bInCollishionPath = 3;
			//my_printf("bin 3\n");
		}
		else
		{
			//my_printf("bin unknow 1!\n");
		}

	}
	else
	{
		//my_printf("bin unknow 2!\n");
	}
	//my_printf("bin %d, :%d, %f ,%f, %f, %f, %f\n", pgroup->nGroupId, pNearestMotion->bInCollishionPath, W_HostCar_Lef, W_HostCar_Rig, W_Group_Lef, W_Group_Rig, pNearestMotion->z_fdis);

	//delete left & right side target
	/*if ( (pNearestMotion->bInCollishionPath != 1) && (pNearestMotion->groupRec.width << nId) > 200 )
	{
	mvClearGroupIfo(pgroup, 1);
	}*/

#if 0
	//calculate the TTC
	if (pgroup->nMotionLeng  > nLengTrd)
	{
		unsigned char tempLeng = (pgroup->nMotionLeng - 1) % nLengTrd;

		/*system Kalman filter*/
		if (tempLeng == 0)
		{
			pfarestMotion = pgroup->pMotion
				+ ((pgroup->nMotionLeng - 1 - nLengTrd) & MAX_MOTION_BIT);

#if 1
			mvSubTime(&pNearestMotion->ts, &pfarestMotion->ts, &SystimeSub);
			//fTimeSub = SystimeSub.wHour * 3600 + SystimeSub.wMin * 60 + SystimeSub.wSec
			//	+ SystimeSub.wMilSec / 1000.0f;
			fTimeSub = SystimeSub.wMilSec / 1000.0f;
			//my_printf("fTimeSub : %f\n", fTimeSub);
#else
			fTimeSub = WS_MIN(nLengTrd, pgroup->nMotionLeng) * 1.0f / FRAM_RATIO;
#endif

			WissenObjectRectTracked tempRec;
			tempRec.object.x = pNearestMotion->groupRec.object.x;
			tempRec.object.y = pNearestMotion->groupRec.object.y + pNearestMotion->groupRec.object.height;
			tempRec.object.width = pNearestMotion->groupRec.object.width;
			tempRec.object.height = pNearestMotion->groupRec.object.height;

			int Cy = LDWS_GetVanishY();
			//sysKalmanFilter(&pgroup->sysKalmState, 0.05 * nLengTrd, tempRec, (double)(Cy));
			sysKalmanFilter(&pgroup->sysKalmState, fTimeSub, tempRec, (double)(Cy));

			double zSpeed = (pgroup->sysKalmState.x[3] == 0) ? 1e-5 : pgroup->sysKalmState.x[3];
			pNearestMotion->fTTC = -pgroup->sysKalmState.x[1] / zSpeed;
		}
		else
		{
			pfarestMotion = pgroup->pMotion
				+ ((pgroup->nMotionLeng - 1 - tempLeng) & MAX_MOTION_BIT);

#if 1
			mvSubTime(&pNearestMotion->ts, &pfarestMotion->ts, &SystimeSub);
			//fTimeSub = SystimeSub.wHour * 3600 + SystimeSub.wMin * 60 + SystimeSub.wSec
			//	+ SystimeSub.wMilSec / 1000.0f;
			fTimeSub = SystimeSub.wMilSec / 1000.0f;
			//my_printf("fTimeSub : %f\n", fTimeSub);
#else
			fTimeSub = WS_MIN(nLengTrd, pgroup->nMotionLeng) * 1.0f / FRAM_RATIO;
#endif

			double zSpeed = (pgroup->sysKalmState.x[3] == 0) ? 1e-5 : pgroup->sysKalmState.x[3];
			//pNearestMotion->fTTC = -(pgroup->sysKalmState.x[1] / zSpeed + 0.05 * tempLeng);
			pNearestMotion->fTTC = -(pgroup->sysKalmState.x[1] / zSpeed + fTimeSub);
		}
	}


	float vspMean = 0, vspVar = 0;
	int i;
	for (i = 0; i < nLengTrd; i++)
	{
		int temp;
		pfarestMotion = pgroup->pMotion
			+ ((pgroup->nMotionLeng - 1 - i) & MAX_MOTION_BIT);
		vspMean += pfarestMotion->delVanish;
	}
	vspMean /= nLengTrd;

	for (i = 0; i < nLengTrd; i++)
	{
		pfarestMotion = pgroup->pMotion
			+ ((pgroup->nMotionLeng - 1 - i) & MAX_MOTION_BIT);
		vspVar += (float)WS_ABS(pfarestMotion->delVanish - vspMean);
	}
	vspVar /= nLengTrd;
	if (vspVar > 1.5f)
	{
		pNearestMotion->fTTC = TTC_MAX;
		//printf("vspVar is %f\n", vspVar);
	}

	if (pNearestMotion->z_fdis > 0)
	{
		pNearestMotion->dTTC = pNearestMotion->z_fdis / CANData->fSpeed;
	}

#else

	//calculate the TTC
	if (pgroup->nMotionLeng >= nLengTrd)//&& (pNearestMotion->groupRec.width << nId) > 48)
	{
		short vanishPty;
		pfarestMotion = pgroup->pMotion
			+ ((pgroup->nMotionLeng - nLengTrd) & MAX_MOTION_BIT);

		vanishPty = (short)((pNearestMotion->delVanish + pfarestMotion->delVanish) / 2);

		dLmda = -1;
		if (pfarestMotion->groupRec.object.y + pfarestMotion->groupRec.object.height > vanishPty &&
			pNearestMotion->groupRec.object.y + pNearestMotion->groupRec.object.height > vanishPty)
		{
			dLmda = (float)(pNearestMotion->groupRec.object.y + pNearestMotion->groupRec.object.height - vanishPty)
				/ (pfarestMotion->groupRec.object.y + pfarestMotion->groupRec.object.height - vanishPty);
		}

		wLmda = -1;
		if (pfarestMotion->groupRec.object.width != 0)
		{
			wLmda = (float)(pNearestMotion->groupRec.object.width)
				/ (pfarestMotion->groupRec.object.width);
		}

		lmda = 1;
		if (wLmda > 1.1f && dLmda > 1)
		{
			lmda = wLmda;// WS_MIN(dLmda, wLmda);
		}

#if 1
		mvSubTime(&pNearestMotion->ts, &pfarestMotion->ts, &SystimeSub);
		//fTimeSub = SystimeSub.wHour * 3600 + SystimeSub.wMin * 60 + SystimeSub.wSec
		//	+ SystimeSub.wMilSec / 1000.0f;
		fTimeSub = SystimeSub.wMilSec / 1000.0f;
		//my_printf("fTimeSub : %f\n", fTimeSub);
#else
		fTimeSub = WS_MIN(nLengTrd, pgroup->nMotionLeng)*1.0f / FRAM_RATIO;
#endif

		if (lmda != 1)
		{
			pNearestMotion->fTTC = -fTimeSub / (1 - lmda);
		}

		if (pNearestMotion->z_fdis > 0)
		{
			pNearestMotion->dTTC = pNearestMotion->z_fdis / CANData->fSpeed;
		}
	}
#endif

}

/*
Function process:
+ caculate TTC
*/
static void mvTTC(CANInfo *CANData)
{
	int i, j;
	int nObjGroupNum = 0;
	obj_group * pGroup = 0;
	obj_group * pOtherGroup = 0;
	obj_group * pObjGroup[MAX_OBJ_GROUP_NUMS] = { 0 };
	int pObjGroupScaleId[MAX_OBJ_GROUP_NUMS] = { 0 };
	Motion *pNearestMotion = 0;
	Motion *pOtherNearestMotion = 0;

	for (i = scale_shink_1_id; i <= scale_shink_4_id; i++) 
	{
		for (j = 0; j < MAX_LAYER_OBJ_GROUP_NUMS; j++) 
		{
			pGroup = m_globlparam[i].m_pGroupSets + j;

			if (-1 != pGroup->nGroupId) 
			{
				mvGroupTimeToCollison(CANData, pGroup, i);
				
				pObjGroup[nObjGroupNum] = pGroup;
				pObjGroupScaleId[nObjGroupNum] = i;
				nObjGroupNum++;
			}
		}
	}


	for (i = 0; i < nObjGroupNum - 1; i++) 
	{
		pGroup = pObjGroup[i];

		pNearestMotion = pGroup->pMotion
			+ ((pGroup->nMotionLeng - 1) & MAX_MOTION_BIT);

		if (!pNearestMotion->bInCollishionPath) 
		{
			continue;
		}

		for (j = i + 1; j < nObjGroupNum; j++) 
		{
			pOtherGroup = pObjGroup[j];
			pOtherNearestMotion = pOtherGroup->pMotion
				+ ((pOtherGroup->nMotionLeng - 1) & MAX_MOTION_BIT);

			if (pOtherNearestMotion->bInCollishionPath
				== pNearestMotion->bInCollishionPath) 
			{
				if (((pOtherGroup->rtContour.object.y + pOtherGroup->rtContour.object.height)
					<< pObjGroupScaleId[j])
		>(pGroup->rtContour.object.y + pGroup->rtContour.object.height)
		<< pObjGroupScaleId[i]) {
					pNearestMotion->bInCollishionPath = 0;

				}
				else {
					pOtherNearestMotion->bInCollishionPath = 0;
				}
			}

		}
	}

}

/*    
 Realized function:
 + Set the tracking result to g_MuliTracker
 */
static void mvSetTrackerReult(CANInfo *CANData)
{

	int i, j, nTrackNum;
	obj_group *pgroup = 0;
	Motion *pNearestMotion = 0;
	//trakobj *pobjTrack;
	float fTTC_WARNING_TIME;
	int nId;

	g_MuliTracker[g_OutIndex & 1].nTrackeNum = 0;

	for (nId = scale_shink_1_id; nId <= scale_shink_4_id; nId++) 
	{
		nTrackNum = g_MuliTracker[g_OutIndex & 1].nTrackeNum;

		if (nTrackNum >= MAX_OBJ_GROUP_NUMS) 
		{
			return;
		}

		for (i = 0; i < MAX_LAYER_OBJ_GROUP_NUMS; i++) 
		{
			pgroup = m_globlparam[nId].m_pGroupSets + i;

			if (-1 != pgroup->nGroupId) 
			{
				g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].nFramSeq =
					m_globlparam[nId].scaleInput.nFramSeq;

				if (scale_shink_1_id == nId) 
				{
					g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].objRec.object.x =
						pgroup->rtContour.object.x >> scale_shink_2_id;
					g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].objRec.object.y =
						pgroup->rtContour.object.y >> scale_shink_2_id;
					g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].objRec.object.width =
						pgroup->rtContour.object.width >> scale_shink_2_id;
					g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].objRec.object.height =
						pgroup->rtContour.object.height >> scale_shink_2_id;
					g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].objRec.object.confidence =
						pgroup->rtContour.object.confidence;

					for (j = 0;
						j
						< WS_MIN(pgroup->Centr.nTrackLen, MAX_CORNER_OF_TRACK);
					j++) {
						g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].pCenTrajecy[j].point.x =
							(int)(pgroup->Centr.pTrackPoint[j].point.x)
							>> scale_shink_2_id;
						g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].pCenTrajecy[j].point.y =
							(int)(pgroup->Centr.pTrackPoint[j].point.y)
							>> scale_shink_2_id;
					}
				}
				else if (scale_shink_2_id == nId) 
				{
					g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].objRec =
						pgroup->rtContour;
					memcpy(
						g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].pCenTrajecy,
						pgroup->Centr.pTrackPoint,
						sizeof(TrackPoint)
						* WS_MIN(pgroup->Centr.nTrackLen, MAX_CORNER_OF_TRACK));

				}
				else if (scale_shink_4_id == nId) 
				{
					g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].objRec.object.x =
						pgroup->rtContour.object.x << scale_shink_2_id;
					g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].objRec.object.y =
						pgroup->rtContour.object.y << scale_shink_2_id;
					g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].objRec.object.width =
						pgroup->rtContour.object.width << scale_shink_2_id;
					g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].objRec.object.height =
						pgroup->rtContour.object.height << scale_shink_2_id;
					g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].objRec.object.confidence =
						pgroup->rtContour.object.confidence;

					for (j = 0;
						j
						< WS_MIN(pgroup->Centr.nTrackLen, MAX_CORNER_OF_TRACK);
					j++) {
						g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].pCenTrajecy[j].point.x =
							(int)(pgroup->Centr.pTrackPoint[j].point.x)
							<< scale_shink_2_id;
						g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].pCenTrajecy[j].point.y =
							(int)(pgroup->Centr.pTrackPoint[j].point.y)
							<< scale_shink_2_id;
					}
				}

				if (m_globlparam[nId].scaleInput.nFramSeq
					== pgroup->CarBottom.nDetBottomFam) 
				{
					pgroup->CarBottom.nDistoBottom = g_MuliTracker[g_OutIndex
						& 1].pTrackerset[nTrackNum].DisToBottom =
						(pgroup->rtContour.object.y + pgroup->rtContour.object.height
						- pgroup->CarBottom.nDetYPos);
				}
				else if (!pgroup->nMotionLeng) {
					pgroup->CarBottom.nDistoBottom = 0;
				}

				g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].nTrackLen =
					pgroup->Centr.nTrackLen;
				g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].nId =
					pgroup->nGroupId;
				g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].bTrue =
					pgroup->nTruelyObj;
				g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].DisToBottom =
					(pgroup->CarBottom.nDistoBottom << nId) >> 1;
				g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].pMotion =
					pgroup->pMotion;
				g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].nMotionLeng =
					pgroup->nMotionLeng;
				pNearestMotion = pgroup->pMotion
					+ ((pgroup->nMotionLeng - 1) & MAX_MOTION_BIT);
#if 1
				fTTC_WARNING_TIME = 50.0f;
				g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].fTTC = pgroup->pMotion[(pgroup->nMotionLeng - 1) & MAX_MOTION_BIT].fTTC;
				if (WS_ABS(
					pgroup->pMotion[(pgroup->nMotionLeng - 1)
					& MAX_MOTION_BIT].fTTC) < fTTC_WARNING_TIME) {
					if (pgroup->nMotionLeng > 3
						&& (((pgroup->pMotion[(pgroup->nMotionLeng - 1)
						& MAX_MOTION_BIT].fTTC > 0)
						&& (pgroup->pMotion[(pgroup->nMotionLeng - 1)
						& MAX_MOTION_BIT].fTTC
						< fTTC_WARNING_TIME))
						|| ((pgroup->pMotion[(pgroup->nMotionLeng
						- 2) & MAX_MOTION_BIT].fTTC > 0)
						&& (pgroup->pMotion[(pgroup->nMotionLeng
						- 2) & MAX_MOTION_BIT].fTTC
						< fTTC_WARNING_TIME))
						|| (pgroup->pMotion[(pgroup->nMotionLeng - 3)
						& MAX_MOTION_BIT].fTTC > 0
						&& pgroup->pMotion[(pgroup->nMotionLeng
						- 3) & MAX_MOTION_BIT].fTTC
						< fTTC_WARNING_TIME))) {

						//TTC median filter
						if (pgroup->pMotion[(pgroup->nMotionLeng - 1) & MAX_MOTION_BIT].fTTC > WS_MAX(pgroup->pMotion[(pgroup->nMotionLeng - 2) & MAX_MOTION_BIT].fTTC, pgroup->pMotion[(pgroup->nMotionLeng - 3) & MAX_MOTION_BIT].fTTC))
						{
							g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].fTTC = WS_MAX(pgroup->pMotion[(pgroup->nMotionLeng - 2) & MAX_MOTION_BIT].fTTC, pgroup->pMotion[(pgroup->nMotionLeng - 3) & MAX_MOTION_BIT].fTTC);
						}

						if (pgroup->pMotion[(pgroup->nMotionLeng - 1) & MAX_MOTION_BIT].fTTC < WS_MIN(pgroup->pMotion[(pgroup->nMotionLeng - 2) & MAX_MOTION_BIT].fTTC, pgroup->pMotion[(pgroup->nMotionLeng - 3) & MAX_MOTION_BIT].fTTC))
						{
							g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].fTTC = WS_MIN(pgroup->pMotion[(pgroup->nMotionLeng - 2) & MAX_MOTION_BIT].fTTC, pgroup->pMotion[(pgroup->nMotionLeng - 3) & MAX_MOTION_BIT].fTTC);
						}
					}
				}
#else

				// FCW
				if (pgroup->pMotion[(pgroup->nMotionLeng - 1) & MAX_MOTION_BIT].fTTC > 0 && pgroup->pMotion[(pgroup->nMotionLeng - 1) & MAX_MOTION_BIT].fTTC < pgroup->pMotion[(pgroup->nMotionLeng - 1) & MAX_MOTION_BIT].dTTC)
					g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].fTTC = TTC_MAX;
				else
					g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].fTTC = pgroup->pMotion[(pgroup->nMotionLeng - 1) & MAX_MOTION_BIT].fTTC;
#endif

				if (CANData->fSpeed > 0 && pNearestMotion->bInCollishionPath == 1)
				{
					my_printf("ID: %d, speed: %f, limit speed: %f, distance: %f dTTC: %f fTTC: %f pNearestMotion->bInCollishionPath:%d\n", pgroup->nGroupId, pgroup->sysKalmState.x[3], URBAN_SPEED / 3.6, pNearestMotion->z_fdis, pNearestMotion->z_fdis / CANData->fSpeed, pNearestMotion->fTTC, pNearestMotion->bInCollishionPath);
				}

				// HMW
				g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].dTTC = pgroup->pMotion[(pgroup->nMotionLeng - 1) & MAX_MOTION_BIT].dTTC;

				nTrackNum++;
				if (m_globlparam[nId].pOriInPutParam->dayOrNight == 0)
				{
					if (pgroup->Centr.nTrackLen % 10 == 0)
						pgroup->rtContour.object.confidence--;
				}
				else if (m_globlparam[nId].pOriInPutParam->dayOrNight == 1)
				{
					if (pgroup->Centr.nTrackLen % 5 == 0)
						pgroup->rtContour.object.confidence *= 0.9;
				}

				if (pgroup->rtContour.object.confidence < 0)
					pgroup->rtContour.object.confidence = 0;
			}

		}

		g_MuliTracker[g_OutIndex & 1].nTrackeNum = nTrackNum;
	}
}

static void  mvTailNightDetct(const PortInput *pInPutParam, const int nId)
{
	int i, j, k, syAxis;
	obj_group *pGroup = 0;
	unsigned char* src;

	for (i = 0; i < MAX_LAYER_OBJ_GROUP_NUMS; i++)
	{
		//imgage img = { 0 };
		WissenImage img = { 0 };
		WissenObjectRectTracked obj;
		Motion *pNearestMotion = 0;

		pGroup = m_globlparam[nId].m_pGroupSets + i;
		pNearestMotion = pGroup->pMotion
			+ ((pGroup->nMotionLeng - 1) & MAX_MOTION_BIT);
		if ( -1 == pGroup->nGroupId || 1 != pGroup->nTruelyObj || pNearestMotion->bInCollishionPath != 1 ) 
		{
			continue;
		}

		img.nHig = pInPutParam->pOriGrayfram.nHig;
		img.nWid = pInPutParam->pOriGrayfram.nWid;
		img.data = pInPutParam->pOriGrayfram.data;
		//img.putr = pInPutParam->pOriGrayfram.putr;
		//img.pvtr = pInPutParam->pOriGrayfram.pvtr;

		obj.object.x = pGroup->rtContour.object.x << nId;
		obj.object.y = (pGroup->rtContour.object.y << nId);
		obj.object.width = (pGroup->rtContour.object.width << nId);
		obj.object.height = (pGroup->rtContour.object.height << nId);

		// calculate the symmetry
		src = (unsigned char*)my_malloc(obj.object.height * obj.object.width);
		k = 0;
		for (i = obj.object.y; i < obj.object.y + obj.object.height; i++)
		{
			for (j = obj.object.x; j < obj.object.x + obj.object.width; j++)
			{
				src[k] = img.pvtr[i * obj.object.width + j];
				k++;
			}
		}
		syAxis = findSymmetryAxisX(src, obj.object.width, obj.object.height);
		syAxis += obj.object.x;

		//obj.x = WS_MAX(0, obj.x - obj.width * 0.4);
		obj.object.x = obj.object.x - (short)(obj.object.width * 0.2);

		if (obj.object.x + obj.object.width + obj.object.width * 0.4 < img.nWid)
		{
			obj.object.width = obj.object.width + (short)(obj.object.width * 0.4);
		}

		// find the tail light
		computeTailLight(&img, obj, pGroup->nGroupId, &gl_tailnightpara);


#ifdef TAIL_LIGHT_DEBGU
		// show the result
		cv::Mat src0(img.nHig, img.nWid, CV_8UC1, img.ptr);
		cv::Mat dst;
		cv::cvtColor(src0, dst, CV_GRAY2BGR);
		cv::Mat darkImage = cv::Mat::zeros(img.nHig, img.nWid, CV_8UC1);
		cv::Mat tailLight = cv::Mat::zeros(img.nHig, img.nWid, CV_8UC1);

		cvtUcharToMat(gl_tailnightpara.lightImage, img.nWid, img.nHig, darkImage);
		cvtUcharToMat(gl_tailnightpara.tailLightImage, img.nWid, img.nHig, tailLight);

		cv::rectangle(dst, cv::Point((pGroup->rtContour.x << nId), pGroup->rtContour.y << nId), \
			cv::Point((pGroup->rtContour.x << nId) + (pGroup->rtContour.width << nId), (pGroup->rtContour.y << nId) + (pGroup->rtContour.width << nId)), \
			cv::Scalar(255, 0, 0), 2);


		cv::rectangle(dst, cv::Point(obj.x, obj.y), \
			cv::Point(obj.x + obj.width, obj.y + obj.height), \
			cv::Scalar(0, 255, 0), 2);


		TailLightRect temp;
		for (int i = 0; i < gl_tailnightpara.finalResult.count; i++)
		{
			temp = gl_tailnightpara.finalResult.lightRect[i];

			cv::rectangle(dst, cv::Point(temp.x, temp.y), \
				cv::Point(temp.x + temp.width, temp.y + temp.height), \
				cv::Scalar(0, 0, 255), 2);
		}
		std::string winName = std::to_string (nId);
		cv::imshow(winName, dst);
		cv::imshow("light", darkImage);
		cv::imshow("tail", tailLight);
#endif //TAIL_LIGHT_DEBGU

		// use the tailNight information to adjust the object
		if ( gl_tailnightpara.finalResult.count != 0  )
		{
			int objlx =  gl_tailnightpara.finalResult.lightRect[0].x +  gl_tailnightpara.finalResult.lightRect[0].width / 2;
			int objrx =  gl_tailnightpara.finalResult.lightRect[1].x +  gl_tailnightpara.finalResult.lightRect[1].width / 2;
			int objw = (objrx - objlx) / 8;

			if (abs((objlx + objrx) / 2 - ((pGroup->rtContour.object.x + pGroup->rtContour.object.width / 2) << nId)) < pGroup->rtContour.object.width / 8)
			{
				objlx = WS_MAX( objlx - objw, 0);
				objrx = WS_MIN( objrx + objw, img.nWid );
				objlx = objlx >> nId;
				objrx = objrx >> nId;

				pGroup->rtContour.object.x = objlx;
				pGroup->rtContour.object.y -= (objrx - objlx - pGroup->rtContour.object.width) / 2;
				pGroup->rtContour.object.width = objrx - objlx;
				pGroup->rtContour.object.height = pGroup->rtContour.object.width;
			}
		}

		my_free(src);
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
void FCW_TRACK_DetcorBytrain(VerifyObjectList verifyObjectOutput) 
{

	int i, j;
	obj_group *pgroup;
	unsigned char uInterFramTrd = 2;
	WissenObjectRectTracked RioRec;
	int nId = 0;
	for (nId = scale_shink_1_id; nId < scale_shink_4_id; nId++)
	{
		for (i = 0; i < MAX_LAYER_OBJ_GROUP_NUMS; i++)
		{
			pgroup = m_globlparam[nId].m_pGroupSets + i;

			if (Car != pgroup->ntype)
			{
				pgroup->nTruelyObj = 1;
				continue;
			}

			if (-1 != pgroup->nGroupId && !pgroup->nTruelyObj
				&& m_globlparam[nId].scaleInput.nFramSeq - pgroup->nPerLastDetFramseq > uInterFramTrd)
			{
				RioRec = pgroup->rtContour;

				for (j = 0; j < verifyObjectOutput.objectCount; j++)
				{
					if (pgroup->nGroupId == verifyObjectOutput.object[j].nGroupID)
					{
						pgroup->nDetState[pgroup->nStateNum++] = verifyObjectOutput.object[j].nObjState;
						break;
					}
				}
			}
		}
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
static void mvSureTrueGroup(const int nId) {

	int i;
	obj_group *pgroup;
	unsigned char uInterFramTrd = 2;

	for (i = 0; i < MAX_LAYER_OBJ_GROUP_NUMS; i++)
	{
		pgroup = m_globlparam[nId].m_pGroupSets + i;

		if (Car != pgroup->ntype)
		{
			pgroup->nTruelyObj = 1;
			continue;
		}

		if (-1 != pgroup->nGroupId && !pgroup->nTruelyObj
			&& m_globlparam[nId].scaleInput.nFramSeq - pgroup->nPerLastDetFramseq
	> uInterFramTrd)
		{
			int k;
			int nDetNum = 0;
			if (GROUP_DETER_STATE_NUM == pgroup->nStateNum) {
				for (k = 0; k < GROUP_DETER_STATE_NUM; k++) {

					if (pgroup->nDetState[k]) {
						nDetNum++;
					}
				}

				if (nDetNum >= GROUP_DETER_STATE_NUM * 0.6) //3
				{
					pgroup->nTruelyObj = 1;
				}
				else {
					mvClearGroupIfo(pgroup, 1);
				}
			}
			else if (3 == pgroup->nStateNum) {
				for (k = 0; k < pgroup->nStateNum; k++) {

					if (pgroup->nDetState[k]) {
						nDetNum++;
					}
				}

				if (!nDetNum) {
					mvClearGroupIfo(pgroup, 1);
				}
			}

			pgroup->nPerLastDetFramseq = m_globlparam[nId].scaleInput.nFramSeq;
		}
	}

}


/*
 Function process:
 + Get the tracking tragets for single-scale tracking; Given the value of m_globlparam[nId].scaleInput
 Fan-in :
 + mvSingledScaleTrack()
 Fan-out:
 + mvResizeImg()
 ATTENTION: __________
 */
static void mvGetScaleInPut(const PortInput *pInPutParam, const int nId) {
	int nObjNum = 0;
	int i;
	WissenImage Srcgray, Dstgray;
	int nYBoothThresh = (int)((pInPutParam->pOriGrayfram.nHig >> 1) * 0.75f);
	int nscale_shink_1_id_W = scale_shink_1_id_W;
	int nscale_shink_2_id_W = scale_shink_2_id_W;

	/* the minimum size target */
	if (nId == scale_shink_1_id) {
		m_globlparam[nId].scaleInput.pOriGrayfram = pInPutParam->pOriGrayfram;
		m_globlparam[nId].scaleInput.fzoom = 1;
		m_globlparam[nId].scaleInput.pGrayfram = pInPutParam->pOriGrayfram.data;

		for (i = 0; i < pInPutParam->nRecNum; i++) {
			if (pInPutParam->objRec[i].object.width < nscale_shink_1_id_W
				&& pInPutParam->objRec[i].object.width) {
				m_globlparam[nId].scaleInput.objRec[nObjNum] =
						pInPutParam->objRec[i];
				m_globlparam[nId].scaleInput.objRec[nObjNum].object.x =
					pInPutParam->objRec[i].object.x << 1;
				m_globlparam[nId].scaleInput.objRec[nObjNum].object.y =
					pInPutParam->objRec[i].object.y << 1;
				m_globlparam[nId].scaleInput.objRec[nObjNum].object.width =
					pInPutParam->objRec[i].object.width << 1;
				m_globlparam[nId].scaleInput.objRec[nObjNum].object.height =
					pInPutParam->objRec[i].object.height << 1;
				nObjNum++;
			}
		}
		m_globlparam[nId].scaleInput.nRecNum = nObjNum;
		m_globlparam[nId].scaleInput.nFramSeq = pInPutParam->nFramSeq;
		m_globlparam[nId].scaleInput.imgSize.width =
				pInPutParam->pOriGrayfram.nWid;
		m_globlparam[nId].scaleInput.imgSize.height =
				pInPutParam->pOriGrayfram.nHig;
		m_globlparam[nId].scaleInput.objTime = pInPutParam->objTime;
	} else if (nId == scale_shink_2_id)
	{
		m_globlparam[nId].scaleInput.pOriGrayfram = pInPutParam->pOriGrayfram;
		m_globlparam[nId].scaleInput.fzoom = 2;
		m_globlparam[nId].scaleInput.pGrayfram = pInPutParam->pGrayfram;

		for (i = 0; i < pInPutParam->nRecNum; i++) {
			if ((pInPutParam->objRec[i].object.width >= nscale_shink_1_id_W
				&& pInPutParam->objRec[i].object.width < nscale_shink_2_id_W)
				&& (pInPutParam->objRec[i].object.y + pInPutParam->objRec[i].object.height)
							< nYBoothThresh) {

				m_globlparam[nId].scaleInput.objRec[nObjNum] =
						pInPutParam->objRec[i];
				nObjNum++;
			}
		}
		m_globlparam[nId].scaleInput.nRecNum = nObjNum;
		m_globlparam[nId].scaleInput.nFramSeq = pInPutParam->nFramSeq;
		m_globlparam[nId].scaleInput.imgSize.width =
				pInPutParam->pOriGrayfram.nWid >> 1;
		m_globlparam[nId].scaleInput.imgSize.height =
				pInPutParam->pOriGrayfram.nHig >> 1;
		m_globlparam[nId].scaleInput.objTime = pInPutParam->objTime;

	} else if (nId == scale_shink_4_id) {
		Srcgray.nWid = pInPutParam->pOriGrayfram.nWid >> 1;
		Srcgray.nHig = pInPutParam->pOriGrayfram.nHig >> 1;
		Srcgray.data = pInPutParam->pGrayfram;

		Dstgray.nWid = pInPutParam->pOriGrayfram.nWid >> 2;
		Dstgray.nHig = pInPutParam->pOriGrayfram.nHig >> 2;
		Dstgray.data = m_globlparam[nId].scaleInput.pGrayfram;

		m_globlparam[nId].scaleInput.pOriGrayfram = pInPutParam->pOriGrayfram;
		m_globlparam[nId].scaleInput.fzoom = 4;

		for (i = 0; i < pInPutParam->nRecNum; i++) {
			if (pInPutParam->objRec[i].object.width >= nscale_shink_2_id_W
				|| ((pInPutParam->objRec[i].object.y
				+ pInPutParam->objRec[i].object.height) >= nYBoothThresh
				&& pInPutParam->objRec[i].object.width)) {
				m_globlparam[nId].scaleInput.objRec[nObjNum] =
						pInPutParam->objRec[i];
				m_globlparam[nId].scaleInput.objRec[nObjNum].object.x =
					pInPutParam->objRec[i].object.x >> 1;
				m_globlparam[nId].scaleInput.objRec[nObjNum].object.y =
					pInPutParam->objRec[i].object.y >> 1;
				m_globlparam[nId].scaleInput.objRec[nObjNum].object.width =
					pInPutParam->objRec[i].object.width >> 1;
				m_globlparam[nId].scaleInput.objRec[nObjNum].object.height =
					pInPutParam->objRec[i].object.height >> 1;
				nObjNum++;
			}
		}

		if (nObjNum) {
			mvResizeImg(&Srcgray, (int *)m_globlparam[nId].m_pXYCorners, (int *)m_globlparam[nId].m_pXYNoMax, &Dstgray);
		} else {
			for (i = 0; i < MAX_LAYER_OBJ_GROUP_NUMS; i++) {
				if (-1 != m_globlparam[nId].m_pGroupSets[i].nGroupId) {
					mvResizeImg(&Srcgray, (int *)m_globlparam[nId].m_pXYCorners, (int *)m_globlparam[nId].m_pXYNoMax, &Dstgray);
					break;
				}
			}
		}

		m_globlparam[nId].scaleInput.nRecNum = nObjNum;
		m_globlparam[nId].scaleInput.nFramSeq = pInPutParam->nFramSeq;
		m_globlparam[nId].scaleInput.imgSize.width = Dstgray.nWid;
		m_globlparam[nId].scaleInput.imgSize.height = Dstgray.nHig;
		m_globlparam[nId].scaleInput.objTime = pInPutParam->objTime;
	}
	m_globlparam[nId].m_pGrayData = m_globlparam[nId].scaleInput.pGrayfram;
}


/*
 Function process:
 + find m_globlparam[nId].m_pGroupSets->nGroupId != -1; change m_globlparam[nId].m_GroupIndexNum
 and m_globlparam[nId].m_pGroupIndex
 Fan-in :
 + mvSingledScaleTrack()
 Fan-out:
 + N/A
 ATTENTION: __________
 */
static void mvGetCurrenGroupIndex(const int nId) {
	int i;
	obj_group *pgroup = 0;
	m_globlparam[nId].m_GroupIndexNum = 0;

	for (i = 0; i < MAX_LAYER_OBJ_GROUP_NUMS; i++) {
		pgroup = m_globlparam[nId].m_pGroupSets + i;

		if (-1 != pgroup->nGroupId) {
			m_globlparam[nId].m_pGroupIndex[m_globlparam[nId].m_GroupIndexNum++] =
					i;
		}
	}
}

/*
 Function process:
 + Add new tracking target based on the detected result.update m_globlparam.m_PNewRec
 Fan-in :
 + mvSingledScaleTrack()
 Fan-out:
 + isRectOverlapped()
 ATTENTION: __________
 */
static void mvAddNewObjRec(const int nId) {

	int i, j;
	unsigned char bLap = 0;
	obj_group *pGroup = 0;
	WissenObjectRectTracked *tempRect;
	int nDetNum = m_globlparam[nId].scaleInput.nRecNum;
	m_globlparam[nId].m_NewRecNum = 0;

	for (i = 0; i < nDetNum; i++) 
	{
		for (j = 0; j < m_globlparam[nId].m_GroupIndexNum; j++) 
		{
			pGroup = m_globlparam[nId].m_pGroupSets
					+ m_globlparam[nId].m_pGroupIndex[j];
			tempRect = m_globlparam[nId].scaleInput.objRec + i;
			if (isRectOverlapped(&tempRect->object, &pGroup->rtContour.object, 0.7f)
				|| isRectOverlapped(&pGroup->rtContour.object, &tempRect->object, 0.7f))
			{
				bLap = 1;
				pGroup->nTruelyObj = 1;
				break;
			} else 
			{
				bLap = 0;
			}
		}

		if (!bLap) 
		{
			m_globlparam[nId].m_PNewRec[m_globlparam[nId].m_NewRecNum++] =
					m_globlparam[nId].scaleInput.objRec[i];
		}
	}
}

/*
 Function process:
 + update pgroup->PreditRec
 Fan-in :
 + mvSingledScaleTrack()
 Fan-out:
 + mvPredictTrack()
 + preditObjectRegion()
 ATTENTION: __________
 */
static void mvGoupPreditRec(const int nId) {
	int i, j;
	obj_group *pgroup = 0;
	trajecy *pTrajec = 0;
	Wissen16SPoint TopLef, BotmRig;
	float fRio = 0.45f;
	WissenObjectRect tempRect = { 0, 0, m_globlparam[nId].m_ImgWidth, m_globlparam[nId].m_ImgHeight, 0};

	if (scale_shink_2_id == nId) 
	{
		fRio = 0.4f;
	} 
	else if (scale_shink_4_id == nId) 
	{
		fRio = 0.45f;
	}

	for (i = 0; i < m_globlparam[nId].m_GroupIndexNum; i++) 
	{
		pgroup = m_globlparam[nId].m_pGroupSets
				+ m_globlparam[nId].m_pGroupIndex[i];

		TopLef.x = m_globlparam[nId].m_ImgWidth;
		TopLef.y = m_globlparam[nId].m_ImgHeight;
		BotmRig.x = 0;
		BotmRig.y = 0;

		for (j = 0; j < pgroup->ntrajecyNum; j++) 
		{
			pTrajec = pgroup->pObjtr + j;

			mvPredictTrack(pTrajec, m_globlparam[nId].m_ImgSize,
					m_globlparam[nId].scaleInput.nFramSeq);

			pTrajec->pRectPredict.object = preditObjectRegion(&pTrajec->ptPredict,
					pgroup->rtContour.object, pTrajec->nTrackLen);

			TopLef.x = WS_MIN(TopLef.x, pTrajec->pRectPredict.object.x);
			TopLef.y = WS_MIN(TopLef.y, pTrajec->pRectPredict.object.y);
			BotmRig.x =
				WS_MAX(BotmRig.x, pTrajec->pRectPredict.object.x + pTrajec->pRectPredict.object.width);
			BotmRig.y =
				WS_MAX(BotmRig.y, pTrajec->pRectPredict.object.y + pTrajec->pRectPredict.object.height);

		}

		mvPredictTrack(&pgroup->Centr, m_globlparam[nId].m_ImgSize,
   				m_globlparam[nId].scaleInput.nFramSeq);

		TopLef.x =
			(int)WS_MIN(TopLef.x, pgroup->Centr.ptPredict.x - fRio * pgroup->rtContour.object.width);
		TopLef.y =
			(int)WS_MIN(TopLef.y, pgroup->Centr.ptPredict.y - fRio * pgroup->rtContour.object.height);
		BotmRig.x =
			(int)WS_MAX(BotmRig.x, pgroup->Centr.ptPredict.x + fRio * pgroup->rtContour.object.width);
		BotmRig.y =
			(int)WS_MAX(BotmRig.y, pgroup->Centr.ptPredict.y + fRio * pgroup->rtContour.object.height);

		pgroup->PreditRec.object.x = TopLef.x;
		pgroup->PreditRec.object.y = TopLef.y;
		pgroup->PreditRec.object.width = BotmRig.x - TopLef.x;
		pgroup->PreditRec.object.height = BotmRig.y - TopLef.y;

		limitObjectRectRange(tempRect, &pgroup->PreditRec.object);

	}
}



/*
 Function process:
 + update m_globlparam[nId].m_pMask based on pInPutParam->objRec and m_globlparam[nId].m_pGroupSets
 Fan-in :
 + mvSingledScaleTrack()
 Fan-out:
 + GetVerticlBotoomLine()
 ATTENTION: __________
 */
static void mvGenerateRioGrayByMask(const int nId) {
	int i, j;
	int m;
	WissenObjectRectTracked *prec = 0;
	unsigned char *pData = 0;
	obj_group *pGroup = 0;
	int nLength;
	int nRigTrd = m_globlparam[nId].m_ImgWidth - 1;
	m_globlparam[nId].nscope_start_y = m_globlparam[nId].m_ImgHeight;
	m_globlparam[nId].nscope_end_y = 0;
	m_globlparam[nId].nscope_start_x = m_globlparam[nId].m_ImgWidth;
	m_globlparam[nId].nscope_end_x = 0;

	if (m_globlparam[nId].scaleInput.nRecNum == 0 && m_globlparam[nId].m_GroupIndexNum == 0)
		return;

	for (i = 0; i < m_globlparam[nId].scaleInput.nRecNum; i++) {
		prec = m_globlparam[nId].scaleInput.objRec + i;
		m_globlparam[nId].nscope_start_y =
			WS_MIN(prec->object.y - (prec->object.height >> 2), m_globlparam[nId].nscope_start_y);
		m_globlparam[nId].nscope_end_y =
			WS_MAX(prec->object.y + prec->object.height + (prec->object.height >> 2), m_globlparam[nId].nscope_end_y);

		m_globlparam[nId].nscope_start_x =
			WS_MIN(prec->object.x - (prec->object.width >> 2), m_globlparam[nId].nscope_start_x);
		m_globlparam[nId].nscope_end_x =
			WS_MAX(prec->object.x + prec->object.width + (prec->object.width >> 2), m_globlparam[nId].nscope_end_x);
	}

	for (i = 0; i < m_globlparam[nId].m_GroupIndexNum; i++) {
		pGroup = m_globlparam[nId].m_pGroupSets
			+ m_globlparam[nId].m_pGroupIndex[i];
		prec = &(pGroup->rtContour);

		m_globlparam[nId].nscope_start_y =
			WS_MIN(prec->object.y - (prec->object.height >> 2), m_globlparam[nId].nscope_start_y);
		m_globlparam[nId].nscope_end_y =
			WS_MAX(prec->object.y + prec->object.height + (prec->object.height >> 2), m_globlparam[nId].nscope_end_y);

		m_globlparam[nId].nscope_start_x =
			WS_MIN(prec->object.x - (prec->object.width >> 2), m_globlparam[nId].nscope_start_x);
		m_globlparam[nId].nscope_end_x =
			WS_MAX(prec->object.x + prec->object.width + (prec->object.width >> 2), m_globlparam[nId].nscope_end_x);

	}

	m_globlparam[nId].nscope_start_x =
		WS_MAX(0, m_globlparam[nId].nscope_start_x );
	m_globlparam[nId].nscope_end_x =
		WS_MIN(m_globlparam[nId].m_ImgWidth - 1,m_globlparam[nId].nscope_end_x);

	m_globlparam[nId].nscope_start_y =
		WS_MAX(0, m_globlparam[nId].nscope_start_y );
	m_globlparam[nId].nscope_end_y =
		WS_MIN(m_globlparam[nId].m_ImgHeight - 1,m_globlparam[nId].nscope_end_y);

	if ( m_globlparam[nId].nscope_end_x < m_globlparam[nId].nscope_start_x || m_globlparam[nId].nscope_end_y < m_globlparam[nId].nscope_start_y )
	{
		return;
	}

	//
	memset(
		m_globlparam[nId].m_pMask
		+ m_globlparam[nId].nscope_start_y
		* m_globlparam[nId].m_ImgWidth, 0,
		m_globlparam[nId].m_ImgWidth
		* (m_globlparam[nId].nscope_end_y
		- m_globlparam[nId].nscope_start_y + 1));

	// initialize PreditRec area value to be 255.
	if (scale_shink_4_id == nId || scale_shink_2_id == nId) {
		for (i = 0; i < m_globlparam[nId].scaleInput.nRecNum; i++) {
			prec = m_globlparam[nId].scaleInput.objRec + i;
			nLength = prec->object.width;

			for (m = WS_MAX(0, prec->object.y - (prec->object.height >> 3)); m < WS_MIN(m_globlparam[nId].m_ImgHeight - 1, prec->object.y + prec->object.height + (prec->object.height >> 3));
				m++) {
					int memSize;
					j = WS_MAX(0, ADAS_ALIGN_16BYTE_SIZE(prec->object.x - (prec->object.width >> 3) - 16));

					memSize = WS_MAX(0, WS_MIN(m_globlparam[nId].m_ImgWidth - 1, (prec->object.x + prec->object.width + (prec->object.width >> 3))) - j);
					pData = m_globlparam[nId].m_pMask
						+ m_globlparam[nId].m_ImgWidth * m + j;

					memset(pData, 255, ADAS_ALIGN_16BYTE_SIZE(memSize));
			}
		}

	} else {
		for (m = WS_MAX(0, prec->object.y - (prec->object.height >> 2)); m < WS_MIN(m_globlparam[nId].m_ImgHeight - 1, prec->object.y + prec->object.height + (prec->object.height >> 2));
			m++) {
				int memSize;
				j = WS_MAX(0, ADAS_ALIGN_16BYTE_SIZE(prec->object.x - (prec->object.width >> 2) - 16));

				memSize = WS_MAX(0, WS_MIN(m_globlparam[nId].m_ImgWidth - 1, (prec->object.x + prec->object.width + (prec->object.width >> 2))) - j);
				pData = m_globlparam[nId].m_pMask
					+ m_globlparam[nId].m_ImgWidth * m + j;

				memset(pData, 255, ADAS_ALIGN_16BYTE_SIZE(memSize));
		}
	}

	for (i = 0; i < m_globlparam[nId].m_GroupIndexNum; i++) {
		pGroup = m_globlparam[nId].m_pGroupSets
			+ m_globlparam[nId].m_pGroupIndex[i];
		prec = &(pGroup->PreditRec);

		if (scale_shink_4_id == nId || scale_shink_2_id == nId) {
			for (m = WS_MAX(0, prec->object.y - (prec->object.height >> 3)); m < WS_MIN(m_globlparam[nId].m_ImgHeight - 1, prec->object.y + prec->object.height + (prec->object.height >> 3));
				m++) {
					int memSize;
					j = WS_MAX(0, ADAS_ALIGN_16BYTE_SIZE(prec->object.x - (prec->object.width >> 3) - 16));

					memSize = WS_MAX(0, WS_MIN(m_globlparam[nId].m_ImgWidth - 1, (prec->object.x + prec->object.width + (prec->object.width >> 3))) - j);
					pData = m_globlparam[nId].m_pMask
						+ m_globlparam[nId].m_ImgWidth * m + j;

					memset(pData, 255, ADAS_ALIGN_16BYTE_SIZE(memSize));
			}
		} else {
			for (m = WS_MAX(0, prec->object.y - (prec->object.height >> 2)); m < WS_MIN(m_globlparam[nId].m_ImgHeight - 1, prec->object.y + prec->object.height + (prec->object.height >> 2));
				m++) {
					int memSize;
					j = WS_MAX(0, ADAS_ALIGN_16BYTE_SIZE(prec->object.x - (prec->object.width >> 2) - 16));

					memSize = WS_MAX(0, WS_MIN(m_globlparam[nId].m_ImgWidth - 1, (prec->object.x + prec->object.width + (prec->object.width >> 2))) - j);
					pData = m_globlparam[nId].m_pMask
						+ m_globlparam[nId].m_ImgWidth * m + j;

					memset(pData, 255, ADAS_ALIGN_16BYTE_SIZE(memSize));
			}
		}
	}
}

/*************************************************************
 Function process:
 + match the traject point with the fast points and update m_globlparam[nId].m_pGroupSets->pObjtr->pTrackPoint
 Fan-in :
 + mvSingledScaleTrack()
 Fan-out:
 + mvMatchTrackPoint()
 + mvDelUnReasonTrack()
 ATTENTION: __________
 */
static void mvTrajectorymatch(int nId) 
{
	int i, j;
	obj_group *pgroup = 0;
	trajecy *pTrajec = 0;
	TrackPoint *pMacthTrackPoint = 0;
	//trajecy *pPreMacthTrack = 0;
	AdasCorner *pMacthCorner = 0;
	int bMatch, nMathcMinDis;
	int nMatchCornIndex = -1;
	//const int nPushNum = 45;
	//TrackPoint *pCurPoin = 0;
	//TrackPoint *pPrePoin =0;

	for (i = 0; i < m_globlparam[nId].m_GroupIndexNum; i++) {
		pgroup = m_globlparam[nId].m_pGroupSets
				+ m_globlparam[nId].m_pGroupIndex[i];

#ifdef  RET_DS

		if (pgroup->nDetDiscardFam == m_globlparam[nId].scaleInput.nFramSeq - 1
			&& pgroup->nDisCardNum < 4 )
		{
			continue;
		}
#endif

		for (j = 0; j < pgroup->ntrajecyNum; j++) {

			pTrajec = pgroup->pObjtr + j;

			bMatch = mvMatchTrackPoint(pTrajec, m_globlparam[nId].m_pfeature,
					m_globlparam[nId].m_pFastCorner,
					m_globlparam[nId].m_nFastCornerNum, &nMatchCornIndex,
					&nMathcMinDis, nId);

			if (bMatch) {
				pMacthCorner = m_globlparam[nId].m_pFastCorner
						+ nMatchCornIndex;

				if (!pMacthCorner->State.nMacthNum) {
					pMacthCorner->State.nGroupIndex[0] =
							m_globlparam[nId].m_pGroupIndex[i];
					pMacthCorner->State.nTrjIndexofGroup[0] = j;
					pMacthCorner->State.nMatchDis = nMathcMinDis;
					pMacthCorner->State.nMacthNum++;
				} else {
					if (nMathcMinDis < pMacthCorner->State.nMatchDis) {
						pMacthCorner->State.nGroupIndex[0] =
								m_globlparam[nId].m_pGroupIndex[i];
						pMacthCorner->State.nTrjIndexofGroup[0] = j;
						pMacthCorner->State.nMatchDis = nMathcMinDis;

					}
				}

			}

			pMacthTrackPoint = pTrajec->pTrackPoint
					+ (pTrajec->nTrackLen & MAX_CORNER_OF_TRACK_BIT);
			pMacthTrackPoint->nFramseq = m_globlparam[nId].scaleInput.nFramSeq;
			pMacthTrackPoint->nTime = m_globlparam[nId].scaleInput.objTime;
			pMacthTrackPoint->nMatchStatus = 0;
			pMacthTrackPoint->point = pTrajec->ptPredict;

			pTrajec->nTrackLen++;
			pTrajec->nEstTimes++;

		}

	}

	for (i = 0; i < m_globlparam[nId].m_nFastCornerNum; i++) {
		pMacthCorner = m_globlparam[nId].m_pFastCorner + i;

		if (1 == pMacthCorner->State.nMacthNum) {
			pgroup = m_globlparam[nId].m_pGroupSets
					+ pMacthCorner->State.nGroupIndex[0];
#ifdef  RET_DS
			if (pgroup->nDetDiscardFam == m_globlparam[nId].scaleInput.nFramSeq - 1
				&& pgroup->nDisCardNum < 4 )
			{
				continue;
			}
#endif


			pTrajec = pgroup->pObjtr + pMacthCorner->State.nTrjIndexofGroup[0];

			pMacthTrackPoint = pTrajec->pTrackPoint
					+ ((pTrajec->nTrackLen - 1) & MAX_CORNER_OF_TRACK_BIT);
			pMacthTrackPoint->nMatchStatus = 1;
			pMacthTrackPoint->point = pMacthCorner->point;
			pTrajec->nEstTimes = 0;
			//memcpy(pTrajec->pfeature,m_globlparam.m_pfeature + SURF_DESC_DIMENTION * i,SURF_DESC_DIMENTION);
			memcpy(pTrajec->pfeature, m_globlparam[nId].m_pfeature + (i << 6),
					SURF_DESC_DIMENTION);

		}
	}

	for (i = 0; i < m_globlparam[nId].m_GroupIndexNum; i++) {
		pgroup = m_globlparam[nId].m_pGroupSets
				+ m_globlparam[nId].m_pGroupIndex[i];

		mvDelUnReasonTrack(pgroup);

	}
}

/*
 Function process:
 + Get the car temple img based on ROI region of RioRec
 Fan-in :
 + mvMatchDetRec()
 Fan-out:
 + N/A
 ATTENTION: __________
 */
static void mvSetUpdataCarImg(WissenObjectRectTracked RioRec, WissenImage *pCarimg, int nId) 
{
	int i, j;
	//unsigned char *pDstr = 0;
	//unsigned char *pSrctr = 0;
	float fx_zoom, fy_zoom;
	unsigned char *pDst = 0;
	int nWidth = m_globlparam[nId].scaleInput.pOriGrayfram.nWid >> nId;

	fx_zoom = ((float)RioRec.object.width) / UPDATA_IMAGE_SIZE;
	fy_zoom = ((float)RioRec.object.height) / UPDATA_IMAGE_SIZE;

	for (j = 0; j < UPDATA_IMAGE_SIZE; j++) {
		pDst = pCarimg->data + j * UPDATA_IMAGE_SIZE;

		for (i = 0; i < UPDATA_IMAGE_SIZE; i++) {
			pDst[i] = *(m_globlparam[nId].scaleInput.pGrayfram
				+ ((int)(j * fy_zoom) + RioRec.object.y) * nWidth
				+ ((int)(i * fx_zoom) + RioRec.object.x));
		}
	}
}


/*
 Function process:
 + match the detect region in pGroup if matched return 1; else return 0
 Fan-in :
 + mvUpdataByDetctor()
 Fan-out:
 + mvSetUpdataCarImg()
 + mvTemplatMatch()
 ATTENTION: __________
 */
static int mvMatchDetRec(const obj_group *pGroup, WissenObjectRectTracked *pLapCarRec, int nId) 
{

	int i, j;
	int nMatchIndex = -1;
	WissenObjectRectTracked *pCarRec;
	WissenObjectRectTracked GroupRec, CarRec;
	int nDisTance;
	PortInput *oriPut = m_globlparam[nId].pOriInPutParam; //d都与1/4压缩图像上信息进行融合
	WissenImage Carimg;
	float fMatchScore;
	float fAvgScore = 0.0f;
	WissenObjectRectTracked MatchRec;
	int noff = UPDATA_IMAGE_SIZE * UPDATA_IMAGE_SIZE;
	WissenObjectRectTracked tempRect = { 0 };
	tempRect.object.x = 0;
	tempRect.object.y = 0;
	tempRect.object.width = UPDATA_IMAGE_SIZE;
	tempRect.object.height = UPDATA_IMAGE_SIZE;

	Carimg.nWid = UPDATA_IMAGE_SIZE;
	Carimg.nHig = UPDATA_IMAGE_SIZE;
	Carimg.data = (unsigned char*) m_globlparam[nId].m_PublacSape;

	if (scale_shink_1_id == nId)
	{
		for (i = 0; i < oriPut->nRecNum; i++) 
		{
			pCarRec = oriPut->objRec + i;
			if (pCarRec->object.width == 0)
				continue;

			CarRec.object.x = pCarRec->object.x << 1;
			CarRec.object.y = pCarRec->object.y << 1;
			CarRec.object.width = pCarRec->object.width << 1;
			CarRec.object.height = pCarRec->object.height << 1;
			CarRec.object.confidence = pCarRec->object.confidence;
			pCarRec = &CarRec;
			GroupRec = (pGroup->rtContour);

			nDisTance = WS_ABS(GroupRec.object.x + (GroupRec.object.width >> 1) - \
				pCarRec->object.x - (pCarRec->object.width >> 1)) + \
				WS_ABS(GroupRec.object.y + (GroupRec.object.height >> 1) - \
				pCarRec->object.y - (pCarRec->object.height >> 1));

			if ((isRectOverlapped(&pCarRec->object, &GroupRec.object, 0.7f)
				|| isRectOverlapped(&GroupRec.object, &pCarRec->object, 0.7f))
				&& pCarRec->object.confidence > GroupRec.object.confidence)
			{
				if (nDisTance < 0.4f * GroupRec.object.width && (pCarRec->object.width < 1.2f * GroupRec.object.width &&  pCarRec->object.width > 0.8f * GroupRec.object.width))
				{
					*pLapCarRec = CarRec;
					nMatchIndex = i;
					break;
				}
				else
				{
					mvSetUpdataCarImg(*pCarRec, &Carimg, nId);
					for (j = 0; j < WS_MIN(pGroup->nUpdataIndex, UPDATA_IMAGE_NUM); j++) 
					{

						mvTemplatMatch(pGroup->UpdatedImg[j], Carimg,
							tempRect, &MatchRec, noff, 0,
							m_globlparam[nId].m_PublacSape, &fMatchScore);
						fAvgScore += fMatchScore;
					}
					fAvgScore /= 
						WS_MIN(pGroup->nUpdataIndex,UPDATA_IMAGE_NUM);

					if (fAvgScore > 0.5f)
					{
						*pLapCarRec = CarRec;
						nMatchIndex = i;
						break;
					}

				}
			}
		}
	}
	else if (scale_shink_2_id == nId)
	{
		for (i = 0; i < oriPut->nRecNum; i++)
		{
			pCarRec = oriPut->objRec + i;
			if (pCarRec->object.width == 0)
				continue;
			GroupRec = (pGroup->rtContour);

			nDisTance = WS_ABS(GroupRec.object.x + (GroupRec.object.width>>1) - \
				pCarRec->object.x - (pCarRec->object.width>>1) ) +\
				WS_ABS(GroupRec.object.y + (GroupRec.object.height>>1) - \
				pCarRec->object.y - (pCarRec->object.height>>1) );

			if ((isRectOverlapped(&pCarRec->object, &GroupRec.object, 0.7f)
				|| isRectOverlapped(&GroupRec.object, &pCarRec->object, 0.7f))
				&& pCarRec->object.confidence > GroupRec.object.confidence)
			{

				if (nDisTance < 0.4f * GroupRec.object.width && (pCarRec->object.width < 1.2f * GroupRec.object.width &&  pCarRec->object.width > 0.8f * GroupRec.object.width))
				{
					*pLapCarRec = *pCarRec;
					nMatchIndex = i;
					break;
				}
				else
				{
					mvSetUpdataCarImg(*pCarRec, &Carimg, nId);
					for (j = 0; j < WS_MIN(pGroup->nUpdataIndex, UPDATA_IMAGE_NUM); j++) 
					{
						mvTemplatMatch(pGroup->UpdatedImg[j], Carimg,
							tempRect, &MatchRec, noff, 0,
							m_globlparam[nId].m_PublacSape, &fMatchScore);
						fAvgScore += fMatchScore;
					}
					fAvgScore /= 
						WS_MIN(pGroup->nUpdataIndex,UPDATA_IMAGE_NUM);

					if (fAvgScore > 0.5f)
					{
						*pLapCarRec = *pCarRec;
						nMatchIndex = i;
						break;
					}

				}
			}
		}
	} 
	else if (scale_shink_4_id == nId) 
	{
		for (i = 0; i < oriPut->nRecNum; i++)
		{
			pCarRec = oriPut->objRec + i;
			if (pCarRec->object.width == 0)
				continue;
			CarRec.object.x = pCarRec->object.x >> 1;
			CarRec.object.y = pCarRec->object.y >> 1;
			CarRec.object.width = pCarRec->object.width >> 1;
			CarRec.object.height = pCarRec->object.height >> 1;
			CarRec.object.confidence = pCarRec->object.confidence;
			pCarRec = &CarRec;
			GroupRec = (pGroup->rtContour);

			nDisTance = WS_ABS(GroupRec.object.x + (GroupRec.object.width>>1) - \
				pCarRec->object.x - (pCarRec->object.width>>1) ) +\
				WS_ABS(GroupRec.object.y + (GroupRec.object.height>>1) - \
				pCarRec->object.y - (pCarRec->object.height>>1) );

			if ((isRectOverlapped(&pCarRec->object, &GroupRec.object, 0.7f)
				|| isRectOverlapped(&GroupRec.object, &pCarRec->object, 0.7f))
				&& pCarRec->object.confidence > GroupRec.object.confidence)
			{

				if ( nDisTance < 0.4f * GroupRec.object.width  && ( pCarRec->object.width < 1.2f * GroupRec.object.width &&  pCarRec->object.width > 0.8f * GroupRec.object.width ) )
				{
					*pLapCarRec = CarRec;
					nMatchIndex = i;
					break;
				}
				else
				{
					mvSetUpdataCarImg(*pCarRec, &Carimg, nId);
					for (j = 0; j < WS_MIN(pGroup->nUpdataIndex, UPDATA_IMAGE_NUM); j++) 
					{
						mvTemplatMatch(pGroup->UpdatedImg[j], Carimg,
							tempRect, &MatchRec, noff, 0,
							m_globlparam[nId].m_PublacSape, &fMatchScore);
						fAvgScore += fMatchScore;
					}
					fAvgScore /= 
						WS_MIN(pGroup->nUpdataIndex,UPDATA_IMAGE_NUM);

					if (fAvgScore > 0.5f)
					{
						*pLapCarRec = CarRec;
						nMatchIndex = i;
						break;
					}

				}
			}
		}
	}

	return nMatchIndex;
}

/*
 Function process:
 + update the temple of target(pGroup->UpdatedImg) based on the detected region
 Fan-in :
 + mvUpdataByDetctor()
 Fan-out:
 + N/A
 ATTENTION: __________
 */
static void mvUpdataNormalCarImg(obj_group * pGroup, WissenObjectRectTracked RioRec, int nId) {
	int i, j;
	//unsigned char *pDstr = 0;
	//unsigned char *pSrctr = 0;
	float fx_zoom, fy_zoom;
	WissenImage *pupdataimage;
	unsigned char *pDst = 0;
	int nWidth = m_globlparam[nId].scaleInput.pOriGrayfram.nWid >> nId;
	pupdataimage = pGroup->UpdatedImg + pGroup->nUpdataIndex % UPDATA_IMAGE_NUM;

	fx_zoom = ((float) RioRec.object.width) / UPDATA_IMAGE_SIZE;
	fy_zoom = ((float) RioRec.object.height) / UPDATA_IMAGE_SIZE;

	for (j = 0; j < UPDATA_IMAGE_SIZE; j++) {
		pDst = pupdataimage->data + j * UPDATA_IMAGE_SIZE;

		for (i = 0; i < UPDATA_IMAGE_SIZE; i++) {
			pDst[i] = *(m_globlparam[nId].scaleInput.pGrayfram
					+ ((int) (j * fy_zoom) + RioRec.object.y) * nWidth
					+ ((int) (i * fx_zoom) + RioRec.object.x));
		}
	}

	pGroup->nUpdataIndex++;
}

/*
 Function process:
 + Update pGroup by detected result
 Fan-in :
 + mvUpdataByDetctor()
 Fan-out:
 + mvSetOriInitGray()
 + mvDelUnReasonTrack()
 + mvAddTrajecToGroup()
 + mvEstablInitVote()
 + mvSelcImg()
 ATTENTION: __________
 */
static unsigned char mvUpdataByDetctor(obj_group *pgroup, int nId) {
	//size_t  SmallObjTrd = 32;
	WissenObjectRectTracked ShrinkTempRec;
	int nMatchIndex;
	unsigned char bUpadata = 0;
	WissenObjectRectTracked Rec;
	WissenObjectRect tempRect = { 0, 0, m_globlparam[nId].m_ImgWidth, m_globlparam[nId].m_ImgHeight };
	
	float fzoom = (float) GROUP_TEMPLATE_SHINK_RATE;

	if (Car != pgroup->ntype) {
		return 0;
	}

	nMatchIndex = mvMatchDetRec(pgroup, &Rec, nId);

	if (pgroup->nTruelyObj && -1 != nMatchIndex) 
	{

		pgroup->nLastupdataCarWidth = Rec.object.width;

		mvUpdataNormalCarImg(pgroup, Rec, nId);

		// if find object overlap with the target which has been verified, so update the rect.
		limitObjectRectRange(tempRect, &Rec.object);


		pgroup->rtContour = Rec;
		pgroup->InitContour = Rec;

		mvDelUnReasonTrack(pgroup);

		mvAddTrajecToGroup(pgroup, nId);

		mvEstablInitVote(pgroup, 0, nId, m_globlparam[nId].scaleInput.nFramSeq);

		pgroup->OriInitContour = Rec;
		pgroup->histoyRec.nSizNum = 0;
		pgroup->histoyRec.pGroupSiz[0] = Rec.object.width;
		pgroup->histoyRec.pGroupSiz[MAX_HISRORY_GROUP_REC_NUM] = Rec.object.height;
		pgroup->histoyRec.nSizNum++;
		pgroup->updataFrambyCar = m_globlparam[nId].scaleInput.nFramSeq;
		pgroup->SerpreditNum = 0;
		pgroup->nMinCarWid = WS_MIN(pgroup->nMinCarWid,pgroup->rtContour.object.width);
		pgroup->CarBottom.nDetBottomNum = 0;

		mvSetOriInitGray(pgroup, nId);

		if (WS_MAX(pgroup->rtContour.object.width,pgroup->rtContour.object.height)
			< MAX_TEMPLAT_NCC_TRACK_SIZE) {
				mvShinkRect(pgroup->rtContour.object, &ShrinkTempRec.object, fzoom);

				mvSelcImg(&pgroup->Templat, ShrinkTempRec, 1, nId);
		}

		mvUpdataGroupCenTraj(pgroup);

		if (WS_MAX(pgroup->rtContour.object.width,pgroup->rtContour.object.height)
				< MAX_TEMPLAT_NCC_TRACK_SIZE) {

			mvShinkRect(pgroup->rtContour.object, &ShrinkTempRec.object, fzoom);

			mvSelcImg(&pgroup->Templat, ShrinkTempRec, 1, nId);

#ifdef SHOW_TEMPLE
			IplImage *TemplateImg = Trans_Imgage_TO_cvIplImage(pgroup->Templat);
			cvShowImage("MAKE_TemplateImg",TemplateImg);
			cvWaitKey(0);
			cvReleaseImage(&TemplateImg);
#endif

		}

		bUpadata = 1;
	}

	return bUpadata;
}

/*
 Function process:
 + decide if there is overlapped in differnet scales
 Fan-in :
 + mvPreditGroup()
 Fan-out:
 + computeRectOverlappedRegion()
 ATTENTION: __________
 */
static unsigned char mvOccludedbyGroup(unsigned char SelfGroupIndex, WissenObjectRectTracked *OccluedRec,
		int *pFindInxdex, int nId) {
	int i;
	int nIndex;
	unsigned char bFilte = 0;
	obj_group *pDstGroup = m_globlparam[nId].m_pGroupSets + SelfGroupIndex;
	obj_group *pGroup = 0;

	for (i = 0; i < m_globlparam[nId].m_GroupIndexNum; i++) {
		nIndex = m_globlparam[nId].m_pGroupIndex[i];

		if (SelfGroupIndex != nIndex) {
			pGroup = m_globlparam[nId].m_pGroupSets + nIndex;

			bFilte = (isRectOverlapped(&pGroup->rtContour.object, &pDstGroup->rtContour.object, 0.7f)
				|| isRectOverlapped(&pDstGroup->rtContour.object, &pGroup->rtContour.object, 0.7f));

			if (pGroup->nTruelyObj
				&& bFilte ) {
				computeRectOverlappedRegion(&pGroup->rtContour.object, &pDstGroup->rtContour.object, &OccluedRec->object);
				*pFindInxdex = nIndex;
				return 1;
			}
		}
	}

	return 0;
}

/*
 Function process:
 + do the temple matching in pgroup->rtContour by pgroup->Templat
 Fan-in :
 + mvPreditGroup()
 Fan-out:
 + mvTemplatMatch()
 ATTENTION: __________
 */
static unsigned char mvMatchByGroupTemplate(obj_group *pgroup,
	WissenObjectRectTracked *MatchResultRec, int nId, float *pfMatchScore) {
	WissenImage Curnimg;
	int nAddress0ff = 0;
	unsigned char *ptr = 0;
	WissenObjectRectTracked SearcRect;
	unsigned char bTempltMatch = 0;
	WissenObjectRect tempRect = { 0, 0, m_globlparam[nId].m_ImgWidth, m_globlparam[nId].m_ImgHeight };
	WissenObjectRectTracked tempRect1 = { 0 };

	nAddress0ff = 0;
	ptr = (unsigned char *) m_globlparam[nId].m_PublacSape;

	SearcRect.object.width = (int) (pgroup->rtContour.object.width * 1.5f);
	SearcRect.object.height = (int) (pgroup->rtContour.object.height * 1.5f);
	SearcRect.object.x = pgroup->rtContour.object.x
			- ((SearcRect.object.width - pgroup->rtContour.object.width) >> 1);
	SearcRect.object.y = pgroup->rtContour.object.y
			- ((SearcRect.object.height - pgroup->rtContour.object.height) >> 1);
	limitObjectRectRange(tempRect, &SearcRect.object);

	Curnimg.data = ptr;
	Curnimg.nWid = SearcRect.object.width;
	Curnimg.nHig = SearcRect.object.height;
	nAddress0ff += ADAS_ALIGN_16BYTE_SIZE( Curnimg.nWid * Curnimg.nHig);
	ptr = (unsigned char *) m_globlparam[nId].m_PublacSape + nAddress0ff;

	mvSelcImg(&Curnimg, SearcRect, 1, nId);

	tempRect1.object.x = 0;
	tempRect1.object.y = 0;
	tempRect1.object.width = Curnimg.nWid;
	tempRect1.object.height = Curnimg.nHig;
	bTempltMatch = mvTemplatMatch(Curnimg, pgroup->Templat,
		tempRect1, MatchResultRec,
		nAddress0ff, 0, m_globlparam[nId].m_PublacSape, pfMatchScore);

	MatchResultRec->object.x -= (int) (MatchResultRec->object.width
			* GROUP_TEMPLATE_SHINK_RATE);
	MatchResultRec->object.y -= (int) (MatchResultRec->object.height
			* GROUP_TEMPLATE_SHINK_RATE);
	MatchResultRec->object.width += (int) ((MatchResultRec->object.width << 1)
			* GROUP_TEMPLATE_SHINK_RATE);
	MatchResultRec->object.height += (int) ((MatchResultRec->object.height << 1)
			* GROUP_TEMPLATE_SHINK_RATE);

	MatchResultRec->object.x += SearcRect.object.x;
	MatchResultRec->object.y += SearcRect.object.y;

	if (bTempltMatch) 
	{
		limitObjectRectRange(tempRect, &MatchResultRec->object);
	}

	MatchResultRec->object.confidence = pgroup->rtContour.object.confidence;

	return bTempltMatch;

}

/*
 Function process:
 + Do the midle filter for pGroup->rtContour based on pGroup->histoyRec
 Fan-in :
 + mvPreditGroup()
 Fan-out:
 + mvMidGroupRec()
 ATTENTION: __________
 */
static void mvGroupFilter(obj_group *pgroup, unsigned char bUpdata, int nId) {
	//int nBottomWith;
	WissenObjectRect tempRect = { 0, 0, m_globlparam[nId].m_ImgWidth, m_globlparam[nId].m_ImgHeight };

	if (!bUpdata) {

#ifdef RET_DS
	
		pgroup->nDetDiscardFam =  m_globlparam[nId].scaleInput.nFramSeq;
		pgroup->nDisCardNum++;

#else
       mvClearGroupIfo(pgroup, 1);
#endif

	} else {
		pgroup->nDisCardNum = 0;

		if ((pgroup->rtContour.object.x < 5
				&& pgroup->rtContour.object.y + pgroup->rtContour.object.height + 5
						> m_globlparam[nId].m_ImgHeight)
				|| (pgroup->rtContour.object.width > m_globlparam[nId].m_ImgWidth / 2
						|| pgroup->rtContour.object.width < 5)
				/*|| ( pgroup->rtContour.x > 30 && pgroup->rtContour.x +  pgroup->rtContour.width  < m_globlparam[nId].m_ImgWidth - 30
				 && pgroup->rtContour.y + pgroup->rtContour.height + 10 > m_globlparam[nId].m_ImgHeight)*/) {
#ifdef RET_DS

					 pgroup->nDetDiscardFam =  m_globlparam[nId].scaleInput.nFramSeq;
					 pgroup->nDisCardNum++;

#else
					 mvClearGroupIfo(pgroup, 1);
#endif
		} else {

			pgroup->nDisCardNum = 0;
			pgroup->nFramseq = m_globlparam[nId].scaleInput.nFramSeq;
			pgroup->nTime = m_globlparam[nId].scaleInput.objTime;

			pgroup->histoyRec.pGroupSiz[pgroup->histoyRec.nSizNum
					& MAX_HISRORY_GROUP_REC_NUM_BIT] = pgroup->rtContour.object.width;
			pgroup->histoyRec.pGroupSiz[(pgroup->histoyRec.nSizNum
					& MAX_HISRORY_GROUP_REC_NUM_BIT) + MAX_HISRORY_GROUP_REC_NUM] =
					pgroup->rtContour.object.height;
			pgroup->histoyRec.nSizNum++;

			mvMidGroupRec(pgroup);

			limitObjectRectRange(tempRect, &pgroup->rtContour.object);

			mvUpdataGroupCenTraj(pgroup);

		}

	}
}

/*
 *************************************************************
 Function process:
 + filter out the traject that has large center vote bise
 Fan-in :
 + mvInitConsensVote()
 Fan-out:
 + N/A
 ATTENTION: __________
 */
static int mvDelFarCen(obj_group *pGroup, Wissen16SPoint MidVoteCen,
		int * pVoteTrajecIndex, int nVoteNum, int nDisTrd, int nId) {
	int i;
	WissenPoint votePoin;

	trajecy *ptrajecy = 0;
	int nDis = nDisTrd;
	int nsimilary = 0;
	int votDis;

	for (i = 0; i < nVoteNum; i++) 
	{
		votePoin.x = m_globlparam[nId].m_ndx[i];
		votePoin.y = m_globlparam[nId].m_ndy[i];
		
		votDis = mvDisPow(&votePoin, &MidVoteCen);

		if (votDis > nDis) {
			ptrajecy = pGroup->pObjtr + pVoteTrajecIndex[i];

			ptrajecy->nEstTimes = 10;
		} else if (votDis < 25) {

			nsimilary++;
		}

	}


	return nsimilary;
}

/*
 *************************************************************
 Function process:
 + caculate group sacles and location based on pgroup->InitContour and pTrajecy->InitPoint, update pgroup->rtContour
 Fan-in :
 + mvPreditGroup()
 Fan-out:
 + mvScaleVal()
 + mvTrajecyVote()
 + mvDelFarCen()
 ATTENTION: __________
 */
static unsigned char mvInitConsensVote(obj_group *pgroup, int nId) {
	unsigned char bfsalcSucces;
	float fscale;
	int j;
	trajecy *pTrajecy = 0;
//	Wissen16SPoint CenVote,AvgVotec;
	Wissen16SPoint CenVote;
	int nCenVotNum = 0;
	int nSimlarVote = 0;
	int nVoteMidx, nVoteMidy;
	Wissen16SPoint Votec;
	int nVotTrd = VOTE_CONSENSE_TRD;
	int *ptr = m_globlparam[nId].m_PublacSape;

	bfsalcSucces = mvScaleVal(pgroup, m_globlparam[nId].m_fsclare, &fscale,
			InitVote, nId);

	//if( fscale > 1.15f ||  fscale < 0.85f)
	if (fscale > 1.2f || fscale < 0.8f) {
		//my_printf("pgroupID:%d Scale change too mutch!\n", pgroup->nGroupId);
		return 0;
	}

	if (bfsalcSucces) {
		memset(m_globlparam[nId].m_PublacSape, 0,
				sizeof(int) * pgroup->ntrajecyNum);

		for (j = 0; j < pgroup->ntrajecyNum; j++) {
			pTrajecy = pgroup->pObjtr + j;

			if (pTrajecy->bInitTracked && !pTrajecy->nEstTimes) {

				CenVote = mvTrajecyVote(pTrajecy, fscale, InitVote);
				m_globlparam[nId].m_ndx[nCenVotNum] = CenVote.x;
				m_globlparam[nId].m_ndy[nCenVotNum] = CenVote.y;
				m_globlparam[nId].m_PublacSape[nCenVotNum] = j;
				nCenVotNum++;
			}
		}

		memcpy(ptr + nCenVotNum, m_globlparam[nId].m_ndx,
				sizeof(int) * nCenVotNum);
		memcpy(ptr + (nCenVotNum << 1), m_globlparam[nId].m_ndy,
				sizeof(int) * nCenVotNum);

		binSort_INT(ptr + nCenVotNum, nCenVotNum);
		binSort_INT(ptr + (nCenVotNum << 1), nCenVotNum);

		Votec.x = ptr[nCenVotNum + (nCenVotNum >> 1)]; //nCenVotNum + (nCenVotNum>>1)
		Votec.y = ptr[(nCenVotNum << 1) + (nCenVotNum >> 1)]; //
		nVoteMidx = Votec.x;
		nVoteMidy = Votec.y;

		nSimlarVote = mvDelFarCen(pgroup, Votec, m_globlparam[nId].m_PublacSape,
				nCenVotNum,
				pgroup->rtContour.object.width * pgroup->rtContour.object.width / 25, nId);

		if (nSimlarVote < nVotTrd) {
			return 0;
		}

		pgroup->rtContour.object.width = (int) (pgroup->InitContour.object.width * fscale);
		pgroup->rtContour.object.height = (int) (pgroup->InitContour.object.height * fscale);
		pgroup->rtContour.object.x = nVoteMidx - (pgroup->rtContour.object.width >> 1);
		pgroup->rtContour.object.y = nVoteMidy - (pgroup->rtContour.object.height >> 1);

	}

	return nSimlarVote;
}

/*
 Function process:
 + update pGroup->ProcessContour，pGroup->nPreProssVotFram，pTrajecy->bProcessTracked，pTrajecy->ProcessVote，pTrajecy->processfeature
 Fan-in :
 + mvPreditGroup()
 Fan-out:
 + isPointInRect()
 + mvProcessVote()
 ATTENTION: __________
 */
static void mvUpdataProcessVote(obj_group *pGroup, int nId) 
{
	int m;
	trajecy *pTrajecy = 0;
	TrackPoint *Trackpin = 0;
	WissenObjectRectTracked shrinkRec;
	//const unsigned char DispoweMovTrd = 25;
	//unsigned char  bFieled = 0;

	if (m_globlparam[nId].scaleInput.nFramSeq - pGroup->nPreProssVotFram < 3
			&& pGroup->nPreProssVotFram) {
		return;
	}

	//if(pGroup->nGroupId ==3)
	//my_printf("----------------------------------------------mvUpdataProcessVote:x:%d,%d\n",pGroup->ProcessContour.x,pGroup->ProcessContour.y);

	pGroup->ProcessContour = pGroup->rtContour;
	shrinkRec.object.x = pGroup->ProcessContour.object.x
			+ (int) (pGroup->ProcessContour.object.width * 0.15f);
	shrinkRec.object.y = pGroup->ProcessContour.object.y
			+ (int) (pGroup->ProcessContour.object.height * 0.15f);
	shrinkRec.object.width = (int) (pGroup->ProcessContour.object.width * 0.7f);
	shrinkRec.object.height = (int) (pGroup->ProcessContour.object.height * 0.7f);

	pGroup->nPreProssVotFram = m_globlparam[nId].scaleInput.nFramSeq;

	for (m = 0; m < pGroup->ntrajecyNum; m++) {
		pTrajecy = pGroup->pObjtr + m;
		Trackpin = pTrajecy->pTrackPoint
				+ ((pTrajecy->nTrackLen - 1) & MAX_CORNER_OF_TRACK_BIT);

		if (!isPointInRect(&Trackpin->point, &shrinkRec.object)) {
			pTrajecy->bProcessTracked = 0;

		} else {
			pTrajecy->bProcessTracked = 1;
			mvProcessVote(pTrajecy, pGroup->ProcessContour);
			memcpy(pTrajecy->processfeature, pTrajecy->pfeature,
					SURF_DESC_DIMENTION);
		}
	}

}

/*
 *************************************************************
 Function process:
 + copy and resize the image of pgroup->OriInitContour to pResImg, pResImg has the same size of pgroup->rtContour
 Fan-in :
 + mvTempleMatchByOri()
 Fan-out:
 + N/A
 ATTENTION: __________
 */
static void mvOriObjResizeToDstImg(obj_group *pgroup, unsigned char *pResImg, int nId) 
{

	int i, j;
	float fScale;
	float fSum;
	unsigned char *Ptr = 0;
	unsigned char *PSrc = 0;
	int width, height;
	int *arr_y = 0;
	int *arr_x = 0;

	width = pgroup->rtContour.object.width;
	height = pgroup->rtContour.object.height;

	arr_y = (int *) m_globlparam[nId].m_pXYCorners;
	arr_x = (int *) m_globlparam[nId].m_pXYNoMax;

	fScale = pgroup->OriInitContour.object.width / (width + 0.001f);

	fSum = -fScale;
	for (j = 0; j < height; j++) {
		fSum += fScale;

		arr_y[j] = (int) (fSum + 0.5);

		if (j < width) {
			arr_x[j] = arr_y[j];
		}
	}

	if (width > height) {
		for (i = height; i < width; i++) {
			fSum += fScale;

			arr_x[i] = (int) (fSum + 0.5);
		}
	}

	for (j = 0; j < height; j++) {
		Ptr = pResImg + width * j;

		PSrc = pgroup->pOrInitGray + pgroup->OriInitContour.object.width * arr_y[j];

		for (i = 0; i < width; i++) {
			Ptr[i] = PSrc[arr_x[i]];
		}
	}
}

/*
 Function process:
 + Do the temple matching for group if vote tarcking failed
 Fan-in :
 + mvPreditGroup()
 Fan-out:
 + mvOriObjResizeToDstImg()
 + mvSelcTemplatByCen()
 + mvTemplatMatch()
 ATTENTION: __________
 */
static unsigned char mvTempleMatchByOri(obj_group *pgroup, WissenObjectRectTracked *pMatchRec,
		int nId, float *fMatchScore) {
	//int i;
	int nAddress0ff = 0;
	WissenImage ResizeOrimg;
	WissenImage Tempimg, Curnimg;
	WissenObjectRectTracked SearcRe, MatchRec, TempRec;
	float fShunkRio;
	unsigned char bMatchSucces;
	//Wissen16SPoint VoteCorreVal;
	//trajecy *pTrajecy = 0;

	unsigned char * ptr = (unsigned char *) m_globlparam[nId].m_PublacSape;
	ResizeOrimg.data = ptr;
	ResizeOrimg.nWid = pgroup->rtContour.object.width;
	ResizeOrimg.nHig = pgroup->rtContour.object.height;

	if (WS_MAX(ResizeOrimg.nWid ,ResizeOrimg.nHig) > MAX_TEMPLAT_TRACK_SIZE
			|| (!pgroup->InitContour.object.width)
			|| pgroup->rtContour.object.width * pgroup->rtContour.object.height
					>= MAX_GROUP_IMG_BYTE) {
		return 0;
	}

	mvOriObjResizeToDstImg(pgroup, ResizeOrimg.data, nId);

	nAddress0ff += ADAS_ALIGN_16BYTE_SIZE( ResizeOrimg.nWid * ResizeOrimg.nHig);
	ptr = (unsigned char *) m_globlparam[nId].m_PublacSape + nAddress0ff;

	Tempimg.data = ptr;
	mvSelcTemplatByCen(ResizeOrimg, &Tempimg, &TempRec.object);
	nAddress0ff += ADAS_ALIGN_16BYTE_SIZE( Tempimg.nWid * Tempimg.nHig);
	ptr = (unsigned char *) m_globlparam[nId].m_PublacSape + nAddress0ff;

	Curnimg.data = ptr;
	Curnimg.nWid = pgroup->rtContour.object.width;
	Curnimg.nHig = pgroup->rtContour.object.height;
	nAddress0ff += ADAS_ALIGN_16BYTE_SIZE( Curnimg.nWid * Curnimg.nHig);
	ptr = (unsigned char *) m_globlparam[nId].m_PublacSape + nAddress0ff;

	mvSelcImg(&Curnimg, pgroup->rtContour, 1, nId);

	fShunkRio = 1 / 10.0f;
	SearcRe.object.x = (int) (fShunkRio * Curnimg.nWid);
	SearcRe.object.y = (int) (fShunkRio * Curnimg.nWid);
	SearcRe.object.width = (int) ((1 - fShunkRio * 2) * Curnimg.nWid);
	SearcRe.object.height = (int) ((1 - fShunkRio * 2) * Curnimg.nHig);

	bMatchSucces = mvTemplatMatch(Curnimg, Tempimg, SearcRe, &MatchRec,
		nAddress0ff, 1, m_globlparam[nId].m_PublacSape, fMatchScore);

	if (bMatchSucces) {
		pMatchRec->object.x = MatchRec.object.x - TempRec.object.x + pgroup->rtContour.object.x; // ProcessContour
		pMatchRec->object.y = MatchRec.object.y - TempRec.object.y + pgroup->rtContour.object.y; //ProcessContour
		pMatchRec->object.width = ResizeOrimg.nWid;
		pMatchRec->object.height = ResizeOrimg.nHig;
		pMatchRec->object.confidence = pgroup->rtContour.object.confidence;
	}

	return bMatchSucces;
}

/*
 *************************************************************
 Function process:
 + caculate group sacles and location based on pTrajecy->ProcessPoint and pTrajecy->InitPoint, update pgroup->rtContour
 Fan-in :
 + mvScaleVal()
 + mvTrajecyVote()
 + mvDelFarCen()
 Fan-out:
 + N/A
 ATTENTION: __________
 */
static unsigned char mvProcessConsensVote(obj_group *pgroup, int nId) {

	unsigned char bfsalcSucces;
	float fscale = 2.0f;
	int j;
	trajecy *pTrajecy = 0;
	Wissen16SPoint CenVote;
	int nCenVotNum = 0;
	int nSimlarVote = 0;
	Wissen16SPoint Votec;
	int nDisPowTrd;
	int nVotTrd = (scale_shink_4_id == nId) ? 9 : VOTE_CONSENSE_TRD;

	bfsalcSucces = mvScaleVal(pgroup, m_globlparam[nId].m_fsclare, &fscale,
			ProcessVote, nId);

	if ((fscale > 1.15f && pgroup->nMinCarWid < 60) || fscale > 1.3f) {
		return 0;
	}

	//nDisPowTrd = pgroup->rtContour.width * pgroup->rtContour.width/25;
	nDisPowTrd = ((pgroup->nMinCarWid * pgroup->nMinCarWid) >> 6);

	//nDisPowTrd = WS_MIN(225,nDisPowTrd);
	nDisPowTrd = WS_MAX(25,nDisPowTrd);

	if (bfsalcSucces) {
		memset(m_globlparam[nId].m_PublacSape, 0,
				sizeof(int) * pgroup->ntrajecyNum);

		for (j = 0; j < pgroup->ntrajecyNum; j++) {
			pTrajecy = pgroup->pObjtr + j;

			if (pTrajecy->bProcessTracked) {
				CenVote = mvTrajecyVote(pTrajecy, fscale, ProcessVote);

				m_globlparam[nId].m_ndx[nCenVotNum] = CenVote.x;
				m_globlparam[nId].m_ndy[nCenVotNum] = CenVote.y;
				m_globlparam[nId].m_PublacSape[nCenVotNum] = j;
				nCenVotNum++;

			}
		}

		if (!nCenVotNum) {
			return 0;
		}

		memcpy(m_globlparam[nId].m_PublacSape + nCenVotNum,
				m_globlparam[nId].m_ndx, sizeof(int) * nCenVotNum);
		memcpy(m_globlparam[nId].m_PublacSape + (nCenVotNum << 1),
				m_globlparam[nId].m_ndy, sizeof(int) * nCenVotNum);

		binSort_INT(m_globlparam[nId].m_PublacSape + nCenVotNum, nCenVotNum);
		binSort_INT(m_globlparam[nId].m_PublacSape + (nCenVotNum << 1),
				nCenVotNum);

		Votec.x =
				m_globlparam[nId].m_PublacSape[nCenVotNum + (nCenVotNum >> 1)];
		Votec.y = m_globlparam[nId].m_PublacSape[(nCenVotNum << 1)
				+ (nCenVotNum >> 1)];

		nSimlarVote = mvDelFarCen(pgroup, Votec, m_globlparam[nId].m_PublacSape,
				nCenVotNum, nDisPowTrd, nId);

		if (nSimlarVote < nVotTrd) {
			return 0;
		}

		pgroup->rtContour.object.width = (int) (pgroup->ProcessContour.object.width * fscale);
		pgroup->rtContour.object.height =
				(int) (pgroup->ProcessContour.object.height * fscale);
		pgroup->rtContour.object.x = Votec.x - (pgroup->rtContour.object.width >> 1);
		pgroup->rtContour.object.y = Votec.y - (pgroup->rtContour.object.height >> 1);

	}

	return nSimlarVote;

}

/*
 Function process:
 + Tracking the groups in m_globlparam[].m_pGroupSets
 Fan-in :
 + mvSingledScaleTrack()
 Fan-out:
 + mvUpdataByDetctor()
 + mvUpdataGroupCenTraj()
 + mvOccludedbyGroup()
 + mvDetcorBytrainByGroup()
 + mvMatchByGroupTemplate()
 + mvGroupFilter()
 + mvInitConsensVote()
 + mvUpdataProcessVote()
 + mvclearProcesState()
 + mvTempleMatchByOri()
 + mvProcessConsensVote()
 ATTENTION: __________
 */
static void mvPreditGroup(const PortInput *pInPutParam, int nId) {
	int i;
	obj_group *pgroup = 0;
	//const unsigned char nSimlarTrd = 4;
	unsigned char nSimlarVote;
	WissenObjectRectTracked OccluedRec;
	unsigned char bocclued = 0;
	unsigned char bTempltMatch;
	WissenObjectRectTracked MatchResultRec;
	size_t SmallObjTrd = 32;
	WissenObjectRectTracked ShrinkTempRec;
	unsigned char bDetCarByRio = 0;
	WissenObjectRectTracked DetRect;
	float fzoom = (float) GROUP_TEMPLATE_SHINK_RATE;
	unsigned char bUpadataByCar;
	unsigned char bGetNewLocal;
	int nFindLapIndex;
	float fMatchScore;
	long long unFramSeq = m_globlparam[nId].scaleInput.nFramSeq;
	WissenObjectRect tempRect = { 0, 0, m_globlparam[nId].m_ImgWidth, m_globlparam[nId].m_ImgHeight };

	for (i = 0; i < m_globlparam[nId].m_GroupIndexNum; i++) {
		pgroup = m_globlparam[nId].m_pGroupSets
				+ m_globlparam[nId].m_pGroupIndex[i];
		pgroup->nTime = m_globlparam[nId].scaleInput.objTime;
		pgroup->nFramseq = m_globlparam[nId].scaleInput.nFramSeq;
		bGetNewLocal = 0;

		// reserve discard target for 4 frame
#ifdef RET_DS
		if (4 == pgroup->nDisCardNum ) {

			mvClearGroupIfo(pgroup, 1);
		}
#endif

		// update target by detect
		bUpadataByCar = mvUpdataByDetctor(pgroup, nId);
		if (bUpadataByCar) {
			pgroup->SerpreditNum = 0;
			mvUpdataGroupCenTraj(pgroup);
			my_printf("***pgroup ID: %d is updated by detect!\n",
					pgroup->nGroupId);
			continue;
		}

		// delete the dead target
		if (pgroup->nFramseq - pgroup->updataFrambyCar > 30 && pgroup->rtContour.object.confidence == 0)
		{
			my_printf("***pgroup ID: %d don't get the update for a long time and confidence is zero!\n",
				pgroup->nGroupId);
			mvClearGroupIfo(pgroup, 1);
			continue;
		}

		// target close to edge of image
		if ( pgroup->rtContour.object.x < 10
			|| pgroup->rtContour.object.x + pgroup->rtContour.object.width + 10
		> m_globlparam[nId].m_ImgWidth
		|| pgroup->rtContour.object.y < 10
		|| pgroup->rtContour.object.y + pgroup->rtContour.object.height + 10
		> m_globlparam[nId].m_ImgHeight) {
			my_printf("***pgroup ID: %d is too large and be dropped!\n",
				pgroup->nGroupId);
			mvClearGroupIfo(pgroup, 1);
			continue;
		}

		// Occlusion process
		bocclued = mvOccludedbyGroup(m_globlparam[nId].m_pGroupIndex[i],
			&OccluedRec, &nFindLapIndex, nId);
		if (bocclued)
		{
			obj_group *pComparedgroup = m_globlparam[nId].m_pGroupSets
				+ m_globlparam[nId].m_pGroupIndex[nFindLapIndex];

			if ( pgroup->nTruelyObj && pComparedgroup->nTruelyObj )
			{
				if (pComparedgroup->rtContour.object.width > pgroup->rtContour.object.width)
				{
					mvClearGroupIfo(pgroup, 1);
				} 
				else 
				{
					mvClearGroupIfo(pComparedgroup, 1);
				}
			}
			else  if( pgroup->nTruelyObj && !pComparedgroup->nTruelyObj )
			{
				mvClearGroupIfo(pComparedgroup, 1);
			}
			else if( !pgroup->nTruelyObj && pComparedgroup->nTruelyObj )
			{
				mvClearGroupIfo(pgroup, 1);
			}
			mvclearInitState(pgroup, OccluedRec);
		}
	
		// small target use NCC temp match
		if (WS_MAX(pgroup->rtContour.object.width ,pgroup->rtContour.object.height)
				< MAX_TEMPLAT_NCC_TRACK_SIZE) 
		{

			bTempltMatch = mvMatchByGroupTemplate(pgroup, &MatchResultRec, nId,
					&fMatchScore);

			if (bTempltMatch) 
			{
				bGetNewLocal = 1;
				pgroup->rtContour = MatchResultRec; 

				if (scale_shink_4_id == nId) {
					mvEstablInitVote(pgroup, 0.25f, nId, unFramSeq);
				} else {
					mvEstablInitVote(pgroup, 0.15f, nId, unFramSeq);
				}

				pgroup->nPreProssVotFram = 0;

			} 
			else 
			{

#ifdef DETCOR_STAR	
				bDetCarByRio = mvDetcorBytrainByGroup(pInPutParam, pgroup,
						&DetRect, 1, nId);
				if (bDetCarByRio && !bocclued) {
					bGetNewLocal = 1;
					mvUpdataGroupByDet(pgroup, &DetRect, nId);
				}
#endif	
			}

			if (bGetNewLocal == 0) {
				my_printf(
						"***pgroup ID: %d is Small and not Template Matched and not Detected!\n",
						pgroup->nGroupId);
			}

			mvGroupFilter(pgroup, bGetNewLocal, nId);
			continue;
		}

		// big target use CMT vote
		nSimlarVote = mvInitConsensVote(pgroup, nId);
		if (nSimlarVote)
		{
			mvDelUnReasonTrack(pgroup);

			mvAddTrajecToGroup(pgroup, nId);

			mvUpdataProcessVote(pgroup, nId);

			if (bocclued) {
				mvclearProcesState(pgroup, OccluedRec);
			}

			bGetNewLocal = 1;

		} 
		else if (pgroup->nPreProssVotFram)
		{
			if (bocclued) {
				mvclearProcesState(pgroup, OccluedRec);
			}

			bTempltMatch = mvTempleMatchByOri(pgroup, &MatchResultRec, nId,
					&fMatchScore);

			if (bTempltMatch) {
				bGetNewLocal = 1;

				pgroup->rtContour = MatchResultRec; //

				if (scale_shink_4_id == nId) {
					mvEstablInitVote(pgroup, 0.25f, nId, unFramSeq);
				} else {
					mvEstablInitVote(pgroup, 0.15f, nId, unFramSeq);
				}

				pgroup->nPreProssVotFram = 0;

				if (bGetNewLocal == 0) {
					my_printf(
							"pgroup ID: %d is not Tracked and not Template Matched!\n",
							pgroup->nGroupId);
				}

				mvGroupFilter(pgroup, bGetNewLocal, nId);

				continue;
			}

			if (fMatchScore < 0.5f) 
			{
#ifdef DETCOR_STAR	
				bDetCarByRio = mvDetcorBytrainByGroup(pInPutParam, pgroup,
						&DetRect, 1, nId);
#else
				bDetCarByRio = 0;
#endif
				if (!bDetCarByRio) {
					my_printf(
							"***pgroup ID: %d is not Tracked and not Template Matched and not detect!\n",
							pgroup->nGroupId);

#ifdef RET_DS

					pgroup->nDetDiscardFam =  m_globlparam[nId].scaleInput.nFramSeq;
					pgroup->nDisCardNum++;

#else
					mvClearGroupIfo(pgroup, 1);
#endif
				} else {
					pgroup->nDisCardNum = 0;
					mvUpdataGroupByDet(pgroup, &DetRect, nId);
					mvUpdataGroupCenTraj(pgroup);
				}

				continue;
			}

			nSimlarVote = mvProcessConsensVote(pgroup, nId);

			if (nSimlarVote)
			{
				mvDelUnReasonTrack(pgroup);

				mvAddTrajecToGroup(pgroup, nId);

				limitObjectRectRange(tempRect, &pgroup->rtContour.object);
				pgroup->InitContour = pgroup->rtContour;

				mvEstablInitVote(pgroup, 0.15f, nId, unFramSeq);

				bocclued = mvOccludedbyGroup(m_globlparam[nId].m_pGroupIndex[i],
						&OccluedRec, &nFindLapIndex, nId);

				if (bocclued) {
					mvclearInitState(pgroup, OccluedRec);
				} else if (m_globlparam[nId].scaleInput.nFramSeq
						- pgroup->updataOriFramSeq > 30) {
					mvSetOriInitGray(pgroup, nId);
					pgroup->OriInitContour = pgroup->InitContour;
					pgroup->updataOriFramSeq = m_globlparam[nId].scaleInput.nFramSeq;
				}

				bGetNewLocal = 1;
			}

		}

		if (!bGetNewLocal)
		{
			if (WS_MAX(pgroup->rtContour.object.width,pgroup->rtContour.object.height)
				< MAX_PREDIT_TRACK_SIZE) {
					if (pgroup->SerpreditNum < 3) {
						bGetNewLocal = 1;
						pgroup->rtContour = pgroup->rtContour;
						pgroup->SerpreditNum++;

					mvDelUnReasonTrack(pgroup);
					if (bocclued) {
						mvclearProcesState(pgroup, OccluedRec);
					}

				} else {
#ifdef DETCOR_STAR	

					bDetCarByRio = mvDetcorBytrainByGroup(pInPutParam, pgroup,
							&DetRect, 1, nId);
					if (bDetCarByRio && !bocclued) {
						bGetNewLocal = 1;
						mvUpdataGroupByDet(pgroup, &DetRect, nId);
					}

					if (bGetNewLocal == 0) {
						my_printf(
								"***pgroup ID: %d size is < 80 and not updated before and already pretected for 3 frames but not detected!\n",
								pgroup->nGroupId);
					}
#endif			
				}

			} else {

#ifdef DETCOR_STAR	
				bDetCarByRio = mvDetcorBytrainByGroup(pInPutParam, pgroup,
						&DetRect, 1, nId);
				if (bDetCarByRio && !bocclued) {
					bGetNewLocal = 1;
					mvUpdataGroupByDet(pgroup, &DetRect, nId);
				}
				if (bGetNewLocal == 0) {
					my_printf(
							"***pgroup ID: %d size is > 80 and not updated before and not detected!\n",
							pgroup->nGroupId);
				}
#endif	
			}

		}

		mvGroupFilter(pgroup, bGetNewLocal, nId);

		if ((Car == pgroup->ntype
				&& (pgroup->rtContour.object.height
						> pgroup->rtContour.object.width
								+ (pgroup->rtContour.object.width >> 1)))
				|| (pgroup->rtContour.object.width
						> pgroup->rtContour.object.height
								+ (pgroup->rtContour.object.height >> 1))) {
			my_printf("***pgroup ID: %d width and height is not a rect!\n",
					pgroup->nGroupId);
			mvClearGroupIfo(pgroup, 1);
			continue;
		}

		if (WS_MAX(pgroup->rtContour.object.width, pgroup->rtContour.object.height)
				< MAX_TEMPLAT_NCC_TRACK_SIZE)
		{
			mvShinkRect(pgroup->rtContour.object, &ShrinkTempRec.object, fzoom);
			mvSelcImg(&pgroup->Templat, ShrinkTempRec, 1, nId);
		}

	}
}

/*
 *************************************************************
 Function process:
 + add new group to m_globlparam.m_pGroupSets from m_globlparam[nId].m_PNewRec
 Fan-in :
 + mvSingledScaleTrack()
 Fan-out:
 + kalman_rect_init()
 + mvShinkRect()
 + mvSelcImg()
 ATTENTION: __________
 */
static void mvGroupGenerate(int nId) {
	int i, j;
	int nNextGroup = 0;
	int nUseGroup = 0;
	obj_group *pNewgroup = 0;
	AdasCorner *pCorner = 0;
	trajecy *pTrajec = 0;
	AdasCorner *pRectCorer = (AdasCorner*) m_globlparam[nId].m_PublacSape;
	int RectCorenerNum;
	//int nExtend = 20;
	AdasCorner ReCen;
	WissenObjectRectTracked ShrinkTempRec;
	float fzoom = (float) GROUP_TEMPLATE_SHINK_RATE;
	float init_p[4] = { 5e2, 5e2, 5e2, 5e2 };

	for (i = 0; i < m_globlparam[nId].m_NewRecNum; i++) {
		nUseGroup = 0;

		for (j = nNextGroup; j < MAX_LAYER_OBJ_GROUP_NUMS; j++) {
			pNewgroup = m_globlparam[nId].m_pGroupSets + j;

			if (-1 == pNewgroup->nGroupId) {
				nNextGroup = j + 1;
				nUseGroup = 1;
				break;
			}
		}

		//
		if (!nUseGroup) {
			break;
		}

		m_globlparam[0].m_GroupId++;
		m_globlparam[nId].m_GroupId = m_globlparam[0].m_GroupId;

		pNewgroup->nGroupId = m_globlparam[nId].m_GroupId;
		pNewgroup->rtContour = m_globlparam[nId].m_PNewRec[i];
		pNewgroup->InitContour = m_globlparam[nId].m_PNewRec[i];
		pNewgroup->nFramseq = m_globlparam[nId].scaleInput.nFramSeq;
		pNewgroup->nTime = m_globlparam[nId].scaleInput.objTime;
		pNewgroup->ntype = m_globlparam[nId].m_PNewRec[i].nType;
		pNewgroup->nLastupdataCarWidth = pNewgroup->rtContour.object.width;

		pNewgroup->histoyRec.pGroupSiz[0] = pNewgroup->rtContour.object.width;
		pNewgroup->histoyRec.pGroupSiz[MAX_HISRORY_GROUP_REC_NUM] =
				pNewgroup->rtContour.object.height;
		pNewgroup->histoyRec.nSizNum++;

		RectCorenerNum = 0;

		ReCen.point.x = pNewgroup->rtContour.object.x + (pNewgroup->rtContour.object.width >> 1);
		ReCen.point.y = pNewgroup->rtContour.object.y + (pNewgroup->rtContour.object.height >> 1);

		for (j = 0; j < m_globlparam[nId].m_nFastCornerNum; j++) {
			pCorner = m_globlparam[nId].m_pFastCorner + j;
			WissenObjectRectTracked* tempObject = m_globlparam[nId].m_PNewRec + i;
			if (isPointInRect(&pCorner->point, &tempObject->object)
					&& !pCorner->State.nMacthNum) {
				pRectCorer[RectCorenerNum] = *pCorner;
				pRectCorer[RectCorenerNum].val = -(WS_ABS(pCorner->point.x - ReCen.point.x)
						+ WS_ABS(pCorner->point.y - ReCen.point.y));
				pRectCorer[RectCorenerNum].nCornerIndex = j;
				RectCorenerNum++;

			}

		}

		if (RectCorenerNum > MAX_TRACKS_NUM_OF_GROUP) {
			binSort_Corner(pRectCorer, RectCorenerNum);
			RectCorenerNum = MAX_TRACKS_NUM_OF_GROUP;
		}

		for (j = 0; j < WS_MIN(RectCorenerNum,MAX_TRACKS_NUM_OF_GROUP); j++) {
			pCorner = pRectCorer + j;

			pTrajec = pNewgroup->pObjtr + pNewgroup->ntrajecyNum;

			pTrajec->nMapGroupId = m_globlparam[nId].m_GroupId;
			pTrajec->nEstTimes = 0;

			pTrajec->pTrackPoint->point = pCorner->point;

			pTrajec->pTrackPoint->nFramseq = m_globlparam[nId].scaleInput.nFramSeq;
			pTrajec->pTrackPoint->nTime = m_globlparam[nId].scaleInput.objTime;
			pTrajec->pTrackPoint->nMatchStatus = 1;
			pTrajec->nTrackLen++;
			pTrajec->bInitTracked = 1;

			memcpy(pTrajec->pfeature,
					m_globlparam[nId].m_pfeature
							+ (pRectCorer[j].nCornerIndex << 6),
					SURF_DESC_DIMENTION);

			mvInitVote(pTrajec, pNewgroup->rtContour, 0);

			pNewgroup->ntrajecyNum++;
		}

		mvSetOriInitGray(pNewgroup, nId);
		pNewgroup->OriInitContour = pNewgroup->InitContour;
		pNewgroup->updataOriFramSeq = m_globlparam[nId].scaleInput.nFramSeq;
		pNewgroup->nTruelyObj = 0;
		pNewgroup->nPerLastDetFramseq = m_globlparam[nId].scaleInput.nFramSeq;
		pNewgroup->nStateNum = 0;
		pNewgroup->nMinCarWid = pNewgroup->InitContour.object.width;
		pNewgroup->CarBottom.nDetBottomNum = 0;
		pNewgroup->CarBottom.nDetYPos = pNewgroup->rtContour.object.y
				+ pNewgroup->rtContour.object.height;


		//initial discard target 
		pNewgroup->nDisCardNum = 0;

		kalman_rect_init(&pNewgroup->KalmState, pNewgroup->rtContour.object, init_p);

		WissenObjectRectTracked tempRec;
		tempRec.object.x = (pNewgroup->rtContour.object.x << nId);
		tempRec.object.y = (pNewgroup->rtContour.object.y << nId) + (pNewgroup->rtContour.object.height << nId);
		tempRec.object.width = (pNewgroup->rtContour.object.width << nId);
		tempRec.object.height = (pNewgroup->rtContour.object.height << nId);
		sysKalmanInit(&pNewgroup->sysKalmState, tempRec);

#ifdef DETCOR_STAR	
		pNewgroup->updataFrambyCar = m_globlparam[nId].scaleInput.nFramSeq;

		if (m_globlparam[nId].pOriInPutParam->dayOrNight == 0)
		{
			if (m_globlparam[nId].m_PNewRec[i].object.confidence > 20) {
				pNewgroup->nTruelyObj = 1;
			}
		}
		else if (m_globlparam[nId].pOriInPutParam->dayOrNight == 1)
		{
			if (m_globlparam[nId].m_PNewRec[i].object.confidence > 60) {
				pNewgroup->nTruelyObj = 1;
			}
		}
	
#endif
		if (WS_MAX(pNewgroup->rtContour.object.width,pNewgroup->rtContour.object.height)
  				< MAX_TEMPLAT_NCC_TRACK_SIZE) {
			mvShinkRect(pNewgroup->rtContour.object, &ShrinkTempRec.object, fzoom);

			mvSelcImg(&pNewgroup->Templat, ShrinkTempRec, 1, nId);

		}
		mvUpdataGroupCenTraj(pNewgroup);

	}

}

/*
Function process:
+ Do the Kalman filter for pGroup->rtContour
Fan-in :
+ mvSingledScaleTrack()
Fan-out:
+ kalman_rect_filter()
ATTENTION: __________
*/
static void mvKalmanFiter(int nId) 
{
	int i;
	obj_group *pGroup = 0;

	for (i = 0; i < MAX_LAYER_OBJ_GROUP_NUMS; i++) 
	{
		pGroup = m_globlparam[nId].m_pGroupSets + i;
		if (-1 != pGroup->nGroupId) 
		{
			pGroup->rtContour.object = kalman_rect_filter(&pGroup->KalmState,
				pGroup->rtContour.object);
		}
	}

}

/*
 Function process:
 + Do the singe-sacle tracking for pInPutParam in scale index of nId.
 Fan-out:
 + mvGetScaleInPut()
 + mvReLoaclDetRec()
 + mvGetCurrenGroupIndex()
 + mvAddNewObjRec()
 + mvGoupPreditRec()
 + DetcorBytrain()
 + mvGenerateRioGrayByMask()
 + mvCornerDetct()
 + mvTrajectorymatch()
 + mvPreditGroup()
 + mvGroupGenerate()
 + mvKalmanFiter()

 ATTENTION: __________
 */
static void mvSingledScaleTrack(PortInput *pInPutParam, const int nId) {

	unsigned char *pTemptr = 0;
	WissenImage srcImage;
	srcImage.data = m_globlparam[nId].m_pGrayData;
	srcImage.nWid = m_globlparam[nId].m_ImgWidth;
	srcImage.nHig = m_globlparam[nId].m_ImgHeight;

	mvGetScaleInPut(pInPutParam, nId);

#ifdef OBJRLCT 
	mvReLoaclDetRec(pInPutParam, nId);

#ifdef TIME_TEST
	TIME_TAKE(cvGetTickCount(),ts,"mvReLoaclDetRec");
	ts = cvGetTickCount();
#endif

#endif

	mvGetCurrenGroupIndex(nId);
	if (!m_globlparam[nId].scaleInput.nRecNum && !m_globlparam[nId].m_GroupIndexNum) 
	{
		return;
	}

#ifdef TIME_TEST
	TIME_TAKE(cvGetTickCount(),ts,"mvGetCurrenGroupIndex");
	ts = cvGetTickCount();
#endif

	mvAddNewObjRec(nId);

#ifdef TIME_TEST
	TIME_TAKE(cvGetTickCount(),ts,"mvAddNewObjRec");
	ts = cvGetTickCount();
#endif

	mvGoupPreditRec(nId);

#ifdef TIME_TEST
	TIME_TAKE(cvGetTickCount(),ts,"mvGoupPreditRec");
	ts = cvGetTickCount();
#endif



	// confirm the target which has low level confidence whether or not a vehicle 
#ifdef DETCOR_STAR
	//DetcorBytrain();
	mvSureTrueGroup(nId);

#ifdef TIME_TAKE
	TIME_TAKE(cvGetTickCount(),ts,"DetcorBytrain");
	ts = cvGetTickCount();
#endif

#endif

	mvGetCurrenGroupIndex(nId);

	mvGenerateRioGrayByMask(nId);


#ifdef TIME_TEST
	TIME_TAKE(cvGetTickCount(),ts,"mvGenerateRioGrayByMask");
	ts = cvGetTickCount();
#endif


#ifdef TIME_TEST
	TIME_TAKE(cvGetTickCount(),ts,"mvImgEnhance");
	ts = cvGetTickCount();
#endif

	//fast角点检测
	mvCornerDetct(&srcImage, m_globlparam[nId].m_pMask,
			m_globlparam[nId].m_nCornerThresh, nId);

#ifdef TIME_TEST
	TIME_TAKE(cvGetTickCount(),ts,"mvCornerDetct");
	ts = cvGetTickCount();
#endif

	//角点描述
	mvComputeSurfDescriptor(&srcImage, m_globlparam[nId].m_pFastCorner,
		m_globlparam[nId].m_nFastCornerNum, m_globlparam[nId].m_pfeature);

#ifdef TIME_TEST
	TIME_TAKE(cvGetTickCount(),ts,"角点描述");
	ts = cvGetTickCount();
#endif

	mvTrajectorymatch(nId);

#ifdef TIME_TEST
	TIME_TAKE(cvGetTickCount(),ts,"mvTrajectorymatch");
	ts = cvGetTickCount();
#endif

	mvPreditGroup(pInPutParam, nId);

#ifdef TIME_TEST
	TIME_TAKE(cvGetTickCount(),ts,"mvPreditGroup");
	ts = cvGetTickCount();
#endif

	mvGroupGenerate(nId);

	if (m_globlparam[nId].pOriInPutParam->dayOrNight == 0)
	{
		mvGroundLineDet(pInPutParam,nId);
	}
	else if (m_globlparam[nId].pOriInPutParam->dayOrNight == 1)
	{
		mvGroundLineDet(pInPutParam, nId);
#ifdef USE_TAIL_LIGHT
		mvTailNightDetct(pInPutParam, nId);
#endif // USE_TAIL_LIGHT
	}

	mvKalmanFiter(nId);

#ifdef TIME_TEST
	TIME_TAKE(cvGetTickCount(),ts,"mvKalmanFiter");
	ts = cvGetTickCount();
#endif


	WS_SWAP(m_globlparam[nId].m_preGrayData, m_globlparam[nId].m_pGrayData,
			pTemptr);
}

/*
 Function process:
 + set the value of pgroup->pMotion
 Fan-in :
 + mvMotionState()
 Fan-out:
 + N/A
 ATTENTION: __________
 */
static void mvSetGroupMotion(obj_group *pgroup, int nId, float fZDis,
							 float fXDis, float fDelVanish) 
{
	Motion *pGroupMotion;

	pGroupMotion = pgroup->pMotion + (pgroup->nMotionLeng & MAX_MOTION_BIT);

	pGroupMotion->z_fdis = fZDis;
	pGroupMotion->x_fdis = fXDis;
	pGroupMotion->ts = m_globlparam[nId].scaleInput.objTime;
	pGroupMotion->delVanish = fDelVanish;
								 
	pGroupMotion->groupRec.object.x = (pgroup->rtContour.object.x << nId);
	pGroupMotion->groupRec.object.y = (pgroup->rtContour.object.y << nId);
	pGroupMotion->groupRec.object.width = (pgroup->rtContour.object.width << nId);
	pGroupMotion->groupRec.object.height = (pgroup->rtContour.object.height << nId);

	pgroup->nMotionLeng++;
}

/*
 Function process:
 + set the value of pgroup->pMotion of each sacle in m_globlparam[nId].m_pGroupSets
 Fan-in :
 + FCW_TRACK_MultieTrack()
 Fan-out:
 + mvSetGroupMotion()
 ATTENTION: __________
 */
static void mvMotionState() 
{
	int i, nId;
	obj_group *pgroup = 0;
	int nBottom_Z_Car, nBottom_X_Car;
	double fx, fz, delVanish;
	g_MuliTracker[g_OutIndex & 1].nTrackeNum = 0;

	for (nId = scale_shink_1_id; nId <= scale_shink_4_id; nId++) {

		for (i = 0; i < MAX_LAYER_OBJ_GROUP_NUMS; i++) {
			pgroup = m_globlparam[nId].m_pGroupSets + i;

			if (-1 != pgroup->nGroupId) {

				nBottom_Z_Car = 
						((pgroup->rtContour.object.y + pgroup->rtContour.object.height
								) << nId );

				nBottom_X_Car = 
						((pgroup->rtContour.object.x + pgroup->rtContour.object.width / 2)
								<<  nId );

#ifdef DETCOR_STAR
				LDWS_Get_Dist_xz(nBottom_X_Car, nBottom_Z_Car, &fx, &fz,
						&delVanish);
#else
				fz =0;
				fx =0;
				delVanish = 0;
#endif
				mvSetGroupMotion(pgroup, nId, (float)fz, (float)fx, (float)delVanish);
			}
		}
	}

}

/****************************************************************************************************************/
/*
 Function process:
 + malloc the memory for Tracking
 Fan-in :
 + main()
 Fan-out:
 + N/A
 ATTENTION: ImgSize is the size of resized img (1/2 of orig image)
 */
void FCW_TRACK_Init_adas_global_param(WissenSize ImgSize) 
{

	int i, j;
	unsigned char *pTr = 0;
	obj_group *pObjGroup = 0;
	trajecy *ptrajecy = 0;
	int nalloclSize = 0;
	int nCornerThresh[3] = { 7, 15, 15 };
	int nId = 0;
	static int initMalloc = 0;
	WissenSize oriSiz = ImgSize;
	float fscale[3] = { 2.0f, 1, 0.5f };

#ifdef USE_TAIL_LIGHT
	int rt = 0;
#endif // USE_TAIL_LIGHT


	for (nId = 0; nId < 3; nId++) {
		ImgSize.width = (short)(oriSiz.width * fscale[nId]);
		ImgSize.height = (short)(oriSiz.height * fscale[nId]);

		if (ALLOC_SUCCESS_STATE == m_globlparam[nId].m_nInitSuccess) {
			return;
		}

		nalloclSize = MAX_OBJ_GROUP_NUMS * sizeof(trakobj)
				+ MAX_OBJ_GROUP_NUMS * sizeof(TrackPoint) * MAX_CORNER_OF_TRACK
				+\
 +MAX_OBJ_GROUP_NUMS * sizeof(Motion) * MAX_MOTION_NUM\

 + sizeof(WissenPoint)* MAX_XY_CORNER + sizeof(WissenPoint)* MAX_XY_CORNER
				+ sizeof(int) * MAX_IMG_HEIGHT + sizeof(int) * MAX_XY_CORNER
				+ sizeof(AdasCorner) * MAX_CORNERS_OF_NONMAX
				+ sizeof(AdasCorner)* MAX_CORNERS_OF_NONMAX
				+ sizeof(unsigned char) * MAX_CORNERS_PER_FRAME * SURF_DESC_DIMENTION
				+ sizeof(WissenObjectRectTracked)* MAX_NEW_GROUP_PER_FRAM
				+ sizeof(unsigned char) * ImgSize.width * ImgSize.height
				+ sizeof(unsigned char) * ImgSize.width * ImgSize.height
				+ sizeof(unsigned char) * ImgSize.width * ImgSize.height
				+ sizeof(obj_group) * MAX_LAYER_OBJ_GROUP_NUMS
				+ MAX_LAYER_OBJ_GROUP_NUMS * sizeof(int) * 2
				+ MAX_LAYER_OBJ_GROUP_NUMS * sizeof(TrackPoint)
						* MAX_CORNER_OF_TRACK\

				+ MAX_LAYER_OBJ_GROUP_NUMS * sizeof(Motion) * MAX_MOTION_NUM
				+ MAX_LAYER_OBJ_GROUP_NUMS * sizeof(trajecy)
						* MAX_TRACKS_NUM_OF_GROUP
				+ MAX_LAYER_OBJ_GROUP_NUMS * MAX_TRACKS_NUM_OF_GROUP
						* (sizeof(TrackPoint) * MAX_CORNER_OF_TRACK
								+ sizeof(unsigned char) * SURF_DESC_DIMENTION
								+ sizeof(unsigned char) * SURF_DESC_DIMENTION)
				+ MAX_LAYER_OBJ_GROUP_NUMS
						* (MAX_TEMPLAT_NCC_TRACK_SIZE
								* MAX_TEMPLAT_NCC_TRACK_SIZE)
				+ MAX_TRACKS_NUM_OF_GROUP * sizeof(int)
				+ MAX_TRACKS_NUM_OF_GROUP * sizeof(int)
				+ MAX_TRACKS_NUM_OF_GROUP * MAX_TRACKS_NUM_OF_GROUP
						* sizeof(float)
				+ MAX_PUBLIC_SPACE_SIZE * sizeof(int)
				+ MAX_LAYER_OBJ_GROUP_NUMS * MAX_GROUP_IMG_BYTE
				+ MAX_LAYER_OBJ_GROUP_NUMS
				+ MAX_LAYER_OBJ_GROUP_NUMS * UPDATA_IMAGE_NUM
						* (UPDATA_IMAGE_SIZE * UPDATA_IMAGE_SIZE)
				+ sizeof(unsigned char) * ImgSize.width * ImgSize.height
				+ sizeof(WissenObjectRectTracked)* MAX_LAYER_OBJ_GROUP_NUMS
				+ MAX_EXTERN_SPACE_SIZE * sizeof(int);

		nalloclSize += (1024 << 8);

		m_globlparam[nId].m_pAlloc = (unsigned char* )my_malloc(nalloclSize);
		m_globlparam[nId].m_nInitSuccess = ALLOC_SUCCESS_STATE;

		m_globlparam[nId].m_ImgWidth = ImgSize.width;
		m_globlparam[nId].m_ImgHeight = ImgSize.height;
		m_globlparam[nId].m_ImgSize = ImgSize;

		pTr = m_globlparam[nId].m_pAlloc;

		if (initMalloc == 0) {
			for (j = 0; j < 2; j++) {
				g_MuliTracker[j].pTrackerset = (trakobj*) pTr;
				pTr =
						ADAS_ALIGN_16BYTE(g_MuliTracker[j].pTrackerset + MAX_OBJ_GROUP_NUMS );
				g_MuliTracker[j].nTrackeNum = 0;

				for (i = 0; i < MAX_OBJ_GROUP_NUMS; i++) {
					g_MuliTracker[j].pTrackerset[i].pCenTrajecy =
							(TrackPoint*) pTr;
					pTr =
							ADAS_ALIGN_16BYTE(g_MuliTracker[j].pTrackerset[i].pCenTrajecy + MAX_CORNER_OF_TRACK);

					g_MuliTracker[j].pTrackerset[i].pMotion = (Motion*) pTr;
					pTr =
							ADAS_ALIGN_16BYTE(g_MuliTracker[j].pTrackerset[i].pMotion + MAX_MOTION_NUM);
				}
			}
			initMalloc = 1;
		}

		m_globlparam[nId].m_pXYCorners = (WissenPoint*)pTr;
		pTr = ADAS_ALIGN_16BYTE(m_globlparam[nId].m_pXYCorners + MAX_XY_CORNER);

		m_globlparam[nId].m_pXYNoMax = (WissenPoint*)pTr;
		pTr = ADAS_ALIGN_16BYTE(m_globlparam[nId].m_pXYNoMax + MAX_XY_CORNER);

		m_globlparam[nId].m_pRowStart = (int *) pTr;
		pTr = ADAS_ALIGN_16BYTE(m_globlparam[nId].m_pRowStart + MAX_IMG_HEIGHT);

		m_globlparam[nId].m_pScore = (int *) pTr;
		pTr = ADAS_ALIGN_16BYTE(m_globlparam[nId].m_pScore + MAX_XY_CORNER);

		m_globlparam[nId].m_pCornerPass = (AdasCorner*) pTr;
		pTr =
				ADAS_ALIGN_16BYTE(m_globlparam[nId].m_pCornerPass + MAX_CORNERS_OF_NONMAX);
		memset(m_globlparam[nId].m_pCornerPass, 0,
				sizeof(AdasCorner) * MAX_CORNERS_OF_NONMAX);

		m_globlparam[nId].m_pFastCorner = (AdasCorner*) pTr;
		memset(m_globlparam[nId].m_pFastCorner, 0,
				sizeof(AdasCorner) * MAX_CORNERS_OF_NONMAX);
		pTr =
				ADAS_ALIGN_16BYTE(m_globlparam[nId].m_pFastCorner + MAX_CORNERS_OF_NONMAX);

		m_globlparam[nId].m_pfeature = (unsigned char *) pTr;
		memset(m_globlparam[nId].m_pfeature, 0,
			sizeof(unsigned char)* MAX_CORNERS_PER_FRAME * SURF_DESC_DIMENTION);
		pTr =
				ADAS_ALIGN_16BYTE(m_globlparam[nId].m_pfeature + MAX_CORNERS_PER_FRAME * SURF_DESC_DIMENTION);

		m_globlparam[nId].m_NewRecNum = 0;
		m_globlparam[nId].m_PNewRec = (WissenObjectRectTracked*)pTr;
		pTr =
				ADAS_ALIGN_16BYTE(m_globlparam[nId].m_PNewRec + MAX_NEW_GROUP_PER_FRAM);

		m_globlparam[nId].m_pGrayData = (unsigned char *) pTr;
		pTr =
				ADAS_ALIGN_16BYTE(m_globlparam[nId].m_pGrayData+ ImgSize.width *ImgSize.height);

		m_globlparam[nId].m_preGrayData = (unsigned char *) pTr;
		pTr =
				ADAS_ALIGN_16BYTE(m_globlparam[nId].m_preGrayData + ImgSize.width *ImgSize.height);

		m_globlparam[nId].m_pMask = (unsigned char *) pTr;
		memset(m_globlparam[nId].m_pMask, 255,
				sizeof(unsigned char) * ImgSize.width * ImgSize.height);
		pTr =
				ADAS_ALIGN_16BYTE(m_globlparam[nId].m_pMask + ImgSize.width * ImgSize.height);

		m_globlparam[nId].m_pGroupSets = (obj_group *) pTr;
		pTr =
				ADAS_ALIGN_16BYTE(m_globlparam[nId].m_pGroupSets + MAX_LAYER_OBJ_GROUP_NUMS);

		for (i = 0; i < MAX_LAYER_OBJ_GROUP_NUMS; i++) {
			pObjGroup = m_globlparam[nId].m_pGroupSets + i;
			pObjGroup->histoyRec.pGroupSiz = (int *) pTr;
			pTr =
					ADAS_ALIGN_16BYTE(pObjGroup->histoyRec.pGroupSiz + MAX_HISRORY_GROUP_REC_NUM * 2 );

		}

		for (i = 0; i < MAX_LAYER_OBJ_GROUP_NUMS; i++) {
			pObjGroup = m_globlparam[nId].m_pGroupSets + i;
			pObjGroup->Templat.data = (unsigned char *) pTr;
			pObjGroup->Templat.nWid = 0;
			pObjGroup->Templat.nHig = 0;
			pObjGroup->updataFrambyCar = -1;
			pObjGroup->SerpreditNum = 0;
			pTr =
					ADAS_ALIGN_16BYTE(pObjGroup->Templat.data + (MAX_TEMPLAT_NCC_TRACK_SIZE * MAX_TEMPLAT_NCC_TRACK_SIZE) );
		}

		for (i = 0; i < MAX_LAYER_OBJ_GROUP_NUMS; i++) {
			pObjGroup = m_globlparam[nId].m_pGroupSets + i;
			pObjGroup->Centr.pTrackPoint = (TrackPoint *) pTr;
			pObjGroup->Centr.pfeature = 0;
			pTr =
					ADAS_ALIGN_16BYTE(pObjGroup->Centr.pTrackPoint + MAX_CORNER_OF_TRACK);

			pObjGroup->pMotion = (Motion *) pTr;
			pTr = ADAS_ALIGN_16BYTE(pObjGroup->pMotion + MAX_MOTION_NUM);

			mvClearTrajIfo(&pObjGroup->Centr);
			mvClearGroupIfo(pObjGroup, 0);
		}

		for (i = 0; i < MAX_LAYER_OBJ_GROUP_NUMS; i++) {
			pObjGroup = m_globlparam[nId].m_pGroupSets + i;
			pObjGroup->pObjtr = (trajecy*) pTr;
			pTr =
					ADAS_ALIGN_16BYTE(pObjGroup->pObjtr + MAX_TRACKS_NUM_OF_GROUP);
			pObjGroup->pOrInitGray = (unsigned char *) pTr;
			pTr =
					ADAS_ALIGN_16BYTE(pObjGroup->pOrInitGray + MAX_GROUP_IMG_BYTE);

			for (j = 0; j < UPDATA_IMAGE_NUM; j++) {
				pObjGroup->UpdatedImg[j].data = (unsigned char *) pTr;
				pTr =
						ADAS_ALIGN_16BYTE(pObjGroup->UpdatedImg[j].data + UPDATA_IMAGE_SIZE * UPDATA_IMAGE_SIZE);
				pObjGroup->UpdatedImg[j].nWid = UPDATA_IMAGE_SIZE;
				pObjGroup->UpdatedImg[j].nHig = UPDATA_IMAGE_SIZE;
				//pObjGroup->UpdatedImg[j].nChannle = 1;
			}

		}

		for (i = 0; i < MAX_LAYER_OBJ_GROUP_NUMS; i++) {
			pObjGroup = m_globlparam[nId].m_pGroupSets + i;

			for (j = 0; j < MAX_TRACKS_NUM_OF_GROUP; j++) {
				ptrajecy = pObjGroup->pObjtr + j;

				ptrajecy->pTrackPoint = (TrackPoint*) pTr;
				pTr =
						ADAS_ALIGN_16BYTE(ptrajecy->pTrackPoint + MAX_CORNER_OF_TRACK);

				ptrajecy->pfeature = (unsigned char*) pTr;
				pTr =
						ADAS_ALIGN_16BYTE(ptrajecy->pfeature + SURF_DESC_DIMENTION);

				ptrajecy->processfeature = (unsigned char*) pTr;
				pTr =
						ADAS_ALIGN_16BYTE(ptrajecy->processfeature + SURF_DESC_DIMENTION);

				mvClearTrajIfo(ptrajecy);
			}

		}

		m_globlparam[nId].m_ndx = (int *) pTr;
		pTr =
				ADAS_ALIGN_16BYTE(m_globlparam[nId].m_ndx + MAX_TRACKS_NUM_OF_GROUP);

		m_globlparam[nId].m_ndy = (int *) pTr;
		pTr =
				ADAS_ALIGN_16BYTE(m_globlparam[nId].m_ndy + MAX_TRACKS_NUM_OF_GROUP);

		m_globlparam[nId].m_fsclare = (float *) pTr;
		pTr =
				ADAS_ALIGN_16BYTE(m_globlparam[nId].m_fsclare + MAX_TRACKS_NUM_OF_GROUP * MAX_TRACKS_NUM_OF_GROUP);

		m_globlparam[nId].m_PublacSape = (int*) pTr;
		pTr =
				ADAS_ALIGN_16BYTE(m_globlparam[nId].m_PublacSape + MAX_PUBLIC_SPACE_SIZE);

		m_globlparam[nId].m_pGroupIndex = (unsigned char *) pTr;
		pTr =
				ADAS_ALIGN_16BYTE(m_globlparam[nId].m_pGroupIndex + MAX_LAYER_OBJ_GROUP_NUMS);

		m_globlparam[nId].scaleInput.pGrayfram = (unsigned char *) pTr;
		pTr =
				ADAS_ALIGN_16BYTE(m_globlparam[nId].scaleInput.pGrayfram + ImgSize.width * ImgSize.height);

		m_globlparam[nId].scaleInput.objRec = (WissenObjectRectTracked *)pTr;
		pTr =
				ADAS_ALIGN_16BYTE(m_globlparam[nId].scaleInput.objRec + MAX_LAYER_OBJ_GROUP_NUMS);

		m_globlparam[nId].m_extern_Space = (int *) pTr;
		pTr =
				ADAS_ALIGN_16BYTE(m_globlparam[nId].m_extern_Space + MAX_EXTERN_SPACE_SIZE);

		m_globlparam[nId].m_nCornerThresh = nCornerThresh[nId];
		m_globlparam[nId].m_nCornerPassNum = 0;
		m_globlparam[nId].m_nFastCornerNum = 0;
		m_globlparam[nId].m_NewRecNum = 0;
		m_globlparam[nId].m_GroupId = -1;
		m_globlparam[nId].scaleInput.nFramSeq = 0;
		m_globlparam[nId].m_GroupIndexNum = 0;
	}

#ifdef USE_TAIL_LIGHT
	rt = initTailLight(1280, 720, &gl_tailnightpara);
#endif // USE_TAIL_LIGHT
		
}

/*
 Function process:
 + The main realize of multi-tracking
 Fan-in :
 + main()
 Fan-out:
 + N/A
 ATTENTION: __________
 */
void FCW_TRACK_MultieTrack(PortInput *pInPutParam, CANInfo *CANData)
{
	int nScaleId;

	g_OutIndex++;

	removeOverlappedRects(pInPutParam->nRecNum, pInPutParam->objRec);

	for (nScaleId = scale_shink_1_id; nScaleId <= scale_shink_4_id; nScaleId++)
	{
		m_globlparam[nScaleId].pOriInPutParam = pInPutParam;
		mvSingledScaleTrack(pInPutParam, nScaleId);
	}

	neighborScaleObjectFusion(scale_shink_1_id, scale_shink_2_id, m_globlparam[scale_shink_1_id].m_pGroupSets, m_globlparam[scale_shink_2_id].m_pGroupSets);
	neighborScaleObjectFusion(scale_shink_2_id, scale_shink_4_id, m_globlparam[scale_shink_2_id].m_pGroupSets, m_globlparam[scale_shink_4_id].m_pGroupSets);
	neighborScaleObjectFusion(scale_shink_1_id, scale_shink_4_id, m_globlparam[scale_shink_1_id].m_pGroupSets, m_globlparam[scale_shink_4_id].m_pGroupSets);

	mvMotionState();

	mvTTC(CANData);

	mvSetTrackerReult(CANData);
}

/*
 Function process:
 + Get the multi-tracking result
 Fan-in :
 + main()
 Fan-out:
 + N/A
 ATTENTION: tarcking rect is based on resized image (1/2 of orig image)
 */
void FCW_TRACK_GetResult(MuliTracker **pTrackOutput) 
{
	*pTrackOutput = g_MuliTracker + (g_OutIndex & 1);
}

/*
 Function process:
 + Clear m_globlparam[nId].m_pGroupSets
 Fan-in :
 + main()
 Fan-out:
 + N/A
 ATTENTION:
 */
void FCW_TRACK_mvClearWholeGroup() {
	int i;
	int nId;
	obj_group * pgroup = 0;

	for (nId = 0; nId < 16; nId++) {

		if (ALLOC_SUCCESS_STATE == m_globlparam[nId].m_nInitSuccess) {

			for (i = 0; i < MAX_LAYER_OBJ_GROUP_NUMS; i++) {
				pgroup = m_globlparam[nId].m_pGroupSets + i;

				if (pgroup->nGroupId != -1) {
					mvClearGroupIfo(pgroup, 1);
				}

			}
		}

	}

}

/*
 Function process:
 + Free memory
 Fan-in :
 + main()
 Fan-out:
 + N/A
 ATTENTION:
 */
void FCW_TRACK_Unit_adas_global_param() {
	int nId;

	for (nId = 0; nId < inner_global_param_num; nId++) {
		if (ALLOC_SUCCESS_STATE == m_globlparam[nId].m_nInitSuccess) {
			my_free(m_globlparam[nId].m_pAlloc);
			m_globlparam[nId].m_pAlloc = NULL;
			m_globlparam[nId].m_nInitSuccess = ALLOC_SUCCESS_STATE - 1;
		}
	}
#ifdef USE_TAIL_LIGHT
	freeTailLight(&gl_tailnightpara);
#endif // USE_TAIL_LIGHT		
}

/*
 **************************************************************/
static unsigned char mvOrinitConsensVote(obj_group *pgroup, int nId) {

	unsigned char bfsalcSucces;
	float fscale;
	int j;
	trajecy *pTrajecy = 0;
	Wissen16SPoint CenVote;
	int nCenVotNum = 0;
	int nSimlarVote = 0;
	//Wissen16SPoint Votec,AvgVotec;
	Wissen16SPoint Votec;
	int nDisPowTrd;
	WissenObjectRect tempRect = { 0, 0, m_globlparam[nId].m_ImgWidth, m_globlparam[nId].m_ImgHeight };

	bfsalcSucces = mvScaleVal(pgroup, m_globlparam[nId].m_fsclare, &fscale,
			OrinitVote, nId);

	nDisPowTrd = pgroup->rtContour.object.width * pgroup->rtContour.object.width / 25;

	nDisPowTrd = WS_MIN(255,nDisPowTrd);

	if (bfsalcSucces) {
		memset(m_globlparam[nId].m_PublacSape, 0,
				sizeof(int) * pgroup->ntrajecyNum);
		for (j = 0; j < pgroup->ntrajecyNum; j++) {
			pTrajecy = pgroup->pObjtr + j;

			if (pTrajecy->bOrInitVoteTracked) {
				CenVote = mvTrajecyVote(pTrajecy, fscale, OrinitVote);

				if (pgroup->Centr.nTrackLen > 2
						&& mvDisPow(
								&pgroup->Centr.pTrackPoint[(pgroup->Centr.nTrackLen
										- 1) & MAX_CORNER_OF_TRACK_BIT].point,
								&CenVote) < nDisPowTrd)

						{
					m_globlparam[nId].m_ndx[nCenVotNum] = CenVote.x;
					m_globlparam[nId].m_ndy[nCenVotNum] = CenVote.y;
					m_globlparam[nId].m_PublacSape[nCenVotNum] = j;
					nCenVotNum++;
				} else {
					pTrajecy->bOrInitVoteTracked = 0;
				}

			}
		}

		if (!nCenVotNum) {
			return 0;
		}

		memcpy(m_globlparam[nId].m_PublacSape + nCenVotNum,
				m_globlparam[nId].m_ndx, sizeof(int) * nCenVotNum);
		memcpy(m_globlparam[nId].m_PublacSape + (nCenVotNum << 1),
				m_globlparam[nId].m_ndy, sizeof(int) * nCenVotNum);

		binSort_INT(m_globlparam[nId].m_PublacSape + nCenVotNum, nCenVotNum);
		binSort_INT(m_globlparam[nId].m_PublacSape + (nCenVotNum << 1),
				nCenVotNum);

		Votec.x =
				m_globlparam[nId].m_PublacSape[nCenVotNum + (nCenVotNum >> 1)];
		Votec.y = m_globlparam[nId].m_PublacSape[(nCenVotNum << 1)
				+ (nCenVotNum >> 1)];

		nSimlarVote = mvDelFarCen(pgroup, Votec, m_globlparam[nId].m_PublacSape,
				nCenVotNum, nDisPowTrd, nId);


		if (nSimlarVote < VOTE_CONSENSE_TRD) {
			return 0;
		}

		pgroup->rtContour.object.width = (int) (pgroup->OriInitContour.object.width * fscale);
		pgroup->rtContour.object.height =
				(int) (pgroup->OriInitContour.object.height * fscale);
		pgroup->rtContour.object.x = Votec.x - (pgroup->OriInitContour.object.width >> 1);
		pgroup->rtContour.object.y = Votec.y - (pgroup->OriInitContour.object.height >> 1);

		pgroup->rtContour.object.height = pgroup->rtContour.object.width;

		pgroup->nFramseq = m_globlparam[nId].scaleInput.nFramSeq;
		pgroup->nTime = m_globlparam[nId].scaleInput.objTime;
		pgroup->histoyRec.pGroupSiz[pgroup->histoyRec.nSizNum
				% MAX_HISRORY_GROUP_REC_NUM] = pgroup->rtContour.object.width;
		pgroup->histoyRec.pGroupSiz[pgroup->histoyRec.nSizNum
				% MAX_HISRORY_GROUP_REC_NUM + MAX_HISRORY_GROUP_REC_NUM] =
				pgroup->rtContour.object.height;

		pgroup->histoyRec.nSizNum++;

		mvMidGroupRec(pgroup);

		limitObjectRectRange(tempRect, &pgroup->rtContour.object);

	}

	return nSimlarVote;

}

/*
 **************************************************************/
static unsigned char mvCorrectProcessVotebyTempMatch(obj_group *pgroup, WissenObjectRectTracked *pMatchRec,
		int nId, float *fMatchScore) {
	int i;
	int nAddress0ff = 0;
	WissenImage ResizeOrimg;
	WissenImage Tempimg, Curnimg;
	WissenObjectRectTracked SearcRe, MatchRec, TempRec;
	float fShunkRio;
	unsigned char bMatchSucces;
	Wissen16SPoint VoteCorreVal;
	trajecy *pTrajecy = 0;
	unsigned char * ptr = (unsigned char *) m_globlparam[nId].m_PublacSape;
	ResizeOrimg.data = ptr;

	ResizeOrimg.nWid = pgroup->ProcessContour.object.width;
	ResizeOrimg.nHig = pgroup->ProcessContour.object.height;

	if (WS_MAX(ResizeOrimg.nWid ,ResizeOrimg.nHig) > MAX_TEMPLAT_TRACK_SIZE
			|| (!pgroup->InitContour.object.width)
			|| pgroup->ProcessContour.object.width * pgroup->ProcessContour.object.height
					>= MAX_GROUP_IMG_BYTE) {
		return 0;
	}

	//
	mvOriObjResizeToDstImg(pgroup, ResizeOrimg.data, nId);

	nAddress0ff += ADAS_ALIGN_16BYTE_SIZE( ResizeOrimg.nWid * ResizeOrimg.nHig);
	ptr = (unsigned char *) m_globlparam[nId].m_PublacSape + nAddress0ff;

	Tempimg.data = ptr;
	mvSelcTemplatByCen(ResizeOrimg, &Tempimg, &TempRec.object);
	nAddress0ff += ADAS_ALIGN_16BYTE_SIZE( Tempimg.nWid * Tempimg.nHig);
	ptr = (unsigned char *) m_globlparam[nId].m_PublacSape + nAddress0ff;

	Curnimg.data = ptr;
	Curnimg.nWid = pgroup->ProcessContour.object.width;
	Curnimg.nHig = pgroup->ProcessContour.object.height;
	nAddress0ff += ADAS_ALIGN_16BYTE_SIZE( Curnimg.nWid * Curnimg.nHig);
	ptr = (unsigned char *) m_globlparam[nId].m_PublacSape + nAddress0ff;

	mvSelcImg(&Curnimg, pgroup->ProcessContour, 0, nId);

	fShunkRio = 1 / 10.0f;
	SearcRe.object.x = (int) (fShunkRio * Curnimg.nWid);
	SearcRe.object.y = (int) (fShunkRio * Curnimg.nWid);
	SearcRe.object.width = (int) ((1 - fShunkRio * 2) * Curnimg.nWid);
	SearcRe.object.height = (int) ((1 - fShunkRio * 2) * Curnimg.nHig);

	bMatchSucces = mvTemplatMatch(Curnimg, Tempimg, SearcRe, &MatchRec,
		nAddress0ff, 1, m_globlparam[nId].m_PublacSape, fMatchScore);

	if (bMatchSucces) {
		VoteCorreVal.x = (Curnimg.nWid >> 1)
				- (MatchRec.object.x + (MatchRec.object.width >> 1));
		VoteCorreVal.y = (Curnimg.nHig >> 1)
				- (MatchRec.object.y + (MatchRec.object.height >> 1));

		for (i = 0; i < pgroup->ntrajecyNum; i++) {
			pTrajecy = pgroup->pObjtr + i;

			if (pTrajecy->bProcessTracked) {
				mvCorrectProcessVote(pTrajecy, VoteCorreVal);

			}
		}

		pgroup->ProcessContour.object.x -= VoteCorreVal.x;
		pgroup->ProcessContour.object.y -= VoteCorreVal.y;

		pMatchRec->object.x = MatchRec.object.x - TempRec.object.x + pgroup->ProcessContour.object.x; // ProcessContour
		pMatchRec->object.y = MatchRec.object.y - TempRec.object.y + pgroup->ProcessContour.object.y; //ProcessContour
		pMatchRec->object.width = ResizeOrimg.nWid;
		pMatchRec->object.height = ResizeOrimg.nHig;

		if ((MatchRec.object.x - TempRec.object.x) || (MatchRec.object.y - TempRec.object.y))
			mvSetTrajLenthOne(pgroup);
	}

	return bMatchSucces;

}

/*
 *************************************************************
 Function process:
 + Do the sample tracking based on temple tracking
 Fan-in :
 + main()
 Fan-out:
 + N/A
 ATTENTION: __________
 */
int FCW_TRACK_SimpleTrack(const WissenObjectRectTracked SrcRec, WissenImage *pSrcImg,
	const WissenImage *pDstImg, Wissen16SPoint MotionVec, WissenObjectRectTracked *pDstRec) {

	WissenImage TempleImg;
	WissenObjectRectTracked TemplRec;
	float fShink = 0.15f;
	float fLager = 0.2f;
	unsigned char bTemplat;
	WissenObjectRectTracked SearcRec, MatchRec;
	int nPubliBuffOff;
	int nId = 0;
	WissenObjectRect tempRect = { 0, 0, pSrcImg->nWid, pSrcImg->nHig };

	if (m_globlparam[nId].m_nInitSuccess != ALLOC_SUCCESS_STATE) {
		pDstRec->object.x = SrcRec.object.x + MotionVec.x;
		pDstRec->object.y = SrcRec.object.y + MotionVec.y;
		pDstRec->object.width = SrcRec.object.width;
		pDstRec->object.height = SrcRec.object.height;
		limitObjectRectRange(tempRect, &pDstRec->object);
		return 1;
	}

	TemplRec.object.x = SrcRec.object.x + (short)(SrcRec.object.width * fShink);
	TemplRec.object.y = SrcRec.object.y + (short)(SrcRec.object.height * fShink);
	TemplRec.object.width = SrcRec.object.width - (short)(SrcRec.object.width * fShink * 2);
	TemplRec.object.height = SrcRec.object.height - (short)(SrcRec.object.height * fShink * 2);

	SearcRec.object.x = SrcRec.object.x - (short)(SrcRec.object.width * fLager);
	SearcRec.object.y = SrcRec.object.y - (short)(SrcRec.object.height * fLager);
	SearcRec.object.width = SrcRec.object.width + (short)(SrcRec.object.width * fLager * 2);
	SearcRec.object.height = SrcRec.object.height + (short)(SrcRec.object.height * fLager * 2);
	limitObjectRectRange(tempRect, &SearcRec.object);

	if (WS_MIN(TemplRec.object.width,TemplRec.object.height ) < 5) {
		TemplRec = SrcRec;
	}

	TempleImg.data = (unsigned char *) m_globlparam[nId].m_extern_Space;
	nPubliBuffOff = TemplRec.object.width * TemplRec.object.height;

	mvDerivemgRio(pSrcImg, &TempleImg, TemplRec.object);

	bTemplat = matchByTemple(pDstImg, &TempleImg, SearcRec.object, &MatchRec.object,
		nPubliBuffOff, m_globlparam[nId].m_extern_Space);

	if (!bTemplat) {
		pDstRec->object.x = SrcRec.object.x + MotionVec.x;
		pDstRec->object.y = SrcRec.object.y + MotionVec.y;
		pDstRec->object.width = SrcRec.object.width;
		pDstRec->object.height = SrcRec.object.height;
	} else {
		pDstRec->object.x = MatchRec.object.x - (short)(SrcRec.object.width * fShink);
		pDstRec->object.y = MatchRec.object.y - (short)(SrcRec.object.height * fShink);
		pDstRec->object.width = SrcRec.object.width;
		pDstRec->object.height = SrcRec.object.height;
		limitObjectRectRange(tempRect, &pDstRec->object);
	}

	return 1;
}

