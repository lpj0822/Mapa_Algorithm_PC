/**************************************************************************************************************
Copyright © Wissen Intelligent Sensing Technology Co., Ltd. 2011-2017. All rights reserved.
File name: FCWSD_Interface.h
Version: 1.0		Date: 2017-02-28		Author: Yanming Wang		ID: 1407930

Description:
	The functions in this file are defined to cluster and merge the rects of detector.
	The following function types are included:
	+ getGroupedRectanglesCar(): Clustered and merge the deteced list of rects
	+ predicateCar(): Decide two rects are adjacent
	+ partitionCar: Clustered the list of rects.
Deviation:

History:
	+ Version: 1.0		Date: 2017-02-28		Author: Yanming Wang	ID: 1407930
	  Modification: Coding Standards are added.
**************************************************************************************************************/

#include "group_rect.h"
#include "clustering_rect.h"

/*
Function process:
	+ Clustered and merge the deteced list of rects.
	Fan-in : 
	        + lbpGroupWindowsCar()()
	Fan-out: 
	        + clusteredRects()

	ATTENTION: 
*/
int getGroupedRectanglesCar(FCWSDetectorGlobalPara *pVehicleDetor, WissenObjectRect *rectList, int *prectNum, const int groupThreshold, const float eps)
{
	unsigned char *pTr = NULL;
	
	int *rWeights = NULL;
	int *labels   = NULL;

	WissenObjectRect *rRects = NULL;

	int classesNum, cls, n1, n2, dx, dy, i, j, nLabels;
	
	float s; 
	WissenObjectRect r, r1, r2;
	
	classesNum = clusterRects(rectList, *prectNum, eps, pVehicleDetor->pTreeNode, pVehicleDetor->pLabNode);
	//classesNum = partitionCar(pVehicleDetor, rectList, *prectNum, eps);

	rRects = (WissenObjectRect*)pVehicleDetor->pbulicBuff;
	pTr = ALIGN_16BYTE(rRects + classesNum );
	memset(rRects, 0, classesNum * sizeof(WissenObjectRect));

	rWeights = (int *)pTr;
	pTr = ALIGN_16BYTE(rWeights  +  classesNum ); 
	memset(rWeights,0,classesNum * sizeof(int) );

    nLabels = *prectNum;
	labels = pVehicleDetor->pLabNode;

    for (i = 0; i < nLabels; i++)
	{
        cls = labels[i];
        rRects[cls].x		+= rectList[i].x;
        rRects[cls].y	    += rectList[i].y;
        rRects[cls].width	+= rectList[i].width;
		rRects[cls].height	+= rectList[i].height;
        rWeights[cls]++;
    }
    for (i = 0; i < classesNum; i++) 
	{
		r = rRects[i];
		s = 1.f / rWeights[i];
		rRects[i].x = WS_MyRound(r.x*s);
		rRects[i].y = WS_MyRound(r.y*s);
		rRects[i].width = WS_MyRound(r.width*s);
		rRects[i].height = WS_MyRound(r.height*s);
		rRects[i].confidence = rWeights[i];
    }

    *prectNum = 0;

	for (i = 0; i < classesNum; i++) 
	{
		r1 = rRects[i];
		n1 = rWeights[i];
		if( n1 < groupThreshold )
		{
			continue;
		}

		/* filter out small car rectangles inside large rectangles */
		for (j = 0; j < classesNum; j++) 
		{
			n2 = rWeights[j];
			/* if it is the same rectangle, or the number of rectangles in class j is < group threshold, 
			 do nothing */
			if (j == i || n2 <= groupThreshold)
			{
				continue;
			}
			r2 = rRects[j];

			/*dx = WS_MyRound( r2.width * eps );
			dy = WS_MyRound( r2.height * eps );

			if (i != j &&
				r1.x >= r2.x - dx &&
				r1.y >= r2.y - dy &&
				r1.x + r1.width <= r2.x + r2.width + dx &&
				r1.y + r1.height <= r2.y + r2.height + dy &&
			(n2 > WS_MAX(3, n1) || n1 < 3))
			{
				break;
			}*/


			dx = WS_MyRound(r2.width * 0.3);
			dy = WS_MyRound(r2.height * 0.3);

			if (i != j &&
				r1.x >= (r2.x) &&
				r1.x <= r2.x + dx &&
				r1.y >= (r2.y) &&
				r1.y <= r2.y + dy &&
				r1.x + r1.width <= r2.x + r2.width  &&
				r1.x + r1.width >= r2.x + r2.width - dx &&
				r1.y + r1.height <= r2.y + r2.height &&
				r1.y + r1.height >= r2.y + r2.height - dy && 
			(n2 > WS_MAX(3, n1) || n1 < 3))
			{
				break;
			}


			dx = WS_MyRound(r1.width * 0.3);
			dy = WS_MyRound(r1.height * 0.3);

			if (i != j &&
				r2.x >= (r1.x) &&
				r2.x <= r1.x+dx &&
				r2.y >= (r1.y) &&
				r2.y<=r1.y +dy &&
				r2.x + r2.width <= r1.x + r1.width  &&
				r2.x + r2.width >= r1.x + r1.width-dx &&
				r2.y + r2.height <= r1.y + r1.height &&
				r2.y + r2.height >= r1.y + r1.height-dy && 
			(n2 >= WS_MAX(3, n1) || n1 < 3))
			{
				break;
			}
		}

		if (j == classesNum) 
		{
			rectList[(*prectNum)++] = r1;
		}
	}


    return 0;
}
