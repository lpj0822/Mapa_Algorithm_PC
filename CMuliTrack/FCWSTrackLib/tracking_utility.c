#include "tracking_utility.h"
#include "utility_function.h"

#ifdef TAIL_LIGHT_DEBGU
void cvtUcharToMat(unsigned char *image, int width, int height, cv::Mat &dst)
{
	int index = 0;
	for (int row = 0; row < height; row++)
	{
		for (int col = 0; col < width; col++)
		{
			if (image[index] != 0)
			{
				dst.at<uchar>(row, col) = 255;
			}
			else
			{
				dst.at<uchar>(row, col) = 0;
			}
			index++;
		}
	}
}
#endif //TAIL_LIGHT_DEBGU

/*
Function process:
+ Remove the overlapped rects in pInPutParam.
*/
void removeOverlappedRects(const int objectCount, WissenObjectRectTracked *trackingObjects)
{
	int i, j;
	WissenObjectRectTracked *pSrcRec = 0;
	WissenObjectRectTracked *pDstRec = 0;

	for (i = 0; i < objectCount - 1; i++)
	{
		pSrcRec = trackingObjects + i;

		if (pSrcRec->object.width == 0)
			continue;

		for (j = i + 1; j < objectCount; j++)
		{
			pDstRec = trackingObjects + j;

			if (pDstRec->object.width == 0)
				continue;

			if (isRectOverlapped(&pSrcRec->object, &pDstRec->object, 0.7f) || isRectOverlapped(&pDstRec->object, &pSrcRec->object, 0.7f))
			{
				if (pSrcRec->object.confidence < pDstRec->object.confidence)
				{
					pSrcRec->object.width = 0;
				}
				else
				{
					pDstRec->object.width = 0;
				}
			}
		}
	}
}

/*
I/O:	    Name		        Type	     		  Content

[in]	    pA		            const WissenSystime*        time A
[in]	    pB		            const WissenSystime*        time B
[in/out]	pC		            WissenSystime*		      time C

Realized function:
+ caculate the time difference
*/
void mvSubTime(const WissenSystime *pA, const WissenSystime *pB, WissenSystime *pC) 
{
	pC->wHour = pA->wHour - pB->wHour;
	pC->wMin = pA->wMin - pB->wMin;
	pC->wSec = pA->wSec - pB->wSec;
	pC->wMilSec = pA->wMilSec - pB->wMilSec;

}