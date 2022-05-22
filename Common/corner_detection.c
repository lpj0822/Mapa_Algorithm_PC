#include "corner_detection.h"
#include "sort_algorithm.h"
#include "utils.h"


const static int gaussianKernel[9] = { 115731, 116895, 115731, 116895, 118070, 116895, 115731, 116895, 115731 };

/*
I/O:	    Name		          Type	     		  Content

[in]	    imp		              const unsigned char*	  ptr for the point
[in]	    pointer_dir		      const int*	      integer pointer offstes
[in]	    barrier		          int	              threshold

[out]	    returned		      int	              score

Realized function:
+ caculate the score of fast corner
*/
static int corner_score(const unsigned char*  imp, const int *pointer_dir, int barrier)
{
	/*The score for a positive feature is sum of the difference between the pixels
	and the barrier if the difference is positive. Negative is similar.
	The score is the max of those two.

	B = {x | x = points on the Bresenham circle around c}
	Sp = { I(x) - t | x E B , I(x) - t > 0 }
	Sn = { t - I(x) | x E B, t - I(x) > 0}
	Score = max sum(Sp), sum(Sn)*/

	int cb = *imp + barrier;
	int c_b = *imp - barrier;
	int sp = 0, sn = 0;

	int i = 0;

	for (i = 0; i<16; i++)
	{
		int p = imp[pointer_dir[i]];

		if (p > cb)
			sp += p - cb;
		else if (p < c_b)
			sn += c_b - p;
	}

	if (sp > sn)
		return sp;
	else
		return sn;
}

void fastCornerDetect9_16(const WissenImage* srcImage, const unsigned char* pMask, const WissenRect roi, const int barrier, const int maxCornerCount, int* num_corners, WissenPoint *resultCorners)
{
	int total = 0;
	register int x, y;
	//register int xsizeB=xsize - 10;
	//register int ysizeB=ysize - 10;
	register int offset0, offset1, offset2, offset3, offset4, offset5, offset6, offset7, \
		offset8, offset9, offset10, offset11, offset12, offset13, offset14, offset15;
	register int width;
	register int njump_x;
	int cb;
	int c_b;
	int nStart_x = roi.x;
	int nEnd_x = roi.x + roi.width;
	int nStart_y = roi.y;
	int nEnd_y = roi.y + roi.height;

	njump_x = 16;

	offset0 = (-3) + (0) * srcImage->nWid;
	offset1 = (-3) + (-1) * srcImage->nWid;
	offset2 = (-2) + (-2) * srcImage->nWid;
	offset3 = (-1) + (-3) * srcImage->nWid;
	offset4 = (0) + (-3) * srcImage->nWid;
	offset5 = (1) + (-3) * srcImage->nWid;
	offset6 = (2) + (-2) * srcImage->nWid;
	offset7 = (3) + (-1) * srcImage->nWid;
	offset8 = (3) + (0) * srcImage->nWid;
	offset9 = (3) + (1) * srcImage->nWid;
	offset10 = (2) + (2) * srcImage->nWid;
	offset11 = (1) + (3) * srcImage->nWid;
	offset12 = (0) + (3) * srcImage->nWid;
	offset13 = (-1) + (3) * srcImage->nWid;
	offset14 = (-2) + (2) * srcImage->nWid;
	offset15 = (-3) + (1) * srcImage->nWid;

	width = srcImage->nWid;

	for (y = nStart_y; y < nEnd_y; y++)
	{
		x = nStart_x;
		while (1)
		{
			x++;
			if (x > nEnd_x)
				break;
			else
			{
				register const unsigned char* const p = srcImage->data + y * width + x;
				register const unsigned char* const pmask = pMask + y * width + x;
				if (!*pmask)
				{
					x += njump_x;
					continue;
				}

				cb = *p + barrier;
				c_b = *p - barrier;
				if (p[offset0] > cb)
				if (p[offset2] > cb)
				if (p[offset4] > cb)
				if (p[offset5] > cb)
				if (p[offset7] > cb)
				if (p[offset3] > cb)
				if (p[offset1] > cb)
				if (p[offset6] > cb)
				if (p[offset8] > cb)
				{
				}
				else
				if (p[offset15] > cb)
				{
				}
				else
					continue;
				else
				if (p[offset13] > cb)
				if (p[offset14] > cb)
				if (p[offset15] > cb)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset8] > cb)
				if (p[offset9] > cb)
				if (p[offset10] > cb)
				if (p[offset6] > cb)
				{
				}
				else
				if (p[offset11] > cb)
				if (p[offset12] > cb)
				if (p[offset13] > cb)
				if (p[offset14] > cb)
				if (p[offset15] > cb)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset10] > cb)
				if (p[offset11] > cb)
				if (p[offset12] > cb)
				if (p[offset8] > cb)
				if (p[offset9] > cb)
				if (p[offset6] > cb)
				{
				}
				else
				if (p[offset13] > cb)
				if (p[offset14] > cb)
				if (p[offset15] > cb)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset1] > cb)
				if (p[offset13] > cb)
				if (p[offset14] > cb)
				if (p[offset15] > cb)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset1] > cb)
				if (p[offset13] > cb)
				if (p[offset14] > cb)
				if (p[offset15] > cb)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else if (p[offset7] < c_b)
				if (p[offset14] > cb)
				if (p[offset15] > cb)
				if (p[offset1] > cb)
				if (p[offset3] > cb)
				if (p[offset6] > cb)
				{
				}
				else
				if (p[offset13] > cb)
				{
				}
				else
					continue;
				else
				if (p[offset10] > cb)
				if (p[offset11] > cb)
				if (p[offset12] > cb)
				if (p[offset13] > cb)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset8] > cb)
				if (p[offset9] > cb)
				if (p[offset10] > cb)
				if (p[offset11] > cb)
				if (p[offset12] > cb)
				if (p[offset13] > cb)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else if (p[offset14] < c_b)
				if (p[offset8] < c_b)
				if (p[offset9] < c_b)
				if (p[offset10] < c_b)
				if (p[offset11] < c_b)
				if (p[offset12] < c_b)
				if (p[offset13] < c_b)
				if (p[offset6] < c_b)
				{
				}
				else
				if (p[offset15] < c_b)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset14] > cb)
				if (p[offset15] > cb)
				if (p[offset1] > cb)
				if (p[offset3] > cb)
				if (p[offset6] > cb)
				{
				}
				else
				if (p[offset13] > cb)
				{
				}
				else
					continue;
				else
				if (p[offset10] > cb)
				if (p[offset11] > cb)
				if (p[offset12] > cb)
				if (p[offset13] > cb)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset8] > cb)
				if (p[offset9] > cb)
				if (p[offset10] > cb)
				if (p[offset11] > cb)
				if (p[offset12] > cb)
				if (p[offset13] > cb)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else if (p[offset5] < c_b)
				if (p[offset12] > cb)
				if (p[offset13] > cb)
				if (p[offset14] > cb)
				if (p[offset15] > cb)
				if (p[offset1] > cb)
				if (p[offset3] > cb)
				{
				}
				else
				if (p[offset10] > cb)
				if (p[offset11] > cb)
				{
				}
				else
					continue;
				else
					continue;
				else
				if (p[offset8] > cb)
				if (p[offset9] > cb)
				if (p[offset10] > cb)
				if (p[offset11] > cb)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset6] > cb)
				if (p[offset7] > cb)
				if (p[offset8] > cb)
				if (p[offset9] > cb)
				if (p[offset10] > cb)
				if (p[offset11] > cb)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else if (p[offset12] < c_b)
				if (p[offset7] < c_b)
				if (p[offset8] < c_b)
				if (p[offset9] < c_b)
				if (p[offset10] < c_b)
				if (p[offset11] < c_b)
				if (p[offset13] < c_b)
				if (p[offset6] < c_b)
				{
				}
				else
				if (p[offset14] < c_b)
				if (p[offset15] < c_b)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset12] > cb)
				if (p[offset13] > cb)
				if (p[offset14] > cb)
				if (p[offset15] > cb)
				if (p[offset1] > cb)
				if (p[offset3] > cb)
				{
				}
				else
				if (p[offset10] > cb)
				if (p[offset11] > cb)
				{
				}
				else
					continue;
				else
					continue;
				else
				if (p[offset8] > cb)
				if (p[offset9] > cb)
				if (p[offset10] > cb)
				if (p[offset11] > cb)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset6] > cb)
				if (p[offset7] > cb)
				if (p[offset8] > cb)
				if (p[offset9] > cb)
				if (p[offset10] > cb)
				if (p[offset11] > cb)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else if (p[offset12] < c_b)
				if (p[offset7] < c_b)
				if (p[offset8] < c_b)
				if (p[offset9] < c_b)
				if (p[offset10] < c_b)
				if (p[offset11] < c_b)
				if (p[offset13] < c_b)
				if (p[offset14] < c_b)
				if (p[offset6] < c_b)
				{
				}
				else
				if (p[offset15] < c_b)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else if (p[offset4] < c_b)
				if (p[offset11] > cb)
				if (p[offset12] > cb)
				if (p[offset13] > cb)
				if (p[offset10] > cb)
				if (p[offset14] > cb)
				if (p[offset15] > cb)
				if (p[offset1] > cb)
				{
				}
				else
				if (p[offset8] > cb)
				if (p[offset9] > cb)
				{
				}
				else
					continue;
				else
					continue;
				else
				if (p[offset6] > cb)
				if (p[offset7] > cb)
				if (p[offset8] > cb)
				if (p[offset9] > cb)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset5] > cb)
				if (p[offset6] > cb)
				if (p[offset7] > cb)
				if (p[offset8] > cb)
				if (p[offset9] > cb)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset1] > cb)
				if (p[offset3] > cb)
				if (p[offset14] > cb)
				if (p[offset15] > cb)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else if (p[offset11] < c_b)
				if (p[offset7] < c_b)
				if (p[offset8] < c_b)
				if (p[offset9] < c_b)
				if (p[offset10] < c_b)
				if (p[offset6] < c_b)
				if (p[offset5] < c_b)
				if (p[offset3] < c_b)
				{
				}
				else
				if (p[offset12] < c_b)
				{
				}
				else
					continue;
				else
				if (p[offset12] < c_b)
				if (p[offset13] < c_b)
				if (p[offset14] < c_b)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset12] < c_b)
				if (p[offset13] < c_b)
				if (p[offset14] < c_b)
				if (p[offset15] < c_b)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset11] > cb)
				if (p[offset12] > cb)
				if (p[offset13] > cb)
				if (p[offset10] > cb)
				if (p[offset14] > cb)
				if (p[offset15] > cb)
				if (p[offset1] > cb)
				{
				}
				else
				if (p[offset8] > cb)
				if (p[offset9] > cb)
				{
				}
				else
					continue;
				else
					continue;
				else
				if (p[offset6] > cb)
				if (p[offset7] > cb)
				if (p[offset8] > cb)
				if (p[offset9] > cb)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset5] > cb)
				if (p[offset6] > cb)
				if (p[offset7] > cb)
				if (p[offset8] > cb)
				if (p[offset9] > cb)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset1] > cb)
				if (p[offset3] > cb)
				if (p[offset14] > cb)
				if (p[offset15] > cb)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else if (p[offset11] < c_b)
				if (p[offset7] < c_b)
				if (p[offset8] < c_b)
				if (p[offset9] < c_b)
				if (p[offset10] < c_b)
				if (p[offset12] < c_b)
				if (p[offset13] < c_b)
				if (p[offset6] < c_b)
				if (p[offset5] < c_b)
				{
				}
				else
				if (p[offset14] < c_b)
				{
				}
				else
					continue;
				else
				if (p[offset14] < c_b)
				if (p[offset15] < c_b)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else if (p[offset2] < c_b)
				if (p[offset9] > cb)
				if (p[offset10] > cb)
				if (p[offset11] > cb)
				if (p[offset8] > cb)
				if (p[offset12] > cb)
				if (p[offset13] > cb)
				if (p[offset14] > cb)
				if (p[offset15] > cb)
				{
				}
				else
				if (p[offset6] > cb)
				if (p[offset7] > cb)
				{
				}
				else
					continue;
				else
					continue;
				else
				if (p[offset5] > cb)
				if (p[offset6] > cb)
				if (p[offset7] > cb)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset4] > cb)
				if (p[offset5] > cb)
				if (p[offset6] > cb)
				if (p[offset7] > cb)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset3] > cb)
				if (p[offset4] > cb)
				if (p[offset5] > cb)
				if (p[offset6] > cb)
				if (p[offset7] > cb)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset1] > cb)
				if (p[offset12] > cb)
				if (p[offset13] > cb)
				if (p[offset14] > cb)
				if (p[offset15] > cb)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else if (p[offset9] < c_b)
				if (p[offset7] < c_b)
				if (p[offset8] < c_b)
				if (p[offset6] < c_b)
				if (p[offset5] < c_b)
				if (p[offset4] < c_b)
				if (p[offset3] < c_b)
				if (p[offset1] < c_b)
				{
				}
				else
				if (p[offset10] < c_b)
				{
				}
				else
					continue;
				else
				if (p[offset10] < c_b)
				if (p[offset11] < c_b)
				if (p[offset12] < c_b)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset10] < c_b)
				if (p[offset11] < c_b)
				if (p[offset12] < c_b)
				if (p[offset13] < c_b)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset10] < c_b)
				if (p[offset11] < c_b)
				if (p[offset12] < c_b)
				if (p[offset13] < c_b)
				if (p[offset14] < c_b)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset10] < c_b)
				if (p[offset11] < c_b)
				if (p[offset12] < c_b)
				if (p[offset13] < c_b)
				if (p[offset14] < c_b)
				if (p[offset15] < c_b)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset9] > cb)
				if (p[offset10] > cb)
				if (p[offset11] > cb)
				if (p[offset8] > cb)
				if (p[offset12] > cb)
				if (p[offset13] > cb)
				if (p[offset14] > cb)
				if (p[offset15] > cb)
				{
				}
				else
				if (p[offset6] > cb)
				if (p[offset7] > cb)
				{
				}
				else
					continue;
				else
					continue;
				else
				if (p[offset5] > cb)
				if (p[offset6] > cb)
				if (p[offset7] > cb)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset4] > cb)
				if (p[offset5] > cb)
				if (p[offset6] > cb)
				if (p[offset7] > cb)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset3] > cb)
				if (p[offset4] > cb)
				if (p[offset5] > cb)
				if (p[offset6] > cb)
				if (p[offset7] > cb)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset1] > cb)
				if (p[offset12] > cb)
				if (p[offset13] > cb)
				if (p[offset14] > cb)
				if (p[offset15] > cb)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else if (p[offset9] < c_b)
				if (p[offset7] < c_b)
				if (p[offset8] < c_b)
				if (p[offset10] < c_b)
				if (p[offset11] < c_b)
				if (p[offset6] < c_b)
				if (p[offset5] < c_b)
				if (p[offset4] < c_b)
				if (p[offset3] < c_b)
				{
				}
				else
				if (p[offset12] < c_b)
				{
				}
				else
					continue;
				else
				if (p[offset12] < c_b)
				if (p[offset13] < c_b)
				{
				}
				else
					continue;
				else
					continue;
				else
				if (p[offset12] < c_b)
				if (p[offset13] < c_b)
				if (p[offset14] < c_b)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset12] < c_b)
				if (p[offset13] < c_b)
				if (p[offset14] < c_b)
				if (p[offset15] < c_b)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else if (p[offset0] < c_b)
				if (p[offset2] > cb)
				if (p[offset9] > cb)
				if (p[offset7] > cb)
				if (p[offset8] > cb)
				if (p[offset6] > cb)
				if (p[offset5] > cb)
				if (p[offset4] > cb)
				if (p[offset3] > cb)
				if (p[offset1] > cb)
				{
				}
				else
				if (p[offset10] > cb)
				{
				}
				else
					continue;
				else
				if (p[offset10] > cb)
				if (p[offset11] > cb)
				if (p[offset12] > cb)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset10] > cb)
				if (p[offset11] > cb)
				if (p[offset12] > cb)
				if (p[offset13] > cb)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset10] > cb)
				if (p[offset11] > cb)
				if (p[offset12] > cb)
				if (p[offset13] > cb)
				if (p[offset14] > cb)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset10] > cb)
				if (p[offset11] > cb)
				if (p[offset12] > cb)
				if (p[offset13] > cb)
				if (p[offset14] > cb)
				if (p[offset15] > cb)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else if (p[offset9] < c_b)
				if (p[offset10] < c_b)
				if (p[offset11] < c_b)
				if (p[offset8] < c_b)
				if (p[offset12] < c_b)
				if (p[offset13] < c_b)
				if (p[offset14] < c_b)
				if (p[offset15] < c_b)
				{
				}
				else
				if (p[offset6] < c_b)
				if (p[offset7] < c_b)
				{
				}
				else
					continue;
				else
					continue;
				else
				if (p[offset5] < c_b)
				if (p[offset6] < c_b)
				if (p[offset7] < c_b)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset4] < c_b)
				if (p[offset5] < c_b)
				if (p[offset6] < c_b)
				if (p[offset7] < c_b)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset3] < c_b)
				if (p[offset4] < c_b)
				if (p[offset5] < c_b)
				if (p[offset6] < c_b)
				if (p[offset7] < c_b)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset1] < c_b)
				if (p[offset12] < c_b)
				if (p[offset13] < c_b)
				if (p[offset14] < c_b)
				if (p[offset15] < c_b)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else if (p[offset2] < c_b)
				if (p[offset4] > cb)
				if (p[offset11] > cb)
				if (p[offset7] > cb)
				if (p[offset8] > cb)
				if (p[offset9] > cb)
				if (p[offset10] > cb)
				if (p[offset6] > cb)
				if (p[offset5] > cb)
				if (p[offset3] > cb)
				{
				}
				else
				if (p[offset12] > cb)
				{
				}
				else
					continue;
				else
				if (p[offset12] > cb)
				if (p[offset13] > cb)
				if (p[offset14] > cb)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset12] > cb)
				if (p[offset13] > cb)
				if (p[offset14] > cb)
				if (p[offset15] > cb)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else if (p[offset11] < c_b)
				if (p[offset12] < c_b)
				if (p[offset13] < c_b)
				if (p[offset10] < c_b)
				if (p[offset14] < c_b)
				if (p[offset15] < c_b)
				if (p[offset1] < c_b)
				{
				}
				else
				if (p[offset8] < c_b)
				if (p[offset9] < c_b)
				{
				}
				else
					continue;
				else
					continue;
				else
				if (p[offset6] < c_b)
				if (p[offset7] < c_b)
				if (p[offset8] < c_b)
				if (p[offset9] < c_b)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset5] < c_b)
				if (p[offset6] < c_b)
				if (p[offset7] < c_b)
				if (p[offset8] < c_b)
				if (p[offset9] < c_b)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset1] < c_b)
				if (p[offset3] < c_b)
				if (p[offset14] < c_b)
				if (p[offset15] < c_b)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else if (p[offset4] < c_b)
				if (p[offset5] > cb)
				if (p[offset12] > cb)
				if (p[offset7] > cb)
				if (p[offset8] > cb)
				if (p[offset9] > cb)
				if (p[offset10] > cb)
				if (p[offset11] > cb)
				if (p[offset13] > cb)
				if (p[offset6] > cb)
				{
				}
				else
				if (p[offset14] > cb)
				if (p[offset15] > cb)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else if (p[offset12] < c_b)
				if (p[offset13] < c_b)
				if (p[offset14] < c_b)
				if (p[offset15] < c_b)
				if (p[offset1] < c_b)
				if (p[offset3] < c_b)
				{
				}
				else
				if (p[offset10] < c_b)
				if (p[offset11] < c_b)
				{
				}
				else
					continue;
				else
					continue;
				else
				if (p[offset8] < c_b)
				if (p[offset9] < c_b)
				if (p[offset10] < c_b)
				if (p[offset11] < c_b)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset6] < c_b)
				if (p[offset7] < c_b)
				if (p[offset8] < c_b)
				if (p[offset9] < c_b)
				if (p[offset10] < c_b)
				if (p[offset11] < c_b)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else if (p[offset5] < c_b)
				if (p[offset7] > cb)
				if (p[offset14] > cb)
				if (p[offset8] > cb)
				if (p[offset9] > cb)
				if (p[offset10] > cb)
				if (p[offset11] > cb)
				if (p[offset12] > cb)
				if (p[offset13] > cb)
				if (p[offset6] > cb)
				{
				}
				else
				if (p[offset15] > cb)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else if (p[offset14] < c_b)
				if (p[offset15] < c_b)
				if (p[offset1] < c_b)
				if (p[offset3] < c_b)
				if (p[offset6] < c_b)
				{
				}
				else
				if (p[offset13] < c_b)
				{
				}
				else
					continue;
				else
				if (p[offset10] < c_b)
				if (p[offset11] < c_b)
				if (p[offset12] < c_b)
				if (p[offset13] < c_b)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset8] < c_b)
				if (p[offset9] < c_b)
				if (p[offset10] < c_b)
				if (p[offset11] < c_b)
				if (p[offset12] < c_b)
				if (p[offset13] < c_b)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else if (p[offset7] < c_b)
				if (p[offset3] < c_b)
				if (p[offset1] < c_b)
				if (p[offset6] < c_b)
				if (p[offset8] < c_b)
				{
				}
				else
				if (p[offset15] < c_b)
				{
				}
				else
					continue;
				else
				if (p[offset13] < c_b)
				if (p[offset14] < c_b)
				if (p[offset15] < c_b)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset8] < c_b)
				if (p[offset9] < c_b)
				if (p[offset10] < c_b)
				if (p[offset6] < c_b)
				{
				}
				else
				if (p[offset11] < c_b)
				if (p[offset12] < c_b)
				if (p[offset13] < c_b)
				if (p[offset14] < c_b)
				if (p[offset15] < c_b)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset10] < c_b)
				if (p[offset11] < c_b)
				if (p[offset12] < c_b)
				if (p[offset8] < c_b)
				if (p[offset9] < c_b)
				if (p[offset6] < c_b)
				{
				}
				else
				if (p[offset13] < c_b)
				if (p[offset14] < c_b)
				if (p[offset15] < c_b)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset1] < c_b)
				if (p[offset13] < c_b)
				if (p[offset14] < c_b)
				if (p[offset15] < c_b)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset1] < c_b)
				if (p[offset13] < c_b)
				if (p[offset14] < c_b)
				if (p[offset15] < c_b)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset14] < c_b)
				if (p[offset15] < c_b)
				if (p[offset1] < c_b)
				if (p[offset3] < c_b)
				if (p[offset6] < c_b)
				{
				}
				else
				if (p[offset13] < c_b)
				{
				}
				else
					continue;
				else
				if (p[offset10] < c_b)
				if (p[offset11] < c_b)
				if (p[offset12] < c_b)
				if (p[offset13] < c_b)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset8] < c_b)
				if (p[offset9] < c_b)
				if (p[offset10] < c_b)
				if (p[offset11] < c_b)
				if (p[offset12] < c_b)
				if (p[offset13] < c_b)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset12] > cb)
				if (p[offset7] > cb)
				if (p[offset8] > cb)
				if (p[offset9] > cb)
				if (p[offset10] > cb)
				if (p[offset11] > cb)
				if (p[offset13] > cb)
				if (p[offset14] > cb)
				if (p[offset6] > cb)
				{
				}
				else
				if (p[offset15] > cb)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else if (p[offset12] < c_b)
				if (p[offset13] < c_b)
				if (p[offset14] < c_b)
				if (p[offset15] < c_b)
				if (p[offset1] < c_b)
				if (p[offset3] < c_b)
				{
				}
				else
				if (p[offset10] < c_b)
				if (p[offset11] < c_b)
				{
				}
				else
					continue;
				else
					continue;
				else
				if (p[offset8] < c_b)
				if (p[offset9] < c_b)
				if (p[offset10] < c_b)
				if (p[offset11] < c_b)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset6] < c_b)
				if (p[offset7] < c_b)
				if (p[offset8] < c_b)
				if (p[offset9] < c_b)
				if (p[offset10] < c_b)
				if (p[offset11] < c_b)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset11] > cb)
				if (p[offset7] > cb)
				if (p[offset8] > cb)
				if (p[offset9] > cb)
				if (p[offset10] > cb)
				if (p[offset12] > cb)
				if (p[offset13] > cb)
				if (p[offset6] > cb)
				if (p[offset5] > cb)
				{
				}
				else
				if (p[offset14] > cb)
				{
				}
				else
					continue;
				else
				if (p[offset14] > cb)
				if (p[offset15] > cb)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else if (p[offset11] < c_b)
				if (p[offset12] < c_b)
				if (p[offset13] < c_b)
				if (p[offset10] < c_b)
				if (p[offset14] < c_b)
				if (p[offset15] < c_b)
				if (p[offset1] < c_b)
				{
				}
				else
				if (p[offset8] < c_b)
				if (p[offset9] < c_b)
				{
				}
				else
					continue;
				else
					continue;
				else
				if (p[offset6] < c_b)
				if (p[offset7] < c_b)
				if (p[offset8] < c_b)
				if (p[offset9] < c_b)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset5] < c_b)
				if (p[offset6] < c_b)
				if (p[offset7] < c_b)
				if (p[offset8] < c_b)
				if (p[offset9] < c_b)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset1] < c_b)
				if (p[offset3] < c_b)
				if (p[offset14] < c_b)
				if (p[offset15] < c_b)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset9] > cb)
				if (p[offset7] > cb)
				if (p[offset8] > cb)
				if (p[offset10] > cb)
				if (p[offset11] > cb)
				if (p[offset6] > cb)
				if (p[offset5] > cb)
				if (p[offset4] > cb)
				if (p[offset3] > cb)
				{
				}
				else
				if (p[offset12] > cb)
				{
				}
				else
					continue;
				else
				if (p[offset12] > cb)
				if (p[offset13] > cb)
				{
				}
				else
					continue;
				else
					continue;
				else
				if (p[offset12] > cb)
				if (p[offset13] > cb)
				if (p[offset14] > cb)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset12] > cb)
				if (p[offset13] > cb)
				if (p[offset14] > cb)
				if (p[offset15] > cb)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else if (p[offset9] < c_b)
				if (p[offset10] < c_b)
				if (p[offset11] < c_b)
				if (p[offset8] < c_b)
				if (p[offset12] < c_b)
				if (p[offset13] < c_b)
				if (p[offset14] < c_b)
				if (p[offset15] < c_b)
				{
				}
				else
				if (p[offset6] < c_b)
				if (p[offset7] < c_b)
				{
				}
				else
					continue;
				else
					continue;
				else
				if (p[offset5] < c_b)
				if (p[offset6] < c_b)
				if (p[offset7] < c_b)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset4] < c_b)
				if (p[offset5] < c_b)
				if (p[offset6] < c_b)
				if (p[offset7] < c_b)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset3] < c_b)
				if (p[offset4] < c_b)
				if (p[offset5] < c_b)
				if (p[offset6] < c_b)
				if (p[offset7] < c_b)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset1] < c_b)
				if (p[offset12] < c_b)
				if (p[offset13] < c_b)
				if (p[offset14] < c_b)
				if (p[offset15] < c_b)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset7] > cb)
				if (p[offset8] > cb)
				if (p[offset9] > cb)
				if (p[offset6] > cb)
				if (p[offset5] > cb)
				if (p[offset4] > cb)
				if (p[offset3] > cb)
				if (p[offset2] > cb)
				if (p[offset1] > cb)
				{
				}
				else
				if (p[offset10] > cb)
				{
				}
				else
					continue;
				else
				if (p[offset10] > cb)
				if (p[offset11] > cb)
				{
				}
				else
					continue;
				else
					continue;
				else
				if (p[offset10] > cb)
				if (p[offset11] > cb)
				if (p[offset12] > cb)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset10] > cb)
				if (p[offset11] > cb)
				if (p[offset12] > cb)
				if (p[offset13] > cb)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset10] > cb)
				if (p[offset11] > cb)
				if (p[offset12] > cb)
				if (p[offset13] > cb)
				if (p[offset14] > cb)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset10] > cb)
				if (p[offset11] > cb)
				if (p[offset12] > cb)
				if (p[offset13] > cb)
				if (p[offset14] > cb)
				if (p[offset15] > cb)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else if (p[offset7] < c_b)
				if (p[offset8] < c_b)
				if (p[offset9] < c_b)
				if (p[offset6] < c_b)
				if (p[offset5] < c_b)
				if (p[offset4] < c_b)
				if (p[offset3] < c_b)
				if (p[offset2] < c_b)
				if (p[offset1] < c_b)
				{
				}
				else
				if (p[offset10] < c_b)
				{
				}
				else
					continue;
				else
				if (p[offset10] < c_b)
				if (p[offset11] < c_b)
				{
				}
				else
					continue;
				else
					continue;
				else
				if (p[offset10] < c_b)
				if (p[offset11] < c_b)
				if (p[offset12] < c_b)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset10] < c_b)
				if (p[offset11] < c_b)
				if (p[offset12] < c_b)
				if (p[offset13] < c_b)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset10] < c_b)
				if (p[offset11] < c_b)
				if (p[offset12] < c_b)
				if (p[offset13] < c_b)
				if (p[offset14] < c_b)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
				if (p[offset10] < c_b)
				if (p[offset11] < c_b)
				if (p[offset12] < c_b)
				if (p[offset13] < c_b)
				if (p[offset14] < c_b)
				if (p[offset15] < c_b)
				{
				}
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
				else
					continue;
			}


			if (total >= maxCornerCount)
			{
				goto mv_fast_end;
			}


			resultCorners[total].x = x;
			resultCorners[total].y = y;
			total++;
			x += 5;
		}
	}

mv_fast_end:
	*num_corners = total;

}


WissenPoint*  fast_nonmax(const WissenImage* srcImage, WissenPoint* corners, int numcorners, int barrier, int* numnx,
	int *pRowStart, int * pScore, WissenPoint *pXYNoMax)
{

	/*Create a list of integer pointer offstes, corresponding to the */
	/*direction offsets in dir[]*/
	int	pointer_dir[16];

#ifdef MV_ADAS_USE_FAST_STATIC
	int *row_start = pRowStart;
	int *scores = pScore;
	WissenPoint*  nonmax_corners = pXYNoMax;
#else
	int* row_start = (int*)my_malloc(srcImage->nHig * sizeof(int));
	int* scores = (int*)my_malloc(numcorners * sizeof(int));
	WissenPoint*  nonmax_corners = (WissenPoint*)my_malloc(numcorners* sizeof(WissenPoint));
#endif

	int num_nonmax = 0;
	int prev_row = -1;
	int i, j;
	int point_above = 0;
	int point_below = 0;

	pointer_dir[0] = 0 + 3 * srcImage->nWid;
	pointer_dir[1] = 1 + 3 * srcImage->nWid;
	pointer_dir[2] = 2 + 2 * srcImage->nWid;
	pointer_dir[3] = 3 + 1 * srcImage->nWid;
	pointer_dir[4] = 3 + 0 * srcImage->nWid;
	pointer_dir[5] = 3 + -1 * srcImage->nWid;
	pointer_dir[6] = 2 + -2 * srcImage->nWid;
	pointer_dir[7] = 1 + -3 * srcImage->nWid;
	pointer_dir[8] = 0 + -3 * srcImage->nWid;
	pointer_dir[9] = -1 + -3 * srcImage->nWid;
	pointer_dir[10] = -2 + -2 * srcImage->nWid;
	pointer_dir[11] = -3 + -1 * srcImage->nWid;
	pointer_dir[12] = -3 + 0 * srcImage->nWid;
	pointer_dir[13] = -3 + 1 * srcImage->nWid;
	pointer_dir[14] = -2 + 2 * srcImage->nWid;
	pointer_dir[15] = -1 + 3 * srcImage->nWid;

	if (numcorners < 5)
	{
#ifndef MV_ADAS_USE_FAST_STATIC
		my_free(row_start, srcImage->nHig * sizeof(int));
		my_free(scores, numcorners * sizeof(int));
		my_free(nonmax_corners);
#endif
		return 0;
	}

	/*srcImage->nWid srcImage->nHig numcorners corners*/

	/*Compute the score for each detected corner, and find where each row begins*/
	/* (the corners are output in raster scan order). A beginning of -1 signifies*/
	/* that there are no corners on that row.*/


	for (i = 0; i < srcImage->nHig; i++)
		row_start[i] = -1;


	for (i = 0; i< numcorners; i++)
	{
		if (corners[i].y != prev_row)
		{
			row_start[corners[i].y] = i;
			prev_row = corners[i].y;
		}

		scores[i] = corner_score(srcImage->data + corners[i].x + corners[i].y * srcImage->nWid, pointer_dir, barrier);
	}


	/*Point above points (roughly) to the pixel above the one of interest, if there*/
	/*is a feature there.*/

	for (i = 1; i < numcorners - 1; i++)
	{
		int score = scores[i];
		WissenPoint pos = corners[i];

		//Check left 
		if (corners[i - 1].x == pos.x - 1 && corners[i - 1].y == pos.y && scores[i - 1] > score)
			continue;

		//Check right
		if (corners[i + 1].x == pos.x + 1 && corners[i + 1].y == pos.y && scores[i + 1] > score)
			continue;

		//Check above (if there is a valid row above)
		if (pos.y != 0 && row_start[pos.y - 1] != -1)
		{
			//Make sure that current point_above is one
			//row above.
			if (corners[point_above].y < pos.y - 1)
				point_above = row_start[pos.y - 1];

			//Make point_above point to the first of the pixels above the current point,
			//if it exists.
			for (; corners[point_above].y < pos.y && corners[point_above].x < pos.x - 1; point_above++)
			{
			}


			for (j = point_above; corners[j].y < pos.y && corners[j].x <= pos.x + 1; j++)
			{
				int x = corners[j].x;
				if ((x == pos.x - 1 || x == pos.x || x == pos.x + 1) && (scores[j] > score))
					goto cont;
			}

		}

		//Check below (if there is anything below)
		if (pos.y != srcImage->nHig - 1 && row_start[pos.y + 1] != -1 && point_below < numcorners) //Nothing below
		{
			if (corners[point_below].y < pos.y + 1)
				point_below = row_start[pos.y + 1];

			// Make point below point to one of the pixels belowthe current point, if it
			// exists.
			for (; point_below < numcorners && corners[point_below].y == pos.y + 1 && corners[point_below].x < pos.x - 1; point_below++)
			{
			}

			for (j = point_below; j < numcorners && corners[j].y == pos.y + 1 && corners[j].x <= pos.x + 1; j++)
			{
				int x = corners[j].x;
				if ((x == pos.x - 1 || x == pos.x || x == pos.x + 1) && (scores[j] >score))
					goto cont;
			}
		}

		nonmax_corners[num_nonmax].x = corners[i].x;
		nonmax_corners[num_nonmax].y = corners[i].y;

		num_nonmax++;

	cont:
		;
	}

	*numnx = num_nonmax;

#ifndef MV_ADAS_USE_FAST_STATIC
	my_free(row_start, srcImage->nHig * sizeof(int));
	my_free(scores, numcorners * sizeof(int));
#endif
	return nonmax_corners;
}

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in]	    srcImg		          const WissenImage*  input image
[in/out]	harris_response		  AdasCorner*		  points.
[in]	    num		              int		          Num of points.

Realized function:
    + caculate the Harris response for input points, given the value of harris_response.val
*/
void CalculateHarrisResponse(const WissenImage *srcImage, AdasCorner *harris_response, const int num)
{
	const unsigned char* imageData = srcImage->data;
	const int width = srcImage->nWid;
	const int height = srcImage->nHig;
	int i, j, count;
	int c0, c1, c2, cr;
	int dx0, dx1, dx2, dx3, dx4, dx5, dx6, dx7, dx8;
	int dy0, dy1, dy2, dy3, dy4, dy5, dy6, dy7, dy8;

	unsigned char ImgT2L2, ImgT2L1, ImgT2L0;
	unsigned char ImgT2R2, ImgT2R1;

	unsigned char ImgT1L2, ImgT1L1, ImgT1L0;
	unsigned char ImgT1R2, ImgT1R1;

	unsigned char ImgB1L2, ImgB1L1, ImgB1L0;
	unsigned char ImgB1R2, ImgB1R1;

	unsigned char ImgB2L2, ImgB2L1, ImgB2L0;
	unsigned char ImgB2R2, ImgB2R1;

	unsigned char ImgL2, ImgL1, ImgL0;
	unsigned char ImgR2, ImgR1;

	int CurHig;
	int CurHigUp2;
	int CurHigUp1;
	int CurHiglow1;
	int CurHiglow2;
	long nMidVal;

	for (count = 0; count < num; ++count)
	{
		j = harris_response[count].point.y;
		i = harris_response[count].point.x;

		CurHig = j * width + i;
		CurHigUp1 = CurHig - width;
		CurHigUp2 = CurHigUp1 - width;
		CurHiglow1 = CurHig + width;
		CurHiglow2 = CurHiglow1 + width;


		ImgT2L2 = *(imageData + CurHigUp2 - 2);
		ImgT2L1 = *(imageData + CurHigUp2 - 1);
		ImgT2L0 = *(imageData + CurHigUp2);
		ImgT2R2 = *(imageData + CurHigUp2 + 2);
		ImgT2R1 = *(imageData + CurHigUp2 + 1);


		ImgT1L2 = *(imageData + CurHigUp1 - 2);
		ImgT1L1 = *(imageData + CurHigUp1 - 1);
		ImgT1L0 = *(imageData + CurHigUp1);
		ImgT1R2 = *(imageData + CurHigUp1 + 2);
		ImgT1R1 = *(imageData + CurHigUp1 + 1);

		ImgB1L2 = *(imageData + CurHiglow1 - 2);
		ImgB1L1 = *(imageData + CurHiglow1 - 1);
		ImgB1L0 = *(imageData + CurHiglow1);
		ImgB1R2 = *(imageData + CurHiglow1 + 2);
		ImgB1R1 = *(imageData + CurHiglow1 + 1);

		ImgB2L2 = *(imageData + CurHiglow2 - 2);
		ImgB2L1 = *(imageData + CurHiglow2 - 1);
		ImgB2L0 = *(imageData + CurHiglow2);
		ImgB2R2 = *(imageData + CurHiglow2 + 2);
		ImgB2R1 = *(imageData + CurHiglow2 + 1);

		ImgL2 = *(imageData + CurHig - 2);
		ImgL1 = *(imageData + CurHig - 1);
		ImgL0 = *(imageData + CurHig);
		ImgR2 = *(imageData + CurHig + 2);
		ImgR1 = *(imageData + CurHig + 1);

		dx0 = -ImgT2L2 - (ImgT1L2 << 1) - ImgL2 + ImgT2L0 + (ImgT1L0 << 1) + ImgL0;
		dx0 = dx0 >> 2;

		/*dy0   =  -*(imageData+width*(j-2)+i-2)-*(imageData+width*(j-2)+i-1)*2-*(imageData+width*(j-2)+i)
		+*(imageData+width*j+i-2)+*(imageData+width*j+i-1)*2+*(imageData+width*j+i);*/

		dy0 = -ImgT2L2 - (ImgT2L1 << 1) - ImgT2L0 + ImgL2 + (ImgL1 << 1) + ImgL0;

		dy0 = dy0 >> 2;

		nMidVal = dy0 * gaussianKernel[0];
		c0 = (dx0*dx0*gaussianKernel[0]) >> 20;
		c1 = (dy0*nMidVal) >> 20;
		c2 = (dx0*nMidVal) >> 20;

		/*dx1   = -*(imageData+width*(j-2)+i-1)-*(imageData+width*(j-1)+i-1)*2-*(imageData+width*j+i-1)
		+*(imageData+width*(j-2)+i+1)+*(imageData+width*(j-1)+i+1)*2+*(imageData+width*j+i+1);*/

		dx1 = -ImgT2L1 - (ImgT1L1 << 1) - ImgL1 + ImgT2R1 + (ImgT1R1 << 1) + ImgR1;

		dx1 = dx1 >> 2;

		/*dy1   =  -*(imageData+width*(j-2)+i-1)-*(imageData+width*(j-2)+i)*2-*(imageData+width*(j-2)+i+1)
		+*(imageData+width*j+i-1)+*(imageData+width*j+i)*2+*(imageData+width*j+i+1);*/
		dy1 = -ImgT2L1 - (ImgT2L0 << 1) - ImgT2R1 + ImgL1 + (ImgL0 << 1) + ImgR1;

		dy1 = dy1 >> 2;

		nMidVal = dy1 * gaussianKernel[1];
		c0 += (dx1*dx1*gaussianKernel[1]) >> 20;
		c1 += (dy1*nMidVal) >> 20;
		c2 += (dx1*nMidVal) >> 20;


		dx2 = -ImgT2L0 - (ImgT1L0 << 1) - ImgL0 + ImgT2R2 + (ImgT1R2 << 1) + ImgR2;

		dx2 = dx2 >> 2;


		dy2 = -ImgT2L0 - (ImgT2R1 << 1) - ImgT2R2 + ImgL0 + (ImgR1 << 1) + ImgR2;

		dy2 = dy2 >> 2;

		nMidVal = dy2 * gaussianKernel[2];
		c0 += (dx2*dx2*gaussianKernel[2]) >> 20;
		c1 += (dy2*nMidVal) >> 20;
		c2 += (dx2*nMidVal) >> 20;


		dx3 = -ImgT1L2 - (ImgL2 << 1) - ImgB1L2 + ImgT1L0 + (ImgL0 << 1) + ImgB1L0;

		dx3 = dx3 >> 2;


		dy3 = -ImgT1L2 - (ImgT1L1 << 1) - ImgT1L0 + ImgB1L2 + (ImgB1L1 << 1) + ImgB1L0;

		dy3 = dy3 >> 2;

		nMidVal = dy3 * gaussianKernel[3];
		c0 += (dx3*dx3*gaussianKernel[3]) >> 20;
		c1 += (dy3*nMidVal) >> 20;
		c2 += (dx3*nMidVal) >> 20;


		dx4 = -ImgT1L1 - (ImgL1 << 1) - ImgB1L1 + ImgT1R1 + (ImgR1 << 1) + ImgB1R1;

		dx4 = dx4 >> 2;

		dy4 = -ImgT1L1 - (ImgT1L0 << 1) - ImgT1R1 + ImgB1L1 + (ImgB1L0 << 1) + ImgB1R1;
		dy4 = dy4 >> 2;

		nMidVal = dy4 * gaussianKernel[4];
		c0 += (dx4*dx4*gaussianKernel[4]) >> 20;
		c1 += (dy4*nMidVal) >> 20;
		c2 += (dx4*nMidVal) >> 20;


		dx5 = -ImgT1L0 - (ImgL0 << 1) - ImgB1L0 + ImgT1R2 + (ImgR2 << 1) + ImgB1R2;

		dx5 = dx5 >> 2;


		dy5 = -ImgT1L0 - (ImgT1R1 << 1) - ImgT1R2 + ImgB1L0 + (ImgB1R1 << 1) + ImgB1R2;
		dy5 = dy5 >> 2;

		nMidVal = dy5 * gaussianKernel[5];
		c0 += (dx5*dx5*gaussianKernel[5]) >> 20;
		c1 += (dy5*nMidVal) >> 20;
		c2 += (dx5*nMidVal) >> 20;


		dx6 = -ImgL2 - (ImgB1L2 << 1) - ImgB2L2 + ImgL0 + (ImgB1L0 << 1) + ImgB2L0;

		dx6 = dx6 >> 2;
		dy6 = -ImgL2 - (ImgL1 << 1) - ImgL0 + ImgB2L2 + (ImgB2L1 << 1) + ImgB2L0;
		dy6 = dy6 >> 2;

		nMidVal = dy6 * gaussianKernel[6];
		c0 += (dx6*dx6*gaussianKernel[6]) >> 20;
		c1 += (dy6*nMidVal) >> 20;
		c2 += (dx6*nMidVal) >> 20;


		dx7 = -ImgL1 - (ImgB1L1 << 1) - ImgB2L1 + ImgR1 + (ImgB1R1 << 1) + ImgB2R1;

		dx7 = dx7 >> 2;
		dy7 = -ImgL1 - (ImgL0 << 1) - ImgR1 + ImgB2L1 + (ImgB2L0 << 1) + ImgB2R1;
		dy7 = dy7 >> 2;

		nMidVal = dy7 * gaussianKernel[7];
		c0 += (dx7*dx7*gaussianKernel[7]) >> 20;
		c1 += (dy7*nMidVal) >> 20;
		c2 += (dx7*nMidVal) >> 20;

		dx8 = -ImgL0 - (ImgB1L0 << 1) - ImgB2L0 + ImgR2 + (ImgB1R2 << 1) + ImgB2R2;

		dx8 = dx8 >> 2;
		dy8 = -ImgL0 - (-ImgR1 << 1) - ImgR2 + ImgB2L0 + (ImgB2R1 << 1) + ImgB2R2;
		dy8 = dy8 >> 2;

		nMidVal = dy8 * gaussianKernel[8];
		c0 += (dx8*dx8*gaussianKernel[8]) >> 20;
		c1 += (dy8*nMidVal) >> 20;
		c2 += (dx8*nMidVal) >> 20;

		cr = (int)(c0*c1 - c2*c2 - (c0 + c1)*(((c0 + c1) * 1311) >> 15));//0.04

		harris_response[count].val = cr;
	}
}

/*
Function process:
+ sort the corner in decreasing order by corner_pass.val and update corner_max
Fan-in :
+ binSort_Corner()
Fan-out:
+ N/A
ATTENTION: __________
*/
void CornerResponseRestrain(AdasCorner *corner_pass, AdasCorner *corner_max, int num_pass, int *num_max, int max_num)
{
	int i;

	if (num_pass <= max_num)
	{
		for (i = 0; i < num_pass; ++i)
		{
			corner_max[i] = corner_pass[i];
			corner_max[i].State.nMacthNum = 0;
		}
		*num_max = num_pass;
	}
	else
	{

		binSort_Corner(corner_pass, num_pass);
		for (i = 0; i < max_num; ++i)
		{
			corner_max[i] = corner_pass[i];
			corner_max[i].State.nMacthNum = 0;
		}

		*num_max = max_num;
	}
}