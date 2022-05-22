#include "utility_function.h"
#include <math.h>

/*
I/O:	    Name		    Type	     		  Content

[in]	    A		        WissenPoint*		      point A.
[in]	    B		        WissenPoint*		      point B.
[out]       returned        float             the pow of norm of point A and B

Realized function:
+ caculate the pow of norm of point A and B;
*/
int mvDisPow(WissenPoint *A, WissenPoint *B)
{
	int ndis;

	ndis = (A->x - B->x) * (A->x - B->x) + (A->y - B->y) * (A->y - B->y);

	return ndis;
}

/*
I/O:	    Name		    Type	     		  Content

[in]	    A		        WissenPoint*		      point A.
[in]	    B		        WissenPoint*		      point B.
[out]       returned        float             the norm of point A and B

Realized function:
+ caculate the norm of point A and B;
*/
float mvNormPoint(WissenPoint *A, WissenPoint *B)
{
	float fdis;

	fdis = (float)sqrt((float)(A->x - B->x) * (A->x - B->x) + (A->y - B->y) * (A->y - B->y));

	return fdis;
}

/*
I/O:	    Name		    Type	     		  Content

[in]	    pVec		    WissenPoint*		      Vector.
[out]       returned        float             the norm of Vector

Realized function:
+ caculate the norm of Vector;
*/
float mvNormVec(WissenPoint *pVec)
{
	float fdis;

	fdis = (float)sqrt((float)pVec->x * pVec->x + pVec->y * pVec->y);

	return fdis;
}

/*
I/O:	    Name		    Type	     		        Content

[in]	    point		    const WissenPoint*	        point
[in]	    rec		        const WissenObjectRect*	    rect.

[out]	    returned        unsigned char		        if point in rect return 1; else return 0.

Realized function:
+ if point in rect return 1; else return 0.
*/
unsigned char isPointInRect(const WissenPoint *point, const WissenObjectRect *rec)
{
	if (point->x > rec->x && point->x < rec->x + rec->width \
		&& point->y > rec->y && point->y < rec->y + rec->height)
	{
		return 1;
	}

	return 0;
}

/*
I/O:	    Name		          Type	     		  Content

[in/out]	dstRect		          WissenObjectRect*		      limit Rect.
[in]	    rect		          WissenObjectRect*		      Rect.

Realized function:
+ make sure dstRect inside of rect;
*/
void limitObjectRectRange(const WissenObjectRect rect, WissenObjectRect *dstRect)
{
	if (dstRect->x < rect.x)
	{
		dstRect->x = rect.x;
	}

	if (dstRect->y < rect.y)
	{
		dstRect->y = rect.y;
	}

	if (dstRect->x + dstRect->width > rect.x + rect.width)
	{
		dstRect->width = rect.x + rect.width - 1 - dstRect->x;
	}

	if (dstRect->y + dstRect->height > rect.y + rect.height)
	{
		dstRect->height = rect.y + rect.height - 1 - dstRect->y;
	}
}

/*
I/O:	    Name		          Type	     		  Content

[in]	    srcRec		          AdasRect		      input rect.
[in]	    pDstRec		          AdasRect		      output rect.
[in]	    fShrinkRate		      float		      shrinking factor.

Realized function:
+ do the shrinking for srcRec and get pDstRec
*/
void mvShinkRect(WissenObjectRect srcRec, WissenObjectRect *pDstRec, float fShrinkRate)
{
	pDstRec->x = srcRec.x + (short)(fShrinkRate * srcRec.width);
	pDstRec->y = srcRec.y + (short)(fShrinkRate * srcRec.height);
	pDstRec->width = srcRec.width - (short)(fShrinkRate * (srcRec.width << 1));
	pDstRec->height = srcRec.height - (short)(fShrinkRate * (srcRec.height << 1));
}

/*
I/O:	    Name		          Type	     		          Content

[in]	    pDetRecw		      const AdasRect*		      Rect1.
[in]	    pGroupRec		      const   AdasRect*		      Rect2.
[in/out]	LapRec		          AdasRect*		              overlap rect.

[out]	    returned              unsigned char		              if overlapped return 1;else return 0.

Realized function:
+ if psrc and pdst are overlapped, return 1 and LapRec is the overlapped region; else return 0;
*/
unsigned char computeRectOverlappedRegion(const WissenObjectRect *pDetRecw, const WissenObjectRect *pGroupRec, WissenObjectRect *LapRec)
{
	int Lapwidth = pDetRecw->width + pGroupRec->width - (WS_MAX(pDetRecw->x + pDetRecw->width, pGroupRec->x + pGroupRec->width) \
		- WS_MIN(pDetRecw->x, pGroupRec->x));
	int Lapheigt = pDetRecw->height + pGroupRec->height - (WS_MAX(pDetRecw->y + pDetRecw->height, pGroupRec->y + pGroupRec->height) \
		- WS_MIN(pDetRecw->y, pGroupRec->y));

	if (Lapwidth > 0 && Lapheigt > 0)
	{
		LapRec->width = Lapwidth;
		LapRec->height = Lapheigt;
		LapRec->x = WS_MAX(pGroupRec->x, pDetRecw->x);
		LapRec->y = WS_MAX(pGroupRec->y, pDetRecw->y);
		return 1;
	}

	return 0;

}

/*
I/O:	    Name		          Type	     		  Content

[in]	    psrc		          AdasRect*		      Rect1.
[in]	    pdst		          AdasRect*		      Rect2.
[in/out]	    flaprioTrd		      float		      ratio of overlap.

[out]	    returned              unsigned char		      if overlapped return 1;else return 0.

Realized function:
+ if psrc and pdst are overlapped, return 1; else return 0;
*/
unsigned char isRectOverlapped(const WissenObjectRect *psrc, const WissenObjectRect *pdst, float flaprioTrd)
{
	int Lapwidth = psrc->width + pdst->width - (WS_MAX(pdst->x + pdst->width, psrc->x + psrc->width) \
		- WS_MIN(pdst->x, psrc->x));

	int Lapheigt = psrc->height + pdst->height - (WS_MAX(psrc->y + psrc->height, pdst->y + pdst->height) \
		- WS_MIN(pdst->y, psrc->y));

	if (Lapwidth < 0 || Lapheigt < 0 || Lapwidth * Lapheigt < psrc->width * psrc->height * flaprioTrd)
	{
		return 0;
	}

	return 1;
}