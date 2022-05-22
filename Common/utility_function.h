/**************************************************************************************************************
Copyright © Wissen Intelligent Sensing Technology Co., Ltd. 2011-2018. All rights reserved.
File name: Geometry.h
Version: 1.0		Date: 2018-11-29		Author: Peijie Li		ID: 1059886

Description:
The functions in this file are defined as the realized of commonly used operates for points and rects.

ATTENTION:

Deviation:

History:

**************************************************************************************************************/
#ifndef UTILITY_FUNCTION_H
#define UTILITY_FUNCTION_H

#include "common.h"

/*
I/O:	    Name		    Type	     		  Content

[in]	    A		        WissenPoint*		      point A.
[in]	    B		        WissenPoint*		      point B.
[out]       returned        float             the pow of norm of point A and B

Realized function:
+ caculate the pow of norm of point A and B;
*/
int mvDisPow(WissenPoint *A, WissenPoint *B);

/*
I/O:	    Name		    Type	     		  Content

[in]	    A		        WissenPoint*		      point A.
[in]	    B		        WissenPoint*		      point B.
[out]       returned        float             the norm of point A and B

Realized function:
+ caculate the norm of point A and B;
*/
float mvNormPoint(WissenPoint *A, WissenPoint *B);

/*
I/O:	    Name		    Type	     		  Content

[in]	    pVec		    WissenPoint*		      Vector.
[out]       returned        float             the norm of Vector

Realized function:
+ caculate the norm of Vector;
*/
float mvNormVec(WissenPoint *pVec);

/*
I/O:	    Name		    Type	     		        Content

[in]	    point		    const WissenPoint*	        point
[in]	    rec		        const WissenObjectRect*	    rect.

[out]	    returned        unsigned char		        if point in rect return 1; else return 0.

Realized function:
+ if point in rect return 1; else return 0.
*/
unsigned char isPointInRect(const WissenPoint *point, const WissenObjectRect *rec);

/*
I/O:	    Name		          Type	     		  Content

[in/out]	dstRect		          WissenObjectRect*		      limit Rect.
[in]	    rect		          WissenObjectRect*		      Rect.

Realized function:
+ make sure dstRect inside of rect;
*/
void limitObjectRectRange(const WissenObjectRect rect, WissenObjectRect *dstRect);

/*
I/O:	    Name		          Type	     		  Content

[in]	    srcRec		          AdasRect		      input rect.
[in]	    pDstRec		          AdasRect		      output rect.
[in]	    fShrinkRate		      float		      shrinking factor.

Realized function:
+ do the shrinking for srcRec and get pDstRec
*/
void mvShinkRect(WissenObjectRect srcRec, WissenObjectRect *pDstRec, float fShrinkRate);

/*
I/O:	    Name		          Type	     		          Content

[in]	    pDetRecw		      const AdasRect*		      Rect1.
[in]	    pGroupRec		      const   AdasRect*		      Rect2.
[in/out]	LapRec		          AdasRect*		              overlap rect.

[out]	    returned              unsigned char		              if overlapped return 1;else return 0.

Realized function:
+ if psrc and pdst are overlapped, return 1 and LapRec is the overlapped region; else return 0;
*/
unsigned char computeRectOverlappedRegion(const WissenObjectRect *pDetRecw, const WissenObjectRect *pGroupRec, WissenObjectRect *LapRec);

/*
I/O:	    Name		          Type	     		  Content

[in]	    psrc		          AdasRect*		      Rect1.
[in]	    pdst		          AdasRect*		      Rect2.
[in/out]	    flaprioTrd		      float		      ratio of overlap.

[out]	    returned              unsigned char		      if overlapped return 1;else return 0.

Realized function:
+ if psrc and pdst are overlapped, return 1; else return 0;
*/
unsigned char isRectOverlapped(const WissenObjectRect *psrc, const WissenObjectRect *pdst, float flaprioTrd);


#endif //UTILITY_FUNCTION_H