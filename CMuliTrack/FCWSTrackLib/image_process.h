/**************************************************************************************************************
Copyright © Wissen Intelligent Sensing Technology Co., Ltd. 2011-2018. All rights reserved.
File name: trajectory.h
Version: 1.0		Date: 2018-11-29		Author: Yanming Wang		ID: 1047930

Description:
The functions in this file are defined as the operate of image.
The following function types are included:


ATTENTION:

Deviation:

History:

**************************************************************************************************************/

#ifndef IMAGE_PROCESS_H
#define IMAGE_PROCESS_H

#include "common.h"

/*
I/O:	    Name		    Type	     		  Content

[in]        srcimg	        const WissenImage*	      input image buffer
[in]	    pointRio		WissenPoint		      Left and up ROI point.
[in/out]	IntelImg		integral WissenImage*	  output integral image

Realized function:
+  caculate the integral image of ROI of srcimg
*/
void mvInterlimg(const WissenImage* srcimg, const WissenPoint pointRio, WissenImage* IntelImg);

/*
I/O:	    Name		    Type	     		  Content

[in]	    IntelImg		integral WissenImage*	  input integral image
[in]	    lt		        Wissen16SPoint		      Left and up point of ROI.
[in]	    br		        Wissen16SPoint		      Right and bottom point of ROI.

Realized function:
+  Get the integral value of ROI based on the interval image
*/
int mvSumImgRio(WissenImage *IntelImg, Wissen16SPoint lt, Wissen16SPoint br);

/*
I/O:	    Name		          Type	     		  Content

[in]	    pSrcImg		          const WissenImage*		  input image buffer.
[in/out]	pDstImg		          WissenImage*		      output resized image buffer.

Realized function:
+ Get the resized image buffer;
*/
void mvResizeImg(const WissenImage *pSrcImg, int *arr_y, int *arr_x, WissenImage *pDstImg);

int findSymmetryAxisX(unsigned char *src, int width, int height);

/*
I/O:	    Name		        Type	     		  Content

[in]	    img		            const WissenImage	      input image
[in/out]	pTempimg		    WissenImage*	              Temple image
[in]	    pTempRec		    AdasRect*	          Temple rect acorrding to img.

Realized function:
+ Get the temple image from img
*/
void mvSelcTemplatByCen(const WissenImage img, WissenImage *pTempimg, WissenObjectRect *pTempRec);

unsigned char matchByTemple(const WissenImage *pimg, const WissenImage *pTempimg,
	WissenObjectRect SearcRec, WissenObjectRect *pMatchRec, int nPubliBuffOff, int *tempSpace);

/*
I/O:	    Name		    Type	     		  Content

[in]	    img		        const WissenImage	      input searched image
[in]	    Tempimg		    const WissenImage		  Temple image.
[in]	    SearcRec		AdasRect		      search region.
[in/out]	pMatchRec		AdasRect		      Matched result.
[in]	    nPubliBuffOff	int		              public buffer offset.
[in]	    bComparSecd	    unsigned char		          if 1 Caculate the second matched region, else just caculate the best one.
[in]	    nId		        int		              scale factor.
[in]	    fMatchScore		float*		      matching score.

[out]       returned        unsigned char               if 1 find the matched result; else, return 0

Realized function:
+  Do the Temple matching
*/
unsigned char mvTemplatMatch(const WissenImage img, const WissenImage Tempimg,
	WissenObjectRectTracked SearcRec, WissenObjectRectTracked *pMatchRec, int nPubliBuffOff,
	unsigned char bComparSecd, int* tempSpace, float *fMatchScore);

void mvDerivemgRio(WissenImage * pSrcImg, WissenImage * RioImg, WissenObjectRect RioRec);

#endif//IMAGE_PROCESS_H