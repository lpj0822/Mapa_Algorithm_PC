#ifndef TRACKING_UTILITY_H
#define TRACKING_UTILITY_H

#include "track_type.h"

//#define TAIL_LIGHT_DEBGU
//#define USE_TAIL_LIGHT

#ifdef TAIL_LIGHT_DEBGU
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#endif // TAIL_LIGHT_DEBGU

#ifdef TAIL_LIGHT_DEBGU
void cvtUcharToMat(unsigned char *image, int width, int height, cv::Mat &dst);
#endif //TAIL_LIGHT_DEBGU

/*
Function process:
+ Remove the overlapped rects in pInPutParam.
*/
void removeOverlappedRects(const int objectCount, WissenObjectRectTracked *trackingObjects);

/*
I/O:	    Name		        Type	     		  Content

[in]	    pA		            const WissenSystime*        time A
[in]	    pB		            const WissenSystime*        time B
[in/out]	pC		            WissenSystime*		      time C

Realized function:
+ caculate the time difference
*/
void mvSubTime(const WissenSystime *pA, const WissenSystime *pB, WissenSystime *pC);

#endif //TRACKING_UTILITY_H