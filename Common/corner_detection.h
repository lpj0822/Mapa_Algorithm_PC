#ifndef CORNER_DETECTION_H
#define CORNER_DETECTION_H

#include "common.h"

/*
I/O:	    Name		          Type	     		  Content

[in]	    im		              const WissenImage*	  input image
[in]	    pMask		          const unsigned char*	  input mask image buffer
[in]        roi                   WissenRect          input image roi
[in]	    b		              int	              Corner Thresh
[in/out]	num_corners		      int*	              Num of fast points
[in/out]	resultCorners		  int*	              location fast points
Realized function:
+ fast corner detect
*/
void fastCornerDetect9_16(const WissenImage* srcImage, const unsigned char* pMask, const WissenRect roi, const int barrier, const int maxCornerCount, int* num_corners, WissenPoint *resultCorners);

/*
I/O:	    Name		          Type	     		  Content

[in]	    im		              const WissenImage*	  input image buffer
[in]	    corners		          WissenPoint*	          the input fast corner
[in]	    num_corners		      int	              Num of input fast points
[in]	    barrier		          int	              Corner Thresh
[in/out]	numnx		          int*	              Output points Num after nonmax suppress
[in/out]	pRowStart		      int*	              the point index of each row
[in/out]	pScore		          int*	              score of points
[in/out]	pXYNoMax		      WissenPoint*	              the output fast corner after nonmax suppress

[out]       returned              WissenPoint*	              the output fast corner after nonmax suppress

Realized function:
+ Do the nonmax suppress for corner points
*/
WissenPoint*  fast_nonmax(const WissenImage* srcImage, WissenPoint* corners, int numcorners, int barrier, int* numnx,
	int *pRowStart, int * pScore, WissenPoint *pXYNoMax);


/*
I/O:	    Name		          Type	     		  Content

[in]	    srcImg		          const WissenImage*  input image
[in/out]	harris_response		  AdasCorner*		  points.
[in]	    num		              int		          Num of points.

Realized function:
+ caculate the Harris response for input points, given the value of harris_response.val
*/
void CalculateHarrisResponse(const WissenImage *srcImage, AdasCorner *harris_response, const int num);

/*
I/O:	    Name		          Type	     		  Content

[in/out]	corner_pass		      AdasCorner*		  input points.
[in/out]	corner_max		      AdasCorner*		  output points.
[in]	    num_pass		      int		          Num of input points.
[in/out]	num_max		          int*		          Num of output points.
[in]	    max_num		          int		          The max output Num.

Realized function:
+ sort the corner in decreasing order by corner_pass.val and update corner_max
*/
void CornerResponseRestrain(AdasCorner *corner_pass, AdasCorner *corner_max, int num_pass, int *num_max, int max_num);

#endif //CORNER_DETECTION_H