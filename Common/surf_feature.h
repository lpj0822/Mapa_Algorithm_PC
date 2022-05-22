#ifndef SURF_FEATURE_H
#define SURF_FEATURE_H

#include "common.h"

#define SURF_DESC_DIMENTION 64

/*
I/O:	    Name		          Type	     		  Content

[in]	    srcImage		      const WissenImage*	  input image
[in]	    pPoint		          const AdasCorner*   input fast points
[in]	    nPtCnt		          int		          input fast points Num

[in/out]	pSurfFeature		  unsigned char*		      surf feature.

Realized function:
+ caculate the surf feature of input point
*/
void mvComputeSurfDescriptor(const WissenImage* srcImage, const AdasCorner *pPoint, int nPtCnt, unsigned char *pSurfFeature);

/*
I/O:	    Name		  Type	     		  Content

[in]	    p1		      const unsigned char*	  input surf feature
[in]	    p2		      const unsigned char*	  input surf feature

[out]	    returned	  int*		          surf feature distance.

Realized function:
+ caculate the distance between surf features
*/
int mvSurfDist(const unsigned char *p1, const unsigned char *p2);

#endif //SURF_FEATURE_H