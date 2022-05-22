#ifndef INTERPOLATION_ALGORITHM_H
#define INTERPOLATION_ALGORITHM_H

#include "common.h"

/*
Function process:
+ Grayscale image resize and use neighbor interpolation algorithm.
*/
extern void grayImageResizeOfNeighborInterpolation(const WissenImage srcImage, const WissenRect roi, int *tempX, int *tempY, WissenImage *dstRoiImage);

#endif // INTERPOLATION_ALGORITHM_H