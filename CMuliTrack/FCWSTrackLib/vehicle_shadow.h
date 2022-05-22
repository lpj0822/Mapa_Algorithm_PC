#ifndef VEHICLE_SHADOW_H
#define VEHICLE_SHADOW_H

#ifdef __cplusplus
extern "C" {
#endif

#include "common.h"
#include "LDWS_Interface.h"

extern int initShadowDetection(const int srcWidth, const int srcHeight, const float srcROIYFactor);

extern int computeShadow(const WissenImage *pOriGrayImg);

extern int freeShadowDetection();

extern int mvfindGroundValue(const WissenImage *pOriGrayImg, const WissenObjectRect *objectList, const int objectCount);

#ifdef __cplusplus
}
#endif

#endif // VEHICLE_SHADOW_H
