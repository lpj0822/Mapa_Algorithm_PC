#ifndef VEHICLE_TAILLIGHT
#define VEHICLE_TAILLIGHT

#include "common.h"

typedef struct TailLightResult
{
	WissenObjectRect *lightRect;
    int count;
}TailLightResult;

typedef struct DarkTailLightGlobalPara
{
    //all memory
	unsigned char *allMemory;
	unsigned char *imageHist;
	unsigned char *lightImage;
	unsigned char *tailLightImage;
    int *treeNode;
    int *labelsNode;
	TailLightResult leftLightResult;
	TailLightResult rightLightResult;
	TailLightResult finalResult;
	TailLightResult preLeftLight;
	TailLightResult preRightLight;
	WissenSize dataSize;
	int confirmTailLightIndex;
	int preObjectID;
}DarkTailLightGlobalPara;

int initTailLight(const int srcWidth, const int srcHeight, DarkTailLightGlobalPara *tailLightPara);

void computeTailLight(const WissenImage *pBuff, const WissenObjectRectTracked object, const int objectID, DarkTailLightGlobalPara *tailLightPara);

void freeTailLight(DarkTailLightGlobalPara *tailLightPara);

#endif // VEHICLE_TAILLIGHT

