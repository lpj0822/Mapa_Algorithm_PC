#ifndef _TARGETVERIFY_H
#define _TARGETVERIFY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "common.h"
#include "FCWSD_Interface.h"

#define  MAX_TRACK_MUM  128

typedef struct TrackedObjectList
{
	WissenObjectRectTracked *trackedObject;
	long long objectCount;
}TrackedObjectList;

typedef struct VerifyObject
{
	long long nGroupID;
	unsigned char nObjState;
}VerifyObject;

typedef struct VerifyObjectList
{
	VerifyObject *object;
	long long objectCount;
}VerifyObjectList;

extern void verifyObjectInit(VerifyObjectList *verifyObjectOutput);
extern void verifyObjectDetcor(const WissenImage pOriGrayfram, const TrackedObjectList *inputObject, DayNightMode mode);
extern void getVerifyObjectResult(VerifyObjectList *verifyObjectOutput);
extern void verifyObjectRelease(VerifyObjectList *verifyObjectOutput);

//index=0/1
extern void initResizeCnn(const int index, const int resizeWidth, const int resizeHeight);
extern void resizeCnnInputObject(const int index, const WissenImage *pOriGrayImg, const WissenObjectRect rect, const int resizeWidth, const int resizeHeight, float* objectData);
extern void releaseResizeCnn(const int index, const int resizeWidth, const int resizeHeight);

#ifdef __cplusplus
}
#endif

#endif
