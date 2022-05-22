/**************************************************************************************************************
Copyright © Wissen Intelligent Sensing Technology Co., Ltd. 2011-2018. All rights reserved.
File name: common_type_def.h
Version: 1.0		Date: 2018-11-15		Author:  Peijie Li		ID: 1059886

Description:
This file define the commonly used types for all.

ATTENTION:

Deviation:

History:

**************************************************************************************************************/

#ifndef COMMON_H 
#define COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>

#define  INPUT_YUV

typedef struct Wissen16SPoint
{
	short x;
	short y;
}Wissen16SPoint;

typedef struct WissenPoint
{
	int x;
	int y;
} WissenPoint;

typedef struct Wissen32FPoint
{
	float x;
	float y;
}Wissen32FPoint;

typedef struct WissenRect
{
	int x;
	int y;
	int width;
	int height;
}WissenRect;

typedef struct WissenSize
{
	int width;
	int height;
}WissenSize;

typedef struct WissenSystime
{
	unsigned int  wHour;
	unsigned int  wMin;
	unsigned int  wSec;
	unsigned int wMilSec;
}WissenSystime;

typedef struct WissenObjectRect
{
	int x;
	int y;
	int width;
	int height;
	int	confidence;
}WissenObjectRect;

typedef enum OBJTYPE
{
	Car = 0,
	People,
	UnknownYet,
}objtype;

typedef struct WissenObjectRectTracked
{
	objtype nType;
	WissenObjectRect object;
	long long nGroupID;//rect ID
	long long nFramSeq;
}WissenObjectRectTracked;

typedef struct WissenImage
{
	unsigned char	*data;

#ifdef INPUT_YUV
	unsigned char *putr;
	unsigned char *pvtr;
#endif

	int				nWid;
	int				nHig;
}WissenImage;

typedef struct WissenCorner
{
	int x;
	int y;
	int value;
}WissenCorner;

typedef struct  MATCHSTATE
{
	int  nGroupIndex[2];
	int  nTrjIndexofGroup[2];
	int  nMatchDis;
	unsigned char nMacthNum;

}MatchState;

typedef struct  ADASCORNER
{
	int  val;
	int  nCornerIndex;
	MatchState  State;
	WissenPoint point;
}AdasCorner;

#define WS_SWAP(a,b,t) ((t) = (a), (a) = (b), (b) = (t))

#ifndef WS_MIN
#define WS_MIN(a, b)   ((a) > (b) ? (b) : (a))
#endif

#ifndef WS_MAX
#define WS_MAX(a, b)   ((a) > (b) ? (a) : (b))
#endif

#ifndef WS_MyRound
#define WS_MyRound(value)		(int)((value) + ( (value) >= 0 ? 0.5 : -0.5))
#endif

#ifndef WS_ABS
#define WS_ABS(x) ((x) >= 0 ? (x) : (-(x)))
#endif

//#if __x86_64
#define ADAS_ALIGN_16BYTE(x) ((unsigned char*)(((unsigned long long)(x) + 15) >> 4 << 4))

#define ADAS_ALIGN_16BYTE_SIZE(x)   ((((unsigned long long)(x) + 15) >> 4) << 4)

#ifdef __cplusplus
}
#endif

#endif