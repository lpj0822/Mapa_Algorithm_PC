#ifndef SAVE_RESULT_H
#define SAVE_RESULT_H

#include <stdio.h>
extern "C"
{
#include "FCWSD_Interface.h"
#include "LDWS_Interface.h"
#include "CMulitTrack.h"
#include "3Dmap_Interface.h"
}

int writeTrackerReault(FILE *writeFile, int frameNo, float fzoom, MuliTracker *pTrackOutput, LDWS_Output *pTest);

int writeDetecctorReault(FILE *writeFile, int frameNo, objectSetsCar *pFCWSOutput, LDWS_Output *pTest);

int write3DMapResult(FILE *writeFile, int frameNo, FENCE_LIST drawFrences);

#endif //SAVE_RESULT_H