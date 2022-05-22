#ifndef VEHICLE_PROPOSALS
#define VEHICLE_PROPOSALS

#include "vehicle_type.h"

typedef struct ProposalsGlobalPara
{
	//all memory
	unsigned char *allMemory;
	unsigned char *sobleTable;
	unsigned char *dyImage;
	unsigned char *dyLineImage;
	unsigned int *dyIntegralImage;
	WissenRect proposalsRoi;
	WissenRect integrateRoi;
	int relativeRoiY;
}ProposalsGlobalPara;

int initProposals(const int srcWidth, const int srcHeight, const float srcROIYFactor);

//flag 0-day 1-night
void computeProposals(const unsigned char flag, const WissenImage *pOriGrayImg);

//flag 0-day 1-night
int filterCarTask(const unsigned char flag, const WissenPoint* task, const FCWSDetectorGlobalPara *pVehicleDetor, const float factor);

int freeProposals();

#endif // VEHICLE_PROPOSALS

