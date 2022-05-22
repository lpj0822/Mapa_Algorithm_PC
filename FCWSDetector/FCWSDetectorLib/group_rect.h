/**************************************************************************************************************
Copyright © Wissen Intelligent Sensing Technology Co., Ltd. 2011-2017. All rights reserved.
File name: FCWSD_Interface.h
Version: 1.0		Date: 2017-02-28		Author: Yanming Wang		ID: 1407930

Description:
	The functions in this file are defined to cluster and merge the rects of detector.
	The following function types are included:
	+ getGroupedRectanglesCar(): Clustered and merge the deteced list of rects

Deviation:

History:
	+ Version: 1.0		Date: 2017-02-28		Author: Yanming Wang	ID: 1407930
	  Modification: Coding Standards are added.
**************************************************************************************************************/

#ifndef GROUP_RECT_H
#define GROUP_RECT_H

#include "vehicle_type.h"

/*
I/O:	   Name		        Type	                 Size			  	    Content

[in]       pVehicleDetor    FCWSDetectorGlobalPara*                         Global parameter for detector
[in/out]   rectList		    lbpRectCar*	             sizeof(lbpRectCar*)    Input/Output rects to be clustered
[in/out]   prectNum		    int	                     4-Byte	                Number of input/output rects
[in]       groupThreshold	int	                     4-Byte	                Threshold for a rect group being a cluster
[in]       eps		        float	             sizeof(float)	    Threshold for rects cluster.
					     			  					  
[out]	   returned value   int                      4-Byte	                Clustering number.

Realized function:
	+ Clustered and merge the deteced list of rects.
*/
int getGroupedRectanglesCar(FCWSDetectorGlobalPara *pVehicleDetor, WissenObjectRect *rectList, int *prectNum, const int groupThreshold, const float eps);

#endif