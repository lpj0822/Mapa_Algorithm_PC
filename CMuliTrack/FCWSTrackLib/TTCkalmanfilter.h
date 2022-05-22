/**************************************************************************************************************
Copyright  Wissen Intelligent Sensing Technology Co., Ltd. 2011-2017. All rights reserved.
File name: kalmanfilter.h
Version: 1.0		Date: 2017-06-05		Author: Yanming Wang		ID: 1047930

Description:	
	The functions in this file are defined as the realized of kalman filter.
	The following function types are included:
	+ kalman1_init(): init the values for kalman1_state for one Dimension.
	+ kalman1_filter(): Do the Kalman filter for x based on the measurement z.
    + kalman_rect_init(): init the values for kalman1_state for four Dimension.
	+ kalman_rect_filter(): Do the Kalman filter for x based on the measurement z.
	
ATTENTION:

Deviation:

History:
	+ Version: 1.0		Date: 2017-06-05		Author: Yanming Wang	ID: 1047930
	  Modification: Coding Standards are added.
**************************************************************************************************************/

#ifndef  TTC_KALMAN_FILTER_H
#define  TTC_KALMAN_FILTER_H

#include "common.h"

/* system state */
typedef struct TTCKalmanState{
	double x[7];
	double p[7 * 7];
	double q[7 * 7];
	double r[5 * 5];

	double A[7 * 7];
	double At[7 * 7];
	double M[7 * 7];
	double Mt[7 * 7];
	double H[5 * 7];
	double Ht[7 * 5];

	double fx;
	double fy;
	double u0;
	double v0;
	double camH;
} TTCKalmanState;

/******************************************************/
/* system kalman filter function*/
/******************************************************/
void sysKalmanInit(TTCKalmanState *state, WissenObjectRectTracked init_rec);

void sysKalmanFilter(TTCKalmanState *state, double tpf, WissenObjectRectTracked Measure_Rec, double vasy);

#endif  /*TTC_KALMAN_FILTER_H*/

