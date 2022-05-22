/**************************************************************************************************************
Copyright © Wissen Intelligent Sensing Technology Co., Ltd. 2011-2018. All rights reserved.
File name: kalmanfilter.cpp
Version: 1.0		Date: 2018-11-27		Author: Peijie Li		ID: 1059886

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

**************************************************************************************************************/

#include "kalman_filter.h"

/*
* @brief
*   Init fields of structure @kalman1_state.
*   I make some defaults in this init function:
*     A = 1;
*     H = 1;
*   and @q,@r are valued after prior tests.
*
*   NOTES: Please change A,H,q,r according to your application.
*
* @inputs
*   state - Klaman filter structure
*   init_x - initial x state value
*   init_p - initial estimated error convariance
* @outputs
* @retval
*/
void kalman1_init(kalman1_state *state, float init_x, float init_p)
{
	state->x = init_x;
	state->p = init_p;
	state->A = 1;
	state->H = 1;
	state->q = 2e2;//10e-6;  /* predict noise convariance */2e2
	state->r = 5e2;//10e-5;  /* measure error convariance */
}


/*
* @brief
*   1 Dimension Kalman filter
* @inputs
*   state - Klaman filter structure
*   z_measure - Measure value
* @outputs
* @retval
*   Estimated result
*/
float kalman1_filter(kalman1_state *state, float z_measure)
{
	/* Predict */
	state->x = state->A * state->x;
	state->p = state->A * state->A * state->p + state->q;  /* p(n|n-1)=A^2*p(n-1|n-1)+q */

	/* Measurement */
	state->gain = state->p * state->H / (state->p * state->H * state->H + state->r);
	state->x = state->x + state->gain * (z_measure - state->H * state->x);
	state->p = (1 - state->gain * state->H) * state->p;

	return state->x;
}

/*
* @brief
*   Init fields of structure @kalman1_state.
*   I make some defaults in this init function:
*     A = {{1, 0.1}, {0, 1}};
*     H = {1,0};
*   and @q,@r are valued after prior tests.
*
*   NOTES: Please change A,H,q,r according to your application.
*
* @inputs
* @outputs
* @retval
*/
void kalman_rect_init(kalman_rect_state *state, WissenObjectRect rec, float init_p[4])
{
	int i;
	float init_rect[4] = { rec.x, rec.y, rec.width, rec.height };

	state->A = 1;
	state->H = 1;

	for (i = 0; i< 4; i++)
	{
		state->x[i] = init_rect[i];
		state->p[i] = init_p[i];
		state->q[i] = 2e2;//10e-6;  /* predict noise convariance */
		state->r[i] = 5e2;//10e-5;  /* measure error convariance */
	}
}


/*
Function process:
+ Do the Kalman filter for state->x
Fan-in :
+ mvGroupGenerate()
Fan-out:
+ N/A
ATTENTION: __________
*/
WissenObjectRect kalman_rect_filter(kalman_rect_state *state, WissenObjectRect Measure_Rec)
{
	int i;
	WissenObjectRect OutRec;
	float z_measure[4] = { Measure_Rec.x, Measure_Rec.y, Measure_Rec.width, Measure_Rec.height };

	/* Predict */
	//for (i =0 ;i< 4;i++)
	//{
	//	state->x[i] = state->A * state->x[i];
	//	state->p[i] = state->A * state->A * state->p[i] + state->q[i];  /* p(n|n-1)=A^2*p(n-1|n-1)+q */

	//	/* Measurement */
	//	state->gain[i] = state->p[i] * state->H / (state->p[i] * state->H * state->H + state->r[i]);
	//	state->x[i] = state->x[i] + state->gain[i] * (z_measure[i] - state->H * state->x[i]);
	//	state->p[i] = (1 - state->gain[i] * state->H) * state->p[i];
	//}


	/* Predict，consider A，H= I*/
	for (i = 0; i< 5; i++)
	{
		//state->x[i] = state->x[i];
		state->p[i] = state->p[i] + state->q[i];  /* p(n|n-1)=A^2*p(n-1|n-1)+q */

		/* Measurement */
		state->gain[i] = state->p[i] / (state->p[i] + state->r[i]);
		state->x[i] = state->x[i] + state->gain[i] * (z_measure[i] - state->x[i]);
		state->p[i] = (1 - state->gain[i]) * state->p[i];
	}

	OutRec.x = (short)state->x[0];
	OutRec.y = (short)state->x[1];
	OutRec.width = (short)state->x[2];
	OutRec.height = (short)state->x[3];
	OutRec.confidence = Measure_Rec.confidence;
	return OutRec;
}