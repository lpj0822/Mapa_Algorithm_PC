#ifndef  KALMAN_FILTER_H
#define  KALMAN_FILTER_H

#include "common.h"

/*
* NOTES: n Dimension means the state is n dimension,
* measurement always 1 dimension
*/

/* 1 Dimension */
typedef struct {
	float x;  /* state */
	float A;  /* x(n)=A*x(n-1)+u(n),u(n)~N(0,q) */
	float H;  /* z(n)=H*x(n)+w(n),w(n)~N(0,r)   */
	float q;  /* process(predict) noise convariance */
	float r;  /* measure noise convariance */
	float p;  /* estimated error convariance */
	float gain;
} kalman1_state;


/* Rect Dimension{x,y,w,h} */
typedef struct KALMAL_RECT_STATE
{
	float x[4];  /* state */
	float A;  /* x(n)=A*x(n-1)+u(n),u(n)~N(0,q) */
	float H;  /* z(n)=H*x(n)+w(n),w(n)~N(0,r)   */
	float q[4];  /* process(predict) noise convariance */
	float r[4];  /* measure noise convariance */
	float p[4];  /* estimated error convariance */
	float gain[4];
} kalman_rect_state;

/*
I/O:	    Name		          Type	     		          Content

[in/out]	state		          kalman1_state*		      kalman1_state.
[in]	    init_x		          float		                  init value for state->x.
[in]	    init_p		          float		                  init value for state->p.

Realized function:
+ init the values for kalman1_state for one Dimension.
*/
void kalman1_init(kalman1_state *state, float init_x, float init_p);

/*
I/O:	    Name		          Type	     		          Content

[in/out]	state		          kalman1_state*		      kalman1_state.
[in]	    z_measure		      float		                  measurement for x.

Realized function:
+ Do the Kalman filter for x based on the measurement z.
*/
float kalman1_filter(kalman1_state *state, float z_measure);

/*
I/O:	    Name		          Type	     		          Content

[in/out]	state		          kalman1_state*		      kalman1_state.
[in]	    init_x		          AdasRect		              init value for state->x.
[in]	    init_p		          float		              init value for state->p.

Realized function:
+ init the values for kalman1_state for four Dimension.
*/
void kalman_rect_init(kalman_rect_state *state, WissenObjectRect rec, float init_p[4]);

/*
I/O:	    Name		          Type	     		          Content

[in/out]	state		          kalman1_state*		      kalman1_state.
[in]	    z_measure		      float		                  measurement for x.

Realized function:
+ Do the Kalman filter for x based on the measurement z.
*/
WissenObjectRect kalman_rect_filter(kalman_rect_state *state, WissenObjectRect Measure_Rec);

#endif //KALMAN_FILTER_H