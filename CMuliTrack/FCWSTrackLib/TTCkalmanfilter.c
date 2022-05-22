#include "TTCkalmanfilter.h"
#include "matrix_computation.h"
#include "LDWS_Interface.h"
#include <math.h>

void imgToWorld(TTCKalmanState *state, float u, float v, float w, float h, double WCS[])
{
	float coeff = 3.1415926 / 180;
	float thata = 0;

	WCS[1] = state->camH * ((v - state->v0) * sin(thata * coeff) + state->fy * cos(thata * coeff)) / ((v - state->v0) * cos(thata * coeff) - state->fy * sin(thata * coeff));
	WCS[0] = (u - state->u0) * (-state->camH * sin(thata * coeff) + WCS[1] * cos(thata * coeff)) / state->fx;
	WCS[2] = 0;
	WCS[3] = 0;
	WCS[4] = w * (-state->camH * sin(thata * coeff) + WCS[1] * cos(thata * coeff)) / state->fx;
	float temp = (WCS[1] * sin(thata * coeff) + state->camH * cos(thata * coeff)) * state->fy / (-state->camH * sin(thata * coeff) + WCS[1] * cos(thata * coeff)) - h;
	WCS[5] = WCS[1] * (sin(thata * coeff) * state->fy - temp * cos(thata * coeff)) / (temp * sin(thata * coeff) + state->fy * cos(thata * coeff)) + state->camH;
	WCS[6] = 0;
}

/******************************************************/
/* system kalman filter function*/
/******************************************************/
void sysKalmanInit(TTCKalmanState *state, WissenObjectRectTracked init_rec)
{
	int i, j;
	for (i = 0; i < 7; i++)
	{
		for (j = 0; j < 7; j++)
		{
			if (i == j)
			{
				state->A[i * 7 + j] = 1;
				state->M[i * 7 + j] = 1;
			}
			else
			{
				state->A[i * 7 + j] = 0;
				state->M[i * 7 + j] = 0;
			}
			state->p[i * 7 + j] = 0;
			state->q[i * 7 + j] = 0;
		}
	}

	for (i = 0; i < 35; i++)
		state->H[i] = 0;
	for (i = 0; i < 25; i++)
		state->r[i] = 0;

	int Eu, Ev, Cx, Cy;
	LDWS_Get_inter_Pamer_N(&Eu, &Ev, &Cx, &Cy);
	state->fx = (double)(Eu);
	state->fy = (double)(Ev);
	state->camH = LDWS_GetCameraHeight();
	state->u0 = (double)(Cx);
	state->v0 = (double)(Cy);

	imgToWorld(state, init_rec.object.x, init_rec.object.y, init_rec.object.width, init_rec.object.height, state->x);

	state->p[0 * 7 + 0] = 2e7;
	state->p[1 * 7 + 1] = 2e7;
	state->p[2 * 7 + 2] = 2e7;
	state->p[3 * 7 + 3] = 2e7;
	state->p[4 * 7 + 4] = 2e7;
	state->p[5 * 7 + 5] = 2e7;
	state->p[6 * 7 + 6] = 2e7;

	state->q[0 * 7 + 0] = 1e-5;//x
	state->q[1 * 7 + 1] = 1e-5;//z
	state->q[2 * 7 + 2] = 1e-2;//vx
	state->q[3 * 7 + 3] = 1e-2;//vz
	state->q[4 * 7 + 4] = 1e-2;//w
	state->q[5 * 7 + 5] = 1e-7;//h
	state->q[6 * 7 + 6] = 1e-2;//sita

	state->r[0 * 5 + 0] = 1e-7;//x
	state->r[1 * 5 + 1] = 1e-14;//y
	state->r[2 * 5 + 2] = 2e-5;//w
	state->r[3 * 5 + 3] = 2e-7;//h
	state->r[4 * 5 + 4] = 1e-7;//vanishPty
}


void sysKalmanFilter(TTCKalmanState *state, double tpf, WissenObjectRectTracked Measure_Rec, double vasy)
{
	state->A[0 * 7 + 2] = tpf;
	state->A[1 * 7 + 3] = tpf;
	Trans(state->A, 7, 7, state->At);

	state->M[0 * 7 + 0] = tpf*tpf / 2;
	state->M[1 * 7 + 1] = tpf*tpf / 2;
	state->M[2 * 7 + 2] = tpf;
	state->M[3 * 7 + 3] = tpf;
	Trans(state->M, 7, 7, state->Mt);

	/* Predict */

	/*eq1：xk' = A*xk-1*/
	double arrPriorXk[7 * 1];
	MatMul(state->A, state->x, 7, 7, 1, arrPriorXk);

	/* eq2：pk' = A*pk-1'*AT + M*qk-1*N_T */
	double arrAmulPk_1[7 * 7];
	MatMul(state->A, state->p, 7, 7, 7, arrAmulPk_1);
	double arrAmulPk_1mulAt[7 * 7];
	MatMul(arrAmulPk_1, state->At, 7, 7, 7, arrAmulPk_1mulAt);

	double arrNmulQ[7 * 7];
	MatMul(state->M, state->q, 7, 7, 7, arrNmulQ);
	double arrNmulQmulNt[7 * 7];
	MatMul(arrNmulQ, state->Mt, 7, 7, 7, arrNmulQmulNt);

	double arrPriorPk[7 * 7];
	add(arrAmulPk_1mulAt, arrNmulQmulNt, 7, 7, arrPriorPk);

	/* update */
	double coeff = 3.1415926 / 180;
	double part0, part1, temp = -state->camH * sin(state->x[6] * coeff) + state->x[1] * cos(state->x[6] * coeff);
	double tempSquare = temp * temp;
	state->H[0 * 7 + 0] = state->fx / temp;
	state->H[0 * 7 + 1] = -state->x[0] * state->fx * cos(state->x[6] * coeff) / tempSquare;
	state->H[0 * 7 + 2] = 0;
	state->H[0 * 7 + 3] = 0;
	state->H[0 * 7 + 4] = 0;
	state->H[0 * 7 + 5] = 0;
	state->H[0 * 7 + 6] = -state->x[0] * state->fx *(-state->camH * cos(state->x[6] * coeff) - state->x[1] * sin(state->x[6] * coeff)) / tempSquare;

	state->H[1 * 7 + 0] = 0;
	part0 = state->fy * sin(state->x[6] * coeff) * temp;
	part1 = (state->x[1] * sin(state->x[6] * coeff) + state->camH * cos(state->x[6] * coeff)) * state->fy * cos(state->x[6] * coeff);
	state->H[1 * 7 + 1] = (part0 - part1) / tempSquare;
	state->H[1 * 7 + 2] = 0;
	state->H[1 * 7 + 3] = 0;
	state->H[1 * 7 + 4] = 0;
	state->H[1 * 7 + 5] = 0;
	part0 = (state->fy * state->x[1] * cos(state->x[6] * coeff) - state->fy * state->camH * sin(state->x[6] * coeff)) * temp;
	part1 = (state->x[1] * sin(state->x[6] * coeff) + state->camH * cos(state->x[6] * coeff)) * state->fy * (-state->camH * cos(state->x[6] * coeff) - state->x[1] * sin(state->x[6] * coeff));
	state->H[1 * 7 + 6] = (part0 - part1) / tempSquare;

	state->H[2 * 7 + 0] = 0;
	state->H[2 * 7 + 1] = -state->x[4] * state->fx * cos(state->x[6] * coeff) / tempSquare;
	state->H[2 * 7 + 2] = 0;
	state->H[2 * 7 + 3] = 0;
	state->H[2 * 7 + 4] = state->fx / temp;
	state->H[2 * 7 + 5] = 0;
	state->H[2 * 7 + 6] = -state->x[4] * state->fx *(-state->camH * cos(state->x[6] * coeff) - state->x[1] * sin(state->x[6] * coeff)) / tempSquare;

	state->H[3 * 7 + 0] = 0;
	temp = (state->x[5] - state->camH) * sin(state->x[6] * coeff) + state->x[1] * cos(state->x[6] * coeff);
	tempSquare = temp * temp;
	part0 = state->fy * sin(state->x[6] * coeff) * temp;
	part1 = (state->x[1] * sin(state->x[6] * coeff) + (state->camH - state->x[5]) * cos(state->x[6] * coeff)) * state->fy * cos(state->x[6] * coeff);
	state->H[3 * 7 + 1] = state->H[1 * 7 + 1] - (part0 - part1) / tempSquare;//Z
	state->H[3 * 7 + 2] = 0;
	state->H[3 * 7 + 3] = 0;
	state->H[3 * 7 + 4] = 0;
	part0 = -state->fy * cos(state->x[6] * coeff) * temp;
	part1 = (state->x[1] * sin(state->x[6] * coeff) + (state->camH - state->x[5]) * cos(state->x[6] * coeff)) * state->fy * sin(state->x[6] * coeff);
	state->H[3 * 7 + 5] = -(part0 - part1) / tempSquare;//state->H
	part0 = state->fy * (state->x[1] * cos(state->x[6] * coeff) - (state->camH - state->x[5]) * sin(state->x[6] * coeff)) * temp;
	part1 = (state->x[1] * sin(state->x[6] * coeff) + (state->camH - state->x[5]) * cos(state->x[6] * coeff)) * state->fy * ((state->x[5] - state->camH) * cos(state->x[6] * coeff) - state->x[1] * sin(state->x[6] * coeff));
	state->H[3 * 7 + 6] = state->H[1 * 7 + 6] - (part0 - part1) / tempSquare;//sita

	state->H[4 * 7 + 0] = 0;
	state->H[4 * 7 + 1] = 0;
	state->H[4 * 7 + 2] = 0;
	state->H[4 * 7 + 3] = 0;
	state->H[4 * 7 + 4] = 0;
	state->H[4 * 7 + 5] = 0;
	state->H[4 * 7 + 6] = state->fy / (cos(state->x[6] * coeff) * cos(state->x[6] * coeff));
	Trans(state->H, 5, 7, state->Ht);

	/*eq3: Kk = p_k*Ht / (H*p_k*Ht+R) */
	double arrPriorlPkmulHt[7 * 5];
	MatMul(arrPriorPk, state->Ht, 7, 7, 5, arrPriorlPkmulHt);

	double arrHmulPriorlPk[5 * 7];
	MatMul(state->H, arrPriorPk, 5, 7, 7, arrHmulPriorlPk);

	double arrHmulPriorlPkmulHt[5 * 5];
	MatMul(arrHmulPriorlPk, state->Ht, 5, 7, 5, arrHmulPriorlPkmulHt);

	double arrHmulPriorlPkmulHtaddR[5 * 5];
	add(arrHmulPriorlPkmulHt, state->r, 5, 5, arrHmulPriorlPkmulHtaddR);

	brinv(arrHmulPriorlPkmulHtaddR, 5);

	double arrK[7 * 5];
	MatMul(arrPriorlPkmulHt, arrHmulPriorlPkmulHtaddR, 7, 5, 5, arrK);

	/*eq4: Pk = (1-Kk*H)*P_k */
	double arrKmulH[7 * 7];
	MatMul(arrK, state->H, 7, 5, 7, arrKmulH);
	EYEsub(arrKmulH, 7);
	MatMul(arrKmulH, arrPriorPk, 7, 7, 7, state->p);

	/*eq5: Xk = X_K + Kk(z - f(x_k)) */
	double arrfPriorXk[5 * 1];
	arrfPriorXk[0] = state->u0 + arrPriorXk[0] * state->fx / (-state->camH * sin(arrPriorXk[6] * coeff) + arrPriorXk[1] * cos(arrPriorXk[6] * coeff));
	arrfPriorXk[1] = state->v0 + (arrPriorXk[1] * sin(arrPriorXk[6] * coeff) + state->camH * cos(arrPriorXk[6] * coeff)) * state->fy / (-state->camH * sin(arrPriorXk[6] * coeff) + arrPriorXk[1] * cos(arrPriorXk[6] * coeff));
	arrfPriorXk[2] = arrPriorXk[4] * state->fx / (-state->camH * sin(arrPriorXk[6] * coeff) + arrPriorXk[1] * cos(arrPriorXk[6] * coeff));
	arrfPriorXk[3] = (arrPriorXk[1] * sin(arrPriorXk[6] * coeff) + state->camH * cos(arrPriorXk[6] * coeff)) * state->fy / (-state->camH * sin(arrPriorXk[6] * coeff) + arrPriorXk[1] * cos(arrPriorXk[6] * coeff)) - \
		(arrPriorXk[1] * sin(arrPriorXk[6] * coeff) + (state->camH - arrPriorXk[5]) * cos(arrPriorXk[6] * coeff)) * state->fy / ((arrPriorXk[5] - state->camH) * sin(arrPriorXk[6] * coeff) + arrPriorXk[1] * cos(arrPriorXk[6] * coeff));
	arrfPriorXk[4] = state->v0 + state->fy * tan(arrPriorXk[6] * coeff);

	double arrZsubfPriorXk[5 * 1];
	double Z[5];
	Z[0] = (double)(Measure_Rec.object.x);
	Z[1] = (double)(Measure_Rec.object.y);
	Z[2] = (double)(Measure_Rec.object.width);
	Z[3] = (double)(Measure_Rec.object.height);
	Z[4] = (double)(vasy);
	sub(Z, arrfPriorXk, 5, 1, arrZsubfPriorXk);

	double arrKkmulZsubfPriorXk[7 * 1];
	MatMul(arrK, arrZsubfPriorXk, 7, 5, 1, arrKkmulZsubfPriorXk);

	add(arrPriorXk, arrKkmulZsubfPriorXk, 7, 1, state->x);
}