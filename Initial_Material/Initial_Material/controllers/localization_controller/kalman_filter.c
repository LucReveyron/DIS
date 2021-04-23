
#include <math.h>
#include <stdbool.h>

#include "kalman_filter.h"

#define STATE_SIZE 		2
#define MEASURE_SIZE 	4

void kalman_filter_init(measurement_t measure, kalman_state *state, int ts)
{
	double dt = ts / 1000.0;

	state->x[0] = measure.gps[0];
	state->x[1] = measure.gps[1];

	state->u[0] = measure.acc[0]*dt;
	state->u[0] = measure.acc[1]*dt;

	state->z[0] = measure.left_enc;
	state->z[1] = measure.right_enc;
	state->z[2] = measure.gps[0];
	state->z[3] = measure.gps[1];

	state->sigma[0][0] = 0.01;
	state->sigma[0][1] = 0.0;
	state->sigma[1][0] = 0.0;
	state->sigma[1][1] = 0.01;

	state->A[0][0] = 1.0;
	state->A[0][1] = 0.0;
	state->A[1][0] = 0.0;
	state->A[1][1] = 1.0;

	state->B[0][0] = 1.0;
	state->B[0][1] = 0.0;
	state->B[1][0] = 0.0;
	state->B[1][1] = 1.0;

	state->C[0][0] = 1.0;
	state->C[0][1] = 0.0;
	state->C[1][0] = 0.0;
	state->C[1][1] = 1.0;
	state->C[2][0] = 1.0;
	state->C[2][1] = 0.0;
	state->C[3][0] = 0.0;
	state->C[3][1] = 1.0;

	// To be determine !
	// Odometry error proportionnal to distance traveled
	// GPS error can be supposed fixed ~ 1 m

	state->Q[0][0] = 0;
	state->Q[0][1] = 0;
	state->Q[0][2] = 0;
	state->Q[0][3] = 0;
	state->Q[1][0] = 0;
	state->Q[1][1] = 0;
	state->Q[1][2] = 0;
	state->Q[1][3] = 0;
	state->Q[2][0] = 0;
	state->Q[2][1] = 0;
	state->Q[2][2] = 0;
	state->Q[2][3] = 0;
	state->Q[3][0] = 0;
	state->Q[3][1] = 0;
	state->Q[3][2] = 0;
	state->Q[3][3] = 0;

	state->R[0][0] = 0.05;
	state->R[0][1] = 0.0;
	state->R[1][0] = 0.0;
	state->R[1][1] = 0.05;

}

void kalman_filter_update(measurement_t measure, kalman_state *state, int ts)
{
	float z_dif[MEASURE_SIZE] = {0,0,0,0}; // Correspond to (z - C*x)
	float KC[STATE_SIZE][STATE_SIZE] = {{0,0},
										{0,0}}; // Correspond to K*C 

	/* Prediction */
	for(int i; i < STATE_SIZE; i++)
	{

		for(int j; j < STATE_SIZE; j++)
		{
			if(j < 1)
				state->x[i] = state->A[i][j]*state->x[j] + state->B[i][j]*state->u[j];
			else
				state->x[i] += state->A[i][j]*state->x[j] + state->B[i][j]*state->u[j];

			state->sigma[i][j] = state->A[i][j]*state->sigma[i][j]*state->A[j][i] + state->R[i][j];
		}
	}

	/* Correction */
	compute_kalman_gain(state);

	compute_measure_distance(state, z_dif);

	// State update
	for(int i; i < STATE_SIZE; i++)
	{
		for(int j; j < MEASURE_SIZE; j++)
		{
			state->x[i] += state->K[i][j]*z_dif[j];

			// Compute KC for the covariance matrix update part
			KC[i][0] += state->K[i][j]*state->C[j][0];
			KC[i][1] += state->K[i][j]*state->C[j][1]; 
		}
	}

	// Covariance update 
	for(int i; i < STATE_SIZE; i++)
	{
		for(int j; j < STATE_SIZE; j++)
		{
			if(i < 1)
				state->sigma[i][j] = (1 - KC[i][0])*state->sigma[0][j] - KC[i][1]*state->sigma[1][j];
			else
				state->sigma[i][j] = (1 - KC[i][1])*state->sigma[1][j] - KC[i][0]*state->sigma[0][j];
		}
	}

}

void compute_kalman_gain(kalman_state *state)
{
	float bloc1[STATE_SIZE][MEASURE_SIZE]; // save sigma*C^T
	float bloc2[MEASURE_SIZE][MEASURE_SIZE]; // save C*sigma*C^T + Q
	float bloc3[MEASURE_SIZE][MEASURE_SIZE]; // save inverse(C*sigma*C^T + Q)

  	for(int i; i < STATE_SIZE; i++)
  	{
  		for(int j; j < MEASURE_SIZE; j++)
  		{	
  			bloc1[i][j] = state->sigma[i][0]*state->C[j][0] + state->sigma[i][1]*state->C[j][1];
  		}
  	}

  	for(int i; i < MEASURE_SIZE; i++)
  	{
  		for(int j; j < MEASURE_SIZE; j++)
  		{
			bloc2[i][j] = state->C[i][0]*bloc1[0][j] + state->C[i][1]*bloc1[1][j] + state->Q[i][j];		
  		}	
  	}

  	inverseMatrix4x4(bloc2,bloc3);

  	for(int i; i < STATE_SIZE; i++)
  	{
  		for(int j; j < MEASURE_SIZE; j++)
  		{	
  			state->K[i][j] = bloc1[i][0]*bloc3[j][0] + bloc1[i][1]*bloc3[j][1] + bloc1[i][2]*bloc3[j][2] + bloc1[i][3]*bloc3[j][3];
  		}
  	}

}

void compute_measure_distance(kalman_state *state, float* z_dif)
{
	float CX[MEASURE_SIZE] = {0,0,0,0};

	for(int i; i < MEASURE_SIZE; i++)
	{
		for(int j; j < STATE_SIZE; j++)
		{
			CX[i] += state->C[i][j]*state->x[j]; 
		}
		z_dif[i] = state->z[i] - CX[i];
	}


}

/* Implementation of 4x4 matrix inversion mostly take from : https://github.com/eelstork/Matrix-Inversion */
float invf(int i, int j, const float* m)
{
    int o = 2+(j-i);
    i += 4+o;
    j += 4-o;
    #define e(a,b) m[ ((j+b)%4)*4 + ((i+a)%4) ]
    float inv =
      + e(+1,-1) * e(+0,+0) * e(-1,+1)
      + e(+1,+1) * e(+0,-1) * e(-1,+0)
      + e(-1,-1) * e(+1,+0) * e(+0,+1)
      - e(-1,-1) * e(+0,+0) * e(+1,+1)
      - e(-1,+1) * e(+0,-1) * e(+1,+0)
      - e(+1,-1) * e(-1,+0) * e(+0,+1);
    return (o%2)?inv : -inv;
    #undef e
}

bool inverseMatrix4x4(const float mr[4][4], float out[4][4])
{
	int r = 0;
	int c = 0;
    float inv[16];
    float m[16];
    for (int i = 0; i < 16; i++)
    {	
    	if(i%4 == 0 && i != 0)
    	{
    		c = 0;
    		r++;
    	}

        m[i] = mr[r][c];
        c++;
    }

    r = 0;
    c = 0;
    for(int i=0;i<4;i++) for(int j=0;j<4;j++) inv[j*4+i] = invf(i,j,m);
    double D = 0;
    for(int k=0;k<4;k++) D += m[k] * inv[k*4];
    if (D == 0) return false;
    D = 1.0 / D;
    for (int i = 0; i < 16; i++)
    {	
    	if(i%4 == 0 && i != 0)
    	{
    		c = 0;
    		r++;
    	}

        out[r][c] = inv[i] * D;
        c++;
    }
    return true;
}

