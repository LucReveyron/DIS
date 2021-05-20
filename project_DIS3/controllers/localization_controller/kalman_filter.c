

#include <stdio.h>
#include <stdlib.h>
#include <gsl/gsl_linalg.h>
#include "kalman_filter.h"

#define STATE_SIZE 		4
#define MEASURE_SIZE 	4

static kalman_state _state;
static double _dt;


void kalman_filter_init(measurement_t measure, int ts)
{
	_dt = ts / 1000.0;

	//init. estimation state

	_state.x = gsl_vector_alloc(4);

	gsl_vector_set(_state.x,0,measure.gps[0]);
	gsl_vector_set(_state.x,1,measure.gps[1]);
	gsl_vector_set(_state.x,2,0.0);
	gsl_vector_set(_state.x,3,0.0);

	//init. command and measure vector
	_state.u = gsl_vector_alloc(2);
	_state.z = gsl_vector_alloc(4);

	//init. covariance matrix, K, A, B and R
	_state.sigma = gsl_matrix_alloc (4, 4);
	_state.K = gsl_matrix_alloc (4, 4);
	_state.A = gsl_matrix_alloc (4, 4);
	_state.B = gsl_matrix_alloc (4, 2);
	_state.R = gsl_matrix_alloc (4, 4);

	gsl_matrix_set_identity(_state.sigma);
	gsl_matrix_scale(_state.sigma,0.001);

	gsl_matrix_set_identity(_state.A);
	gsl_matrix_set(_state.A,0,2,_dt);
	gsl_matrix_set(_state.A,1,3,_dt);

	gsl_matrix_set_zero(_state.B);
	gsl_matrix_set(_state.B,2,0,_dt);
	gsl_matrix_set(_state.B,3,1,_dt);	

	gsl_matrix_set_zero(_state.R);
	gsl_matrix_set(_state.R,0,0,0.015);
	gsl_matrix_set(_state.R,1,1,0.015);
	gsl_matrix_set(_state.R,2,2,0.005);
	gsl_matrix_set(_state.R,3,3,0.005);



	//init. matrix C

	_state.C = gsl_matrix_alloc (4, 4);
	gsl_matrix_set_zero(_state.C);
	gsl_matrix_set(_state.C,0,0,1.0);
	gsl_matrix_set(_state.C,1,1,1.0);
	gsl_matrix_set(_state.C,2,0,1.0);
	gsl_matrix_set(_state.C,3,1,1.0);

	//init. matrix Q

	_state.Q = gsl_matrix_alloc (4, 4);

	gsl_matrix_set_zero(_state.Q);
	gsl_matrix_set(_state.Q,0,0,1.0);
	gsl_matrix_set(_state.Q,1,1,1.0);
	gsl_matrix_set(_state.Q,2,2,3.0);
	gsl_matrix_set(_state.Q,3,3,3.0);

	printf("[INFO KALMAN] Init. done \n");

}

void kalman_filter_update(measurement_t measure, pose_t pose)
{

	//update current control and measure state
	gsl_vector_set(_state.u,0, measure.acc[0]);
	gsl_vector_set(_state.u,1, measure.acc[1]);

	//printf("speed x = %f speed y = %f , dt = %f \n",_speed[0],_speed[1], _dt);

	gsl_vector_set(_state.z,0,pose.x);
	gsl_vector_set(_state.z,1,pose.y);
	gsl_vector_set(_state.z,2,measure.gps[0]);
	gsl_vector_set(_state.z,3,measure.gps[1]);
	
	//prediction

	//update x
	gsl_vector * Ax = gsl_vector_alloc(STATE_SIZE);
	gsl_vector * Bu = gsl_vector_alloc(STATE_SIZE);

	gsl_blas_dgemv(CblasNoTrans, 1.0, _state.A, _state.x, 0.0, Ax);
	gsl_blas_dgemv(CblasNoTrans, 1.0, _state.B, _state.u, 0.0, Bu);

	gsl_vector_add(Ax, Bu);
	gsl_vector_add(_state.x, Bu);

	gsl_vector_free(Ax);
	gsl_vector_free(Bu);

	//update sigma
	gsl_matrix * SAT = gsl_matrix_alloc(STATE_SIZE,STATE_SIZE); // sigma*A^T

	gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, _state.sigma, _state.A, 0.0, SAT); 
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, _state.A, SAT, 0.0, _state.sigma);
	gsl_matrix_add(_state.sigma,_state.R);

	gsl_matrix_free(SAT);

	//correction

	//compute K

	gsl_matrix * bloc1 = gsl_matrix_alloc(STATE_SIZE,MEASURE_SIZE); // sigma*C^T
	gsl_matrix * bloc2 = gsl_matrix_alloc(MEASURE_SIZE,MEASURE_SIZE); // C*sigma*C^T + Q

	gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, _state.sigma, _state.C, 0.0, bloc1);
	
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, _state.C, bloc1, 0.0, bloc2);

	gsl_matrix_add(bloc2,_state.Q);
	// compute inverse of bloc2
	int s; 
	gsl_matrix * inverse = gsl_matrix_alloc(MEASURE_SIZE, MEASURE_SIZE);
	gsl_permutation * perm = gsl_permutation_alloc(MEASURE_SIZE);

	// Make LU decomposition of matrix m
	gsl_linalg_LU_decomp (bloc2, perm, &s);

	// Invert the matrix m
	gsl_linalg_LU_invert (bloc2, perm, inverse);

	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, bloc1, bloc2, 0.0, _state.K);

	gsl_matrix_free(bloc1);
	gsl_matrix_free(bloc2);
	gsl_matrix_free(inverse);
	gsl_permutation_free(perm);

	//update x
	gsl_vector* Cx = gsl_vector_alloc(MEASURE_SIZE);
	gsl_vector* diff = gsl_vector_alloc(MEASURE_SIZE);
	gsl_vector* Kdiff = gsl_vector_alloc(STATE_SIZE);

	gsl_blas_dgemv(CblasNoTrans, 1.0, _state.C, _state.x, 0.0, Cx);
	
	gsl_vector_memcpy(diff,_state.z);
	gsl_vector_sub(diff,Cx);
	gsl_blas_dgemv(CblasNoTrans, 1.0, _state.K,diff, 0.0, Kdiff);

	gsl_vector_add(_state.x,Kdiff);

	gsl_vector_free(Cx);
	gsl_vector_free(diff);
	gsl_vector_free(Kdiff);

	//update sigma
	gsl_matrix* I = gsl_matrix_alloc(STATE_SIZE,STATE_SIZE);
	gsl_matrix* KC = gsl_matrix_alloc(STATE_SIZE,STATE_SIZE);
	gsl_matrix* sigma_c = gsl_matrix_alloc(STATE_SIZE,STATE_SIZE);
	gsl_matrix_set_identity(I);
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, _state.K, _state.C, 0.0, KC);
	gsl_matrix_sub(I,KC);
	gsl_matrix_memcpy(sigma_c,_state.sigma);
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, I, sigma_c, 0.0, _state.sigma);

	gsl_matrix_free(I);
	gsl_matrix_free(KC);
	gsl_matrix_free(sigma_c);

}

void kalman_filter_return_state(float x[2])
{
	x[0] = (float) gsl_vector_get(_state.x,0);
	x[1] = (float) gsl_vector_get(_state.x,1);
}

void kalman_filter_free()
{
	gsl_vector_free(_state.x);
	gsl_vector_free(_state.u);
	gsl_vector_free(_state.z);

	gsl_matrix_free(_state.sigma);
	gsl_matrix_free(_state.K);
	gsl_matrix_free(_state.A);
	gsl_matrix_free(_state.B);
	gsl_matrix_free(_state.C);
	gsl_matrix_free(_state.Q);
	gsl_matrix_free(_state.R);

	printf("[INFO KALMAN] Free memory done \n");
}

void kalman_print_all()
{
	print_gslmat(_state.sigma);
	print_gslmat(_state.K);
}

void print_gslmat (const gsl_matrix * m)
{
  for (int i = 0; i < m->size1; ++i)
    {
      for (int j = 0; j < m->size2; ++j)
	    printf ("%g ", gsl_matrix_get (m, i, j));
      printf ("\n");
    }
}


