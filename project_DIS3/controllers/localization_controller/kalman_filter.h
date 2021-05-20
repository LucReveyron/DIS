#ifndef  _KALMAN_FILTER_H
#define  _KALMAN_FILTER_H

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include "utils.h"

typedef struct {

    gsl_vector * x;     	        /* state: [0] x, [1] y, 4x1 */
    gsl_vector * u;     	        /* control vector, 4x1 */
    gsl_vector * z;			        /* measurement vector: [0] x-odo, [1] y-odo, [2] x-gps and [3] y-gps, 4x1 */
    gsl_matrix * sigma;             /* estimated error convariance,4x4 [s0 s1; s2 s3] */
    gsl_matrix * A;  	            /* Process model, 4x4 */
    gsl_matrix * B;		            /* Control matrix, 4x4 */
    gsl_matrix * K;		            /* Kalman gain, 4x4 */
    gsl_matrix * C;		            /* Measurement model, 4x4 */
    gsl_matrix * Q;                 /* Measure noise convariance,4x4 */
    gsl_matrix * R;  	            /* Process (predict) noise convariance, 4x4 */

} kalman_state;  

void kalman_filter_init(measurement_t measure, int ts);
void kalman_filter_update();
void kalman_filter_return_state(float x[2]);
void kalman_filter_free();
void kalman_print_all();
void print_gslmat (const gsl_matrix * m);

#endif // _KALMAN_FILTER_H