#ifndef  _KALMAN_FILTER_H
#define  _KALMAN_FILTER_H

typedef struct {

    float x[2];     	/* state: [0] x, [1] y, 2x1 */
    float u[2];     	/* control vector, 2x1 */
    float z[4];			/* measurement vector: [0] x-odo, [1] y-odo, [2] x-gps and [3] y-gps, 4x1 */
    float sigma[2][2];  /* estimated error convariance,4x4 [s0 s1; s2 s3] */
    float A[2][2];  	/* Process model, 2x2 */
    float B[2][2];		/* Control matrix, 2x2 */
    float K[2][4];		/* Kalman gain, 2x4 */
    float C[4][2];		/* Measurement model, 4x2 */
    float Q[4][4];  /* Measure noise convariance,2x2 [q0,0; 0,q1] */
    float R[2][2];  	/* Process (predict) noise convariance, 2x2 */

} kalman_state;  

typedef struct 
{
  double prev_gps[3];
  double gps[3];
  double acc_mean[3];
  double acc[3];
  double prev_left_enc;
  double left_enc;
  double prev_right_enc;
  double right_enc;
} measurement_t;

void kalman_filter_init(measurement_t measure, kalman_state *state, int ts);
void kalman_filter_update(measurement_t measure, kalman_state *state, int ts);
void compute_kalman_gain(kalman_state *state);
void compute_measure_distance(kalman_state *state, float* z_dif);
float invf(int i, int j, const float* m);
bool inverseMatrix4x4(const float m[4][4], float out[4][4]);

#endif // _KALMAN_FILTER_H