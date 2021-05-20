#ifndef  _UTILS_H
#define  _UTILS_H

#define TIME_INIT_ACC 5

typedef struct 
{
  double x;
  double y;
  double heading;
} pose_t;

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
  double dist_wheel_left;
  double dist_wheel_right;
} measurement_t;

#endif
