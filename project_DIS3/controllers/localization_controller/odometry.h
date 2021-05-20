#ifndef ODOMETRY_H
#define ODOMETRY_H 

#include <webots/robot.h>
#include <webots/gps.h>
#include <webots/position_sensor.h>

#include "utils.h"

void odo_compute_encoders(pose_t* odo, measurement_t* measurement);
void odo_get_encoder(measurement_t* measure, WbDeviceTag *dev_left_encoder, WbDeviceTag *dev_right_encoder);
void odo_reset(int time_step, measurement_t* measure);

#endif
