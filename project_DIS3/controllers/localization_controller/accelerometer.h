#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H 

#include <webots/robot.h>
#include <webots/motor.h>

#include "utils.h"

void accelerometer_compute_mean_acc(int time_step, measurement_t * measurement);
void accelerometer_get_acc(measurement_t * measurement, WbDeviceTag *dev_acc, pose_t pose);


#endif
