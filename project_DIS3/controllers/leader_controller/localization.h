#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include <webots/emitter.h>

#include "communication.h"
#include "../localization_controller/utils.h"
#include "../localization_controller/accelerometer.h"
#include "../localization_controller/gps.h"
#include "../localization_controller/kalman_filter.h"
#include "../localization_controller/odometry.h"

void localization_reset(int time_step);
int localization_get(int time_step, pose_t * current_pos, WbDeviceTag emitter);

#endif