#ifndef GPS_H
#define GPS_H 

#include <stdio.h>
#include <string.h>

#include "utils.h"

#include <webots/robot.h>
#include <webots/gps.h>

#define VERBOSE_GPS true        // Print GPS values
#define VERBOSE_POSE true        // Print GPS values

void gps_get_pose(WbDeviceTag * dev_gps, measurement_t * measurement, pose_t * pose);
double gps_get_heading(measurement_t * measurement);

#define RAD2DEG(X)      X / M_PI * 180.0

#endif
