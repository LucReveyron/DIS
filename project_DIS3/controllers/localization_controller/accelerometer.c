#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/accelerometer.h>
#include <webots/position_sensor.h>

#include "accelerometer.h"


/**
 * @brief      Compute the mean of the 3-axis accelerometer. The result is stored in array _meas.acc
 */
void accelerometer_compute_mean_acc(int time_step, measurement_t * measurement)
{
	static int count = 0;
	int i;
  
	count++;
  
	if(count > 20) // Remove the effects of strong acceleration at the begining
	{
		for(i = 0; i < 3; i++)  
			measurement->acc_mean[i] = (measurement->acc_mean[i] * (count - 1) + measurement->acc[i]) / (double) count;
	}
  
	if(count == (int) (TIME_INIT_ACC / (double) time_step * 1000) )
		printf("Accelerometer initialization Done ! \n");

	//printf("ROBOT acc mean : %g %g %g\n", measurement->acc_mean[0], measurement->acc_mean[1] , measurement->acc_mean[2]);
}


void accelerometer_get_acc(measurement_t * measurement, WbDeviceTag *dev_acc, pose_t pose)
{
  const double * acc_values = wb_accelerometer_get_values(*dev_acc);

  memcpy(measurement->acc, acc_values, sizeof(measurement->acc));

  measurement->acc[0] = measurement->acc[1]*cos(pose.heading);
  measurement->acc[1] = measurement->acc[1]*sin(pose.heading);

  //printf("ROBOT acc : %g %g %g\n", measurement->acc[0], measurement->acc[1] , measurement->acc[2]);
}
/*
void odo_compute_acc(int time_step, pose_t * position, measurement_t * measurement double heading)
{
    static double odo_acc_v;
    
    static double odo_acc_x;
    static double odo_acc_y;
    
    double odo_acc = measurement->acc[0] - measurement->acc_mean[0];

	odo_acc_v += odo_acc*time_step;
	
	position->x += odo_acc_v*time_step*cos(RAD2DEG(heading));
	position->y += odo_acc_v*time_step*sin(RAD2DEG(heading));
	position->heading = heading;

 	printf("ODO with acceleration : %g %g %g\n", position->x , position->y , position->heading);
	
}


void odo_compute_acc(pose_t* odo, const double acc[3], const double acc_mean[3])
{



    _odo_speed_acc.x += acc_wx*_T;
    _odo_speed_acc.y += acc_wy*_T;


    _odo_pose_acc.x += _odo_speed_acc.x*_T;
    _odo_pose_acc.y += _odo_speed_acc.y*_T;


    memcpy(odo, &_odo_pose_acc, sizeof(pose_t));	
}
*/
