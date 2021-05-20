#include "gps.h"


void gps_get_pose(WbDeviceTag * dev_gps, measurement_t * measurement, pose_t * pose) 
{
	static double last_gps_time_s = 0.0;
	double time_now = wb_robot_get_time();
  
	if (time_now - last_gps_time_s >= 1.0) 
	{
		last_gps_time_s = time_now;
    
		// Store the previous measurement in gps_meas
		memcpy(measurement->prev_gps, measurement->gps, sizeof(measurement->gps));
    
		// Get position from GPS
		const double * gps_position = wb_gps_get_values(* dev_gps);
    
		// Store the new measurement in gps_meas
		memcpy(measurement->gps, gps_position, sizeof(measurement->gps));
    
		// Print info if wanted
		if(VERBOSE_GPS)
		{
			//printf("ROBOT gps is at position: %g %g %g\n", measurement->gps[0], measurement->gps[1], measurement->gps[2]);
		}
		// If first 
    
		// Get the position
		measurement->gps[1] = -(measurement->gps[2]);

		measurement->gps[2] = gps_get_heading(measurement);
		pose->heading = measurement->gps[2];
  
		if(VERBOSE_POSE) 
		{
			//printf("ROBOT pose : x : %g y : %g theta : %g\n", measurement->gps[0] , measurement->gps[1] , measurement->gps[2]);
		}
	}
}

double gps_get_heading(measurement_t * measurement) 
{
	static bool first_gps = true;
	double heading = -M_PI/2;
  
	// Compute difference in x and y position
	double d_x = measurement->gps[0] - measurement->prev_gps[0];
	double d_y = -(measurement->gps[1] - measurement->prev_gps[1]);
  
	// Compute heading from x and y difference
	// Test if first time the gps has been, if it's the case, heading doesn't change
	if (first_gps) {
		first_gps = false;
	} 
	else{
		heading = heading - atan2(d_y, d_x);
		while(heading < - M_PI) heading += 2*M_PI;
		while(heading > M_PI) heading -= 2*M_PI;
	}
  
	return heading;
}
