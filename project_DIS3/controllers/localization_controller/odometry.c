
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include "odometry.h"

//-----------------------------------------------------------------------------------//
/*CONSTANTES*/
#define WHEEL_AXIS 		0.052 			// Distance between the two wheels in meter
#define WHEEL_RADIUS 	0.0205			// Radius of the wheel in meter

//-----------------------------------------------------------------------------------//

/*GLOBAL*/
static double T;

//-----------------------------------------------------------------------------------//

/**
 * @brief      Compute the odometry using the encoders
 *
 * @param      odo         The odometry
 * @param[in]  measurement The measurements of a robot
 */
void odo_compute_encoders(pose_t* odo, measurement_t* measurement)
{
	double Aleft_enc = 0;
	double Aright_enc =0;
	
	// Compute delta of encoders
	Aleft_enc = measurement->left_enc - measurement->prev_left_enc;
	Aright_enc = measurement->right_enc - measurement->prev_right_enc;
	
	// Rad to meter
	Aleft_enc  *= WHEEL_RADIUS;

	Aright_enc *= WHEEL_RADIUS;
	
	// Save the wheels distance
	measurement->dist_wheel_left += Aleft_enc;
	measurement->dist_wheel_right += Aright_enc;

	// Compute forward speed and angular speed
	double omega = -( Aright_enc - Aleft_enc ) / ( WHEEL_AXIS * T );

	double speed = ( Aright_enc + Aleft_enc ) / ( 2.0 * T );

	// Apply rotation (Body to World)
	
	// (1/2) smaller integration step for the angle to have a more precise speed_wx and speed_wy
	odo->heading += omega * T / 2.0;

	// Computation of X and Y absolute
	double a = odo->heading; // With wheels
	//double a = measurement->gps[2]; // With gps
	double speed_wx = speed * cos(a);
	double speed_wy = speed * sin(a);

	odo->x += speed_wx * T;
	//printf("[INFO ODO.] odo x : %f, odo y : %f \n",odo->x,odo->y);
	odo->y += speed_wy * T;

	// (2/2) smaller integration step for the angle 
	odo->heading += omega * T/ 2.0;

}

/**
 * @brief      Reset the odometry inner variables to zeros
 *
 * @param[in]  time_step  The time step used in the simulation in miliseconds
 */
void odo_reset(int time_step, measurement_t* measurement)
{
	measurement->left_enc = 0;
	measurement->right_enc = 0;

	measurement->dist_wheel_left = 0;
	measurement->dist_wheel_right = 0;
	T = time_step / 1000.0;
}


/**
 * @brief      Read the encoders values from the sensors
 */
void odo_get_encoder(measurement_t* measure, WbDeviceTag * dev_left_encoder, WbDeviceTag * dev_right_encoder)
{
  // Store previous value of encoders
  measure->prev_left_enc = measure->left_enc;
  measure->prev_right_enc = measure->right_enc;
   
  
  // Get new value
  measure->left_enc = wb_position_sensor_get_value(*dev_left_encoder);
  measure->right_enc = wb_position_sensor_get_value(*dev_right_encoder);

}
