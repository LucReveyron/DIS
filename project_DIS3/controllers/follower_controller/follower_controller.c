#include <stdio.h>
#include <string.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/gps.h>
#include <webots/accelerometer.h>
#include <webots/position_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>

#include "localization.h"
#include "communication.h"
#include "definition.h"
#include "../localization_controller/utils.h"
#include "../localization_controller/accelerometer.h"
#include "../localization_controller/gps.h"
#include "../localization_controller/odometry.h"

WbDeviceTag dev_gps;
WbDeviceTag dev_acc;
WbDeviceTag dev_left_encoder;
WbDeviceTag dev_right_encoder;
WbDeviceTag dev_left_motor; 
WbDeviceTag dev_right_motor;
WbDeviceTag emitter; 		        // Handle for the emitter node
WbDeviceTag receiver; 		        // Handle for the receiver node
WbDeviceTag ds[NB_SENSORS];         // Handle for the infrared distance sensors

int initialized[FLOCK_SIZE];		// != 0 if initial positions have been received

float dist_to_leader[4] = {0.15, 0.15, 0.15, 0.15};
float theta_to_leader[4] = {-M_PI/4, -3*M_PI/4, M_PI/4, 3*M_PI/4};

float migr[2];	                    // Migration vector
static int robot_id, robot_id_u;	// Unique and normalized (between 0 and FLOCK_SIZE-1), robot ID

void init_devices(int ts);
void formation_limit(int *number, int limit);
void formation_follower_compute_wheel_speeds(int *msl, int *msr, float range, float bearing, int robot_id);

int main()
{
    wb_robot_init();

    pose_t pos;
	pose_t leader_pos;
	int acc_calib = 0;
    int time_step = wb_robot_get_basic_time_step();
	float range;
	int msl, msr;										// Wheels speed
	float msl_w, msr_w;
	float diff_to_leader[3];							// Difference to leader position
	float diff_to_goal[3];								// Difference to formation position
	int my_leader_id_u;

    //Init. pos structure 
	pos.x = 0;
	pos.y = 0;
	pos.heading = 0;

	//initialization robot + odometry
	init_devices(time_step);
    localization_reset(time_step);


    while (wb_robot_step(time_step) != -1)
    {
        // update current position
        acc_calib = localization_get(time_step, &pos, emitter);
		if(!acc_calib)
			continue;

		// Get position of the leader
		get_leader_loc(receiver, &leader_pos, &my_leader_id_u);

		// Compute difference between leader position and current estimate of own position
		diff_to_leader[0] = leader_pos.x - pos.x;
		diff_to_leader[1] = leader_pos.y - pos.y;
		diff_to_goal[0] = diff_to_leader[0] - dist_to_leader[robot_id-1]*sinf(theta_to_leader[robot_id-1]);
		diff_to_goal[1] = diff_to_leader[1] - dist_to_leader[robot_id-1]*cosf(theta_to_leader[robot_id-1]);

		diff_to_goal[2] = - pos.heading - M_PI/2 + atan2(diff_to_goal[1], diff_to_goal[0]);			// angle to leader
		range = sqrt(pow(diff_to_goal[0], 2) + pow(diff_to_goal[1], 2));							// distance to leader

		// Compute speed towards goal
		formation_follower_compute_wheel_speeds(&msl, &msr, range, diff_to_goal[2], robot_id);

		msl *= FOLLOWER_WEIGHT;
		msr *= FOLLOWER_WEIGHT;

		// Set motor speed
		msl_w = msl*MAX_SPEED_WEB/1000;
		msr_w = msr*MAX_SPEED_WEB/1000;
		wb_motor_set_velocity(dev_left_motor, msl_w);
		wb_motor_set_velocity(dev_right_motor, msr_w);

    }
}

void init_devices(int ts)
{
	dev_gps = wb_robot_get_device("gps");
	wb_gps_enable(dev_gps, 1000);
  
	dev_acc = wb_robot_get_device("accelerometer");
	wb_accelerometer_enable(dev_acc, ts);
  
	dev_left_encoder = wb_robot_get_device("left wheel sensor");
	dev_right_encoder = wb_robot_get_device("right wheel sensor");
	wb_position_sensor_enable(dev_left_encoder,  ts);
	wb_position_sensor_enable(dev_right_encoder, ts);

	dev_left_motor = wb_robot_get_device("left wheel motor");
	dev_right_motor = wb_robot_get_device("right wheel motor");
	wb_motor_set_position(dev_left_motor, INFINITY);
	wb_motor_set_position(dev_right_motor, INFINITY);
	wb_motor_set_velocity(dev_left_motor, 0.0);
	wb_motor_set_velocity(dev_right_motor, 0.0);

	emitter = wb_robot_get_device("emitter");

	robot_id = communication_init(&receiver, &emitter, ds, initialized, &robot_id_u);
}

void formation_follower_compute_wheel_speeds(int *msl, int *msr, float range, float bearing, int robot_id) 
{	
	// Compute forward control
	float u = KU_FOLLOWER*range*cosf(bearing);
	// Compute rotational control
	float w = KW_FOLLOWER*bearing;
	//printf("u : %f, w : %f, ", u, w);
	
	// Convert to wheel speeds!
	*msl = (u - AXLE_LENGTH*w/2.0) * (1000.0 / WHEEL_RADIUS);
	*msr = (u + AXLE_LENGTH*w/2.0) * (1000.0 / WHEEL_RADIUS);
	//printf("msl : %d, msr : %d\n", *msl, *msr);

	formation_limit(msl,MAX_SPEED);
	formation_limit(msr,MAX_SPEED);
	
/*	if (robot_id == 3) {
		printf("FORMATION INFO : u %f, w %f, range %f, bearing %f, motor speed (%d, %d)\n", u, w , range, bearing, *msl, *msr);
	}*/
}

//Keep given int number within interval {-limit, limit}
void formation_limit(int *number, int limit) 
{
	if (*number > limit)
		*number = limit;
	if (*number < -limit)
		*number = -limit;
}