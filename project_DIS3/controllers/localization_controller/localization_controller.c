#include <stdio.h>
#include <string.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/accelerometer.h>
#include <webots/position_sensor.h>
#include <webots/emitter.h>


#include "trajectories.h"
#include "utils.h"
#include "accelerometer.h"
#include "gps.h"
#include "kalman_filter.h"
#include "odometry.h"

WbDeviceTag dev_gps;
WbDeviceTag dev_acc;
WbDeviceTag dev_left_encoder;
WbDeviceTag dev_right_encoder;
WbDeviceTag dev_left_motor; 
WbDeviceTag dev_right_motor;
WbDeviceTag emitter;

void init_devices(int ts);
void send_position(measurement_t measure, pose_t pos);
void controller_init_log(const char* filename);
void controller_print_log(double time);

static FILE *fp;
static pose_t gps_pose;
static pose_t odo_pose;
static pose_t kal_pose;

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
	
	controller_init_log("Data_position.csv");
}

int main() 
{
	wb_robot_init();
	
	int time_step = wb_robot_get_basic_time_step();
	int first_kalman = 0;
	float state[2];
  
	measurement_t measure;
	pose_t pos;
	
	//Init. pos structure 
	pos.x = 0;
	pos.y = 0;
	pos.heading = 0;

	//initialization kalmann + robot + odometry
	init_devices(time_step);
	//kalman_filter_init(measure, time_step);
	odo_reset(time_step, &measure);
 
	while (wb_robot_step(time_step) != -1)  
	{
		//initialization accelerometer
		if(wb_robot_get_time() < TIME_INIT_ACC )
		{
			pos.x = measure.gps[0];
			pos.y = measure.gps[1];
			pos.heading = measure.gps[2];
			accelerometer_compute_mean_acc(time_step, &measure);
			gps_get_pose(&dev_gps, &measure, &pos);
		}
		else
		{
          		if(first_kalman == 0)
          		{
                	    kalman_filter_init(measure, time_step);
                	    first_kalman +=1;
          		}
                      
			//trajectory_1(dev_left_motor, dev_right_motor);
			trajectory_2(dev_left_motor, dev_right_motor);
			
			//measurement GPS
			gps_get_pose(&dev_gps, &measure, &pos);
			gps_pose.x = measure.gps[0];
			gps_pose.y = measure.gps[1];
	
			//measurement odometry (wheel encoder)
			odo_get_encoder(&measure, &dev_left_encoder, &dev_right_encoder);
			odo_compute_encoders(&pos, &measure);
			odo_pose = pos;
			//printf("[INFO CONTROL.] pos = %f %f \n",pos.x,pos.y);

			//Fix state with wheel odometry if no gps available
			//state[0] = pos.x;
			//state[1] = pos.y;
			//measurement accelerometer
			accelerometer_get_acc(&measure, &dev_acc, pos);
			
			//kalman

			kalman_filter_update(measure, pos);

			kalman_filter_return_state(state);
			
			kal_pose.x = state[0];
			kal_pose.y = state[1];
			
			pos.x = state[0];
			pos.y = state[1];
			
			//printf("[INFO MAIN] Kalman state : %f !\n",state.x[0]);
			//printf("[INFO MAIN] pose : %f !\n",pos.x);
			
			// Send actual state to superviser
			send_position(measure, pos);
			controller_print_log(wb_robot_get_time());
			//state[0] = pos.x;
			//state[1] = pos.y;
			//send_position(measure,state);
						
		}
	}
}

void send_position(measurement_t measure, pose_t pos)
{
	char outbuffer[255];

	//printf("[INFO SEND_POS.] x_estim = [%f %f] \n",x[0],x[1]);
	
	sprintf(outbuffer,"#%f#%f#%f", pos.x, pos.y, pos.heading);
	wb_emitter_send(emitter, outbuffer, strlen(outbuffer));
}	

void controller_print_log(double time)
{

  if( fp != NULL)
  {
    fprintf(fp, "%g; %g; %g; %g; %g; %g; %g;\n", 
            time, gps_pose.x, gps_pose.y, odo_pose.x, odo_pose.y, kal_pose.x, kal_pose.y);
  } 

}

void controller_init_log(const char* filename)
{
  fp = fopen(filename,"w");
  
  if (fp == NULL) {
    printf("Fails to create a log file\n");
  } else {
    fprintf(fp, "time; gps_x; gps_y; odometry_x; odometry_y; kalman_x; kalman_y; \n"); 
  }
}
