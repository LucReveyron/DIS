
#include "localization.h"

WbDeviceTag dev_gps;
WbDeviceTag dev_acc;
WbDeviceTag dev_left_encoder;
WbDeviceTag dev_right_encoder;

int first_kalman = 0;
int acc_calib = 0;
float state[2];
measurement_t measure;

void send_position(WbDeviceTag emitter, measurement_t measure, pose_t pos);

void localization_reset(int time_step)
{
    odo_reset(time_step, &measure);
}

int localization_get(int time_step, pose_t * current_pos, WbDeviceTag emitter)
{
    pose_t pos = *current_pos;

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
        acc_calib = 1; 
        // initialization of the kalman filter
        if(first_kalman == 0)
        {
            kalman_filter_init(measure, time_step);
            first_kalman +=1;
        }

        // measurement GPS
	    gps_get_pose(&dev_gps, &measure, &pos);

        // measurement odometry (wheel encoder)
		odo_get_encoder(&measure, &dev_left_encoder, &dev_right_encoder);
		odo_compute_encoders(&pos, &measure);

        // measurement accelerometer
		accelerometer_get_acc(&measure, &dev_acc, pos);

        // update kalman filter
        kalman_filter_update(measure, pos);
        kalman_filter_return_state(state);

        // update measure with kalman estimation
        pos.x = state[0];
		pos.y = state[1];

        // Send actual state to superviser for metric estimation
	    send_position(emitter, measure, pos);

        // update current position 
        *current_pos = pos;
    }
    return acc_calib;
}

void send_position(WbDeviceTag emitter, measurement_t measure, pose_t pos)
{
	char outbuffer[255];
	
	sprintf(outbuffer,"#%f#%f#%f", pos.x, pos.y, pos.heading);
	wb_emitter_send(emitter, outbuffer, strlen(outbuffer));
}