#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <webots/robot.h>
#include <webots/receiver.h>
#include <webots/supervisor.h>
#include "../common/utils.h"

#define FLOCK_SIZE	1
	
WbDeviceTag receiver;			// Handle for the receiver node
WbNodeRef   robs[FLOCK_SIZE];		// Robots nodes
WbFieldRef  robs_trans[FLOCK_SIZE];	// Robots translation fields
WbFieldRef  robs_rotation[FLOCK_SIZE];	// Robots rotation fields
WbFieldRef  robs_arg;

float rob_exact_loc[FLOCK_SIZE][3]	;           // array to save exact position & orientation of the robots
float rob_estim_loc[FLOCK_SIZE][3]	;           // array to save estimated position & orientation of the robots

double last_metric_time_s = 0.0;

int POS_INTO_METRICS = 2;                     // Number of positions to put into the metric, 2 for (x,y) only and 3 to add orientation

static FILE *fp;

void controller_init_log(const char* filename);
void controller_print_log(double time, pose_t position);

// Function to reset the supervisor, inspired by tp4
void reset(void) {
  wb_robot_init();
  receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver,64);	
  
  int i = 0;
  	
  char rob[7] = "ROBOT1";
  robs[i] = wb_supervisor_node_get_from_def(rob);
  robs_trans[i] = wb_supervisor_node_get_field(robs[i],"translation");
  robs_rotation[i] = wb_supervisor_node_get_field(robs[i],"rotation");
  robs_arg = wb_supervisor_node_get_field(robs[i],"controllerArgs");
  
  controller_init_log("True_position.csv");
}
	
// Function to compute error metric and print it
void metric(float rob_exact_loc[FLOCK_SIZE][3], float rob_estim_loc[FLOCK_SIZE][3]) {
  float error[FLOCK_SIZE] = {0.0};

  int i; int j;
	
  for (i=0; i<FLOCK_SIZE; i++) {
    for (j = 0; j<POS_INTO_METRICS; j++) {
      error[i] += powf(rob_exact_loc[i][j] - rob_estim_loc[i][j],2);
    }
    error[i] = sqrtf(error[i]);
  }

printf("Metric error: %f\n", error[0]);
}
	
// Main function
int main() {
  printf("Supervisor test\n");
  
  reset();
  
  int time_step = wb_robot_get_basic_time_step();
		
  int i;
  char *inbuffer;
  float est_x, est_y, est_theta;	// Storage for the location from the robot		
	
  for(;;) {
    wb_robot_step(time_step);
			
    // Get the position estimated from the robot
    // wait for message from robot
    while (wb_receiver_get_queue_length(receiver) == 0)	wb_robot_step(time_step);
    
    // Get the message
    inbuffer = (char*) wb_receiver_get_data(receiver);
    sscanf(inbuffer,"%f#%f#%f",&est_x,&est_y,&est_theta);
		
    // Store the data received
    rob_estim_loc[0][0] = est_x;
    rob_estim_loc[0][1] = est_y;
    rob_estim_loc[0][2] = est_theta;
	
    // Get exact position, see if only needed for the robot who communicated or all
    for(i=0;i<FLOCK_SIZE;i++) {
      rob_exact_loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[0]; // X
      rob_exact_loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[2]; // Z
      rob_exact_loc[i][2] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3]; // THETA
    }
			
    // Compute metric for the robot who sent data
    double time_now = wb_robot_get_time();
  
    if (time_now - last_metric_time_s > 0.5) {
      last_metric_time_s = time_now;
      metric(rob_exact_loc, rob_estim_loc);
    }
    
    
    	
    wb_receiver_next_packet(receiver);
			
  }
  
  if(fp != NULL)
    fclose(fp);
}

void controller_print_log(double time, pose_t position)
{

  if( fp != NULL)
  {
    fprintf(fp, "%g; %g; %g; %g;\n", 
            time, position.x, position.y , position.heading);
            //%g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g\n",
            //, _meas.gps[0], _meas.gps[1], 
            //_meas.gps[2], _meas.acc[0], _meas.acc[1], _meas.acc[2], _meas.right_enc, _meas.left_enc, 
            //_odo_acc.x, _odo_acc.y, _odo_acc.heading, _odo_enc.x, _odo_enc.y, _odo_enc.heading, _odo_enc_bonus.x, _odo_enc_bonus.y, _odo_enc_bonus.heading);
  }

}

void controller_init_log(const char* filename)
{

  fp = fopen(filename,"w");
  
  if (fp == NULL) {
    printf("Fails to create a log file\n");
  } else {
    fprintf(fp, "time; exact_x; exact_y; exact_heading;\n"); 
    // gps_x; gps_y; gps_z; acc_0; acc_1; acc_2; right_enc; left_enc; odo_acc_x; odo_acc_y; odo_acc_heading; odo_enc_x; odo_enc_y; odo_enc_heading; odo_enc_bonus_x; odo_enc_bonus_y; odo_enc_bonus_heading\n");
  }
}