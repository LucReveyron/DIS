#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <webots/robot.h>
#include <webots/receiver.h>
#include <webots/supervisor.h>

//#define FLOCK_SIZE	1
	
WbDeviceTag receiver;			// Handle for the receiver node
WbNodeRef   robs;		           // Robots nodes
WbFieldRef  robs_trans;	           // Robots translation fields
WbFieldRef  robs_rotation;                   // Robots rotation fields
WbFieldRef  robs_arg;

float rob_exact_loc[3]	;           // array to save exact position & orientation of the robots
float rob_estim_loc[3]	;           // array to save estimated position & orientation of the robots

double last_metric_time_s = 0.0;

int POS_INTO_METRICS = 2;                     // Number of positions to put into the metric, 2 for (x,y) only and 3 to add orientation

static FILE *fp;

void controller_init_log(const char* filename);
void controller_print_log(double time, float exact_position[3], float estimated_position[3]);

// Function to reset the supervisor, inspired by tp4
void reset(void) {
  wb_robot_init();
  receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver,64);	
  	
  char rob[7] = "ROBOT1";
  robs = wb_supervisor_node_get_from_def(rob);
  robs_trans = wb_supervisor_node_get_field(robs,"translation");
  robs_rotation = wb_supervisor_node_get_field(robs,"rotation");
  robs_arg = wb_supervisor_node_get_field(robs,"controllerArgs");
  
  controller_init_log("True_position.csv");
}
	
// Function to compute error metric and print it
void metric(float rob_exact_loc[3], float rob_estim_loc[3]) {
  float error = 0.0;

  int j;
	
  for (j = 0; j<POS_INTO_METRICS; j++) {
    error += powf(rob_exact_loc[j] - rob_estim_loc[j],2);
  }
  error = sqrtf(error);
  printf("Exact pose : %f %f %f\n", rob_exact_loc[0], rob_exact_loc[1], rob_exact_loc[2]);
  printf("Estim pose : %f %f %f\n", rob_estim_loc[0], rob_estim_loc[1], rob_estim_loc[2]);

printf("Metric error: %f\n", error);
}
	
// Main function
int main() {
  printf("Supervisor test\n");
  
  reset();
  
  int time_step = wb_robot_get_basic_time_step();
		
  char *inbuffer;
  float est_x, est_y, est_theta;	// Storage for the location from the robot		
	
  for(;;) {
    wb_robot_step(time_step);
			
    // Get the position estimated from the robot
    // wait for message from robot
    while (wb_receiver_get_queue_length(receiver) == 0)	wb_robot_step(time_step);
    
    // Get the message
    inbuffer = (char*) wb_receiver_get_data(receiver);
    sscanf(inbuffer,"#%f#%f#%f",&est_x,&est_y,&est_theta);
    
    //printf("est_x = %f\n",est_x);
	
    // Store the data received
    rob_estim_loc[0] = (float) est_x;
    rob_estim_loc[1] = (float) est_y;
    rob_estim_loc[2] = (float) est_theta;
	
    // Get exact position, see if only needed for the robot who communicated or all
    rob_exact_loc[0] = (float) wb_supervisor_field_get_sf_vec3f(robs_trans)[0]; // X
    rob_exact_loc[1] = (float) -wb_supervisor_field_get_sf_vec3f(robs_trans)[2]; // Z
    rob_exact_loc[2] = (float) wb_supervisor_field_get_sf_rotation(robs_rotation)[3]; // THETA
			
    // Compute metric for the robot who sent data
    double time_now = wb_robot_get_time();
  
    if (time_now - last_metric_time_s > 0.5) {
      last_metric_time_s = time_now;
      metric(rob_exact_loc, rob_estim_loc);
    }
    
    controller_print_log(time_now, rob_exact_loc, rob_estim_loc);
    	
    wb_receiver_next_packet(receiver);
			
  }
  
  if(fp != NULL)
    fclose(fp);
}

void controller_print_log(double time, float exact_position[3], float estimated_position[3])
{

  if( fp != NULL)
  {
    fprintf(fp, "%g; %g; %g; %g; %g; %g; %g;\n", 
            time, exact_position[0], exact_position[1] , exact_position[2], estimated_position[0], estimated_position[1], estimated_position[2]);
  } 

}

void controller_init_log(const char* filename)
{

  fp = fopen(filename,"w");
  
  if (fp == NULL) {
    printf("Fails to create a log file\n");
  } else {
    fprintf(fp, "time; exact_x; exact_y; exact_heading; estimated_x; estimated_y; estimated_heading; \n"); 
  }
}
