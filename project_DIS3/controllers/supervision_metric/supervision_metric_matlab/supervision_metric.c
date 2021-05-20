#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <webots/robot.h>
#include <webots/supervisor.h>
#include "../common/utils.h"

#define FLOCK_SIZE	1
	
WbNodeRef   robs[FLOCK_SIZE];		// Robots nodes
WbFieldRef  robs_trans[FLOCK_SIZE];	// Robots translation fields
WbFieldRef  robs_rotation[FLOCK_SIZE];	// Robots rotation fields

double last_metric_time_s = 0.0;

static FILE *fp;

void controller_init_log(const char* filename);
void controller_print_log(double time, pose_t position);

// Function to reset the supervisor, inspired by tp4
void reset(void) {
  wb_robot_init();
  printf("Supervisor initialized\n");
  
  int i = 0;
  	
  char rob[7] = "ROBOT1";
  robs[i] = wb_supervisor_node_get_from_def(rob);
  robs_trans[i] = wb_supervisor_node_get_field(robs[i],"translation");
  robs_rotation[i] = wb_supervisor_node_get_field(robs[i],"rotation");
  
  controller_init_log("Exact_position.csv");
}
	
// Main function
int main() {
  
  reset();
  
  int time_step = wb_robot_get_basic_time_step();
		
  int i = 0;
  
  pose_t exact_position;		
  
  while (wb_robot_step(time_step) != -1)  {
    
    exact_position.x = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[0]; // X
    exact_position.y = -(wb_supervisor_field_get_sf_vec3f(robs_trans[i])[2]); // Z
    exact_position.heading = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3]; // THETA
    
    controller_print_log(wb_robot_get_time(), exact_position);
  
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
  }
}