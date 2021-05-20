#include <stdio.h>
#include <math.h>
#include <string.h>

#include "communication.h"

static int robot_id_u;	// Unique and normalized (between 0 and FLOCK_SIZE-1), robot ID

static char buffer[255];
static char *inbuffer;

int communication_init(WbDeviceTag * receiver, WbDeviceTag * emitter, WbDeviceTag ds[NB_SENSORS], int initialized[FLOCK_SIZE], int * unique_robot_id)
{
	int robot_id;
    *receiver = wb_robot_get_device("receiver");
	*emitter = wb_robot_get_device("emitter");

    int i;
	char s[4]="ps0";
	for(i=0; i<NB_SENSORS;i++) 
    {
		ds[i]=wb_robot_get_device(s);			// the device name is specified in the world file
		s[2]++;									// increases the device number
	}
	char* robot_name; 
	robot_name=(char*) wb_robot_get_name(); 

	for(i=0;i<NB_SENSORS;i++) 
    {
		wb_distance_sensor_enable(ds[i],64);
	}
	wb_receiver_enable(*receiver,64);

	//Reading the robot's name. Pay attention to name specification when adding robots to the simulation!
	sscanf(robot_name,"epuck%d",&robot_id_u); 	// read robot id from the robot's name
	robot_id = robot_id_u%FLOCK_SIZE;	  		// normalize between 0 and FLOCK_SIZE-1
	* unique_robot_id = robot_id_u;

	for(i=0; i<FLOCK_SIZE; i++) {
		initialized[i] = 0; 		  			// Set initialization to 0 (= not yet initialized)
	}

    return robot_id;
}

void communication_send_loc(WbDeviceTag emitter, float loc[FLOCK_SIZE][3], int robot_id)
{
    sprintf(buffer,"%d#%f#%f#%f",robot_id,loc[robot_id][0],loc[robot_id][1],loc[robot_id][2]);
    wb_emitter_send(emitter,buffer,strlen(buffer));
    
}

void leader_send_loc(WbDeviceTag emitter, pose_t pose, int robot_id_u)
{
    sprintf(buffer,"%d#%f#%f#%f",robot_id_u,pose.x,pose.y,pose.heading);
    wb_emitter_send(emitter,buffer,strlen(buffer));
    // printf("COMMUNICATION : leader is sending pose = (%f,%f)\n", pose.x, pose.y);
}

void get_leader_loc(WbDeviceTag receiver, pose_t * pose, int * leader_id)
{
	static int comm_id;
	static float rob_x, rob_z, rob_theta; // Robot position and orientation

    // wait for message
	while (wb_receiver_get_queue_length(receiver) == 0)	wb_robot_step(TIME_STEP);

    inbuffer = (char*) wb_receiver_get_data(receiver);
    sscanf(inbuffer,"%d#%f#%f#%f",&comm_id,&rob_x,&rob_z,&rob_theta);
    * leader_id = comm_id;
    
	// Initialize self position
	pose->x = rob_x; 					// x-position
	pose->y = rob_z; 					// z-position
	pose->heading = rob_theta; 			// theta
	
	wb_receiver_next_packet(receiver);
	// printf("COMMUNICATION : follower is receiving pose = (%f,%f)\n", pose->x, pose->y);
}

void communication_get_self_pos(WbDeviceTag receiver,float loc[FLOCK_SIZE][3],float prev_loc[FLOCK_SIZE][3],float migr[2], int robot_id)
{
    static int rob_nb;
	static float rob_x, rob_z, rob_theta; // Robot position and orientation

    // wait for message
	while (wb_receiver_get_queue_length(receiver) == 0)	wb_robot_step(TIME_STEP);

    inbuffer = (char*) wb_receiver_get_data(receiver);
    sscanf(inbuffer,"%d#%f#%f#%f##%f#%f",&rob_nb,&rob_x,&rob_z,&rob_theta, &migr[0], &migr[1]);

    if (rob_nb == robot_id) 
    {
		// Initialize self position
		loc[rob_nb][0] = rob_x; 		// x-position
		loc[rob_nb][1] = rob_z; 		// z-position
		loc[rob_nb][2] = rob_theta; 		// theta
		prev_loc[rob_nb][0] = loc[rob_nb][0];
		prev_loc[rob_nb][1] = loc[rob_nb][1];
	}		
	wb_receiver_next_packet(receiver);

}

void communication_get_all_pos(WbDeviceTag receiver, int initialized[FLOCK_SIZE],float loc[FLOCK_SIZE][3],float prev_loc[FLOCK_SIZE][3],float speed[FLOCK_SIZE][2], int robot_id )
{
    int rob_nb;			// Robot number
	float rob_x, rob_z, rob_theta;  // Robot position and orientation

    /* Get information */
    int count = 0;
    while (wb_receiver_get_queue_length(receiver) > 0 && count < FLOCK_SIZE) 
    {
        inbuffer = (char*) wb_receiver_get_data(receiver);
        sscanf(inbuffer,"%d#%f#%f#%f",&rob_nb,&rob_x,&rob_z,&rob_theta);
        
        if ((int) rob_nb/FLOCK_SIZE == (int) robot_id/FLOCK_SIZE) {
			rob_nb %= FLOCK_SIZE;
			if (initialized[rob_nb] == 0) {
				// Get initial positions
				loc[rob_nb][0] = rob_x; //x-position
				loc[rob_nb][1] = rob_z; //z-position
				loc[rob_nb][2] = rob_theta; //theta
				prev_loc[rob_nb][0] = loc[rob_nb][0];
				prev_loc[rob_nb][1] = loc[rob_nb][1];
				initialized[rob_nb] = 1;
			} else {
				// Get position update
				// printf("\n got update robot[%d] = (%f,%f) \n",rob_nb,loc[rob_nb][0],loc[rob_nb][1]);
				prev_loc[rob_nb][0] = loc[rob_nb][0];
				prev_loc[rob_nb][1] = loc[rob_nb][1];
				loc[rob_nb][0] = rob_x; //x-position
				loc[rob_nb][1] = rob_z; //z-position
				loc[rob_nb][2] = rob_theta; //theta
			}
        
			speed[rob_nb][0] = (1/DELTA_T)*(loc[rob_nb][0]-prev_loc[rob_nb][0]);
			speed[rob_nb][1] = (1/DELTA_T)*(loc[rob_nb][1]-prev_loc[rob_nb][1]);
			count++;
        }
        
        wb_receiver_next_packet(receiver);
    }
}
