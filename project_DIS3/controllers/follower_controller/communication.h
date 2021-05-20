#ifndef COMMUNICATION_H
#define COMMUNICATION_H 

#include <webots/robot.h>
/*Webots 2018b*/
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/distance_sensor.h>
#include "definition.h"
#include "../localization_controller/utils.h"

int communication_init(WbDeviceTag * receiver, WbDeviceTag * emitter, WbDeviceTag ds[NB_SENSORS], int initialized[FLOCK_SIZE], int * unique_robot_id);
void communication_send_loc(WbDeviceTag emitter, float loc[FLOCK_SIZE][3], int robot_id);
void leader_send_loc(WbDeviceTag emitter, pose_t pose, int robot_id_u);
void get_leader_loc(WbDeviceTag receiver, pose_t * pose, int * leader_id);
void communication_get_self_pos(WbDeviceTag receiver,float loc[FLOCK_SIZE][3],float prev_loc[FLOCK_SIZE][3],float migr[2], int robot_id);
void communication_get_all_pos(WbDeviceTag receiver, int initialized[FLOCK_SIZE],float loc[FLOCK_SIZE][3],float prev_loc[FLOCK_SIZE][3],float speed[FLOCK_SIZE][2], int robot_id);



#endif
