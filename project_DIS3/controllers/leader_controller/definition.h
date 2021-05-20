#ifndef DEFINITION_H
#define DEFINITION_H

#define MODE				0			// Choose between flocking (1) or formation (0)
#define WORLD				1			// Choose between obstacle (1) or crossing  (2)

#define NB_SENSORS	  	  	8	  		// Number of distance sensors
#define MIN_SENS          	350     	// Minimum sensibility value
#define MAX_SENS          	4096    	// Maximum sensibility value
#define MAX_SPEED         	800     	// Maximum speed
/*Webots 2018b*/
#define MAX_SPEED_WEB      	6.28    	// Maximum speed webots
/*Webots 2018b*/
#define FLOCK_SIZE	  		5	  		// Size of flock
#define TIME_STEP	  		64	  		// [ms] Length of time step

#define AXLE_LENGTH 		0.052		// Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS		0.00628		// Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS		0.0205		// Wheel radius (meters)
#define DELTA_T				0.064		// Timestep (seconds)

#define NEIG_DIS       		0.4

#define RULE1_THRESHOLD     0.20   		// Threshold to activate aggregation rule. default 0.20
#define RULE1_WEIGHT        (0.6/10)	// Weight of aggregation rule. default 0.6/10

#define RULE2_THRESHOLD     0.15   		// Threshold to activate dispersion rule. default 0.15
#define RULE2_WEIGHT        (0.02/10)	// Weight of dispersion rule. default 0.02/10

#define RULE3_THRESHOLD     0.40
#define RULE3_WEIGHT        (1.0/10)	// Weight of consistency rule. default 1.0/10

#define MIGRATION_WEIGHT    (10)   		// Weight of attraction towards the common goal. default 0.01/10
#define FOLLOWER_WEIGHT     (1)	// Weight of attraction towards the goal for formation

#define KU_LEADER		0.002   // Forward control coefficient
#define KW_LEADER		0.125   // Rotational control coefficient
#define KU_FOLLOWER		0.01     // Forward control coefficient
#define KW_FOLLOWER		0.125   // Rotational control coefficient

#define ABS(x) ((x>=0)?(x):-(x))

#endif
