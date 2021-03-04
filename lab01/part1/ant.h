#ifndef _ANT_H
#define _ANT_H

#include "environment.h"

typedef struct {
	int x; // current positon coordinate x
	int y; // current position coordinate y
	int px; // previous position coordinate x
	int py; // previous position coordinate y

	int food; // food caried by the ant
	double supply[PHER_KINDS]; // pheromone supply
	int active; // if this ant is wandering in the environment
} ant_t;

void ant_out(ant_t * ant, environment_t * env, double *supply);
void ant_in(ant_t * ant, environment_t * env);
void ant_move(ant_t * ant, environment_t * env, int dx, int dy);

float rnd(void);

#endif
