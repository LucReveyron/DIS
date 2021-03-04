#ifndef _SPECIES_H
#define _SPECIES_H

#include "ant.h"
#include "environment.h"

#define ANT_NUM   1000

typedef struct {
	ant_t ants[ANT_NUM];

	double pher_drop; // size of a single pheromone drop
	double evap[PHER_KINDS]; // evaporation rate coefficient 
	double wander; // wandering probability
	double wght; // the power of a pheromone trail, i.e. pow(pheromone, wght), defines how distinct different values of deposited pheromone are, this is the alpha, the betta here is 0
	double supply[PHER_KINDS]; // the pheromone capacity
} species_t;

int species_init(species_t * species, unsigned int id);
void species_reset(species_t * species, environment_t * env);
void species_activate(species_t * species, environment_t * env);

#endif
