#include <stdio.h>
#include "species.h"
#include "antsim.h"

#define SPECIES_NUM 4

/* species */
static double pher_drops[SPECIES_NUM] = { .3, .3, .3, .0045 };
static double wanders[SPECIES_NUM] = { .15, .15, 0.0, .006 };
static double wghts[SPECIES_NUM] = { 35.0, 0.0, 35.0, 2.0 };

static double evaps[PHER_KINDS][SPECIES_NUM] =
    {
        { .4, .4, .4, .82 } /* for pheromone type 0 */
        /*,{ .4, .4, .4, .82 } *//* for pheromone type 1 (unused) */ // Uncomment this line for part 3.5
    };

static double supplies[PHER_KINDS][SPECIES_NUM] =
    {
        { 28.0, 28.0, 28.0, .3 } /* for pheromone type 0 */
        /*,{ 28.0, 28.0, 28.0, .3 } *//* for pheromone type 1 (unused) */ // Uncomment this line for part 3.5
    };

int species_init(species_t * species, unsigned int id)
{
	int i;
	if (id > SPECIES_NUM || id == 0)
    	{
		fprintf(stderr, "Species not found.");
		return -1;
	}

	species->pher_drop = pher_drops[id - 1];
	species->wander = wanders[id - 1];
	species->wght = wghts[id - 1];

    
	for (i = 0; i < PHER_KINDS; ++i)
    	{
        	species->evap[i] = evaps[i][id - 1];
        	species->supply[i] = supplies[i][id - 1];
    	}

	/* header of matlab file */
	printf("%% Species parameters (don't change them here! run the simulation again with other parameters!)\n");
	printf("species=%d;\n", id);
	printf("pher_drop=%f;\n", species->pher_drop);
	printf("evap=%f;\n", species->evap[0]);
	printf("wander=%f;\n", species->wander);
	printf("wght=%f;\n", species->wght);
	printf("supply=%f;\n", species->supply[0]);

	return 0;
}

void species_reset(species_t * species, environment_t * env)
{
	int i, j;

	for (i = 0; i < ANT_NUM; i++)
    	{
		species->ants[i].active = 0;
		species->ants[i].food = 0;
		species->ants[i].x = species->ants[i].px = env->nest_x;
		species->ants[i].y = species->ants[i].py = env->nest_y;
        	for (j = 0; j < PHER_KINDS; ++j)
        	    species->ants[i].supply[j] = species->supply[j];
	}
}

void species_activate(species_t * species, environment_t * env)
{
	int i;

	for (i = 0; i < ANT_NUM; i++)
    	{
		if (ANT_PROB > rnd())
        	{
			if (!species->ants[i].active)
				ant_out(&species->ants[i], env, species->supply);
		}
	}
}
