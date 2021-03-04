#ifndef _ENVIRONMENT_H
#define _ENVIRONMENT_H

#define PHER_KINDS 1   // Change to 2 for psrt 3.5

#define NEST 1
#define OPEN 2
#define WALL 3
#define FOOD 4

#define MAX_HEIGHT 100
#define MAX_WIDTH  100

typedef struct {
	int type;
	double pher_qty[PHER_KINDS];
} cell_t;

typedef struct {
	cell_t grid[MAX_HEIGHT][MAX_WIDTH];

	int nest_x;
	int nest_y;

	int width;
	int height;
} environment_t;

int environment_load(environment_t * env, unsigned int id);

// Loads a similar but modified environment. Doesn't change existing pheromone levels.
int environment_load_modified(environment_t * env, unsigned int id);

void environment_reset(environment_t * env);

int environment_get_neighborhood(environment_t * env, int x, int y, cell_t cells[3][3]);

void environment_evaporate(environment_t * env, double *evap);

#endif
