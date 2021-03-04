#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include "antsim.h"
#include "ant.h"
#include "matlab.h"


static environment_t env;
static species_t species;

static void ant_strategy(ant_t * ant, cell_t n[3][3], int *dx, int *dy, double *pher);

int main(int argc, char **args)
{
	unsigned int species_id, environment_id, runs;
	unsigned int run, iteration;
	unsigned int total;
	
	unsigned int retfood, getfood;
	int i, j;
	int dx, dy;
	double pher[PHER_KINDS];

	if (argc != 4) {
		fprintf(stderr, "Usage: %s [environment] [species] [runs]\n", args[0]);
		exit(-1);
	}
	/* You can add your own command-line arguments after these 3 
		(hint: to convert from string to double, use "double atof(char*)" */
	environment_id = atoi(args[1]);
	species_id = atoi(args[2]);
	runs = atoi(args[3]);

	/* initialize random number generator */
	srand(time(NULL));

	/* initialize species */
    if (species_init(&species, species_id))
        return 1;

	/* initialize environment */
	if (environment_load(&env, environment_id))
		return 1;

	printf("%% Parameters that were used in the simulation (don't change them here! run the simulation again with other parameters!)\n");
	printf("runs=%d;\n", runs);
	total = 0;
	matlab_header();

	/* Start main simulation */
	for (run = 0; run < runs; run++) {
		retfood = 0;
		getfood = 0;
		
		/* For Part 3.6, reset to the original environment here WITHOUT affecting pheromone levels. */
		
		/* reset pheromones */
		environment_reset(&env);
		/* reset ants */
		species_reset(&species, &env);

		for (iteration = 0; iteration < ITERS; iteration++) {
	
			cell_t neighborhood[3][3];
			//fprintf(stderr, "-------- iteration %d --------\n", iteration);

			/* activate ants */
			species_activate(&species, &env);

			/* move each active ant into the environment */
			for (i = 0; i < ANT_NUM; i++) {
				if (species.ants[i].active) {
					dx = 0;
					dy = 0;
					/* initialize pheromone adjustment */
					for (j = 0; j < PHER_KINDS; j++)
						pher[j] = 0.0;

					/* get ant neighborhood */
					environment_get_neighborhood(&env, species.ants[i].x, species.ants[i].y, neighborhood);

					/* where should the ant move ? (use separate function for user-defined ants) */
                    			ant_strategy(&(species.ants[i]), neighborhood, &dx, &dy, pher);

					/* move ant */
					ant_move(&(species.ants[i]), &env, dx, dy);

					/* update pheromones */
					for (j = 0; j < PHER_KINDS; j++) {
						if (species.ants[i].supply[j] > 0) {
							species.ants[i].supply[j] -= pher[j];
							env.grid[species.ants[i].y][species.ants[i].x].pher_qty[j] += pher[j];
							if (env.grid[species.ants[i].y][species.ants[i].x].pher_qty[j] > 1.0)
								env.grid[species.ants[i].y][species.ants[i].x].pher_qty[j] = 1.0;
						}
					}

					/* move on nest with food? */
					if (species.ants[i].x == env.nest_x && species.ants[i].y == env.nest_y) {
						if (species.ants[i].food)
							retfood++;
						ant_in(&species.ants[i], &env);
					}

					/* move on food without food? */
					if (env.grid[species.ants[i].y][species.ants[i].x].type == FOOD) {
						if (!species.ants[i].food) {
							species.ants[i].food = 1;
							getfood++;
						}

						/* go back to nest now */
						species.ants[i].px = species.ants[i].x;
						species.ants[i].py = species.ants[i].y;
					}
				}
			}

			/* evaporate pheromones */
			environment_evaporate(&env, species.evap);

			/* add boards to the matlab file */
			if (runs == 1) {
				matlab_pheromone(&env, 0, iteration + 1);
				matlab_numants(&env, &species, iteration + 1);
			}
			
			/* Load modified environment here for Part 3.6 at an appropriate time
			 * WITHOUT affecting pheromone levels. 
			 Make sure to report error / stop simulation if loading failed. */

		}

		/* add performance to the matlab data */
		printf("\n");
		printf("total_food_picked_up(%d)=%d;\n", run + 1, getfood);
		printf("total_food_returned(%d)=%d;\n", run + 1, retfood);
		fprintf(stderr, "Run %d complete, total food acquired: %d.\n", run + 1, retfood);
		total += retfood;
	}

	fprintf(stderr, "Average food acquired: %d\n", total / runs);
	matlab_footer(runs);
	return 0;
}

/* This function implements the following and trail laying behovior
 * of a single ant. THIS IS THE FUNCTION YOU WILL HAVE TO MODIFY.
 *
 * ant is the current ant (see ant.h)
 * n is the direct neighborhood of the ant (see environment.h)
 * +---+---+---+
 * |   |   |   | The neighborhood is index like [row][col] <-> [y][x]
 * +---+---+---+
 * |   | A |   | The ant is located at (1,1) in the neighborhood
 * +---+---+---+
 * |   |   |   |
 * +---+---+---+
 * 
 * [dx, dy] is the direction to which the ant should move
 * pher is the amount of pheromone the ant should leave on its next location
 */
void ant_strategy(ant_t * ant, cell_t n[3][3], int *dx, int *dy, double *pher)
{
	int i, j;
	double pher_total = 0.0;
	int open_cells = 0;

	/* previous neighborhood cell */
	int pnj = ant->px - ant->x + 1;
	int pni = ant->py - ant->y + 1;

	/* always deposit the same amount of pheromones */
	//pher[ant->food] = species.pher_drop; // Uncomment this line for part 3.5
	pher[0] = species.pher_drop; // Comment this line for part 3.5

	/* get total amount of pheromone or move to nest or food source if near */
	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			/* Ignore current cell */
			if (i == 1 && j == 1)
				continue;

			/* Ignore walls */
			if (n[i][j].type == WALL)
				continue;

			/* Count the number of open cells - we need that later */
			open_cells++;

			/* Ignore the cell where we came from */
			if (i == pni && j == pnj)
				continue;

			/* Near food source? */
			if (n[i][j].type == FOOD && !ant->food) {
				*dx = j - 1;
				*dy = i - 1;
				return;
			}

			/* Near nest? */
			if (n[i][j].type == NEST && ant->food) {
				*dx = j - 1;
				*dy = i - 1;
				return;
			}

			/* Sum the total pheromone weight - we need that later */
			//pher_total += pow(n[i][j].pher_qty[!ant->food], species.wght); // Uncomment this line for part 3.5
			pher_total += pow(n[i][j].pher_qty[0], species.wght); // Comment this line for part 3.5
		}
	}

	/* Should I wander or follow the pheromone? */
	if (pher_total == 0.0 || rnd() < species.wander) {
		/* Wander, i.e. draw any open cell at random */
		double current_wander = 0.0;
		double wander = rnd();
		for (i = 0; i < 3; i++) {
			for (j = 0; j < 3; j++) {
				/* Ignore current cell */
				if (i == 1 && j == 1)
					continue;

				/* Ignore walls */
				if (n[i][j].type == WALL)
					continue;

				/* This implements a probabilistic choice between intervals */
				current_wander += 1.0 / (double) open_cells;
				if (wander <= current_wander) {
					*dx = j - 1;
					*dy = i - 1;
					return;
				}
			}
		}
	} else {
		/* Follow the pheromone, i.e. draw a cell according to its amount of pheromones */
		double current_pher = 0.0;
		double rand_pher = rnd() * pher_total;
		for (i = 0; i < 3; i++) {
			for (j = 0; j < 3; j++) {
				/* Ignore current cell */
				if (i == 1 && j == 1)
					continue;
	
				/* Ignore the cell where we came from */
				if (i == pni && j == pnj)
					continue;
	
				/* Ignore walls */
				if (n[i][j].type == WALL)
					continue;
	
				/* This implements a probabistic choice between intervals */
				//current_pher += pow(n[i][j].pher_qty[!ant->food], species.wght); // Uncomment this line for part 3.5
				current_pher += pow(n[i][j].pher_qty[0], species.wght); // Comment this line for part 3.5
				if  ((rand_pher <= current_pher) || (i==2&&j==2)) {
					*dx = j - 1;
					*dy = i - 1;
					return;
				}

			}
		}
	}

	/* Should never be reached */
	//fprintf(stderr, "Something went wrong here ... this statement should never be reached!\n");
	*dx = 0;
	*dy = 0;
	return;
}
