#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "matlab.h"
#include "antsim.h"

// prints pheromone concentrations in the environment
void matlab_header()
{
	int i;

	printf("\n");
	printf("%% Initialize variables\n");
	for (i = 0; i < PHER_KINDS; i++)
		printf("pher%d={};\n", i);
	printf("numants={};\n");
	printf("numactive=zeros(1, %d);\n", ITERS);
	printf("total_food_picked_up=zeros(1, runs);\n");
	printf("total_food_returned=zeros(1, runs);\n");
}

// prints pheromone concentrations in the environment
void matlab_pheromone(environment_t * env, int kind, int tstep)
{
	int i, j;

	printf("\n");
	printf("%% Pheromone concentrations at time step %d\n", tstep);
	printf("pher%d{%d}=[\n", kind, tstep);

	for (i = 0; i < env->height; i++) {
		for (j = 0; j < env->width; j++) {
			printf("%.4f, ", env->grid[i][j].pher_qty[kind]);
		}
		printf("\n");
	}
	printf("];\n");
}

// prints location of ants in the environment
void matlab_numants(environment_t * env, species_t * species, int tstep)
{
	int i, j;
	int numactive = 0;
	int size_y = env->height;
	int size_x = env->width;
	int numants[size_y][size_x];

	for (i = 0; i < size_y; i++)
		for (j = 0; j < size_x; j++)
			numants[i][j] = 0;

	for (i = 0; i < ANT_NUM; i++) {
		numants[species->ants[i].y][species->ants[i].x]++;
		if (species->ants[i].active) {
			numactive++;
		}
	}

	printf("\n");
	printf("%% Number of ants at time step %d\n", tstep);
	printf("numactive(%d)=%d;\n", tstep, numactive);
	printf("numants{%d}=[\n", tstep);

	for (i = 0; i < size_y; i++) {
		for (j = 0; j < size_x; j++) {
			printf("%d, ", numants[i][j]);
		}
		printf("\n");
	}
	printf("];\n");
}

// Prints the matlab script that executes the simulation
void matlab_footer(int runs)
{
	printf("\n");
	printf("%% Display main parameters\n");
	printf("disp(['Environment: ' num2str(environment)]);\n");
	printf("disp(['Species: ' num2str(species)]);\n");
	printf("disp(['Food picked up (avg over ' num2str(runs) ' runs): ' num2str(mean(total_food_picked_up))]);\n");
	printf("disp(['Food returned to the nest (avg over ' num2str(runs) ' runs): ' num2str(mean(total_food_returned))]);\n");

	if (runs == 1) {
		printf("disp('Type antsim_play or antsim_stepbystep to visualize the results.');\n");
	} else {
		printf("disp('Type antsim_histogram to visualize the results.');\n");
	}
}
