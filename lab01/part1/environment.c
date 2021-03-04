#include <stdio.h>
#include "environment.h"

int environment_load(environment_t * env, unsigned int id)
{
	FILE *fp = NULL;
	char filename[256];
	int size_x, size_y;
	int nest_found = 0;
	int i, j, l;

	snprintf(filename, sizeof(filename) - 1, "env%d.map", id);
	fp = fopen(filename, "r");

	if (!fp) {
		fprintf(stderr, "Environment file %d not found.\n", id);
		return -1;
	}

	if(!fscanf(fp, "%d %d\n", &size_y, &size_x)){
		fprintf(stderr, "Error while reading environment file %d.\n", id);
		return -1;
	}
	if ((size_x < 1) || (size_x > 99)) {
		fprintf(stderr, "X-dimension doesn't make sense.\n");
		return -1;
	}
	if ((size_y < 1) || (size_y > 99)) {
		fprintf(stderr, "Y-dimension doesn't make sense.\n");
		return -1;
	}
	env->width = size_x;
	env->height = size_y;

	for (i = 0; i < size_y; i++) {
		for (j = 0; j < size_x; j++) {
			env->grid[i][j].type = fgetc(fp);

			// convert ascii to decimal
			env->grid[i][j].type -= 48;

			for (l = 0; l < PHER_KINDS; l++)
				env->grid[i][j].pher_qty[l] = 0;
		}

		// chew up the newline
		fgetc(fp);
	}

	// locate home, error if not found
	for (i = 0; i < size_y; i++)
		for (j = 0; j < size_x; j++)
			if (env->grid[i][j].type == NEST) {
				nest_found = 1;
				env->nest_y = i;
				env->nest_x = j;
			}
	if (!nest_found) {
		fprintf(stderr, "Home not found in environment.\n");
		return -1;
	}

	/* header of matlab file */
	printf("%% Environment parameters (don't change them here! run the simulation again with other parameters!)\n");
	printf("environment=%d;\n", id);

	return 0;
}

int environment_load_modified(environment_t * env, unsigned int id)
{
	FILE *fp = NULL;
	char filename[256];
	int size_x, size_y;
	int nest_found = 0;
	int i, j, l;

	snprintf(filename, sizeof(filename) - 1, "env%d_mod.map", id);
	fp = fopen(filename, "r");

	if (!fp) {
		fprintf(stderr, "Environment file %d_mod not found.\n", id);
		return -1;
	}

	if(!fscanf(fp, "%d %d\n", &size_y, &size_x)){
		fprintf(stderr, "Error while reading environment file %d.\n", id);
		return -1;
	}
	if ((size_x < 1) || (size_x > 99)) {
		fprintf(stderr, "X-dimension doesn't make sense.\n");
		return -1;
	}
	if ((size_y < 1) || (size_y > 99)) {
		fprintf(stderr, "Y-dimension doesn't make sense.\n");
		return -1;
	}
	
	if (env->width != size_x || env->height != size_y) {
		fprintf(stderr, "Loading modified environment failed. The mod and original maps are not similar.\n");
		return -1;
	}
	

	for (i = 0; i < size_y; i++) {
		for (j = 0; j < size_x; j++) {
			env->grid[i][j].type = fgetc(fp);

			// convert ascii to decimal
			env->grid[i][j].type -= 48;
		}
		// chew up the newline
		fgetc(fp);
	}

	// locate home, error if not found
	for (i = 0; i < size_y; i++)
		for (j = 0; j < size_x; j++)
			if (env->grid[i][j].type == NEST) {
				nest_found = 1;
				if (env->nest_y != i || env->nest_x != j) {
					fprintf(stderr, "Loading modified environment failed. They have different home positions.\n");
					return -1;
				}
			}
	if (!nest_found) {
		fprintf(stderr, "Loading modified environment failed. Home not found in environment.\n");
		return -1;
	}
	return 0;
}

void environment_reset(environment_t * env)
{
	int i, j, k;

	for (i = 0; i < env->height; i++)
		for (j = 0; j < env->width; j++)
			for (k = 0; k < PHER_KINDS; k++)
				env->grid[i][j].pher_qty[k] = 0.0;
}

int environment_get_neighborhood(environment_t * env, int x, int y, cell_t cells[3][3])
{
	int i, j;

	if (x < 1 || x >= env->width - 1)
		return -1;
	if (y < 1 || y >= env->height - 1)
		return -1;

	for (i = -1; i < 2; i++) {
		for (j = -1; j < 2; j++) {
			cells[i + 1][j + 1] = env->grid[i + y][j + x];
		}
	}

	return 0;
}

void environment_evaporate(environment_t * env, double *evap)
{
	int i, j, k;

	for (i = 0; i < env->height; i++)
		for (j = 0; j < env->width; j++)
			for (k = 0; k < PHER_KINDS; k++)
				env->grid[i][j].pher_qty[k] *= evap[k];
}
