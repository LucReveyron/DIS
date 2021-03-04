#include <stdlib.h>
#include "ant.h"


void ant_out(ant_t * ant, environment_t * env, double *supply)
{
	int i;

	ant->x = ant->px = env->nest_x;
	ant->y = ant->py = env->nest_y;

	ant->food = 0;
	for (i = 0; i < PHER_KINDS; i++)
		ant->supply[i] = supply[i];
	ant->active = 1;
}

void ant_in(ant_t * ant, environment_t * env)
{
	ant->x = ant->px = env->nest_x;
	ant->y = ant->py = env->nest_y;
	ant->food = 0;
	ant->active = 0;
}

void ant_move(ant_t * ant, environment_t * env, int dx, int dy)
{
	int x = ant->x + dx;
	int y = ant->y + dy;

	if (dx > 1 || dx < -1)
		return;
	if (dy > 1 || dy < -1)
		return;

	if (x < 0 || x >= env->width)
		return;
	if (y < 0 || y >= env->height)
		return;
	if (env->grid[y][x].type == WALL)
		return;

	ant->px = ant->x;
	ant->py = ant->y;

	ant->x = x;
	ant->y = y;
}

float rnd(void)
{
	return (float) rand() / RAND_MAX;
}
