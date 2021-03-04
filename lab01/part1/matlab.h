#ifndef _MATLAB_H
#define _MATLAB_H

#include "environment.h"
#include "species.h"

void matlab_header();
void matlab_pheromone(environment_t * env, int kind, int tstep);
void matlab_numants(environment_t * env, species_t * species, int tstep);
void matlab_footer(int runs);

#endif
