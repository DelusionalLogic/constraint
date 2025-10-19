#pragma once

#include "cad/construction.h"
#include <stddef.h>

struct constraint;
struct drawing;
struct component;

struct constraints;

void alloc_constraints(struct constraints *system);

void constrain_distance_between_points(struct constraints *system, struct component *p1, struct component *p2, double distance);
void constrain_distance_to_line(struct constraints *system, struct component *point, struct component *line, double distance);
void constrain_angle(struct constraints *system, struct component *l1, struct component *l2, double theta);

void solve_constraints(struct constraint *constraints, size_t constraints_num, struct drawing *drawing);
