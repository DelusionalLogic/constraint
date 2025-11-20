#pragma once

#include "cad/construction.h"

#define SEARCH_DEPTH 16

enum component_type {
	COM_POINT,
	COM_LINE,
};

struct component {
	enum component_type type;

	struct element *e;

	bool show_when_placed;

	double min;
	double max;
	bool min_max_init;
	bool drawn;
};

struct alias {
	struct component *alias;
	struct component *target;
};

struct constraints {
	struct constraint *elements;
	size_t length;
	size_t capacity;

	struct alias aliases[16];
	size_t aliases_num;
};

extern char *constraint_type_name[];
enum constraint_type {
	CT_POINT_POINT_DISTANCE,
	CT_POINT_LINE_DISTANCE,
	CT_LINE_LINE_ANGLE,

	CT_END,
};

struct path_step {
	uint64_t i;
	bool direction;
};

struct constraint {
	enum constraint_type type;
	double v;

	struct component *c1;
	struct component *c2;

	uint64_t order;
	struct path_step path[SEARCH_DEPTH];
	bool forward;
	bool used;
};

void alias_point(struct constraints *c, struct component *alias, struct component *target);
void add_constraint(struct constraints *c, struct constraint *new);
bool solve_constraints(struct constraints *constraints, struct drawing *drawing);
