#include "cad.h"

void draw_from_constraints(struct constraints *c, struct topology *t, struct canvas *cv) {
	struct drawing drawing = {};
	struct subassembly assemblies[16] = {};
	size_t assemblies_num;
	bool rc = solve_constraints(c, &drawing, assemblies, &assemblies_num);
	assert(rc);

	// Copy over all the parameter values to a new array
	// @PERF: Maybe we should just store them in a separate array to start with
	double *params = malloc(sizeof(double) * (c->length));
	for(size_t i = 0; i < c->length; i++) {
		params[i] = c->elements[i].v;
	}

	place_points(&drawing, params);
	free(params);

	reconstruct_drawing(c, assemblies, &assemblies_num);

	begin_drawing(cv);
	draw_topology(cv, t);
	draw_constraints(cv, c);
	end_drawing(cv);

	free_drawing(&drawing);
}

int main(int argc, char *argv[]) {
	struct constraints constraints = {};
	struct topology topo = {};

	struct component mid_bot = {.type = COM_POINT};
	struct component pseudo = {.type = COM_POINT};
	struct component bot = {.type = COM_LINE};

	struct component symmetry = {.type = COM_LINE};

	struct component right_lower = {.type = COM_LINE};
	struct component left_lower = {.type = COM_LINE};

	struct component right_under_hole_mid = {.type = COM_POINT};

	struct component right_hole_bot_tan = {.type = COM_LINE};
	struct component left_hole_bot_tan = {.type = COM_LINE};
	struct component right_hole_top_tan = {.type = COM_LINE};
	struct component left_hole_top_tan = {.type = COM_LINE};

	struct component right_hole_mid = {.type = COM_POINT};
	struct component left_hole_mid = {.type = COM_POINT};

	struct component swoop_mid = {.type = COM_POINT};
	struct component swoop_top_tan = {.type = COM_POINT};

	struct component left_bend = {.type = COM_POINT};
	struct component lb2 = {.type = COM_POINT};

	struct component l1 = {.type = COM_LINE};

	add_constraint(&constraints, (struct constraint[]){
		PP_DISTANCE(&mid_bot, &pseudo, 10),
		POINT_ON_LINE(&mid_bot, &bot),
		POINT_ON_LINE(&pseudo, &bot),

		LL_ANGLE(&bot, &symmetry, DEG(90)),
		POINT_ON_LINE(&mid_bot, &symmetry),

		// @HACK This is going in the wrong direction?
		PL_DISTANCE(&mid_bot, &right_lower, 35),
		LL_ANGLE(&bot, &right_lower, DEG(-90)),
		PL_DISTANCE(&left_lower, &mid_bot, 35),
		LL_ANGLE(&bot, &left_lower, DEG(90)),

		PL_DISTANCE(&right_hole_mid, &bot, 80),
		PL_DISTANCE(&left_hole_mid, &bot, 80),
		PL_DISTANCE(&right_hole_mid, &symmetry, -60),
		PL_DISTANCE(&left_hole_mid, &symmetry, 60),

		PL_DISTANCE(&right_under_hole_mid, &right_lower, 20),
		PL_DISTANCE(&right_under_hole_mid, &right_hole_bot_tan, 20),

		LL_ANGLE(&right_hole_bot_tan, &symmetry, DEG(270)),

		PL_DISTANCE(&right_hole_mid, &right_hole_bot_tan, -28),
		// PL_DISTANCE(&left_hole_mid, &left_hole_bot_tan, 56),
		PL_DISTANCE(&right_hole_mid, &right_hole_top_tan, 28),
		// PL_DISTANCE(&left_hole_mid, &left_hole_top_tan, 56),

		POINT_ON_LINE(&swoop_mid, &symmetry),
		PP_DISTANCE(&swoop_mid, &right_hole_mid, 120 - 28),
		// PL_DISTANCE(&swoop_mid, &swoop_top_tan, 120),
		PL_DISTANCE(&swoop_mid, &right_hole_top_tan, 120),


		// Make a triangle
		CEND(),
	});

	add_fragment(&topo, (struct topology_elem[]){
		TOPO_MOVETO(&mid_bot),
		TOPO_LINETO(&right_hole_mid),

		TOPO_MOVETO(&mid_bot),
		// TOPO_LINETO(&p2),
		// TOPO_LINETO(&p3),
		TOPO_END(),
	});

	struct canvas canvas = {
		.state = CANVAS_INIT,
		.f = stdout,
	};

	draw_from_constraints(&constraints, &topo, &canvas);

	free_constraints(&constraints);
	return 0;
}
