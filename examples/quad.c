#include "cad.h"
#include "cad/svg.h"
#include "cad/debug.h"

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
	print_subassemblies(c, assemblies, assemblies_num);

	begin_drawing(cv);
	draw_topology(cv, t);
	draw_constraints(cv, c);
	end_drawing(cv);

	free_drawing(&drawing);
}

int main(int argc, char *argv[]) {
	struct constraints constraints = {};
	struct topology topo = {};

	struct component p1 = {.type = COM_POINT};
	struct component p2 = {.type = COM_POINT};
	struct component p3 = {.type = COM_POINT};
	struct component p4 = {.type = COM_POINT};

	struct component l1 = {.type = COM_LINE};
	struct component l2 = {.type = COM_LINE};
	struct component l3 = {.type = COM_LINE};
	struct component l4 = {.type = COM_LINE};

	add_constraint(&constraints, (struct constraint[]){
		// Make a triangle
		PP_DISTANCE(&p1, &p2, 10),
		PP_DISTANCE(&p4, &p1, 20),

		POINT_ON_LINE(&p1, &l1),
		POINT_ON_LINE(&p2, &l1),

		POINT_ON_LINE(&p2, &l2),
		POINT_ON_LINE(&p3, &l2),

		POINT_ON_LINE(&p3, &l3),
		POINT_ON_LINE(&p4, &l3),

		POINT_ON_LINE(&p4, &l4),
		POINT_ON_LINE(&p1, &l4),

		LL_ANGLE(&l1, &l2, DEG(90)),
		LL_ANGLE(&l2, &l3, DEG(90)),
		LL_ANGLE(&l3, &l4, DEG(90)),

		CEND(),
	});

	add_fragment(&topo, (struct topology_elem[]){
		TOPO_MOVETO(&p1),
		TOPO_LINETO(&p2),
		TOPO_LINETO(&p3),
		TOPO_LINETO(&p4),
		TOPO_LINETO(&p1),

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
