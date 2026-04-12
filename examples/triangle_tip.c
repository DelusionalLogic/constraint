#include "cad.h"
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
	struct component p5 = {.type = COM_POINT};

	struct component t1base = {.type = COM_LINE};
	struct component t1bas2 = {.type = COM_LINE};
	struct component t2base = {.type = COM_LINE};

	add_constraint(&constraints, (struct constraint[]){
		// Make a triangle
		PP_DISTANCE(&p1, &p2, 30),
		PP_DISTANCE(&p3, &p1, 30),
		PP_DISTANCE(&p2, &p3, 30),

		POINT_ON_LINE(&p1, &t1base),
		POINT_ON_LINE(&p2, &t1base),

		POINT_ON_LINE(&p1, &t1bas2),
		LL_ANGLE(&t1base, &t1bas2, DEG(-10)),

		// With another triangle sharing a point
		PP_DISTANCE(&p4, &p5, 30),
		PP_DISTANCE(&p3, &p4, 30),
		PP_DISTANCE(&p5, &p3, 30),

		POINT_ON_LINE(&p4, &t2base),
		POINT_ON_LINE(&p5, &t2base),

		// The edge they don't share is constrained
		LL_ANGLE(&t1bas2, &t2base, DEG(70)),
		CEND(),
	});

	add_fragment(&topo, (struct topology_elem[]){
		TOPO_MOVETO(&p1),
		TOPO_LINETO(&p2),
		TOPO_LINETO(&p3),
		TOPO_LINETO(&p1),

		TOPO_MOVETO(&p3),
		TOPO_LINETO(&p4),
		TOPO_LINETO(&p5),
		TOPO_LINETO(&p3),

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
