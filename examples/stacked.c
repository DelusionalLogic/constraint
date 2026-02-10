#include "cad.h"

struct box {
	struct component corner[4];
	struct component side[4];
};

void a_box(struct box *box, struct topology *topo, struct constraints *constr) {
	assert(box != NULL);

	*box = (struct box){
		.corner = {
			{.type = COM_POINT},
			{.type = COM_POINT},
			{.type = COM_POINT},
			{.type = COM_POINT},
		},
		.side = {
			{.type = COM_LINE},
			{.type = COM_LINE},
			{.type = COM_LINE},
			{.type = COM_LINE},
		},
	};

	if(topo != NULL) {
		add_fragment(topo, (struct topology_elem[]){
			TOPO_MOVETO(&box->corner[0]),
			TOPO_LINETO(&box->corner[1]),
			TOPO_LINETO(&box->corner[2]),
			TOPO_LINETO(&box->corner[3]),
			TOPO_LINETO(&box->corner[0]),

			TOPO_END(),
		});
	}

	if(constr != NULL) {
		add_constraint(constr, (struct constraint[]){
			POINT_ON_LINE(&box->corner[0], &box->side[0]),
			POINT_ON_LINE(&box->corner[1], &box->side[0]),

			POINT_ON_LINE(&box->corner[1], &box->side[1]),
			POINT_ON_LINE(&box->corner[2], &box->side[1]),

			POINT_ON_LINE(&box->corner[2], &box->side[2]),
			POINT_ON_LINE(&box->corner[3], &box->side[2]),

			POINT_ON_LINE(&box->corner[0], &box->side[3]),
			POINT_ON_LINE(&box->corner[3], &box->side[3]),

			LL_ANGLE(&box->side[3], &box->side[0], DEG(90)),
			LL_ANGLE(&box->side[1], &box->side[2], DEG(90)),
			LL_ANGLE(&box->side[2], &box->side[3], DEG(90)),
			CEND(),
		});
	}
}

void draw_from_constraints(struct constraints *c, struct topology *t, struct canvas *cv) {
	struct drawing drawing = {};
	bool rc = solve_constraints(c, &drawing);
	assert(rc);

	// Copy over all the parameter values to a new array
	// @PERF: Maybe we should just store them in a separate array to start with
	double *params = malloc(sizeof(double) * (c->length));
	for(size_t i = 0; i < c->length; i++) {
		params[i] = c->elements[i].v;
	}

	place_points(&drawing, params);
	free(params);

	begin_drawing(cv);
	draw_topology(cv, t);
	draw_constraints(cv, c);
	end_drawing(cv);
}

int main(int argc, char *argv[]) {
	struct constraints constraints = {};
	struct topology topo = {};

	struct box box1;
	a_box(&box1, &topo, &constraints);

	struct box box2;
	a_box(&box2, &topo, &constraints);

	struct box box3;
	a_box(&box3, &topo, &constraints);

	add_constraint(&constraints, (struct constraint[]){
		PP_DISTANCE(&box1.corner[0], &box1.corner[1], 10),
		PP_DISTANCE(&box1.corner[1], &box1.corner[2], 10),

		PP_SAME(&box2.corner[0], &box1.corner[3]),
		PP_DISTANCE(&box2.corner[0], &box2.corner[1], 10),
		PP_DISTANCE(&box2.corner[1], &box2.corner[2], 10),
		LL_ANGLE(&box2.side[0], &box1.side[0], DEG(-60)),

		PP_SAME(&box3.corner[0], &box2.corner[3]),
		PP_DISTANCE(&box3.corner[1], &box3.corner[2], 10),
		PP_DISTANCE(&box3.corner[0], &box3.corner[1], 7),
		LL_ANGLE(&box3.side[0], &box2.side[0], DEG(-20)),
		CEND(),
	});

	struct canvas canvas = {
		.state = CANVAS_INIT,
		.f = stdout,
	};
	
	draw_from_constraints(&constraints, &topo, &canvas);
}
