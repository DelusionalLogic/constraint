#include "cad.h"

struct box {
	struct component bottom_left;
	struct component bottom_right;
	struct component top_right;
	struct component top_left;

	struct component bottom;
	struct component right;
	struct component top;
	struct component left;
};

void a_box(struct box *box, struct topology *topo, struct constraints *constr) {
	assert(box != NULL);

	*box = (struct box){
		.bottom_left = {.type = COM_POINT},
		.bottom_right = {.type = COM_POINT},
		.top_right = {.type = COM_POINT},
		.top_left = {.type = COM_POINT},

		.bottom = {.type = COM_LINE},
		.right = {.type = COM_LINE},
		.top = {.type = COM_LINE},
		.left = {.type = COM_LINE},
	};

	if(topo != NULL) {
		add_fragment(topo, (struct topology_elem[]){
			TOPO_MOVETO(&box->bottom_left),
			TOPO_LINETO(&box->bottom_right),
			TOPO_LINETO(&box->top_right),
			TOPO_LINETO(&box->top_left),
			TOPO_LINETO(&box->bottom_left),

			TOPO_END(),
		});
	}

	if(constr != NULL) {
		add_constraint(constr, (struct constraint[]){
			POINT_ON_LINE(&box->bottom_left, &box->bottom),
			POINT_ON_LINE(&box->bottom_right, &box->bottom),

			POINT_ON_LINE(&box->bottom_right, &box->right),
			POINT_ON_LINE(&box->top_right, &box->right),

			POINT_ON_LINE(&box->top_right, &box->top),
			POINT_ON_LINE(&box->top_left, &box->top),

			POINT_ON_LINE(&box->bottom_left, &box->left),
			POINT_ON_LINE(&box->top_left, &box->left),

			LL_ANGLE(&box->left, &box->bottom, DEG(90)),
			LL_ANGLE(&box->right, &box->top, DEG(90)),
			LL_ANGLE(&box->top, &box->left, DEG(90)),
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

	free_drawing(&drawing);
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
		PP_DISTANCE(&box1.bottom_left, &box1.bottom_right, 10),
		PP_DISTANCE(&box1.bottom_right, &box1.top_right, 10),

		PP_SAME(&box2.bottom_left, &box1.top_left),
		PP_DISTANCE(&box2.bottom_left, &box2.bottom_right, 10),
		PP_DISTANCE(&box2.bottom_right, &box2.top_right, 10),
		LL_ANGLE(&box2.bottom, &box1.bottom, DEG(-60)),

		PP_SAME(&box3.bottom_left, &box2.top_left),
		PP_DISTANCE(&box3.bottom_right, &box3.top_right, 10),
		PP_DISTANCE(&box3.bottom_left, &box3.bottom_right, 7),
		LL_ANGLE(&box3.bottom, &box2.bottom, DEG(-20)),
		CEND(),
	});

	struct canvas canvas = {
		.state = CANVAS_INIT,
		.f = stdout,
	};

	draw_from_constraints(&constraints, &topo, &canvas);

	free_constraints(&constraints);
	free_topology(&topo);
}
