#include "cad.h"

struct mid {
	struct component p1;
	struct component p2;

	struct component lb;

	struct component l1;
	struct component l2;

	struct component x1;

	struct component p;

	struct component m;
};

void a_midpoint(struct mid *mid, struct topology *topo, struct constraints *constr) {
	*mid = (struct mid){
		.p1 = {.type = COM_POINT},
		.p2 = {.type = COM_POINT},
		.lb = {.type = COM_LINE},

		.l1 = {.type = COM_LINE},
		.l2 = {.type = COM_LINE},

		.x1 = {.type = COM_POINT},

		.p = {.type = COM_LINE},

		.m = {.type = COM_POINT},
	};

	if(constr != NULL) {
		add_constraint(constr, (struct constraint[]){
			POINT_ON_LINE(&mid->p1, &mid->lb),
			POINT_ON_LINE(&mid->p2, &mid->lb),

			LL_ANGLE(&mid->lb, &mid->l1, DEG(-30)),
			POINT_ON_LINE(&mid->p1, &mid->l1),
			LL_ANGLE(&mid->lb, &mid->l2, DEG(30)),
			POINT_ON_LINE(&mid->p2, &mid->l2),

			POINT_ON_LINE(&mid->x1, &mid->l1),
			POINT_ON_LINE(&mid->x1, &mid->l2),

			LL_ANGLE(&mid->lb, &mid->p, DEG(90)),
			POINT_ON_LINE(&mid->x1, &mid->p),

			POINT_ON_LINE(&mid->m, &mid->lb),
			POINT_ON_LINE(&mid->m, &mid->p),

			CEND(),
		});
	}
}

struct box {
	struct component bottom_left;
	struct component bottom_right;
	struct component top_right;
	struct component top_left;

	struct component bottom;
	struct component right;
	struct component top;
	struct component left;

	struct mid left_mid;
	struct mid right_mid;
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

	a_midpoint(&box->left_mid, topo, constr);
	a_midpoint(&box->right_mid, topo, constr);

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

			PP_DISTANCE(&box->bottom_left, &box->bottom_right, 7),
			PP_DISTANCE(&box->bottom_right, &box->top_right, 3),

			// Bind the mids
			PP_SAME(&box->left_mid.p1, &box->top_left),
			PP_SAME(&box->left_mid.p2, &box->bottom_left),

			PP_SAME(&box->right_mid.p1, &box->bottom_right),
			PP_SAME(&box->right_mid.p2, &box->top_right),
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
		POINT_ON_LINE(&box2.bottom_left, &box1.bottom),
		PP_DISTANCE(&box1.bottom_right, &box2.bottom_left, 3),
		LL_ANGLE(&box2.bottom, &box1.bottom, 0),

		POINT_ON_LINE(&box3.bottom_left, &box1.bottom),
		PP_DISTANCE(&box2.bottom_right, &box3.bottom_left, 3),
		LL_ANGLE(&box3.bottom, &box2.bottom, 0),
		CEND(),
	});

	add_fragment(&topo, (struct topology_elem[]){
		TOPO_MOVETO(&box1.right_mid.m),
		TOPO_LINETO(&box2.left_mid.m),

		TOPO_MOVETO(&box2.right_mid.m),
		TOPO_LINETO(&box3.left_mid.m),

		TOPO_END(),
	});

	struct canvas canvas = {
		.state = CANVAS_INIT,
		.f = stdout,
	};

	draw_from_constraints(&constraints, &topo, &canvas);

	free_constraints(&constraints);
	free_topology(&topo);
}
