#include "cad.h"
#include "cad/debug.h"

struct smooth_line {
	struct component l1;
	struct component l2;

	struct component corner;
	struct component corner_center;
	struct component end;
	struct component start;

	struct component perp1;
	struct component perp2;

	struct component corner_start;
	struct component corner_end;
};

struct smooth_line a_smooth_line() {
	return  (struct smooth_line){
		.l1 = {.type = COM_LINE},
		.l2 = {.type = COM_LINE},
		.corner = {.type = COM_POINT},
		.corner_center = {.type = COM_POINT},
		.end = {.type = COM_POINT},
		.start = {.type = COM_POINT},

		.perp1 = {.type = COM_LINE},
		.perp2 = {.type = COM_LINE},

		.corner_start = {.type = COM_POINT},
		.corner_end = {.type = COM_POINT},
	};
}

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

struct mid {
	struct component l1;
	struct component l2;

	struct component x1;

	struct component p1;

	struct component p;
};

struct mid a_midpoint() {
	return (struct mid) {
		.l1 = {.type = COM_LINE},
		.l2 = {.type = COM_LINE},

		.x1 = {.type = COM_POINT},

		.p1 = {.type = COM_LINE},

		.p = {.type = COM_POINT},
	};
}

void draw_from_constraints(struct constraints *c, struct topology *t, struct canvas *cv) {
	struct drawing drawing = {};
	struct subassembly assemblies[16] = {};
	size_t assembly_num;
	solve_constraints(c, &drawing, assemblies, &assembly_num);

	// Copy over all the parameter values to a new array
	// @PERF: Maybe we should just store them in a separate array to start with
	double *params = malloc(sizeof(double) * (c->length));
	for(size_t i = 0; i < c->length; i++) {
		params[i] = c->elements[i].v;
	}

	place_points(&drawing, params);
	free(params);

	print_subassemblies(c, assemblies, assembly_num);
	begin_drawing(cv);
	// for(struct command *current = drawing.root; current != NULL && current != drawing.error; current = current->next) {
	// 	if(current->hidden) continue;
	// 	plot_generic(current->result);
	// }

	// if(drawing.error != NULL) {
	// 	plot_generic(*drawing.error->arg1);
	// 	plot_generic(*drawing.error->arg2);
	// 	fprintf(stderr, "Solver error detected. Drawing will be incomplete\n");
	// }

	draw_topology(cv, t);
	draw_constraints(cv, c);
	end_drawing(cv);

	free_drawing(&drawing);
}

int main(int argc, char *argv[]) {
	struct constraints constraints = {};
	struct topology topo = {};

	struct component components[] = {
		{.type = COM_POINT},
		{.type = COM_POINT},
		{.type = COM_POINT},
		{.type = COM_LINE},
		{.type = COM_LINE},
		{.type = COM_POINT},
	};

	add_fragment(&topo, (struct topology_elem[]){
		TOPO_MOVETO(&components[1]),
		TOPO_LINETO(&components[5]),
		TOPO_LINETO(&components[0]),
		TOPO_LINETO(&components[2]),
		TOPO_LINETO(&components[1]),

		TOPO_END(),
	});

	add_constraint(&constraints, (struct constraint[]){
		PP_DISTANCE(&components[0], &components[1], 13),
		PP_DISTANCE(&components[1], &components[2], 7),
		PP_DISTANCE(&components[2], &components[0], 7),

		POINT_ON_LINE(&components[0], &components[3]),
		POINT_ON_LINE(&components[1], &components[3]),

		LL_ANGLE(&components[3], &components[4], M_PI/1.7),

		POINT_ON_LINE(&components[4], &components[1]),
		POINT_ON_LINE(&components[4], &components[5]),

		PP_DISTANCE(&components[0], &components[5], 12.8),

		CEND(),
	});

	struct smooth_line line = a_smooth_line();

	add_fragment(&topo, (struct topology_elem[]){
		TOPO_MOVETO(&line.start),
		TOPO_LINETO(&line.corner_start),
		TOPO_ARCTO(&line.corner_center, &line.corner_end),
		TOPO_LINETO(&line.end),

		TOPO_END(),
	});

	add_constraint(&constraints, (struct constraint[]){
		PP_SAME(&line.start, &components[5]),
		LL_ANGLE(&components[3], &line.l1, DEG(90)),

		POINT_ON_LINE(&line.start, &line.l1),

		LL_ANGLE(&line.l2, &line.l1, DEG(90)),
		POINT_ON_LINE(&line.corner, &line.l1),
		POINT_ON_LINE(&line.corner, &line.l2),

		POINT_ON_LINE(&line.end, &line.l2),
		PL_DISTANCE(&line.end, &line.l1, 10),
		PL_DISTANCE(&line.end, &components[4], 5),

		PL_DISTANCE(&line.corner_center, &line.l1, 2),
		PL_DISTANCE(&line.corner_center, &line.l2, -2),

		LL_ANGLE(&line.l1, &line.perp1, DEG(90)),
		POINT_ON_LINE(&line.corner_center, &line.perp1),
		POINT_ON_LINE(&line.corner_start, &line.perp1),
		POINT_ON_LINE(&line.corner_start, &line.l1),

		LL_ANGLE(&line.l2, &line.perp2, DEG(90)),
		POINT_ON_LINE(&line.corner_center, &line.perp2),
		POINT_ON_LINE(&line.corner_end, &line.perp2),
		POINT_ON_LINE(&line.corner_end, &line.l2),

		CEND(),
	});

	struct box box;
	a_box(&box, &topo, &constraints);

	add_constraint(&constraints, (struct constraint[]){
		PP_DISTANCE(&box.corner[0], &box.corner[1], 8),
		PP_DISTANCE(&box.corner[1], &box.corner[2], 4),

		PP_DISTANCE(&line.corner_end, &box.corner[0], 9.5),
		PL_DISTANCE(&box.corner[0], &components[3], 20),
		CEND(),
	});

	add_constraint(&constraints, (struct constraint[]){
		LL_ANGLE(&components[3], &box.side[1], DEG(90)),
		CEND(),
	});

	// struct mid box_enter = a_midpoint();

	// add_constraint(&constraints, (struct constraint[]){
	// 	LL_ANGLE(&box.side[3], &box_enter.l1, DEG(-30)),
	// 	POINT_ON_LINE(&box.corner[0], &box_enter.l1),
	// 	LL_ANGLE(&box.side[3], &box_enter.l2, DEG(30)),
	// 	POINT_ON_LINE(&box.corner[3], &box_enter.l2),

	// 	POINT_ON_LINE(&box_enter.l1, &box_enter.x1),
	// 	POINT_ON_LINE(&box_enter.l2, &box_enter.x1),

	// 	LL_ANGLE(&box.side[3], &box_enter.p1, DEG(90)),

	// 	POINT_ON_LINE(&box_enter.x1, &box_enter.p1),
	// 	POINT_ON_LINE(&box_enter.p, &box.side[3]),
	// 	POINT_ON_LINE(&box_enter.p1, &box_enter.p),

	// 	PP_DISTANCE(&box_enter.p, &box.corner[0], 5),
	// 	CEND(),
	// });

	struct canvas canvas = {
		.state = CANVAS_INIT,
		.f = stdout,
	};

	draw_from_constraints(&constraints, &topo, &canvas);

	free_constraints(&constraints);
	free_topology(&topo);
}
