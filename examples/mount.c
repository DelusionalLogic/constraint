#include "cad.h"

static const char *component_type_name[] = {
	[COM_POINT] = "point",
	[COM_LINE] = "line",
};

static void print_subassemblies(struct constraints *c, struct subassembly *assemblies, size_t assemblies_num) {
	fprintf(stderr, "=== Subassemblies: %zu ===\n", assemblies_num);
	for(size_t i = 0; i < assemblies_num; i++) {
		struct subassembly *a = &assemblies[i];
		fprintf(stderr, "Assembly %zu: %zu steps, fix=%zu, fixed=%s\n",
			i, a->steps_num, a->fix, a->fixed ? "yes" : "no");

		if(a->articulation_num > 0) {
			fprintf(stderr, "  Articulations (%zu):\n", a->articulation_num);
			for(size_t j = 0; j < a->articulation_num; j++) {
				struct component *comp = a->articulation[j];
				fprintf(stderr, "    [%zu] %s %p\n", j, component_type_name[comp->type], (void*)comp);
			}
		}

		fprintf(stderr, "  Constraints used:\n");
		for(size_t j = 0; j < c->length; j++) {
			struct constraint *ct = &c->elements[j];
			if(ct->used != i + 1) continue;
			fprintf(stderr, "    [%zu] %s v=%.2f (%s %p, %s %p)\n",
				j, constraint_type_name[ct->type], ct->v,
				component_type_name[ct->c1->type], (void*)ct->c1,
				component_type_name[ct->c2->type], (void*)ct->c2);
		}
	}
	fprintf(stderr, "  Unused constraints:\n");
	for(size_t j = 0; j < c->length; j++) {
		struct constraint *ct = &c->elements[j];
		if(ct->used != 0) continue;
		fprintf(stderr, "    [%zu] %s v=%.2f (%s %p, %s %p)\n",
			j, constraint_type_name[ct->type], ct->v,
			component_type_name[ct->c1->type], (void*)ct->c1,
			component_type_name[ct->c2->type], (void*)ct->c2);
	}
	fprintf(stderr, "=========================\n");
}

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
	struct constraints constraints = {0};
	struct topology topo = {};

	struct component mid_bot = {.type = COM_POINT};

	struct component right_corner = {.type = COM_POINT};
	struct component below_right_hole_corner = {.type = COM_POINT};
	struct component right_hole_bot = {.type = COM_POINT};
	struct component right_hole_mid = {.type = COM_POINT};
	struct component right_hole_top = {.type = COM_POINT};

	struct component swoop_top = {.type = COM_POINT};
	struct component swoop_mid = {.type = COM_POINT};

	struct component left_hole_top = {.type = COM_POINT};
	struct component left_hole_mid = {.type = COM_POINT};
	struct component left_hole_bot = {.type = COM_POINT};
	struct component below_left_hole_corner = {.type = COM_POINT};
	struct component left_corner = {.type = COM_POINT};

	struct component symmetry = {.type = COM_LINE};
	struct component bottom = {.type = COM_LINE};

	struct component right_edge = {.type = COM_LINE};
	struct component right_bottom_hole_tan = {.type = COM_LINE};
	struct component left_edge = {.type = COM_LINE};

	struct component up_tan_axis = {.type = COM_LINE};

	struct component right_inner_hole_bisect = {.type = COM_LINE};
	struct component right_inner_hole_top = {.type = COM_POINT};
	struct component right_inner_hole_bot = {.type = COM_POINT};
	struct component left_inner_hole_bisect = {.type = COM_LINE};
	struct component left_inner_hole_top = {.type = COM_POINT};
	struct component left_inner_hole_bot = {.type = COM_POINT};


	struct component right_hole_line = {.type = COM_LINE};

	add_constraint(&constraints, (struct constraint[]){
		PP_DISTANCE(&mid_bot, &right_corner, 35),
		POINT_ON_LINE(&mid_bot, &bottom),
		POINT_ON_LINE(&right_corner, &bottom),

		LL_ANGLE(&bottom, &symmetry, DEG(90)),
		POINT_ON_LINE(&mid_bot, &symmetry),

		PL_DISTANCE(&right_hole_mid, &bottom, 80),
		PL_DISTANCE(&right_hole_mid, &symmetry, -60),

		LL_ANGLE(&bottom, &right_edge, DEG(90)),
		// LL_ANGLE(&bottom, &left_edge, DEG(-90)),

		POINT_ON_LINE(&right_corner, &right_edge),
		//POINT_ON_LINE(&left_corner, &left_edge),

		LL_ANGLE(&symmetry, &right_bottom_hole_tan, DEG(-90)),
		PL_DISTANCE(&right_hole_mid, &right_bottom_hole_tan, -28),

		POINT_ON_LINE(&below_right_hole_corner, &right_edge),
		POINT_ON_LINE(&below_right_hole_corner, &right_bottom_hole_tan),

		POINT_ON_LINE(&right_hole_bot, &right_bottom_hole_tan),
		PP_DISTANCE(&right_hole_mid, &right_hole_bot, 28),

		POINT_ON_LINE(&swoop_mid, &symmetry),

		PP_DISTANCE(&swoop_mid, &right_hole_top, 120),
		PP_DISTANCE(&right_hole_mid, &right_hole_top, 28),

		POINT_ON_LINE(&right_hole_mid, &right_hole_line),
		POINT_ON_LINE(&right_hole_top, &right_hole_line),
		POINT_ON_LINE(&swoop_mid, &right_hole_line),

		POINT_ON_LINE(&swoop_top, &symmetry),
		PP_DISTANCE(&swoop_top, &swoop_mid, 120),

		// LL_ANGLE(&right_inner_hole_bisect, &bottom, DEG(90)),
		// LL_ANGLE(&left_inner_hole_bisect, &bottom, DEG(-90)),
		// POINT_ON_LINE(&right_hole_mid, &right_inner_hole_bisect),
		// POINT_ON_LINE(&left_hole_mid, &left_inner_hole_bisect),
		// PP_DISTANCE(&right_hole_mid, &right_inner_hole_top, 14),
		// PP_DISTANCE(&right_hole_mid, &right_inner_hole_bot, -14),
		// PP_DISTANCE(&left_hole_mid, &left_inner_hole_top, 14),
		// PP_DISTANCE(&left_hole_mid, &left_inner_hole_bot, -14),
		// POINT_ON_LINE(&right_inner_hole_top, &right_inner_hole_bisect),
		// POINT_ON_LINE(&right_inner_hole_bot, &right_inner_hole_bisect),
		// POINT_ON_LINE(&left_inner_hole_top, &left_inner_hole_bisect),
		// POINT_ON_LINE(&left_inner_hole_bot, &left_inner_hole_bisect),


		CEND(),
	});

	add_fragment(&topo, (struct topology_elem[]){
		TOPO_MOVETO(&mid_bot),
		TOPO_LINETO(&right_corner),
		TOPO_LINETO(&below_right_hole_corner),
		TOPO_LINETO(&right_hole_bot),
		TOPO_ARCTO(&right_hole_mid, &right_hole_top),

		TOPO_ARCTO(&swoop_mid, &swoop_top),
		TOPO_ARCTO(&swoop_mid, &left_hole_top),
		TOPO_ARCTO(&left_hole_mid, &left_hole_bot),

		TOPO_LINETO(&below_left_hole_corner),
		TOPO_LINETO(&left_corner),
		TOPO_LINETO(&mid_bot),

		TOPO_MOVETO(&right_inner_hole_bot),
		TOPO_ARCTO(&right_hole_mid, &right_inner_hole_top),
		TOPO_ARCTO(&right_hole_mid, &right_inner_hole_bot),

		TOPO_MOVETO(&left_inner_hole_bot),
		TOPO_ARCTO(&left_hole_mid, &left_inner_hole_top),
		TOPO_ARCTO(&left_hole_mid, &left_inner_hole_bot),
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
