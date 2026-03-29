#include "cad/solve.h"
#include <assert.h>
#include <string.h>

int main(int argc, char *argv[]) {
	struct subassembly assemblies[16] = {};
	size_t assemblies_num;
	{
		printf("Triangle by 3 distances\n");
		struct constraints constraints = {};

		struct component p1 = {.type = COM_POINT};
		struct component p2 = {.type = COM_POINT};
		struct component p3 = {.type = COM_POINT};

		add_constraint(&constraints, (struct constraint[]){
			PP_DISTANCE(&p1, &p2, 30),
			PP_DISTANCE(&p1, &p3, 30),
			PP_DISTANCE(&p2, &p3, 30),
			CEND(),
		});

		struct drawing drawing = {};
		bool solved = solve_constraints(&constraints, &drawing, assemblies, &assemblies_num);

		assert(solved);

		free_drawing(&drawing);
		free_constraints(&constraints);
	}

	{
		printf("Triangle by 2 angles and a distance\n");
		struct constraints constraints = {};

		struct component l1 = {.type = COM_LINE};
		struct component l2 = {.type = COM_LINE};
		struct component l3 = {.type = COM_LINE};

		struct component p1 = {.type = COM_POINT};
		struct component p2 = {.type = COM_POINT};
		struct component p3 = {.type = COM_POINT};

		add_constraint(&constraints, (struct constraint[]){
			POINT_ON_LINE(&p1, &l3),
			POINT_ON_LINE(&p1, &l1),

			POINT_ON_LINE(&p2, &l1),
			POINT_ON_LINE(&p2, &l2),

			POINT_ON_LINE(&p3, &l2),
			POINT_ON_LINE(&p3, &l3),

			PP_DISTANCE(&p1, &p2, 30),
			LL_ANGLE(&l1, &l2, DEG(60)),
			LL_ANGLE(&l2, &l3, DEG(60)),
			CEND(),
		});

		struct drawing drawing = {};
		bool solved = solve_constraints(&constraints, &drawing, assemblies, &assemblies_num);

		assert(solved);

		free_drawing(&drawing);
		free_constraints(&constraints);
	}

	{
		printf("Two triangles sharing a point one defined by angles\n");
		struct constraints constraints = {};

		struct component p1 = {.type = COM_POINT};
		struct component p2 = {.type = COM_POINT};
		struct component p3 = {.type = COM_POINT};

		struct component p4 = {.type = COM_POINT};
		struct component p5 = {.type = COM_POINT};

		struct component t1base = {.type = COM_LINE};
		struct component t2base = {.type = COM_LINE};
		struct component t2side = {.type = COM_LINE};

		add_constraint(&constraints, (struct constraint[]){
			// Make a triangle
			PP_DISTANCE(&p1, &p2, 30),
			PP_DISTANCE(&p1, &p3, 30),
			PP_DISTANCE(&p2, &p3, 30),

			POINT_ON_LINE(&p1, &t1base),
			POINT_ON_LINE(&p2, &t1base),

			// With another triangle sharing a point
			PP_DISTANCE(&p4, &p5, 30),
			PP_DISTANCE(&p3, &p4, 30),

			LL_ANGLE(&t2side, &t2base, DEG(60)),

			POINT_ON_LINE(&p3, &t2side),
			POINT_ON_LINE(&p4, &t2side),

			POINT_ON_LINE(&p4, &t2base),
			POINT_ON_LINE(&p5, &t2base),

			// The edge they don't share is constrained
			LL_ANGLE(&t1base, &t2base, DEG(40)),
			CEND(),
		});

		struct drawing drawing = {};
		bool solved = solve_constraints(&constraints, &drawing, assemblies, &assemblies_num);
		assert(solved);

		free_drawing(&drawing);
		free_constraints(&constraints);
	}

	{
		printf("Two triangles sharing a point with an angle constrained base\n");
		struct constraints constraints = {};

		struct component p1 = {.type = COM_POINT};
		struct component p2 = {.type = COM_POINT};
		struct component p3 = {.type = COM_POINT};

		struct component p4 = {.type = COM_POINT};
		struct component p5 = {.type = COM_POINT};

		struct component t1base = {.type = COM_LINE};
		struct component t2base = {.type = COM_LINE};

		add_constraint(&constraints, (struct constraint[]){
			// Make a triangle
			PP_DISTANCE(&p1, &p2, 30),
			PP_DISTANCE(&p1, &p3, 30),
			PP_DISTANCE(&p2, &p3, 30),

			POINT_ON_LINE(&p1, &t1base),
			POINT_ON_LINE(&p2, &t1base),

			// With another triangle sharing a point
			PP_DISTANCE(&p4, &p5, 30),
			PP_DISTANCE(&p3, &p4, 30),
			PP_DISTANCE(&p3, &p5, 30),

			POINT_ON_LINE(&p4, &t2base),
			POINT_ON_LINE(&p5, &t2base),

			// The edge they don't share is constrained
			LL_ANGLE(&t1base, &t2base, DEG(70)),
			CEND(),
		});

		struct drawing drawing = {};
		bool solved = solve_constraints(&constraints, &drawing, assemblies, &assemblies_num);

		// Copy over all the parameter values to a new array
		// @PERF: Maybe we should just store them in a separate array to start with
		double *params = malloc(sizeof(double) * (constraints.length));
		for(size_t i = 0; i < constraints.length; i++) {
			params[i] = constraints.elements[i].v;
		}
		place_points(&drawing, params);
		free(params);

		reconstruct_drawing(&constraints, assemblies, &assemblies_num);

		assert(solved);

		assert(glm_vec2_eqv_eps(p1.e->point.pos, (vec2){ 0.0     ,  0.0     }));
		assert(glm_vec2_eqv_eps(p2.e->point.pos, (vec2){30.0     ,  0.0     }));
		assert(glm_vec2_eqv_eps(p3.e->point.pos, (vec2){15.0     , 25.980762}));

		assert(glm_vec2_eqv_eps(p4.e->point.pos, (vec2){34.283630,  2.999428}));
		assert(glm_vec2_eqv_eps(p5.e->point.pos, (vec2){44.544235, 31.190207}));

		free_drawing(&drawing);
		free_constraints(&constraints);
	}

	return 0;
}
