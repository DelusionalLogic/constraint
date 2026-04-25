#include "cad/solve.h"
#include "cad/debug.h"
#include <assert.h>
#include <math.h>
#include <string.h>

static double point_line_distance_abs(struct point p, struct line l) {
	return fabs((glm_vec2_dot(l.norm, p.pos) + l.C) / glm_vec2_norm(l.norm));
}

static double point_point_distance(struct point p1, struct point p2) {
	vec2 between;
	glm_vec2_sub(p1.pos, p2.pos, between);
	return glm_vec2_norm(between);
}

static double line_line_angle_abs(struct line l1, struct line l2) {
	double dot = glm_vec2_dot(l1.norm, l2.norm);
	double mag = glm_vec2_norm(l1.norm) * glm_vec2_norm(l2.norm);
	double c = fabs(dot / mag);
	c = fmin(1.0, fmax(-1.0, c));
	return acos(c);
}

static void assert_near(double actual, double expected) {
	assert(fabs(actual - expected) < 1e-6);
}

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

	memset(assemblies, 0, sizeof(assemblies));
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

	memset(assemblies, 0, sizeof(assemblies));
	{
		printf("Corner by point-on-line root\n");
		struct constraints constraints = {};

		struct component base = {.type = COM_LINE};
		struct component side = {.type = COM_LINE};
		struct component corner = {.type = COM_POINT};
		struct component tip = {.type = COM_POINT};

		add_constraint(&constraints, (struct constraint[]){
			POINT_ON_LINE(&corner, &base),
			POINT_ON_LINE(&corner, &side),
			LL_ANGLE(&base, &side, DEG(90)),
			PL_DISTANCE(&tip, &base, 10),
			PL_DISTANCE(&tip, &side, -20),
			CEND(),
		});

		struct drawing drawing = {};
		bool solved = solve_constraints(&constraints, &drawing, assemblies, &assemblies_num);

		assert(solved);
		assert(assemblies_num == 1);
		assert(drawing.root != NULL);

		double *params = malloc(sizeof(double) * constraints.length);
		for(size_t i = 0; i < constraints.length; i++) {
			params[i] = constraints.elements[i].v;
		}
		place_points(&drawing, params);
		free(params);

		assert(corner.e != NULL);
		assert(tip.e != NULL);
		assert(base.e != NULL);
		assert(side.e != NULL);

		print_subassemblies(&constraints, assemblies, assemblies_num);

		assert_near(point_line_distance_abs(corner.e->point, base.e->line), 0.0);
		assert_near(point_line_distance_abs(corner.e->point, side.e->line), 0.0);
		assert_near(line_line_angle_abs(base.e->line, side.e->line), DEG(90));
		assert_near(point_line_distance_abs(tip.e->point, base.e->line), 10.0);
		assert_near(point_line_distance_abs(tip.e->point, side.e->line), 20.0);

		free_drawing(&drawing);
		free_constraints(&constraints);
	}

	memset(assemblies, 0, sizeof(assemblies));
	{
		printf("Corner by offset point-line root\n");
		struct constraints constraints = {};

		struct component base = {.type = COM_LINE};
		struct component side = {.type = COM_LINE};
		struct component corner = {.type = COM_POINT};
		struct component tip = {.type = COM_POINT};

		add_constraint(&constraints, (struct constraint[]){
			PL_DISTANCE(&corner, &base, 5),
			POINT_ON_LINE(&corner, &side),
			LL_ANGLE(&base, &side, DEG(90)),
			PL_DISTANCE(&tip, &base, 10),
			PL_DISTANCE(&tip, &side, -20),
			CEND(),
		});

		struct drawing drawing = {};
		bool solved = solve_constraints(&constraints, &drawing, assemblies, &assemblies_num);

		assert(solved);
		assert(assemblies_num == 1);
		assert(drawing.root != NULL);

		double *params = malloc(sizeof(double) * constraints.length);
		for(size_t i = 0; i < constraints.length; i++) {
			params[i] = constraints.elements[i].v;
		}
		place_points(&drawing, params);
		free(params);

		assert(corner.e != NULL);
		assert(tip.e != NULL);
		assert(base.e != NULL);
		assert(side.e != NULL);

		assert_near(point_line_distance_abs(corner.e->point, base.e->line), 5.0);
		assert_near(point_line_distance_abs(corner.e->point, side.e->line), 0.0);
		assert_near(line_line_angle_abs(base.e->line, side.e->line), DEG(90));
		assert_near(point_line_distance_abs(tip.e->point, base.e->line), 10.0);
		assert_near(point_line_distance_abs(tip.e->point, side.e->line), 20.0);
		assert_near(point_point_distance(corner.e->point, tip.e->point), sqrt(425.0));

		free_drawing(&drawing);
		free_constraints(&constraints);
	}

	memset(assemblies, 0, sizeof(assemblies));
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

	memset(assemblies, 0, sizeof(assemblies));
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
