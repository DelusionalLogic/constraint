#include "cad/solve.h"
#include <assert.h>

int main(int argc, char *argv[]) {
	{
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

			// With another triangle sharing the topmost point
			PP_DISTANCE(&p4, &p5, 30),
			PP_DISTANCE(&p3, &p4, 30),
			PP_DISTANCE(&p3, &p5, 30),

			POINT_ON_LINE(&p4, &t2base),
			POINT_ON_LINE(&p5, &t2base),

			// The edge they don't share is constrained
			LL_ANGLE(&t1base, &t2base, DEG(40)),
			CEND(),
		});

		struct drawing drawing = {};
		bool solved = solve_constraints(&constraints, &drawing);

		assert(!solved);
	}

	return 0;
}
