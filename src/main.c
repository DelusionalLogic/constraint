#include <assert.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <cglm/cglm.h>
#include <string.h>
#include <limits.h>

#include "cad/construction.h"
#include "cad/solve.h"
#include "cad/svg.h"

#define SETSIGN(b, v) ((v) * ((2 * (b)) - 1))

#define DEG(x) ((x) * M_PI / 180.0)

#define PP_DISTANCE(C1, C2, D) \
	{ \
		.type = CT_POINT_POINT_DISTANCE, \
		.v = D, \
		.c1 = C1, \
		.c2 = C2, \
	}

#define PL_DISTANCE(C1, C2, D) \
	{ \
		.type = CT_POINT_LINE_DISTANCE, \
		.v = D, \
		.c1 = C1, \
		.c2 = C2, \
	}

#define POINT_ON_LINE(C1, C2) \
	PL_DISTANCE(C1, C2, 0)

#define LL_ANGLE(C1, C2, D) \
	{ \
		.type = CT_LINE_LINE_ANGLE, \
		.v = D, \
		.c1 = C1, \
		.c2 = C2, \
	}

#define CEND() \
	{ \
		.type = CT_END, \
	}

void plot_point(struct point p) {
	printf("<circle cx=\"%f\" cy=\"%f\" r=\".4\" fill=\"black\" />\n", p.pos[0], -p.pos[1]);
}

void plot_line_style(struct line l, enum LineStyle style) {
	char *style_str;
	switch(style) {
		case LSTYLE_NORMAL:
			style_str = "stroke=\"black\" stroke-width=\"0.2\"";
			break;
		case LSTYLE_CONSTRUCTION:
			style_str = "stroke=\"blue\" stroke-width=\"0.1\" stroke-dasharray=\"0.7,0.2\" stroke-opacity=\"0.3\"";
			break;
		case LSTYLE_INDICATOR:
			abort();
			break;
	}
	if(fabs(l.norm[0]) < fabs(l.norm[1])) {
		double minx = -50;
		double maxx =  50;

		double miny = -(l.norm[0] * minx + l.C) / l.norm[1];
		double maxy = -(l.norm[0] * maxx + l.C) / l.norm[1];
		printf("<line %s x1=\"%f\" y1=\"%f\" x2=\"%f\" y2=\"%f\" />\n", style_str, minx, -miny, maxx, -maxy);
	} else {
		double miny = -50;
		double maxy =  50;

		double minx = -(l.norm[1] * miny + l.C) / l.norm[0];
		double maxx = -(l.norm[1] * maxy + l.C) / l.norm[0];
		printf("<line %s x1=\"%f\" y1=\"%f\" x2=\"%f\" y2=\"%f\" />\n", style_str, minx, -miny, maxx, -maxy);
	}

	if(style == LSTYLE_NORMAL) {
		double d0 = glm_vec2_norm2(l.norm);

		vec2 p0 = {0, 0};
		glm_vec2_mulsubs(l.norm, l.C, p0);
		glm_vec2_divs(p0, d0, p0);

		vec2 p1;
		glm_vec2_add(p0, l.norm, p1);
		// printf("%f %f\n", l.norm[0], l.norm[1]);

		printf("<line x1=\"%f\" y1=\"%f\" x2=\"%f\" y2=\"%f\" stroke=\"black\" stroke-width=\".2\" />\n", p0[0], -p0[1], p1[0], -p1[1]);
	}
}

void plot_line(struct line l) {
	plot_line_style(l, LSTYLE_NORMAL);
}

void plot_circle(struct circle c) {
	printf("<circle cx=\"%f\" cy=\"%f\" r=\"%f\" fill=\"none\" stroke=\"black\" stroke-width=\".1\" />\n", c.center[0], -c.center[1], c.radius);
}

void plot_generic(struct element e) {
	switch(e.type) {
		case ETYPE_VALUE:
			break;
		case ETYPE_CIRCLE:
			plot_circle(e.circle);
			break;
		case ETYPE_POINT:
			plot_point(e.point);
			break;
		case ETYPE_LINE:
			plot_line(e.line);
			break;
	}
}

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

struct smooth_line init_smooth_line() {
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

struct box init_box() {
	return (struct box){
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
}

struct mid {
	struct component l1;
	struct component l2;

	struct component x1;

	struct component p1;

	struct component p;
};

struct mid init_mid() {
	return (struct mid) {
		.l1 = {.type = COM_LINE},
		.l2 = {.type = COM_LINE},

		.x1 = {.type = COM_POINT},

		.p1 = {.type = COM_LINE},

		.p = {.type = COM_POINT},
	};
}

int main(int argc, char *argv[]) {
	struct constraints constraints = {0};
	struct topology topo = {0};

	// Figure 4
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

	struct smooth_line line = init_smooth_line();

	add_fragment(&topo, (struct topology_elem[]){
		TOPO_MOVETO(&line.start),
		TOPO_LINETO(&line.corner_start),
		TOPO_ARCTO(&line.corner_center, &line.corner_end),
		TOPO_LINETO(&line.end),

		TOPO_END(),
	});

	alias_point(&constraints, &line.start, &components[5]);

	add_constraint(&constraints, (struct constraint[]){
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

	struct box box = init_box();
	add_fragment(&topo, (struct topology_elem[]){
		TOPO_MOVETO(&box.corner[0]),
		TOPO_LINETO(&box.corner[1]),
		TOPO_LINETO(&box.corner[2]),
		TOPO_LINETO(&box.corner[3]),
		TOPO_LINETO(&box.corner[0]),

		TOPO_END(),
	});

	add_constraint(&constraints, (struct constraint[]){
		POINT_ON_LINE(&box.corner[0], &box.side[0]),
		POINT_ON_LINE(&box.corner[1], &box.side[0]),

		POINT_ON_LINE(&box.corner[1], &box.side[1]),
		POINT_ON_LINE(&box.corner[2], &box.side[1]),

		POINT_ON_LINE(&box.corner[2], &box.side[2]),
		POINT_ON_LINE(&box.corner[3], &box.side[2]),

		POINT_ON_LINE(&box.corner[0], &box.side[3]),
		POINT_ON_LINE(&box.corner[3], &box.side[3]),

		LL_ANGLE(&box.side[3], &box.side[0], DEG(80)),
		LL_ANGLE(&box.side[1], &box.side[2], DEG(100)),
		LL_ANGLE(&box.side[1], &box.side[3], DEG(180)),
		LL_ANGLE(&components[3], &box.side[1], DEG(90)),

		PL_DISTANCE(&box.side[1], &box.corner[0], 20),

		PP_DISTANCE(&line.corner_end, &box.corner[0], 9.5),
		PP_DISTANCE(&line.corner_start, &box.corner[0], 10),
		CEND(),
	});

	struct mid box_enter = init_mid();

	add_constraint(&constraints, (struct constraint[]){
		LL_ANGLE(&box.side[3], &box_enter.l1, DEG(-30)),
		POINT_ON_LINE(&box.corner[0], &box_enter.l1),
		LL_ANGLE(&box.side[3], &box_enter.l2, DEG(30)),
		POINT_ON_LINE(&box.corner[3], &box_enter.l2),

		POINT_ON_LINE(&box_enter.l1, &box_enter.x1),
		POINT_ON_LINE(&box_enter.l2, &box_enter.x1),

		LL_ANGLE(&box.side[3], &box_enter.p1, DEG(90)),

		POINT_ON_LINE(&box_enter.x1, &box_enter.p1),
		POINT_ON_LINE(&box_enter.p, &box.side[3]),
		POINT_ON_LINE(&box_enter.p1, &box_enter.p),

		PP_DISTANCE(&box_enter.p, &box.corner[0], 5),
		CEND(),
	});

	struct drawing drawing = {};
	solve_constraints(&constraints, &drawing);
	// line.start.e = components[5].e;

	double *params = malloc(sizeof(double) * (constraints.length));
	for(size_t i = 0; i < constraints.length; i++) {
		if(!constraints.elements[i].used) continue;
		if(constraints.elements[i].order == 0) continue;

		params[constraints.elements[i].order-1] = SETSIGN(constraints.elements[i].forward, constraints.elements[i].v);

		// We need to offset angles based on the path we took to use this
		// constraint
		if(constraints.elements[i].path[0].i != -1) {
			// printf("PATH %ld order %llu\n", i, constraints[i].order-1);
			for(struct path_step *p = constraints.elements[i].path; p <= constraints.elements[i].path+SEARCH_DEPTH && p->i != -1; p++) {
				params[constraints.elements[i].order-1] += SETSIGN(p->direction, constraints.elements[p->i].v);
				// printf("%llu [%d:%f] [%f] -> ", p->i, p->direction, constraints[p->i].v, params[constraints[i].order-1]);
			}
			// printf("\n");
			params[constraints.elements[i].order-1] -= (M_PI*2.0) * floor(params[constraints.elements[i].order-1] / (M_PI*2.0));
			// printf("Final Angle is %f\n", params[constraints[i].order-1]);
		}

	}

	execute_drawing(&drawing, params);

	printf("<svg version=\"1.1\" viewBox=\"-50 -50 100 100\" width=\"1200\" height=\"1200\" xmlns=\"http://www.w3.org/2000/svg\">\n");

	// Axis lines
	printf("<line x1=\"-1000\" y1=\"0\" x2=\"1000\" y2=\"0\" stroke=\"black\" stroke-width=\"0.1\" stroke-opacity=\"0.4\" />\n");
	printf("<line y1=\"-1000\" x1=\"0\" y2=\"1000\" x2=\"0\" stroke=\"black\" stroke-width=\"0.1\" stroke-opacity=\"0.4\" />\n");

	for(struct command *current = drawing.root; current != NULL && current != drawing.error; current = current->next) {
		if(current->hidden) continue;
		plot_generic(current->result);
	}

	if(drawing.error != NULL) {
		plot_generic(*drawing.error->arg1);
		plot_generic(*drawing.error->arg2);
		fprintf(stderr, "Solver error detected. Drawing will be incomplete\n");
	}

	struct component *head = NULL;
	for(size_t i = 0; i < topo.length; i++) {
		struct topology_elem *cur = &topo.elements[i];
		switch(cur->cmd.op) {
			case TOPO_MOVETO:
				cur++;
				if(cur->arg.c->e != NULL) head = cur->arg.c;
				break;
			case TOPO_LINETO:
				cur++;
				if(cur->arg.c->e != NULL) {
					plot_line_between(head->e->point, cur->arg.c->e->point);
					head = cur->arg.c;
				}
				break;
			case TOPO_ARCTO:
				cur++;
				if(cur->arg.c->e != NULL && (cur+1)->arg.c->e != NULL) {
					plot_arc_between(cur->arg.c->e->point, (cur+1)->arg.c->e->point, head->e->point);
					head = (cur+1)->arg.c;
				}
				break;
			case TOPO_END:
				abort();
		}
	}

	// printf("%f %f %f\n", components[3].e->line.norm[0], components[3].e->line.norm[1], components[3].e->line.C);
	
	draw_constraints(&constraints);

	printf("</svg>\n");
}
