#pragma once

#include "cad/solve.h"
#include "cad/topology.h"

enum LineStyle {
	LSTYLE_NORMAL,
	LSTYLE_CONSTRUCTION,
	LSTYLE_INDICATOR,
};
#define TEXT_OFFSET 0.4

void plot_line_between(struct point p1, struct point p2);
void plot_arc_between_style(struct point c, struct point p1, struct point p2, enum LineStyle style);
void plot_arc_between(struct point c, struct point p1, struct point p2);
void plot_text(struct point p, double angle, char* str);

void draw_constraints(struct constraints *constraints);
void draw_topology(struct topology *topo);
