#pragma once

#include "cad/solve.h"
#include "cad/topology.h"

enum LineStyle {
	LSTYLE_NORMAL,
	LSTYLE_CONSTRUCTION,
	LSTYLE_INDICATOR,
	LSTYLE_INDICATOR_INLINE,
};
#define TEXT_OFFSET 0.4

enum CanvasState {
	CANVAS_INIT,
	CANVAS_DRAWING,
};

struct canvas {
	enum CanvasState state;
	FILE *f;
};

void line_distance_to_point(struct line l, double d, struct point *p);

void plot_line_between(struct canvas *canvas, struct point p1, struct point p2);
void plot_arc_between_style(struct canvas *canvas, struct point c, struct point p1, struct point p2, enum LineStyle style);
void plot_arc_between(struct canvas *canvas, struct point c, struct point p1, struct point p2);
void plot_text(struct canvas *canvas, struct point p, double angle, char* str);

void begin_drawing(struct canvas *canvas);
void draw_constraints(struct canvas *canvas, struct constraints *constraints);
void draw_topology(struct canvas *canvas, struct topology *topo);
void end_drawing(struct canvas *canvas);
