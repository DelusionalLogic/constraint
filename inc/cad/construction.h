#pragma once
#include <cglm/cglm.h>

enum operation {
	CMD_VALUE_INPUT,
	CMD_ORIGIN,
	CMD_LINE_X,
	CMD_CIRCLE_CENTER_RADIUS,
	CMD_CIRCLE_CENTER_POINT,
	CMD_LINE_POINT_POINT,
	CMD_LINE_POINT_LINE_ANGLE,
	CMD_POINT_CIRCLE_LINE,
	CMD_POINT_CIRCLE_CIRCLE,
	CMD_POINT_LINE_LINE,

	// Additional operations we need to handle explicitly to avoid degenerate
	// cases
	CMD_LINE_CIRCLE_CIRCLE_TANGENT,
	CMD_LINE_LINE_DISTANCE_PARALLEL,
};

struct point {
	vec2 pos;
};

struct circle {
	vec2 center;
	double radius;
};

struct line {
	vec2 norm;
	double C;
};

enum EType {
	ETYPE_VALUE,
	ETYPE_CIRCLE,
	ETYPE_POINT,
	ETYPE_LINE,
};

struct element {
	enum EType type;

	union {
		double value;
		struct circle circle;
		struct point point;
		struct line line;
	};
};

struct command {
	enum operation op;
	uint8_t root;
	bool hidden;

	size_t index;
	struct element *arg1;
	struct element *arg2;
	struct element *arg3;

	struct element result;

	struct command *next;
};

struct drawing {
	struct command *root;
	struct command *tail;

	struct command *error;
};

struct element* insert_cmd(struct drawing *drawing, struct command cmd);
void execute_drawing(struct drawing *drawing, double inputs[]);

bool circle_line_intersect(struct circle circle, struct line line, uint8_t root, struct point *point);
void line_through_points(struct point p1, struct point p2, struct line* l);
void line_line_intersect(struct line l1, struct line l2, struct point* p);
