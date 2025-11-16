#include <assert.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <cglm/cglm.h>
#include <string.h>
#include <limits.h>

#include "cad/construction.h"
#include "cad/solve.h"

#define TEXT_OFFSET 0.4

#define SETSIGN(b, v) ((v) * ((2 * (b)) - 1))
#define SIGNOF(x) ((typeof(x))((x)>0) - ((x)<0))

#define DEG(x) ((x) * M_PI / 180.0)

enum topology_op {
	TOPO_MOVETO,
	TOPO_LINETO,
	TOPO_ARCTO,

	TOPO_END,
};

struct topology_cmd {
	enum topology_op op;
};

struct topology_arg {
	struct component *c;
};

struct topology_elem {
	union {
		struct topology_cmd cmd;
		struct topology_arg arg;
	};
};

#define MOVETO(c) \
	{ .cmd = {TOPO_MOVETO} }, \
	{ .arg = {c} } \

#define LINETO(c) \
	{ .cmd = {TOPO_LINETO} }, \
	{ .arg = {c} } \

#define ARCTO(center, end) \
	{ .cmd = {TOPO_ARCTO} }, \
	{ .arg = {center} }, \
	{ .arg = {end} } \

#define END() \
	{ .cmd = {TOPO_END} } \

struct topology {
	struct topology_elem *elements;
	size_t length;
	size_t capacity;
};

static void resize_buffer(void **buffer, size_t *capacity, size_t elems, size_t elemSize) {
	assert(elems > 0);
	uint64_t newpower = (sizeof(elems) * CHAR_BIT) - __builtin_clzl(elems-1);
	elems = 1 << newpower;

	if(elems != *capacity) {
		*capacity = elems;
		*buffer = realloc(*buffer, *capacity * elemSize);
	}
}

static void topo_resize(struct topology *topo, uint64_t newcapacity) {
	resize_buffer((void**)&topo->elements, &topo->capacity, newcapacity, sizeof(struct topology_elem));
}

size_t frag_len(struct topology_elem *elems) {
	struct topology_elem *cur = elems;
	while(cur->cmd.op != TOPO_END) {
		switch(cur->cmd.op) {
			case TOPO_MOVETO:
				cur += 2;
				break;
			case TOPO_LINETO:
				cur += 2;
				break;
			case TOPO_ARCTO:
				cur += 3;
				break;
			case TOPO_END:
				abort();
		}
	}

	return cur - elems;
}

void add_topo_fragment(struct topology *topo, struct topology_elem *new) {
	size_t new_num = frag_len(new);
	if(new_num + topo->length > topo->capacity) {
		topo_resize(topo, new_num + topo->length);
	}

	memcpy(topo->elements + topo->length, new, new_num * sizeof(struct topology_elem));
	topo->length += new_num;
}

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

struct constraints {
	struct constraint *elements;
	size_t length;
	size_t capacity;
};

void add_constraint(struct constraints *c, struct constraint *new) {
	struct constraint *end = new;
	while(end->type != CT_END) end++;
	size_t new_num = end - new;

	if(c->length + new_num > c->capacity) {
		resize_buffer((void**)&c->elements, &c->capacity, c->length + new_num, sizeof(struct constraint));
	}

	memcpy(c->elements + c->length, new, new_num * sizeof(struct constraint));
	c->length += new_num;
}

double project_point_to_line_distance(struct point p, struct line l) {
	vec2 offset = {-p.pos[0], -p.pos[1]};
	double c = glm_vec2_dot(l.norm, offset);

	double det = (l.norm[0] * p.pos[1]) - (l.norm[1] * p.pos[0]);
	det = -det;

	// @HACK There's a rounding error here that can cause d1 to end up
	// negative. Just take the abolute value of it to get around that.
	double d1 = fabs(glm_vec2_norm2(p.pos) - pow(c, 2)/glm_vec2_norm2(l.norm));
	assert(d1 >= 0.0);
	double m1 = sqrt(d1 / glm_vec2_norm2(l.norm));

	return SIGNOF(det) * m1;
}

void line_distance_to_point(struct line l, double d, struct point *p) {
	glm_vec2_zero(p->pos);
	glm_vec2_muladds(l.norm, l.C, p->pos);
	glm_vec2_divs(p->pos, glm_vec2_norm2(l.norm), p->pos);
	glm_vec2_negate(p->pos);

	vec2 perp = {l.norm[1], -l.norm[0]};
	glm_vec2_muladds(perp, d, p->pos);
}

void plot_point(struct point p) {
	printf("<circle cx=\"%f\" cy=\"%f\" r=\".4\" fill=\"black\" />\n", p.pos[0], -p.pos[1]);
}

enum LineStyle {
	LSTYLE_NORMAL,
	LSTYLE_CONSTRUCTION,
	LSTYLE_INDICATOR,
};

void plot_line_between_style(struct point p1, struct point p2, enum LineStyle style) {
	char *style_str;
	switch(style) {
		case LSTYLE_NORMAL:
			style_str = "stroke=\"black\" stroke-width=\"0.2\"";
			break;
		case LSTYLE_CONSTRUCTION:
			style_str = "stroke=\"blue\" stroke-width=\"0.1\" stroke-dasharray=\"0.7,0.2\" stroke-opacity=\"0.3\"";
			break;
		case LSTYLE_INDICATOR:
			style_str = "stroke=\"blue\" stroke-width=\"0.1\" stroke-opacity=\"0.3\"";
			break;
	}

	printf("<line %s x1=\"%f\" y1=\"%f\" x2=\"%f\" y2=\"%f\" />\n", style_str, p1.pos[0], -p1.pos[1], p2.pos[0], -p2.pos[1]);
}

void plot_line_between(struct point p1, struct point p2) {
	plot_line_between_style(p1, p2, LSTYLE_NORMAL);
}

void plot_text(struct point p, double angle, char* str) {
	printf("<text text-anchor=\"middle\" dominant-baseline=\"central\" transform=\"translate(%f, %f) scale(1, -1) rotate(%f) scale(1, -1)\" font-size=\"0.75\">%s</text>\n", p.pos[0], -p.pos[1], angle * (180.0/M_PI), str);
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

void plot_arc_between_style(struct point c, struct point p1, struct point p2, enum LineStyle style) {
	char *style_str;
	switch(style) {
		case LSTYLE_NORMAL:
			style_str = "stroke=\"black\" stroke-width=\"0.2\"";
			break;
		case LSTYLE_CONSTRUCTION:
			abort();
			break;
		case LSTYLE_INDICATOR:
			style_str = "stroke=\"blue\" stroke-width=\"0.1\" stroke-opacity=\"0.3\"";
			break;
	}

	bool above_pi;
	{
		vec2 p1l;
		glm_vec2_sub(p1.pos, c.pos, p1l);
		glm_vec2_normalize(p1l);
		vec2 p2l;
		glm_vec2_sub(p2.pos, c.pos, p2l);
		glm_vec2_normalize(p2l);

		glm_vec2_copy((vec2){-p2l[1], p2l[0]}, p2l);

		above_pi = glm_vec2_dot(p1l, p2l) < 0;
	}

	// @COML: There's something missing here about which side of the arc we
	// want. I think we can figure that out from the relation of the center and
	// the points
	vec2 t;
	glm_vec2_sub(c.pos, p1.pos, t);
	double r = glm_vec2_norm(t);
	printf("<path %s d=\"M %f %f A %f %f 0 %d 0 %f %f\" fill=\"none\" />\n", style_str, p2.pos[0], -p2.pos[1], r, r, above_pi, p1.pos[0], -p1.pos[1]);
}

void plot_arc_between(struct point c, struct point p1, struct point p2) {
	plot_arc_between_style(c, p1, p2, LSTYLE_NORMAL);
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

void plot_distance_indicator(struct point p1, struct point p2, double distance) {
	vec2 dir;
	glm_vec2_sub(p2.pos, p1.pos, dir);
	glm_vec2_normalize(dir);
	vec2 norm = {-dir[1], dir[0]};

	struct point start;
	struct point end;
	{
		glm_vec2_add(p1.pos, norm, start.pos);
		glm_vec2_add(p2.pos, norm, end.pos);
		plot_line_between_style(start, end, LSTYLE_INDICATOR);
	}

	// The little wings to highlight the ends
	vec2 tip = {.3, .3};
	glm_vec2_mul(norm, tip, tip);
	{
		struct point p1;
		struct point p2;
		glm_vec2_add(start.pos, tip, p1.pos);
		glm_vec2_sub(start.pos, tip, p2.pos);
		plot_line_between_style(p1, p2, LSTYLE_INDICATOR);
	}
	{
		struct point p1;
		struct point p2;
		glm_vec2_add(end.pos, tip, p1.pos);
		glm_vec2_sub(end.pos, tip, p2.pos);
		plot_line_between_style(p1, p2, LSTYLE_INDICATOR);
	}

	// The text
	{
		struct point p;
		glm_vec2_lerp(start.pos, end.pos, 0.5, p.pos);

		glm_vec2_muladds(norm, TEXT_OFFSET, p.pos);

		double angle = atan2(dir[1], dir[0]);

		// Flip upside down labels
		if(angle > M_PI/2) {
			glm_vec2_muladds(norm, 0.1, p.pos);
			angle -= M_PI;
		}
		if(angle < -M_PI/2) {
			glm_vec2_muladds(norm, 0.1, p.pos);
			angle += M_PI;
		}

		assert(angle >= -M_PI);
		assert(angle <=  M_PI);

		char buf[512];
		snprintf(buf, sizeof(buf), "%.1f u", distance);
		plot_text(p, angle, buf);
	}
}

void plot_angle(struct line l1, struct line l2, double theta, struct point *intersect, struct point *p1, struct point *p2) {
	line_line_intersect(l1, l2, intersect);

	struct circle c = { .radius = 1.3 };
	glm_vec2_copy(intersect->pos, c.center);

	circle_line_intersect(c, l1, 0, p1);
	circle_line_intersect(c, l2, 0, p2);

	plot_arc_between_style(*intersect, *p2, *p1, LSTYLE_INDICATOR);

	vec2 l1v;
	vec2 l2v;
	glm_vec2_normalize_to(l1.norm, l1v);
	glm_vec2_normalize_to(l2.norm, l2v);

	// Label
	glm_vec2_negate(l2v);

	vec2 x;
	glm_vec2_add(l1v, l2v, x);
	glm_vec2_normalize(x);

	struct point label_point;
	glm_vec2_copy(intersect->pos, label_point.pos);
	glm_vec2_muladds(x, c.radius + TEXT_OFFSET, label_point.pos);

	double dot = x[0];
	double det = x[1];

	double angle = atan2(det, dot);

	char buf[512];
	snprintf(buf, sizeof(buf), "%.1f°", theta);
	plot_text(label_point, angle - M_PI/2, buf);

	// plot_line_style(l1, LSTYLE_CONSTRUCTION);
	// plot_line_style(l2, LSTYLE_CONSTRUCTION);
}

struct element *create_perp(struct drawing *cmds, struct element *l, struct element *p, bool hide) {
	static struct element radius = {
		.type = ETYPE_VALUE,
		.value = 5, // Just an arbitrary number
	};

	struct element *c1 = insert_cmd(cmds, (struct command){
		.op = CMD_CIRCLE_CENTER_RADIUS,
		.result.type = ETYPE_CIRCLE,
		.hidden = true,
		.arg1 = p,
		.arg2 = &radius,
	});
	struct element *p1 = insert_cmd(cmds, (struct command){
		.op = CMD_POINT_CIRCLE_LINE,
		.result.type = ETYPE_POINT,
		.hidden = true,
		.root = 0,
		.arg1 = c1,
		.arg2 = l,
	});
	struct element *p2 = insert_cmd(cmds, (struct command){
		.op = CMD_POINT_CIRCLE_LINE,
		.result.type = ETYPE_POINT,
		.hidden = true,
		.root = 1,
		.arg1 = c1,
		.arg2 = l,
	});

	struct element *c2 = insert_cmd(cmds, (struct command){
		.op = CMD_CIRCLE_CENTER_POINT,
		.result.type = ETYPE_CIRCLE,
		.hidden = true,
		.arg1 = p2,
		.arg2 = p1,
	});
	struct element *c3 = insert_cmd(cmds, (struct command){
		.op = CMD_CIRCLE_CENTER_POINT,
		.result.type = ETYPE_CIRCLE,
		.hidden = true,
		.arg1 = p1,
		.arg2 = p2,
	});

	struct element *perp1 = insert_cmd(cmds, (struct command){
		.op = CMD_POINT_CIRCLE_CIRCLE,
		.result.type = ETYPE_POINT,
		.hidden = true,
		.arg1 = c2,
		.arg2 = c3,
	});
	struct element *perp2 = insert_cmd(cmds, (struct command){
		.op = CMD_POINT_CIRCLE_CIRCLE,
		.result.type = ETYPE_POINT,
		.hidden = true,
		.root = 1,
		.arg1 = c2,
		.arg2 = c3,
	});

	return insert_cmd(cmds, (struct command){
		.op = CMD_LINE_POINT_POINT,
		.result.type = ETYPE_LINE,
		.hidden = hide,
		.arg1 = perp1,
		.arg2 = perp2,
	});
}

void create_rounded_3line(struct drawing *next, struct element *r, struct element *p1, struct element *p2, struct element *p3, struct element **center, struct element **start, struct element **end) {
	{
		struct element *par1;
		struct element *par2;
		{
			struct element *l = insert_cmd(next, (struct command){
				.op = CMD_LINE_POINT_POINT,
				.hidden = true,
				.result.type = ETYPE_LINE,
				.arg1 = p1,
				.arg2 = p2,
			});

			struct element *perp = create_perp(next, l, p2, true);

			struct element *c = insert_cmd(next, (struct command){
				.op = CMD_CIRCLE_CENTER_RADIUS,
				.hidden = true,
				.result.type = ETYPE_CIRCLE,
				.arg1 = p2,
				.arg2 = r,
			});

			struct element *p = insert_cmd(next, (struct command){
				.op = CMD_POINT_CIRCLE_LINE,
				.hidden = true,
				.result.type = ETYPE_POINT,
				.arg1 = c,
				.arg2 = perp,
			});

			par1 = create_perp(next, perp, p, true);
		}

		{
			struct element *l = insert_cmd(next, (struct command){
				.op = CMD_LINE_POINT_POINT,
				.hidden = true,
				.result.type = ETYPE_LINE,
				.arg1 = p2,
				.arg2 = p3,
			});

			struct element *perp = create_perp(next, l, p2, true);

			struct element *c = insert_cmd(next, (struct command){
				.op = CMD_CIRCLE_CENTER_RADIUS,
				.hidden = true,
				.result.type = ETYPE_CIRCLE,
				.arg1 = p2,
				.arg2 = r,
			});

			struct element *p = insert_cmd(next, (struct command){
				.op = CMD_POINT_CIRCLE_LINE,
				.hidden = true,
				.result.type = ETYPE_POINT,
				.arg1 = c,
				.arg2 = perp,
			});

			par2 = create_perp(next, perp, p, true);
		}

		struct element *pc = insert_cmd(next, (struct command){
			.op = CMD_POINT_LINE_LINE,
			.hidden = true,
			.result.type = ETYPE_POINT,
			.arg1 = par1,
			.arg2 = par2,
		});
		*center = pc;

		struct element *c = insert_cmd(next, (struct command){
			.op = CMD_CIRCLE_CENTER_RADIUS,
			.hidden = true,
			.result.type = ETYPE_CIRCLE,
			.arg1 = pc,
			.arg2 = r,
		});

		{
			struct element *l = insert_cmd(next, (struct command){
				.op = CMD_LINE_POINT_POINT,
				.hidden = true,
				.result.type = ETYPE_LINE,
				.arg1 = p1,
				.arg2 = p2,
			});

			struct element *p = insert_cmd(next, (struct command){
				.op = CMD_POINT_CIRCLE_LINE,
				.hidden = true,
				.result.type = ETYPE_POINT,
				.arg1 = c,
				.arg2 = l,
			});
			*start = p;
		}

		{
			struct element *l = insert_cmd(next, (struct command){
				.op = CMD_LINE_POINT_POINT,
				.hidden = true,
				.result.type = ETYPE_LINE,
				.arg1 = p2,
				.arg2 = p3,
			});

			struct element *p = insert_cmd(next, (struct command){
				.op = CMD_POINT_CIRCLE_LINE,
				.hidden = true,
				.result.type = ETYPE_POINT,
				.arg1 = c,
				.arg2 = l,
			});
			*end = p;
		}

	}
}

void create_drawing(struct drawing* drawing, struct element **line_start, struct element **line_bend_start, struct element **line_ctr, struct element **line_bend_end, struct element **line_end) {
	*line_start = insert_cmd(drawing, (struct command){
		.op = CMD_ORIGIN,
		.hidden = true,
		.result.type = ETYPE_POINT,
	});

	struct element *element_spacing = insert_cmd(drawing, (struct command){
		.op = CMD_VALUE_INPUT,
		.result.type = ETYPE_VALUE,
	});

	struct element *line_corner;
	{
		struct element *xaxis = insert_cmd(drawing, (struct command){
			.op = CMD_LINE_X,
			.hidden = true,
			.result.type = ETYPE_LINE,
		});

		struct element *root_angle = insert_cmd(drawing, (struct command){
			.op = CMD_VALUE_INPUT,
			.result.type = ETYPE_VALUE,
		});

		struct element *l = insert_cmd(drawing, (struct command){
			.op = CMD_LINE_POINT_LINE_ANGLE,
			.hidden = true,
			.result.type = ETYPE_LINE,
			.arg1 = *line_start,
			.arg2 = xaxis,
			.arg3 = root_angle,
		});

		struct element *c = insert_cmd(drawing, (struct command){
			.op = CMD_CIRCLE_CENTER_RADIUS,
			.hidden = true,
			.result.type = ETYPE_CIRCLE,
			.arg1 = *line_start,
			.arg2 = element_spacing,
		});

		struct element *p = insert_cmd(drawing, (struct command){
			.op = CMD_POINT_CIRCLE_LINE,
			.hidden = true,
			.result.type = ETYPE_POINT,
			.arg1 = c,
			.arg2 = l,
		});
		line_corner = p;
	}

	{
		struct element *li = insert_cmd(drawing, (struct command){
			.op = CMD_LINE_POINT_POINT,
			.hidden = true,
			.result.type = ETYPE_LINE,
			.arg1 = *line_start,
			.arg2 = line_corner,
		});

		struct element *bend_angle = insert_cmd(drawing, (struct command){
			.op = CMD_VALUE_INPUT,
			.hidden = true,
			.result.type = ETYPE_VALUE,
		});

		struct element *l = insert_cmd(drawing, (struct command){
			.op = CMD_LINE_POINT_LINE_ANGLE,
			.hidden = true,
			.result.type = ETYPE_LINE,
			.arg1 = line_corner,
			.arg2 = li,
			.arg3 = bend_angle,
		});

		struct element *c = insert_cmd(drawing, (struct command){
			.op = CMD_CIRCLE_CENTER_RADIUS,
			.hidden = true,
			.result.type = ETYPE_CIRCLE,
			.arg1 = line_corner,
			.arg2 = element_spacing,
		});

		struct element *p = insert_cmd(drawing, (struct command){
			.op = CMD_POINT_CIRCLE_LINE,
			.hidden = true,
			.result.type = ETYPE_POINT,
			.arg1 = c,
			.arg2 = l,
		});
		*line_end = p;
	}


	struct element *round_rad = insert_cmd(drawing, (struct command){
		.op = CMD_VALUE_INPUT,
		.hidden = true,
		.result.type = ETYPE_VALUE,
	});
	create_rounded_3line(drawing, round_rad, *line_start, line_corner, *line_end, line_ctr, line_bend_start, line_bend_end);
}

void extend_line_to(struct component* c, struct point *p) {
	assert(c->type == COM_LINE);

	double val = project_point_to_line_distance(*p, c->e->line);
	if(!c->min_max_init) {
		c->min_max_init = true;
		c->min = val;
		c->max = val;
	} else {
		c->min = fmin(c->min, val - 0.1);
		c->max = fmax(c->max, val + 0.1);
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
	// struct element *line_start;
	// struct element *line_bend_start;
	// struct element *line_ctr;
	// struct element *line_bend_end;
	// struct element *line_end;

	// struct drawing drawing = {};
	// create_drawing(&drawing, &line_start, &line_bend_start, &line_ctr, &line_bend_end, &line_end);
	// execute_drawing(&drawing, (double[]){8, M_PI * 0.5, M_PI * 0.4, .3});

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

	add_topo_fragment(&topo, (struct topology_elem[]){
		MOVETO(&components[1]),
		LINETO(&components[5]),
		LINETO(&components[0]),
		LINETO(&components[2]),
		LINETO(&components[1]),

		END(),
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

	add_topo_fragment(&topo, (struct topology_elem[]){
		MOVETO(&components[5]),
		LINETO(&line.corner_start),
		ARCTO(&line.corner_center, &line.corner_end),
		LINETO(&line.end),

		END(),
	});

	add_constraint(&constraints, (struct constraint[]){
		LL_ANGLE(&components[3], &line.l1, DEG(90)),
		POINT_ON_LINE(&components[5], &line.l1),

		LL_ANGLE(&line.l2, &line.l1, DEG(90)),
		POINT_ON_LINE(&line.corner, &line.l1),
		POINT_ON_LINE(&line.corner, &line.l2),

		POINT_ON_LINE(&line.end, &line.l2),
		PL_DISTANCE(&line.end, &line.l1, 10),
		PL_DISTANCE(&line.end, &components[3], 12),

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
	add_topo_fragment(&topo, (struct topology_elem[]){
		MOVETO(&box.corner[0]),
		LINETO(&box.corner[1]),
		LINETO(&box.corner[2]),
		LINETO(&box.corner[3]),
		LINETO(&box.corner[0]),

		END(),
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
	solve_constraints(constraints.elements, constraints.length, &drawing);

	double *params = malloc(sizeof(double) * (constraints.length));
	for(size_t i = 0; i < constraints.length; i++) {
		if(!constraints.elements[i].used) continue;

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
	
	if(true) {
		for(size_t i = 0; i < constraints.length; i++) {
			struct constraint *constraint = &constraints.elements[i];
			if(!constraint->used) continue;

			switch(constraint->type) {
				case CT_POINT_POINT_DISTANCE: {
				} break;
				case CT_LINE_LINE_ANGLE: {
					assert(constraint->c1->type == COM_LINE);
					assert(constraint->c2->type == COM_LINE);

					struct point intersect;
					line_line_intersect(constraint->c1->e->line, constraint->c2->e->line, &intersect);
				} break;
				case CT_POINT_LINE_DISTANCE:
					break;
				case CT_END: abort();
			}
		}

		for(size_t i = 0; i < constraints.length; i++) {
			struct constraint *constraint = &constraints.elements[i];
			if(!constraint->used) continue;

			switch(constraint->type) {
				case CT_POINT_POINT_DISTANCE: {
					assert(constraint->c1->type == COM_POINT);
					assert(constraint->c2->type == COM_POINT);
					plot_distance_indicator(constraint->c1->e->point, constraint->c2->e->point, constraint->v);
				} break;
				case CT_LINE_LINE_ANGLE: {
					assert(constraint->c1->type == COM_LINE);
					assert(constraint->c2->type == COM_LINE);

					// Check if it's more likely a "parallel" constraint
					if(fabs(sin(constraint->v)) < 0.1) continue;

					struct component *l1 = constraint->c1;
					struct component *l2 = constraint->c2;
					// Negative angles are counterclockwise, we have to swap
					// the arguments to get proper rendering.
					if(constraint->v < 0) {
						l1 = constraint->c2;
						l2 = constraint->c1;
					}

					struct point p1;
					struct point p2;
					struct point intersect;
					plot_angle(l1->e->line, l2->e->line, constraint->v, &intersect, &p1, &p2);

					extend_line_to(constraint->c1, &intersect);
					extend_line_to(constraint->c2, &intersect);
					extend_line_to(l1, &p1);
					extend_line_to(l2, &p2);
				} break;
				case CT_POINT_LINE_DISTANCE: {
					struct component *line;
					struct component *point;
					if(constraint->c1->type == COM_LINE) {
						line = constraint->c1;
						point = constraint->c2;
					} else {
						line = constraint->c2;
						point = constraint->c1;
					}
					assert(point->type == COM_POINT);
					assert(line->type == COM_LINE);

					if(constraint->v > 0) {
						double dist = project_point_to_line_distance(point->e->point, line->e->line);
						struct point closest;
						line_distance_to_point(line->e->line, dist, &closest);

						plot_distance_indicator(point->e->point, closest, constraint->v);
					}

					extend_line_to(line, &point->e->point);
				} break;
				case CT_END: abort();
			}
		}

		for(size_t i = 0; i < constraints.length; i++) {
			struct constraint *constraint = &constraints.elements[i];
			if(!constraint->used) continue;

			switch(constraint->type) {
				case CT_POINT_POINT_DISTANCE: {
				} break;
				case CT_LINE_LINE_ANGLE: {
					assert(constraint->c1->type == COM_LINE);
					assert(constraint->c2->type == COM_LINE);

					struct point p1;
					struct point p2;
					if(!constraint->c1->drawn) {
						line_distance_to_point(constraint->c1->e->line, constraint->c1->min, &p1);
						line_distance_to_point(constraint->c1->e->line, constraint->c1->max, &p2);
						plot_line_between_style(p1, p2, LSTYLE_CONSTRUCTION);
						constraint->c1->drawn = true;
					}

					if(!constraint->c2->drawn) {
						line_distance_to_point(constraint->c2->e->line, constraint->c2->min, &p1);
						line_distance_to_point(constraint->c2->e->line, constraint->c2->max, &p2);
						plot_line_between_style(p1, p2, LSTYLE_CONSTRUCTION);
						constraint->c2->drawn = true;
					}
				} break;
				case CT_POINT_LINE_DISTANCE:
					break;
				case CT_END: abort();
			}
		}
	}

	// plot_line_between(components[0].e->point, components[3].e->point);
	// plot_line_between(components[3].e->point, components[1].e->point);
	// plot_line_between(line_bend_end->point, line_end->point);
	// plot_arc_between(line_ctr->point, line_bend_start->point, line_bend_end->point);
	// plot_line_between(detector_right->point, decomposer_left->point);
	// plot_line_between(decomposer_right->point, solver_left->point);
	// plot_line_between(solver_right->point, solutions_left->point);

	printf("</svg>\n");
}
