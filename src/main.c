#include <assert.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <cglm/cglm.h>
#include <string.h>

#include "cad/construction.h"

#define TEXT_OFFSET 0.4

#define SETSIGN(b, v) ((v) * ((2 * (b)) - 1))
#define SIGNOF(x) ((typeof(x))((x)>0) - ((x)<0))

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

	// @COML: There's something missing here about which side of the arc we
	// want. I think we can figure that out from the relation of the center and
	// the points
	vec2 t;
	glm_vec2_sub(c.pos, p1.pos, t);
	double r = glm_vec2_norm(t);
	printf("<path %s d=\"M %f %f A %f %f 0 0 0 %f %f\" fill=\"none\" />\n", style_str, p2.pos[0], -p2.pos[1], r, r, p1.pos[0], -p1.pos[1]);
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

enum component_type {
	COM_POINT,
	COM_LINE,
};

struct component {
	enum component_type type;

	struct element *e;

	double min;
	double max;
	bool min_max_init;
	bool drawn;
};

void extend_line_to(struct component* c, struct point *p) {
	assert(c->type == COM_LINE);

	double val = project_point_to_line_distance(*p, c->e->line);
	if(!c->min_max_init) {
		c->min_max_init = true;
		c->min = val;
		c->max = val;
	} else {
		c->min = fmin(c->min, val);
		c->max = fmax(c->max, val);
	}
}

enum constraint_type {
	CT_POINT_POINT_DISTANCE,
	CT_POINT_LINE_DISTANCE,
	CT_LINE_LINE_ANGLE,
};

char *constraint_type_name[] = {
	[CT_POINT_POINT_DISTANCE] = "Point Point Distance",
	[CT_POINT_LINE_DISTANCE] = "Point Line Distance",
	[CT_LINE_LINE_ANGLE] = "Line Line Angle",
};

#define SEARCH_DEPTH 16

struct path_step {
	uint64_t i;
	bool direction;
};

struct constraint {
	enum constraint_type type;
	double v;

	struct component *c1;
	struct component *c2;

	uint64_t order;
	struct path_step path[SEARCH_DEPTH];
	bool forward;
	bool used;
};

struct frontier {
	struct component *elems[32];
	size_t n;
};

bool frontier_scan(struct frontier *frontier, struct component *component) {
	for(size_t i = 0; i < sizeof(frontier->elems)/sizeof(frontier->elems[0]); i++) {
		if(frontier->elems[i] == component) return true;
	}

	return false;
}

void add_frontier(struct frontier *frontier, struct component *component) {
	assert(!frontier_scan(frontier, component));
	frontier->elems[frontier->n++] = component;
}

void build_angle_point_line(struct drawing *drawing, struct component *local_i, struct component *local_j, struct component *oppo_i) {
	assert(local_i->type == COM_LINE);
	assert(local_j->type == COM_POINT);
	struct element *theta = insert_cmd(drawing, (struct command){
		.op = CMD_VALUE_INPUT,
		.result.type = ETYPE_VALUE,
	});

	struct element *d = insert_cmd(drawing, (struct command){
		.op = CMD_VALUE_INPUT,
		.result.type = ETYPE_VALUE,
	});

	struct element* l = insert_cmd(drawing, (struct command){
		.op = CMD_LINE_POINT_LINE_ANGLE,
		.hidden = true,
		.result.type = ETYPE_LINE,
		.arg1 = local_j->e,
		.arg2 = local_i->e,
		.arg3 = theta,
	});

	oppo_i->e = insert_cmd(drawing, (struct command){
		.op = CMD_LINE_LINE_DISTANCE_PARALLEL,
		.hidden = true,
		.result.type = ETYPE_LINE,
		.arg1 = l,
		.arg2 = d,
	});
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

struct box {
	struct component corner[4];
	struct component side[4];
};

struct angle_search_frame {
	size_t constraint_i;
	struct component *head;
	bool dir;
};

bool find_angle(struct constraint *constraints, size_t constraints_num, struct component *first_component, struct component *needle, struct path_step *path) {
	struct angle_search_frame frames[SEARCH_DEPTH];
	struct angle_search_frame *frame = frames;

	frame->constraint_i = 0;
	frame->head = first_component;

	bool *checked = calloc(sizeof(bool), constraints_num);

	while(frame >= frames) {
		assert(frame < frames + SEARCH_DEPTH);
		assert(frame >= frames);

		assert(frame->constraint_i <= constraints_num);
		if(frame->constraint_i == constraints_num) {
#if 0
			printf("Dead end at: ");
			for(struct angle_search_frame *i = frames; i <= frame; i++) {
				printf("%ld -> ", i->constraint_i);
			}
			printf("\n");
#endif
			frame--;
			frame->constraint_i++;
			continue;
		}

		if(checked[frame->constraint_i]) {
			frame->constraint_i++;
			continue;
		}

		struct constraint *constraint = &constraints[frame->constraint_i];
		if(constraint->type != CT_LINE_LINE_ANGLE) {
			frame->constraint_i++;
			continue;
		}

		struct component *other;

		if(constraint->c1 == frame->head) {
			other = constraint->c2;
			frame->dir = true;
		} else if(constraint->c2 == frame->head) {
			other = constraint->c1;
			frame->dir = false;
		} else {
			frame->constraint_i++;
			continue;
		}

		if(other == needle) {
			// printf("Found path %p %p: ", first_component, needle);
			for(struct angle_search_frame *i = frames; i <= frame; i++) {
				path[i - frames].i = i->constraint_i;
				path[i - frames].direction = i->dir;
				// printf("%ld [%d] [%p] -> ", i->constraint_i, i->dir, i->head);
			}
			// printf("\n");
			if(frame < frames + SEARCH_DEPTH-1) {
				path[frame - frames + 1].i = -1;
			}
			return true;
		}

		checked[frame->constraint_i] = true;

		frame++;
		frame->constraint_i = 0;
		frame->head = other;
	}

	return false;
}

void solve_constraints(struct constraint *constraints, size_t constraints_num, struct drawing *drawing) {
	struct frontier frontier = {};
	uint64_t order = 1;

	for(size_t i = 0; i < constraints_num; i++) {
		constraints[i].path[0].i = -1;
		constraints[i].forward = true;
	}

	// Step 1 Pick some point point distance constraint as the base
	for(size_t i = 0; i < constraints_num; i++) {
		assert(!constraints[i].used);

		if(constraints[i].type != CT_POINT_POINT_DISTANCE) continue;

		constraints[i].c1->e = insert_cmd(drawing, (struct command){
			.op = CMD_ORIGIN,
			.hidden = true,
			.result.type = ETYPE_POINT,
		});

		struct element *xaxis = insert_cmd(drawing, (struct command){
			.op = CMD_LINE_X,
			.hidden = true,
			.result.type = ETYPE_LINE,
		});

		struct element *distance = insert_cmd(drawing, (struct command){
			.op = CMD_VALUE_INPUT,
			.result.type = ETYPE_VALUE,
		});

		struct element *c = insert_cmd(drawing, (struct command){
			.op = CMD_CIRCLE_CENTER_RADIUS,
			.hidden = true,
			.result.type = ETYPE_CIRCLE,
			.arg1 = constraints[i].c1->e,
			.arg2 = distance,
		});

		constraints[i].c2->e = insert_cmd(drawing, (struct command){
			.op = CMD_POINT_CIRCLE_LINE,
			.hidden = true,
			.result.type = ETYPE_POINT,
			.arg1 = c,
			.arg2 = xaxis,
		});

		constraints[i].used = true;
		constraints[i].order = order++;
		add_frontier(&frontier, constraints[i].c1);
		add_frontier(&frontier, constraints[i].c2);
		break;
	}

	// Step 2 we iteratively expand the frontier from the selected base
	while(true) {
		for(size_t i = 0; i < constraints_num; i++) {
			if(constraints[i].used) continue;

			struct component *oppo_i;
			struct component *local_i;

			if(frontier_scan(&frontier, constraints[i].c1)) {
				oppo_i = constraints[i].c2;
				local_i = constraints[i].c1;
			} else if(frontier_scan(&frontier, constraints[i].c2)) {
				oppo_i = constraints[i].c1;
				local_i = constraints[i].c2;
			} else continue;

			for(size_t j = i+1; j < constraints_num; j++) {
				if(constraints[j].used) continue;

				struct component *oppo_j;
				struct component *local_j;

				if(frontier_scan(&frontier, constraints[j].c1)) {
					oppo_j = constraints[j].c2;
					local_j = constraints[j].c1;
				} else if(frontier_scan(&frontier, constraints[j].c2)) {
					oppo_j = constraints[j].c1;
					local_j = constraints[j].c2;
				} else continue;

				// One of the constraints can't be an angle one
				if(constraints[i].type == CT_LINE_LINE_ANGLE && constraints[j].type == CT_LINE_LINE_ANGLE) continue;

				if(oppo_i != oppo_j) {
					if(constraints[i].type == CT_LINE_LINE_ANGLE) {
						if(!find_angle(constraints, constraints_num, oppo_i, oppo_j, constraints[i].path)) continue;

						oppo_i = oppo_j;
					} else if(constraints[j].type == CT_LINE_LINE_ANGLE) {
						if(!find_angle(constraints, constraints_num, oppo_j, oppo_i, constraints[j].path)) continue;

						oppo_j = oppo_i;
					} else continue;
				}


				// printf("Detected %ld %ld\n", i, j);

				if(constraints[i].type == CT_POINT_POINT_DISTANCE
					&& local_i->type == COM_POINT
					&& constraints[j].type == CT_POINT_POINT_DISTANCE
					&& local_j->type == COM_POINT) {
					assert(oppo_i->type == COM_POINT);

					struct element *d1 = insert_cmd(drawing, (struct command){
						.op = CMD_VALUE_INPUT,
						.result.type = ETYPE_VALUE,
					});

					struct element *d2 = insert_cmd(drawing, (struct command){
						.op = CMD_VALUE_INPUT,
						.result.type = ETYPE_VALUE,
					});

					struct element *c1 = insert_cmd(drawing, (struct command){
						.op = CMD_CIRCLE_CENTER_RADIUS,
						.hidden = true,
						.result.type = ETYPE_CIRCLE,
						.arg1 = local_i->e,
						.arg2 = d1,
					});

					struct element *c2 = insert_cmd(drawing, (struct command){
						.op = CMD_CIRCLE_CENTER_RADIUS,
						.hidden = true,
						.result.type = ETYPE_CIRCLE,
						.arg1 = local_j->e,
						.arg2 = d2,
					});

					oppo_i->e = insert_cmd(drawing, (struct command){
						.op = CMD_POINT_CIRCLE_CIRCLE,
						.hidden = true,
						.result.type = ETYPE_POINT,
						.arg1 = c1,
						.arg2 = c2,
					});
				} else if(constraints[i].type == CT_POINT_LINE_DISTANCE
					&& local_i->type == COM_POINT
					&& constraints[j].type == CT_POINT_LINE_DISTANCE
					&& local_j->type == COM_POINT) {
					assert(oppo_i->type == COM_LINE);

					struct element *d1 = insert_cmd(drawing, (struct command){
						.op = CMD_VALUE_INPUT,
						.result.type = ETYPE_VALUE,
					});
					struct element *d2 = insert_cmd(drawing, (struct command){
						.op = CMD_VALUE_INPUT,
						.result.type = ETYPE_VALUE,
					});

					struct element *c1 = insert_cmd(drawing, (struct command){
						.op = CMD_CIRCLE_CENTER_RADIUS,
						.hidden = true,
						.result.type = ETYPE_CIRCLE,
						.arg1 = local_i->e,
						.arg2 = d1,
					});

					struct element *c2 = insert_cmd(drawing, (struct command){
						.op = CMD_CIRCLE_CENTER_RADIUS,
						.hidden = true,
						.result.type = ETYPE_CIRCLE,
						.arg1 = local_j->e,
						.arg2 = d2,
					});

					oppo_i->e = insert_cmd(drawing, (struct command){
						.op = CMD_LINE_CIRCLE_CIRCLE_TANGENT,
						.hidden = true,
						.result.type = ETYPE_LINE,
						.arg1 = c1,
						.arg2 = c2,
					});
				} else if(constraints[i].type == CT_POINT_LINE_DISTANCE
					&& local_i->type == COM_LINE
					&& constraints[j].type == CT_POINT_LINE_DISTANCE
					&& local_j->type == COM_LINE) {
					assert(oppo_i->type == COM_POINT);

					struct element *d1 = insert_cmd(drawing, (struct command){
						.op = CMD_VALUE_INPUT,
						.result.type = ETYPE_VALUE,
					});
					struct element *d2 = insert_cmd(drawing, (struct command){
						.op = CMD_VALUE_INPUT,
						.result.type = ETYPE_VALUE,
					});

					struct element *l1 = insert_cmd(drawing, (struct command){
						.op = CMD_LINE_LINE_DISTANCE_PARALLEL,
						.hidden = true,
						.result.type = ETYPE_LINE,
						.arg1 = local_i->e,
						.arg2 = d1,
					});

					struct element *l2 = insert_cmd(drawing, (struct command){
						.op = CMD_LINE_LINE_DISTANCE_PARALLEL,
						.hidden = true,
						.result.type = ETYPE_LINE,
						.arg1 = local_j->e,
						.arg2 = d2,
					});

					oppo_i->e = insert_cmd(drawing, (struct command){
						.op = CMD_POINT_LINE_LINE,
						.hidden = true,
						.result.type = ETYPE_POINT,
						.arg1 = l1,
						.arg2 = l2,
					});
				} else if(constraints[i].type == CT_LINE_LINE_ANGLE
					&& local_i->type == COM_LINE
					&& constraints[j].type == CT_POINT_LINE_DISTANCE
					&& local_j->type == COM_POINT) {
					assert(oppo_i->type == COM_LINE);

					constraints[i].forward = constraints[i].c1 == local_i;
					build_angle_point_line(drawing, local_i, local_j, oppo_i);
				} else if(constraints[i].type == CT_POINT_LINE_DISTANCE
					&& local_i->type == COM_POINT
					&& constraints[j].type == CT_LINE_LINE_ANGLE
					&& local_j->type == COM_LINE) {
					assert(oppo_i->type == COM_LINE);
					constraints[j].forward = constraints[j].c1 == local_j;

					// @HACK Swap the two constraints to reuse the construction
					// steps. This sucks, and we need to figure out some better
					// way of doing it.
					struct component* tmp = local_i;
					local_i = local_j;
					local_j = tmp;

					i ^= j;
					j = j ^ i;
					i ^= j;

					build_angle_point_line(drawing, local_i, local_j, oppo_i);
				} else if(constraints[i].type == CT_POINT_LINE_DISTANCE
					&& local_i->type == COM_LINE
					&& constraints[j].type == CT_POINT_POINT_DISTANCE
					&& local_j->type == COM_POINT) {
					assert(oppo_i->type == COM_POINT);

					struct element *d1 = insert_cmd(drawing, (struct command){
						.op = CMD_VALUE_INPUT,
						.result.type = ETYPE_VALUE,
					});
					struct element *d2 = insert_cmd(drawing, (struct command){
						.op = CMD_VALUE_INPUT,
						.result.type = ETYPE_VALUE,
					});

					struct element *l = insert_cmd(drawing, (struct command){
						.op = CMD_LINE_LINE_DISTANCE_PARALLEL,
						.hidden = true,
						.result.type = ETYPE_LINE,
						.arg1 = local_i->e,
						.arg2 = d1,
					});

					struct element *c = insert_cmd(drawing, (struct command){
						.op = CMD_CIRCLE_CENTER_RADIUS,
						.hidden = true,
						.result.type = ETYPE_CIRCLE,
						.arg1 = local_j->e,
						.arg2 = d2,
					});

					oppo_i->e = insert_cmd(drawing, (struct command){
						.op = CMD_POINT_CIRCLE_LINE,
						.hidden = true,
						.result.type = ETYPE_POINT,
						.arg1 = c,
						.arg2 = l,
					});
				} else {
					printf("Unknown constraint combination %s and %s\n", constraint_type_name[constraints[i].type], constraint_type_name[constraints[j].type]);
					abort();
				}

				add_frontier(&frontier, oppo_i);
				constraints[i].used = true;
				constraints[i].order = order++;
				constraints[j].used = true;
				constraints[j].order = order++;
				goto candidate_found;
			}
		}
		// No candidate found
		break;

candidate_found:
		;
	}

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

	// Figure 4
	struct component components[] = {
		{.type = COM_POINT},
		{.type = COM_POINT},
		{.type = COM_POINT},
		{.type = COM_LINE},
		{.type = COM_LINE},
		{.type = COM_POINT},
	};

	struct smooth_line line = {
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

	struct box box = {
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

	struct constraint constraints[] = {
		{
			.type = CT_POINT_POINT_DISTANCE,
			.v = 13,
			.c1 = &components[0],
			.c2 = &components[1],
		},
		{
			.type = CT_POINT_POINT_DISTANCE,
			.v = 10,
			.c1 = &components[1],
			.c2 = &components[2],
		},
		{
			.type = CT_POINT_POINT_DISTANCE,
			.v = 10,
			.c1 = &components[2],
			.c2 = &components[0],
		},
		{
			.type = CT_POINT_LINE_DISTANCE,
			.v = 0,
			.c1 = &components[0],
			.c2 = &components[3],
		},
		{
			.type = CT_POINT_LINE_DISTANCE,
			.v = 0,
			.c1 = &components[1],
			.c2 = &components[3],
		},
		{
			.type = CT_LINE_LINE_ANGLE,
			.v = M_PI/1.7,
			.c1 = &components[3],
			.c2 = &components[4],
		},
		{
			.type = CT_POINT_LINE_DISTANCE,
			.v = 0,
			.c1 = &components[4],
			.c2 = &components[1],
		},
		{
			.type = CT_POINT_LINE_DISTANCE,
			.v = 0,
			.c1 = &components[4],
			.c2 = &components[5],
		},
		{
			.type = CT_POINT_POINT_DISTANCE,
			.v = 12.8,
			.c1 = &components[0],
			.c2 = &components[5],
		},

		{
			.type = CT_LINE_LINE_ANGLE,
			.v = M_PI/2,
			.c1 = &components[3],
			.c2 = &line.l1,
		},
		{
			.type = CT_POINT_LINE_DISTANCE,
			.v = 0,
			.c1 = &components[5],
			.c2 = &line.l1,
		},

		{
			.type = CT_LINE_LINE_ANGLE,
			.v = M_PI/2,
			.c1 = &line.l2,
			.c2 = &line.l1,
		},
		{
			.type = CT_POINT_LINE_DISTANCE,
			.v = 0,
			.c1 = &line.corner,
			.c2 = &line.l2,
		},

		{
			.type = CT_POINT_LINE_DISTANCE,
			.v = 0,
			.c1 = &line.corner,
			.c2 = &line.l1,
		},
		{
			.type = CT_POINT_LINE_DISTANCE,
			.v = 0,
			.c1 = &line.end,
			.c2 = &line.l2,
		},

		{
			.type = CT_POINT_LINE_DISTANCE,
			.v = 10,
			.c1 = &line.end,
			.c2 = &line.l1,
		},
		{
			.type = CT_POINT_LINE_DISTANCE,
			.v = 12,
			.c1 = &line.end,
			.c2 = &components[3],
		},

		{
			.type = CT_POINT_LINE_DISTANCE,
			.v = 1,
			.c1 = &line.corner_center,
			.c2 = &line.l1,
		},
		{
			.type = CT_POINT_LINE_DISTANCE,
			.v = -1,
			.c1 = &line.corner_center,
			.c2 = &line.l2,
		},

		{
			.type = CT_LINE_LINE_ANGLE,
			.v = M_PI/2,
			.c1 = &line.l1,
			.c2 = &line.perp1,
		},
		{
			.type = CT_POINT_LINE_DISTANCE,
			.v = 0,
			.c1 = &line.corner_center,
			.c2 = &line.perp1,
		},

		{
			.type = CT_POINT_LINE_DISTANCE,
			.v = 0,
			.c1 = &line.corner_start,
			.c2 = &line.perp1,
		},
		{
			.type = CT_POINT_LINE_DISTANCE,
			.v = 0,
			.c1 = &line.corner_start,
			.c2 = &line.l1,
		},

		{
			.type = CT_LINE_LINE_ANGLE,
			.v = M_PI/2,
			.c1 = &line.l2,
			.c2 = &line.perp2,
		},
		{
			.type = CT_POINT_LINE_DISTANCE,
			.v = 0,
			.c1 = &line.corner_center,
			.c2 = &line.perp2,
		},

		{
			.type = CT_POINT_LINE_DISTANCE,
			.v = 0,
			.c1 = &line.corner_end,
			.c2 = &line.perp2,
		},
		{
			.type = CT_POINT_LINE_DISTANCE,
			.v = 0,
			.c1 = &line.corner_end,
			.c2 = &line.l2,
		},

		{
			.type = CT_POINT_LINE_DISTANCE,
			.v = 0,
			.c1 = &box.corner[0],
			.c2 = &box.side[0],
		},
		{
			.type = CT_POINT_LINE_DISTANCE,
			.v = 0,
			.c1 = &box.corner[1],
			.c2 = &box.side[0],
		},

		{
			.type = CT_POINT_LINE_DISTANCE,
			.v = 0,
			.c1 = &box.corner[1],
			.c2 = &box.side[1],
		},
		{
			.type = CT_POINT_LINE_DISTANCE,
			.v = 0,
			.c1 = &box.corner[2],
			.c2 = &box.side[1],
		},

		{
			.type = CT_POINT_LINE_DISTANCE,
			.v = 0,
			.c1 = &box.corner[2],
			.c2 = &box.side[2],
		},
		{
			.type = CT_POINT_LINE_DISTANCE,
			.v = 0,
			.c1 = &box.corner[3],
			.c2 = &box.side[2],
		},

		{
			.type = CT_POINT_LINE_DISTANCE,
			.v = 0,
			.c1 = &box.corner[3],
			.c2 = &box.side[3],
		},
		{
			.type = CT_POINT_LINE_DISTANCE,
			.v = 0,
			.c1 = &box.corner[0],
			.c2 = &box.side[3],
		},

		{
			.type = CT_LINE_LINE_ANGLE,
			.v = M_PI/2,
			.c1 = &box.side[3],
			.c2 = &box.side[0],
		},
		{
			.type = CT_LINE_LINE_ANGLE,
			.v = M_PI/2,
			.c1 = &box.side[1],
			.c2 = &box.side[2],
		},
		{
			.type = CT_LINE_LINE_ANGLE,
			.v = M_PI/2,
			.c1 = &box.side[2],
			.c2 = &box.side[3],
		},
		{
			.type = CT_LINE_LINE_ANGLE,
			.v = M_PI*1/2,
			.c1 = &components[3],
			.c2 = &box.side[1],
		},

		{
			.type = CT_POINT_POINT_DISTANCE,
			.v = 10,
			.c1 = &line.corner_start,
			.c2 = &box.corner[0],
		},
		{
			.type = CT_POINT_POINT_DISTANCE,
			.v = 10,
			.c1 = &line.corner_end,
			.c2 = &box.corner[0],
		},

		{
			.type = CT_POINT_POINT_DISTANCE,
			.v = 20,
			.c1 = &box.corner[0],
			.c2 = &box.corner[1],
		},
		{
			.type = CT_POINT_POINT_DISTANCE,
			.v = 10,
			.c1 = &box.corner[1],
			.c2 = &box.corner[2],
		},
	};

	struct drawing drawing = {};
	solve_constraints(constraints, sizeof(constraints)/sizeof(constraints[0]), &drawing);

	double *params = malloc(sizeof(double) * (sizeof(constraints)/sizeof(constraints[0])));
	for(size_t i = 0; i < sizeof(constraints)/sizeof(constraints[0]); i++) {
		if(!constraints[i].used) continue;

		params[constraints[i].order-1] = SETSIGN(constraints[i].forward, constraints[i].v);

		// We need to offset angles based on the path we took to use this
		// constraint
		if(constraints[i].path[0].i != -1) {
			// printf("PATH %ld\n", i);
			for(struct path_step *p = constraints[i].path; p <= constraints[i].path+SEARCH_DEPTH && p->i != -1; p++) {
				params[constraints[i].order-1] += SETSIGN(p->direction, constraints[p->i].v);
				// printf("%ld [%d:%f] [%f] -> ", p->i, p->direction, constraints[p->i].v, params[constraints[i].order-1]);
			}
			// printf("\n");
			params[constraints[i].order-1] -= (M_PI*2.0) * floor(params[constraints[i].order-1] / (M_PI*2.0));
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
	}

	// plot_line_between(components[0].e->point, components[1].e->point);
	plot_line_between(components[1].e->point, components[2].e->point);
	plot_line_between(components[2].e->point, components[0].e->point);
	plot_line_between(components[1].e->point, components[5].e->point);
	plot_line_between(components[5].e->point, components[0].e->point);

	// plot_line_between(line.corner_start.e->point, box.corner[0].e->point);
	// printf("%f %f %f\n", box.side[0].e->line.norm[0], box.side[0].e->line.norm[1], box.side[0].e->line.C);
	// printf("%f %f %f\n", components[3].e->line.norm[0], components[3].e->line.norm[1], components[3].e->line.C);
	// plot_generic(*box.side[0].e);
	plot_line_between(box.corner[0].e->point, box.corner[1].e->point);
	plot_line_between(box.corner[1].e->point, box.corner[2].e->point);
	plot_line_between(box.corner[2].e->point, box.corner[3].e->point);
	plot_line_between(box.corner[3].e->point, box.corner[0].e->point);

	plot_line_between(components[5].e->point, line.corner_start.e->point);
	plot_line_between(line.corner_end.e->point, line.end.e->point);
	// plot_generic(*bend[0].e);
	// plot_generic(*bend[1].e);
	// plot_generic(*bend[2].e);
	// plot_generic(*bend[3].e);
	// plot_generic(*bend[4].e);
	plot_arc_between(line.corner_center.e->point, line.corner_end.e->point, line.corner_start.e->point);
	// plot_generic(*bend[8].e);
	// plot_generic(*box.corner[0].e);
	// plot_line_style(box.side[0].e->line, LSTYLE_NORMAL);
	// plot_line_style(box.side[1].e->line, LSTYLE_NORMAL);
	// plot_line_style(box.side[2].e->line, LSTYLE_NORMAL);
	// plot_generic(*line.perp2.e);
	// plot_generic(*line.l1.e);
	// plot_generic(*components[3].e);
	// plot_generic(*line.l2.e);
	// plot_generic(*line.corner_center.e);
	// plot_angle(components[3].e->line, line.l1.e->line, 90);
	// plot_angle(line.l2.e->line, line.perp2.e->line, -90);

	if(true) {
		for(size_t i = 0; i < sizeof(constraints)/sizeof(constraints[0]); i++) {
			struct constraint *constraint = &constraints[i];
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
			}
		}

		for(size_t i = 0; i < sizeof(constraints)/sizeof(constraints[0]); i++) {
			struct constraint *constraint = &constraints[i];
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
					struct point p1;
					struct point p2;
					struct point intersect;
					plot_angle(constraint->c1->e->line, constraint->c2->e->line, constraint->v, &intersect, &p1, &p2);

					extend_line_to(constraint->c1, &intersect);
					extend_line_to(constraint->c2, &intersect);
					extend_line_to(constraint->c1, &p1);
					extend_line_to(constraint->c2, &p2);
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

					if(constraint->v == 0) {
						extend_line_to(line, &point->e->point);
					}
				} break;
			}
		}

		for(size_t i = 0; i < sizeof(constraints)/sizeof(constraints[0]); i++) {
			struct constraint *constraint = &constraints[i];
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
			}
		}
	}

	printf("Component addr %p", &box.side[1]);

	// plot_line_between(components[0].e->point, components[3].e->point);
	// plot_line_between(components[3].e->point, components[1].e->point);
	// plot_line_between(line_bend_end->point, line_end->point);
	// plot_arc_between(line_ctr->point, line_bend_start->point, line_bend_end->point);
	// plot_line_between(detector_right->point, decomposer_left->point);
	// plot_line_between(decomposer_right->point, solver_left->point);
	// plot_line_between(solver_right->point, solutions_left->point);

	printf("</svg>\n");
}
