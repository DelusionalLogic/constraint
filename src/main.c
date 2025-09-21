#include <assert.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <cglm/cglm.h>
#include <string.h>

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

	struct element *arg1;
	struct element *arg2;
	struct element *arg3;

	struct element result;

	struct command *next;
};

struct drawing {
	struct command *root;
	struct command *tail;
};

void circle_line_intersect(struct circle circle, struct line line, uint8_t root, struct point *point) {
	double a = line.norm[0];
	double b = line.norm[1];
	double c = line.C + glm_vec2_dot(line.norm, circle.center);

	double rec = pow(a, 2) + pow(b, 2);
	double x0 = -a*c / rec;
	double y0 = -b*c / rec;

	double r2 = pow(circle.radius, 2); 
	double test = r2 * rec; 
	if(pow(c, 2.0) - test > DBL_EPSILON * 1e14) {
		printf("Expected an intersection between circle and line\n");
		assert(false);
	} else if(fabs(pow(c, 2) - test) < DBL_EPSILON * 1e14) {
		point->pos[0] = x0;
		point->pos[1] = y0;
	} else {
		double d = r2 - pow(c, 2)/rec;
		double mult = sqrt(d / rec);

		if(root == 0) {
			point->pos[0] = x0 + b * mult;
			point->pos[1] = y0 - a * mult;
		} else {
			point->pos[0] = x0 - b * mult;
			point->pos[1] = y0 + a * mult;
		}
	}

	glm_vec2_add(point->pos, circle.center, point->pos);
}

void line_through_points(struct point p1, struct point p2, struct line* l) {
	l->norm[0] = p1.pos[1] - p1.pos[1];
	l->norm[1] = p2.pos[0] - p1.pos[0];
	l->C = p1.pos[1] * -l->norm[1] + p1.pos[0] * -l->norm[1];
}

void plot_point(struct point p) {
	printf("<circle cx=\"%f\" cy=\"%f\" r=\".4\" fill=\"black\" />\n", p.pos[0], -p.pos[1]);
}

void plot_line_between(struct point p1, struct point p2) {
	printf("<line x1=\"%f\" y1=\"%f\" x2=\"%f\" y2=\"%f\" stroke=\"black\" stroke-width=\".2\" />\n", p1.pos[0], -p1.pos[1], p2.pos[0], -p2.pos[1]);
}

void plot_line(struct line l) {
	// @HACK I've just duplicated this for vertical lines. There's a more
	// efficient solution
	if(l.norm[1] != 0) {
		double minx = -50;
		double maxx =  50;

		double miny = -(l.norm[0] * minx + l.C) / l.norm[1];
		double maxy = -(l.norm[0] * maxx + l.C) / l.norm[1];
		printf("<line x1=\"%f\" y1=\"%f\" x2=\"%f\" y2=\"%f\" stroke=\"black\" stroke-width=\".2\" />\n", minx, -miny, maxx, -maxy);
	} else {
		double miny = -50;
		double maxy =  50;

		double minx = -(l.norm[1] * miny + l.C) / l.norm[0];
		double maxx = -(l.norm[1] * maxy + l.C) / l.norm[0];
		printf("<line x1=\"%f\" y1=\"%f\" x2=\"%f\" y2=\"%f\" stroke=\"black\" stroke-width=\".2\" />\n", minx, -miny, maxx, -maxy);
	}

	double d0 = glm_vec2_norm2(l.norm);

	vec2 p0 = {0, 0};
	glm_vec2_mulsubs(l.norm, l.C, p0);
	glm_vec2_divs(p0, d0, p0);

	vec2 p1;
	glm_vec2_add(p0, l.norm, p1);
	// printf("%f %f\n", l.norm[0], l.norm[1]);

	printf("<line x1=\"%f\" y1=\"%f\" x2=\"%f\" y2=\"%f\" stroke=\"black\" stroke-width=\".2\" />\n", p0[0], -p0[1], p1[0], -p1[1]);
}

void plot_arc_between(struct point c, struct point p1, struct point p2) {
	// @COML: There's something missing here about which side of the arc we
	// want. I think we can figure that out from the relation of the center and
	// the points
	vec2 t;
	glm_vec2_sub(c.pos, p1.pos, t);
	double r = glm_vec2_norm(t);
	printf("<path d=\"M %f %f A %f %f 0 0 0 %f %f\" stroke=\"black\" stroke-width=\".2\" fill=\"none\" />\n", p2.pos[0], -p2.pos[1], r, r, p1.pos[0], -p1.pos[1]);
}

void plot_circle(struct circle c) {
	printf("<circle cx=\"%f\" cy=\"%f\" r=\"%f\" fill=\"none\" stroke=\"black\" stroke-width=\".1\" />\n", c.center[0], -c.center[1], c.radius);
}

struct element* insert_cmd(struct drawing *drawing, struct command cmd) {
	struct command *new = calloc(sizeof(struct command), 1);
	memcpy(new, &cmd, sizeof(struct command));
	assert(new != NULL);

	if(drawing->tail != NULL) drawing->tail->next = new;
	else drawing->root = new;
	drawing->tail = new;

	return &new->result;
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

void execute_drawing(struct drawing *drawing, double inputs[]) {
	size_t input_i = 0;
	for(struct command *current = drawing->root; current != NULL; current = current->next) {
		switch(current->op) {
			case CMD_VALUE_INPUT: {
				assert(current->result.type == ETYPE_VALUE);
				current->result.value = inputs[input_i++];
			}break;
			case CMD_ORIGIN: {
				assert(current->result.type == ETYPE_POINT);
				glm_vec2_zero(current->result.point.pos);
			}break;
			case CMD_LINE_X: {
				assert(current->result.type == ETYPE_LINE);
				current->result.line.norm[0] = 0;
				current->result.line.norm[1] = 1;
				current->result.line.C = 0;
			}break;
			case CMD_CIRCLE_CENTER_RADIUS: {
				assert(current->result.type == ETYPE_CIRCLE);
				glm_vec2_copy(current->arg1->point.pos, current->result.circle.center);
				current->result.circle.radius = current->arg2->value;
			}break;
			case CMD_LINE_POINT_POINT: {
				assert(current->result.type == ETYPE_LINE);
				line_through_points(current->arg1->point, current->arg2->point, &current->result.line);
			}break;
			case CMD_LINE_POINT_LINE_ANGLE: {
				assert(current->arg1->type == ETYPE_POINT);
				assert(current->arg2->type == ETYPE_LINE);
				assert(current->arg3->type == ETYPE_VALUE);
				assert(current->result.type == ETYPE_LINE);

				// printf("Rotate %f\n", current->arg3->value);
				// printf("%f %f\n", current->arg2->line.norm[0], current->arg2->line.norm[1]);
				glm_vec2_rotate(current->arg2->line.norm, current->arg3->value, current->result.line.norm);
				// printf("%f %f\n", current->result.line.norm[0], current->result.line.norm[1]);

				vec2 offset = {-current->arg1->point.pos[0], -current->arg1->point.pos[1]};
				current->result.line.C = glm_vec2_dot(current->result.line.norm, offset);
			}break;
			case CMD_LINE_LINE_DISTANCE_PARALLEL: {
				assert(current->arg1->type == ETYPE_LINE);
				assert(current->arg2->type == ETYPE_VALUE);
				assert(current->result.type == ETYPE_LINE);

				glm_vec2_copy(current->arg1->line.norm, current->result.line.norm);
				double mag = glm_vec2_norm(current->result.line.norm);
				if(current->root == 1) mag = -mag;
				current->result.line.C = current->arg1->line.C - mag * current->arg2->value;
			}break;
			case CMD_LINE_CIRCLE_CIRCLE_TANGENT: {
				assert(current->arg1->type == ETYPE_CIRCLE);
				assert(current->arg2->type == ETYPE_CIRCLE);
				assert(current->result.type == ETYPE_LINE);

				if(current->arg1->circle.radius == 0 && current->arg2->circle.radius == 0) {
					// The tangent is just a line through the two centers
					// @FAST: This is wasteful. If the the procedure too the
					// two vectors directly, we wouldn't have to copy here.
					struct point p1;
					glm_vec2_copy(current->arg1->circle.center, p1.pos);
					struct point p2;
					glm_vec2_copy(current->arg2->circle.center, p2.pos);
					// printf("%f %f\n", current->arg1->circle.center[0], current->arg1->circle.center[1]);
					// printf("%f %f\n", current->arg2->circle.center[0], current->arg2->circle.center[1]);
					line_through_points(p1, p2, &current->result.line);
				} else if (fabs(current->arg1->circle.radius - current->arg2->circle.radius) < DBL_EPSILON * 1e14) {
					// The tangent is parallel to the line through the two
					// centers
					abort();
				} else {
					// We should probably implement tangents for circles that
					// don't happen to be 0 radius
					abort();
				}
			}break;
			case CMD_POINT_CIRCLE_LINE: {
				assert(current->arg1->type == ETYPE_CIRCLE);
				assert(current->arg2->type == ETYPE_LINE);
				assert(current->result.type == ETYPE_POINT);
				struct line translated;
				glm_vec2_copy(current->arg2->line.norm, translated.norm);
				translated.C = translated.C - glm_vec2_dot(translated.norm, current->arg1->circle.center);
				circle_line_intersect(current->arg1->circle, current->arg2->line, current->root, &current->result.point);
			}break;
			case CMD_POINT_CIRCLE_CIRCLE: {
				assert(current->result.type == ETYPE_POINT);
				vec2 between_centers;
				glm_vec2_sub(current->arg1->circle.center, current->arg2->circle.center, between_centers);
				glm_vec2_mul(between_centers, (vec2){2, 2}, between_centers);
				double a = between_centers[0];
				double b = between_centers[1];

				double c = (pow(current->arg2->circle.center[0], 2) - pow(current->arg1->circle.center[0], 2)) + \
						(pow(current->arg2->circle.center[1], 2) - pow(current->arg1->circle.center[1], 2)) - \
						(pow(current->arg2->circle.radius, 2) - pow(current->arg1->circle.radius, 2));

				struct line radical_axis = { {a, b}, c };
				circle_line_intersect(current->arg1->circle, radical_axis, current->root, &current->result.point);
			}break;
			case CMD_POINT_LINE_LINE: {
				assert(current->result.type == ETYPE_POINT);
				double a1 = current->arg1->line.norm[0];
				double b1 = current->arg1->line.norm[1];
				double c1 = current->arg1->line.C;

				double a2 = current->arg2->line.norm[0];
				double b2 = current->arg2->line.norm[1];
				double c2 = current->arg2->line.C;

				current->result.point.pos[0] = -(c1*b2 - c2*b1) / (a1*b2 - a2*b1);
				current->result.point.pos[1] = -(a1*c2 - a2*c1) / (a1*b2 - a2*b1);
			}break;
			case CMD_CIRCLE_CENTER_POINT: {
				assert(current->result.type == ETYPE_CIRCLE);
				glm_vec2_copy(current->arg1->point.pos, current->result.circle.center);
				vec2 imm;
				glm_vec2_sub(current->arg1->point.pos, current->arg2->point.pos, imm);
				current->result.circle.radius = glm_vec2_norm(imm);
			}break;

		}
		// printf("%fx + %fy + %f = 0\n", cmd[2].line.norm[0], cmd[2].line.norm[1], cmd[2].line.C);
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
};

enum constraint_type {
	CT_POINT_POINT_DISTANCE,
	CT_POINT_LINE_DISTANCE,
	CT_LINE_LINE_ANGLE,
};

struct constraint {
	enum constraint_type type;
	double v;

	struct component *c1;
	struct component *c2;

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

	struct constraint constraints[] = {
		{
			.type = CT_POINT_POINT_DISTANCE,
			.v = 10,
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
			.c2 = &components[0],
			.c1 = &components[3],
		},
		{
			.type = CT_POINT_LINE_DISTANCE,
			.v = 0,
			.c1 = &components[1],
			.c2 = &components[3],
		},
		{
			.type = CT_LINE_LINE_ANGLE,
			.v = 30,
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
			.c2 = &components[4],
			.c1 = &components[5],
		},
		{
			.type = CT_POINT_POINT_DISTANCE,
			.v = 20,
			.c2 = &components[5],
			.c1 = &components[0],
		},
	};

	struct frontier frontier = {};
	struct drawing drawing = {};

	// Step 1 Pick some point point distance constraint as the base
	for(size_t i = 0; i < sizeof(constraints)/sizeof(constraints[0]); i++) {
		assert(!constraints[i].used);

		if(constraints[i].type != CT_POINT_POINT_DISTANCE) continue;

		constraints[i].c1->e = insert_cmd(&drawing, (struct command){
			.op = CMD_ORIGIN,
			.hidden = true,
			.result.type = ETYPE_POINT,
		});

		struct element *xaxis = insert_cmd(&drawing, (struct command){
			.op = CMD_LINE_X,
			.hidden = true,
			.result.type = ETYPE_LINE,
		});

		struct element *distance = insert_cmd(&drawing, (struct command){
			.op = CMD_VALUE_INPUT,
			.result.type = ETYPE_VALUE,
		});

		struct element *c = insert_cmd(&drawing, (struct command){
			.op = CMD_CIRCLE_CENTER_RADIUS,
			.hidden = true,
			.result.type = ETYPE_CIRCLE,
			.arg1 = constraints[i].c1->e,
			.arg2 = distance,
		});

		constraints[i].c2->e = insert_cmd(&drawing, (struct command){
			.op = CMD_POINT_CIRCLE_LINE,
			.hidden = true,
			.result.type = ETYPE_POINT,
			.arg1 = c,
			.arg2 = xaxis,
		});

		add_frontier(&frontier, constraints[i].c1);
		add_frontier(&frontier, constraints[i].c2);
		break;
	}

	// Step 2 we iteratively expand the frontier from the selected base
	while(true) {
		for(size_t i = 0; i < sizeof(constraints)/sizeof(constraints[0]); i++) {
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

			for(size_t j = i+1; j < sizeof(constraints)/sizeof(constraints[0]); j++) {
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

				if(oppo_i != oppo_j) continue;

				// printf("Detected %ld %ld\n", i, j);

				if(constraints[i].type == CT_POINT_POINT_DISTANCE
					&& local_i->type == COM_POINT
					&& constraints[j].type == CT_POINT_POINT_DISTANCE
					&& local_j->type == COM_POINT) {
					assert(oppo_i->type == COM_POINT);

					struct element *d1 = insert_cmd(&drawing, (struct command){
						.op = CMD_VALUE_INPUT,
						.result.type = ETYPE_VALUE,
					});

					struct element *d2 = insert_cmd(&drawing, (struct command){
						.op = CMD_VALUE_INPUT,
						.result.type = ETYPE_VALUE,
					});

					struct element *c1 = insert_cmd(&drawing, (struct command){
						.op = CMD_CIRCLE_CENTER_RADIUS,
						.hidden = true,
						.result.type = ETYPE_CIRCLE,
						.arg1 = local_i->e,
						.arg2 = d1,
					});

					struct element *c2 = insert_cmd(&drawing, (struct command){
						.op = CMD_CIRCLE_CENTER_RADIUS,
						.hidden = true,
						.result.type = ETYPE_CIRCLE,
						.arg1 = local_j->e,
						.arg2 = d2,
					});

					oppo_i->e = insert_cmd(&drawing, (struct command){
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

					struct element *d1 = insert_cmd(&drawing, (struct command){
						.op = CMD_VALUE_INPUT,
						.result.type = ETYPE_VALUE,
					});
					struct element *d2 = insert_cmd(&drawing, (struct command){
						.op = CMD_VALUE_INPUT,
						.result.type = ETYPE_VALUE,
					});

					struct element *c1 = insert_cmd(&drawing, (struct command){
						.op = CMD_CIRCLE_CENTER_RADIUS,
						.hidden = true,
						.result.type = ETYPE_CIRCLE,
						.arg1 = local_i->e,
						.arg2 = d1,
					});

					struct element *c2 = insert_cmd(&drawing, (struct command){
						.op = CMD_CIRCLE_CENTER_RADIUS,
						.hidden = true,
						.result.type = ETYPE_CIRCLE,
						.arg1 = local_j->e,
						.arg2 = d2,
					});

					oppo_i->e = insert_cmd(&drawing, (struct command){
						.op = CMD_LINE_CIRCLE_CIRCLE_TANGENT,
						.hidden = true,
						.result.type = ETYPE_LINE,
						.arg1 = c1,
						.arg2 = c2,
					});
				} else if(constraints[i].type == CT_LINE_LINE_ANGLE
					&& local_i->type == COM_LINE
					&& constraints[j].type == CT_POINT_LINE_DISTANCE
					&& local_j->type == COM_POINT) {
					assert(oppo_i->type == COM_LINE);

					struct element *theta = insert_cmd(&drawing, (struct command){
						.op = CMD_VALUE_INPUT,
						.result.type = ETYPE_VALUE,
					});

					struct element *d = insert_cmd(&drawing, (struct command){
						.op = CMD_VALUE_INPUT,
						.result.type = ETYPE_VALUE,
					});

					struct element* l = insert_cmd(&drawing, (struct command){
						.op = CMD_LINE_POINT_LINE_ANGLE,
						.hidden = true,
						.result.type = ETYPE_LINE,
						.arg1 = local_j->e,
						.arg2 = local_i->e,
						.arg3 = theta,
					});

					oppo_i->e = insert_cmd(&drawing, (struct command){
						.op = CMD_LINE_LINE_DISTANCE_PARALLEL,
						.hidden = true,
						.result.type = ETYPE_LINE,
						.arg1 = l,
						.arg2 = d,
					});
				} else if(constraints[i].type == CT_POINT_LINE_DISTANCE
					&& local_i->type == COM_LINE
					&& constraints[j].type == CT_POINT_POINT_DISTANCE
					&& local_j->type == COM_POINT) {
					assert(oppo_i->type == COM_POINT);

					struct element *d1 = insert_cmd(&drawing, (struct command){
						.op = CMD_VALUE_INPUT,
						.result.type = ETYPE_VALUE,
					});
					struct element *d2 = insert_cmd(&drawing, (struct command){
						.op = CMD_VALUE_INPUT,
						.result.type = ETYPE_VALUE,
					});

					struct element *l = insert_cmd(&drawing, (struct command){
						.op = CMD_LINE_LINE_DISTANCE_PARALLEL,
						.hidden = true,
						.result.type = ETYPE_LINE,
						.arg1 = local_i->e,
						.arg2 = d1,
					});

					struct element *c = insert_cmd(&drawing, (struct command){
						.op = CMD_CIRCLE_CENTER_RADIUS,
						.hidden = true,
						.result.type = ETYPE_CIRCLE,
						.arg1 = local_j->e,
						.arg2 = d2,
					});

					oppo_i->e = insert_cmd(&drawing, (struct command){
						.op = CMD_POINT_CIRCLE_LINE,
						.hidden = true,
						.result.type = ETYPE_POINT,
						.arg1 = c,
						.arg2 = l,
					});
				} else {
					printf("Unknown constraint combination\n");
					printf("%d %d\n", constraints[i].type, constraints[j].type);
					abort();
				}

				add_frontier(&frontier, oppo_i);
				constraints[i].used = true;
				constraints[j].used = true;
				goto candidate_found;
			}
		}
		// No candidate found
		break;

candidate_found:
		;
	}

	execute_drawing(&drawing, (double[]){8, 8, 8, 0, 0, M_PI * 0.5, 0, 0, 15});

	printf("<svg version=\"1.1\" viewBox=\"-50 -50 100 100\" width=\"1200\" height=\"1200\" xmlns=\"http://www.w3.org/2000/svg\">\n");

	// Axis lines
	printf("<line x1=\"-1000\" y1=\"0\" x2=\"1000\" y2=\"0\" stroke=\"black\" stroke-width=\"0.1\" stroke-opacity=\"0.4\" />\n");
	printf("<line y1=\"-1000\" x1=\"0\" y2=\"1000\" x2=\"0\" stroke=\"black\" stroke-width=\"0.1\" stroke-opacity=\"0.4\" />\n");

	for(struct command *current = drawing.root; current != NULL; current = current->next) {
		if(current->hidden) continue;
		switch(current->op) {
			case CMD_VALUE_INPUT:
				break;
			case CMD_ORIGIN:
			case CMD_POINT_CIRCLE_LINE:
			case CMD_POINT_CIRCLE_CIRCLE:
			case CMD_POINT_LINE_LINE:
				assert(current->result.type == ETYPE_POINT);
				plot_point(current->result.point);
				break;
			case CMD_CIRCLE_CENTER_RADIUS:
			case CMD_CIRCLE_CENTER_POINT:
				assert(current->result.type == ETYPE_CIRCLE);
				plot_circle(current->result.circle);
				break;
			case CMD_LINE_X:
			case CMD_LINE_POINT_POINT:
			case CMD_LINE_POINT_LINE_ANGLE:
			case CMD_LINE_LINE_DISTANCE_PARALLEL:
			case CMD_LINE_CIRCLE_CIRCLE_TANGENT:
				assert(current->result.type == ETYPE_LINE);
				plot_line(current->result.line);
				break;
		}
	}

	// plot_line_between(components[0].e->point, components[1].e->point);
	plot_line_between(components[1].e->point, components[2].e->point);
	plot_line_between(components[2].e->point, components[0].e->point);
	plot_line_between(components[1].e->point, components[5].e->point);
	plot_line_between(components[5].e->point, components[0].e->point);

	// plot_line_between(components[0].e->point, components[3].e->point);
	// plot_line_between(components[3].e->point, components[1].e->point);
	// plot_line_between(line_bend_end->point, line_end->point);
	// plot_arc_between(line_ctr->point, line_bend_start->point, line_bend_end->point);
	// plot_line_between(detector_right->point, decomposer_left->point);
	// plot_line_between(decomposer_right->point, solver_left->point);
	// plot_line_between(solver_right->point, solutions_left->point);

	printf("</svg>\n");
}
