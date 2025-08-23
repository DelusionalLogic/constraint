#include <assert.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <cglm/cglm.h>
#include <string.h>

enum operation {
	CMD_ORIGIN,
	CMD_CIRCLE_CENTER_RADIUS,
	CMD_CIRCLE_CENTER_POINT,
	CMD_LINE_POINT_ANGLE,
	CMD_LINE_POINT_POINT,
	CMD_POINT_CIRCLE_LINE,
	CMD_POINT_CIRCLE_CIRCLE,
	CMD_POINT_LINE_LINE,
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

struct command {
	enum operation op;
	uint8_t root;
	bool hidden;

	union {
		struct command *as_cmd;
		double as_input;
	} arg1;

	union {
		struct command *as_cmd;
		double as_input;
	} arg2;

	union {
		struct circle circle;
		struct point point;
		struct line line;
	};

	struct command *next;
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

void plot_point(struct point p) {
	printf("<circle cx=\"%f\" cy=\"%f\" r=\".4\" fill=\"black\" />\n", p.pos[0], -p.pos[1]);
}

void plot_line_between(struct point p1, struct point p2) {
	printf("<line x1=\"%f\" y1=\"%f\" x2=\"%f\" y2=\"%f\" stroke=\"black\" stroke-width=\".2\" />\n", p1.pos[0], -p1.pos[1], p2.pos[0], -p2.pos[1]);
}

void plot_line(struct line l) {
	double minx = -50;
	double maxx =  50;

	double miny = -(l.norm[0] * minx + l.C) / l.norm[1];
	double maxy = -(l.norm[0] * maxx + l.C) / l.norm[1];

	printf("<line x1=\"%f\" y1=\"%f\" x2=\"%f\" y2=\"%f\" stroke=\"black\" stroke-width=\".2\" />\n", minx, -miny, maxx, -maxy);
}

void plot_arc_between(double r, struct point p1, struct point p2) {
	printf("<path d=\"M %f %f A %f %f 0 0 0 %f %f\" stroke=\"black\" stroke-width=\".2\" fill=\"none\" />\n", p1.pos[0], -p1.pos[1], r, r, p2.pos[0], -p2.pos[1]);
}

void plot_circle(struct circle c) {
	printf("<circle cx=\"%f\" cy=\"%f\" r=\"%f\" fill=\"none\" stroke=\"black\" stroke-width=\".1\" />\n", c.center[0], -c.center[1], c.radius);
}

struct command* insert_cmd(struct command **cmds, struct command cmd) {
	struct command *new = calloc(sizeof(struct command), 1);
	memcpy(new, &cmd, sizeof(struct command));
	assert(new != NULL);

	if(*cmds != NULL) (*cmds)->next = new;
	*cmds = new;

	return new;
}

struct command *create_perp(struct command **cmds, struct command *l, struct command *p) {
	struct command *c1 = insert_cmd(cmds, (struct command){
		.op = CMD_CIRCLE_CENTER_RADIUS,
		.hidden = true,
		.arg1.as_cmd = p,
		.arg2.as_input = 5, // Not actually input, just a random number
	});
	struct command *p1 = insert_cmd(cmds, (struct command){
		.op = CMD_POINT_CIRCLE_LINE,
		.hidden = true,
		.root = 0,
		.arg1.as_cmd = c1,
		.arg2.as_cmd = l,
	});
	struct command *p2 = insert_cmd(cmds, (struct command){
		.op = CMD_POINT_CIRCLE_LINE,
		.hidden = true,
		.root = 1,
		.arg1.as_cmd = c1,
		.arg2.as_cmd = l,
	});

	struct command *c2 = insert_cmd(cmds, (struct command){
		.op = CMD_CIRCLE_CENTER_POINT,
		.hidden = true,
		.arg1.as_cmd = p2,
		.arg2.as_cmd = p1,
	});
	struct command *c3 = insert_cmd(cmds, (struct command){
		.op = CMD_CIRCLE_CENTER_POINT,
		.hidden = true,
		.arg1.as_cmd = p1,
		.arg2.as_cmd = p2,
	});

	struct command *perp1 = insert_cmd(cmds, (struct command){
		.op = CMD_POINT_CIRCLE_CIRCLE,
		.hidden = true,
		.arg1.as_cmd = c2,
		.arg2.as_cmd = c3,
	});
	struct command *perp2 = insert_cmd(cmds, (struct command){
		.op = CMD_POINT_CIRCLE_CIRCLE,
		.hidden = true,
		.root = 1,
		.arg1.as_cmd = c2,
		.arg2.as_cmd = c3,
	});

	return insert_cmd(cmds, (struct command){
		.op = CMD_LINE_POINT_POINT,
		.hidden = true,
		.arg1.as_cmd = perp1,
		.arg2.as_cmd = perp2,
	});
}

int main(int argc, char *argv[]) {

	struct command *root = NULL;
	struct command *next = NULL;
	root = insert_cmd(&next, (struct command){
		.op = CMD_ORIGIN,
	});

	struct command *p2;
	{
		struct command *circle = insert_cmd(&next, (struct command){
			.op = CMD_CIRCLE_CENTER_RADIUS,
			.hidden = true,
			.arg1.as_cmd = root,
			.arg2.as_input = 20,
		});
		struct command *line = insert_cmd(&next, (struct command){
			.op = CMD_LINE_POINT_ANGLE,
			.hidden = true,
			.arg1.as_cmd = root,
			.arg2.as_input = M_PI*0.66666,
		});
		p2 = insert_cmd(&next, (struct command){
			.op = CMD_POINT_CIRCLE_LINE,
			.arg1.as_cmd = circle,
			.arg2.as_cmd = line,
		});
	}

	struct command *p3;
	{
		struct command *c1 = insert_cmd(&next, (struct command){
			.op = CMD_CIRCLE_CENTER_RADIUS,
			.hidden = true,
			.arg1.as_cmd = root,
			.arg2.as_input = 20,
		});
		struct command *c2 = insert_cmd(&next, (struct command){
			.op = CMD_CIRCLE_CENTER_RADIUS,
			.hidden = true,
			.arg1.as_cmd = p2,
			.arg2.as_input = 20,
		});
		p3 = insert_cmd(&next, (struct command){
			.op = CMD_POINT_CIRCLE_CIRCLE,
			.arg1.as_cmd = c1,
			.arg2.as_cmd = c2,
		});
	}

	struct command *round_start;
	struct command *round_end;
	{
		struct command *l1 = insert_cmd(&next, (struct command){
			.op = CMD_LINE_POINT_POINT,
			.hidden = true,
			.arg1.as_cmd = root,
			.arg2.as_cmd = p2,
		});
		struct command *l2 = insert_cmd(&next, (struct command){
			.op = CMD_LINE_POINT_POINT,
			.hidden = true,
			.arg1.as_cmd = p2,
			.arg2.as_cmd = p3,
		});

		struct command *l1_parallel;
		{
			struct command *perp = create_perp(&next, l1, p2);
			struct command *c = insert_cmd(&next, (struct command){
				.op = CMD_CIRCLE_CENTER_RADIUS,
				.hidden = true,
				.arg1.as_cmd = p2,
				.arg2.as_input = 4,
			});

			struct command *d = insert_cmd(&next, (struct command){
				.op = CMD_POINT_CIRCLE_LINE,
				.hidden = true,
				.root = 1,
				.arg1.as_cmd = c,
				.arg2.as_cmd = perp,
			});

			l1_parallel = create_perp(&next, perp, d);
		}

		struct command *l2_parallel;
		{
			struct command *perp = create_perp(&next, l2, p2);
			struct command *c = insert_cmd(&next, (struct command){
				.op = CMD_CIRCLE_CENTER_RADIUS,
				.hidden = true,
				.arg1.as_cmd = p2,
				.arg2.as_input = 4,
			});

			struct command *d = insert_cmd(&next, (struct command){
				.op = CMD_POINT_CIRCLE_LINE,
				.hidden = true,
				.root = 1,
				.arg1.as_cmd = c,
				.arg2.as_cmd = perp,
			});

			l2_parallel = create_perp(&next, perp, d);
		}

		struct command *rounding_center = insert_cmd(&next, (struct command){
			.op = CMD_POINT_LINE_LINE,
			.hidden = true,
			.arg1.as_cmd = l1_parallel,
			.arg2.as_cmd = l2_parallel,
		});

		struct command *rounding_circle = insert_cmd(&next, (struct command){
			.op = CMD_CIRCLE_CENTER_RADIUS,
			.hidden = true,
			.arg1.as_cmd = rounding_center,
			.arg2.as_input = 4,
		});

		round_start = insert_cmd(&next, (struct command){
			.op = CMD_POINT_CIRCLE_LINE,
			.hidden = true,
			.arg1.as_cmd = rounding_circle,
			.arg2.as_cmd = l1,
		});
		round_end = insert_cmd(&next, (struct command){
			.op = CMD_POINT_CIRCLE_LINE,
			.hidden = true,
			.arg1.as_cmd = rounding_circle,
			.arg2.as_cmd = l2,
		});

		// insert_cmd(&next, (struct command){
		// 	.op = CMD_CIRCLE_CENTER_POINT,
		// 	.arg1.as_cmd = rounding_center,
		// 	.arg2.as_cmd = p2,
		// });
	}

		// {
		// 	.op = CMD_POINT_CIRCLE_LINE,
		// 	.root = 1,
		// 	.arg1.as_cmd = &cmd[7],
		// 	.arg2.as_cmd = &cmd[2],
		// },
		// {
		// 	.op = CMD_LINE_POINT_POINT,
		// 	.arg1.as_cmd = &cmd[3],
		// 	.arg2.as_cmd = &cmd[6],
		// },
		// {
		// 	.op = CMD_POINT_CIRCLE_LINE,
		// 	.hidden = true,
		// 	.root = 1,
		// 	.arg1.as_cmd = &cmd[7],
		// 	.arg2.as_cmd = &cmd[9],
		// },
		// {
		// 	.op = CMD_CIRCLE_CENTER_POINT,
		// 	.hidden = true,
		// 	.arg1.as_cmd = &cmd[8],
		// 	.arg2.as_cmd = &cmd[3],
		// },
		// {
		// 	.op = CMD_POINT_CIRCLE_LINE,
		// 	.hidden = true,
		// 	.root = 1,
		// 	.arg1.as_cmd = &cmd[11],
		// 	.arg2.as_cmd = &cmd[2],
		// },
		// {
		// 	.op = CMD_CIRCLE_CENTER_POINT,
		// 	.arg1.as_cmd = &cmd[12],
		// 	.arg2.as_cmd = &cmd[3],
		// },
		// {
		// 	.op = CMD_CIRCLE_CENTER_POINT,
		// 	.arg1.as_cmd = &cmd[3],
		// 	.arg2.as_cmd = &cmd[12],
		// },
		// {
		// 	.op = CMD_POINT_CIRCLE_CIRCLE,
		// 	.arg1.as_cmd = &cmd[13],
		// 	.arg2.as_cmd = &cmd[14],
		// },
		// {
		// 	.op = CMD_POINT_CIRCLE_CIRCLE,
		// 	.root = 1,
		// 	.arg1.as_cmd = &cmd[13],
		// 	.arg2.as_cmd = &cmd[14],
		// },
		// {
		// 	.op = CMD_LINE_POINT_POINT,
		// 	.root = 1,
		// 	.arg1.as_cmd = &cmd[15],
		// 	.arg2.as_cmd = &cmd[16],
		// },
	// };

	for(struct command *current = root; current != NULL; current = current->next) {
		switch(current->op) {
			case CMD_ORIGIN: {
				glm_vec2_zero(current->point.pos);
			}break;
			case CMD_CIRCLE_CENTER_RADIUS: {
				glm_vec2_copy(current->arg1.as_cmd->point.pos, current->circle.center);
				current->circle.radius = current->arg2.as_input;
			}break;
			case CMD_LINE_POINT_ANGLE: {
				double theta = current->arg2.as_input;
				current->line.norm[0] = -sin(theta);
				current->line.norm[1] = cos(theta);
				vec2 offset = {-current->arg1.as_cmd->point.pos[0], -current->arg1.as_cmd->point.pos[1]};
				current->line.C = glm_vec2_dot(current->line.norm, offset);
			}break;
			case CMD_LINE_POINT_POINT: {
				current->line.norm[0] = current->arg2.as_cmd->point.pos[1] - current->arg1.as_cmd->point.pos[1];
				current->line.norm[1] = current->arg1.as_cmd->point.pos[0] - current->arg2.as_cmd->point.pos[0];
				current->line.C = current->arg1.as_cmd->point.pos[1] * -current->line.norm[1] - current->line.norm[0] * current->arg1.as_cmd->point.pos[0];
			}break;
			case CMD_POINT_CIRCLE_LINE: {
				circle_line_intersect(current->arg1.as_cmd->circle, current->arg2.as_cmd->line, current->root, &current->point);
			}break;
			case CMD_POINT_CIRCLE_CIRCLE: {
				vec2 between_centers;
				glm_vec2_sub(current->arg1.as_cmd->circle.center, current->arg2.as_cmd->circle.center, between_centers);
				glm_vec2_mul(between_centers, (vec2){2, 2}, between_centers);
				double a = between_centers[0];
				double b = between_centers[1];

				double c = (pow(current->arg2.as_cmd->circle.center[0], 2) - pow(current->arg1.as_cmd->circle.center[0], 2)) + \
						(pow(current->arg2.as_cmd->circle.center[1], 2) - pow(current->arg1.as_cmd->circle.center[1], 2)) - \
						(pow(current->arg2.as_cmd->circle.radius, 2) - pow(current->arg1.as_cmd->circle.radius, 2));

				struct line radical_axis = { {a, b}, c };
				circle_line_intersect(current->arg1.as_cmd->circle, radical_axis, current->root, &current->point);
			}break;
			case CMD_POINT_LINE_LINE: {
				double a1 = current->arg1.as_cmd->line.norm[0];
				double b1 = current->arg1.as_cmd->line.norm[1];
				double c1 = current->arg1.as_cmd->line.C;

				double a2 = current->arg2.as_cmd->line.norm[0];
				double b2 = current->arg2.as_cmd->line.norm[1];
				double c2 = current->arg2.as_cmd->line.C;

				current->point.pos[0] = -(c1*b2 - c2*b1) / (a1*b2 - a2*b1);
				current->point.pos[1] = -(a1*c2 - a2*c1) / (a1*b2 - a2*b1);
			}break;
			case CMD_CIRCLE_CENTER_POINT: {
				glm_vec2_copy(current->arg1.as_cmd->point.pos, current->circle.center);
				vec2 imm;
				glm_vec2_sub(current->arg1.as_cmd->point.pos, current->arg2.as_cmd->point.pos, imm);
				current->circle.radius = glm_vec2_norm(imm);
			}break;

		}
		// printf("%fx + %fy + %f = 0\n", cmd[2].line.norm[0], cmd[2].line.norm[1], cmd[2].line.C);
	}
	printf("<svg version=\"1.1\" viewBox=\"-50 -50 100 100\" width=\"1200\" height=\"1200\" xmlns=\"http://www.w3.org/2000/svg\">\n");
	printf("<line x1=\"-1000\" y1=\"0\" x2=\"1000\" y2=\"0\" stroke=\"black\" stroke-width=\".1\" />\n");
	printf("<line y1=\"-1000\" x1=\"0\" y2=\"1000\" x2=\"0\" stroke=\"black\" stroke-width=\".1\" />\n");

	for(struct command *current = root; current != NULL; current = current->next) {
		if(current->hidden) continue;
		switch(current->op) {
			case CMD_ORIGIN:
			case CMD_POINT_CIRCLE_LINE:
			case CMD_POINT_CIRCLE_CIRCLE:
			case CMD_POINT_LINE_LINE:
				plot_point(current->point);
				break;
			case CMD_CIRCLE_CENTER_RADIUS:
			case CMD_CIRCLE_CENTER_POINT:
				plot_circle(current->circle);
				break;
			case CMD_LINE_POINT_ANGLE:
			case CMD_LINE_POINT_POINT:
				plot_line(current->line);
				break;
		}
	}

	plot_line_between(root->point, round_start->point);
	plot_line_between(round_end->point, p3->point);
	plot_line_between(p3->point, root->point);

	plot_arc_between(4, round_start->point, round_end->point);

	printf("</svg>\n");
}
