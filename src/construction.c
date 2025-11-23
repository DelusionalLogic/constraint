#include "cad.h"
#include <cglm/cglm.h>
#include <string.h>

bool circle_line_intersect(struct circle circle, struct line line, uint8_t root, struct point *point) {
	double a = line.norm[0];
	double b = line.norm[1];
	double c = line.C + glm_vec2_dot(line.norm, circle.center);

	double rec = pow(a, 2) + pow(b, 2);
	double x0 = -a*c / rec;
	double y0 = -b*c / rec;

	double r2 = pow(circle.radius, 2); 
	double test = r2 * rec; 
	if(pow(c, 2.0) - test > DBL_EPSILON * 1e14) {
		return false;
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
	return true;
}

void line_through_points(struct point p1, struct point p2, struct line* l) {
	l->norm[0] = p1.pos[1] - p1.pos[1];
	l->norm[1] = p2.pos[0] - p1.pos[0];
	l->C = p1.pos[1] * -l->norm[1] + p1.pos[0] * -l->norm[1];
}

void line_line_intersect(struct line l1, struct line l2, struct point* p) {
	double a1 = l1.norm[0];
	double b1 = l1.norm[1];
	double c1 = l1.C;

	double a2 = l2.norm[0];
	double b2 = l2.norm[1];
	double c2 = l2.C;

	p->pos[0] = -(c1*b2 - c2*b1) / (a1*b2 - a2*b1);
	p->pos[1] = -(a1*c2 - a2*c1) / (a1*b2 - a2*b1);
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


void execute_drawing(struct drawing *drawing, double inputs[]) {
	for(struct command *current = drawing->root; current != NULL; current = current->next) {
		switch(current->op) {
			case CMD_VALUE_INPUT: {
				assert(current->result.type == ETYPE_VALUE);
				current->result.value = inputs[current->index];
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
				// printf("Magnitude is %f %f\n", mag, current->arg2->value);
				current->result.line.C = current->arg1->line.C - mag * current->arg2->value;
			}break;
			case CMD_LINE_CIRCLE_CIRCLE_TANGENT: {
				assert(current->arg1->type == ETYPE_CIRCLE);
				assert(current->arg2->type == ETYPE_CIRCLE);
				assert(current->result.type == ETYPE_LINE);

				if(current->arg1->circle.radius == 0 && current->arg2->circle.radius == 0) {
					// The tangent is just a line through the two centers
					// @FAST: This is wasteful. If the procedure took the two
					// vectors directly, we wouldn't have to copy here.
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
				assert(current->arg1->type == ETYPE_CIRCLE);
				assert(current->arg2->type == ETYPE_CIRCLE);
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
				if(!circle_line_intersect(current->arg1->circle, radical_axis, current->root, &current->result.point)) {
					drawing->error = current;
					return;
				}
			}break;
			case CMD_POINT_LINE_LINE: {
				assert(current->result.type == ETYPE_POINT);
				line_line_intersect(current->arg1->line, current->arg2->line, &current->result.point);
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


