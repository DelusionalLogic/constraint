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
	l->norm[0] = p1.pos[1] - p2.pos[1];
	l->norm[1] = p2.pos[0] - p1.pos[0];
	l->C = p1.pos[0] * -l->norm[0] + p1.pos[1] * -l->norm[1];
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

#define SETSIGN(b, v) ((v) * ((2 * (b)) - 1))

static void affine_transform_vec2(mat3 m, vec2 in, vec2 out) {
	vec3 h = {in[0], in[1], 1.0f};
	vec3 result;
	glm_mat3_mulv(m, h, result);
	out[0] = result[0];
	out[1] = result[1];
}

static void transform_part(struct command *cmd, const struct command *last_cmd, mat3 transform) {
	mat3 rotate;
	glm_mat3_copy(transform, rotate);
	rotate[2][0] = 0.0;
	rotate[2][1] = 0.0;
	rotate[2][2] = 0.0;

	while(cmd != NULL) {
		switch(cmd->op) {
			case CMD_VALUE_INPUT:
			case CMD_OFFSET_INPUT:
			break;
			case CMD_LINE_X:
			case CMD_LINE_POINT_POINT:
			case CMD_LINE_POINT_LINE_ANGLE:
			case CMD_LINE_CIRCLE_CIRCLE_TANGENT:
			case CMD_LINE_LINE_DISTANCE_PARALLEL:
				// Find a point on the line, what point doesn't matter
				// since the whole line is moving
				struct line *line = &cmd->result.line;

				vec2 p;
				glm_vec2_zero(p);

				if(line->norm[0] > line->norm[1])
					glm_vec2_copy((vec2){-line->C / line->norm[0], 0}, p);
				else
					glm_vec2_copy((vec2){0, -line->C / line->norm[1]}, p);

				// Rotate the line to the new orientation
				affine_transform_vec2(rotate, line->norm, line->norm);

				// Transform the fixed point
				affine_transform_vec2(transform, p, p);

				// Calculate a C to follow the new point
				glm_vec2_negate(p);
				line->C = glm_vec2_dot(line->norm, p);
			break;
			case CMD_CIRCLE_CENTER_RADIUS:
			case CMD_CIRCLE_CENTER_POINT:
				affine_transform_vec2(transform, cmd->result.circle.center, cmd->result.circle.center);
			break;
			case CMD_ORIGIN:
			case CMD_POINT_CIRCLE_LINE:
			case CMD_POINT_CIRCLE_CIRCLE:
			case CMD_POINT_LINE_LINE:
				affine_transform_vec2(transform, cmd->result.point.pos, cmd->result.point.pos);
			break;

			case CMD_IMPORT_POINT_LINE:
				transform_part(cmd->d->first_command, cmd->d->last_command, transform);
				break;
		}

		// @HACK The last command is also included in this assembly
		if(cmd == last_cmd) break;
		cmd = cmd->next;
	}
}

void place_points(struct drawing *drawing, double inputs[]) {
	struct command *outer_object = drawing->root;

	for(struct command *current = drawing->root; current != NULL; current = current->next) {
		switch(current->op) {
			case CMD_VALUE_INPUT: {
				assert(current->result.type == ETYPE_VALUE);
				current->result.value = SETSIGN(current->dir, inputs[current->index]);
			}break;
			case CMD_OFFSET_INPUT: {
				assert(current->arg1->type == ETYPE_VALUE);
				assert(current->result.type == ETYPE_VALUE);
				current->result.value = current->arg1->value + SETSIGN(current->dir, inputs[current->index]);
				current->result.value -= (M_PI*2.0) * floor(current->result.value / (M_PI*2.0));
			}break;
			case CMD_ORIGIN: {
				assert(current->result.type == ETYPE_POINT);
				glm_vec2_zero(current->result.point.pos);
				outer_object = current;
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
				double value = current->arg3->value;
				value = current->root == 0 ? value : -value;
				glm_vec2_rotate(current->arg2->line.norm, value, current->result.line.norm);
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
			case CMD_IMPORT_POINT_LINE: {
				assert(current->arg1->type == ETYPE_POINT);
				assert(current->arg2->type == ETYPE_LINE);

				assert(current->d != NULL);
				assert(current->attachp != NULL);
				assert(current->attachl != NULL);

				double theta;
				// Align the two lines
				theta = atan2(current->arg2->line.norm[1], current->arg2->line.norm[0]) - atan2(current->attachl->line.norm[1], current->attachl->line.norm[0]);
				fprintf(stderr, "%f\n", theta / M_PI * 180.0);

				mat3 transform;
				glm_mat3_identity(transform);

				glm_translate2d(transform, current->attachp->point.pos);

				glm_rotate2d(transform, theta);

				{
					vec2 negative_translate;
					glm_vec2_negate_to(current->arg1->point.pos, negative_translate);
					glm_translate2d(transform, negative_translate);
				}

				fprintf(stderr, "Assembly %p %p %p %f\n", current->d, current->d->first_command, current->d->last_command, theta);
				transform_part(current->d->first_command, current->d->last_command, transform);
			}break;
		}
		// printf("%fx + %fy + %f = 0\n", cmd[2].line.norm[0], cmd[2].line.norm[1], cmd[2].line.C);
	}


	// Realign the very first component as the base again for simplicty
	{
		struct command *origin = drawing->root;
		struct command *xaxis = origin->next;

		assert(origin->op == CMD_ORIGIN);
		assert(origin->result.type == ETYPE_POINT);
		assert(xaxis->op == CMD_LINE_X);
		assert(xaxis->result.type == ETYPE_LINE);

		double theta;
		theta =  atan2(1, 0) - atan2(xaxis->result.line.norm[1], xaxis->result.line.norm[0]);

		mat3 transform;
		glm_mat3_identity(transform);

		glm_rotate2d(transform, theta);

		{
			vec2 negative_translate;
			glm_vec2_negate_to(origin->result.point.pos, negative_translate);
			glm_translate2d(transform, negative_translate);
		}

		fprintf(stderr, "W %f %f\n", origin->result.point.pos[0], origin->result.point.pos[1]);
		transform_part(outer_object, NULL, transform);
	}
}

void free_drawing(struct drawing *drawing) {
	struct command *c = drawing->root;
	while (c) {
		struct command *next = c->next;
		free(c);
		c = next;
	}
	drawing->root = NULL;
	drawing->tail = NULL;
	drawing->error = NULL;
}


