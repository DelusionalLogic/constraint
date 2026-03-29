#include "cad/solve.h"

#include "cad/log.h"
#include "cad/util.h"
#include <string.h>

#define SWAP(x, y) do { \
		typeof(x) tmp = x; \
		x = y; \
		y = tmp; \
	}while(0)

char *constraint_type_name[] = {
	[CT_POINT_POINT_DISTANCE] = "Point Point Distance",
	[CT_POINT_LINE_DISTANCE] = "Point Line Distance",
	[CT_LINE_LINE_ANGLE] = "Line Line Angle",
};

void alias_point(struct constraints *c, struct component *alias, struct component *target) {
	assert(alias->type == COM_POINT);
	assert(target->type == COM_POINT);

	assert(c->aliases_num < 16);

	struct alias *new = &c->aliases[c->aliases_num++];
	new->alias = alias;
	new->target = target;
}

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

void free_constraints(struct constraints *c) {
	free(c->elements);
	c->elements = NULL;
	c->length = 0;
	c->capacity = 0;
	c->aliases_num = 0;
}


// @COMPL: We should do something better than this. I don't really know what.
struct frontier {
	struct component *elems[128];
	size_t n;
};

static bool frontier_scan(struct frontier *frontier, struct component *component) {
	for(size_t i = 0; i < sizeof(frontier->elems)/sizeof(frontier->elems[0]); i++) {
		if(frontier->elems[i] == component) return true;
	}

	return false;
}

static void add_frontier(struct frontier *frontier, struct component *component) {
	assert(!frontier_scan(frontier, component));
	assert(frontier->n < 128);
	frontier->elems[frontier->n++] = component;
}

static void build_angle_point_line(struct drawing *drawing, struct constraint *constraints, size_t index_i, size_t index_j, struct component *local_i, struct component *local_j, struct component *oppo_i) {
	assert(local_i->e != NULL);
	assert(local_j->e != NULL);
	assert(local_i->type == COM_LINE);
	assert(local_j->type == COM_POINT);
	struct element *theta = insert_cmd(drawing, (struct command){
		.op = CMD_VALUE_INPUT,
		.index = index_i,
		.dir = constraints[index_i].forward,
		.result.type = ETYPE_VALUE,
	});

	for(struct path_step *p = constraints[index_i].path; p <= constraints[index_i].path+SEARCH_DEPTH && p->i != -1; p++) {
		theta = insert_cmd(drawing, (struct command){
			.op = CMD_OFFSET_INPUT,
			.arg1 = theta,
			.index = p->i,
			.dir = p->direction,
			.result.type = ETYPE_VALUE,
		});
	}

	struct element *d = insert_cmd(drawing, (struct command){
		.op = CMD_VALUE_INPUT,
		.index = index_j,
		.dir = constraints[index_i].forward,
		.result.type = ETYPE_VALUE,
	});

	struct element* l = insert_cmd(drawing, (struct command){
		.op = CMD_LINE_POINT_LINE_ANGLE,
		.hidden = !oppo_i->show_when_placed,
		.result.type = ETYPE_LINE,
		.arg1 = local_j->e,
		.arg2 = local_i->e,
		.arg3 = theta,
	});
	assert(l != NULL);

	oppo_i->e = insert_cmd(drawing, (struct command){
		.op = CMD_LINE_LINE_DISTANCE_PARALLEL,
		.hidden = !oppo_i->show_when_placed,
		.result.type = ETYPE_LINE,
		.arg1 = l,
		.arg2 = d,
	});
}

// CLEANUP: None of this makes any sense. We shouldn't have to resolve the
// angles separately. We should just be building up the subgraphs correctly,
// and then the solutions for angles will pop out by themselves. So remove this
// once we get subgraph solving working
struct angle_search_frame {
	size_t constraint_i;
	struct component *head;
	bool dir;
};
static bool find_angle(struct constraint *constraints, size_t constraints_num, struct component *first_component, struct component *needle, struct path_step *path) {
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
			for(struct angle_search_frame *i = frames; i <= frame; i++) {
				path[i - frames].i = i->constraint_i;
				path[i - frames].direction = i->dir;
			}
			if(frame < frames + SEARCH_DEPTH-1) {
				path[frame - frames + 1].i = -1;
			}
			free(checked);
			return true;
		}

		checked[frame->constraint_i] = true;

		frame++;
		frame->constraint_i = 0;
		frame->head = other;
	}

	free(checked);
	return false;
}

struct solve_step {
	size_t i;
	size_t j;
	bool i_forward;
	bool j_forward;
};

static bool fix_first(struct constraint *constraints, size_t constraints_num, size_t *c) {
	for(size_t i = 0; i < constraints_num; i++) {
		if(constraints[i].used) continue;
		if(constraints[i].type != CT_POINT_POINT_DISTANCE) continue;
		if(constraints[i].v == 0.0) continue;

		*c = i;
		return true;
	}

	return false;
}

static size_t build_triangles(struct constraint *constraints, size_t constraints_num, size_t origin, uint8_t useid, struct subassembly *assembly) {
	struct solve_step *steps = assembly->steps;
	struct frontier frontier = {};
	uint64_t order = 1;

	for(size_t i = 0; i < constraints_num; i++) {
		if(constraints[i].used) continue;

		constraints[i].path[0].i = -1;
		constraints[i].forward = true;
	}

	constraints[origin].used = useid;
	constraints[origin].order = order++;
	add_frontier(&frontier, constraints[origin].c1);
	add_frontier(&frontier, constraints[origin].c2);

	size_t steps_i = 0;

	while(true) {
		for(size_t i = 0; i < constraints_num; i++) {
			if(constraints[i].used) continue;

			if(constraints[i].type != CT_POINT_POINT_DISTANCE) continue;
			if(constraints[i].v != 0.0) continue;

			struct component *oppo;
			bool forward;

			if(frontier_scan(&frontier, constraints[i].c1)) {
				oppo = constraints[i].c2;
				forward = true;
			} else if(frontier_scan(&frontier, constraints[i].c2)) {
				oppo = constraints[i].c1;
				forward = false;
			} else continue;

			add_frontier(&frontier, oppo);
			constraints[i].order = 0;
			constraints[i].used = useid;

			steps[steps_i].i = i;
			steps[steps_i].j = i;
			steps[steps_i].i_forward = forward;
			steps[steps_i].j_forward = forward;
			steps_i++;
		}

		for(size_t i = 0; i < constraints_num; i++) {
			if(constraints[i].used) continue;

			struct component *oppo_i;
			bool i_forward;

			if(frontier_scan(&frontier, constraints[i].c1)) {
				oppo_i = constraints[i].c2;
				i_forward = true;
			} else if(frontier_scan(&frontier, constraints[i].c2)) {
				oppo_i = constraints[i].c1;
				i_forward = false;
			} else continue;

			for(size_t j = i+1; j < constraints_num; j++) {
				if(constraints[j].used) continue;

				struct component *oppo_j;
				bool j_forward;

				if(frontier_scan(&frontier, constraints[j].c1)) {
					oppo_j = constraints[j].c2;
					j_forward = true;
				} else if(frontier_scan(&frontier, constraints[j].c2)) {
					oppo_j = constraints[j].c1;
					j_forward = false;
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

				add_frontier(&frontier, oppo_i);
				constraints[i].used = useid;
				constraints[i].order = order++;
				constraints[j].used = useid;
				constraints[j].order = order++;
				steps[steps_i].i = i;
				steps[steps_i].j = j;
				steps[steps_i].i_forward = i_forward;
				steps[steps_i].j_forward = j_forward;
				steps_i++;
				goto candidate_found;
			}
		}
		// No candidate found
		break;

candidate_found:
		;
	}

	// Find the articulations (the points where we connect to the outside
	// world)

	for(size_t i = 0; i < constraints_num; i++) {
		if(constraints[i].used == useid) continue;

		if(frontier_scan(&frontier, constraints[i].c1)) {
			for(size_t j = 0; j < assembly->articulation_num; j++) {
				if(assembly->articulation[j] == constraints[i].c1) {
					goto nomatch;
				}
			}
			assembly->articulation[assembly->articulation_num++] = constraints[i].c1;
		} else if(frontier_scan(&frontier, constraints[i].c2)) {
			for(size_t j = 0; j < assembly->articulation_num; j++) {
				if(assembly->articulation[j] == constraints[i].c2) {
					goto nomatch;
				}
			}
			assembly->articulation[assembly->articulation_num++] = constraints[i].c2;
		} else continue;
nomatch:
		;
	}

	return steps_i;
}

static void draw_solution(struct constraint *constraints, size_t fix, struct solve_step* steps, size_t steps_num, struct drawing *drawing) {
	{
		constraints[fix].c1->e = insert_cmd(drawing, (struct command){
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
			.index = fix,
			.dir = constraints[fix].forward,
			.result.type = ETYPE_VALUE,
		});

		struct element *c = insert_cmd(drawing, (struct command){
			.op = CMD_CIRCLE_CENTER_RADIUS,
			.hidden = true,
			.result.type = ETYPE_CIRCLE,
			.arg1 = constraints[fix].c1->e,
			.arg2 = distance,
		});

		constraints[fix].c2->e = insert_cmd(drawing, (struct command){
			.op = CMD_POINT_CIRCLE_LINE,
			.hidden = true,
			.result.type = ETYPE_POINT,
			.arg1 = c,
			.arg2 = xaxis,
		});
	}

	// Build the solution steps
	for(struct solve_step *step = steps; step < (steps + steps_num); step++) {
		struct component *local_i;
		struct component *oppo_i;
		if(step->i_forward) {
			local_i = constraints[step->i].c1;
			oppo_i = constraints[step->i].c2;
		} else {
			local_i = constraints[step->i].c2;
			oppo_i = constraints[step->i].c1;
		}

		struct component *local_j;
		struct component *oppo_j;
		if(step->j_forward) {
			local_j = constraints[step->j].c1;
			oppo_j = constraints[step->j].c2;
		} else {
			local_j = constraints[step->j].c2;
			oppo_j = constraints[step->j].c1;
		}

		if(oppo_i != oppo_j) {
			if(constraints[step->i].type == CT_LINE_LINE_ANGLE) {
				oppo_i = oppo_j;
			} else if(constraints[step->j].type == CT_LINE_LINE_ANGLE) {
				// Do nothing, oppo_j isn't used below
			} else {
				assert(false);
			}
		}

		assert(local_i->e != NULL);
		assert(local_j->e != NULL);

		bool shown = oppo_i->show_when_placed;

		if(constraints[step->i].type == CT_POINT_POINT_DISTANCE
			&& local_i->type == COM_POINT
			&& constraints[step->j].type == CT_POINT_POINT_DISTANCE
			&& local_j->type == COM_POINT) {
			assert(oppo_i->type == COM_POINT);

			if(step->i == step->j) {
				assert(constraints[step->i].v == 0);
				oppo_i->e = local_i->e;
			} else {
				struct element *d1 = insert_cmd(drawing, (struct command){
					.op = CMD_VALUE_INPUT,
					.index = step->i,
					.dir = constraints[step->i].forward,
					.result.type = ETYPE_VALUE,
				});

				struct element *d2 = insert_cmd(drawing, (struct command){
					.op = CMD_VALUE_INPUT,
					.index = step->j,
					.dir = constraints[step->j].forward,
					.result.type = ETYPE_VALUE,
				});

				struct element *c1 = insert_cmd(drawing, (struct command){
					.op = CMD_CIRCLE_CENTER_RADIUS,
					.hidden = !shown,
					.result.type = ETYPE_CIRCLE,
					.arg1 = local_i->e,
					.arg2 = d1,
				});

				struct element *c2 = insert_cmd(drawing, (struct command){
					.op = CMD_CIRCLE_CENTER_RADIUS,
					.hidden = !shown,
					.result.type = ETYPE_CIRCLE,
					.arg1 = local_j->e,
					.arg2 = d2,
				});

				oppo_i->e = insert_cmd(drawing, (struct command){
					.op = CMD_POINT_CIRCLE_CIRCLE,
					.hidden = !shown,
					.result.type = ETYPE_POINT,
					.arg1 = c1,
					.arg2 = c2,
				});
			}
		} else if(constraints[step->i].type == CT_POINT_LINE_DISTANCE
			&& local_i->type == COM_POINT
			&& constraints[step->j].type == CT_POINT_LINE_DISTANCE
			&& local_j->type == COM_POINT) {
			assert(oppo_i->type == COM_LINE);

			struct element *d1 = insert_cmd(drawing, (struct command){
				.op = CMD_VALUE_INPUT,
				.index = step->i,
				.dir = constraints[step->i].forward,
				.result.type = ETYPE_VALUE,
			});
			struct element *d2 = insert_cmd(drawing, (struct command){
				.op = CMD_VALUE_INPUT,
				.index = step->j,
				.dir = constraints[step->j].forward,
				.result.type = ETYPE_VALUE,
			});

			struct element *c1 = insert_cmd(drawing, (struct command){
				.op = CMD_CIRCLE_CENTER_RADIUS,
				.hidden = !shown,
				.result.type = ETYPE_CIRCLE,
				.arg1 = local_i->e,
				.arg2 = d1,
			});

			struct element *c2 = insert_cmd(drawing, (struct command){
				.op = CMD_CIRCLE_CENTER_RADIUS,
				.hidden = !shown,
				.result.type = ETYPE_CIRCLE,
				.arg1 = local_j->e,
				.arg2 = d2,
			});

			oppo_i->e = insert_cmd(drawing, (struct command){
				.op = CMD_LINE_CIRCLE_CIRCLE_TANGENT,
				.hidden = !shown,
				.result.type = ETYPE_LINE,
				.arg1 = c1,
				.arg2 = c2,
			});
		} else if(constraints[step->i].type == CT_POINT_LINE_DISTANCE
			&& local_i->type == COM_LINE
			&& constraints[step->j].type == CT_POINT_LINE_DISTANCE
			&& local_j->type == COM_LINE) {
			assert(oppo_i->type == COM_POINT);

			struct element *d1 = insert_cmd(drawing, (struct command){
				.op = CMD_VALUE_INPUT,
				.index = step->i,
				.dir = constraints[step->i].forward,
				.result.type = ETYPE_VALUE,
			});
			struct element *d2 = insert_cmd(drawing, (struct command){
				.op = CMD_VALUE_INPUT,
				.index = step->j,
				.dir = constraints[step->j].forward,
				.result.type = ETYPE_VALUE,
			});

			struct element *l1 = insert_cmd(drawing, (struct command){
				.op = CMD_LINE_LINE_DISTANCE_PARALLEL,
				.hidden = !shown,
				.result.type = ETYPE_LINE,
				.arg1 = local_i->e,
				.arg2 = d1,
			});

			struct element *l2 = insert_cmd(drawing, (struct command){
				.op = CMD_LINE_LINE_DISTANCE_PARALLEL,
				.hidden = !shown,
				.result.type = ETYPE_LINE,
				.arg1 = local_j->e,
				.arg2 = d2,
			});

			oppo_i->e = insert_cmd(drawing, (struct command){
				.op = CMD_POINT_LINE_LINE,
				.hidden = !shown,
				.result.type = ETYPE_POINT,
				.arg1 = l1,
				.arg2 = l2,
			});
		} else if(constraints[step->i].type == CT_LINE_LINE_ANGLE
			&& local_i->type == COM_LINE
			&& constraints[step->j].type == CT_POINT_LINE_DISTANCE
			&& local_j->type == COM_POINT) {
			assert(oppo_i->type == COM_LINE);

			constraints[step->i].forward = step->i_forward;
			constraints[step->j].forward = step->j_forward;

			build_angle_point_line(drawing, constraints, step->i, step->j, local_i, local_j, oppo_i);
		} else if(constraints[step->i].type == CT_POINT_LINE_DISTANCE
			&& local_i->type == COM_POINT
			&& constraints[step->j].type == CT_LINE_LINE_ANGLE
			&& local_j->type == COM_LINE) {
			assert(oppo_i->type == COM_LINE);

			constraints[step->i].forward = step->i_forward;
			constraints[step->j].forward = step->j_forward;

			build_angle_point_line(drawing, constraints, step->j, step->i, local_j, local_i, oppo_i);
		} else if(constraints[step->i].type == CT_POINT_LINE_DISTANCE
			&& local_i->type == COM_LINE
			&& constraints[step->j].type == CT_POINT_POINT_DISTANCE
			&& local_j->type == COM_POINT) {
			assert(oppo_i->type == COM_POINT);

			struct element *d1 = insert_cmd(drawing, (struct command){
				.op = CMD_VALUE_INPUT,
				.index = step->i,
				.dir = constraints[step->i].forward,
				.result.type = ETYPE_VALUE,
			});
			struct element *d2 = insert_cmd(drawing, (struct command){
				.op = CMD_VALUE_INPUT,
				.index = step->j,
				.dir = constraints[step->j].forward,
				.result.type = ETYPE_VALUE,
			});

			struct element *l = insert_cmd(drawing, (struct command){
				.op = CMD_LINE_LINE_DISTANCE_PARALLEL,
				.hidden = !shown,
				.result.type = ETYPE_LINE,
				.arg1 = local_i->e,
				.arg2 = d1,
			});

			struct element *c = insert_cmd(drawing, (struct command){
				.op = CMD_CIRCLE_CENTER_RADIUS,
				.hidden = !shown,
				.result.type = ETYPE_CIRCLE,
				.arg1 = local_j->e,
				.arg2 = d2,
			});

			oppo_i->e = insert_cmd(drawing, (struct command){
				.op = CMD_POINT_CIRCLE_LINE,
				.hidden = !shown,
				.root = constraints[step->j].c2 == local_j,
				.result.type = ETYPE_POINT,
				.arg1 = c,
				.arg2 = l,
			});
		} else if(constraints[step->i].type == CT_POINT_POINT_DISTANCE
			&& local_i->type == COM_POINT
			&& constraints[step->j].type == CT_POINT_LINE_DISTANCE
			&& local_j->type == COM_LINE) {
			// @COPYPASTA: Taken from above but with params swapped
			assert(oppo_i->type == COM_POINT);

			struct element *d1 = insert_cmd(drawing, (struct command){
				.op = CMD_VALUE_INPUT,
				.index = step->i,
				.dir = constraints[step->i].forward,
				.result.type = ETYPE_VALUE,
			});
			struct element *d2 = insert_cmd(drawing, (struct command){
				.op = CMD_VALUE_INPUT,
				.index = step->j,
				.dir = constraints[step->j].forward,
				.result.type = ETYPE_VALUE,
			});

			struct element *l = insert_cmd(drawing, (struct command){
				.op = CMD_LINE_LINE_DISTANCE_PARALLEL,
				.hidden = !shown,
				.result.type = ETYPE_LINE,
				.arg1 = local_j->e,
				.arg2 = d2,
			});

			struct element *c = insert_cmd(drawing, (struct command){
				.op = CMD_CIRCLE_CENTER_RADIUS,
				.hidden = !shown,
				.result.type = ETYPE_CIRCLE,
				.arg1 = local_i->e,
				.arg2 = d1,
			});

			oppo_i->e = insert_cmd(drawing, (struct command){
				.op = CMD_POINT_CIRCLE_LINE,
				.hidden = !shown,
				.root = constraints[step->j].c2 == local_i,
				.result.type = ETYPE_POINT,
				.arg1 = c,
				.arg2 = l,
			});
		} else {
			CRASH("Unknown constraint combination %s and %s\n", constraint_type_name[constraints[step->i].type], constraint_type_name[constraints[step->j].type]);
		}

	}
}

bool solve_constraints(struct constraints *constraints, struct drawing *drawing, struct subassembly *assemblies, size_t *assemblies_num) {
	*assemblies_num = 0;
	// Replace all the aliased points with the point they point to
	for(size_t i = 0; i < constraints->aliases_num; i++) {
		struct alias *alias = &constraints->aliases[i];

		for(size_t j = 0; j < constraints->length; j++) {
			struct constraint *it = &constraints->elements[j];
			if(it->c1 == alias->alias) it->c1 = alias->target;
			if(it->c2 == alias->alias) it->c2 = alias->target;
		}
	}

	// Pick some point point distance constraint as the base
	size_t fix;

	while(fix_first(constraints->elements, constraints->length, &fix)) {
		// Build triangles on that root
		assemblies[*assemblies_num].steps = malloc(sizeof(struct solve_step) * constraints->length);
		assemblies[*assemblies_num].articulation = malloc(sizeof(struct component*) * constraints->length);
		assemblies[*assemblies_num].articulation_position = malloc(sizeof(struct element*) * constraints->length);
		assemblies[*assemblies_num].steps_num = build_triangles(constraints->elements, constraints->length, fix, *assemblies_num+1, &assemblies[*assemblies_num]);
		assemblies[*assemblies_num].fix = fix;

		// printf("Assembly %ld\n", *assemblies_num);
		// for(size_t i = 0; i < assemblies[*assemblies_num].articulation_num; i++) {
		// 	printf("  Articulation %p\n", assemblies[*assemblies_num].articulation[i]);
		// }

		// printf("Solved in %ld steps\n", steps_num);

		draw_solution(constraints->elements, fix, assemblies[*assemblies_num].steps, assemblies[*assemblies_num].steps_num, drawing);
		for(size_t i = 0; i < assemblies[*assemblies_num].articulation_num; i++) {
			assemblies[*assemblies_num].articulation_position[i] = assemblies[*assemblies_num].articulation[i]->e;
		}
		(*assemblies_num)++;
		assert(*assemblies_num <= 16);
	}

	// Fill out the aliased points out with the values from their targets
	for(size_t i = 0; i < constraints->aliases_num; i++) {
		struct alias *alias = &constraints->aliases[i];

		memcpy(alias->alias, alias->target, sizeof(struct component));
	}

	// Check for unsolved constraints
	bool complete = true;
	// for(size_t i = 0; i < constraints->length; i++) {
	// 	if(!constraints->elements[i].used) {
	// 		complete = false;
	// 	}
	// }

	return complete;
}

static void affine_transform_vec2(mat3 m, vec2 in, vec2 out) {
	vec3 h = {in[0], in[1], 1.0f};
	vec3 result;
	glm_mat3_mulv(m, h, result);
	out[0] = result[0];
	out[1] = result[1];
}

void reconstruct_drawing(struct constraints *constraints, struct subassembly *assemblies, size_t *assemblies_num) {
	// We build everything from the first assembly
	assemblies[0].fixed = true;
	while(true) {
		// Look for unfixed assembly we can connect to something that is fixed
		for(size_t i = 0; i < *assemblies_num; i++) {
			if(assemblies[i].fixed) continue;

			// Find a fixed asssembly it connects to
			for(size_t j = 0; j < *assemblies_num; j++) {
				if(!assemblies[j].fixed) continue;

				size_t articulation_i;
				size_t articulation_j;

				// Find a shared articulation
				for(articulation_i = 0; articulation_i < assemblies[i].articulation_num; articulation_i++) {
					for(articulation_j = 0; articulation_j < assemblies[j].articulation_num; articulation_j++) {
						if(assemblies[i].articulation[articulation_i] == assemblies[j].articulation[articulation_j]) {
							goto articulation_found;
						}
					}
				}
				continue;
articulation_found:
				;

				struct constraint *constraint = NULL;
				bool forward;

				// An unused constraint would let us match disjoint articulations
				for(size_t constraint_i = 0; constraint_i < constraints->length; constraint_i++) {
					struct constraint *c = &constraints->elements[constraint_i];
					if(c->used) continue;

					for(size_t articuation_i = 0; articuation_i < assemblies[i].articulation_num; articuation_i++) {
						if(c->c1 == assemblies[i].articulation[articuation_i]) {
							forward = true;
							goto constraint_matches_i;
						} else if(c->c2 == assemblies[i].articulation[articuation_i]) {
							forward = false;
							goto constraint_matches_i;
						}
					}
					continue;
constraint_matches_i:
					;

					{
						struct component *needle = forward ? c->c2 : c->c1;
						for(size_t articuation_j = 0; articuation_j < assemblies[j].articulation_num; articuation_j++) {
							if(needle == assemblies[j].articulation[articuation_j]) {
								goto constraint_matches_j;
							}
						}
					}
					continue;
constraint_matches_j:
					;

					constraint = c;
				}

				// Here we have two assemblies, one fixed and the other not,
				// that share a single point and each one other point that
				// share a constraint. Try place the rigid body based on that
				// information

				float theta = atan2(constraint->c1->e->line.norm[1], constraint->c1->e->line.norm[0]) - atan2(constraint->c2->e->line.norm[1], constraint->c2->e->line.norm[0]);
				theta = forward ? -theta : theta;

				constraint->used = true;
				theta += constraint->forward ? constraint->v : -constraint->v;

				assert(assemblies[i].articulation_position[articulation_i]->type == ETYPE_POINT);
				assert(assemblies[j].articulation_position[articulation_j]->type == ETYPE_POINT);

				mat3 transform;
				glm_mat3_identity(transform);

				glm_translate2d(transform, assemblies[i].articulation_position[articulation_i]->point.pos);

				glm_rotate2d(transform, theta);

				{
					vec2 negative_translate;
					glm_vec2_negate_to(assemblies[j].articulation_position[articulation_j]->point.pos, negative_translate);
					glm_translate2d(transform, negative_translate);
				}

				// We have to transform the fixed point separately, since it
				// doesn't have a build step
				{
					struct component *c = constraints->elements[assemblies[i].fix].c1;
					assert(c->type == COM_POINT);

					affine_transform_vec2(transform, c->e->point.pos, c->e->point.pos);
				}
				{
					struct component *c = constraints->elements[assemblies[i].fix].c2;
					assert(c->type == COM_POINT);

					affine_transform_vec2(transform, c->e->point.pos, c->e->point.pos);
				}

				// Transform all other points in the body by iterating the
				// steps. Each step places a single component.
				for(size_t k = 0; k < assemblies[i].steps_num; k++) {
					struct solve_step *step = &assemblies[i].steps[k];

					struct component *c = step->i_forward ?
						constraints->elements[step->i].c2 :
						constraints->elements[step->i].c1;

					if(c->type == COM_POINT) {
						affine_transform_vec2(transform, c->e->point.pos, c->e->point.pos);
					} else if(c->type == COM_LINE) {
						// Find a point on the line, what point doesn't matter
						// since the whole line is moving
						struct line line = c->e->line;

						vec2 p;
						glm_vec2_zero(p);

						glm_vec2_muladds(line.norm, line.C, p);
						double rec = glm_vec2_norm2(line.norm);
						glm_vec2_divs(p, rec, p);

						// Rotate the line to the new orientation
						glm_vec2_rotate(line.norm, theta, line.norm);

						// Transform the fixed point
						affine_transform_vec2(transform, p, p);

						// Calculate a C to follow the new point
						glm_vec2_negate(p);
						line.C = glm_vec2_dot(line.norm, p);

						c->e->line = line;
					} else {
						abort();
					}
				}
			}
		}
		break;
	}
}
