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

struct subassembly {
	struct solve_step *steps;
	size_t steps_num;

	struct component **articulation;
	struct element *articulation_position;
	size_t articulation_num;

	bool fixed;
};

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

bool solve_constraints(struct constraints *constraints, struct drawing *drawing) {
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

	struct subassembly assemblies[16] = {0};
	size_t assemblies_num = 0;
	while(fix_first(constraints->elements, constraints->length, &fix)) {
		// Build triangles on that root
		assemblies[assemblies_num].steps = malloc(sizeof(struct solve_step) * constraints->length);
		assemblies[assemblies_num].articulation = malloc(sizeof(struct component*) * constraints->length);
		assemblies[assemblies_num].articulation_position = malloc(sizeof(struct element) * constraints->length);
		assemblies[assemblies_num].steps_num = build_triangles(constraints->elements, constraints->length, fix, assemblies_num+1, &assemblies[assemblies_num]);

		printf("Assembly %ld\n", assemblies_num);
		for(size_t i = 0; i < assemblies[assemblies_num].articulation_num; i++) {
			printf("  Articulation %p\n", assemblies[assemblies_num].articulation[i]);
		}

		// printf("Solved in %ld steps\n", steps_num);

		draw_solution(constraints->elements, fix, assemblies[assemblies_num].steps, assemblies[assemblies_num].steps_num, drawing);
		for(size_t i = 0; i < assemblies[assemblies_num].articulation_num; i++) {
			memcpy(&assemblies[assemblies_num].articulation_position[i], assemblies[assemblies_num].articulation[i]->e, sizeof(struct component));
		}
		assemblies_num++;
		assert(assemblies_num <= 16);
	}

	// We build everything from the first assembly
	assemblies[0].fixed = true;
	while(true) {
		// Look for unfixed assembly we can connect to something that is fixed
		for(size_t i = 0; i < assemblies_num; i++) {
			if(assemblies[i].fixed) continue;

			// Find a fixed asssembly it connects to
			for(size_t j = 0; j < assemblies_num; j++) {
				if(!assemblies[j].fixed) continue;

				struct component *articulation1 = NULL;

				// Find a shared articulation
				for(size_t articuation_i = 0; articuation_i < assemblies[i].articulation_num; articuation_i++) {
					for(size_t articuation_j = 0; articuation_j < assemblies[j].articulation_num; articuation_j++) {
						if(assemblies[i].articulation[articuation_i] == assemblies[j].articulation[articuation_j]) {
							articulation1 = assemblies[i].articulation[articuation_i];
							break;
						}
					}
				}

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
				// share a constraint. We can hopefully place the rest of the
				// assembly from that information

				printf("We found a match %p, %p\n", articulation1, constraint);
			}
		}
		break;
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
