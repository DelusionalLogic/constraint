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


static bool frontier_scan(struct component *component) {
	return component->fixed;
}

static void add_frontier(struct component *component) {
	assert(!component->fixed);
	component->fixed = true;
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

static bool try_fix_component(struct constraints *constraints_in, struct component *c, struct constraint **not_angle, bool *f1, struct constraint **possibly_angle, bool *f2) {
	struct constraint *constraints = constraints_in->elements;
	size_t constraints_num = constraints_in->length;

	// Find something that is not an angle
	for(size_t i = 0; i < constraints_num; i++) {
		if(constraints[i].used) continue;
		if(constraints[i].type == CT_LINE_LINE_ANGLE) continue;

		bool f;
		struct component *oppo;
		if(constraints[i].c1 == c) {
			oppo = constraints[i].c2;
			f = false;
		} else if(constraints[i].c2 == c) {
			oppo = constraints[i].c1;
			f = true;
		} else {
			continue;
		}

		if(!oppo->fixed) continue;

		*not_angle = &constraints[i];
		*f1 = f;
		break;
	}

	// No way to fix this component was found
	if(*not_angle == NULL) return false;

	// Look for another distance constraint
	for(size_t i = 0; i < constraints_num; i++) {
		if(constraints[i].used) continue;
		if(constraints[i].type == CT_LINE_LINE_ANGLE) continue;

		// We already selected this one, we can't use it again
		if(&constraints[i] == *not_angle) continue;

		bool f;
		struct component *oppo;
		if(constraints[i].c1 == c) {
			oppo = constraints[i].c2;
			f = false;
		} else if(constraints[i].c2 == c) {
			oppo = constraints[i].c1;
			f = true;
		} else {
			continue;
		}

		if(!oppo->fixed) continue;

		*possibly_angle = &constraints[i];
		*f2 = f;
		break;
	}

	if(*possibly_angle == NULL) {
		// We can't find any distance constraint to use, look for an angle
		// constraint
		for(size_t i = 0; i < constraints_num; i++) {
			if(constraints[i].used) continue;
			if(constraints[i].type != CT_LINE_LINE_ANGLE) continue;

			// We already selected this one, we can't use it again
			if(&constraints[i] == *not_angle) continue;

			// Unlike for distance constraints, we support doing a walk through
			// angle constraints that relate to the same singular point. This
			// means we have to process ALL the currently unused angle
			// constraints where one half is fixed.
			bool f;
			struct component *oppo;
			if(constraints[i].c1->fixed) {
				oppo = constraints[i].c2;
				f = true;
			} else if(constraints[i].c2->fixed) {
				oppo = constraints[i].c1;
				f = false;
			} else {
				continue;
			}

			if(oppo -> fixed) continue;
			assert(!oppo->fixed);

			if(oppo != c) {
				if(!find_angle(constraints, constraints_num, oppo, c, constraints[i].path))
					continue;
			}

			*possibly_angle = &constraints[i];
			*f2 = f;
		}
	}

	// No way to fix this component was found
	if(*possibly_angle == NULL) return false;

	return true;
}

static size_t build_triangles(struct constraints *constraints_in, struct component **components, size_t component_num, size_t origin, uint8_t useid, struct subassembly *assembly) {
	struct constraint *constraints = constraints_in->elements;
	size_t constraints_num = constraints_in->length;

	struct solve_step *steps = assembly->steps;
	uint64_t order = 1;

	for(size_t i = 0; i < component_num; i++) {
		// Reset all the fixed points
		components[i]->fixed = false;
	}

	for(size_t i = 0; i < constraints_num; i++) {
		if(constraints[i].used) continue;
		constraints[i].path[0].i = -1;
		constraints[i].forward = true;
	}

	constraints[origin].used = useid;
	constraints[origin].order = order++;
	add_frontier(constraints[origin].c1);
	add_frontier(constraints[origin].c2);

	size_t steps_i = 0;

	while(true) {
		// 0 distance PP_DISTANCE fixes the point without anything else
		// @INVEST: Is this really required anymore? I thought we had solved
		// this with the alias system?
		for(size_t i = 0; i < constraints_num; i++) {
			if(constraints[i].used) continue;

			if(constraints[i].type != CT_POINT_POINT_DISTANCE) continue;
			if(constraints[i].v != 0.0) continue;

			struct component *oppo;
			bool forward;

			if(frontier_scan(constraints[i].c1)) {
				oppo = constraints[i].c2;
				forward = true;
			} else if(frontier_scan(constraints[i].c2)) {
				oppo = constraints[i].c1;
				forward = false;
			} else continue;

			add_frontier(oppo);
			constraints[i].order = 0;
			constraints[i].used = useid;

			steps[steps_i].i = i;
			steps[steps_i].j = i;
			steps[steps_i].i_forward = forward;
			steps[steps_i].j_forward = forward;
			steps_i++;
		}

		for(size_t i = 0; i < component_num; i++) {
			// If a component was already fixed, we don't need to do anything special
			if(components[i]->fixed) continue;

			struct constraint *not_angle = NULL;
			bool f1;
			struct constraint *possibly_angle = NULL;
			bool f2;

			if(try_fix_component(constraints_in, components[i], &not_angle, &f1, &possibly_angle, &f2)) {
				add_frontier(components[i]);
				not_angle->used = useid;
				not_angle->order = order++;
				possibly_angle->used = useid;
				possibly_angle->order = order++;
				steps[steps_i].i = not_angle - constraints;
				steps[steps_i].j = possibly_angle - constraints;
				steps[steps_i].i_forward = f1;
				steps[steps_i].j_forward = f2;
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

		if(frontier_scan(constraints[i].c1)) {
			for(size_t j = 0; j < assembly->articulation_num; j++) {
				if(assembly->articulation[j] == constraints[i].c1) {
					goto nomatch;
				}
			}
			assembly->articulation[assembly->articulation_num++] = constraints[i].c1;
		} else if(frontier_scan(constraints[i].c2)) {
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

int unt64_t_compar(const void *a, const void *b) {
	uint64_t x = *(const uint64_t*)a;
	uint64_t y = *(const uint64_t*)b;

	return (x > y) - (x < y);
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

	// Collect all the components into a single set
	struct component **components = malloc(sizeof(struct component*) * constraints->length * 2);
	size_t component_num = 0;

	for(size_t i = 0; i < constraints->length; i++) {
		components[component_num++] = constraints->elements[i].c1;
		components[component_num++] = constraints->elements[i].c2;
	}

	qsort(components, component_num, sizeof(struct component*), unt64_t_compar);

	size_t dest_num = 1;
	for(size_t i = 1; i < component_num; i++) {
		if(components[dest_num-1] != components[i])
			components[dest_num++] = components[i];
	}

	// Pick some point point distance constraint as the base
	size_t fix;
	while(fix_first(constraints->elements, constraints->length, &fix)) {
		// Build triangles on that root
		assemblies[*assemblies_num].steps = malloc(sizeof(struct solve_step) * constraints->length);
		assemblies[*assemblies_num].articulation = malloc(sizeof(struct component*) * constraints->length);
		assemblies[*assemblies_num].articulation_position = malloc(sizeof(struct element*) * constraints->length);
		assemblies[*assemblies_num].steps_num = build_triangles(constraints, components, dest_num, fix, *assemblies_num+1, &assemblies[*assemblies_num]);
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
					goto constraint_found;
				}
				continue;
constraint_found:
				;

				// Here we have two assemblies, one fixed and the other not,
				// that share a single point and each one other point that
				// share a constraint. Try place the rigid body based on that
				// information

				float theta;
				if(constraint->type == CT_LINE_LINE_ANGLE) {
					assert(constraint->c1->e->type == ETYPE_LINE);
					assert(constraint->c2->e->type == ETYPE_LINE);

					// Align the two lines
					theta = atan2(constraint->c1->e->line.norm[1], constraint->c1->e->line.norm[0]) - atan2(constraint->c2->e->line.norm[1], constraint->c2->e->line.norm[0]);
					theta = forward ? -theta : theta;

					// Then rotate by whatever the constraint says
					theta += constraint->forward ? constraint->v : -constraint->v;
				} else if(constraint->type == CT_POINT_LINE_DISTANCE) {
					assert(constraint->c1->e->type == ETYPE_POINT);
					assert(constraint->c2->e->type == ETYPE_LINE);

					struct line line = constraint->c2->e->line;
					float norm_len = glm_vec2_norm(line.norm);
					float beta = atan2(line.norm[1], line.norm[0]);
					float v_signed = constraint->forward ? constraint->v : -constraint->v;

					if(forward) {
						// Point (c1) is in assembly i (unfixed), line (c2) is in assembly j (fixed)
						vec2 v_src;
						glm_vec2_sub(constraint->c1->e->point.pos, assemblies[i].articulation_position[articulation_i]->point.pos, v_src);
						float r = glm_vec2_norm(v_src);
						float alpha = atan2(v_src[1], v_src[0]);

						float d_pivot = (glm_vec2_dot(line.norm, assemblies[j].articulation_position[articulation_j]->point.pos) + line.C) / norm_len;
						float cos_val = (v_signed - d_pivot) / r;
						if(cos_val > 1.0f) cos_val = 1.0f;
						if(cos_val < -1.0f) cos_val = -1.0f;

						theta = beta - alpha + acos(cos_val);
					} else {
						// Line (c2) is in assembly i (unfixed), point (c1) is in assembly j (fixed)
						vec2 v_ext;
						glm_vec2_sub(constraint->c1->e->point.pos, assemblies[j].articulation_position[articulation_j]->point.pos, v_ext);
						float r = glm_vec2_norm(v_ext);
						float alpha_ext = atan2(v_ext[1], v_ext[0]);

						float d_pivot_i = (glm_vec2_dot(line.norm, assemblies[i].articulation_position[articulation_i]->point.pos) + line.C) / norm_len;
						float cos_val = (v_signed - d_pivot_i) / r;
						if(cos_val > 1.0f) cos_val = 1.0f;
						if(cos_val < -1.0f) cos_val = -1.0f;

						theta = alpha_ext - beta + acos(cos_val);
					}
				} else {
					abort();
				}
				constraint->used = i+1;

				assert(assemblies[i].articulation_position[articulation_i]->type == ETYPE_POINT);
				assert(assemblies[j].articulation_position[articulation_j]->type == ETYPE_POINT);

				mat3 transform;
				glm_mat3_identity(transform);

				glm_translate2d(transform, assemblies[j].articulation_position[articulation_j]->point.pos);

				glm_rotate2d(transform, theta);

				{
					vec2 negative_translate;
					glm_vec2_negate_to(assemblies[i].articulation_position[articulation_i]->point.pos, negative_translate);
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
				assemblies[i].fixed = true;
			}
		}
		break;
	}
}
