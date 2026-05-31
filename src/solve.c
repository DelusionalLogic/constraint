#include "cad/solve.h"

#include "cad/log.h"
#include "cad/util.h"
#include <string.h>

#define SWAP(x, y) do { \
		typeof(x) tmp = x; \
		x = y; \
		y = tmp; \
	}while(0)

#define CONTAINER_OF(ptr, Type, member) ({ \
		const typeof(((Type*)0)->member) *__mptr = (ptr); \
		(Type*)((char*)__mptr - offsetof(Type, member)); \
	})

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

static void build_angle_point_line2(struct drawing *drawing, struct constraint *constraints, size_t index_i, size_t index_j, struct element *local_i, struct element *local_j, bool inverse, struct element **oppo_i) {
	assert(local_i != NULL);
	assert(local_j != NULL);
	assert(local_i->type == ETYPE_LINE);
	assert(local_j->type == ETYPE_POINT);
	struct element *theta = insert_cmd(drawing, (struct command){
		.op = CMD_VALUE_INPUT,
		.index = index_i,
		.dir = constraints[index_i].forward ^ inverse,
		.result.type = ETYPE_VALUE,
	});

	for(struct path_step *p = constraints[index_i].path; p <= constraints[index_i].path+SEARCH_DEPTH && p->i != -1; p++) {
		theta = insert_cmd(drawing, (struct command){
			.op = CMD_OFFSET_INPUT,
			.arg1 = theta,
			.index = p->i,
			.dir = p->direction ^ inverse,
			.result.type = ETYPE_VALUE,
		});
	}

	struct element *d = insert_cmd(drawing, (struct command){
		.op = CMD_VALUE_INPUT,
		.index = index_j,
		.dir = constraints[index_i].forward ^ inverse,
		.result.type = ETYPE_VALUE,
	});

	struct element* l = insert_cmd(drawing, (struct command){
		.op = CMD_LINE_POINT_LINE_ANGLE,
		.hidden = false,
		.result.type = ETYPE_LINE,
		.arg1 = local_j,
		.arg2 = local_i,
		.arg3 = theta,
	});
	assert(l != NULL);

	*oppo_i = insert_cmd(drawing, (struct command){
		.op = CMD_LINE_LINE_DISTANCE_PARALLEL,
		.hidden = false,
		.result.type = ETYPE_LINE,
		.arg1 = l,
		.arg2 = d,
	});
}

static void build_angle_point_line(struct drawing *drawing, struct constraint *constraints, size_t index_i, size_t index_j, struct component *local_i, struct component *local_j, struct component *oppo_i) {
	return build_angle_point_line2(drawing, constraints, index_i, index_j, local_i->e, local_j->e, false, &oppo_i->e);
}

struct solve_step {
	size_t i;
	size_t j;
	size_t k;
	size_t x;
	size_t y;
	size_t z;
	bool i_forward;
	bool j_forward;
	bool k_forward;
	bool x_forward;
	bool y_forward;
	bool z_forward;

	struct subassembly *assembly;
	struct subassembly *assembly2;
};

static bool fix_first(struct constraint *constraints, size_t constraints_num, size_t *c) {
	for(size_t i = 0; i < constraints_num; i++) {
		if(constraints[i].used) continue;

		if(constraints[i].type != CT_POINT_POINT_DISTANCE && constraints[i].type != CT_POINT_LINE_DISTANCE) continue;
		if(constraints[i].type == CT_POINT_POINT_DISTANCE && constraints[i].v == 0.0) continue;

		if(constraints[i].c1->ein != NULL) continue;
		if(constraints[i].c2->ein != NULL) continue;

		*c = i;
		return true;
	}

	return false;
}

static bool try_fix_component(struct constraints *constraints_in, struct component *c, struct constraint **not_angle, bool *f1, struct constraint **possibly_angle, bool *f2, struct constraint **second_not_angle, bool *f3) {
	assert(!c->fixed);

	struct constraint *constraints = constraints_in->elements;
	size_t constraints_num = constraints_in->length;

	// Find something that is not an angle
	// Even though this also handles assemblies, we only check against this
	// exact component. The caller will call us for every component in the
	// assembly.
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

	if(c->ein != NULL) {
		// Find a second non-angle constraint for something in our assembly
		for(size_t i = 0; i < constraints_num; i++) {
			if(constraints[i].used) continue;
			if(constraints[i].type == CT_LINE_LINE_ANGLE) continue;

			// We already selected this one, we can't use it again
			if(&constraints[i] == *not_angle) continue;

			// Find the side that's fixed
			bool f;
			struct component *dest;
			if(constraints[i].c1->fixed) {
				dest = constraints[i].c2;
				f = true;
			} else if(constraints[i].c2->fixed) {
				dest = constraints[i].c1;
				f = false;
			} else {
				continue;
			}

			// It has to relate to something on the same assembly as c
			if(dest->ein != c->ein) continue;

			*second_not_angle = &constraints[i];
			*f3 = f;
			break;
		}

		if(*second_not_angle == NULL) return false;
	}

	// Look for another distance constraint
	for(size_t i = 0; i < constraints_num; i++) {
		if(constraints[i].used) continue;
		if(constraints[i].type == CT_LINE_LINE_ANGLE) continue;

		// We already selected this one, we can't use it again
		if(&constraints[i] == *not_angle) continue;
		if(c->ein != NULL && &constraints[i] == *second_not_angle) continue;

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
			if(c->ein != NULL && &constraints[i] == *second_not_angle) continue;

			// @CLEANUP Look into removing this "angle walking". I don't think
			// we need it if we have proper subassembly inclusion
			// Unlike for distance constraints, we support doing a walk through
			// angle constraints that relate to the same singular point. This
			// means we have to process ALL the currently unused angle
			// constraints where one half is fixed.
			bool f;
			struct component *dest;
			if(constraints[i].c1->fixed) {
				dest = constraints[i].c2;
				f = true;

				// Since C is supposedly current unfixed, the fixed side can't
				// be part of the same assembly
				assert(constraints[i].c1->ein != c->ein);
			} else if(constraints[i].c2->fixed) {
				dest = constraints[i].c1;
				f = false;

				// Since C is supposedly current unfixed, the fixed side can't
				// be part of the same assembly
				assert(constraints[i].c2->ein != c->ein);
			} else {
				continue;
			}

			if(c->ein == NULL) {
				if(dest != c) {
					continue;
				}
			} else {
				// We don't do the whole walking thing if we're solving for assemblies
				if(c->ein != dest->ein) continue;
			}

			// Is this actually correct?
			if(dest->fixed) continue;

			*possibly_angle = &constraints[i];
			*f2 = f;
			if(c == dest) break;
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

	// You can't fix a subcomponent, as it has too much freedom
	assert(constraints[origin].c1->in == NULL);
	assert(constraints[origin].c2->in == NULL);
	assert(constraints[origin].c1->ein == NULL);
	assert(constraints[origin].c2->ein == NULL);

	add_frontier(constraints[origin].c1);
	add_frontier(constraints[origin].c2);
	constraints[origin].c1->in = assembly;
	constraints[origin].c2->in = assembly;
	constraints[origin].c1->ein = assembly;
	constraints[origin].c2->ein = assembly;

	size_t steps_i = 0;

	while(true) {
		// 0 distance PP_DISTANCE fixes the point without anything else
		// @INVEST: Is this really required anymore? I thought we had solved
		// this with the alias system?
		// @HACK Really all of this should go in try_fix_component since we
		// need an additional constraint to fix rotation of assemblies
		for(size_t i = 0; i < constraints_num; i++) {
			if(constraints[i].used) continue;

			if(constraints[i].type != CT_POINT_POINT_DISTANCE) continue;
			if(constraints[i].v != 0.0) continue;

			// We'll handle this elsewhere?
			if(!constraints[i].c1->fixed && constraints[i].c1->ein != NULL) continue;
			if(!constraints[i].c2->fixed && constraints[i].c2->ein != NULL) continue;

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
			assert(oppo->in == NULL);
			oppo->in = assembly;
			oppo->ein = assembly;
			constraints[i].order = 0;
			constraints[i].used = useid;

			steps[steps_i].assembly = NULL;
			steps[steps_i].i = i;
			steps[steps_i].j = i;
			steps[steps_i].i_forward = forward;
			steps[steps_i].j_forward = forward;
			steps_i++;
		}

		for(size_t i = 0; i < component_num; i++) {
			// If a component was already fixed, we don't need to do anything
			if(components[i]->fixed) continue;

			struct constraint *not_angle = NULL;
			bool f1;
			struct constraint *possibly_angle = NULL;
			bool f2;

			struct constraint *second_not_angle = NULL;
			bool f3;

			if(try_fix_component(constraints_in, components[i], &not_angle, &f1, &possibly_angle, &f2, &second_not_angle, &f3)) {
				struct subassembly *sub = components[i]->ein;
				if(sub == NULL) {
					add_frontier(components[i]);
					assert(components[i]->in == NULL);
					components[i]->in = assembly;
					components[i]->ein = assembly;
				} else {
					// We've fixed the subcomponent, so we have to fix the whole thing
					for(size_t j = 0; j < component_num; j++) {
						if(components[j]->ein != sub) continue;

						add_frontier(components[j]);
						components[j]->ein = assembly;
					}
				}

				not_angle->used = useid;
				not_angle->order = order++;
				possibly_angle->used = useid;
				possibly_angle->order = order++;

				steps[steps_i].assembly = second_not_angle == NULL ? NULL : sub;
				steps[steps_i].i = not_angle - constraints;
				steps[steps_i].j = possibly_angle - constraints;
				steps[steps_i].i_forward = f1;
				steps[steps_i].j_forward = f2;

				if(second_not_angle != NULL) {
					second_not_angle->used = useid;
					second_not_angle->order = order++;
					steps[steps_i].k = second_not_angle - constraints;
					steps[steps_i].k_forward = f3;
				}

				steps_i++;
				goto candidate_found;
			}
		}

		// Look for two subassemblies that are each almost fixed (by two
		// constraints) and are mutually constrained by two constraints
		{
			// Find the first subcomponent we can fix
			for(size_t i = 0; i < component_num; i++) {
				// If a component was already fixed, we don't need to do anything
				if(components[i]->fixed) continue;

				// We are only looking for subcomponents
				if(components[i]->ein == NULL) continue;

				struct constraint *not_angle_a = NULL;
				bool f1_a;
				struct constraint *possibly_angle_a = NULL;
				bool f2_a;

				struct constraint *second_not_angle_a = NULL;
				bool f3_a;

				struct subassembly *ein_a = components[i]->ein;
				// @HACK Reset the points assembly information force it to not
				// use the subassembly path
				components[i]->ein = NULL;
				if(try_fix_component(constraints_in, components[i], &not_angle_a, &f1_a, &possibly_angle_a, &f2_a, &second_not_angle_a, &f3_a)) {
					// This should never be set since we reset the subassembly
					components[i]->ein = ein_a;
					assert(second_not_angle_a == NULL);

					// Now find a second assembly
					for(size_t j = 0; j < component_num; j++) {
						// If a component was already fixed, we don't need to do anything
						if(components[j]->fixed) continue;

						// We are only looking for subcomponents
						if(components[j]->ein == NULL) continue;
						if(components[j]->ein == ein_a) continue;

						struct constraint *not_angle_b = NULL;
						bool f1_b;
						struct constraint *possibly_angle_b = NULL;
						bool f2_b;

						struct constraint *second_not_angle_b = NULL;
						bool f3_b;
						struct subassembly *ein_b = components[j]->ein;
						// @HACK Reset the points assembly information force it to not
						// use the subassembly path
						components[j]->ein = NULL;
						if(try_fix_component(constraints_in, components[j], &not_angle_b, &f1_b, &possibly_angle_b, &f2_b, &second_not_angle_b, &f3_b)) {
							components[j]->ein = ein_b;
							// This should never be set since we reset the subassembly
							assert(second_not_angle_b == NULL);


							// We now have two subassmblies that are fixable by
							// two constraints to the current one. Now we need
							// to find two constraints that go between them

							struct constraint *not_angle_c = NULL;
							bool f1_c;
							struct constraint *possibly_angle_c = NULL;
							bool f2_c;

							for(size_t k = 0; k < constraints_num; k++) {
								if(constraints[k].used) continue;

								bool f;
								struct component *oppo;
								if(constraints[k].c1->ein == ein_a && constraints[k].c2->ein == ein_b) {
									oppo = constraints[k].c2;
									f = false;
								} else if(constraints[k].c1->ein == ein_b && constraints[k].c2->ein == ein_a) {
									oppo = constraints[k].c1;
									f = true;
								} else {
									continue;
								}

								assert(!oppo->fixed);

								not_angle_c = &constraints[k];
								f1_c = f;

								break;
							}

							if(not_angle_c == NULL) continue;

							for(size_t k = 0; k < constraints_num; k++) {
								if(constraints[k].used) continue;

								if(&constraints[k] == not_angle_c) continue;

								bool f;
								struct component *oppo;
								if(constraints[k].c1->ein == ein_a && constraints[k].c2->ein == ein_b) {
									oppo = constraints[k].c2;
									f = false;
								} else if(constraints[k].c1->ein == ein_b && constraints[k].c2->ein == ein_a) {
									oppo = constraints[k].c1;
									f = true;
								} else {
									continue;
								}

								assert(!oppo->fixed);

								possibly_angle_c = &constraints[k];
								f2_c = f;

								break;
							}

							if(possibly_angle_c == NULL) continue;

							fprintf(stderr, "Assembly %p and %p, %d %d %s %s\n", ein_a, ein_b, f1_c, f2_c, constraint_type_name[not_angle_c->type], constraint_type_name[possibly_angle_c->type]);

							// We've fixed the subcomponents, so we have to fix
							// the whole thing
							for(size_t j = 0; j < component_num; j++) {
								if(components[j]->ein != ein_a && components[j]->ein != ein_b) continue;

								add_frontier(components[j]);
								components[j]->ein = assembly;
							}

							// There's a total of 6 different constraints involved here
							not_angle_a->used = useid;
							not_angle_a->order = order++;
							not_angle_b->used = useid;
							not_angle_b->order = order++;
							not_angle_c->used = useid;
							not_angle_c->order = order++;

							possibly_angle_a->used = useid;
							possibly_angle_a->order = order++;
							possibly_angle_b->used = useid;
							possibly_angle_b->order = order++;
							possibly_angle_c->used = useid;
							possibly_angle_c->order = order++;

							steps[steps_i].assembly = ein_a;
							steps[steps_i].assembly2 = ein_b;
							steps[steps_i].i = not_angle_a - constraints;
							steps[steps_i].i_forward = f1_a;
							steps[steps_i].j = possibly_angle_a - constraints;
							steps[steps_i].j_forward = f2_a;
							steps[steps_i].k = not_angle_b - constraints;
							steps[steps_i].k_forward = f1_a;
							steps[steps_i].x = possibly_angle_b - constraints;
							steps[steps_i].x_forward = f2_b;
							steps[steps_i].y = not_angle_c - constraints;
							steps[steps_i].y_forward = f1_c;
							steps[steps_i].z = possibly_angle_c - constraints;
							steps[steps_i].z_forward = f2_c;

							steps_i++;

							goto candidate_found;
						}
						components[j]->ein = ein_b;
					}
				}
				components[i]->ein = ein_a;
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

static void draw_point_from_2_distance(struct drawing *drawing, struct constraint *constraints, size_t i, size_t j, struct component *local_i, struct component *local_j, struct element **e, bool shown) {
	struct element *d1 = insert_cmd(drawing, (struct command){
		.op = CMD_VALUE_INPUT,
		.index = i,
		.dir = constraints[i].forward,
		.result.type = ETYPE_VALUE,
	});

	struct element *d2 = insert_cmd(drawing, (struct command){
		.op = CMD_VALUE_INPUT,
		.index = j,
		.dir = constraints[j].forward,
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

	*e = insert_cmd(drawing, (struct command){
		.op = CMD_POINT_CIRCLE_CIRCLE,
		.hidden = !shown,
		.result.type = ETYPE_POINT,
		.arg1 = c1,
		.arg2 = c2,
	});
}

static void draw_for_subassembly(struct constraint *constraints, size_t fix, struct solve_step *step, struct drawing *drawing, struct subassembly *assembly) {
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

	struct component *local_k;
	struct component *oppo_k;
	if(step->k_forward) {
		local_k = constraints[step->k].c1;
		oppo_k = constraints[step->k].c2;
	} else {
		local_k = constraints[step->k].c2;
		oppo_k = constraints[step->k].c1;
	}

	// The local_ side is the "outer" (more leaf) component
	// By the construction we know that i and k are never angle constraints.

	// @HACK for now just solve the simple case where oppo_i and oppo_k are the
	// same component. Technically I think we should be able to solve other
	// cases as well, I just don't want to do the math.
	assert(oppo_i == oppo_k);

	// @HACK Let's only solve for an explicit angle for now
	assert(constraints[step->j].type == CT_LINE_LINE_ANGLE);

	// The point we share with the subassembly
	struct element *p = NULL;
	draw_point_from_2_distance(drawing, constraints, step->i, step->k, local_i, local_k, &p, false);
	assert(p != NULL);

	// The line that matches the angle
	struct element *theta = insert_cmd(drawing, (struct command){
		.op = CMD_VALUE_INPUT,
		.index = step->j,
		.dir = constraints[step->j].forward,
		.result.type = ETYPE_VALUE,
	});
	struct element *l = insert_cmd(drawing, (struct command){
		.op = CMD_LINE_POINT_LINE_ANGLE,
		.hidden = !oppo_j->show_when_placed,
		.result.type = ETYPE_LINE,
		.root = 1,
		.arg1 = p,
		.arg2 = local_j->e,
		.arg3 = theta,
	});
	assert(l != NULL);

	insert_cmd(drawing, (struct command){
		.op = CMD_IMPORT_POINT_LINE,
		.arg1 = p,
		.arg2 = l,
		.d = step->assembly,
		.attachp = oppo_i->e,
		.attachl = oppo_j->e,
	});
}

void point_from_line_and_distance(struct drawing *drawing, struct constraint *constraints, size_t c1, size_t c2, struct element *line, struct element *point, int root, struct element **result) {
	assert(line->type == ETYPE_LINE);
	assert(point->type == ETYPE_POINT);

	struct element *d1 = insert_cmd(drawing, (struct command){
		.op = CMD_VALUE_INPUT,
		.index = c1,
		.dir = constraints[c1].forward,
		.result.type = ETYPE_VALUE,
	});
	struct element *d2 = insert_cmd(drawing, (struct command){
		.op = CMD_VALUE_INPUT,
		.index = c2,
		.dir = constraints[c2].forward,
		.result.type = ETYPE_VALUE,
	});

	struct element *l = insert_cmd(drawing, (struct command){
		.op = CMD_LINE_LINE_DISTANCE_PARALLEL,
		.hidden = false,
		.result.type = ETYPE_LINE,
		.arg1 = line,
		.arg2 = d1,
	});

	struct element *c = insert_cmd(drawing, (struct command){
		.op = CMD_CIRCLE_CENTER_RADIUS,
		.hidden = false,
		.result.type = ETYPE_CIRCLE,
		.arg1 = point,
		.arg2 = d2,
	});

	*result = insert_cmd(drawing, (struct command){
		.op = CMD_POINT_CIRCLE_LINE,
		.hidden = false,
		.root = root,
		.result.type = ETYPE_POINT,
		.arg1 = c,
		.arg2 = l,
	});
}

#define expand_constraint(NAME, IDX) \
	struct constraint *NAME = &constraints[IDX]; \
	struct component *NAME##_local; \
	struct component *NAME##_oppo; \
	do { \
		if(IDX##_forward) { \
			NAME##_local = constraints[IDX].c1; \
			NAME##_oppo = constraints[IDX].c2; \
		} else { \
			NAME##_local = constraints[IDX].c2; \
			NAME##_oppo = constraints[IDX].c1; \
		} \
	} while(0)
	


static void draw_for_double_subassembly(struct constraint *constraints, size_t fix, struct solve_step *step, struct drawing *drawing, struct subassembly *assembly) {
	assert(step->assembly != NULL);
	assert(step->assembly2 != NULL);

	// Here we have two assemblies that relate to the current fixed set through
	// two constraints each, but are also mutually constrained by two
	// constraints.

	expand_constraint(a1, step->i);
	expand_constraint(a2, step->j);

	expand_constraint(b1, step->k);
	expand_constraint(b2, step->x);

	expand_constraint(c1, step->y);
	expand_constraint(c2, step->z);

	// Lets solve for a pretty static case first
	assert(a1->type == CT_POINT_POINT_DISTANCE);
	assert(a2->type == CT_POINT_LINE_DISTANCE);

	assert(b1->type == CT_POINT_LINE_DISTANCE);
	assert(b2->type == CT_LINE_LINE_ANGLE);

	assert(c1->type == CT_POINT_LINE_DISTANCE);
	assert(c2->type == CT_LINE_LINE_ANGLE);

	// All the constraints form triangles, not necessarily required, but
	// simpler
	assert(a1_oppo == a2_oppo);
	assert(a1_local != a2_local);
	assert(b1_oppo == b2_oppo);
	assert(b1_local != b2_local);
	assert(c1_oppo == c2_oppo);
	assert(c1_local != c2_local);

	// The basic idea here is to create some new points for ax_oppo and bx_oppo
	// which we can then use to place a circle or line
	struct element *ax_oppo_locally = {};
	point_from_line_and_distance(drawing, constraints, step->j, step->i, a2_local->e, a1_local->e, constraints[step->j].c2 == a2_local, &ax_oppo_locally);

	struct element *bx_oppo_locally = {};
	build_angle_point_line2(drawing, constraints, step->x, step->k, b2_local->e, b1_local->e, true, &bx_oppo_locally);

	//cx's naming is strange, the two local points are in bx and the opposing
	//point is in ax. We know that since ax was built before bx and had the two
	//local elements been in ax, we would have included the point as well.
	//
	struct element *assembly1_distance = insert_cmd(drawing, (struct command){
		.op = CMD_MEASURE_POINT_LINE_DISTANCE,
		.hidden = true,
		.result.type = ETYPE_VALUE,
		.arg1 = a1_oppo->e,
		.arg2 = c1_oppo->e,
	});

	// This line isn't really in our scope at this point. This is technically
	// dubious, but it works due to the current implementation
	struct element *cx_oppo_bx = {};
	build_angle_point_line2(drawing, constraints, step->z, step->y, c2_local->e, c1_local->e, true, &cx_oppo_bx);

	struct element *assembly2_angle = insert_cmd(drawing, (struct command){
		.op = CMD_MEASURE_LINE_LINE_ANGLE,
		.hidden = true,
		.result.type = ETYPE_VALUE,
		.arg1 = b1_oppo->e,
		.arg2 = cx_oppo_bx,
	});

	struct element* cx_oppo_direction_locally;
	{
		struct element* l = insert_cmd(drawing, (struct command){
			.op = CMD_LINE_POINT_LINE_ANGLE,
			.hidden = false,
			.result.type = ETYPE_LINE,
			.arg1 = ax_oppo_locally,
			.arg2 = bx_oppo_locally,
			.arg3 = assembly2_angle,
		});

		cx_oppo_direction_locally = insert_cmd(drawing, (struct command){
			.op = CMD_LINE_LINE_DISTANCE_PARALLEL,
			.hidden = false,
			.result.type = ETYPE_LINE,
			.root = 1,
			.arg1 = l,
			.arg2 = assembly1_distance,
		});
	}

	insert_cmd(drawing, (struct command){
		.op = CMD_IMPORT_POINT_LINE,
		.arg1 = ax_oppo_locally,
		.arg2 = cx_oppo_direction_locally,
		.d = step->assembly,
		.attachp = a1_oppo->e,
		.attachl = c1_oppo->e,
	});

	{
		struct element *local_intersection = insert_cmd(drawing, (struct command){
			.op = CMD_POINT_LINE_LINE,
			.hidden = true,
			.result.type = ETYPE_POINT,
			.arg1 = bx_oppo_locally,
			.arg2 = cx_oppo_direction_locally,
		});

		struct element *subassembly2_intersection = insert_cmd(drawing, (struct command){
			.op = CMD_POINT_LINE_LINE,
			.hidden = true,
			.result.type = ETYPE_POINT,
			.arg1 = b1_oppo->e,
			.arg2 = cx_oppo_bx,
		});

		insert_cmd(drawing, (struct command){
			.op = CMD_IMPORT_POINT_LINE,
			.arg1 = local_intersection,
			.arg2 = bx_oppo_locally,
			.d = step->assembly2,
			.attachp = subassembly2_intersection,
			.attachl = b1_oppo->e,
		});
	}
}

static void draw_solution(struct constraint *constraints, size_t fix, struct solve_step* steps, size_t steps_num, struct drawing *drawing, struct subassembly *assembly) {
	{
		assert(constraints[fix].type == CT_POINT_POINT_DISTANCE ||
			constraints[fix].type == CT_POINT_LINE_DISTANCE);
		constraints[fix].c1->e = insert_cmd(drawing, (struct command){
			.op = CMD_ORIGIN,
			.hidden = true,
			.result.type = ETYPE_POINT,
		});
		assembly->first_command = drawing->tail;

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

		if(constraints[fix].type == CT_POINT_LINE_DISTANCE) {
			static struct element right_angle = {
				.type = ETYPE_VALUE,
				.value = DEG(90),
			};

			constraints[fix].c2->e = insert_cmd(drawing, (struct command){
				.op = CMD_LINE_POINT_LINE_ANGLE,
				.hidden = true,
				.result.type = ETYPE_LINE,
				.arg1 = constraints[fix].c2->e,
				.arg2 = xaxis,
				.arg3 = &right_angle,
			});
		}
	}

	// Build the solution steps
	for(struct solve_step *step = steps; step < (steps + steps_num); step++) {
		if(step->assembly != NULL) {
			if(step->assembly2 != NULL) {
				draw_for_double_subassembly(constraints, fix, step, drawing, assembly);
				continue;
			}
			draw_for_subassembly(constraints, fix, step, drawing, assembly);
			continue;
		}
		assert(step->assembly2 == NULL);

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
				draw_point_from_2_distance(drawing, constraints, step->i, step->j, local_i, local_j, &oppo_i->e, shown);
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

			point_from_line_and_distance(drawing, constraints, step->i, step->j, local_i->e, local_j->e, constraints[step->j].c2 == local_j, &oppo_i->e);
		} else if(constraints[step->i].type == CT_POINT_POINT_DISTANCE
			&& local_i->type == COM_POINT
			&& constraints[step->j].type == CT_POINT_LINE_DISTANCE
			&& local_j->type == COM_LINE) {
			// @COPYPASTA: Taken from above but with params swapped
			assert(oppo_i->type == COM_POINT);

			point_from_line_and_distance(drawing, constraints, step->j, step->i, local_j->e, local_i->e, constraints[step->j].c2 == local_i, &oppo_i->e);
		} else {
			CRASH("Unknown constraint combination %s and %s\n", constraint_type_name[constraints[step->i].type], constraint_type_name[constraints[step->j].type]);
		}

	}

	assembly->last_command = drawing->tail;
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
		assemblies[*assemblies_num].steps = calloc(1, sizeof(struct solve_step) * constraints->length);
		assemblies[*assemblies_num].articulation = malloc(sizeof(struct component*) * constraints->length);
		assemblies[*assemblies_num].articulation_position = malloc(sizeof(struct element*) * constraints->length);
		assemblies[*assemblies_num].steps_num = build_triangles(constraints, components, dest_num, fix, *assemblies_num+1, &assemblies[*assemblies_num]);
		assemblies[*assemblies_num].fix = fix;

		// printf("Assembly %ld\n", *assemblies_num);
		// for(size_t i = 0; i < assemblies[*assemblies_num].articulation_num; i++) {
		// 	printf("  Articulation %p\n", assemblies[*assemblies_num].articulation[i]);
		// }

		// printf("Solved in %ld steps\n", steps_num);

		draw_solution(constraints->elements, fix, assemblies[*assemblies_num].steps, assemblies[*assemblies_num].steps_num, drawing, &assemblies[*assemblies_num]);
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

void reconstruct_drawing(struct constraints *constraints, struct subassembly *assemblies, size_t *assemblies_num) {
}
