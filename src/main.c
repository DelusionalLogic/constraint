#include <assert.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <cglm/cglm.h>

struct parameter {
	double x;
	double y;
	bool fixed;
};

enum constraint_type {
	CONSTRAINT_DISTANCE,
	CONSTRAINT_ANGLE,
};

struct distance_params {
	double distance;

	struct parameter *p1;
	struct parameter *p2;
};
struct angle_params {
	double distance;

	struct parameter *p11;
	struct parameter *p12;
	struct parameter *p21;
	struct parameter *p22;
};
struct constraint {
	enum constraint_type type;
	bool applied;

	union {
		struct distance_params distance;
		struct angle_params angle;
	};
};

struct placement {
	struct parameter *param;
	vec2 pos;
};

struct cdset {
	struct placement p[16];
	size_t num;
};

struct caset {
	struct parameter *p11;
	struct parameter *p12;

	struct parameter *p21;
	struct parameter *p22;
};

ssize_t find_param_in_cd(struct cdset *cd, struct parameter *param) {
	for(size_t i = 0; i < cd->num; i++) {
		if(cd->p[i].param == param) return i;
	}

	return -1;
}

bool find_connecting_set(struct cdset cd[], size_t cd_count, size_t i, struct parameter *params, size_t *i_idx, size_t *i_rev, size_t *set1, size_t *set1_idx, size_t *set1_rev, size_t *set2, size_t *set2_idx, size_t *set2_rev) {
	struct cdset *base_cd = &cd[i];

	size_t first_seen[10] = {};
	size_t first_seen_idx[10] = {};
	size_t first_seen_shd[10] = {};
	size_t first_seen_i[10] = {};

	for(size_t j = 0; j < base_cd->num; j++) {
		for(size_t k = i+1; k < cd_count; k++) {
			struct cdset *candidate = &cd[k];

			ssize_t contained = find_param_in_cd(candidate, base_cd->p[j].param);

			if(contained >= 0) {
				for(size_t w = 0; w < candidate->num; w++) {
					if(contained == w) continue;

					size_t candidate_param_idx = candidate->p[w].param - params;
					if(first_seen[candidate_param_idx] != 0) {
						*i_rev = j;
						*i_idx = first_seen_i[candidate_param_idx];
						*set1 = first_seen[candidate_param_idx];
						*set1_idx = first_seen_shd[candidate_param_idx];
						*set1_rev = first_seen_idx[candidate_param_idx];
						*set2 = k;
						*set2_idx = contained;
						*set2_rev = w;
						return true;
					}

					first_seen[candidate_param_idx] = k;
					first_seen_shd[candidate_param_idx] = contained;
					first_seen_idx[candidate_param_idx] = w;
					first_seen_i[candidate_param_idx] = j;
				}
			}
		}
	}

	return false;
}

int main(int argc, char *argv[]) {
	struct parameter params[10] = {};

	struct constraint constraints[] = {
		{
			.type = CONSTRAINT_DISTANCE,
			.applied = false,
			.distance = {
				.distance = 10,
				.p1 = &params[0],
				.p2 = &params[1],
			},
		},
		{
			.type = CONSTRAINT_DISTANCE,
			.applied = false,
			.distance = {
				.distance = 10,
				.p1 = &params[1],
				.p2 = &params[2],
			},
		},
		{
			.type = CONSTRAINT_DISTANCE,
			.applied = false,
			.distance = {
				.distance = 10,
				.p1 = &params[0],
				.p2 = &params[2],
			},
		},
	};

	struct cdset cd[128] = {};
	size_t cd_cur = 0;
	struct caset ca[128] = {};
	size_t ca_cur = 0;

	for(size_t i = 0; i < sizeof(constraints)/sizeof(constraints[0]); i++) {
		struct constraint *c = &constraints[i];
		if(c->type == CONSTRAINT_DISTANCE) {
			cd[cd_cur].p[cd[cd_cur].num  ].param = c->distance.p1;
			cd[cd_cur].p[cd[cd_cur].num  ].pos[0] = 0.0;
			cd[cd_cur].p[cd[cd_cur].num++].pos[1] = 0.0;
			cd[cd_cur].p[cd[cd_cur].num  ].param = c->distance.p2;
			cd[cd_cur].p[cd[cd_cur].num  ].pos[0] = c->distance.distance;
			cd[cd_cur].p[cd[cd_cur].num++].pos[1] = 0.0;
			cd_cur++;
		} else if(c->type == CONSTRAINT_ANGLE) {
			ca[ca_cur].p11 = c->angle.p11;
			ca[ca_cur].p12 = c->angle.p12;
			ca[ca_cur].p21 = c->angle.p21;
			ca[ca_cur].p22 = c->angle.p22;
			ca_cur++;
		}
	}

	while(true) {
		for(size_t i = 0; i < cd_cur; i++) {
			struct cdset *cur = &cd[i];

			size_t i_idx;
			size_t i_rev;
			size_t set1;
			size_t set1_idx;
			size_t set1_rev;
			size_t set2;
			size_t set2_idx;
			size_t set2_rev;
			if(find_connecting_set(cd, cd_cur, i, params, &i_idx, &i_rev, &set1, &set1_idx, &set1_rev, &set2, &set2_idx, &set2_rev)) {
				// Find the parameters of the triangle the three sides have to
				// construct
				vec2 side_a;
				glm_vec2_sub(cur->p[i_rev].pos, cur->p[i_idx].pos, side_a);
				double a_len = glm_vec2_norm(side_a);

				vec2 side_b;
				glm_vec2_sub(cd[set1].p[set1_rev].pos, cd[set1].p[set1_idx].pos, side_b);
				double b_len = glm_vec2_norm(side_b);

				vec2 side_c;
				glm_vec2_sub(cd[set2].p[set2_rev].pos, cd[set2].p[set2_idx].pos, side_c);
				double c_len = glm_vec2_norm(side_c);

				double b_theta = acos((pow(c_len, 2) + pow(a_len, 2) - pow(b_len, 2)) / (2 * c_len * a_len));
				double c_theta = acos((pow(a_len, 2) + pow(b_len, 2) - pow(c_len, 2)) / (2 * a_len * b_len));

				// Build the transform matrix for set1 and set2, we don't touch
				// cur since we let that be the reference
				// Merge set1 and set2 into cur

				{
					mat3 b_transform;
					{
						vec2 scratch;
						glm_mat3_identity(b_transform);
						glm_translate2d(b_transform, cur->p[i_idx].pos);
						glm_rotate2d(b_transform, acos(glm_vec2_dot(side_b, side_a) / (b_len * a_len)) + c_theta);
						glm_vec2_negate_to(cd[set1].p[set1_idx].pos, scratch);
						glm_translate2d(b_transform, scratch);
					}

					vec3 scratch;
					scratch[2] = 1;
					for(size_t j = 0; j < cd[set1].num; j++) {
						if(j == set1_idx) continue;

						scratch[0] = cd[set1].p[j].pos[0];
						scratch[1] = cd[set1].p[j].pos[1];
						glm_mat3_mulv(b_transform, scratch, scratch);

						cur->p[cur->num  ].pos[0] = scratch[0];
						cur->p[cur->num  ].pos[1] = scratch[1];
						cur->p[cur->num++].param = cd[set1].p[j].param;
					}
					cd[set1].num = 0;
				}

				{
					mat3 c_transform;
					{
						vec2 scratch;
						glm_mat3_identity(c_transform);
						glm_translate2d(c_transform, cur->p[i_rev].pos);
						glm_rotate2d(c_transform, acos(glm_vec2_dot(side_c, side_a) / (c_len * a_len)) - b_theta);
						glm_vec2_negate_to(cd[set2].p[set2_rev].pos, scratch);
						glm_translate2d(c_transform, scratch);
					}

					vec3 scratch;
					scratch[2] = 1;
					for(size_t j = 0; j < cd[set2].num; j++) {
						if(j == set2_idx || j == set2_rev) continue;

						scratch[0] = cd[set2].p[j].pos[0];
						scratch[1] = cd[set2].p[j].pos[1];
						glm_mat3_mulv(c_transform, scratch, scratch);

						cur->p[cur->num  ].pos[0] = scratch[0];
						cur->p[cur->num  ].pos[1] = scratch[1];
						cur->p[cur->num++].param = cd[set2].p[j].param;
					}
					cd[set2].num = 0;
				}
			}
		}

		// Apply the DDA1 rule
		for(size_t i = 0; i < ca_cur; i++) {
		}

		for(size_t i = 0; i < cd_cur; i++) {
			printf("CD %d\n", i);
			for(size_t j = 0; j < cd[i].num; j++) {
				printf("p %d %f;%f\n", j, cd[i].p[j].pos[0], cd[i].p[j].pos[1]);
			}
		}
	}

	printf("%ld\n", cd_cur);
}
