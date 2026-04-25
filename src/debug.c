#include "cad/debug.h"

static const char *component_type_name[] = {
	[COM_POINT] = "point",
	[COM_LINE] = "line",
};

void print_subassemblies(struct constraints *c, struct subassembly *assemblies, size_t assemblies_num) {
	fprintf(stderr, "=== Subassemblies: %zu ===\n", assemblies_num);
	for(size_t i = 0; i < assemblies_num; i++) {
		struct subassembly *a = &assemblies[i];
		fprintf(stderr, "Assembly %zu: %zu steps, fix=%zu, fixed=%s\n",
			i, a->steps_num, a->fix, a->fixed ? "yes" : "no");

		if(a->articulation_num > 0) {
			fprintf(stderr, "  Articulations (%zu):\n", a->articulation_num);
			for(size_t j = 0; j < a->articulation_num; j++) {
				struct component *comp = a->articulation[j];
				fprintf(stderr, "    [%zu] %s %p\n", j, component_type_name[comp->type], (void*)comp);
			}
		}

		fprintf(stderr, "  Constraints used:\n");
		for(size_t j = 0; j < c->length; j++) {
			struct constraint *ct = &c->elements[j];
			if(ct->used != i + 1) continue;
			fprintf(stderr, "    [%zu] %s v=%.2f (%s %p, %s %p)\n",
				j, constraint_type_name[ct->type], ct->v,
				component_type_name[ct->c1->type], (void*)ct->c1,
				component_type_name[ct->c2->type], (void*)ct->c2);
		}
	}

	fprintf(stderr, "Unused constraints:\n");
	for(size_t j = 0; j < c->length; j++) {
		struct constraint *ct = &c->elements[j];
		if(ct->used != 0) continue;
		fprintf(stderr, "  [%zu] %s v=%.2f (%s %p, %s %p)\n",
			j, constraint_type_name[ct->type], ct->v,
			component_type_name[ct->c1->type], (void*)ct->c1,
			component_type_name[ct->c2->type], (void*)ct->c2);

		for(size_t i = 0; i < assemblies_num; i++) {
			struct subassembly *a = &assemblies[i];
			for(size_t k = 0; k < a->articulation_num; k++) {
				if(a->articulation[k] == ct->c1)
					goto articulation_found;
				if(a->articulation[k] == ct->c2)
					goto articulation_found;
			}
			// Constraint did not touch one of the articulations
			continue;
articulation_found:
			fprintf(stderr, "    Connects to %zu\n", i);
		}
	}
	fprintf(stderr, "=========================\n");
}
