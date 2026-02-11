#include "cad/topology.h"
#include "cad/log.h"
#include "cad/util.h"

#include <string.h>

static void topo_resize(struct topology *topo, uint64_t newcapacity) {
	resize_buffer((void**)&topo->elements, &topo->capacity, newcapacity, sizeof(struct topology_elem));
}

size_t frag_len(struct topology_elem *elems) {
	struct topology_elem *cur = elems;
	while(cur->cmd.op != TOPO_END) {
		switch(cur->cmd.op) {
			case TOPO_MOVETO:
				cur += 2;
				break;
			case TOPO_LINETO:
				cur += 2;
				break;
			case TOPO_ARCTO:
				cur += 3;
				break;
			case TOPO_END:
				CRASH("We should never get to here");
		}
	}

	return cur - elems;
}

void add_fragment(struct topology *topo, struct topology_elem *new) {
	size_t new_num = frag_len(new);
	if(new_num + topo->length > topo->capacity) {
		topo_resize(topo, new_num + topo->length);
	}

	memcpy(topo->elements + topo->length, new, new_num * sizeof(struct topology_elem));
	topo->length += new_num;
}

void free_topology(struct topology *topo) {
	free(topo->elements);
	topo->elements = NULL;
	topo->length = 0;
	topo->capacity = 0;
}
