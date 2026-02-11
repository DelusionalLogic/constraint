#pragma once

#include <stddef.h>

#define TOPO_MOVETO(c) \
	{ .cmd = {TOPO_MOVETO} }, \
	{ .arg = {c} } \

#define TOPO_LINETO(c) \
	{ .cmd = {TOPO_LINETO} }, \
	{ .arg = {c} } \

#define TOPO_ARCTO(center, end) \
	{ .cmd = {TOPO_ARCTO} }, \
	{ .arg = {center} }, \
	{ .arg = {end} } \

#define TOPO_END() \
	{ .cmd = {TOPO_END} } \

enum topology_op {
	TOPO_MOVETO,
	TOPO_LINETO,
	TOPO_ARCTO,

	TOPO_END,
};

struct topology_cmd {
	enum topology_op op;
};

struct topology_arg {
	struct component *c;
};

struct topology_elem {
	union {
		struct topology_cmd cmd;
		struct topology_arg arg;
	};
};

struct topology {
	struct topology_elem *elements;
	size_t length;
	size_t capacity;
};

void add_fragment(struct topology *topo, struct topology_elem *new);
void free_topology(struct topology *topo);
