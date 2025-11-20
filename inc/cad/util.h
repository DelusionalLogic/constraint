#pragma once

#include <assert.h>
#include <limits.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

static void resize_buffer(void **buffer, size_t *capacity, size_t elems, size_t elemSize) {
	assert(elems > 0);
	uint64_t newpower = (sizeof(elems) * CHAR_BIT) - __builtin_clzl(elems-1);
	elems = 1 << newpower;

	if(elems != *capacity) {
		*capacity = elems;
		*buffer = realloc(*buffer, *capacity * elemSize);
	}
}

