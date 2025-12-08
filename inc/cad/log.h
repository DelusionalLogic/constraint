#pragma once

#include <stdio.h>

#define LOG(fmt, ...) \
	fprintf(stderr, "%s:%d: " fmt, __FILE__, __LINE__, ##__VA_ARGS__);

#define CRASH(fmt, ...) \
	fprintf(stderr, "%s:%d: " fmt, __FILE__, __LINE__, ##__VA_ARGS__); \
	abort();
