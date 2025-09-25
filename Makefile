CC ?= gcc

SRCDIR ?= src
OBJDIR ?= obj

LIBS = $(shell pkg-config --libs SDL2) -Wl,-rpath,./SDL/build/ $(shell pkg-config --libs cglm)
INCS = -ISDL/build/include/SDL2 -ISDL/build/include-config-/SDL2

CFLAGS ?= -D_FORTIFY_SOURCE=2 -Wall -Werror -Wno-error=unused-variable -g -Og
CFLAGS += -std=gnu11 -fms-extensions -flto $(shell pkg-config --cflags cglm)

APP_SOURCES = $(shell find $(SRCDIR) -name "*.c")
APP_OBJS = $(APP_SOURCES:%.c=$(OBJDIR)/%.o)
APP_DEPS = $(APP_OBJS:%.o=%.d)

-include $(APP_DEPS)

# We don't really need to run the tests for bear to record them
compile_commands.json: clean Makefile $(APP_SOURCES)
	@rm -f "$@"
	bear -- make main

main: $(APP_OBJS)
	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $(APP_OBJS) $(LIBS)

$(OBJDIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(INCS) -MMD -o $@ -c $<

clean:
	@rm -rf $(OBJDIR)
	@rm -f main

.DEFAULT_GOAL := all
all: main
