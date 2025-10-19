CC ?= gcc

SRCDIR ?= src
INCDIR ?= inc
OBJDIR ?= obj
TSTDIR ?= test

LIBS = -lm
INCS = -Ithirdparty/cglm/include -Iinc/

CFLAGS ?= -D_FORTIFY_SOURCE=2 -Wall -Werror -Wno-error=unused-variable -g -Og
CFLAGS += -std=gnu11 -fms-extensions -flto

APP_MAIN_SOURCES = $(SRCDIR)/main.c
APP_MAIN_OBJS = $(APP_MAIN_SOURCES:%.c=$(OBJDIR)/%.o)
APP_MAIN_DEPS = $(APP_MAIN_OBJS:%.o=%.d)

APP_SOURCES = $(filter-out $(APP_MAIN_SOURCES),$(shell find $(SRCDIR) -name "*.c"))
APP_OBJS = $(APP_SOURCES:%.c=$(OBJDIR)/%.o)
APP_DEPS = $(APP_OBJS:%.o=%.d)

TST_SOURCES = $(shell find $(TSTDIR) -name "*.c")
TST_OBJS = $(TST_SOURCES:%.c=$(OBJDIR)/%.o)
TST_DEPS = $(TST_OBJS:%.o=%.d)
TST_APPS = $(TST_SOURCES:%.c=$(OBJDIR)/%)

-include $(APP_DEPS) $(APP_MAIN_DEPS) $(TST_DEPS)

# We don't really need to run the tests for bear to record them
compile_commands.json: clean Makefile $(APP_SOURCES)
	@rm -f "$@"
	bear -- make main

main: $(APP_MAIN_OBJS) $(APP_OBJS)
	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $(APP_MAIN_OBJS) $(APP_OBJS) $(LIBS)

$(OBJDIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(INCS) -MMD -o $@ -c $<

$(OBJDIR)/test/%: $(OBJDIR)/test/%.o $(APP_OBJS)
	@mkdir -p $(dir $@)
	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $< $(APP_OBJS) $(LIBS)

clean:
	@rm -rf $(OBJDIR)
	@rm -f main

.DEFAULT_GOAL := all
all: main
