CC = gcc
CFLAGS = -Wall -Wextra -O3
LDFLAGS = -lrt -lm

SRCS_A = oscivox.c
SRCS_B = blit.c

OBJS_A = $(addprefix $(BUILD_DIR)/, $(SRCS_A:.c=.o))
OBJS_B = $(addprefix $(BUILD_DIR)/, $(SRCS_B:.c=.o))

DEBUG_CFLAGS = -g -DDEBUG
DEBUG_LDFLAGS =

SOURCE_DIR ?= src
BUILD_DIR ?= build

.PHONY: all debug clean

all: $(BUILD_DIR)/oscivox $(BUILD_DIR)/blit

debug: CFLAGS += $(DEBUG_CFLAGS)
debug: LDFLAGS += $(DEBUG_LDFLAGS)
debug: all

$(BUILD_DIR)/oscivox: $(OBJS_A)
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

$(BUILD_DIR)/blit: $(OBJS_B)
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

$(BUILD_DIR)/%.o: $(SOURCE_DIR)/%.c
	mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@
	$(CC) $(CFLAGS) -MM $< | sed 's|^.*\.o|$(BUILD_DIR)/&|' > $(BUILD_DIR)/$*.d

-include $(addprefix $(BUILD_DIR)/, $(SRCS_A:.c=.d)) $(addprefix $(BUILD_DIR)/, $(SRCS_B:.c=.d))

clean:
	rm -rf $(BUILD_DIR)
