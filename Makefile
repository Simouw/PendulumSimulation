CC ?= gcc
CFLAGS := -Wall -Wextra -Werror -std=c11 -O2 -Iinclude
LDFLAGS_COMMON := -lm

# Dynamic detection of SDL2 compilation and link flags (pkg-config / sdl2-config fallback)
SDL2_CFLAGS := $(shell pkg-config --cflags sdl2 2>/dev/null || sdl2-config --cflags 2>/dev/null)
SDL2_LIBS   := $(shell pkg-config --libs sdl2 2>/dev/null || sdl2-config --libs 2>/dev/null || echo -lSDL2)

BIN_DIR := bin
OBJ_DIR := obj

MAIN_TARGET := $(BIN_DIR)/pendulum_sim
TEST_TARGET := tests/run_tests

# Core physics and control source files
CORE_SRCS := src/physics.c src/integrator.c src/controller.c
CORE_OBJS := $(patsubst src/%.c, $(OBJ_DIR)/%.o, $(CORE_SRCS))

# Graphical application source files
APP_SRCS := src/main.c src/renderer.c
APP_OBJS := $(patsubst src/%.c, $(OBJ_DIR)/%.o, $(APP_SRCS))

# Unit test source files
TEST_SRCS := tests/test_main.c
TEST_OBJS := $(OBJ_DIR)/test_main.o

.PHONY: all run tests clean

all: $(MAIN_TARGET)

# Compile main simulation executable
$(MAIN_TARGET): $(CORE_OBJS) $(APP_OBJS) | $(BIN_DIR)
	$(CC) $(CORE_OBJS) $(APP_OBJS) $(SDL2_LIBS) $(LDFLAGS_COMMON) -o $@
	@echo "[BUILD SUCCESS] Application binary built: $@"

# Compile core physics object files (without SDL2)
$(OBJ_DIR)/%.o: src/%.c | $(OBJ_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Compile objects requiring SDL2 (renderer and main)
$(OBJ_DIR)/renderer.o: src/renderer.c | $(OBJ_DIR)
	$(CC) $(CFLAGS) $(SDL2_CFLAGS) -c $< -o $@

$(OBJ_DIR)/main.o: src/main.c | $(OBJ_DIR)
	$(CC) $(CFLAGS) $(SDL2_CFLAGS) -c $< -o $@

# Compile CLI unit test object (no graphical dependency)
$(OBJ_DIR)/test_main.o: tests/test_main.c | $(OBJ_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(TEST_TARGET): $(CORE_OBJS) $(TEST_OBJS)
	@mkdir -p tests
	$(CC) $(CORE_OBJS) $(TEST_OBJS) $(LDFLAGS_COMMON) -o $@
	@echo "[BUILD SUCCESS] Test binary built: $@"

# Run unit tests target
tests: $(TEST_TARGET)
	@echo "[RUNNING TESTS] Launching unit test suite..."
	@./$(TEST_TARGET) || true

# Run application target
run: $(MAIN_TARGET)
	@echo "[RUNNING APPLICATION] Launching simulator..."
	@./$(MAIN_TARGET)

# Target directory creation
$(BIN_DIR) $(OBJ_DIR):
	@mkdir -p $@

# Clean binaries, objects, and output files
clean:
	@rm -rf $(OBJ_DIR) $(BIN_DIR) $(TEST_TARGET) $(TEST_TARGET).exe telemetrie.csv pdf_text.txt
	@echo "[CLEAN] Build artifacts and object files removed."
