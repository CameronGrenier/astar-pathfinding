CC = clang
CFLAGS = -g -Wall -Wextra -fcolor-diagnostics -fansi-escape-codes -I./include
LDFLAGS = -lm -lpthread -lncurses

ifdef ASAN
  CFLAGS  += -fsanitize=address
  LDFLAGS += -fsanitize=address
endif

SRC = src/main.c src/map.c src/astar.c src/runner.c src/cli.c src/heap.c src/types.c src/menu.c
TARGET = astar

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) $(SRC) -o $(TARGET) $(LDFLAGS)

# ── test suite (no ncurses, no main/cli/menu) ────────────────────────
TEST_SRC = tests/test_pathfinding.c src/map.c src/astar.c src/runner.c src/heap.c src/types.c
TEST_TARGET = tests/test_runner
TEST_LDFLAGS = -lm -lpthread

$(TEST_TARGET): $(TEST_SRC)
	$(CC) $(CFLAGS) $(TEST_SRC) -o $(TEST_TARGET) $(TEST_LDFLAGS)

test: $(TEST_TARGET)
	./$(TEST_TARGET)

# ── benchmark suite (no ncurses, no main/cli/menu) ───────────────────
BENCH_SRC = tests/benchmark.c src/map.c src/astar.c src/runner.c src/heap.c src/types.c
BENCH_TARGET = tests/benchmark_runner

$(BENCH_TARGET): $(BENCH_SRC)
	$(CC) $(CFLAGS) $(BENCH_SRC) -o $(BENCH_TARGET) $(TEST_LDFLAGS)

benchmark: $(BENCH_TARGET)
	./$(BENCH_TARGET)

clean:
	rm -f $(TARGET) $(TEST_TARGET) $(BENCH_TARGET)