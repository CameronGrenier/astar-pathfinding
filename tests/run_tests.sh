#!/bin/bash
# ──────────────────────────────────────────────────────────────
#  run_tests.sh — build and run the A* pathfinding test suite
#
#  Usage:  bash tests/run_tests.sh              (unit tests only)
#          bash tests/run_tests.sh benchmark    (benchmark only)
#          bash tests/run_tests.sh all          (tests + benchmark)
#          make test                            (via makefile target)
#          make benchmark                       (via makefile target)
# ──────────────────────────────────────────────────────────────
set -e

CC="${CC:-clang}"
CFLAGS="-g -Wall -Wextra -I./include"
LDFLAGS="-lm -lpthread"

LIB_SRC="src/map.c src/astar.c src/runner.c src/heap.c src/types.c"

TEST_SRC="tests/test_pathfinding.c"
TEST_BIN="tests/test_runner"

BENCH_SRC="tests/benchmark.c"
BENCH_BIN="tests/benchmark_runner"

EXIT_CODE=0

# ── Unit tests (default, or when "all" is passed) ────────────
if [ "$1" != "benchmark" ]; then
    echo "========================================================"
    echo "  Building test harness (no ncurses)"
    echo "========================================================"
    echo "  ${CC} ${CFLAGS} ${TEST_SRC} ${LIB_SRC} -o ${TEST_BIN} ${LDFLAGS}"

    ${CC} ${CFLAGS} ${TEST_SRC} ${LIB_SRC} -o ${TEST_BIN} ${LDFLAGS}

    echo "  Build succeeded."
    echo ""
    echo "========================================================"
    echo "  Running tests"
    echo "========================================================"

    ./${TEST_BIN}
    EXIT_CODE=$?

    echo ""
    if [ ${EXIT_CODE} -eq 0 ]; then
        echo "=== ALL TESTS PASSED ==="
    else
        echo "=== SOME TESTS FAILED (exit code ${EXIT_CODE}) ==="
    fi
fi

# ── Benchmark (optional, takes longer) ───────────────────────
if [ "$1" = "benchmark" ] || [ "$1" = "all" ]; then
    echo ""
    echo "========================================================"
    echo "  Building benchmark suite (no ncurses)"
    echo "========================================================"
    echo "  ${CC} ${CFLAGS} ${BENCH_SRC} ${LIB_SRC} -o ${BENCH_BIN} ${LDFLAGS}"

    ${CC} ${CFLAGS} ${BENCH_SRC} ${LIB_SRC} -o ${BENCH_BIN} ${LDFLAGS}

    echo "  Build succeeded."
    echo ""
    echo "========================================================"
    echo "  Running benchmark (this may take a while)"
    echo "========================================================"

    ./${BENCH_BIN}
    BENCH_EXIT=$?

    if [ ${BENCH_EXIT} -ne 0 ]; then
        echo "=== BENCHMARK FAILED (exit code ${BENCH_EXIT}) ==="
        EXIT_CODE=${BENCH_EXIT}
    fi
fi

exit ${EXIT_CODE}
