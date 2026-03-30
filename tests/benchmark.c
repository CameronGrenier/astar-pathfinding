/**
 * @file benchmark.c
 * @brief Comprehensive benchmark for A* multi-agent pathfinding.
 *
 * Randomly generates grids of various sizes, runs simulations with varying
 * agent counts, repeats 100 times each, and records average timing and
 * path-length data.
 *
 * Build (no ncurses):
 *   clang -g -Wall -Wextra -I./include tests/benchmark.c \
 *         src/map.c src/astar.c src/runner.c src/heap.c src/types.c \
 *         -o tests/benchmark_runner -lm -lpthread
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <stdatomic.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include <setjmp.h>

#include "types.h"
#include "map.h"
#include "runner.h"

/* ── globals required by runner.c, astar.c ───────────────────────────── */

Node **actual_map;
Node **shared_map;
Node *goal_node;
Agent *agents;
int num_agents;
Zone *zones;
int zone_size_x, zone_size_y;
int num_zones, num_zones_x, num_zones_y;
int rows, cols;
unsigned int speed_delay = 0;

/* CLI sync — runner.c signals these but no CLI thread listens */
pthread_mutex_t cli_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t cli_cond = PTHREAD_COND_INITIALIZER;
volatile atomic_int cli_done = 0;
atomic_uint cli_update_seq = 0;

/* map-changed sync — used by runner.c for inter-agent notification */
pthread_mutex_t map_changed_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t map_changed_cond = PTHREAD_COND_INITIALIZER;

/* ── benchmark parameters ────────────────────────────────────────────── */

#define NUM_ITERATIONS 100
#define OBSTACLE_DENSITY 0.15
#define RUN_TIMEOUT_SEC 30
#define TEMP_FILE_PATH "tests/bench_temp.txt"
#define CSV_FILE_PATH "tests/benchmark_results.csv"

static const int GRID_SIZES[] = {10, 30, 50, 70};
static const int AGENT_COUNTS[] = {1, 2, 4, 8};
#define NUM_GRID_SIZES ((int)(sizeof(GRID_SIZES) / sizeof(GRID_SIZES[0])))
#define NUM_AGENT_COUNTS ((int)(sizeof(AGENT_COUNTS) / sizeof(AGENT_COUNTS[0])))

/* ── per-run timeout via SIGALRM ─────────────────────────────────────── */

static volatile sig_atomic_t run_timed_out = 0;

static void run_timeout_handler(int sig)
{
  (void)sig;
  run_timed_out = 1;
}

/* ── BFS on actual_map (full knowledge) ──────────────────────────────── */

typedef struct
{
  int x, y;
} BFSCoord;

/**
 * Compute the true shortest-path length from (start_x, start_y) to the goal
 * on actual_map, ignoring other agents (ground-truth obstacles only).
 *
 * @return Number of steps, or -1 if unreachable.
 */
static int bfs_shortest_path(int start_x, int start_y)
{
  Coords goal_pos = goal_node->pos;

  if (start_x == goal_pos.x && start_y == goal_pos.y)
    return 0;

  int total = rows * cols;
  int *dist = calloc((size_t)total, sizeof(int));
  char *visited = calloc((size_t)total, sizeof(char));
  BFSCoord *queue = malloc((size_t)total * sizeof(BFSCoord));

  if (!dist || !visited || !queue)
  {
    free(dist);
    free(visited);
    free(queue);
    return -1;
  }

  int front = 0, back = 0;

  visited[start_y * cols + start_x] = 1;
  dist[start_y * cols + start_x] = 0;
  queue[back++] = (BFSCoord){start_x, start_y};

  const int dx[] = {0, 0, -1, 1};
  const int dy[] = {-1, 1, 0, 0};

  while (front < back)
  {
    BFSCoord cur = queue[front++];

    for (int d = 0; d < 4; d++)
    {
      int nx = cur.x + dx[d];
      int ny = cur.y + dy[d];

      if (nx < 0 || nx >= cols || ny < 0 || ny >= rows)
        continue;
      if (visited[ny * cols + nx])
        continue;

      NodeType type = actual_map[ny][nx].type;
      if (type == OBSTACLE)
        continue;

      visited[ny * cols + nx] = 1;
      dist[ny * cols + nx] = dist[cur.y * cols + cur.x] + 1;

      if (nx == goal_pos.x && ny == goal_pos.y)
      {
        int result = dist[ny * cols + nx];
        free(dist);
        free(visited);
        free(queue);
        return result;
      }

      queue[back++] = (BFSCoord){nx, ny};
    }
  }

  free(dist);
  free(visited);
  free(queue);
  return -1;
}

/* ── BFS reachability check (for map generation) ─────────────────────── */

/**
 * Check if (start_x, start_y) can reach (goal_x, goal_y) on a raw grid.
 *
 * @param grid      2D array [r][c], 0=empty, 1=obstacle
 * @param r         Number of rows
 * @param c         Number of columns
 * @param start_x   Start column
 * @param start_y   Start row
 * @param goal_x    Goal column
 * @param goal_y    Goal row
 * @return 1 if reachable, 0 otherwise.
 */
static int bfs_reachable(int **grid, int r, int c,
                         int start_x, int start_y,
                         int goal_x, int goal_y)
{
  if (start_x == goal_x && start_y == goal_y)
    return 1;

  int total = r * c;
  char *visited = calloc((size_t)total, sizeof(char));
  BFSCoord *queue = malloc((size_t)total * sizeof(BFSCoord));
  if (!visited || !queue)
  {
    free(visited);
    free(queue);
    return 0;
  }

  int front = 0, back = 0;
  visited[start_y * c + start_x] = 1;
  queue[back++] = (BFSCoord){start_x, start_y};

  const int dx[] = {0, 0, -1, 1};
  const int dy[] = {-1, 1, 0, 0};

  while (front < back)
  {
    BFSCoord cur = queue[front++];

    for (int d = 0; d < 4; d++)
    {
      int nx = cur.x + dx[d];
      int ny = cur.y + dy[d];

      if (nx < 0 || nx >= c || ny < 0 || ny >= r)
        continue;
      if (visited[ny * c + nx])
        continue;
      if (grid[ny][nx] == 1)
        continue;

      if (nx == goal_x && ny == goal_y)
      {
        free(visited);
        free(queue);
        return 1;
      }

      visited[ny * c + nx] = 1;
      queue[back++] = (BFSCoord){nx, ny};
    }
  }

  free(visited);
  free(queue);
  return 0;
}

/* ── random map generation ───────────────────────────────────────────── */

/**
 * Generate a random solvable map file at TEMP_FILE_PATH.
 *
 * @param r           Number of grid rows
 * @param c           Number of grid columns
 * @param n_agents    Number of agents
 * @return 1 on success, 0 on failure (should not happen in practice).
 */
static int generate_random_map(int r, int c, int n_agents)
{
  int max_attempts = 200;

  for (int attempt = 0; attempt < max_attempts; attempt++)
  {
    /* allocate grid */
    int **grid = malloc((size_t)r * sizeof(int *));
    for (int i = 0; i < r; i++)
    {
      grid[i] = calloc((size_t)c, sizeof(int));
    }

    /* place obstacles at ~15% density */
    for (int y = 0; y < r; y++)
    {
      for (int x = 0; x < c; x++)
      {
        if ((rand() % 100) < (int)(OBSTACLE_DENSITY * 100))
          grid[y][x] = 1;
      }
    }

    /* collect non-obstacle cells */
    int *free_cells_x = malloc((size_t)(r * c) * sizeof(int));
    int *free_cells_y = malloc((size_t)(r * c) * sizeof(int));
    int n_free = 0;

    for (int y = 0; y < r; y++)
    {
      for (int x = 0; x < c; x++)
      {
        if (grid[y][x] == 0)
        {
          free_cells_x[n_free] = x;
          free_cells_y[n_free] = y;
          n_free++;
        }
      }
    }

    /* need at least n_agents + 1 free cells (agents + goal) */
    if (n_free < n_agents + 1)
    {
      for (int i = 0; i < r; i++)
        free(grid[i]);
      free(grid);
      free(free_cells_x);
      free(free_cells_y);
      continue;
    }

    /* Fisher-Yates shuffle to pick random non-obstacle positions */
    for (int i = n_free - 1; i > 0; i--)
    {
      int j = rand() % (i + 1);
      int tmp;
      tmp = free_cells_x[i];
      free_cells_x[i] = free_cells_x[j];
      free_cells_x[j] = tmp;
      tmp = free_cells_y[i];
      free_cells_y[i] = free_cells_y[j];
      free_cells_y[j] = tmp;
    }

    /* assign positions: first n_agents for agents, next for goal */
    int *agent_x = malloc((size_t)n_agents * sizeof(int));
    int *agent_y = malloc((size_t)n_agents * sizeof(int));
    for (int i = 0; i < n_agents; i++)
    {
      agent_x[i] = free_cells_x[i];
      agent_y[i] = free_cells_y[i];
    }
    int gx = free_cells_x[n_agents];
    int gy = free_cells_y[n_agents];

    /* BFS reachability: every agent must reach the goal */
    int solvable = 1;
    for (int i = 0; i < n_agents; i++)
    {
      if (!bfs_reachable(grid, r, c, agent_x[i], agent_y[i], gx, gy))
      {
        solvable = 0;
        break;
      }
    }

    if (!solvable)
    {
      for (int i = 0; i < r; i++)
        free(grid[i]);
      free(grid);
      free(free_cells_x);
      free(free_cells_y);
      free(agent_x);
      free(agent_y);
      continue;
    }

    /* write the input file */
    FILE *f = fopen(TEMP_FILE_PATH, "w");
    if (!f)
    {
      for (int i = 0; i < r; i++)
        free(grid[i]);
      free(grid);
      free(free_cells_x);
      free(free_cells_y);
      free(agent_x);
      free(agent_y);
      return 0;
    }

    fprintf(f, "%d %d\n", r, c);
    fprintf(f, "%d\n", n_agents);
    for (int i = 0; i < n_agents; i++)
    {
      fprintf(f, "%d %d\n", agent_x[i], agent_y[i]);
    }
    fprintf(f, "%d %d\n", gx, gy);

    /* write grid top-to-bottom (row r-1 down to row 0) */
    for (int row = r - 1; row >= 0; row--)
    {
      for (int col = 0; col < c; col++)
      {
        fprintf(f, "%d", grid[row][col]);
      }
      fprintf(f, "\n");
    }

    fclose(f);

    /* cleanup */
    for (int i = 0; i < r; i++)
      free(grid[i]);
    free(grid);
    free(free_cells_x);
    free(free_cells_y);
    free(agent_x);
    free(agent_y);

    return 1;
  }

  return 0; /* failed after max_attempts */
}

/* ── simulation cleanup ──────────────────────────────────────────────── */

static void cleanup_sim(void)
{
  for (int i = 0; i < num_agents; i++)
    free(agents[i].path_history);

  for (int i = 0; i < rows; i++)
  {
    free(actual_map[i]);
    free(shared_map[i]);
  }
  free(actual_map);
  free(shared_map);
  free(agents);

  for (int i = 0; i < num_zones; i++)
  {
    pthread_mutex_destroy(&zones[i].mutex);
    pthread_cond_destroy(&zones[i].cond);
  }
  free(zones);
}

/**
 * Re-initialise the global synchronisation primitives between runs
 * to avoid stale state.
 */
static void reset_sync_primitives(void)
{
  pthread_mutex_destroy(&cli_mutex);
  pthread_mutex_init(&cli_mutex, NULL);
  pthread_cond_destroy(&cli_cond);
  pthread_cond_init(&cli_cond, NULL);
  pthread_mutex_destroy(&map_changed_mutex);
  pthread_mutex_init(&map_changed_mutex, NULL);
  pthread_cond_destroy(&map_changed_cond);
  pthread_cond_init(&map_changed_cond, NULL);
}

/* ── per-run result ──────────────────────────────────────────────────── */

typedef struct
{
  double time_ms;
  int total_actual_steps;
  int total_optimal_steps;
  double overhead_ratio;
  int all_reached_goal;
} RunResult;

/* ── aggregated benchmark result for one (grid_size, agent_count) ────── */

typedef struct
{
  double avg_time_ms;
  double min_time_ms;
  double max_time_ms;
  double avg_actual_steps;
  double avg_optimal_steps;
  double avg_overhead;
  double success_rate;
} BenchmarkResult;

/* ── run a single simulation ─────────────────────────────────────────── */

/**
 * Run one simulation from a map file. Returns the result.
 * On timeout, returns with all_reached_goal = 0.
 */
static RunResult run_simulation(const char *filepath)
{
  RunResult result = {0.0, 0, 0, 0.0, 0};

  /* initialise simulation from input file */
  SimConfig config;
  init(&config, filepath);

  /* copy config values into globals */
  actual_map = config.actual_map;
  shared_map = config.shared_map;
  agents = config.agents;
  num_agents = config.num_agents;
  goal_node = config.goal_node;
  zones = config.zones;
  zone_size_x = config.zone_size_x;
  zone_size_y = config.zone_size_y;
  num_zones = config.num_zones;
  num_zones_x = config.num_zones_x;
  num_zones_y = config.num_zones_y;
  rows = config.rows;
  cols = config.cols;
  speed_delay = 0;

  /* reset CLI sync state */
  atomic_store(&cli_done, 0);
  atomic_store(&cli_update_seq, 0);

  /* compute BFS optimal for each agent before running */
  int *optimal = malloc((size_t)num_agents * sizeof(int));
  for (int i = 0; i < num_agents; i++)
  {
    Coords start = agents[i].start_node->pos;
    optimal[i] = bfs_shortest_path(start.x, start.y);
  }

  /* arm timeout */
  run_timed_out = 0;
  struct sigaction sa, old_sa;
  memset(&sa, 0, sizeof(sa));
  sa.sa_handler = run_timeout_handler;
  sa.sa_flags = 0;
  sigemptyset(&sa.sa_mask);
  sigaction(SIGALRM, &sa, &old_sa);
  alarm(RUN_TIMEOUT_SEC);

  /* start timing */
  struct timespec t_start, t_end;
  clock_gettime(CLOCK_MONOTONIC, &t_start);

  /* spawn agent threads */
  pthread_t *threads = malloc((size_t)num_agents * sizeof(pthread_t));
  for (int i = 0; i < num_agents; i++)
    pthread_create(&threads[i], NULL, runner, &agents[i]);

  /* join all agent threads */
  for (int i = 0; i < num_agents; i++)
  {
    if (run_timed_out)
    {
      /* cancel remaining threads on timeout */
      for (int j = i; j < num_agents; j++)
        pthread_cancel(threads[j]);
      for (int j = i; j < num_agents; j++)
        pthread_join(threads[j], NULL);
      break;
    }
    /* use a polling join to detect timeout while waiting */
    struct timespec join_wait;
    while (1)
    {
      clock_gettime(CLOCK_REALTIME, &join_wait);
      join_wait.tv_sec += 1;
#ifdef __APPLE__
      /* macOS doesn't have pthread_timedjoin_np, use non-blocking check */
      int join_ret = pthread_join(threads[i], NULL);
      if (join_ret == 0)
        break;
#else
      int join_ret = pthread_timedjoin_np(threads[i], NULL, &join_wait);
      if (join_ret == 0)
        break;
#endif
      if (run_timed_out)
      {
        for (int j = i; j < num_agents; j++)
          pthread_cancel(threads[j]);
        for (int j = i; j < num_agents; j++)
          pthread_join(threads[j], NULL);
        goto join_done;
      }
#ifdef __APPLE__
      break; /* pthread_join already completed or errored */
#endif
    }
  }
join_done:

  /* stop timing */
  clock_gettime(CLOCK_MONOTONIC, &t_end);
  alarm(0);                          /* cancel pending alarm */
  sigaction(SIGALRM, &old_sa, NULL); /* restore old handler */

  double elapsed_ms = (t_end.tv_sec - t_start.tv_sec) * 1000.0 +
                      (t_end.tv_nsec - t_start.tv_nsec) / 1e6;
  result.time_ms = elapsed_ms;

  /* check if timed out */
  if (run_timed_out)
  {
    result.all_reached_goal = 0;
    result.total_actual_steps = 0;
    result.total_optimal_steps = 0;
    result.overhead_ratio = 0.0;
    free(threads);
    free(optimal);
    cleanup_sim();
    reset_sync_primitives();
    return result;
  }

  /* collect results */
  Coords goal_pos = goal_node->pos;
  result.all_reached_goal = 1;
  result.total_actual_steps = 0;
  result.total_optimal_steps = 0;

  for (int i = 0; i < num_agents; i++)
  {
    int actual_steps = agents[i].path_len > 0 ? agents[i].path_len - 1 : 0;
    result.total_actual_steps += actual_steps;
    result.total_optimal_steps += (optimal[i] >= 0 ? optimal[i] : 0);

    /* verify agent reached goal */
    if (agents[i].path_len == 0 ||
        agents[i].path_history[agents[i].path_len - 1].x != goal_pos.x ||
        agents[i].path_history[agents[i].path_len - 1].y != goal_pos.y)
    {
      result.all_reached_goal = 0;
    }
  }

  result.overhead_ratio = result.total_optimal_steps > 0
                              ? (double)result.total_actual_steps / (double)result.total_optimal_steps
                              : 0.0;

  free(threads);
  free(optimal);
  cleanup_sim();
  reset_sync_primitives();

  return result;
}

/* ── main benchmark loop ─────────────────────────────────────────────── */

int main(void)
{
  srand((unsigned int)time(NULL));

  printf("========================================================\n");
  printf("    A* Multi-Agent Pathfinding — Benchmark Suite\n");
  printf("========================================================\n");
  printf("  Grid sizes:   ");
  for (int i = 0; i < NUM_GRID_SIZES; i++)
    printf("%d%s", GRID_SIZES[i], i < NUM_GRID_SIZES - 1 ? ", " : "\n");
  printf("  Agent counts: ");
  for (int i = 0; i < NUM_AGENT_COUNTS; i++)
    printf("%d%s", AGENT_COUNTS[i], i < NUM_AGENT_COUNTS - 1 ? ", " : "\n");
  printf("  Iterations:   %d per configuration\n", NUM_ITERATIONS);
  printf("  Run timeout:  %d seconds\n", RUN_TIMEOUT_SEC);
  printf("  Obstacle %%:   %.0f%%\n", OBSTACLE_DENSITY * 100);
  printf("========================================================\n\n");

  /* open CSV file */
  FILE *csv = fopen(CSV_FILE_PATH, "w");
  if (!csv)
  {
    fprintf(stderr, "ERROR: cannot open %s for writing\n", CSV_FILE_PATH);
    return 1;
  }
  fprintf(csv, "grid_size,num_agents,avg_time_ms,min_time_ms,max_time_ms,"
               "avg_actual_steps,avg_optimal_steps,avg_overhead,success_rate\n");

  /* print header for stdout table */
  printf("%-10s %-8s %-12s %-12s %-12s %-14s %-14s %-12s %-10s\n",
         "Grid", "Agents", "Avg ms", "Min ms", "Max ms",
         "Avg Actual", "Avg Optimal", "Avg Overhead", "Success %");
  printf("---------- -------- ------------ ------------ ------------ "
         "-------------- -------------- ------------ ----------\n");

  for (int gi = 0; gi < NUM_GRID_SIZES; gi++)
  {
    for (int ai = 0; ai < NUM_AGENT_COUNTS; ai++)
    {
      int grid_size = GRID_SIZES[gi];
      int n_agents = AGENT_COUNTS[ai];

      /* skip configurations where agents >= cells */
      if (n_agents >= grid_size * grid_size)
      {
        printf("%-10d %-8d SKIPPED (too many agents for grid)\n",
               grid_size, n_agents);
        continue;
      }

      double sum_time = 0.0;
      double min_time = 1e12;
      double max_time = 0.0;
      double sum_actual = 0.0;
      double sum_optimal = 0.0;
      double sum_overhead = 0.0;
      int successes = 0;
      int valid_runs = 0;

      for (int iter = 0; iter < NUM_ITERATIONS; iter++)
      {
        /* seed each iteration with different value */
        srand((unsigned int)time(NULL) ^ (unsigned int)((gi * 1000 + ai * 100 + iter) * 31));

        /* generate a random solvable map */
        if (!generate_random_map(grid_size, grid_size, n_agents))
        {
          fprintf(stderr, "WARNING: failed to generate map for "
                          "%dx%d with %d agents (iter %d)\n",
                  grid_size, grid_size, n_agents, iter);
          continue;
        }

        /* run simulation */
        RunResult rr = run_simulation(TEMP_FILE_PATH);
        valid_runs++;

        sum_time += rr.time_ms;
        if (rr.time_ms < min_time)
          min_time = rr.time_ms;
        if (rr.time_ms > max_time)
          max_time = rr.time_ms;

        if (rr.all_reached_goal)
        {
          successes++;
          sum_actual += rr.total_actual_steps;
          sum_optimal += rr.total_optimal_steps;
          sum_overhead += rr.overhead_ratio;
        }
      }

      /* compute averages */
      BenchmarkResult br;
      if (valid_runs > 0)
      {
        br.avg_time_ms = sum_time / valid_runs;
        br.min_time_ms = min_time;
        br.max_time_ms = max_time;
      }
      else
      {
        br.avg_time_ms = 0.0;
        br.min_time_ms = 0.0;
        br.max_time_ms = 0.0;
      }

      if (successes > 0)
      {
        br.avg_actual_steps = sum_actual / successes;
        br.avg_optimal_steps = sum_optimal / successes;
        br.avg_overhead = sum_overhead / successes;
      }
      else
      {
        br.avg_actual_steps = 0.0;
        br.avg_optimal_steps = 0.0;
        br.avg_overhead = 0.0;
      }

      br.success_rate = valid_runs > 0
                            ? (double)successes / (double)valid_runs * 100.0
                            : 0.0;

      /* print to stdout */
      printf("%-10d %-8d %-12.2f %-12.2f %-12.2f %-14.1f %-14.1f %-12.2f %-10.1f\n",
             grid_size, n_agents,
             br.avg_time_ms, br.min_time_ms, br.max_time_ms,
             br.avg_actual_steps, br.avg_optimal_steps,
             br.avg_overhead, br.success_rate);

      /* write CSV row */
      fprintf(csv, "%d,%d,%.2f,%.2f,%.2f,%.1f,%.1f,%.2f,%.1f\n",
              grid_size, n_agents,
              br.avg_time_ms, br.min_time_ms, br.max_time_ms,
              br.avg_actual_steps, br.avg_optimal_steps,
              br.avg_overhead, br.success_rate);
      fflush(csv);
    }
  }

  fclose(csv);

  /* clean up temp file */
  remove(TEMP_FILE_PATH);

  printf("\n========================================================\n");
  printf("  Benchmark complete. Results saved to %s\n", CSV_FILE_PATH);
  printf("========================================================\n");

  return 0;
}
