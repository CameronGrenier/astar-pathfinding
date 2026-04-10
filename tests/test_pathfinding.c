/**
 * @file test_pathfinding.c
 * @brief Test harness for A* multi-agent pathfinding.
 *
 * Compares each agent's actual path length (with fog-of-war) against the
 * BFS-computed optimal shortest path on the fully-known actual_map.
 *
 * Build without ncurses — links map.c, astar.c, runner.c, heap.c, types.c
 * but NOT main.c, cli.c, or menu.c.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <stdatomic.h>
#include <signal.h>
#include <unistd.h>

#include "types.h"
#include "map.h"
#include "runner.h"
#include "astar.h"

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
unsigned int speed_delay = 0; /* realtime — no sleeps */
enum Heuristic selected_heuristic = MANHATTAN;

/* CLI sync — runner.c signals these but no CLI thread listens */
pthread_mutex_t cli_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t cli_cond = PTHREAD_COND_INITIALIZER;
volatile atomic_int cli_done = 0;
atomic_uint cli_update_seq = 0;

/* map-changed sync — used by runner.c for inter-agent notification */
pthread_mutex_t map_changed_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t map_changed_cond = PTHREAD_COND_INITIALIZER;

/* ── timeout safety net ──────────────────────────────────────────────── */

#define TEST_TIMEOUT_SEC 60

static void timeout_handler(int sig)
{
  (void)sig;
  fprintf(stderr,
          "\n*** TEST TIMEOUT (%d s) — possible deadlock ***\n",
          TEST_TIMEOUT_SEC);
  _exit(2);
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
  return -1; /* no path */
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
 * Re-initialise the global synchronisation primitives between test runs
 * to avoid stale state from previous tests.
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

/* ── per-test result ─────────────────────────────────────────────────── */

typedef struct
{
  const char *filename;
  int all_reached_goal;
  int agent_count;
  int *optimal; /* BFS optimal path lengths  */
  int *actual;  /* actual path lengths taken  */
} TestResult;

/* ── run one test scenario ───────────────────────────────────────────── */

static TestResult run_test(const char *input_file)
{
  TestResult result;
  result.filename = input_file;
  result.all_reached_goal = 1;
  result.agent_count = 0;
  result.optimal = NULL;
  result.actual = NULL;

  /* verify file exists before passing to init() */
  FILE *f = fopen(input_file, "r");
  if (!f)
  {
    fprintf(stderr, "ERROR: cannot open test file: %s\n", input_file);
    result.all_reached_goal = 0;
    return result;
  }
  fclose(f);

  /* initialise simulation from input file */
  SimConfig config;
  init(&config, input_file);

  /* copy config values into globals (mirrors main.c) */
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

  result.agent_count = num_agents;
  result.optimal = malloc((size_t)num_agents * sizeof(int));
  result.actual = malloc((size_t)num_agents * sizeof(int));

  /* ── BFS optimal for each agent (full-knowledge ground truth) ──── */
  for (int i = 0; i < num_agents; i++)
  {
    Coords start = agents[i].start_node->pos;
    result.optimal[i] = bfs_shortest_path(start.x, start.y);
  }

  /* ── spawn agent threads (no CLI thread) ─────────────────────── */
  pthread_t *threads = malloc((size_t)num_agents * sizeof(pthread_t));
  for (int i = 0; i < num_agents; i++)
    pthread_create(&threads[i], NULL, runner, &agents[i]);

  /* ── join all agent threads ──────────────────────────────────── */
  for (int i = 0; i < num_agents; i++)
    pthread_join(threads[i], NULL);
  free(threads);

  /* ── collect results ─────────────────────────────────────────── */
  Coords goal_pos = goal_node->pos;
  for (int i = 0; i < num_agents; i++)
  {
    /* path_len includes the starting position, so steps = path_len - 1 */
    result.actual[i] = agents[i].path_len > 0
                           ? agents[i].path_len - 1
                           : 0;

    /* verify agent actually reached the goal */
    if (agents[i].path_len == 0 ||
        agents[i].path_history[agents[i].path_len - 1].x != goal_pos.x ||
        agents[i].path_history[agents[i].path_len - 1].y != goal_pos.y)
    {
      result.all_reached_goal = 0;
    }
  }

  /* ── tear down ───────────────────────────────────────────────── */
  cleanup_sim();
  reset_sync_primitives();

  return result;
}

/* ── formatted report ────────────────────────────────────────────────── */

static void print_report(TestResult *result)
{
  printf("\n");
  printf("========================================================\n");
  printf("  Test: %s\n", result->filename);
  printf("========================================================\n");

  if (result->agent_count == 0)
  {
    printf("  (no agents — skipped or file error)\n");
    printf("========================================================\n");
    return;
  }

  printf("  Agent  | BFS Optimal | Actual Steps | Overhead\n");
  printf("  -------+-------------+--------------+----------\n");

  int total_optimal = 0;
  int total_actual = 0;

  for (int i = 0; i < result->agent_count; i++)
  {
    double ratio = result->optimal[i] > 0
                       ? (double)result->actual[i] / (double)result->optimal[i]
                       : 0.0;
    printf("     %d   |     %3d     |     %3d      |  %.2fx\n",
           i, result->optimal[i], result->actual[i], ratio);
    total_optimal += result->optimal[i];
    total_actual += result->actual[i];
  }

  printf("  -------+-------------+--------------+----------\n");
  double total_ratio = total_optimal > 0
                           ? (double)total_actual / (double)total_optimal
                           : 0.0;
  printf("  Total  |     %3d     |     %3d      |  %.2fx\n",
         total_optimal, total_actual, total_ratio);
  printf("========================================================\n");
  printf("  All agents reached goal: %s\n",
         result->all_reached_goal ? "YES" : "NO");
  printf("========================================================\n");
}

/* ── main ────────────────────────────────────────────────────────────── */

int main(void)
{
  /* arm a timeout to abort if threads deadlock */
  signal(SIGALRM, timeout_handler);
  alarm(TEST_TIMEOUT_SEC);

  const char *test_files[] = {
      "tests/test_1_agent.txt",
      "tests/test_2_agents.txt",
      "tests/test_3_agents.txt",
      "tests/test_4_agents.txt",
  };
  int num_tests = (int)(sizeof(test_files) / sizeof(test_files[0]));

  printf("========================================================\n");
  printf("    A* Multi-Agent Pathfinding — Test Suite\n");
  printf("========================================================\n");

  /* test both heuristics */
  const char *heuristic_names[] = {"Manhattan", "Euclidean"};
  const enum Heuristic heuristics[] = {MANHATTAN, EUCLIDEAN};

  int all_passed = 1;

  for (int h = 0; h < 2; h++)
  {
    selected_heuristic = heuristics[h];

    printf("\n========================================================\n");
    printf("  Testing with %s heuristic\n", heuristic_names[h]);
    printf("========================================================\n");

    /* generate CSV filename for this heuristic */
    char csv_filename[256];
    snprintf(csv_filename, sizeof(csv_filename),
             "tests/test_results_%s.csv",
             heuristic_names[h]);

    /* open CSV file for this heuristic */
    FILE *csv = fopen(csv_filename, "w");
    if (!csv)
    {
      fprintf(stderr, "ERROR: cannot open %s for writing\n", csv_filename);
      return 1;
    }
    fprintf(csv, "test_file,agents,optimal_steps,actual_steps,overhead_ratio,all_reached_goal\n");

    TestResult results[4];
    int heuristic_passed = 1;

    for (int t = 0; t < num_tests; t++)
    {
      printf("\nRunning test %d/%d: %s ...\n",
             t + 1, num_tests, test_files[t]);
      results[t] = run_test(test_files[t]);
      print_report(&results[t]);

      if (!results[t].all_reached_goal)
        heuristic_passed = 0;

      /* write CSV row for each test */
      if (results[t].agent_count > 0)
      {
        int total_optimal = 0;
        int total_actual = 0;
        for (int i = 0; i < results[t].agent_count; i++)
        {
          total_optimal += results[t].optimal[i];
          total_actual += results[t].actual[i];
        }
        double overhead = total_optimal > 0
                              ? (double)total_actual / (double)total_optimal
                              : 0.0;
        fprintf(csv, "%s,%d,%d,%d,%.2f,%s\n",
                test_files[t],
                results[t].agent_count,
                total_optimal,
                total_actual,
                overhead,
                results[t].all_reached_goal ? "YES" : "NO");
      }
    }

    fclose(csv);

    printf("\n========================================================\n");
    printf("  %s heuristic test results saved to %s\n",
           heuristic_names[h], csv_filename);
    printf("========================================================\n");

    if (!heuristic_passed)
      all_passed = 0;

    /* free result arrays */
    for (int t = 0; t < num_tests; t++)
    {
      free(results[t].optimal);
      free(results[t].actual);
    }
  }

  /* ── summary ─────────────────────────────────────────────────── */
  printf("\n");
  printf("========================================================\n");
  printf("                    FINAL SUMMARY\n");
  printf("========================================================\n");
  printf("  All tests: %s\n",
         all_passed ? "ALL TESTS PASSED" : "SOME TESTS FAILED");
  printf("  CSV files generated:\n");
  printf("    - tests/test_results_Manhattan.csv\n");
  printf("    - tests/test_results_Euclidean.csv\n");
  printf("========================================================\n");

  return all_passed ? 0 : 1;
}
