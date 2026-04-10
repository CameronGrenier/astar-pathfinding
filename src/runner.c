#include "runner.h"
#include "cli.h"
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#define REALTIME_TIMEOUT_US 5000 /* 5 ms — safety net for missed signals */

extern Node **shared_map;
extern Node *goal_node;
extern int zone_size_x, zone_size_y, num_zones_x, num_zones_y, num_zones;
extern Zone *zones;
extern unsigned int speed_delay;
extern enum Heuristic selected_heuristic;

extern pthread_mutex_t map_changed_mutex;
extern pthread_cond_t map_changed_cond;

#include <time.h>
#include <sys/time.h>

/* -- path history helper ---------------------------------------------- */

/**
 * @brief Append a position to the agent's path history, growing the buffer
 *        if necessary.
 */
static void append_path(Agent *agent, Coords pos)
{
  if (agent->path_len >= agent->path_cap)
  {
    int new_cap = agent->path_cap == 0 ? 64 : agent->path_cap * 2;
    Coords *new_hist = realloc(agent->path_history,
                               (size_t)new_cap * sizeof(Coords));
    if (new_hist == NULL)
    {
      perror("Failed to reallocate path_history");
      return;
    }
    agent->path_cap = new_cap;
    agent->path_history = new_hist;
  }
  agent->path_history[agent->path_len++] = pos;
}

/* -- zone helpers ----------------------------------------------------- */

/**
 * @brief Check if a position falls within a zone's (overlapping) bounds.
 */
static int pos_in_zone(Zone *zone, Coords pos)
{
  return pos.x >= zone->top_left.x && pos.x <= zone->bottom_right.x &&
         pos.y >= zone->top_left.y && pos.y <= zone->bottom_right.y;
}

/**
 * @brief Find all zones that contain a given position.
 *
 * Because zones overlap by 25%, a position near a boundary may belong
 * to 1, 2, or 4 zones simultaneously.
 *
 * @param pos        The grid position to query.
 * @param out_zones  Output array of zone pointers (caller must provide space for at least 4).
 * @return           Number of zones found.
 */
static int find_zones(Coords pos, Zone *out_zones[4])
{
  int count = 0;
  for (int i = 0; i < num_zones && count < 4; i++)
  {
    if (pos_in_zone(&zones[i], pos))
    {
      out_zones[count++] = &zones[i];
    }
  }
  return count;
}

/**
 * @brief Collect the unique set of zones covering both current and next positions.
 *
 * Merges the zone sets for two positions, deduplicates, and sorts by zone id
 * (ascending) for deadlock-free ordered locking.
 *
 * @param cur_pos    Current agent position.
 * @param next_pos   Next move position.
 * @param out_zones  Output array (caller provides space for at least 8).
 * @return           Number of unique zones.
 */
static int collect_zones(Coords cur_pos, Coords next_pos, Zone *out_zones[8])
{
  Zone *cur_z[4], *nxt_z[4];
  int nc = find_zones(cur_pos, cur_z);
  int nn = find_zones(next_pos, nxt_z);

  int total = 0;
  for (int i = 0; i < nc; i++)
    out_zones[total++] = cur_z[i];
  for (int i = 0; i < nn; i++)
  {
    /* deduplicate */
    int dup = 0;
    for (int j = 0; j < total; j++)
    {
      if (out_zones[j]->id == nxt_z[i]->id)
      {
        dup = 1;
        break;
      }
    }
    if (!dup)
      out_zones[total++] = nxt_z[i];
  }

  /* insertion sort by zone id (ascending) — at most 8 elements */
  for (int i = 1; i < total; i++)
  {
    Zone *tmp = out_zones[i];
    int j = i - 1;
    while (j >= 0 && out_zones[j]->id > tmp->id)
    {
      out_zones[j + 1] = out_zones[j];
      j--;
    }
    out_zones[j + 1] = tmp;
  }
  return total;
}

/**
 * @brief Lock all zones in ascending id order (deadlock-free).
 */
static void lock_zones(Zone *zs[], int n)
{
  for (int i = 0; i < n; i++)
    pthread_mutex_lock(&zs[i]->mutex);
}

/**
 * @brief Unlock all zones in descending id order.
 */
static void unlock_zones(Zone *zs[], int n)
{
  for (int i = n - 1; i >= 0; i--)
    pthread_mutex_unlock(&zs[i]->mutex);
}

/**
 * @brief Signal the condition variable on every zone in the set.
 */
static void signal_zones(Zone *zs[], int n)
{
  for (int i = 0; i < n; i++)
    pthread_cond_signal(&zs[i]->cond);
}

int within_zone(Zone *zone, Node *node)
{
  return pos_in_zone(zone, node->pos);
}

/* -- runner ----------------------------------------------------------- */

void *runner(void *arg)
{
  Agent *agent = (Agent *)arg;
  Node *current_node = agent->start_node;
  PathNode *next_moves = a_star(current_node, selected_heuristic);

  /* record starting position */
  append_path(agent, current_node->pos);

  while (!nodes_equal(current_node, goal_node))
  {
    if (next_moves == NULL)
    {
      struct timespec ts;
      clock_gettime(CLOCK_REALTIME, &ts);
      unsigned int timeout_us = speed_delay ? speed_delay : REALTIME_TIMEOUT_US;
      ts.tv_sec += timeout_us / 1000000;
      ts.tv_nsec += (timeout_us % 1000000) * 1000;
      if (ts.tv_nsec >= 1000000000)
      {
        ts.tv_sec += 1;
        ts.tv_nsec -= 1000000000;
      }
      pthread_mutex_lock(&map_changed_mutex);
      pthread_cond_timedwait(&map_changed_cond, &map_changed_mutex, &ts);
      pthread_mutex_unlock(&map_changed_mutex);
      next_moves = a_star(current_node, selected_heuristic);
      continue;
    }

    int moved = 0;
    while (!moved)
    {
      PathNode *next_move = next_moves;

      /* collect and lock all zones covering both current and next positions */
      Zone *locked[8];
      int nlocked = collect_zones(current_node->pos, next_move->node->pos, locked);
      lock_zones(locked, nlocked);

      /* check if the immediate next cell is occupied by another agent */
      if (shared_map[next_move->node->pos.y][next_move->node->pos.x].type == AGENT)
      {
        /* unlock all zones before waiting to avoid holding extra locks
           (pthread_cond_wait only releases the one mutex it's given) */
        Zone *wait_zone = NULL;
        for (int z = 0; z < nlocked; z++)
        {
          if (within_zone(locked[z], next_move->node))
          {
            wait_zone = locked[z];
            break;
          }
        }
        if (wait_zone == NULL)
          wait_zone = locked[0]; /* fallback, should not happen */

        for (int z = nlocked - 1; z >= 0; z--)
        {
          if (locked[z] != wait_zone)
            pthread_mutex_unlock(&locked[z]->mutex);
        }
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        unsigned int timeout_us = speed_delay ? speed_delay : REALTIME_TIMEOUT_US;
        ts.tv_sec += timeout_us / 1000000;
        ts.tv_nsec += (timeout_us % 1000000) * 1000;
        if (ts.tv_nsec >= 1000000000)
        {
          ts.tv_sec += 1;
          ts.tv_nsec -= 1000000000;
        }
        pthread_cond_timedwait(&wait_zone->cond, &wait_zone->mutex, &ts);
        pthread_mutex_unlock(&wait_zone->mutex);
        free_path(next_moves);
        next_moves = a_star(current_node, selected_heuristic);
        continue;
      }

      /* check if the remaining path within our locked zones is still valid */
      PathNode *check = next_move->next;
      int valid_path = 1;
      while (check != NULL)
      {
        /* only validate nodes that fall within at least one of our locked zones */
        int in_locked = 0;
        for (int z = 0; z < nlocked; z++)
        {
          if (within_zone(locked[z], check->node))
          {
            in_locked = 1;
            break;
          }
        }
        if (!in_locked)
          break; /* outside our locked region — stop checking */

        NodeType type = shared_map[check->node->pos.y][check->node->pos.x].type;
        if (type == OBSTACLE || type == AGENT)
        {
          valid_path = 0;
          break;
        }
        check = check->next;
      }

      if (!valid_path)
      {
        unlock_zones(locked, nlocked);
        free_path(next_moves);
        next_moves = a_star(current_node, selected_heuristic);
      }
      else if (next_move->node->type == OBSTACLE)
      {
        shared_map[next_move->node->pos.y][next_move->node->pos.x].type = OBSTACLE;
        unlock_zones(locked, nlocked);

        pthread_mutex_lock(&map_changed_mutex);
        pthread_cond_broadcast(&map_changed_cond);
        pthread_mutex_unlock(&map_changed_mutex);

        free_path(next_moves);
        next_moves = a_star(current_node, selected_heuristic);
      }
      else
      {
        /* valid move — update shared map atomically under all zone locks */
        Node *next_node = next_move->node;
        shared_map[current_node->pos.y][current_node->pos.x].type = EMPTY;
        if (nodes_equal(next_node, goal_node))
        {
          shared_map[next_node->pos.y][next_node->pos.x].type = GOAL;
        }
        else
        {
          shared_map[next_node->pos.y][next_node->pos.x].type = AGENT;
        }
        PathNode *old = next_moves;
        next_moves = next_moves->next;
        free(old);
        current_node = next_node;
        moved = 1;

        /* record this move in the agent's path history */
        append_path(agent, current_node->pos);

        signal_zones(locked, nlocked);
        unlock_zones(locked, nlocked);

        pthread_mutex_lock(&map_changed_mutex);
        pthread_cond_broadcast(&map_changed_cond);
        pthread_mutex_unlock(&map_changed_mutex);

        pthread_mutex_lock(&cli_mutex);
        atomic_fetch_add(&cli_update_seq, 1);
        pthread_cond_signal(&cli_cond);
        pthread_mutex_unlock(&cli_mutex);

        usleep(speed_delay);
      }
    }
  }

  free_path(next_moves);
  return NULL;
}
