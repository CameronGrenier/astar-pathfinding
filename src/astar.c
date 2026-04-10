#include "../include/astar.h"
#include "../include/heap.h"
#include <stdlib.h>
#include <limits.h>
#include <math.h>

extern int cols;
extern int rows;
extern Node *goal_node;
extern Node **shared_map;

float manhattan_distance(Coords c1, Coords c2)
{
  return abs(c1.x - c2.x) + abs(c1.y - c2.y);
}

float euclidean_distance(Coords c1, Coords c2)
{
  return sqrt(pow(c1.x - c2.x, 2) + pow(c1.y - c2.y, 2));
}

PathNode *a_star(Node *start_node, enum Heuristic heuristic)
{
  /*
   * closed_set is a 1D array of chars (0 or 1) which represent if a given node has been visited by the algorithm.
   * Each node is addressable in O(1) by taking the node coordinates and doing closed_set[y * cols + x].
   * We use chars for 0 and 1 rather than ints since chars are 4x more space efficient.
   */
  char *closed_set = (char *)calloc(cols * rows, sizeof(char));

  /*
   * came_from is a hashmap which maps a given node to its predecessor in the path. This allows us to reconstruct the path
   * once we find the goal node. Each node is addressable in O(1) by taking the node coordinates and doing came_from[y * cols + x].
   */
  Node **came_from = (Node **)calloc(cols * rows, sizeof(Node *));

  /*
   * g_cost_array is a 1D array of ints which represent the g cost of a given node. This allows us to check if we have found a cheaper path to a node
   * when we encounter it again in the frontier. Each node is addressable in O(1) by taking the node coordinates and doing g_cost_array[y * cols + x].
   */
  int *g_cost_array = (int *)calloc(cols * rows, sizeof(int));

  if (!closed_set || !came_from || !g_cost_array)
  {
    if (closed_set)
      free(closed_set);
    if (came_from)
      free(came_from);
    if (g_cost_array)
      free(g_cost_array);
    return NULL;
  }
  g_cost_array[start_node->pos.y * cols + start_node->pos.x] = 0; // g cost of start node is 0

  // initialize g_cost_array to all INT_MAX since we haven't found any paths to any nodes yet
  for (int i = 0; i < cols * rows; i++)
  {
    g_cost_array[i] = INT_MAX;
  }

  /*
   * open_set is a minimum heap sorted by a nodes f cost which tracks all nodes in the algorithms frontier.
   * A Min Heap is used since we always want the minimum f cost element, and a minimum heap allows
   * O(1) minimum element access.
   */
  MinHeap open_set = heap_create(32);

  // start node as an AStarNode
  AStarNode start = {start_node, 0, manhattan_distance(start_node->pos, goal_node->pos)};
  heap_push(&open_set, start);

  // while the there is some frontier to process
  while (open_set.size != 0)
  {
    AStarNode curr = heap_pop(&open_set);                       // get the node in the frontier with the lowest f cost
    if (closed_set[curr.node->pos.y * cols + curr.node->pos.x]) // if we have already visited this node, skip it
      continue;
    // check if current node is goal node build solution path
    if (nodes_equal(curr.node, goal_node))
    {
      PathNode *prev_path_node = (PathNode *)malloc(sizeof(PathNode)); // keep track of the path head, we will be
      if (!prev_path_node)
        break;
      *prev_path_node = (PathNode){curr.node, NULL};
      Coords current_pos = curr.node->pos;
      while (!coords_equal(current_pos, start_node->pos))
      {
        // create a PathNode for the predecessor of the current node using the came_from hashmap
        PathNode *curr_path_node = (PathNode *)malloc(sizeof(PathNode));
        if (!curr_path_node)
        {
          free_path(prev_path_node);
          prev_path_node = NULL;
          break;
        }
        *curr_path_node = (PathNode){came_from[current_pos.y * cols + current_pos.x], prev_path_node};
        prev_path_node = curr_path_node;
        current_pos = curr_path_node->node->pos;
      }
      if (!prev_path_node)
        break;
      heap_free(&open_set);
      free(closed_set);
      free(came_from);
      free(g_cost_array);
      if (nodes_equal(prev_path_node->node, start_node))
      {
        PathNode *old = prev_path_node;
        prev_path_node = prev_path_node->next;
        free(old);
      }
      return prev_path_node; // head of the linked list
    }
    // current node is not the goal node, so we expand curr
    closed_set[curr.node->pos.y * cols + curr.node->pos.x] = 1; // mark current node as visited
    Node *neighbours[] = {curr.node->up, curr.node->down, curr.node->left, curr.node->right};
    for (int i = 0; i < 4; i++)
    {
      Node *n = neighbours[i];
      if (n == NULL)
      {
        continue;
      }
      if (closed_set[n->pos.y * cols + n->pos.x] == 1)
      {
        continue;
      }
      if (shared_map[n->pos.y][n->pos.x].type == OBSTACLE)
      {
        continue;
      }
      int tentative_g = curr.g + 1; // Normal cost for pathing through an empty cell
      if (shared_map[n->pos.y][n->pos.x].type == AGENT && !nodes_equal(n, goal_node))
      {
        tentative_g = curr.g + 2; // Add a higher cost for pathing through an agent
      }
      if (tentative_g >= g_cost_array[n->pos.y * cols + n->pos.x])
      {
        continue; // not a better path
      }
      g_cost_array[n->pos.y * cols + n->pos.x] = tentative_g;
      int f;
      switch (heuristic)
      {
      case MANHATTAN:
        f = tentative_g + manhattan_distance(n->pos, goal_node->pos);
        break;
      case EUCLIDEAN:
        f = tentative_g + euclidean_distance(n->pos, goal_node->pos);
        break;
      default:
        f = tentative_g + manhattan_distance(n->pos, goal_node->pos);
      }
      heap_push(&open_set, (AStarNode){n, tentative_g, f});
      came_from[n->pos.y * cols + n->pos.x] = curr.node;
    }
  }
  heap_free(&open_set);
  free(closed_set);
  free(came_from);
  free(g_cost_array);
  return NULL; // no solution was found
}

void free_path(PathNode *path)
{
  PathNode *current = path;
  PathNode *next;
  while (current != NULL)
  {
    next = current->next;
    free(current);
    current = next;
  }
}