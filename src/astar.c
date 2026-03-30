#include "../include/astar.h"
#include "../include/heap.h"
#include <stdlib.h>

extern int cols;
extern int rows;
extern Node *goal_node;
extern Node **shared_map;

float manhattan_distance(Coords c1, Coords c2)
{
  return abs(c1.x - c2.x) + abs(c1.y - c2.y);
}

PathNode *a_star(Node *start_node)
{
  /*
   * closed_set is a 2D array of chars (0 or 1) which represent if a given node has been visited by the algorithm.
   * Each node is addressable in O(1) by taking the node coordinates and doing closed_set[y * cols + x].
   * We use chars for 0 and 1 rather than ints since chars are 4x more space efficient.
   */
  char *closed_set = (char *)calloc(cols * rows, sizeof(char));
  Node **came_from = (Node **)calloc(cols * rows, sizeof(Node *));
  if (!closed_set || !came_from)
  {
    if (closed_set)
      free(closed_set);
    if (came_from)
      free(came_from);
    return NULL;
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
    AStarNode curr = heap_pop(&open_set); // cheapest f cost node to explore
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
      if (nodes_equal(prev_path_node->node, start_node))
      {
        PathNode *old = prev_path_node;
        prev_path_node = prev_path_node->next;
        free(old);
      }
      return prev_path_node; // head of the linked list
    }
    // current node is not the goal node, so we expand curr
    closed_set[curr.node->pos.y * cols + curr.node->pos.x] = 1;
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
      if (shared_map[n->pos.y][n->pos.x].type == AGENT && !nodes_equal(n, goal_node))
      {
        int g = curr.g + 10; // Add a higher cost for pathing through an agent
        int f = g + manhattan_distance(n->pos, goal_node->pos);
        heap_push(&open_set, (AStarNode){n, g, f});
        came_from[n->pos.y * cols + n->pos.x] = curr.node;
        continue;
      }
      int g = curr.g + 1;
      int f = g + manhattan_distance(n->pos, goal_node->pos);
      heap_push(&open_set, (AStarNode){n, g, f});
      came_from[n->pos.y * cols + n->pos.x] = curr.node;
    }
  }
  heap_free(&open_set);
  free(closed_set);
  free(came_from);
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