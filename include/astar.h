#ifndef ASTAR_H
#define ASTAR_H
#include "types.h"

/**
 * @brief Internal A* node used during pathfinding.
 *
 * Wraps a grid node with its associated path costs for use
 * in the open set min-heap during A* search.
 */
typedef struct
{
  Node *node; /**< Pointer to the corresponding Node in actual_map */
  int g;      /**< Cost from start_node to this node */
  int f;      /**< Estimated total cost (g + heuristic to goal) */
} AStarNode;

/**
 *  @brief Simple linked list object used to build solution path.
 */
typedef struct PathNode
{
  Node *node;            /**< Pointer to node in actual map */
  struct PathNode *next; /**< Pointer to the next node, forming the linked list */
} PathNode;

/**
 * @brief Calculates the Manhattan Distance between two 2D coordinates
 *
 * @param c1 One coordinate object
 * @param c2 Second coordinate object
 * @return Manhattan Distance between point c1 and c2, as a float.
 */
float manhattan_distance(Coords c1, Coords c2);

/**
 * @brief Finds the optimal path from start_node to goal_node using A*.
 *
 * @param start_node Pointer to the node to start pathfinding from
 * @return The solution path as a pointer to a linked list of PathNodes (wrapper of Node struct).
 */
PathNode *a_star(Node *start_node);

/**
 * @brief Frees all PathNodes in a linked list path.
 *
 * @param path Pointer to the head of the path linked list
 */
void free_path(PathNode *path);

#endif