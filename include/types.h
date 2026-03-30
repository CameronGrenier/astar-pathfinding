#ifndef TYPES_H
#define TYPES_H
#include <pthread.h>
#include <stdatomic.h>

/**
 * @brief Represents the type of a node on the map.
 *
 * EMPTY    - passable cell with no occupant
 * OBSTACLE - impassable cell, permanently blocked
 * AGENT    - cell currently occupied by a robot
 * GOAL     - the rendezvous point all agents navigate toward
 */
typedef enum
{
	EMPTY,
	OBSTACLE,
	AGENT,
	GOAL
} NodeType;

/**
 * @brief 2D grid coordinates.
 */
typedef struct Coords
{
	int x;
	int y;
} Coords;

/**
 * @brief A single node in the grid graph.
 *
 * Nodes are connected to their cardinal neighbours via pointers.
 * NULL neighbours indicate grid boundaries.
 * Used in both actual_map (ground truth) and shared_map (collective knowledge).
 */
typedef struct Node
{
	_Atomic NodeType type;
	struct Node *up, *down, *left, *right;
	Coords pos;
} Node;

/**
 * @brief Represents a robot agent in the simulation.
 *
 * Each agent runs on its own thread via Runner().
 * start_node points into actual_map at the agent's initial position.
 */
typedef struct
{
	int id;
	Node *start_node;
	Coords *path_history; /**< Dynamic array of positions visited */
	int path_len;					/**< Number of entries in path_history */
	int path_cap;					/**< Allocated capacity of path_history */
} Agent;

/**
 * @brief A map region with its own synchronization primitives.
 *
 * The map is divided into overlapping zones (25% overlap) to reduce lock contention.
 * mutex protects concurrent reads/writes within the zone.
 * cond is signalled when an agent moves, waking blocked agents.
 * Locks must always be acquired in ascending zone id order to prevent deadlock.
 */
typedef struct
{
	int id;
	pthread_mutex_t mutex;
	pthread_cond_t cond;
	Coords top_left;
	Coords bottom_right;
} Zone;

typedef struct
{
	Node **actual_map;
	Node **shared_map;
	Agent *agents;
	int num_agents;
	Node *goal_node;
	Zone *zones;
	int zone_size_x;
	int zone_size_y;
	int num_zones;
	int num_zones_x;
	int num_zones_y;
	int rows;
	int cols;
} SimConfig;

int nodes_equal(Node *a, Node *b);

int coords_equal(Coords a, Coords b);

#endif