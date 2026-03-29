#ifndef RUNNER_H
#define RUNNER_H
#include "./types.h"
#include "./astar.h"

/**
 * @brief Thread entry point for each agent.
 *
 * Each agent runs A* to find a path to the goal, then follows it step by step.
 * Before each move, the agent acquires all overlapping zone locks (in ascending
 * id order to prevent deadlock) and checks that the planned path within the
 * locked zones is still valid on the shared map. If an obstacle is discovered
 * or the path is blocked by another agent, the agent replans.
 * On a valid move, the agent updates the shared map atomically and signals
 * the CLI thread and any waiting agents in all locked zones.
 *
 * @param arg Pointer to the Agent struct for this thread
 * @return NULL on completion
 */
void *runner(void *arg);

/**
 * @brief Returns 1 if the given node falls within the zone's bounds.
 *
 * @param zone Zone to check against
 * @param node Node to check
 * @return 1 if within zone, 0 otherwise
 */
int within_zone(Zone *zone, Node *node);

#endif