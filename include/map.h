#ifndef MAP_H
#define MAP_H
#include "./types.h"

/**
 * @brief Initializes the simulation environment from an input file.
 *
 * Parses the input file to build actual_map and shared_map, initializes
 * all agents with their starting positions, sets the goal node, and
 * partitions the map into zones with their synchronization primitives.
 *
 * actual_map reflects ground truth and is never modified after init.
 * shared_map starts with only agent positions and the goal.
 * Obstacles are discovered and added at runtime by agents.
 *
 * @param config  Pointer to SimConfig struct to populate
 * @param input_file Path to the input file
 */
void init(SimConfig *config, const char *input_file);

#endif