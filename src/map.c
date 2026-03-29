#include "../include/map.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

extern int zone_size;

void init(SimConfig *config, const char *input_file)
{
	// open file and check for errors
	FILE *file = fopen(input_file, "r");
	if (file == NULL)
	{
		perror("Error opening file");
		return;
	}

	// read grid dimensions
	fscanf(file, "%d", &config->rows);
	fscanf(file, "%d", &config->cols);

	// read number of agents and allocate agent array
	fscanf(file, "%d", &config->num_agents);
	config->agents = (Agent *)malloc(config->num_agents * sizeof(Agent));

	// read agent starting positions — store temporarily since actual_map isn't built yet
	Coords *start_positions = (Coords *)malloc(config->num_agents * sizeof(Coords));
	for (int i = 0; i < config->num_agents; i++)
	{
		int x, y;
		fscanf(file, "%d", &x);
		fscanf(file, "%d", &y);
		config->agents[i] = (Agent){i, NULL, NULL, 0, 0};
		start_positions[i] = (Coords){x, y};
	}

	// read goal coordinates
	int gx, gy;
	fscanf(file, "%d", &gx);
	fscanf(file, "%d", &gy);

	// allocate actual_map as a 2D array — rows x cols
	config->actual_map = (Node **)malloc(config->rows * sizeof(Node *));
	for (int i = 0; i < config->rows; i++)
	{
		config->actual_map[i] = (Node *)malloc(config->cols * sizeof(Node));
	}

	// build actual_map from input file grid
	// input rows are listed top to bottom (row rows-1 down to row 0)
	for (int row = config->rows - 1; row >= 0; row--)
	{
		char line[256];
		fscanf(file, "%s", line);
		for (int col = 0; col < config->cols; col++)
		{
			NodeType type = (line[col] == '1') ? OBSTACLE : EMPTY;
			config->actual_map[row][col] = (Node){
					type,
					NULL, NULL, NULL, NULL, // neighbours connected below
					(Coords){col, row}};
		}
	}

	// connect neighbours — NULL for out-of-bounds
	for (int row = 0; row < config->rows; row++)
	{
		for (int col = 0; col < config->cols; col++)
		{
			Node *n = &config->actual_map[row][col];
			n->up = (row < config->rows - 1) ? &config->actual_map[row + 1][col] : NULL;
			n->down = (row > 0) ? &config->actual_map[row - 1][col] : NULL;
			n->left = (col > 0) ? &config->actual_map[row][col - 1] : NULL;
			n->right = (col < config->cols - 1) ? &config->actual_map[row][col + 1] : NULL;
		}
	}

	// set goal node on actual_map
	config->actual_map[gy][gx].type = GOAL;
	config->goal_node = &config->actual_map[gy][gx];

	// set start_node for each agent — pointer into actual_map
	for (int i = 0; i < config->num_agents; i++)
	{
		int x = start_positions[i].x;
		int y = start_positions[i].y;
		config->agents[i].start_node = &config->actual_map[y][x];
	}
	free(start_positions); // no longer needed

	// allocate and init shared_map — all EMPTY except agents and goal
	config->shared_map = (Node **)malloc(config->rows * sizeof(Node *));
	for (int i = 0; i < config->rows; i++)
	{
		config->shared_map[i] = (Node *)malloc(config->cols * sizeof(Node));
		for (int j = 0; j < config->cols; j++)
		{
			config->shared_map[i][j] = (Node){
					EMPTY,
					NULL, NULL, NULL, NULL, // shared_map not traversed by pointer
					(Coords){j, i}};
		}
	}
	// place agents on shared_map at their starting positions
	for (int i = 0; i < config->num_agents; i++)
	{
		int x = config->agents[i].start_node->pos.x;
		int y = config->agents[i].start_node->pos.y;
		config->shared_map[y][x].type = AGENT;
	}
	// place goal on shared_map
	config->shared_map[gy][gx].type = GOAL;

	// calculate zone dimensions based on sqrt(rows * cols)
	config->zone_size = (int)round(sqrt(config->rows * config->cols));
	config->num_zones_x = (config->cols + zone_size - 1) / zone_size; // ceil division
	config->num_zones_y = (config->rows + zone_size - 1) / zone_size;
	config->num_zones = config->num_zones_x * config->num_zones_y;

	// 25% overlap in each direction
	int overlap = config->zone_size / 4;

	// allocate and init zones with overlapping bounds and synchronization primitives
	config->zones = (Zone *)malloc(config->num_zones * sizeof(Zone));
	for (int zy = 0; zy < config->num_zones_y; zy++)
	{
		for (int zx = 0; zx < config->num_zones_x; zx++)
		{
			int idx = zy * config->num_zones_x + zx;
			config->zones[idx].id = idx;

			// base boundaries
			int base_left = zx * zone_size;
			int base_top = zy * zone_size;
			int base_right = (zx + 1) * zone_size - 1;
			int base_bottom = (zy + 1) * zone_size - 1;

			// extend by overlap, clamped to map bounds
			int left = base_left - overlap;
			int top = base_top - overlap;
			int right = base_right + overlap;
			int bottom = base_bottom + overlap;
			if (left < 0)
				left = 0;
			if (top < 0)
				top = 0;
			if (right >= config->cols)
				right = config->cols - 1;
			if (bottom >= config->rows)
				bottom = config->rows - 1;

			config->zones[idx].top_left = (Coords){left, top};
			config->zones[idx].bottom_right = (Coords){right, bottom};
			pthread_mutex_init(&config->zones[idx].mutex, NULL);
			pthread_cond_init(&config->zones[idx].cond, NULL);
		}
	}

	fclose(file);
}