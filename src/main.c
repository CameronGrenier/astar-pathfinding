#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <stdatomic.h>
#include <time.h>
#include <ncurses.h>
#include <locale.h>
#include "../include/map.h"
#include "../include/runner.h"
#include "../include/cli.h"
#include "../include/menu.h"
#include "../include/astar.h"

/* -- globals defined here, extern'd everywhere else ------------------- */
Node **actual_map;
Node **shared_map;
Node *goal_node;
Agent *agents;
Zone *zones;
int rows, cols;
int num_agents;
int num_zones, num_zones_x, num_zones_y;
int zone_size_x, zone_size_y;
unsigned int speed_delay = 100000;						 /* default: medium (100ms per step) */
enum Heuristic selected_heuristic = MANHATTAN; /* default heuristic */

pthread_mutex_t cli_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t cli_cond = PTHREAD_COND_INITIALIZER;
volatile atomic_int cli_done = 0;
atomic_uint cli_update_seq = 0;

pthread_mutex_t map_changed_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t map_changed_cond = PTHREAD_COND_INITIALIZER;

/* -- colour pairs for the results screen ------------------------------ */
#define RES_HEADER 10
#define RES_LABEL 11
#define RES_VALUE 12
#define RES_PROMPT 13
#define RES_AGENT 14

/**
 * @brief Display a post-simulation results screen using ncurses.
 *
 * Shows elapsed wall-clock time, each agent's step count, and their
 * full coordinate path.  Returns 1 if the user wants to run again, 0 to exit.
 */
static int show_results(double elapsed_sec)
{
	/* ncurses is still active (cli_runner did NOT call endwin) */
	clear();
	curs_set(0);
	noecho();
	keypad(stdscr, TRUE);

	/* init result-screen colour pairs */
	init_pair(RES_HEADER, COLOR_CYAN, -1);
	init_pair(RES_LABEL, COLOR_YELLOW, -1);
	init_pair(RES_VALUE, COLOR_WHITE, -1);
	init_pair(RES_PROMPT, COLOR_GREEN, -1);
	init_pair(RES_AGENT, COLOR_RED, -1);

	/* calculate total content height so we can scroll if needed */
	int content_lines = 6; /* header + timer + blanks */
	for (int i = 0; i < num_agents; i++)
	{
		content_lines += 2; /* agent header + steps line */
		/* path line(s): estimate ~8 coords per 80-col line */
		int coords_per_line = (COLS - 10) / 8;
		if (coords_per_line < 1)
			coords_per_line = 1;
		int path_lines = (agents[i].path_len + coords_per_line - 1) / coords_per_line;
		content_lines += path_lines + 1; /* path + blank */
	}
	content_lines += 2; /* prompt */

	int scroll_offset = 0;
	int max_scroll = content_lines - LINES + 2;
	if (max_scroll < 0)
		max_scroll = 0;

	for (;;)
	{
		clear();
		int y = -scroll_offset;

		/* title bar */
		if (y >= 0 && y < LINES)
		{
			attron(COLOR_PAIR(RES_HEADER) | A_BOLD);
			mvprintw(y, 0, "  ===  SIMULATION RESULTS  ===");
			attroff(COLOR_PAIR(RES_HEADER) | A_BOLD);
		}
		y += 2;

		/* elapsed time */
		if (y >= 0 && y < LINES)
		{
			attron(COLOR_PAIR(RES_LABEL) | A_BOLD);
			mvprintw(y, 2, "Elapsed time: ");
			attroff(COLOR_PAIR(RES_LABEL) | A_BOLD);
			attron(COLOR_PAIR(RES_VALUE));
			printw("%.3f seconds", elapsed_sec);
			attroff(COLOR_PAIR(RES_VALUE));
		}
		y += 1;

		if (y >= 0 && y < LINES)
		{
			attron(COLOR_PAIR(RES_LABEL) | A_BOLD);
			mvprintw(y, 2, "Agents: ");
			attroff(COLOR_PAIR(RES_LABEL) | A_BOLD);
			attron(COLOR_PAIR(RES_VALUE));
			printw("%d", num_agents);
			attroff(COLOR_PAIR(RES_VALUE));
		}
		y += 2;

		/* per-agent results */
		for (int i = 0; i < num_agents; i++)
		{
			/* agent header */
			if (y >= 0 && y < LINES)
			{
				attron(COLOR_PAIR(RES_AGENT) | A_BOLD);
				mvprintw(y, 2, "Agent %d", agents[i].id);
				attroff(COLOR_PAIR(RES_AGENT) | A_BOLD);
			}
			y++;

			/* steps count */
			int steps = agents[i].path_len > 0 ? agents[i].path_len - 1 : 0;
			if (y >= 0 && y < LINES)
			{
				attron(COLOR_PAIR(RES_LABEL));
				mvprintw(y, 4, "Steps: ");
				attroff(COLOR_PAIR(RES_LABEL));
				attron(COLOR_PAIR(RES_VALUE));
				printw("%d", steps);
				attroff(COLOR_PAIR(RES_VALUE));
			}
			y++;

			/* path coordinates */
			if (y >= 0 && y < LINES)
			{
				attron(COLOR_PAIR(RES_LABEL));
				mvprintw(y, 4, "Path: ");
				attroff(COLOR_PAIR(RES_LABEL));
			}
			int col_pos = 10;
			for (int p = 0; p < agents[i].path_len; p++)
			{
				char buf[16];
				snprintf(buf, sizeof(buf), "(%d,%d)",
								 agents[i].path_history[p].x,
								 agents[i].path_history[p].y);
				int len = (int)strlen(buf);

				/* wrap to next line if needed */
				if (col_pos + len + 1 > COLS - 2)
				{
					y++;
					col_pos = 10;
				}

				if (y >= 0 && y < LINES)
				{
					attron(COLOR_PAIR(RES_VALUE));
					mvprintw(y, col_pos, "%s", buf);
					attroff(COLOR_PAIR(RES_VALUE));
				}
				col_pos += len + 1;
			}
			y += 2;
		}

		/* prompt */
		if (y >= 0 && y < LINES)
		{
			attron(COLOR_PAIR(RES_PROMPT) | A_BOLD);
			mvprintw(y, 2, "[R] Run again    [Q] Quit");
			attroff(COLOR_PAIR(RES_PROMPT) | A_BOLD);
		}
		y++;
		if (y >= 0 && y < LINES)
		{
			attron(COLOR_PAIR(RES_PROMPT) | A_DIM);
			mvprintw(y, 2, "Scroll: Up/Down arrows");
			attroff(COLOR_PAIR(RES_PROMPT) | A_DIM);
		}

		refresh();

		int ch = getch();
		if (ch == 'r' || ch == 'R')
			return 1;
		if (ch == 'q' || ch == 'Q')
			return 0;
		if (ch == KEY_DOWN && scroll_offset < max_scroll)
		{
			scroll_offset++;
		}
		if (ch == KEY_UP && scroll_offset > 0)
		{
			scroll_offset--;
		}
	}
}

/**
 * @brief Free all simulation resources between runs.
 */
static void cleanup_sim(pthread_t *threads)
{
	/* free agent path histories */
	for (int i = 0; i < num_agents; i++)
	{
		free(agents[i].path_history);
	}

	/* free maps */
	for (int i = 0; i < rows; i++)
	{
		free(actual_map[i]);
		free(shared_map[i]);
	}
	free(actual_map);
	free(shared_map);

	/* free agents */
	free(agents);

	/* destroy zone synchronization primitives and free */
	for (int i = 0; i < num_zones; i++)
	{
		pthread_mutex_destroy(&zones[i].mutex);
		pthread_cond_destroy(&zones[i].cond);
	}
	free(zones);

	/* free thread handle array */
	free(threads);
}

int main(int argc, char *argv[])
{
	setlocale(LC_ALL, "");
	int first_run = 1;
	char input_file[256] = {0};
	int use_cli_args = (argc >= 2);

	/* parse CLI speed and heuristic flags once (if provided) */
	if (use_cli_args)
	{
		strncpy(input_file, argv[1], sizeof(input_file) - 1);
		for (int i = 2; i < argc - 1; i++)
		{
			if (strcmp(argv[i], "-speed") == 0)
			{
				const char *s = argv[i + 1];
				if (strcmp(s, "slow") == 0)
					speed_delay = 500000;
				else if (strcmp(s, "medium") == 0)
					speed_delay = 100000;
				else if (strcmp(s, "fast") == 0)
					speed_delay = 20000;
				else if (strcmp(s, "realtime") == 0)
					speed_delay = 0;
				else
				{
					fprintf(stderr, "Unknown speed '%s'. Use: slow, medium, fast, realtime\n", s);
					return 1;
				}
				i++; // skip the speed value
			}
			else if (strcmp(argv[i], "-heuristic") == 0)
			{
				const char *h = argv[i + 1];
				if (strcmp(h, "manhattan") == 0)
					selected_heuristic = MANHATTAN;
				else if (strcmp(h, "euclidean") == 0)
					selected_heuristic = EUCLIDEAN;
				else
				{
					fprintf(stderr, "Unknown heuristic '%s'. Use: manhattan, euclidean\n", h);
					return 1;
				}
				i++; // skip the heuristic value
			}
		}
	}

	/* ---- main loop: menu -> simulation -> results -> prompt --------- */
	for (;;)
	{
		/* get input file, speed, and heuristic — from args on first run, menu otherwise */
		if (!use_cli_args || !first_run)
		{
			int heuristic_choice = 0; /* 0=MANHATTAN, 1=EUCLIDEAN */
			run_menu(input_file, &speed_delay, &heuristic_choice);
			selected_heuristic = (enum Heuristic)heuristic_choice;
		}
		first_run = 0;

		/* initialise simulation from input file */
		SimConfig config;
		init(&config, input_file);

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

		/* reset CLI synchronisation state */
		atomic_store(&cli_done, 0);
		atomic_store(&cli_update_seq, 0);

		/* spawn CLI rendering thread */
		pthread_t cli_thread;
		pthread_create(&cli_thread, NULL, cli_runner, NULL);

		/* start wall-clock timer */
		struct timespec t_start, t_end;
		clock_gettime(CLOCK_MONOTONIC, &t_start);

		/* spawn one thread per agent */
		pthread_t *threads = malloc((size_t)num_agents * sizeof(pthread_t));
		if (!threads)
		{
			perror("Failed to allocate threads");
			return 1;
		}
		for (int i = 0; i < num_agents; i++)
		{
			pthread_create(&threads[i], NULL, runner, &agents[i]);
		}

		/* wait for all agent threads to finish */
		for (int i = 0; i < num_agents; i++)
		{
			pthread_join(threads[i], NULL);
		}

		/* stop timer */
		clock_gettime(CLOCK_MONOTONIC, &t_end);
		double elapsed = (double)(t_end.tv_sec - t_start.tv_sec) + (double)(t_end.tv_nsec - t_start.tv_nsec) / 1e9;

		/* signal CLI thread to shut down and wait for it */
		pthread_mutex_lock(&cli_mutex);
		atomic_store(&cli_done, 1);
		pthread_cond_signal(&cli_cond);
		pthread_mutex_unlock(&cli_mutex);
		pthread_join(cli_thread, NULL);

		/* show results screen (ncurses session still alive) */
		int run_again = show_results(elapsed);

		/* tear down ncurses before next cycle */
		endwin();

		/* free all simulation memory */
		cleanup_sim(threads);

		if (!run_again)
			break;
	}

	return 0;
}
