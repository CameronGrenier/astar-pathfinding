#include "../include/cli.h"
#include <pthread.h>
#include <stdatomic.h>
#include <ncurses.h>
#include <sys/time.h>
#include <errno.h>

extern Node **shared_map;
extern int rows;
extern int cols;

/* -- colour pair IDs --------------------------------------------------- */
#define PAIR_EMPTY 1
#define PAIR_OBSTACLE 2
#define PAIR_AGENT 3
#define PAIR_GOAL 4
#define PAIR_HEADER 5
#define PAIR_LEGEND 6

static int cam_x = 0;
static int cam_y = 0;

static void render_cli(void)
{
	erase();

	/* header */
	attron(COLOR_PAIR(PAIR_HEADER) | A_BOLD);
	mvprintw(0, 0, " ASTAR PATHFINDER ");
	attroff(COLOR_PAIR(PAIR_HEADER) | A_BOLD);

	/* legend */
	mvprintw(1, 0, " Legend: ");
	attron(COLOR_PAIR(PAIR_AGENT) | A_BOLD);
	printw(" A ");
	attroff(COLOR_PAIR(PAIR_AGENT) | A_BOLD);
	printw("Agent ");
	attron(COLOR_PAIR(PAIR_GOAL) | A_BOLD);
	printw(" G ");
	attroff(COLOR_PAIR(PAIR_GOAL) | A_BOLD);
	printw("Goal ");
	attron(COLOR_PAIR(PAIR_OBSTACLE));
	printw("   ");
	attroff(COLOR_PAIR(PAIR_OBSTACLE));
	printw("Wall ");
	attron(COLOR_PAIR(PAIR_EMPTY));
	printw(" . ");
	attroff(COLOR_PAIR(PAIR_EMPTY));
	printw("Empty");

	int term_rows, term_cols;
	getmaxyx(stdscr, term_rows, term_cols);

	int available_rows = term_rows - 3;
	int available_cols = term_cols;

	int cell_width = 3;
	if (cols * 3 > available_cols || rows > available_rows)
	{
		if (cols * 2 <= available_cols && rows <= available_rows)
		{
			cell_width = 2;
		}
		else
		{
			cell_width = 1;
		}
	}

	int visible_cols = available_cols / cell_width;
	int visible_rows = available_rows;

	if (cam_x < 0)
		cam_x = 0;
	if (cam_y < 0)
		cam_y = 0;
	if (cam_x > cols - visible_cols)
		cam_x = cols - visible_cols;
	if (cam_y > rows - visible_rows)
		cam_y = rows - visible_rows;
	if (cam_x < 0)
		cam_x = 0;
	if (cam_y < 0)
		cam_y = 0;

	for (int row = 0; row < visible_rows; row++)
	{
		int map_row = rows - 1 - (cam_y + row);
		if (map_row < 0 || map_row >= rows)
			continue;

		for (int col = 0; col < visible_cols; col++)
		{
			int map_col = cam_x + col;
			if (map_col < 0 || map_col >= cols)
				continue;

			NodeType type = shared_map[map_row][map_col].type;
			int screen_row = row + 3;
			int screen_col = col * cell_width;

			const char *str_empty = cell_width == 3 ? " . " : (cell_width == 2 ? " ." : ".");
			const char *str_obs = cell_width == 3 ? "   " : (cell_width == 2 ? "  " : " ");
			const char *str_agent = cell_width == 3 ? " A " : (cell_width == 2 ? " A" : "A");
			const char *str_goal = cell_width == 3 ? " G " : (cell_width == 2 ? " G" : "G");

			switch (type)
			{
			case EMPTY:
				attron(COLOR_PAIR(PAIR_EMPTY));
				mvprintw(screen_row, screen_col, "%s", str_empty);
				attroff(COLOR_PAIR(PAIR_EMPTY));
				break;
			case OBSTACLE:
				attron(COLOR_PAIR(PAIR_OBSTACLE));
				mvprintw(screen_row, screen_col, "%s", str_obs);
				attroff(COLOR_PAIR(PAIR_OBSTACLE));
				break;
			case AGENT:
				attron(COLOR_PAIR(PAIR_AGENT) | A_BOLD);
				mvprintw(screen_row, screen_col, "%s", str_agent);
				attroff(COLOR_PAIR(PAIR_AGENT) | A_BOLD);
				break;
			case GOAL:
				attron(COLOR_PAIR(PAIR_GOAL) | A_BOLD);
				mvprintw(screen_row, screen_col, "%s", str_goal);
				attroff(COLOR_PAIR(PAIR_GOAL) | A_BOLD);
				break;
			}
		}
	}

	if (cols > visible_cols || rows > visible_rows)
	{
		mvprintw(0, 20, " [Use Arrow Keys to Scroll] ");
	}

	refresh();
}

void *cli_runner(void *arg)
{
	(void)arg;
	initscr();
	cbreak();
	noecho();
	curs_set(0);
	keypad(stdscr, TRUE);

	/* init colours with background fills for a polished look */
	start_color();
	use_default_colors();
	init_pair(PAIR_EMPTY, COLOR_WHITE, -1);							/* dim dot on default bg */
	init_pair(PAIR_OBSTACLE, COLOR_WHITE, COLOR_WHITE); /* solid white block      */
	init_pair(PAIR_AGENT, COLOR_WHITE, COLOR_RED);			/* white letter on red bg  */
	init_pair(PAIR_GOAL, COLOR_BLACK, COLOR_GREEN);			/* black letter on green bg */
	init_pair(PAIR_HEADER, COLOR_CYAN, -1);
	init_pair(PAIR_LEGEND, COLOR_WHITE, -1);

	clear();
	refresh();

	nodelay(stdscr, TRUE);

	unsigned int rendered_seq = atomic_load(&cli_update_seq);
	render_cli();

	for (;;)
	{
		int ch = getch();
		int needs_render = 0;
		if (ch != ERR)
		{
			if (ch == KEY_UP)
			{
				cam_y--;
				needs_render = 1;
			}
			else if (ch == KEY_DOWN)
			{
				cam_y++;
				needs_render = 1;
			}
			else if (ch == KEY_LEFT)
			{
				cam_x--;
				needs_render = 1;
			}
			else if (ch == KEY_RIGHT)
			{
				cam_x++;
				needs_render = 1;
			}
		}

		pthread_mutex_lock(&cli_mutex);
		while (!atomic_load(&cli_done) && atomic_load(&cli_update_seq) == rendered_seq && !needs_render)
		{
			struct timespec ts;
			clock_gettime(CLOCK_REALTIME, &ts);
			ts.tv_nsec += 50000000; // 50ms
			if (ts.tv_nsec >= 1000000000)
			{
				ts.tv_sec += 1;
				ts.tv_nsec -= 1000000000;
			}
			int rc = pthread_cond_timedwait(&cli_cond, &cli_mutex, &ts);
			if (rc == ETIMEDOUT)
				break;
		}

		unsigned int pending_seq = atomic_load(&cli_update_seq);
		int done = atomic_load(&cli_done);
		pthread_mutex_unlock(&cli_mutex);

		if (pending_seq != rendered_seq || needs_render)
		{
			render_cli();
			rendered_seq = pending_seq;
			continue;
		}

		if (done)
		{
			break;
		}
	}

	/* do NOT call endwin() here — main handles the ncurses session
		 so the results screen can be shown after simulation ends. */
	return NULL;
}
