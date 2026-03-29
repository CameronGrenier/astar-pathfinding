#include "../include/cli.h"
#include <pthread.h>
#include <stdatomic.h>
#include <ncurses.h>

extern Node **shared_map;
extern int rows;
extern int cols;

/* -- colour pair IDs --------------------------------------------------- */
#define PAIR_EMPTY    1
#define PAIR_OBSTACLE 2
#define PAIR_AGENT    3
#define PAIR_GOAL     4
#define PAIR_HEADER   5
#define PAIR_LEGEND   6

static void render_cli(void) {
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

    for (int row = rows - 1; row >= 0; row--) {
        for (int col = 0; col < cols; col++) {
            NodeType type = shared_map[row][col].type;
            int screen_row = rows - 1 - row + 3;

            switch (type) {
                case EMPTY:
                    attron(COLOR_PAIR(PAIR_EMPTY));
                    mvprintw(screen_row, col * 3, " . ");
                    attroff(COLOR_PAIR(PAIR_EMPTY));
                    break;
                case OBSTACLE:
                    attron(COLOR_PAIR(PAIR_OBSTACLE));
                    mvprintw(screen_row, col * 3, "   ");
                    attroff(COLOR_PAIR(PAIR_OBSTACLE));
                    break;
                case AGENT:
                    attron(COLOR_PAIR(PAIR_AGENT) | A_BOLD);
                    mvprintw(screen_row, col * 3, " A ");
                    attroff(COLOR_PAIR(PAIR_AGENT) | A_BOLD);
                    break;
                case GOAL:
                    attron(COLOR_PAIR(PAIR_GOAL) | A_BOLD);
                    mvprintw(screen_row, col * 3, " G ");
                    attroff(COLOR_PAIR(PAIR_GOAL) | A_BOLD);
                    break;
            }
        }
    }

    refresh();
}

void *cli_runner(void *arg) {
    (void)arg;
    initscr();
    cbreak();
    noecho();
    curs_set(0);
    keypad(stdscr, TRUE);

    /* init colours with background fills for a polished look */
    start_color();
    use_default_colors();
    init_pair(PAIR_EMPTY,    COLOR_WHITE,  -1);            /* dim dot on default bg */
    init_pair(PAIR_OBSTACLE, COLOR_WHITE,  COLOR_WHITE);   /* solid white block      */
    init_pair(PAIR_AGENT,    COLOR_WHITE,  COLOR_RED);     /* white letter on red bg  */
    init_pair(PAIR_GOAL,     COLOR_BLACK,  COLOR_GREEN);   /* black letter on green bg */
    init_pair(PAIR_HEADER,   COLOR_CYAN,   -1);
    init_pair(PAIR_LEGEND,   COLOR_WHITE,  -1);

    clear();
    refresh();

    unsigned int rendered_seq = atomic_load(&cli_update_seq);
    render_cli();

    for (;;) {
        pthread_mutex_lock(&cli_mutex);
        while (!atomic_load(&cli_done) && atomic_load(&cli_update_seq) == rendered_seq) {
            pthread_cond_wait(&cli_cond, &cli_mutex);
        }

        unsigned int pending_seq = atomic_load(&cli_update_seq);
        int done = atomic_load(&cli_done);
        pthread_mutex_unlock(&cli_mutex);

        if (pending_seq != rendered_seq) {
            render_cli();
            rendered_seq = pending_seq;
            continue;
        }

        if (done) {
            break;
        }
    }

    /* do NOT call endwin() here — main handles the ncurses session
       so the results screen can be shown after simulation ends. */
    return NULL;
}
