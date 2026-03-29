#ifndef CLI_H
#define CLI_H
#include <pthread.h>
#include <stdatomic.h>
#include "./types.h"

extern pthread_mutex_t cli_mutex;
extern pthread_cond_t cli_cond;
extern volatile atomic_int cli_done;
extern atomic_uint cli_update_seq;

/**
 * @brief CLI thread entry point. Redraws the shared_map grid after each move signal.
 *
 * Renders the initial shared_map state, then sleeps on cli_cond until signalled
 * by an agent move or shutdown.
 * Renders the shared_map grid using ncurses on each wake.
 * Exits cleanly when cli_done is set and there are no pending redraws.
 *
 * @return NULL on completion
 */
void *cli_runner(void *arg);

#endif
