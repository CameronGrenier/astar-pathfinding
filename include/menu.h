#ifndef MENU_H
#define MENU_H

/**
 * @brief Runs the interactive ncurses startup menu.
 *
 * Displays the ASCII art title, lists available input files from input/,
 * offers a custom path entry option, and prompts for speed and heuristic selection.
 * Populates the provided buffers and parameters on return.
 *
 * @param filepath_out  Buffer to receive the chosen input file path (at least 256 bytes).
 * @param speed_out     Pointer to receive the chosen speed_delay in microseconds.
 * @param heuristic_out Pointer to receive the chosen heuristic (MANHATTAN or EUCLIDEAN).
 */
void run_menu(char *filepath_out, unsigned int *speed_out, int *heuristic_out);

#endif
