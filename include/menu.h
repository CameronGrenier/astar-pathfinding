#ifndef MENU_H
#define MENU_H

/**
 * @brief Runs the interactive ncurses startup menu.
 *
 * Displays the ASCII art title, lists available input files from input/,
 * offers a custom path entry option, and prompts for speed selection.
 * Populates the provided buffers and speed_delay on return.
 *
 * @param filepath_out  Buffer to receive the chosen input file path (at least 256 bytes).
 * @param speed_out     Pointer to receive the chosen speed_delay in microseconds.
 */
void run_menu(char *filepath_out, unsigned int *speed_out);

#endif
