#include "../include/menu.h"
#include <ncurses.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <dirent.h>

#define MAX_FILES 64
#define PATH_BUF 256

/* -- ASCII art title --------------------------------------------------- */
static const char *TITLE[] = {
    "█████╗ ███████╗████████╗ █████╗ ██████╗                                      ",
    "██╔══██╗██╔════╝╚══██╔══╝██╔══██╗██╔══██╗                                     ",
    "███████║███████╗   ██║   ███████║██████╔╝                                     ",
    "██╔══██║╚════██║   ██║   ██╔══██║██╔══██╗                                     ",
    "██║  ██║███████║   ██║   ██║  ██║██║  ██║                                     ",
    "╚═╝  ╚═╝╚══════╝   ╚═╝   ╚═╝  ╚═╝╚═╝  ╚═╝                                     ",
    "                                                                              ",
    "██████╗  █████╗ ████████╗██╗  ██╗███████╗██╗███╗   ██╗██████╗ ███████╗██████╗ ",
    "██╔══██╗██╔══██╗╚══██╔══╝██║  ██║██╔════╝██║████╗  ██║██╔══██╗██╔════╝██╔══██╗",
    "██████╔╝███████║   ██║   ███████║█████╗  ██║██╔██╗ ██║██║  ██║█████╗  ██████╔╝",
    "██╔═══╝ ██╔══██║   ██║   ██╔══██║██╔══╝  ██║██║╚██╗██║██║  ██║██╔══╝  ██╔══██╗",
    "██║     ██║  ██║   ██║   ██║  ██║██║     ██║██║ ╚████║██████╔╝███████╗██║  ██║",
    "╚═╝     ╚═╝  ╚═╝   ╚═╝   ╚═╝  ╚═╝╚═╝     ╚═╝╚═╝  ╚═══╝╚═════╝ ╚══════╝╚═╝  ╚═╝",
};
static const int TITLE_LINES = (int)(sizeof(TITLE) / sizeof(TITLE[0]));

/* -- colour pair IDs --------------------------------------------------- */
#define C_TITLE 1
#define C_NORMAL 2
#define C_SEL 3
#define C_DIM 4
#define C_BORDER 5

/* -- helpers ----------------------------------------------------------- */
static int utf8_display_width(const char *s)
{
  int width = 0;
  while (*s)
  {
    unsigned char c = (unsigned char)*s;
    if (c < 0x80)
    {
      width++;
      s += 1;
    }
    else if (c < 0xE0)
    {
      width++;
      s += 2;
    }
    else if (c < 0xF0)
    {
      width++;
      s += 3;
    }
    else
    {
      width++;
      s += 4;
    }
  }
  return width;
}

static void draw_title(int start_row)
{
  attron(COLOR_PAIR(C_TITLE) | A_BOLD);
  for (int i = 0; i < TITLE_LINES; i++)
  {
    int len = utf8_display_width(TITLE[i]);
    int x = (COLS - len) / 2;
    if (x < 0)
      x = 0;
    mvprintw(start_row + i, x, "%s", TITLE[i]);
  }
  attroff(COLOR_PAIR(C_TITLE) | A_BOLD);
}

static void draw_box(int y, int x, int h, int w)
{
  attron(COLOR_PAIR(C_BORDER));
  mvhline(y, x + 1, '-', w - 2);
  mvhline(y + h - 1, x + 1, '-', w - 2);
  mvvline(y + 1, x, '|', h - 2);
  mvvline(y + 1, x + w - 1, '|', h - 2);
  mvaddch(y, x, '+');
  mvaddch(y, x + w - 1, '+');
  mvaddch(y + h - 1, x, '+');
  mvaddch(y + h - 1, x + w - 1, '+');
  attroff(COLOR_PAIR(C_BORDER));
}

/* -- file picker ------------------------------------------------------- */
static int collect_input_files(char files[][PATH_BUF], int max)
{
  DIR *dir = opendir("input");
  if (!dir)
    return 0;
  int count = 0;
  struct dirent *ent;
  while ((ent = readdir(dir)) != NULL && count < max - 1)
  {
    const char *name = ent->d_name;
    size_t len = strlen(name);
    if (len > 4 && strcmp(name + len - 4, ".txt") == 0)
    {
      snprintf(files[count], PATH_BUF, "input/%s", name);
      count++;
    }
  }
  closedir(dir);

  /* simple insertion sort by name */
  for (int i = 1; i < count; i++)
  {
    char tmp[PATH_BUF];
    strcpy(tmp, files[i]);
    int j = i - 1;
    while (j >= 0 && strcmp(files[j], tmp) > 0)
    {
      strcpy(files[j + 1], files[j]);
      j--;
    }
    strcpy(files[j + 1], tmp);
  }

  /* append "[ custom path... ]" sentinel */
  snprintf(files[count], PATH_BUF, "[ custom path... ]");
  return count + 1;
}

static void pick_file(char *out)
{
  char files[MAX_FILES][PATH_BUF];
  int nfiles = collect_input_files(files, MAX_FILES);
  int sel = 0;

  const char *label = "Select input file  (Up/Down to navigate, Enter to confirm)";
  int box_w = 52;
  int box_h = nfiles + 4;
  int box_y = TITLE_LINES + 3;
  if (box_y + box_h >= LINES)
    box_y = (LINES - box_h) / 2;
  int box_x = (COLS - box_w) / 2;

  while (1)
  {
    clear();
    draw_title(1);
    draw_box(box_y, box_x, box_h, box_w);

    attron(COLOR_PAIR(C_DIM));
    mvprintw(box_y, box_x + 2, " %s ", label);
    attroff(COLOR_PAIR(C_DIM));

    for (int i = 0; i < nfiles; i++)
    {
      int row = box_y + 2 + i;
      if (i == sel)
      {
        attron(COLOR_PAIR(C_SEL) | A_BOLD);
        mvprintw(row, box_x + 2, "  >  %-42s", files[i]);
        attroff(COLOR_PAIR(C_SEL) | A_BOLD);
      }
      else
      {
        attron(COLOR_PAIR(C_NORMAL));
        mvprintw(row, box_x + 2, "     %-42s", files[i]);
        attroff(COLOR_PAIR(C_NORMAL));
      }
    }
    refresh();

    int ch = getch();
    if (ch == KEY_UP)
    {
      sel = (sel - 1 + nfiles) % nfiles;
    }
    else if (ch == KEY_DOWN)
    {
      sel = (sel + 1) % nfiles;
    }
    else if (ch == '\n' || ch == KEY_ENTER)
    {
      if (strcmp(files[sel], "[ custom path... ]") == 0)
      {
        echo();
        curs_set(1);
        attron(COLOR_PAIR(C_DIM));
        mvprintw(box_y + box_h - 2, box_x + 2, "Custom path: ");
        attroff(COLOR_PAIR(C_DIM));
        clrtoeol();
        mvgetnstr(box_y + box_h - 2, box_x + 15, out, PATH_BUF - 1);
        noecho();
        curs_set(0);
      }
      else
      {
        strncpy(out, files[sel], PATH_BUF - 1);
        out[PATH_BUF - 1] = '\0';
      }
      return;
    }
  }
}

/* -- speed picker ------------------------------------------------------ */
static void pick_speed(unsigned int *out)
{
  static const char *labels[] = {
      "slow    (500ms / step)",
      "medium  (100ms / step)",
      "fast    (20ms / step)",
      "realtime (no delay)",
  };
  static const unsigned int delays[] = {500000, 100000, 20000, 0};
  const int N = 4;
  int sel = 1;

  const char *header = "Select simulation speed  (Up/Down to navigate, Enter to confirm)";
  int box_w = 56;
  int box_h = N + 4;
  int box_y = TITLE_LINES + 4;
  if (box_y + box_h >= LINES)
    box_y = (LINES - box_h) / 2;
  int box_x = (COLS - box_w) / 2;

  while (1)
  {
    clear();
    draw_title(1);
    draw_box(box_y, box_x, box_h, box_w);

    attron(COLOR_PAIR(C_DIM));
    mvprintw(box_y, box_x + 2, " %s ", header);
    attroff(COLOR_PAIR(C_DIM));

    for (int i = 0; i < N; i++)
    {
      int row = box_y + 2 + i;
      if (i == sel)
      {
        attron(COLOR_PAIR(C_SEL) | A_BOLD);
        mvprintw(row, box_x + 2, "  >  %-48s", labels[i]);
        attroff(COLOR_PAIR(C_SEL) | A_BOLD);
      }
      else
      {
        attron(COLOR_PAIR(C_NORMAL));
        mvprintw(row, box_x + 2, "     %-48s", labels[i]);
        attroff(COLOR_PAIR(C_NORMAL));
      }
    }
    refresh();

    int ch = getch();
    if (ch == KEY_UP)
    {
      sel = (sel - 1 + N) % N;
    }
    else if (ch == KEY_DOWN)
    {
      sel = (sel + 1) % N;
    }
    else if (ch == '\n' || ch == KEY_ENTER)
    {
      *out = delays[sel];
      return;
    }
  }
}

/* -- public entry point ------------------------------------------------ */
void run_menu(char *filepath_out, unsigned int *speed_out)
{
  initscr();
  cbreak();
  noecho();
  curs_set(0);
  keypad(stdscr, TRUE);

  start_color();
  use_default_colors();
  init_pair(C_TITLE, COLOR_CYAN, -1);
  init_pair(C_NORMAL, COLOR_WHITE, -1);
  init_pair(C_SEL, COLOR_BLACK, COLOR_CYAN);
  init_pair(C_DIM, COLOR_YELLOW, -1);
  init_pair(C_BORDER, COLOR_BLUE, -1);

  pick_file(filepath_out);
  pick_speed(speed_out);

  endwin();
}