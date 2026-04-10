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
    "█████╗ ███████╗████████╗ █████╗ ██████╗                                       ",
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

#include <time.h>

static void generate_random_map(const char *filepath, int rows, int cols, int num_agents)
{
  FILE *f = fopen(filepath, "w");
  if (!f)
    return;

  fprintf(f, "%d %d\n", rows, cols);
  fprintf(f, "%d\n", num_agents);

  srand(time(NULL));

  int *used = calloc(rows * cols, sizeof(int));

  for (int i = 0; i < num_agents; i++)
  {
    int x, y;
    do
    {
      x = rand() % cols;
      y = rand() % rows;
    } while (used[y * cols + x]);
    used[y * cols + x] = 1;
    fprintf(f, "%d %d\n", x, y);
  }

  int gx, gy;
  do
  {
    gx = rand() % cols;
    gy = rand() % rows;
  } while (used[gy * cols + gx]);
  used[gy * cols + gx] = 2;
  fprintf(f, "%d %d\n", gx, gy);

  for (int r = rows - 1; r >= 0; r--)
  {
    for (int c = 0; c < cols; c++)
    {
      if (used[r * cols + c])
      {
        fprintf(f, "0");
      }
      else
      {
        int is_obs = (rand() % 100) < 15;
        fprintf(f, "%d", is_obs ? 1 : 0);
      }
    }
    fprintf(f, "\n");
  }

  free(used);
  fclose(f);
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

  /* append sentinels */
  snprintf(files[count++], PATH_BUF, "[ custom path... ]");
  snprintf(files[count++], PATH_BUF, "[ generate random map... ]");
  return count;
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
      else if (strcmp(files[sel], "[ generate random map... ]") == 0)
      {
        clear();
        draw_title(1);
        int r_box_w = 40;
        int r_box_h = 6;
        int r_box_y = TITLE_LINES + 3;
        int r_box_x = (COLS - r_box_w) / 2;
        draw_box(r_box_y, r_box_x, r_box_h, r_box_w);

        attron(COLOR_PAIR(C_DIM));
        mvprintw(r_box_y, r_box_x + 2, " Random Map Parameters ");

        echo();
        curs_set(1);

        char rows_str[16] = {0}, cols_str[16] = {0}, agents_str[16] = {0};

        mvprintw(r_box_y + 2, r_box_x + 2, "Rows: ");
        mvgetnstr(r_box_y + 2, r_box_x + 8, rows_str, 15);

        mvprintw(r_box_y + 3, r_box_x + 2, "Cols: ");
        mvgetnstr(r_box_y + 3, r_box_x + 8, cols_str, 15);

        mvprintw(r_box_y + 4, r_box_x + 2, "Agents: ");
        mvgetnstr(r_box_y + 4, r_box_x + 10, agents_str, 15);

        attroff(COLOR_PAIR(C_DIM));
        noecho();
        curs_set(0);

        int r = atoi(rows_str);
        int c = atoi(cols_str);
        int a = atoi(agents_str);

        if (r < 5)
          r = 5;
        if (r > 1000)
          r = 1000;

        if (c < 5)
          c = 5;
        if (c > 1000)
          c = 1000;

        if (a < 1)
          a = 1;

        int max_agents = (r * c) / 10;
        if (max_agents > 100)
          max_agents = 100;
        if (max_agents < 1)
          max_agents = 1;

        if (a > max_agents)
          a = max_agents;

        generate_random_map("input/random.txt", r, c, a);
        strcpy(out, "input/random.txt");
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

/* -- heuristic picker -------------------------------------------------- */
static void pick_heuristic(int *out)
{
  static const char *labels[] = {
      "Manhattan distance",
      "Euclidean distance",
  };
  static const int heuristics[] = {0, 1}; /* 0=MANHATTAN, 1=EUCLIDEAN */
  const int N = 2;
  int sel = 0;

  const char *header = "Select A* heuristic  (Up/Down to navigate, Enter to confirm)";
  int box_w = 52;
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
        mvprintw(row, box_x + 2, "  >  %-40s", labels[i]);
        attroff(COLOR_PAIR(C_SEL) | A_BOLD);
      }
      else
      {
        attron(COLOR_PAIR(C_NORMAL));
        mvprintw(row, box_x + 2, "     %-40s", labels[i]);
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
      *out = heuristics[sel];
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
void run_menu(char *filepath_out, unsigned int *speed_out, int *heuristic_out)
{
  initscr();
  cbreak();
  noecho();
  curs_set(0);
  keypad(stdscr, TRUE);
  nodelay(stdscr, FALSE);

  start_color();
  use_default_colors();
  init_pair(C_TITLE, COLOR_CYAN, -1);
  init_pair(C_NORMAL, COLOR_WHITE, -1);
  init_pair(C_SEL, COLOR_BLACK, COLOR_CYAN);
  init_pair(C_DIM, COLOR_YELLOW, -1);
  init_pair(C_BORDER, COLOR_BLUE, -1);

  pick_file(filepath_out);
  pick_speed(speed_out);
  pick_heuristic(heuristic_out);

  endwin();
}