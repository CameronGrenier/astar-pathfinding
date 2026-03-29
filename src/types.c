#include "../include/types.h"

int nodes_equal(Node *a, Node *b)
{
  return a->pos.x == b->pos.x && a->pos.y == b->pos.y;
}

int coords_equal(Coords a, Coords b)
{
  return a.x == b.x && a.y == b.y;
}