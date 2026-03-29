#include <stdlib.h>
#include <stdio.h>
#include "../include/heap.h"

static void swap(AStarNode *a, AStarNode *b) {
  AStarNode c = *a;
  *a = *b;
  *b = c;
}

MinHeap heap_create(int initial_capacity) {
  // allocate array of AStarNodes
  AStarNode *data = malloc(initial_capacity * sizeof(AStarNode));
  // check if allocation failed
  if (data == NULL) {
    perror("MinHeap initial allocation failed");
    return (MinHeap){NULL, 0, 0};
  }
  int size = 0;                     // init size
  int capacity = initial_capacity;  // init capacity
  return (MinHeap){data, size, capacity};
}

void heap_push(MinHeap *heap, AStarNode node) {
  // if heap is full, double size
  if (heap->size == heap->capacity) {
    AStarNode *temp_arr = (AStarNode*)realloc(heap->data, heap->capacity*2*sizeof(AStarNode));
    // check if reallocation was failed
    if (temp_arr == NULL) {
      perror("MinHeap reallocation failed");
      return;
    }
    heap->capacity *= 2;   // update new capacity
    heap->data = temp_arr; // swap the old array for the new one
  }

  int index = heap->size;   // get the index of where the last element (the inserted one) will be
  heap->data[index] = node; // add new element to the end of the heap
  heap->size++;
  // reorder the heap
  while (index > 0 && heap->data[(index - 1) / 2].f > heap->data[index].f) {
    swap(&heap->data[index], &heap->data[(index - 1) / 2]);
    // Move up the tree to the
    //parent of the current element
    index = (index - 1) / 2;
  }
  return;
}

AStarNode heap_pop(MinHeap *heap) {
  // pop called on empty heap
  if (heap->size == 0) {
    perror("heap_pop called on empty heap");
    return (AStarNode){NULL, 0, 0};
  }

  AStarNode min_node = *heap->data;                  // retrieve the first element of the min heap
  AStarNode last_node = heap->data[heap->size - 1];  // retrieve the last element
  heap->size--;                                      // shrink heap by 1 (removed element)
  *heap->data = last_node;                           // move last node to the top of the heap

  // heapify down
  int index = 0;
  while (1) {
    int left  = 2 * index + 1;
    int right = 2 * index + 2;
    int smallest = index;
    
    if (left < heap->size && heap->data[left].f < heap->data[smallest].f) {
      smallest = left;
    }
    if (right < heap->size && heap->data[right].f < heap->data[smallest].f) {
      smallest = right;
    }
    
    if (smallest == index) break;
    
    swap(&heap->data[index], &heap->data[smallest]);
    index = smallest;
  }

  // if heap is 1/4 its capacity shrink it
  if (heap->size < 0.25 * heap->capacity) {
    AStarNode *temp_arr = (AStarNode*)realloc(heap->data, heap->capacity / 2 * sizeof(AStarNode));
    // check if reallocation was failed
    if (temp_arr == NULL) {
      perror("MinHeap reallocation failed");
      return min_node;
    }
    heap->capacity /= 2; // update new capacity
    heap->data = temp_arr; // swap old array for the new one
  }

  return min_node;
}

void heap_free(MinHeap *heap) {
    free(heap->data);
    heap->data = NULL;
    heap->size = 0;
    heap->capacity = 0;
}