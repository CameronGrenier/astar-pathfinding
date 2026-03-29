#ifndef HEAP_H
#define HEAP_H
#include "astar.h"

/**
 * @brief A min-heap of AStarNodes ordered by f value.
 *
 * Backed by a dynamically resizing array.
 * Doubles in capacity when full, halves when quarter full.
 */
typedef struct {
    AStarNode *data; /**< Dynamically allocated array of AStarNodes */
    int size;        /**< Current number of elements in the heap */
    int capacity;    /**< Current allocated capacity of the array */
} MinHeap;

/**
 * @brief Creates a new empty MinHeap with the given initial capacity.
 *
 * @param initial_capacity Initial size of the backing array
 * @return Initialized MinHeap, or {NULL, 0, 0} if allocation failed
 */
MinHeap heap_create(int initial_capacity);

/**
 * @brief Pushes a new AStarNode onto the heap.
 *
 * Resizes the backing array if at capacity.
 * Maintains heap property via swim-up.
 *
 * @param heap Pointer to the MinHeap
 * @param node AStarNode to insert
 */
void heap_push(MinHeap *heap, AStarNode node);

/**
 * @brief Removes and returns the AStarNode with the lowest f value.
 *
 * Shrinks the backing array if size drops below 1/4 capacity.
 * Maintains heap property via sink-down.
 *
 * @param heap Pointer to the MinHeap
 * @return AStarNode with the lowest f value, or {NULL, 0, 0} if heap is empty
 */
AStarNode heap_pop(MinHeap *heap);

/**
 * @brief Frees the backing array and resets the heap fields.
 *
 * Does not free the MinHeap struct itself as it is stack allocated.
 *
 * @param heap Pointer to the MinHeap to free
 */
void heap_free(MinHeap *heap);

#endif