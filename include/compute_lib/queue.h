#ifndef __QUEUE_H__
#define __QUEUE_H__

#include <stdbool.h>
#include <stdlib.h>

#define QUEUE_MIN_SIZE 32

typedef struct {
	int start_pos;
	int end_pos;
	int max_size;
	void** content;
} queue_t;

queue_t* queue_create(int capacity);
void queue_delete(queue_t* queue);
bool queue_push(queue_t* queue, void* data);
void* queue_pop(queue_t* queue);
void* queue_get(queue_t* queue, int idx);
int queue_size(queue_t* queue);

#endif
