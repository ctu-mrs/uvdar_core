#include "queue.h"

// Creates queue instance
queue_t* queue_create(int capacity)
{
    queue_t* new = (queue_t*)malloc(sizeof(queue_t));
    new->start_pos = 0;
    new->end_pos = 0;
    new->max_size = capacity;
    new->content = (void**)malloc(capacity * sizeof(void*));
    return new;
}

// Deallocates queue resources
void queue_delete(queue_t* queue)
{
    free(queue->content);
    free(queue);
}

// Changes the size of the queue to newsize
// If resize_always equals true, the current size does not have to be equal to maximum
bool queue_resize(queue_t* queue, bool resize_always, int newsize)
{
    int size = queue_size(queue);
    // move elements to the beginning
    if (queue->start_pos > 0) {
        for (int i = 0; i < size; i++) {
            queue->content[i] = queue->content[queue->start_pos + i];
        }
    }
    // reallocate memory
    if (resize_always || size == queue->max_size) {
        queue->max_size = newsize;
        void** new_content = (void**)realloc(queue->content, queue->max_size * sizeof(void*));
        if (new_content == NULL) {
            return false;
        }
        queue->content = new_content;
    }
    queue->start_pos = 0;
    queue->end_pos = size;

    return true;
}

// Appends new element to the end of the queue and increments end_pos by 1
bool queue_push(queue_t* queue, void* data)
{
    if (queue->end_pos == queue->max_size) {
        if (!queue_resize(queue, false, queue->max_size * 2)) {
			return false;
		}
    }
    queue->content[queue->end_pos++] = data;
    return true;
}

// Pops the first element of the queue and increments start_pos by 1
void* queue_pop(queue_t* queue)
{
    int size = queue_size(queue);
    if (size > 0) {
        if (3 * size < queue->max_size && queue->max_size >= 3 * QUEUE_MIN_SIZE) {
            queue_resize(queue, true, (queue->max_size + 2) / 3);
        }
        
        return queue->content[queue->start_pos++];
    }
    return NULL;
}

// Non-destructive read of i-th element in the queue
void* queue_get(queue_t* queue, int idx)
{
    if (idx >= 0 && idx < queue_size(queue)) {
        return queue->content[queue->start_pos + idx];
    }
    return NULL;
}

// Returns the queue size
int queue_size(queue_t* queue)
{
    return queue->end_pos - queue->start_pos;
}
