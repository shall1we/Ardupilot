#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_QURT
#include "Semaphores.h"
#include <stdio.h>

extern const AP_HAL::HAL& hal;

#include "Util.h"

using namespace QURT;

extern "C" {
    void *fc_heap_alloc(size_t size);
    void fc_heap_free(void* ptr);
    size_t fc_heap_size(void);
    size_t fc_heap_usage(void);
}

uint32_t Util::available_memory(void)
{
    return fc_heap_size() - fc_heap_usage();
}

#if ENABLE_HEAP
void *Util::allocate_heap_memory(size_t size)
{
    struct heap *new_heap = (struct heap*)malloc(sizeof(struct heap));
    if (new_heap != nullptr) {
        new_heap->max_heap_size = size;
        new_heap->current_heap_usage = 0;
    }
    return (void *)new_heap;
}

void *Util::heap_realloc(void *h, void *ptr, size_t old_size, size_t new_size)
{
    if (h == nullptr) {
        return nullptr;
    }

    struct heap *heapp = (struct heap*)h;

    // extract appropriate headers. We use the old_size from the
    // header not from the caller. We use SITL to catch cases they
    // don't match (which would be a lua bug)
    old_size = 0;
    heap_allocation_header *old_header = nullptr;
    if (ptr != nullptr) {
        old_header = ((heap_allocation_header *)ptr) - 1;
        old_size = old_header->allocation_size;
    }

    if ((heapp->current_heap_usage + new_size - old_size) > heapp->max_heap_size) {
        // fail the allocation as we don't have the memory. Note that we don't simulate fragmentation
        return nullptr;
    }

    heapp->current_heap_usage -= old_size;
    if (new_size == 0) {
       free(old_header);
       return nullptr;
    }

    heap_allocation_header *new_header = (heap_allocation_header *)malloc(new_size + sizeof(heap_allocation_header));
    if (new_header == nullptr) {
        // total failure to allocate, this is very surprising in SITL
        return nullptr;
    }
    heapp->current_heap_usage += new_size;
    new_header->allocation_size = new_size;
    void *new_mem = new_header + 1;

    if (ptr == nullptr) {
        return new_mem;
    }
    memcpy(new_mem, ptr, old_size > new_size ? new_size : old_size);
    free(old_header);
    return new_mem;
}

#endif // ENABLE_HEAP

#endif // CONFIG_HAL_BOARD == HAL_BOARD_QURT
