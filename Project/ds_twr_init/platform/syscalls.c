/*! ----------------------------------------------------------------------------
 *  @file    syscalls.c
 *  @brief   Standard syscalls platform-specific implementation
 *
 * @attention
 *
 * Copyright 2015 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */
#include <sys/types.h>
#include "port.h"

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn _sbrk()
 *
 * @brief  Increment heap pointer to allow dynamic memory allocation.
 *
 * @param  incr  size of the heap increment to perform, in bytes
 *
 * @return  pointer on the previous heap end or -1 if memory is full and increment cannot be performed.
 */
caddr_t _sbrk(int incr)
{
    extern char _ebss; /* Defined by the linker. */
    static char *heap_end;
    char *prev_heap_end;
    char *stack;

    if (heap_end == 0)
    {
        heap_end = &_ebss;
    }

    prev_heap_end = heap_end;

    stack = (char*) port_GET_stack_pointer();

    if (heap_end + incr > stack)
    {
        return (caddr_t) -1;
    }

    heap_end += incr;

    return (caddr_t) prev_heap_end;
}
