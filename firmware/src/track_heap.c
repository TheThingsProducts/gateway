// Copyright © 2016-2018 The Things Products
// Use of this source code is governed by the MIT license that can be found in
// the LICENSE file.

/* ************************************************************************** */
/** This is an pass through implementation of a heap. It keeps track of the used
 * heap.

  @File Name
    track_heap.c

  @Summary
    This is an pass through implementation of a heap. It keeps track of the used
    heap.

  @Description
    The implementation is based on FreeRTOS's heap_3.c. In addition it adds
 *  magic markers around the functional memory and the allocated size. All
 *  functions do bookkeeping to track the total used heap.
/* ************************************************************************** */

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

/* Defining MPU_WRAPPERS_INCLUDED_FROM_API_FILE prevents task.h from redefining
all the API functions to use the MPU wrappers.  That should only be done when
task.h is included from an application file. */
#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#include "FreeRTOS.h"
#include "task.h"

#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#include "system_definitions.h"
#include "track_heap.h"
#include "utilities.h"

#define PRINT(fmt, ...)
//#define PRINT(fmt, ...)   SYS_PRINT(fmt, ##__VA_ARGS__)
#define WARN(fmt, ...) SYS_PRINT(fmt, ##__VA_ARGS__)
#define PRINT_PTR(_MSG_, _PTR_, _SIZE_)
//#define PRINT_PTR(_MSG_, _PTR_, _SIZE_)    SYS_PRINT("%s: 0x%08X 0x%08X\r\n", _MSG_, _PTR_, _SIZE_)

#define _OVERHEAD_SIZE (sizeof(TRACK_HEAP_HEAD_t) + sizeof(TRACK_HEAP_TAIL_t))
#define _PTR_FROM_DATA_PTR(_DATA_PTR_) (void*)(((void*)(_DATA_PTR_)) - sizeof(TRACK_HEAP_HEAD_t))
#define _HEAD_PTR(_PTR_) ((TRACK_HEAP_HEAD_t*)(_PTR_))
#define _DATA_PTR(_PTR_) (void*)(((void*)(_PTR_)) + sizeof(TRACK_HEAP_HEAD_t))
#define _TAIL_PTR(_PTR_, _SIZE_) (TRACK_HEAP_TAIL_t*)(((void*)(_PTR_)) + ((_SIZE_) - sizeof(TRACK_HEAP_TAIL_t)))

typedef char HEAD_MAGIC_t[8];
typedef char TAIL_MAGIC_t[8];

typedef struct
{
    HEAD_MAGIC_t magic;
    size_t       size; // including header and tail
} TRACK_HEAP_HEAD_t;

typedef struct
{
    TAIL_MAGIC_t magic;
} TRACK_HEAP_TAIL_t;

/* useful for debugging memory leaks
typedef struct {
    void* ptr;
    size_t size;
    uint32_t seq;
} TRACK_ITEM_t;

TRACK_ITEM_t list[100];
uint32_t listSeq = 0;

static void listItemAdd(void* ptr, size_t size)
{
    // find oldest
    uint32_t i, iOld = 0;
    for (i=1; i<sizeof(list)/sizeof(list[0]); i++) {
        if (list[i].seq < list[iOld].seq) {
            iOld = i;
        }
    }

    listSeq++;

    list[iOld].ptr = ptr;
    list[iOld].size = size;
    list[iOld].seq = listSeq;
}

static void listItemRemove(void* ptr)
{
    uint32_t i;
    for (i=0; i<sizeof(list)/sizeof(list[0]); i++) {
        if (list[i].ptr == ptr) {
            list[i].ptr = 0;
            list[i].size = 0;
            list[i].seq = 0;
            return;
        }
    }
}
bool needCheck = false;
void TrackHeap_ClearList(void) {
    memset(list, 0, sizeof(list));
    needCheck = true;
}

void TrackHeap_PrintList(void)
{
    uint32_t i;
    for (i=0; i<sizeof(list)/sizeof(list[0]); i++) {
        if (list[i].seq > 0) {
            SYS_PRINT("%d\t0x%08X\t%d\r\n", list[i].seq, list[i].ptr, list[i].size);
        }
    }
}
*/
#define listItemAdd(_X_, _Y_)
#define listItemRemove(_X_)

/*
 * This file is a layer above the original pvPortMalloc/Free from FreeRTOS. The
 * original functions are renamed to _orig. The functions are 'overloaded'
 * here, wrapping the original functions.
 */
void* pvPortMalloc_orig(size_t xSize) PRIVILEGED_FUNCTION;
void  vPortFree_orig(void* pv) PRIVILEGED_FUNCTION;

const HEAD_MAGIC_t _HEAD_MAGIC = {'h', 'e', 'a', 'p', 's', 't', 'r', 't'};
const HEAD_MAGIC_t _VOID_MAGIC = {'h', 'e', 'a', 'p', 'v', 'o', 'i', 'd'};
const TAIL_MAGIC_t _TAIL_MAGIC = {'h', 'e', 'a', 'p', 'f', 'i', 'n', 'i'};

static size_t _total = 0;
static size_t _max   = 0;

static void  _increase(size_t size);
static void  _decrease(size_t size);
static void* _initialize_to_data_ptr(void* ptr, size_t size);
static void  _deinitialize(void* ptr, size_t size);
static void* _verify_from_data_ptr(void* data_ptr);

void* pvPortMalloc(size_t xWantedSize)
{
    return malloc(xWantedSize);
}
void* malloc(size_t xWantedSize)
{
    void* pvReturn;
    PRINT("+");
    if(xWantedSize == 0)
    {
        PRINT("0");
    }

    vTaskSuspendAll();
    {
        // increase buffer size to hold the size value
        xWantedSize += _OVERHEAD_SIZE;
        // actual allocation
        pvReturn = pvPortMalloc_orig(xWantedSize);
        if(pvReturn != NULL)
        {
            listItemAdd(pvReturn, xWantedSize);
            PRINT_PTR("M", pvReturn, xWantedSize);
            pvReturn = _initialize_to_data_ptr(pvReturn, xWantedSize);
            // bookkeeping: adding newly allocated size total
            _increase(xWantedSize);
        }
    }
    (void)xTaskResumeAll();

    if(pvReturn == NULL)
    {
        FATAL("TRACK_MALLOC NULL");
    }
    return pvReturn;
}

void* calloc(size_t nmemb, size_t xSize)
{
    void* pvReturn;
    PRINT("/");
    if((nmemb * xSize) == 0)
    {
        PRINT("0");
    }

    vTaskSuspendAll();
    {
        // increase buffer size to hold the size value
        size_t xWantedSize = _OVERHEAD_SIZE + nmemb * xSize;
        // actual allocation (used malloc in stead of calloc as it starts with size value)
        pvReturn = pvPortMalloc_orig(xWantedSize);
        if(pvReturn != NULL)
        {
            listItemAdd(pvReturn, xWantedSize);
            PRINT_PTR("C", pvReturn, xWantedSize);
            // initialize memory to zero (as normally done by calloc)
            memset(pvReturn, 0, xWantedSize);
            pvReturn = _initialize_to_data_ptr(pvReturn, xWantedSize);
            // bookkeeping: adding newly allocated size total
            _increase(xWantedSize);
        }
    }
    (void)xTaskResumeAll();

    if(pvReturn == NULL)
    {
        FATAL("TRACK_CALLOC NULL");
    }
    return pvReturn;
}

void* realloc(void* pv, size_t xWantedSize)
{
    void* pvReturn;
    PRINT("=");
    if(xWantedSize == 0)
    {
        PRINT("0");
    }
    vTaskSuspendAll();
    {
        // increase buffer size to hold the size value
        xWantedSize += _OVERHEAD_SIZE;
        // actual allocation
        pvReturn = pvPortMalloc_orig(xWantedSize);
        if(pvReturn != NULL)
        {
            listItemAdd(pvReturn, xWantedSize);
            PRINT_PTR("RN", pvReturn, xWantedSize);
            pvReturn = _initialize_to_data_ptr(pvReturn, xWantedSize);

            if(pv != NULL)
            {
                // Copy and clean up previous allocated data
                pv                            = _verify_from_data_ptr(pv);
                TRACK_HEAP_HEAD_t* head       = _HEAD_PTR(pv);
                size_t             previously = head->size;
                size_t             minSize    = min(xWantedSize, previously);
                listItemRemove(pv);
                PRINT_PTR("RO", pv, previously);
                if(minSize > _OVERHEAD_SIZE)
                {
                    memcpy(pvReturn, _DATA_PTR(pv), minSize - _OVERHEAD_SIZE);
                }
                // make sure the memory is not used anymore
                _deinitialize(pv, previously);
                // bookkeeping: subtract previously allocated size, before adding new
                _decrease(previously);
                vPortFree_orig(pv);
            }
            // bookkeeping: adding newly allocated size total
            _increase(xWantedSize);
        }
    }
    (void)xTaskResumeAll();

    if(pvReturn == NULL)
    {
        FATAL("TRACK_MALLOC NULL");
    }

    return pvReturn;
}

void vPortFree(void* pv)
{
    return free(pv);
}
void free(void* pv)
{
    PRINT("-");
    if(pv)
    {
        vTaskSuspendAll();
        {
            pv = _verify_from_data_ptr(pv);

            TRACK_HEAP_HEAD_t* head       = _HEAD_PTR(pv);
            size_t             previously = head->size;
            if(previously != 0)
            {
                listItemRemove(pv);
                PRINT_PTR("F", pv, previously);
                // make sure the memory is not used anymore
                _deinitialize(pv, previously);
                // bookkeeping: subtract previously allocated size, before adding new
                _decrease(previously);
                // actual free
                vPortFree_orig(pv);
            }
            else
            {
                PRINT("#");
            }
        }
        (void)xTaskResumeAll();
    }
    else
    {
        PRINT("*");
    }
}

size_t TrackHeap_TotalUsage()
{
    return _total;
}

size_t TrackHeap_MaxUsage()
{
    return _max;
}

static void _increase(size_t size)
{
    _total += size;
    if(_total > _max)
    {
        _max = _total;
        PRINT("TRACK_HEAP new max: %dKB\r\n", _max / 1024);
    }
}

static void _decrease(size_t size)
{
    _total -= size;
}

static void* _initialize_to_data_ptr(void* ptr, size_t size)
{
    TRACK_HEAP_HEAD_t* head = _HEAD_PTR(ptr);
    TRACK_HEAP_TAIL_t* tail = _TAIL_PTR(ptr, size);

    ASSERT(size >= _OVERHEAD_SIZE, "Size should be at least size of overhead");

    memcpy(head->magic, _HEAD_MAGIC, sizeof(head->magic));
    head->size = size;
    memcpy(tail->magic, _TAIL_MAGIC, sizeof(tail->magic));

    // set start of buffer for user behind the size
    return _DATA_PTR(ptr);
}

static void _deinitialize(void* ptr, size_t size)
{
    TRACK_HEAP_HEAD_t* head = _HEAD_PTR(ptr);
    TRACK_HEAP_TAIL_t* tail = _TAIL_PTR(ptr, size);

    memcpy(head->magic, _VOID_MAGIC, sizeof(head->magic));
    head->size = 0;
    memset(tail, 0, sizeof(TRACK_HEAP_TAIL_t));
}

static void* _verify_from_data_ptr(void* data_ptr)
{
    void*              ptr  = _PTR_FROM_DATA_PTR(data_ptr);
    TRACK_HEAP_HEAD_t* head = _HEAD_PTR(ptr);
    TRACK_HEAP_TAIL_t* tail;

    if(memcmp(head->magic, _HEAD_MAGIC, sizeof(head->magic)) != 0)
    {
        // if header magic is not correct, check if it contains the void magic
        if(memcmp(head->magic, _VOID_MAGIC, sizeof(head->magic)) == 0 && head->size == 0)
        {
            // contains the void magic, means that it was already freed: warn and ignore
            WARN("Head magic match void: trying to free an already freed block, ignore\r\n");
        }
        else
        {
            // header magic does not contain correct magic nor void magic, trying to free unknown pointer
            FATAL("Head magic should match: trying to free an unallocated block\r\n");
        }
    }
    else
    {
        // if header contains correct magic, check the size and tail magic
        ASSERT(head->size >= _OVERHEAD_SIZE, "Size should be at least size of overhead");
        tail = _TAIL_PTR(ptr, head->size);
        ASSERT(memcmp(tail->magic, _TAIL_MAGIC, sizeof(tail->magic)) == 0, "Tail magic should match");
    }

    // set start of buffer for user behind the size
    return ptr;
}
