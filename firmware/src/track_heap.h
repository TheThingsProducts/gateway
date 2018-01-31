// Copyright © 2016-2018 The Things Products
// Use of this source code is governed by the MIT license that can be found in
// the LICENSE file.

/* ************************************************************************** */
/** This is an pass through implementation of a heap. It keeps track of the used
 * heap.

  @File Name
    track_heap.h

  @Summary
    This is an pass through implementation of a heap. It keeps track of the used
    heap.

  @Description
    This is an pass through implementation of a heap. It keeps track of the used
    heap.
 */
/* ************************************************************************** */

#ifndef _TRACK_HEAP_H /* Guard against multiple inclusion */
#define _TRACK_HEAP_H

#include <stddef.h>

void*  TrackHeap_Malloc(size_t xWantedSize);
void*  TrackHeap_Calloc(size_t nmemb, size_t xSize);
void*  TrackHeap_Realloc(void* pv, size_t xWantedSize);
void   TrackHeap_Free(void* pv);
size_t TrackHeap_TotalUsage();
size_t TrackHeap_MaxUsage();
void   TrackHeap_ClearList(void);
void   TrackHeap_PrintList(void);

#endif /* _TRACK_HEAP_H */

/* *****************************************************************************
 End of File
 */
