/*
 * Copyright (c) 2013, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#if !defined(__RING_BUFFER_H__)
#define __RING_BUFFER_H__

#include <stdint.h>

#if !defined(WIN32)
#include <stdbool.h>
#endif

//! @addtogroup ring_buffer
//! @{

////////////////////////////////////////////////////////////////////////////////
// Declarations
////////////////////////////////////////////////////////////////////////////////

//! @brief Ring buffer object.
//!
//! The read and write heads will point to the same byte either when the ring
//! buffer is empty, or when it is full. You can use the filled member to tell
//! which of those two cases applies.
typedef struct _ring_buffer
{
    uint8_t *buffer;    //!< Pointer to the buffer.
    uint32_t length;    //!< Length in bytes of the buffer.
    uint32_t filled;    //!< Number of bytes currently in the buffer.
    uint8_t *readHead;  //!< Pointer to next byte to read.
    uint8_t *writeHead; //!< Pointer to next byte to write.
} ring_buffer_t;

////////////////////////////////////////////////////////////////////////////////
// API
////////////////////////////////////////////////////////////////////////////////

#if __cplusplus
extern "C" {
#endif

//! @name Ring buffer
//@{
//! @brief Initialize a ring buffer.
extern void ring_buffer_init(ring_buffer_t *ring, uint8_t *buffer, uint32_t length);

//! @brief Read data from a ring buffer.
extern uint32_t ring_buffer_read(ring_buffer_t *ring, uint8_t *buffer, uint32_t count);

//! @brief Put data into a ring buffer.
extern uint32_t ring_buffer_write(ring_buffer_t *ring, const uint8_t *buffer, uint32_t count);

//! @brief Returns the number of bytes in a ring buffer.
extern uint32_t ring_buffer_get_count(ring_buffer_t *ring);

//! @brief Returns whether a ring buffer is empty.
extern bool ring_buffer_is_empty(ring_buffer_t *ring);
//@}

#if __cplusplus
}
#endif

//! @}

#endif // __RING_BUFFER_H__
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
