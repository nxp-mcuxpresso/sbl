/*
 * Copyright (c) 2013, Freescale Semiconductor, Inc.
 * All rights reserved.
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

#include "utilities/fsl_assert.h"
#include "utilities/ring_buffer.h"

#if !defined(BOOTLOADER_HOST)
#include "fsl_device_registers.h"
#include "utilities/fsl_rtos_abstraction.h"

#endif

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////
// The ring buffer could protect itself by disabling/enabling IRQs

void ring_buffer_init(ring_buffer_t *ring, uint8_t *buffer, uint32_t length)
{
    ring->buffer = buffer;
    ring->length = length;
    ring->filled = 0;
    ring->readHead = buffer;
    ring->writeHead = buffer;
}

uint32_t ring_buffer_read(ring_buffer_t *ring, uint8_t *buffer, uint32_t count)
{
    assert(ring);
    assert(ring->buffer);

#if !defined(BOOTLOADER_HOST)
    lock_acquire();
#endif

    uint32_t actual = 0;
    while (actual < count && ring->filled > 0)
    {
        *buffer++ = *(ring->readHead)++;
        --ring->filled;
        ++actual;

        if (ring->readHead >= ring->buffer + ring->length)
        {
            ring->readHead = ring->buffer;
        }
    }

#if !defined(BOOTLOADER_HOST)
    lock_release();
#endif
    return actual;
}

uint32_t ring_buffer_write(ring_buffer_t *ring, const uint8_t *buffer, uint32_t count)
{
    assert(ring);
    assert(ring->buffer);

#if !defined(BOOTLOADER_HOST)
    lock_acquire();
#endif

    uint32_t actual = 0;
    while (actual < count && (ring->filled == 0 || ring->writeHead != ring->readHead))
    {
        *(ring->writeHead)++ = *buffer++;
        ++ring->filled;
        ++actual;

        if (ring->writeHead >= ring->buffer + ring->length)
        {
            ring->writeHead = ring->buffer;
        }
    }

#if !defined(BOOTLOADER_HOST)
    lock_release();
#endif
    return actual;
}

uint32_t ring_buffer_get_count(ring_buffer_t *ring)
{
    assert(ring);

#if !defined(BOOTLOADER_HOST)
    lock_acquire();
    uint32_t temp = ring->filled;
    lock_release();

    return temp;
#else
    return ring->filled;
#endif
}

bool ring_buffer_is_empty(ring_buffer_t *ring)
{
    assert(ring);

#if !defined(BOOTLOADER_HOST)
    lock_acquire();
    bool temp = (ring->filled == 0);
    lock_release();

    return temp;
#else
    return (ring->filled == 0);
#endif
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
