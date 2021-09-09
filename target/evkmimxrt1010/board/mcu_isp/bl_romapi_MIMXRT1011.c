/*
* The Clear BSD License
* Copyright 2016-2021 NXP
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted (subject to the limitations in the disclaimer below) provided
*  that the following conditions are met:
*
* o Redistributions of source code must retain the above copyright notice, this list
*   of conditions and the following disclaimer.
*
* o Redistributions in binary form must reproduce the above copyright notice, this
*   list of conditions and the following disclaimer in the documentation and/or
*   other materials provided with the distribution.
*
* o Neither the name of the copyright holder nor the names of its
*   contributors may be used to endorse or promote products derived from this
*   software without specific prior written permission.
*
* NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
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

// Functions from ROM are applied so as to reduce Flashloader's binary size

#include "fsl_clock.h"
#include "fsl_flexspi.h"
#include "flexspi_nor_flash.h"

/*****************************************************************************************************/
/*                                        FlexSPI APIs                                               */
/*****************************************************************************************************/
bool flexspi_is_parallel_mode(flexspi_mem_config_t *config)
{
    bool (*const _flexspi_is_parallel_mode)(flexspi_mem_config_t *) = (bool (*)(flexspi_mem_config_t *))0x0020c687;
    return _flexspi_is_parallel_mode(config);
}

status_t flexspi_device_wait_busy(uint32_t instance,
                                  flexspi_mem_config_t *config,
                                  bool isParallelMode,
                                  uint32_t baseAddr)
{
    status_t (*const _flexspi_device_wait_busy)(uint32_t, flexspi_mem_config_t *, bool, uint32_t) =
        (status_t (*)(uint32_t, flexspi_mem_config_t *, bool, uint32_t))0x0020c025;
    return _flexspi_device_wait_busy(instance, config, isParallelMode, baseAddr);
}

status_t flexspi_command_xfer(uint32_t instance, flexspi_xfer_t *xfer)
{
    status_t (*const _flexspi_command_xfer)(uint32_t, flexspi_xfer_t *) =
        (status_t (*)(uint32_t, flexspi_xfer_t *))0x0020bb75;
    return _flexspi_command_xfer(instance, xfer);
}

status_t flexspi_update_lut(uint32_t instance, uint32_t seqIndex, const uint32_t *lutBase, uint32_t seqNumber)
{
    status_t (*const _flexspi_update_lut)(uint32_t, uint32_t, const uint32_t *, uint32_t) =
        (status_t (*)(uint32_t, uint32_t, const uint32_t *, uint32_t))0x0020c815;
    return _flexspi_update_lut(instance, seqIndex, lutBase, seqNumber);
}

void flexspi_clear_cache(uint32_t instance)
{
    void (*const _flexspi_clear_cache)(uint32_t) = (void (*)(uint32_t))0x0020ba3f;
    _flexspi_clear_cache(instance);
}

status_t flexspi_device_write_enable(uint32_t instance,
                                     flexspi_mem_config_t *config,
                                     bool isParallelMode,
                                     uint32_t baseAddr)
{
    status_t (*const _flexspi_device_write_enable)(uint32_t, flexspi_mem_config_t *, bool, uint32_t) =
        (status_t (*)(uint32_t, flexspi_mem_config_t *, bool, uint32_t))0x0020c1cd;
    return _flexspi_device_write_enable(instance, config, isParallelMode, baseAddr);
}

void flexspi_wait_idle(uint32_t instance)
{
    void (*const _flexspi_wait_idle)(uint32_t) = (void (*)(uint32_t))0x0020c87d;
    _flexspi_wait_idle(instance);
}

status_t flexspi_configure_dll(uint32_t instance, flexspi_mem_config_t *config)
{
    status_t (*const _flexspi_configure_dll)(uint32_t, flexspi_mem_config_t *) =
        (status_t (*)(uint32_t, flexspi_mem_config_t *))0x0020be25;
    return _flexspi_configure_dll(instance, config);
}

status_t flexspi_init(uint32_t instance, flexspi_mem_config_t *config)
{
    status_t (*const _flexspi_init)(uint32_t, flexspi_mem_config_t *) =
        (status_t (*)(uint32_t, flexspi_mem_config_t *))0x0020c339;
    return _flexspi_init(instance, config);
}

void flexspi_half_clock_control(uint32_t instance, uint32_t option)
{
    do
    {
        FLEXSPI_Type *(*const _flexspi_get_module_base)(uint32_t) = (FLEXSPI_Type * (*)(uint32_t))0x0020c2f1;
        FLEXSPI_Type *base = _flexspi_get_module_base(instance);

        if (base == NULL)
        {
            break;
        }

        void (*const _flexspi_wait_until_ip_idle)(FLEXSPI_Type *) = (void (*)(FLEXSPI_Type *))0x0020c893;
        _flexspi_wait_until_ip_idle(base);

        if (option)
        {
            base->MCR0 |= FLEXSPI_MCR0_HSEN_MASK;
        }
        else
        {
            base->MCR0 &= (uint32_t)~FLEXSPI_MCR0_HSEN_MASK;
        }

    } while (0);
}

/*****************************************************************************************************/
/*                                          Clocking APIs                                                */
/*****************************************************************************************************/
/*
void clock_setup(void)
{
#define _SystemCoreClock (*(const uint32_t *)0x20203a00)
    void (*const _clock_setup)(void) = (void (*)(void))0x0020cf67;
    _clock_setup();
    SystemCoreClock = _SystemCoreClock;
}
*/

status_t flexspi_get_clock(uint32_t instance, flexspi_clock_type_t type, uint32_t *freq)
{
    status_t (*const _flexspi_get_clock)(uint32_t, flexspi_clock_type_t, uint32_t *) =
        (status_t (*)(uint32_t, flexspi_clock_type_t, uint32_t *))0x0020c225;
    return _flexspi_get_clock(instance, type, freq);
}
