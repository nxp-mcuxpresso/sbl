/*
 * Copyright 2017-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include "bootloader/bl_context.h"
#include "bootloader_common.h"
#include "fsl_clock.h"
#include "fsl_device_registers.h"
#include "fsl_reset.h"
#include "microseconds/microseconds.h"
#include "peripherals_pinmux.h"
#include "utilities/fsl_assert.h"
#if BL_FEATURE_AUTHENTICATION
#include "authentication/authentication.h"
#include "authentication/authentication_hal.h"
#include "authentication/secure_kboot.h"
#include "authentication/skboot_debug_auth_hal.h"
#include "authentication/skboot_image.h"
#endif
//#include "bm_usb/dfu_timer.h"
#include "bootloader/bootloader.h"
//#include "fsl_flexspi.h"
#include "fusemap.h"
//#include "mrt/fsl_mrt.h"
#if BL_FEATURE_OTP_MODULE
#include "otp/fsl_otp.h"
#endif
#include "flexspi_nor_flash.h"
#include "target_config.h"
#include "usb_device_descriptor.h"
//#include "usb_ksdk/api_usb_ksdk.h"

#include "bootloader/bootloader.h"
//#include "mailbox/hw_dbg_mailbox.h"
#include "property/property.h"
//#include "romcp/fsl_romcp.h"
//#include "api_tree_root.h"

//#include "trustzone/tzm.h"
//#include "trustzone/tzm_hal.h"
//#include "trustzone/tzm_mpu.h"

//#include "flexcomm/fsl_spi.h"

#include "clock_config.h"
#include "board.h"
   
////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

#define HARDWARE_IS_XIP_FLEXSPI()                                                                               \
    ((((uint32_t)update_available_peripherals >= 0x08000000U) && ((uint32_t)update_available_peripherals < 0x10000000U)) || \
     (((uint32_t)update_available_peripherals >= 0x18000000U) && ((uint32_t)update_available_peripherals < 0x20000000U)))

#define BOOT_PIN_DEBOUNCE_READ_COUNT 500
enum
{
    kSize_32KB = 32u * 1024u,
    kSize_64KB = 64u * 1024u,
    kSize_128KB = 128u * 1024u,
    kSize_256KB = 256u * 1024u
};

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////
extern bool usb_hs0_device_clock_enable(void);
extern void USB_DeviceClockInit(void);
extern void flexspi_sw_delay_us(uint64_t us);

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

void go_fatal_mode(void)
{
    while (1);
}

bool is_boot_pin_asserted(void)
{
    // Boot pin for Flash only target
    return false;
}

void update_memory_map_lpc_sram(void)
{
    const uint32_t kSramPartitionSize[32] = {
        kSize_32KB,  kSize_32KB,  kSize_32KB,  kSize_32KB,  // 0 - 3
        kSize_32KB,  kSize_32KB,  kSize_32KB,  kSize_32KB,  // 4 - 7
        kSize_64KB,  kSize_64KB,  kSize_64KB,  kSize_64KB,  // 8 - 11
        kSize_128KB, kSize_128KB, kSize_128KB, kSize_128KB, // 12 - 15
        kSize_256KB, kSize_256KB, kSize_256KB, kSize_256KB, // 16 - 19
        kSize_256KB, kSize_256KB, kSize_256KB, kSize_256KB, // 20 - 23
        kSize_256KB, kSize_256KB, kSize_256KB, kSize_256KB, // 24 - 27
        kSize_256KB, kSize_256KB, 0,           0,           // 28 - 31
    };
    static uint32_t s_sramIndex = 0;

    // Detect the maximum RAM size
    uint32_t ramPartionSizeIndex = 31;
    uint32_t sramCfgEnable0 = *(uint32_t *)(SYSCTL0_BASE + 0x40);
    while ((sramCfgEnable0 & (1u << ramPartionSizeIndex)) == 0)
    {
        ramPartionSizeIndex--;
    }

    // Update the RAM size after all required SRAM partitions are powered up.
    uint32_t actualRamSize = 0;
    for (uint32_t i = 0; i <= ramPartionSizeIndex; i++)
    {
        actualRamSize += kSramPartitionSize[i];
    }

    // Update the memory map to match the actual SoC configuration.
    memory_map_entry_t *mapEntry = (memory_map_entry_t *)g_bootloaderContext.memoryMap;
    uint32_t index = 0;
    while ((mapEntry != NULL) && mapEntry->memoryInterface != NULL)
    {
        if (mapEntry->memoryProperty & kMemoryType_RAM)
        {
            if (s_sramIndex == index)
            {
                mapEntry->endAddress = mapEntry->startAddress + actualRamSize - 1;
                ++s_sramIndex;
                debug_printf("memory_map_entry: start=%x, end=%x\n", mapEntry->startAddress, mapEntry->endAddress);
            }
        }
        ++index;
        ++mapEntry;
    }
}

void init_hardware_api(void)
{
}

void init_system(void)
{
    if (HARDWARE_IS_XIP_FLEXSPI() && (CACHE64_POLSEL->POLSEL == 0x1u)) /* Enable cache to accelerate boot. */
    {
        /* set command to invalidate all ways and write GO bit to initiate command */
        CACHE64->CCR = CACHE64_CTRL_CCR_INVW1_MASK | CACHE64_CTRL_CCR_INVW0_MASK;
        CACHE64->CCR |= CACHE64_CTRL_CCR_GO_MASK;
        /* Wait until the command completes */
        while (CACHE64->CCR & CACHE64_CTRL_CCR_GO_MASK)
        {
        }
        /* Disable cache */
        CACHE64->CCR &= ~CACHE64_CTRL_CCR_ENCACHE_MASK;
        CACHE64_POLSEL->POLSEL = 0;

        __ISB();
        __DSB();
    }
}

void deinit_system(void)
{
    if (HARDWARE_IS_XIP_FLEXSPI() && (CACHE64_POLSEL->POLSEL == 0)) /* Enable cache to accelerate boot. */
    {
        /* set command to invalidate all ways and write GO bit to initiate command */
        CACHE64->CCR = CACHE64_CTRL_CCR_INVW1_MASK | CACHE64_CTRL_CCR_INVW0_MASK;
        CACHE64->CCR |= CACHE64_CTRL_CCR_GO_MASK;
        /* Wait until the command completes */
        while (CACHE64->CCR & CACHE64_CTRL_CCR_GO_MASK)
        {
        }
        /* Enable cache, enable write buffer */
        CACHE64->CCR = (CACHE64_CTRL_CCR_ENWRBUF_MASK | CACHE64_CTRL_CCR_ENCACHE_MASK);

        /* Set whole FlexSPI0 space to write through. */
        CACHE64_POLSEL->REG0_TOP = 0x07FFFC00U;
        CACHE64_POLSEL->REG1_TOP = 0x0U;
        CACHE64_POLSEL->POLSEL = 0x1U;

        __ISB();
        __DSB();
    }
}

void init_hardware(void)
{
    init_system();

    debug_printf("%s\n", __func__);

    // Power on OSPI RAM as needed
    uint32_t flexspiEnMask = 0x10000;
    uint32_t periCfgEnable0 = *(uint32_t *)(SYSCTL0_BASE + 0x50);
    if (periCfgEnable0 & flexspiEnMask)
    {
#if BL_TARGET_RAM
        /* Clear OSPI_OTFAD reset bit */
        RESET_PeripheralReset(kFLEXSPI_RST_SHIFT_RSTn);
#endif
        // Power on OSPI RAM
        SYSCTL0->PDRUNCFG1_CLR = (SYSCTL0_PDRUNCFG1_FLEXSPI_SRAM_APD_MASK | SYSCTL0_PDRUNCFG1_FLEXSPI_SRAM_PPD_MASK);
    }

    CLOCK_EnableClock(kCLOCK_Flexcomm0);

    /* reset FLEXCOMM0 for USART0 */
    RESET_PeripheralReset(kFC0_RST_SHIFT_RSTn);
    /* Reset Inputmux  for auto-buad detection feature */
    RESET_PeripheralReset(kINPUTMUX_RST_SHIFT_RSTn);

    // Enable CRC module
    CLOCK_EnableClock(kCLOCK_Crc);
    RESET_PeripheralReset(kCRC_RST_SHIFT_RSTn);

    // Configure clock here because the configure_clocks has been removed from bl_main.c
    //configure_clocks(kClockOption_EnterBootloader);
    BOARD_BootClockHSRUN();

    /* attach FRG0 clock to FLEXCOMM0 (debug console) */
    CLOCK_SetFRGClock(BOARD_DEBUG_UART_FRG_CLK);
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

    LOG_ADD_ENTRY(kLog_HardwareInit_Pass);
}

void deinit_hardware(void)
{
    // Shutdown USB RAM
    SYSCTL0->PDRUNCFG1_SET = SYSCTL0_PDRUNCFG1_USBHS_SRAM_APD_MASK | SYSCTL0_PDRUNCFG1_USBHS_SRAM_PPD_MASK;

#if BL_FEATURE_OTP_MODULE
    // Shutdown OTP
    OCOTP->OTP_PDN |= OCOTP_OTP_PDN_PDN_MASK;
#endif

    LOG_ADD_ENTRY(kLog_HardwareDeInit_Pass);

    debug_printf("%s, SCB->VTOR=%x\n", __func__, SCB->VTOR);

    deinit_system();
}

void usb_update_serial_number_str_desc_according_to_uuid(void)
{
    // ROM shall create 16 byte unicode string from 128-bit UUID of the device
    // as "serial number string descriptor"

    uint8_t *uuid = (uint8_t *)&SYSCTL0->UUID[0];
    for (uint32_t i = 0; i < 16; i++)
    {
        g_usb_str_4[2 + 2 * i] = *uuid++;
    }
}

void usb_peripheral_reset(void)
{
    RESET_PeripheralReset(kUSBHS_PHY_RST_SHIFT_RSTn);
    RESET_PeripheralReset(kUSBHS_DEVICE_RST_SHIFT_RSTn);
    RESET_PeripheralReset(kUSBHS_HOST_RST_SHIFT_RSTn);
    RESET_PeripheralReset(kUSBHS_SRAM_RST_SHIFT_RSTn);
}

bool usb_clock_init(uint32_t controllerId)
{
    USB_DeviceClockInit();

    return true;
}
    
bool usb_clock_init_backup(uint32_t controllerId)
{
    // Power on USB RAM
    SYSCTL0->PDRUNCFG1_CLR = SYSCTL0_PDRUNCFG1_USBHS_SRAM_APD_MASK | SYSCTL0_PDRUNCFG1_USBHS_SRAM_PPD_MASK;

    /* reset USBs */
    usb_peripheral_reset();

#if USB_DEVICE_CONFIG_LPCIP3511HS
    /* enable usb1 phy */
    uint32_t usbPdMask = 0x4000000;
    SYSCTL0->PDRUNCFG0_CLR |= usbPdMask;
    /* enable usb1 host clock */
    CLOCK_EnableClock(kCLOCK_UsbhsHost);
    uint32_t portModeMask = 0x10000;
    *((uint32_t *)(USBHSH_BASE + 0x50)) |= portModeMask;
    /* enable usb1 host clock */
    CLOCK_DisableClock(kCLOCK_UsbhsHost);
    /* enable USB IP clock */
    bool result = usb_hs0_device_clock_enable();
    if (result != true)
    {
        return result;
    }
#if defined(FSL_FEATURE_USBHSD_USB_RAM) && (FSL_FEATURE_USBHSD_USB_RAM)
    uint32_t ramSize32bit = FSL_FEATURE_USBHSD_USB_RAM / 4;
    volatile uint32_t *usbRamStart = (volatile uint32_t *)FSL_FEATURE_USBHSD_USB_RAM_BASE_ADDRESS;
    for (int i = 0; i < ramSize32bit; i++)
    {
        *usbRamStart++ = 0x00U;
    }
#endif
#endif

    return true;
}

//! @brief Returns the current flexcomm clock frequency in Hertz.
uint32_t get_flexcomm_clock(uint32_t instance)
{
#if !defined(BL_TARGET_FPGA)
    switch (instance)
    {
        case 0:
            return CLOCK_GetFreq(kCLOCK_Flexcomm0Clk);
        case 2:
            return CLOCK_GetFreq(kCLOCK_Flexcomm2Clk);
        case 9:
        // return CLOCK_GetFreq(kCLOCK_Flexcomm9Clk);
        default:
            break;
    }

    return 0;
#else
    return SystemCoreClock;
#endif
}

bool flexspi_is_differential_clock_enable(flexspi_mem_config_t *config)
{
    if (config->controllerMiscOption & FLEXSPI_BITMASK(kFlexSpiMiscOffset_DiffClkEnable))
    {
        return true;
    }
    else
    {
        return false;
    }
}

void flexspi_iomux_config(uint32_t instance, flexspi_mem_config_t *config)
{
#define CS_PAD_SETTING (IOPCTL_PIO_FSEL(1) | IOPCTL_PIO_FULLDRIVE(1) | IOPCTL_PIO_IBENA(1))
#define SCLK_PAD_SETTING (IOPCTL_PIO_FSEL(1) | IOPCTL_PIO_FULLDRIVE(1) | IOPCTL_PIO_IBENA(1))
#define DATA_PAD_SETTING (IOPCTL_PIO_FSEL(1) | IOPCTL_PIO_FULLDRIVE(1) | IOPCTL_PIO_IBENA(1))
#define DQS_PAD_SETTING (IOPCTL_PIO_FSEL(1) | IOPCTL_PIO_FULLDRIVE(1) | IOPCTL_PIO_PUPDENA(1) | IOPCTL_PIO_IBENA(1))

#define CS_CHNB_PAD_SETTING (IOPCTL_PIO_FSEL(6) | IOPCTL_PIO_FULLDRIVE(1) | IOPCTL_PIO_IBENA(1))
#define SCLK_CHNB_PAD_SETTING (IOPCTL_PIO_FSEL(6) | IOPCTL_PIO_FULLDRIVE(1) | IOPCTL_PIO_IBENA(1))
#define DATA_CHNB_PAD_SETTING (IOPCTL_PIO_FSEL(6) | IOPCTL_PIO_FULLDRIVE(1) | IOPCTL_PIO_IBENA(1))
#define DQS_CHNB_PAD_SETTING \
    (IOPCTL_PIO_FSEL(6) | IOPCTL_PIO_FULLDRIVE(1) | IOPCTL_PIO_PUPDENA(1) | IOPCTL_PIO_IBENA(1))

    if (OTP_QSPI_PAD_CONTROL_OVERRIDE_VALUE())
    {
        // set this highspeed pad comp timeout to 100
        int32_t timeout = HS_PAD_COMP_TMO;
        while (((SYSCTL0->FLEXSPIPADCTRL & SYSCTL0_FLEXSPIPADCTRL_COMPOK_MASK) == 0) || (timeout-- > 0))
        {
        }

        if (OTP_QSPI_PAD_CTL_FREEZE_VALUE() != 0)
        {
            if (!(SYSCTL0->FLEXSPIPADCTRL & (1u << 9)))
            {
                SYSCTL0->FLEXSPIPADCTRL |= (1 << 9);
            }
        }
        else if (OTP_QSPI_PAD_CTL_FAST_FREEZE_VALUE() != 0)
        {
            if (!(SYSCTL0->FLEXSPIPADCTRL & (1u << 8)))
            {
                SYSCTL0->FLEXSPIPADCTRL |= (1 << 8);
            }
        }
    }

    if (config->sflashA1Size || config->sflashA2Size)
    {
        uint32_t sclkPadSetting = config->sclkPadSettingOverride ? config->sclkPadSettingOverride : SCLK_PAD_SETTING;
        uint32_t dqsPadSetting = config->dqsPadSettingOverride ? config->dqsPadSettingOverride : DQS_PAD_SETTING;
        uint32_t csPadSetting = config->csPadSettingOverride ? config->csPadSettingOverride : CS_PAD_SETTING;
        uint32_t dataPadSetting = config->dataPadSettingOverride ? config->dataPadSettingOverride : DATA_PAD_SETTING;

        sclkPadSetting = (sclkPadSetting & ~IOPCTL_PIO_FSEL_MASK) | IOPCTL_PIO_FSEL(1);
        dqsPadSetting = (sclkPadSetting & ~IOPCTL_PIO_FSEL_MASK) | IOPCTL_PIO_FSEL(1);
        csPadSetting = (csPadSetting & ~IOPCTL_PIO_FSEL_MASK) | IOPCTL_PIO_FSEL(1);
        dataPadSetting = (dataPadSetting & ~IOPCTL_PIO_FSEL_MASK) | IOPCTL_PIO_FSEL(1);

        if (config->sflashA1Size)
        {
            // CS0
            IOPCTL->PIO[1][19] = csPadSetting;
        }

        if (config->sflashA2Size)
        {
            // CS1
            IOPCTL->PIO[1][29] = csPadSetting;
        }

        // CLK0
        IOPCTL->PIO[1][18] = sclkPadSetting;
        // D0
        IOPCTL->PIO[1][20] = dataPadSetting;
        // D1
        IOPCTL->PIO[1][21] = dataPadSetting;

        /* Enable Quad Pads by default in case some Flash has the hold pin enabled by default */
        // D2
        IOPCTL->PIO[1][22] = dataPadSetting;
        // D3
        IOPCTL->PIO[1][23] = dataPadSetting;

        if (config->sflashPadType == kSerialFlash_8Pads)
        {
            // D4
            IOPCTL->PIO[1][24] = dataPadSetting;
            // D5
            IOPCTL->PIO[1][25] = dataPadSetting;
            // D6
            IOPCTL->PIO[1][26] = dataPadSetting;
            // D7
            IOPCTL->PIO[1][27] = dataPadSetting;
        }

        // DQS
        IOPCTL->PIO[1][28] = dqsPadSetting;

        if (flexspi_is_differential_clock_enable(config))
        {
            // CLK0_B
            IOPCTL->PIO[1][29] = (sclkPadSetting & ((uint32_t)~IOPCTL_PIO_FSEL_MASK)) | IOPCTL_PIO_FSEL(5);
        }
    }

    if (config->sflashB1Size || config->sflashB2Size)
    {
        uint32_t sclkChnbPadSetting =
            config->sclkPadSettingOverride ? config->sclkPadSettingOverride : SCLK_CHNB_PAD_SETTING;
        uint32_t csChnbPadSetting = config->csPadSettingOverride ? config->csPadSettingOverride : CS_CHNB_PAD_SETTING;
        uint32_t dataChnbPadSetting =
            config->dataPadSettingOverride ? config->dataPadSettingOverride : DATA_CHNB_PAD_SETTING;

        sclkChnbPadSetting = (sclkChnbPadSetting & ~IOPCTL_PIO_FSEL_MASK) | IOPCTL_PIO_FSEL(6);
        csChnbPadSetting = (csChnbPadSetting & ~IOPCTL_PIO_FSEL_MASK) | IOPCTL_PIO_FSEL(6);
        dataChnbPadSetting = (dataChnbPadSetting & ~IOPCTL_PIO_FSEL_MASK) | IOPCTL_PIO_FSEL(6);

        if (config->sflashB1Size)
        {
            // CS0
            IOPCTL->PIO[2][19] = csChnbPadSetting;
        }

        if (config->sflashB2Size)
        {
            // CS1
            IOPCTL->PIO[2][21] = csChnbPadSetting;
        }

        // CLK0
        IOPCTL->PIO[1][29] = (sclkChnbPadSetting & ((uint32_t)~IOPCTL_PIO_FSEL_MASK)) | IOPCTL_PIO_FSEL(5);
        // D0
        IOPCTL->PIO[1][11] = dataChnbPadSetting;
        // D1
        IOPCTL->PIO[1][12] = dataChnbPadSetting;

        /* Enable Quad Pads by default in case some Flash has the hold pin enabled by default */
        // D2
        IOPCTL->PIO[1][13] = dataChnbPadSetting;
        // D3
        IOPCTL->PIO[1][14] = dataChnbPadSetting;

        if (config->sflashPadType == kSerialFlash_8Pads)
        {
            // D4
            IOPCTL->PIO[2][17] = dataChnbPadSetting;
            // D5
            IOPCTL->PIO[2][18] = dataChnbPadSetting;
            // D6
            IOPCTL->PIO[2][22] = dataChnbPadSetting;
            // D7
            IOPCTL->PIO[2][23] = dataChnbPadSetting;
        }
        // No DQS support, so force it to kFlexSPIReadSampleClk_LoopbackInternally
        config->readSampleClkSrc = kFlexSPIReadSampleClk_LoopbackInternally;
    }
}

#if !BL_FEATURE_UART_AUTOBAUD_IRQ
uint32_t read_autobaud_pin(uint32_t instance)
{
    switch (instance)
    {
        case 0:
            return UART0_RX_GPIO_BASE->B[UART0_RX_GPIO_PIN_GROUP][UART0_RX_GPIO_PIN_NUM];
        default:
            return 0;
    }
}
#endif

void dummy_byte_callback(uint8_t byte)
{
    (void)byte;
}

void debug_init(void)
{
}

#if __ICCARM__

size_t __write(int handle, const unsigned char *buf, size_t size)
{
    return size;
}

#endif // __ICCARM__

void update_available_peripherals()
{

}

uint32_t get_boot_mode(void)
{
    return kBootMode_Isp;
}

void go_fail_through_mode(void)
{

}

// Update Reserved RAM regions
void bootloader_property_soc_update(void)
{
    property_store_t *propertyStore = g_bootloaderContext.propertyInterface->store;

    uint32_t reservedSize = kBootloader_ReservedRAM_Size;

    // Set address range of RAM in property interface
    uint32_t ramIndex = 0;
    const memory_map_entry_t *map = (memory_map_entry_t *)&g_bootloaderContext.memoryMap[0];
    while (map->memoryInterface != NULL)
    {
        if ((map->memoryProperty & kMemoryType_RAM) && (map->memoryProperty & kMemoryIsExecutable))
        {
            assert(ramIndex < kRAMCount);
            propertyStore->reservedRegions[ramIndex].startAddress = map->startAddress;
            propertyStore->reservedRegions[ramIndex].endAddress = map->startAddress + reservedSize - 1;
            ramIndex++;
        }
        ++map;
    }
}

#if BL_FEATURE_PUT_BCA_IN_FUSE
void usb_get_usbid_from_fuse(uint16_t *vid, uint16_t *pid)
{
    uint32_t usbId = 0;

    status_t status = otp_fuse_read(OTP_USB_ID_FUSE_IDX, &usbId);
    if (status != kStatus_Success)
    {
        usbId = 0;
    }

    uint16_t fusePid = (uint16_t)((usbId & OTP_USB_ID_PID_MASK) >> OTP_USB_ID_PID_SHIFT);
    uint16_t fuseVid = (uint16_t)((usbId & OTP_USB_ID_VID_MASK) >> OTP_USB_ID_VID_SHIFT);

    if (fuseVid == 0)
    {
        *vid = 0x1fc9;
        *pid = 0x0020;
    }
    else
    {
        *vid = fuseVid;
        *pid = fusePid;
    }
}
#endif

int memset_s(void *s, size_t smax, int c, size_t n)
{
    if (n > smax)
    {
        return 1;
    }
    memset(s, c, n);
    return 0;
}

bool isp_cleanup_exit(bool *isInfiniteIsp)
{
    CLOCK_EnableClock(kCLOCK_Rtc);
    uint32_t flag = RTC->GPREG[0];
    switch (flag)
    {
        case CLEANUP_SBL_TO_ISP:
            *isInfiniteIsp = true;
            flag = 0x0;
            break;
        case CLEANUP_ISP_TO_SBL:
        default:
            break;
    }
    RTC->GPREG[0] = 0x0;
    return flag;
}

void isp_cleanup_enter(uint32_t flag)
{
    CLOCK_EnableClock(kCLOCK_Rtc);
    RTC->CTRL &= ~RTC_CTRL_SWRESET_MASK;
    RTC->CTRL &= ~RTC_CTRL_RTC_OSC_PD_MASK;
    RTC->GPREG[0] = flag;
    // Reset Flash to normal mode by ROM
    OCOTP->OTP_SHADOW[0x61] = 0x314000;
    __DSB();
    NVIC_SystemReset();
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
