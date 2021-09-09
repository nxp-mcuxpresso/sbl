/*
 * Copyright 2018-2021 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "bootloader_common.h"
#include "bootloader/bl_context.h"
#include "fsl_device_registers.h"
#include "peripherals_pinmux.h"
#include "utilities/fsl_assert.h"
#include "fsl_clock.h"
#include "microseconds/microseconds.h"
//#include "authentication/secure_kboot.h"
//#include "authentication/skboot_debug_auth_hal.h"
//#include "mailbox/hw_dbg_mailbox.h"
#include "fsl_iocon.h"
#include "fsl_clock.h"
#include "fsl_reset.h"
#include "bootloader/bootloader.h"
//#include "flexcomm/fsl_spi.h"

//#include "bm_usb/dfu_timer.h"
//#include "mrt/fsl_mrt.h"
#include "target_config.h"
#include "usb_device_descriptor.h"
#include "composite.h"
#include "usb_ksdk/api_usb_ksdk.h"
//#include "romcp/fsl_romcp.h"
#include "fsl_iap.h"
#include "fsl_iap_ffr.h"
//#include "mbedtls/sha256.h"

//#include "trustzone/tzm.h"
//#include "authentication/authentication_hal.h"
//#include "authentication/fsl_hashcrypt.h"

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

#define BOOT_PIN_DEBOUNCE_READ_COUNT 500

enum
{
    kNxpDefinedBootFreq_12MHz = 0,
    kNxpDefinedBootFreq_24MHz = 1,
    kNxpDefinedBootFreq_48MHz = 2,
    kNxpDefinedBootFreq_96MHz = 3,
};

enum
{
    kBootFailPin_BitOffset = 0x18,
    kBootFailPin_ValueGet = 0xFF,
    kPortBit_ValueGet = 0x07,
    kPinBit_ValueGet = 0x1F,
};

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////
static void usb_dedicated_ram_init(bool *hasDone);
extern status_t usb_fs0_device_clock_enable(void);
extern status_t usb_hs0_device_clock_enable(void);

// Do SW delay
//static void sw_delay_us(uint32_t us);
// Manage GPIO clock
//static void gpio_clock_control(uint32_t index, bool enable);

/*******************************************************************************
 * Extern Variables
 ******************************************************************************/

/*******************************************************************************
 * Local Variables
 ******************************************************************************/
static uint32_t s_pendingHashEnableMask = 0;

/*******************************************************************************
 * Codes
 ******************************************************************************/

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

void go_fatal_mode(void)
{
    while (1);
}

/*
void __stack_chk_fail(void)
{
    go_fatal_mode();
}
*/

bool is_boot_pin_asserted(void)
{
    // Boot pin for Flash only target
    return false;
}

void update_memory_map_lpc_sram(void)
{
    typedef struct
    {
        uint32_t ramxSize : 2;
        uint32_t ram0Size : 2;
        uint32_t ram1Size : 2;
        uint32_t ram2Size : 2;
        uint32_t ram3Size : 2;
        uint32_t ram4Size : 2;
        uint32_t reserved : 20;
    } sram_config_t;

    static bool isSramConfigured = false;

    if (!isSramConfigured)
    {
        /*
         *   Confirmed with Didier, The rule of disabling SRAM always started from RAM4 -> RAM0, so, here we use below
         *   codes to detect the actual RAM size. And The SoC also supported unaligned access crossing RAM bank
         * boundary,
         *   So, we merge the 4 RAM regions into single RAM region
         *
         *   Note: Only RAM0-RAM3 are supported in ROM code
         */
        //sram_config_t sramConfig = *(volatile sram_config_t *)&SYSCON->RAMSIZECFG;
        sram_config_t sramConfig = *(volatile sram_config_t *)&(*(volatile uint32_t *)(SYSCON_BASE + 0xFE8));
        uint32_t ramSizeFlagList[5] = { 0 };

        ramSizeFlagList[0] = sramConfig.ram0Size;
        ramSizeFlagList[1] = sramConfig.ram1Size;
        ramSizeFlagList[2] = sramConfig.ram2Size;
        ramSizeFlagList[3] = sramConfig.ram3Size;
        ramSizeFlagList[4] = sramConfig.ram4Size;

        uint32_t totalSramSize = 0u;
        for (uint32_t i = 0; i < ARRAY_SIZE(ramSizeFlagList); i++)
        {
            uint32_t currentSramBankSize = 0;
            switch (ramSizeFlagList[i])
            {
                case 1u:
                    currentSramBankSize = 32 * 1024u; // 32KB
                    break;
                case 2:
                    currentSramBankSize = 64 * 1024u; // 64KB
                    break;
                case 3:
                    currentSramBankSize = 64 * 1024u; // 64KB
                    break;
                default:
                    currentSramBankSize = 0u; // 0KB
                    break;
            }

            if (i == 4)
            {
                currentSramBankSize /= 4;
            }
            totalSramSize += currentSramBankSize;
        }

        // Update the memory map to match the actual SoC configuration.
        memory_map_entry_t *mapEntry = (memory_map_entry_t *)g_bootloaderContext.memoryMap;
        //uint32_t ramIndex = 0;
        while ((mapEntry != NULL) && mapEntry->memoryInterface != NULL)
        {
            if (mapEntry->memoryProperty & kMemoryType_RAM)
            {
                mapEntry->endAddress = mapEntry->startAddress + totalSramSize - 1;
                debug_printf("memory_map_entry: start=%x, end=%x\n", mapEntry->startAddress, mapEntry->endAddress);
            }
            ++mapEntry;
        }

        // Finish SRAM memory map re-configuration
        isSramConfigured = true;
    }
}

//void sw_delay_us(uint32_t us)
//{
//    uint32_t coreClock = CLOCK_GetFreq(kCLOCK_CoreSysClk);
//    uint32_t ticksPerUs = coreClock / 1000000u / 4 + 1;

//    while (us--)
//    {
//        register uint32_t ticks = ticksPerUs;
//        while (ticks--)
//        {
//            __NOP();
//        }
//    }
//}

//void gpio_clock_control(uint32_t index, bool enable)
//{
//    const clock_ip_name_t k_GpioClocks[] = GPIO_CLOCKS;

//    const SYSCON_RSTn_t k_GpioResets[] = GPIO_RSTS_N;

//    if (index < (sizeof(k_GpioResets) / sizeof(k_GpioResets[0])))
//    {
//        if (enable)
//        {
//            CLOCK_EnableClock(k_GpioClocks[index]);
//            RESET_PeripheralReset(k_GpioResets[index]);
//        }
//        else
//        {
//            CLOCK_DisableClock(k_GpioClocks[index]);
//            RESET_SetPeripheralReset(k_GpioResets[index]);
//        }
//    }

//    __DSB();
//    __ISB();
//}

void init_hardware_api(void)
{
}

void bootloader_flash_init(void)
{
    uint32_t coreClock = CLOCK_GetFreq(kCLOCK_CoreSysClk);
    FLASH_SetProperty(&g_bootloaderContext.allFlashState[0], kFLASH_PropertyPflashSystemFreq, (coreClock / 1000000u));
    FLASH_Init(&g_bootloaderContext.allFlashState[0]);
    FFR_Init(&g_bootloaderContext.allFlashState[kFlashIndex_Main]);
}

void init_hardware(void)
{
    bootloader_flash_init();

    //configure_clocks(kClockOption_EnterBootloader);

    //SYSCON->PERIPHENCFG &= (uint32_t)~s_pendingHashEnableMask;
    *(volatile uint32_t *)(SYSCON_BASE + 0xFEC) &= (uint32_t)~s_pendingHashEnableMask;
    //SYSCON->CONFIGLOCKOUT |= SYSCON_CONFIGLOCKOUT_LOCK_MASK;
    *(volatile uint32_t *)(SYSCON_BASE + 0xFE4) |= 0x1U;

    /* enable clock for IOCON */
    CLOCK_EnableClock(kCLOCK_Iocon);
    /* enable clock for GPIO*/
    CLOCK_EnableClock(kCLOCK_Gpio0);
    CLOCK_EnableClock(kCLOCK_Gpio1);
}

void deinit_hardware(void)
{
    // Note: The Firewalls have been locked during executing init_hardware(), we don't need to lock all of them by this
    //       API call.
    //    FFR_Deinit(&g_bootloaderContext.allFlashState[kFlashIndex_Main]);

    // Double check here in case the lock step is skipped by attacks
    //if ((SYSCON->CONFIGLOCKOUT & SYSCON_CONFIGLOCKOUT_LOCK_MASK) != SYSCON_CONFIGLOCKOUT_LOCK_MASK)
    if (((*(volatile uint32_t *)(SYSCON_BASE + 0xFE4)) & 0x1U) != 0x1U)
    {
        go_fatal_mode();
    }
}

void usb_get_usbid_from_ifr(uint16_t *vid, uint16_t *pid)
{
    volatile uint32_t ffrVal = 0;
    cmpa_cfg_info_t *cmpaCfgInfo = (cmpa_cfg_info_t *)0;
    uint32_t usbidOffset = (uint8_t *)&cmpaCfgInfo->usbId - (uint8_t *)cmpaCfgInfo;
    flash_config_t *flashConfig = &g_bootloaderContext.allFlashState[kFlashIndex_Main];
    status_t status = FFR_GetCustomerData(flashConfig, (uint8_t *)&ffrVal, usbidOffset, 4);

    if ((status != kStatus_Success) || (ffrVal == 0))
    {
        // Default setting, Blank chip
        *vid = kProduct_USB_VID;
        *pid = kProduct_USB_PID;
    }
    else
    {
        *vid = (uint16_t)((ffrVal & FFR_USBID_VENDORID_MASK) >> FFR_USBID_VENDORID_SHIFT);
        *pid = (uint16_t)((ffrVal & FFR_USBID_PRODUCTID_MASK) >> FFR_USBID_PRODUCTID_SHIFT);
    }
}

void usb_update_serial_number_str_desc_according_to_uuid(void)
{
    // ROM shall create 16 byte unicode string from 128-bit UUID of the device
    // as "serial number string descriptor"
    uint8_t uuid[16];
    status_t status = FFR_GetUUID(&g_bootloaderContext.allFlashState[0], uuid);
    if (status == kStatus_Success)
    {
        for (uint32_t i = 0; i < 16; i++)
        {
            g_usb_str_4[2 + 2 * i] = uuid[i];
        }
    }
}

void usb_peripheral_reset(bool *hasDone)
{
    if (!*hasDone)
    {
        RESET_PeripheralReset(kUSB0D_RST_SHIFT_RSTn);
        RESET_PeripheralReset(kUSB0HMR_RST_SHIFT_RSTn);
        RESET_PeripheralReset(kUSB0HSL_RST_SHIFT_RSTn);
        RESET_PeripheralReset(kUSB1H_RST_SHIFT_RSTn);
        RESET_PeripheralReset(kUSB1D_RST_SHIFT_RSTn);
        RESET_PeripheralReset(kUSB1_RST_SHIFT_RSTn);
        RESET_PeripheralReset(kUSB1RAM_RST_SHIFT_RSTn);
    }
}

#if BL_CONFIG_USB_HID
const uint32_t port0_pin22_config = (IOCON_FUNC7 |      /* Pin is configured as USB0_VBUS */
                                     IOCON_MODE_INACT | /* No addition pin function */
                                     IOCON_DIGITAL_EN | /* Enables digital function */
                                     IOCON_INPFILT_OFF  /* Input filter disabled */
                                     );

static inline void IOCON_SetFsUsbVbus(void)
{
    IOCON_PinMuxSet(IOCON, 0, 22, port0_pin22_config);
}
#endif

static void usb_dedicated_ram_init(bool *hasDone)
{
    if (!*hasDone)
    {
        for (int i = 0; i < FSL_FEATURE_USB_USB_RAM; i++)
        {
            ((uint8_t *)FSL_FEATURE_USB_USB_RAM_BASE_ADDRESS)[i] = 0x00U;
        }

        *hasDone = true;
    }
}

bool usb_clock_init(uint32_t controllerId)
{
#if USB_DEVICE_CONFIG_LPCIP3511FS || USB_DEVICE_CONFIG_LPCIP3511HS
    // Ensure FRO96M is enabled
    ANACTRL->FRO192M_CTRL |= ANACTRL_FRO192M_CTRL_ENA_96MHZCLK(1);
    static bool hasDone = false;
    if (controllerId == kUSB_ControllerLpcIp3511Fs0)
    {
        /* reset USBs */
        usb_peripheral_reset(&hasDone);
        // Enable USBFS clock
        CLOCK_AttachClk(kFRO_HF_to_USB0_CLK);

        CLOCK_SetClkDiv(kCLOCK_DivUsb0Clk, 2, true);
#if !defined(BL_TARGET_FPGA)
        // Wait until clock change completes
        while (SYSCON->USB0CLKDIV & SYSCON_USB0CLKDIV_REQFLAG_MASK)
        {
        }
#endif

        PMC->PDRUNCFGCLR0 = PMC_PDRUNCFG0_PDEN_USBFSPHY_MASK;
        /* enable usb0 host clock */
        CLOCK_EnableClock(kCLOCK_UsbRam1);
        CLOCK_EnableClock(kCLOCK_Usbd0);
        CLOCK_EnableClock(kCLOCK_Usbhsl0);
        *((uint32_t *)(USBFSH_BASE + 0x5C)) |= USBFSH_PORTMODE_DEV_ENABLE_MASK;
        /* disable usb0 host clock */
        CLOCK_DisableClock(kCLOCK_Usbhsl0);
        /* enable USB IP clock */
        status_t status = usb_fs0_device_clock_enable();
        if (status != kStatus_Success)
        {
            return false;
        }
        // Set FS Vbus
        IOCON_SetFsUsbVbus();
        usb_dedicated_ram_init(&hasDone);
    }
    else if (controllerId == kUSB_ControllerLpcIp3511Hs0)
    // USB HS clock should be awalys enabled as USB RAM clock is related USB HS only
    {
        /* Power up usb0 phy */
        PMC->PDRUNCFGCLR0 |= PMC_PDRUNCFG0_PDEN_USBHSPHY_MASK;

        usb_peripheral_reset(&hasDone);
        /* enable USB1 IP clock */
        status_t status = usb_hs0_device_clock_enable();
        if (status != kStatus_Success)
        {
            return false;
        }
        /* enable usb1 host clock */
        CLOCK_EnableClock(kCLOCK_Usbh1);
        *((uint32_t *)(USBHSH_BASE + 0x50)) |= USBHSH_PORTMODE_DEV_ENABLE_MASK;
        /* disable usb1 host clock */
        CLOCK_DisableClock(kCLOCK_Usbh1);
        usb_dedicated_ram_init(&hasDone);
    }
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
            return CLOCK_GetFlexCommClkFreq(0U) /
                   (1 + (SYSCON->FLEXFRGXCTRL[0] & 0xff00U) / ((SYSCON->FLEXFRGXCTRL[0] & 0xffU) + 1U));;
        case 1:
            return CLOCK_GetFlexCommClkFreq(1U) /
                   (1 + (SYSCON->FLEXFRGXCTRL[1] & 0xff00U) / ((SYSCON->FLEXFRGXCTRL[1] & 0xffU) + 1U));
        case 2:
            return CLOCK_GetFlexCommClkFreq(2U) /
                   (1 + (SYSCON->FLEXFRGXCTRL[2] & 0xff00U) / ((SYSCON->FLEXFRGXCTRL[2] & 0xffU) + 1U));
        case 3:
            return CLOCK_GetFlexCommClkFreq(3U) /
                   (1 + (SYSCON->FLEXFRGXCTRL[3] & 0xff00U) / ((SYSCON->FLEXFRGXCTRL[3] & 0xffU) + 1U));
        default:
            break;
    }

    return 0;
#else
    return SystemCoreClock;
#endif
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
    uint32_t enabledPeripherals =
                kPeripheralType_UART | kPeripheralType_I2CSlave | kPeripheralType_SPISlave | kPeripheralType_USB_HID;

    bootloader_configuration_data_t *config = &g_bootloaderContext.propertyInterface->store->configurationData;
    config->enabledPeripherals = enabledPeripherals;
}

bool is_flashless_part(void)
{
    uint32_t flashTotalSize = 0;

    status_t status = g_bootloaderContext.flashDriverInterface->flash_get_property(
        g_bootloaderContext.allFlashState, kFLASH_PropertyPflashTotalSize, &flashTotalSize);
    if (status != kStatus_Success)
    {
        return false;
    }

    debug_printf("%s, flashTotalSize=%x\n", __func__, flashTotalSize);

    return flashTotalSize > 0 ? false : true;
}

bool is_isp_pin_asserted(void)
{
    bool isPinAsserted = false;
    uint32_t readCount = 0;
    uint32_t port = 0;
    uint32_t pin = 5;

    // Note: Confirmed by Niobe4 Design Owner, the ISP pin is configured to input mode with pull-up resitors during any
    //       reset, so, ROM no longer need to configure this pin
    //    IOCON->PIO[port][pin] = IOCON_PIO_FUNC(0) | IOCON_PIO_SLEW(1) | IOCON_PIO_DIGIMODE(1) | IOCON_PIO_MODE(2);
    //    GPIO->DIR[port] &= ~(1U << pin);

    // Sample the pin a number of times
    for (uint32_t i = 0; i < BOOT_PIN_DEBOUNCE_READ_COUNT; i++)
    {
        readCount += GPIO->B[port][pin] & 0x1;
    }

    if (readCount < BOOT_PIN_DEBOUNCE_READ_COUNT / 2)
    {
        isPinAsserted = true;
    }

    // BL-6151[Niobe4] Enable the enter ISP mode via the ROM patch
    // Avoid to secure counter error when using the pending error to force entering the isp mode
    // SC_ADD(isPinAsserted);

    return isPinAsserted;
}

void bootloader_property_soc_update(void)
{
    // Manually updated the reserved Region
    property_store_t *propertyStore = g_bootloaderContext.propertyInterface->store;
    // Flash reserved region
    propertyStore->reservedRegions[0].startAddress = 0x0u;
    propertyStore->reservedRegions[0].endAddress = 0x0u;
    // RAM reserved region
    propertyStore->reservedRegions[1].startAddress = 0x14000000u;
    propertyStore->reservedRegions[1].endAddress = 0x14005fffu;
    propertyStore->reservedRegions[2].startAddress = 0x04000000u;
    propertyStore->reservedRegions[2].endAddress = 0x04007fffu;
    propertyStore->reservedRegions[3].startAddress = 0x30000000u;
    propertyStore->reservedRegions[3].endAddress =
        propertyStore->reservedRegions[3].startAddress + kReserved_SRAM_Size - 1;
    propertyStore->reservedRegions[4].startAddress = 0x20000000u;
    propertyStore->reservedRegions[4].endAddress =
        propertyStore->reservedRegions[4].startAddress + kReserved_SRAM_Size - 1;

    debug_printf("List reserved ram region:\n");
    for (uint32_t i = 1; i < 5; i++)
    {
        debug_printf("%d: %x - %x\n", i, propertyStore->reservedRegions[i].startAddress,
                     propertyStore->reservedRegions[i].endAddress);
    }
}

bool has_pending_errors(void)
{
    bool hasErrors = false;

    if (0x0a == ((PMC->AOREG1 & 0xF0000) >> 0x10))
    {
        hasErrors = true;
    }

    debug_printf("%s, hasErrors=%d\n", __func__, hasErrors);

    // Clear the pending errors bits
    PMC->AOREG1 &= ~0xF0000;

    return hasErrors;
}

bool is_redundant_boot_enabled(void)
{
    return false;
}

bool redundant_boot_init(void)
{
    return false;
}

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
    __DSB();
    NVIC_SystemReset();
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
