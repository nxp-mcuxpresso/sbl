# MCU-OTA SBL Release Notes

## --- Version 1.1.0

### What's new
- Add the security function for evkmimxrt500 and evkmimxrt600
- evkmimxrtxxx:
  * Soc sign-verify: ROM RSA(2048,3072,4096)
  * Soc encrypted XIP boot: Yes
- Optimize the revert flow

## --- Version 1.0

### What's new
This is the first release of MCU-OTA SBL, a secure bootloader for NXP
MCU-OTA project. It is designed to work with firmware (MCU-OTA SFW) to
provide a complete secure OTA solution. The SBL can guarantee the OTA
trust chain with NXP Soc secure engine. It can be built conveniently
with GCC toolchain in Linux/Windows, or IAR, MDK in Windows.

### Platforms

- evkmimxrt500
- evkmimxrt600
- evkmimxrt1010
- evkmimxrt1020
- evkbmimxrt1050
- evkmimxrt1060
- evkmimxrt1064
- evkmimxrt1170
- lpc55s69

### Framework

- SCons based on Python environment for both Windows and Linux
- Self-contained Python environment for Windows
- Conveniently create IAR, MDK project by SCons extended commands
- High and easy scalability via Kconfig mechanism for both Windows and Linux

### Toolchain

- Linux: GCC_ARM
- Windows: GCC_ARM, IAR, MDK

### FOTA

- Program image via UART/USB interface
- OTA swap images with rollback support
- Swap status record and recovery
- OTA Soc remap with rollback support
- Various flash devices support

### Security

- All platforms: RSA(2048), ECDSA(P256) sign and verify by software
- evkmimxrt10xx:
  * Soc sign-verify: HAB RSA(2048,3072,4096)
  * Soc encrypted XIP boot: Yes
- evkmimxrt1170:
  * Soc sign-verify: HAB RSA(2048,3072,4096), HAB ECDSA(p-256/384/521)
  * Soc encrypted XIP boot: Yes
- lpc55s69:
  * Soc sign-verify: ROM RSA(2048,4096)
  * Soc encrypted XIP boot: Yes

### Known issues
