::***************************************************************************
:: This file show how to burnning ocotp by blhost
:: 
::***************************************************************************

::***************************************************************************
:: Set utility path
:: User need to update the path according MCUX_Provi position!!!!!!!!!!
::***************************************************************************
@echo off
SET "PATH=C:\nxp\MCUX_Provi_v3\bin\tools\elftosb\win;%PATH%"
SET "PATH=C:\nxp\MCUX_Provi_v3\bin\tools\sdphost\win;%PATH%"
SET "PATH=C:\nxp\MCUX_Provi_v3\bin\tools\blhost\win;%PATH%"
SET "PATH=C:\nxp\MCUX_Provi_v3\bin\tools\cst\mingw32\bin;%PATH%"
@echo on

::***************************************************************************
:: Parepre flashloader
::***************************************************************************
elftosb -f imx -V -c .\imx-flexspinor-flashloader-signed.bd  -o   .\ivt_flashloader.bin  .\flashloader.srec
del ivt_flashloader_nopadding.bin

::***************************************************************************
:: Download signed flashloader
::***************************************************************************
sdphost -u 0x1FC9,0x0135    -- error-status
sdphost -u 0x1FC9,0x0135 -j -- write-file   0x20001000 .\ivt_flashloader.bin
sdphost -u 0x1fc9,0x0135    -- jump-address 0x20001000

:: Wait for flashloader ready
sleep 1

::***************************************************************************
:: Program SRK hash value
:: Below is an example SRK hash
::      5d 22 e8 f7 c5 09 46 91 33 00 e0 d3 84 92 3a 29
::      a8 65 97 c5 e3 fd d1 46 46 14 c0 dd ca 0b 8d bb
::***************************************************************************
::blhost -u 0x15A2,0x0073 -j -- efuse-program-once 0x18 f7e8225d
::blhost -u 0x15A2,0x0073 -j -- efuse-program-once 0x19 914609c5
::blhost -u 0x15A2,0x0073 -j -- efuse-program-once 0x1a d3e00033
::blhost -u 0x15A2,0x0073 -j -- efuse-program-once 0x1b 293a9284
::blhost -u 0x15A2,0x0073 -j -- efuse-program-once 0x1c c59765a8
::blhost -u 0x15A2,0x0073 -j -- efuse-program-once 0x1d 46d1fde3
::blhost -u 0x15A2,0x0073 -j -- efuse-program-once 0x1e ddc01446
::blhost -u 0x15A2,0x0073 -j -- efuse-program-once 0x1f bb8d0bca

::***************************************************************************
:: Read SRK hash value back
::***************************************************************************
blhost -u 0x15A2,0x0073 -- efuse-read-once 0x18
blhost -u 0x15A2,0x0073 -- efuse-read-once 0x19
blhost -u 0x15A2,0x0073 -- efuse-read-once 0x1a
blhost -u 0x15A2,0x0073 -- efuse-read-once 0x1b
blhost -u 0x15A2,0x0073 -- efuse-read-once 0x1c
blhost -u 0x15A2,0x0073 -- efuse-read-once 0x1d
blhost -u 0x15A2,0x0073 -- efuse-read-once 0x1e
blhost -u 0x15A2,0x0073 -- efuse-read-once 0x1f

::***************************************************************************
:: Program SW_GP2
:: User may need to update the key value below!!!!!!!!!!
:: i.e. 00112233445566778899aabbccddeeff
::***************************************************************************
::blhost -u 0x15A2,0x0073 -j -- efuse-program-once 0x29 ccddeeff
::blhost -u 0x15A2,0x0073 -j -- efuse-program-once 0x2a 8899aabb
::blhost -u 0x15A2,0x0073 -j -- efuse-program-once 0x2b 44556677
::blhost -u 0x15A2,0x0073 -j -- efuse-program-once 0x2c 00112233

::***************************************************************************
:: Enable SW_GP2 as BEE key
::***************************************************************************
::blhost -u 0x15A2,0x0073 -j -- efuse-program-once 0x06 0000f000

::***************************************************************************
:: Close HAB
::***************************************************************************
::blhost.exe -u 0x15A2,0x0073 -j -- efuse-program-once 0x06 00000002

pause