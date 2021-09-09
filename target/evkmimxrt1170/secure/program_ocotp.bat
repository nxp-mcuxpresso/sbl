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
SET "PATH=C:\nxp\MCUX_Provi_v3\bin\tools\blhost\win;%PATH%"
SET "PATH=C:\nxp\MCUX_Provi_v3\bin\tools\cst\mingw32\bin;%PATH%"
@echo on

::***************************************************************************
:: Parepre flashloader
::***************************************************************************
elftosb -f imx -V -c .\flashloader-signed.bd   -o   .\ivt_flashloader_signed.bin  .\flashloader.srec

::***************************************************************************
:: Download flashloader
::***************************************************************************
blhost -t 5242000  -u 0x1FC9,0x013D -j -- load-image .\ivt_flashloader_signed.bin

:: Wait for flashloader ready
sleep 1

::***************************************************************************
:: Program SRK hash value
:: User need to update the SRK hash value below!!!!!!!!!!
:: Below is an example SRK hash
::      5d 22 e8 f7 c5 09 46 91 33 00 e0 d3 84 92 3a 29
::      a8 65 97 c5 e3 fd d1 46 46 14 c0 dd ca 0b 8d bb
::***************************************************************************
::blhost -t 50000 -u 0x15A2,0x0073 -j -- efuse-program-once 0x30 f7e8225d
::blhost -t 50000 -u 0x15A2,0x0073 -j -- efuse-program-once 0x31 914609c5
::blhost -t 50000 -u 0x15A2,0x0073 -j -- efuse-program-once 0x32 d3e00033
::blhost -t 50000 -u 0x15A2,0x0073 -j -- efuse-program-once 0x33 293a9284
::blhost -t 50000 -u 0x15A2,0x0073 -j -- efuse-program-once 0x34 c59765a8
::blhost -t 50000 -u 0x15A2,0x0073 -j -- efuse-program-once 0x35 46d1fde3
::blhost -t 50000 -u 0x15A2,0x0073 -j -- efuse-program-once 0x36 ddc01446
::blhost -t 50000 -u 0x15A2,0x0073 -j -- efuse-program-once 0x37 bb8d0bca

::***************************************************************************
:: Read SRK hash value back
::***************************************************************************
blhost -u 0x15A2,0x0073 -- efuse-read-once 0x30
blhost -u 0x15A2,0x0073 -- efuse-read-once 0x31
blhost -u 0x15A2,0x0073 -- efuse-read-once 0x32
blhost -u 0x15A2,0x0073 -- efuse-read-once 0x33
blhost -u 0x15A2,0x0073 -- efuse-read-once 0x34
blhost -u 0x15A2,0x0073 -- efuse-read-once 0x35
blhost -u 0x15A2,0x0073 -- efuse-read-once 0x36
blhost -u 0x15A2,0x0073 -- efuse-read-once 0x37

::***************************************************************************
:: Program USER_KEY5
:: User may need to update the key value below!!!!!!!!!!
:: i.e. 00112233445566778899aabbccddeeff
::***************************************************************************
::blhost.exe -t 50000 -u 0x15A2,0x0073 -j -- efuse-program-once 0x80 ffeeddcc
::blhost.exe -t 50000 -u 0x15A2,0x0073 -j -- efuse-program-once 0x81 bbaa9988
::blhost.exe -t 50000 -u 0x15A2,0x0073 -j -- efuse-program-once 0x82 77665544
::blhost.exe -t 50000 -u 0x15A2,0x0073 -j -- efuse-program-once 0x83 33221100

::***************************************************************************
:: Close HAB
::***************************************************************************
::blhost.exe -u 0x15A2,0x0073 -j -- efuse-program-once 0x16 00000002

pause