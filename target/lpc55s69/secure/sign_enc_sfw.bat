::***************************************************************************
:: This file show how to generate signed SFW application
:: 
::***************************************************************************

::***************************************************************************
:: Set utility path
:: User need to update the path according MCUX_Provi position!!!!!!!!!!
::***************************************************************************
@echo off
SET "PATH=C:\nxp\MCUX_Provi_v3\bin\tools\elftosb\win;%PATH%"
SET "PATH=C:\nxp\MCUX_Provi_v3\bin\tools\blhost\win;%PATH%"
SET imgtool_path=..\..\..\component\secure\mcuboot\scripts
@echo on

::***************************************************************************
:: User need to set correct com port!!!!!!!!!!
::***************************************************************************
set com_port=COM14,115200

::***************************************************************************
:: Configure signing method RSA2048, ECDSAP256 or ROM_RSA
:: User need to specify one signing method!!!!!!!!!!
::***************************************************************************
set signing_type=ROM_RSA

set mcu_header_size=0x400

if not exist ".\sfw.bin" (
    echo Can't find file sfw.bin
    pause
)

::***************************************************************************
:: Sign SFW image
::***************************************************************************
if %signing_type% == ROM_RSA (
    elftosb -V -f lpc55xx -J mbi_config.json
    :: Add mcuboot header and tlvs
    python %imgtool_path%\imgtool.py create --align 4  --version "1.0"  --header-size %mcu_header_size% --pad-header --slot-size 0x30000 .\sfw_signed.bin .\sfw_signed_final.bin
) else (
    :: Sign plain image with imgtool
    if %signing_type%==RSA2048 (
        python %imgtool_path%\imgtool.py sign --key %imgtool_path%\sign-rsa2048-priv.pem --align 4 --version "1.0" --header-size %mcu_header_size% --pad-header --slot-size 0x30000 --max-sectors 32 .\sfw.bin  .\sfw_signed_final.bin
    ) else (
        python %imgtool_path%\imgtool.py sign --key %imgtool_path%\sign-ecdsap256-priv.pem --align 4 --version "1.0" --header-size %mcu_header_size% --pad-header --slot-size 0x30000 --max-sectors 32 .\sfw.bin  .\sfw_signed_final.bin
    )
)

@echo sfw_signed_final.bin is booting image, press any key to download
pause

::***************************************************************************
:: Configure PRINCE region1 to encrypt SFW image in flash
:: For a larger image, user may need to configure PRINCE region2.
::***************************************************************************
blhost -p %com_port% -- fill-memory 0x20034000 4 0x50000001
blhost -p %com_port% -- fill-memory 0x20034004 4 0x20000
blhost -p %com_port% -- fill-memory 0x20034008 4 0x2000
blhost -p %com_port% -- configure-memory       0 0x20034000

::***************************************************************************
:: Download SFW image
::***************************************************************************
blhost -p %com_port% flash-erase-region 0x20000 0x30000  0
blhost -p %com_port% write-memory       0x20000 .\sfw_signed_final.bin

pause
