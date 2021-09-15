::***************************************************************************
:: This file show how to generate signed SBL and application
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

SET com_port=COM24,115200

@echo on

::***************************************************************************
:: Configure signing method RSA2048, ECDSAP256 or ROM_API
::***************************************************************************
set signing_type=ROM_API

set mcu_header_size=0x400

::***************************************************************************
:: Prepare signed SBL image
::***************************************************************************

if not exist ".\sbl.bin" (
    echo Can't find file sbl.bin
    pause
    exit
)

elftosb -V -f rt6xx -J .\signed_sbl_xip.json

::***************************************************************************
:: Prepare signed SFW image
::***************************************************************************
if not exist ".\sfw.bin" (
    echo Can't find file sfw.bin
    pause
)

if %signing_type% == ROM_API (

    elftosb -V -f rt6xx -J signed_sfw_xip.json
    :: Add mcuboot header and tlvs
    python %imgtool_path%\imgtool.py create --align 4  --version "1.0"  --header-size %mcu_header_size% --pad-header --slot-size 0x100000 .\sfw_1_signed.bin .\sfw_1_sign.bin

    del sfw_1_signed.bin
) else (

    if %signing_type%==RSA2048 (
        python %imgtool_path%\imgtool.py sign --key %imgtool_path%\sign-rsa2048-priv.pem --align 4 --version "1.0" --header-size  %mcu_header_size% --pad-header --slot-size 0x100000 --max-sectors 32  .\sfw.bin  .\sfw_1_sign.bin
    ) else (
        python %imgtool_path%\imgtool.py sign --key %imgtool_path%\sign-ecdsap256-priv.pem --align 4 --version "1.0" --header-size  %mcu_header_size% --pad-header --slot-size 0x100000 --max-sectors 32  .\sfw.bin  .\sfw_1_sign.bin
    )
)

@echo sbl_signed.bin and sfw_1_sign.bin are the final signed image, press any key to download them
pause
@echo on

::***************************************************************************
:: Download image
::***************************************************************************
blhost  -p %com_port% -t 15000 -- get-property 1
blhost  -p %com_port% -t 15000 -- fill-memory 0x1c000 4 0xC1503051
blhost  -p %com_port% -t 15000 -- fill-memory 0x1c004 4 0x20000014
blhost  -p %com_port% -t 15000 -- configure-memory 0x9 0x1c000
blhost  -p %com_port% -t 15000 -- flash-erase-region 0x08000000 0x30000

blhost  -p %com_port% -t 15000 -- fill-memory 0x1d000 4 0xf000000f
blhost  -p %com_port% -t 15000 -- configure-memory 0x9 0x1d000

blhost  -p %com_port% -t 15000 -- write-memory 0x08001000 .\sbl_signed.bin

blhost  -p %com_port% -t 15000 -- flash-erase-region 0x08100000 0x50000
blhost  -p %com_port% -t 15000 -- write-memory 0x08100000 .\sfw_1_sign.bin

pause