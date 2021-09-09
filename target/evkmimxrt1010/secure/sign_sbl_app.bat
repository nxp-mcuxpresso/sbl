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
SET "PATH=C:\nxp\MCUX_Provi_v3\bin\tools\sdphost\win;%PATH%"
SET "PATH=C:\nxp\MCUX_Provi_v3\bin\tools\blhost\win;%PATH%"
SET "PATH=C:\nxp\MCUX_Provi_v3\bin\tools\cst\mingw32\bin;%PATH%"
SET imgtool_path=..\..\..\component\secure\mcuboot\scripts
@echo on

::***************************************************************************
:: Configure signing method RSA2048, ECDSAP256 or HAB
::***************************************************************************
set signing_type=HAB

if %signing_type% == HAB (
    set mcu_header_size=0x1000
) else (
    set mcu_header_size=0x400
)

::***************************************************************************
:: Parepre signed flashloader
::***************************************************************************
elftosb -f imx -V -c .\flashloader-signed.bd   -o   .\ivt_flashloader_signed.bin  .\flashloader.srec

::***************************************************************************
:: Parepre signed SBL
::***************************************************************************
elftosb -f imx -V -c .\imx-flexspinor-normal-sbl-signed.bd         -o   .\ivt_sbl.bin  .\sbl.srec
:: Generate sbl.sb file
elftosb -f kinetis -V -c .\program_flexspinor_image_qspi.bd -o   .\sbl.sb    .\ivt_sbl_nopadding.bin

::***************************************************************************
:: Prepare SFW image
::***************************************************************************
if %signing_type% == HAB (

    if not exist ".\sfw.srec" (
        echo Can't find file sfw.srec
        pause
    )

    elftosb -f imx -V -c .\imx-flexspinor-normal-sfw-signed.bd  -o .\ivt_sfw1.bin .\sfw.srec
    python %imgtool_path%\imgtool.py create --align 4 --version "1.0" --header-size %mcu_header_size% --pad-header --slot-size 0x100000 .\ivt_sfw1_nopadding.bin .\sfw_1_sign.bin

) else (

    if not exist ".\sfw.bin" (
        echo Can't find file sfw.bin
        pause
    )

    if %signing_type%==RSA2048 (
        python %imgtool_path%\imgtool.py sign --key %imgtool_path%\sign-rsa2048-priv.pem --align 4 --version "1.0" --header-size  %mcu_header_size% --pad-header --slot-size 0x100000 --max-sectors 32  .\sfw.bin  .\sfw_1_sign.bin
    ) else (
        python %imgtool_path%\imgtool.py sign --key %imgtool_path%\sign-ecdsap256-priv.pem --align 4 --version "1.0" --header-size  %mcu_header_size% --pad-header --slot-size 0x100000 --max-sectors 32  .\sfw.bin  .\sfw_1_sign.bin
    )
)


@echo sbl.sb and sfw_1_sign.bin are the final signed image, press any key to download them
pause


::***************************************************************************
:: Download signed flashloader
::***************************************************************************
sdphost -u 0x1FC9,0x0145 -- error-status
sdphost -u 0x1FC9,0x0145 -j -- write-file   0x20205800 .\ivt_flashloader_signed.bin
sdphost -u 0x1fc9,0x0145 -j -- jump-address 0x20205800

sleep 1

::***************************************************************************
:: Download SBL image
::***************************************************************************
blhost -u 0x15A2,0x0073  -- get-property  1
blhost -u 0x15A2,0x0073  -- receive-sb-file .\sbl.sb

::***************************************************************************
:: Download SFW image
::***************************************************************************
blhost -t 50000  -u 0x15A2,0x0073 -j -- flash-erase-region 0x60100000 0x80000 9
blhost -t 50000  -u 0x15A2,0x0073    -- write-memory       0x60100000 .\sfw_1_sign.bin

pause