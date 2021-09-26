::***************************************************************************
:: This file show how to generate XIP encrypted and signed ota image for application
:: 
::***************************************************************************

::***************************************************************************
:: Set utility path
:: User need to update the path according MCUX_Provi position!!!!!!!!!!
::***************************************************************************
@echo off
SET "PATH=C:\nxp\MCUX_Provi_v3\bin\tools\elftosb\win;%PATH%"
SET "PATH=C:\nxp\MCUX_Provi_v3\bin\tools\cst\mingw32\bin;%PATH%"
SET "PATH=C:\nxp\MCUX_Provi_v3\bin\tools\image_enc\win;%PATH%"
SET imgtool_path=..\..\..\component\secure\mcuboot\scripts
@echo on

::***************************************************************************
:: Set KEK and encrypted region
:: User need to update these parameters!!!!!!!!!!
::***************************************************************************
set region_key=region1_key=00112233445566778899aabbccddeeff
set demo_region1_arg=region1_arg=1,[0x60101000,0x3000,0]

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
:: Prepare SFW2 image
::***************************************************************************
if %signing_type% == HAB (

    if not exist ".\sfw2.srec" (
        echo Can't find file sfw2.srec
        pause
    )
    
    elftosb -f imx -V -c .\imx-flexspinor-normal-sfw-signed.bd  -o .\ivt_sfw2.bin .\sfw2.srec

    image_enc hw_eng=bee ifile=.\ivt_sfw2.bin ofile=.\sfw_2_enc.bin base_addr=0x60100000 ^
                    %region_key% %demo_region1_arg% use_zero_key=1 is_boot_image=0
                    
    :: Delete encrypted image the first 4K bytes data.
    python img_helper.py deleteheader --image .\sfw_2_enc.bin --size 4096

    :: Add mcuboot header
    python %imgtool_path%\imgtool.py create --align 4  --version "1.1"  --header-size %mcu_header_size% --pad-header --slot-size 0x100000 --key-info .\ehdr1.bin .\sfw_2_enc.bin .\sfw_2_signed.bin

    :: Merge mcuboot header and tlvs into encrypted image 
    python img_helper.py merge --header-size %mcu_header_size% --sign-image .\sfw_2_sign.bin --enc-image .\sfw_2_enc.bin

    del sfw_2_enc.bin
    rename "sfw_2_signed.bin" "sfw_2_enc.bin" 

) else (

    if not exist ".\sfw2.bin" (
        echo Can't find file sfw2.bin
        pause
    )
    
    :: File sfw start at 0x400. Insert 0x400 bytes at the beginning of sfw for tool image_enc.exe
    python img_helper.py paddingimage --pad-size %mcu_header_size% --input .\sfw2.bin --output .\sfw_2.bin

    image_enc.exe hw_eng=bee ifile=.\sfw_2.bin  ofile=.\sfw_2_enc.bin  base_addr=0x60100000 ^
                  %region_key% %demo_region1_arg% use_zero_key=1 is_boot_image=0

    :: Delete encrypted image the first 0x400 bytes data.
    python img_helper.py deleteheader --image .\sfw_2_enc.bin --size 1024

    :: Sign plain image with imgtool
    if %signing_type%==RSA2048 (
        python %imgtool_path%\imgtool.py sign --key %imgtool_path%\sign-rsa2048-priv.pem --align 4 --version "1.1" --header-size %mcu_header_size% --pad-header --slot-size 0x100000 --max-sectors 32 --key-info .\ehdr1.bin  .\sfw_2_enc.bin  .\sfw_2_signed.bin
    ) else (
        python %imgtool_path%\imgtool.py sign --key %imgtool_path%\sign-ecdsap256-priv.pem --align 4 --version "1.1" --header-size %mcu_header_size% --pad-header --slot-size 0x100000 --max-sectors 32 --key-info .\ehdr1.bin  .\sfw_2_enc.bin  .\sfw_2_signed.bin
    )
    
    del sfw_2_enc.bin
    rename "sfw_2_signed.bin" "sfw_2_enc.bin" 




)

@echo File sfw_2_enc.bin is the final image
pause