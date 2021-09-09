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
::set user_kek=kek=00112233445566778899aabbccddeeff
set user_kek=kek=00000000000000000000000000000000
set sfw2_otfad_arg=otfad_arg=[00112233445566778899aabbccddeeff,0020406001030507,0x30101000,0x5000]

::***************************************************************************
:: Configure signing method RSA2048, ECDSAP256 or HAB
:: User need to select correct signing type !!!!!!!!!!
::***************************************************************************
set signing_type=HAB

if %signing_type%==HAB (
    set mcu_header_size=0x1000
) else (
    set mcu_header_size=0x400
)

::***************************************************************************
:: Prepare SFW2 image
::***************************************************************************
if %signing_type%==HAB (

    if not exist ".\sfw2.srec" (
        echo Can't find file sfw2.srec
        pause
    )
    
    elftosb.exe -f imx -V -c .\imx-flexspinor-normal-sfw-signed.bd  -o .\ivt_sfw2.bin .\sfw2.srec

    image_enc.exe hw_eng=otfad ifile=.\ivt_sfw2.bin ofile=.\sfw_2_enc.bin base_addr=0x30100000 ^
            %user_kek% %sfw2_otfad_arg% scramble=0x00000000 scramble_align=0x00
    
    python img_helper.py extract-keycontext --type otfad --enc_image .\sfw_2_enc.bin --output .\sfw_2_keyblob.bin
    
    python %imgtool_path%\imgtool.py create --align 4  --version "1.1"  --header-size %mcu_header_size% --pad-header --slot-size 0x100000 --key-info .\sfw_2_keyblob.bin .\ivt_sfw2_nopadding.bin .\sfw_2_sign.bin

    python img_helper.py merge --header-size %mcu_header_size% --sign-image .\sfw_2_sign.bin --enc-image .\sfw_2_enc.bin

) else (
    if not exist ".\sfw2.bin" (
        echo Can't find file sfw2.bin
        pause
    )

    python img_helper.py paddingimage --pad-size %mcu_header_size% --input .\sfw2.bin --output .\sfw_2.bin

    image_enc hw_eng=otfad ifile=.\sfw_2.bin ofile=.\sfw_2_enc.bin base_addr=0x30100000 %user_kek% %sfw2_otfad_arg% scramble=0x00000000 scramble_align=0x00
    
    :: Extract the keycontext from the start of file sfw_2_enc.bin
    python img_helper.py extract-keycontext --type otfad --enc_image .\sfw_2_enc.bin --output .\sfw_2_keyblob.bin
    
    if %signing_type%==RSA2048 (
        python %imgtool_path%\imgtool.py sign --key %imgtool_path%\sign-rsa2048-priv.pem --align 4 --version "1.1" --header-size %mcu_header_size% --slot-size 0x100000 --max-sectors 32 --key-info .\sfw_2_keyblob.bin  .\sfw_2.bin .\sfw_2_sign.bin
    ) else (
        python %imgtool_path%\imgtool.py sign --key %imgtool_path%\sign-ecdsap256-priv.pem --align 4 --version "1.1" --header-size %mcu_header_size% --slot-size 0x100000 --max-sectors 32 --key-info .\sfw_2_keyblob.bin  .\sfw_2.bin .\sfw_2_sign.bin
    )

    python img_helper.py merge --header-size %mcu_header_size% --sign-image .\sfw_2_sign.bin --enc-image .\sfw_2_enc.bin
)

@echo File sfw_2_enc.bin is the final image
pause