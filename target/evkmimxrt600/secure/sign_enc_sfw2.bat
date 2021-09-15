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
SET "PATH=C:\nxp\MCUX_Provi_v3\bin\tools\image_enc\win;%PATH%"
SET imgtool_path=..\..\..\component\secure\mcuboot\scripts

SET user_kek=kek=0102030405060708090a0b0c0d0e0f00
SET sfw2_otfad_arg=otfad_arg=[00112233445566778899aabbccddeeff,0020406001030507,0x08101000,0x6000]

@echo on

::***************************************************************************
:: Configure signing method RSA2048, ECDSAP256 or ROM_API
::***************************************************************************
set signing_type=ROM_API

set mcu_header_size=0x400


::***************************************************************************
:: Prepare signed SFW image
::***************************************************************************

if not exist ".\sfw2.bin" (
    echo Can't find file sfw2.bin
    pause
    exit
)

if %signing_type% == ROM_API (

    :: Sign image
    elftosb -V -f rt6xx -J .\signed_sfw2_xip.json

    :: Insert 0x400 bytes at the beginning of sfw for tool image_enc.exe
    python img_helper.py paddingimage --pad-size %mcu_header_size% --input .\sfw_2_signed.bin --output .\sfw_2_padding.bin

    image_enc.exe hw_eng=otfad ifile=.\sfw_2_padding.bin ofile=.\sfw_2_enc.bin base_addr=0x08100000 %user_kek% %sfw2_otfad_arg%
    
    python img_helper.py extract-keycontext --type otfad --enc_image .\sfw_2_enc.bin --output .\sfw_2_keyblob.bin
    
    :: Add mcuboot header and tlvs
    python %imgtool_path%\imgtool.py create --align 4  --version "1.1"  --header-size %mcu_header_size% --pad-header --slot-size 0x100000 --key-info .\sfw_2_keyblob.bin .\sfw_2_signed.bin .\sfw_2_bootheader.bin

    :: Merge mcuboot header and tlvs into encrypted image 
    python img_helper.py merge --header-size %mcu_header_size% --sign-image .\sfw_2_bootheader.bin --enc-image .\sfw_2_enc.bin

) else (

    :: File sfw start at 0x400. Insert 0x400 bytes at the beginning of sfw for tool image_enc.exe
    python img_helper.py paddingimage --pad-size %mcu_header_size% --input .\sfw2.bin --output .\sfw_2.bin

    image_enc hw_eng=otfad ifile=.\sfw_2.bin ofile=.\sfw_2_enc.bin base_addr=0x08100000 %user_kek% %sfw2_otfad_arg%

    :: Extract the key context, then insert it into mcuboot header
    python img_helper.py extract-keycontext --type otfad --enc_image .\sfw_2_enc.bin --output .\sfw_2_keyblob.bin

    :: Sign plain image with imgtool
    if %signing_type% == RSA2048 (
        python %imgtool_path%\imgtool.py sign --key %imgtool_path%\sign-rsa2048-priv.pem --align 4 --version "1.1" --header-size %mcu_header_size% --pad-header --slot-size 0x100000 --max-sectors 32 --key-info .\sfw_2_keyblob.bin  .\sfw2.bin .\sfw_2_sign.bin
    ) else (
        python %imgtool_path%\imgtool.py sign --key %imgtool_path%\sign-ecdsap256-priv.pem --align 4 --version "1.1" --header-size %mcu_header_size% --pad-header --slot-size 0x100000 --max-sectors 32 --key-info .\sfw_2_keyblob.bin  .\sfw2.bin .\sfw_2_sign.bin
    )

    :: Merge mcuboot header and tlvs into encrypted image 
    python img_helper.py merge --header-size %mcu_header_size% --sign-image .\sfw_2_sign.bin --enc-image .\sfw_2_enc.bin
)

@echo sfw_2_enc.bin are the final image, press any key to download them
pause
