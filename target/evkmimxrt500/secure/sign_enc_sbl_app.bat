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

SET com_port=COM26,115200

SET user_kek=kek=0102030405060708090a0b0c0d0e0f00
SET initial_otfad_arg=otfad_arg=[00112233445566778899aabbccddeeff,0020406001030507,0x08001000,0x1000],[00112233445566778899aabbccddeeff,0020406001030507,0x08101000,0x5000]
SET sfw1_otfad_arg=otfad_arg=[00112233445566778899aabbccddeeff,0020406001030507,0x08101000,0x5000]

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
)

elftosb -V -f rt5xx -J .\signed_sbl_xip.json

:: File sbl should start at 0x08001000. Insert 0x1000 bytes at the beginning of sbl for tool image_enc.exe
python img_helper.py paddingimage --pad-size 0x1000 --input .\sbl_signed.bin --output .\sbl_padding.bin

image_enc.exe hw_eng=otfad ifile=.\sbl_padding.bin ofile=.\sbl_enc.bin base_addr=0x08000000 %user_kek% %initial_otfad_arg%

:: Extract keyblob
python img_helper.py extract-keycontext --type otfad --enc_image .\sbl_enc.bin --output .\keyblob_initial.bin

:: Delete the first 4K bytes data. We program area KeyBlob and Flash Config Block later
python img_helper.py deleteheader --image .\sbl_enc.bin --size 4096

::***************************************************************************
:: Prepare signed SFW image
::***************************************************************************
if %signing_type% == ROM_API (

    if not exist ".\sfw.bin" (
        echo Can't find file sfw.bin
        pause
    )

    :: Sign image
    elftosb -V -f rt5xx -J .\signed_sfw_xip.json
    
    :: Insert 0x400 bytes at the beginning of sfw for tool image_enc.exe
    python img_helper.py paddingimage --pad-size %mcu_header_size% --input .\sfw_1_signed.bin --output .\sfw_1_padding.bin

    image_enc.exe hw_eng=otfad ifile=.\sfw_1_padding.bin ofile=.\sfw_1_enc.bin base_addr=0x08100000 %user_kek% %sfw1_otfad_arg%
    
    python img_helper.py extract-keycontext --type otfad --enc_image .\sfw_1_enc.bin --output .\sfw_1_keyblob.bin
    
    :: Add mcuboot header and tlvs
    python %imgtool_path%\imgtool.py create --align 4  --version "1.0"  --header-size %mcu_header_size% --pad-header --slot-size 0x100000 --key-info .\sfw_1_keyblob.bin .\sfw_1_signed.bin .\sfw_1_bootheader.bin

    :: Merge mcuboot header and tlvs into encrypted image 
    python img_helper.py merge --header-size %mcu_header_size% --sign-image .\sfw_1_bootheader.bin --enc-image .\sfw_1_enc.bin

) else (

    if not exist ".\sfw.bin" (
        echo Can't find file sfw.bin
        pause
    )

    :: File sfw start at 0x400. Insert 0x400 bytes at the beginning of sfw for tool image_enc.exe
    python img_helper.py paddingimage --pad-size %mcu_header_size% --input .\sfw.bin --output .\sfw_1.bin

    image_enc hw_eng=otfad ifile=.\sfw_1.bin ofile=.\sfw_1_enc.bin base_addr=0x08100000 %user_kek% %sfw1_otfad_arg%

    :: Extract the key context, then insert it into mcuboot header
    python img_helper.py extract-keycontext --type otfad --enc_image .\sfw_1_enc.bin --output .\sfw_1_keyblob.bin

    :: Sign plain image with imgtool
    if %signing_type% == RSA2048 (
        python %imgtool_path%\imgtool.py sign --key %imgtool_path%\sign-rsa2048-priv.pem --align 4 --version "1.0" --header-size %mcu_header_size% --pad-header --slot-size 0x100000 --max-sectors 32 --key-info .\sfw_1_keyblob.bin  .\sfw.bin .\sfw_1_sign.bin
    ) else (
        python %imgtool_path%\imgtool.py sign --key %imgtool_path%\sign-ecdsap256-priv.pem --align 4 --version "1.0" --header-size %mcu_header_size% --pad-header --slot-size 0x100000 --max-sectors 32 --key-info .\sfw_1_keyblob.bin  .\sfw.bin .\sfw_1_sign.bin
    )

    :: Merge mcuboot header and tlvs into encrypted image 
    python img_helper.py merge --header-size %mcu_header_size% --sign-image .\sfw_1_sign.bin --enc-image .\sfw_1_enc.bin
)


@echo sbl_enc.bin and sfw_1_enc.bin are the final image, press any key to download them
pause
@echo on

::***************************************************************************
:: Download image
::***************************************************************************
blhost  -p %com_port% -t 15000 -- get-property 1
blhost  -p %com_port% -t 15000 -- fill-memory 0x1c000 4 0xC0403003
blhost  -p %com_port% -t 15000 -- fill-memory 0x1c004 4 0x20000014
blhost  -p %com_port% -t 15000 -- configure-memory 0x9 0x1c000
blhost  -p %com_port% -t 15000 -- flash-erase-region 0x08000000 0x30000
blhost  -p %com_port% -t 15000 -- fill-memory 0x1d000 4 0xf000000f
blhost  -p %com_port% -t 15000 -- configure-memory 0x9 0x1d000

blhost  -p %com_port% -t 15000 -- write-memory 0x08000000 .\keyblob_initial.bin
blhost  -p %com_port% -t 15000 -- write-memory 0x08001000 .\sbl_enc.bin

blhost  -p %com_port% -t 15000 -- flash-erase-region 0x08100000 0x40000
blhost  -p %com_port% -t 15000 -- write-memory 0x08100000 .\sfw_1_enc.bin

pause