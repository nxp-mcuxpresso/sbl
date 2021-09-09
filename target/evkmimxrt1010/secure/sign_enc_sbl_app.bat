::***************************************************************************
:: This file show how to generate XIP encrypted and signed SBL and application
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
SET "PATH=C:\nxp\MCUX_Provi_v3\bin\tools\image_enc\win;%PATH%"
SET imgtool_path=..\..\..\component\secure\mcuboot\scripts
@echo on

::***************************************************************************
:: Configure signing method as RSA2048, ECDSAP256 or HAB
::***************************************************************************
set signing_type=HAB

if %signing_type% == HAB (
    set mcu_header_size=0x1000
) else (
    set mcu_header_size=0x400
)

::***************************************************************************
:: Set KEK and encrypted region
:: User need to update these parameters!!!!!!!!!!
::***************************************************************************
:: set user_kek=kek=00112233445566778899aabbccddeeff
set user_kek=kek=00000000000000000000000000000000
set sbl_otfad_arg=otfad_arg=[00112233445566778899aabbccddeeff,0020406001030507,0x60001000,0x0000],[00112233445566778899aabbccddeeff,0020406001030507,0x60101000,0x6000]
set sfw1_otfad_arg=otfad_arg=[00112233445566778899aabbccddeeff,0020406001030507,0x60101000,0x6000]


::***************************************************************************
:: Parepre signed flashloader
::***************************************************************************
elftosb -f imx -V -c .\flashloader-signed.bd  -o   .\ivt_flashloader_signed.bin  .\flashloader.srec


::***************************************************************************
:: Parepre signed SBL
::***************************************************************************
if not exist ".\sbl.srec" (
    echo Can't find file sbl.srec
    pause
)

elftosb.exe -f imx -V -c .\imx-flexspinor-normal-sbl-signed.bd  -o .\ivt_sbl.bin .\sbl.srec

image_enc.exe hw_eng=otfad ifile=.\ivt_sbl.bin ofile=.\ivt_sbl_enc.bin   base_addr=0x60000000 %user_kek% %sbl_otfad_arg% scramble=0x00000000 scramble_align=0x00

python img_helper.py extract-keycontext --type otfad --enc_image .\ivt_sbl_enc.bin --output .\keyblob.bin
:: Delete the first 4K bytes data.
python img_helper.py deleteheader --image .\ivt_sbl_enc.bin --size 4096

elftosb.exe -f kinetis -V -c .\program_flexspinor_image_qspi_keyblob.bd  -o .\sbl.sb .\ivt_sbl_enc.bin .\keyblob.bin


::***************************************************************************
:: Prepare SFW1 image
::***************************************************************************
if %signing_type% == HAB (

    if not exist ".\sfw.srec" (
        echo Can't find file sfw.srec
        pause
    )

    :: Sign image
    elftosb.exe -f imx -V -c .\imx-flexspinor-normal-sfw-signed.bd  -o .\ivt_sfw1.bin .\sfw.srec

    image_enc.exe hw_eng=otfad ifile=.\ivt_sfw1.bin ofile=.\sfw_1_enc.bin base_addr=0x60100000 %user_kek% %sfw1_otfad_arg% scramble=0x00000000 scramble_align=0x00
    
    python img_helper.py extract-keycontext --type otfad --enc_image .\sfw_1_enc.bin --output .\sfw_1_keyblob.bin
    
    python %imgtool_path%\imgtool.py create --align 4  --version "1.0"  --header-size %mcu_header_size% --pad-header --slot-size 0x100000 --key-info .\sfw_1_keyblob.bin .\ivt_sfw1_nopadding.bin .\sfw_1_sign.bin

    :: Merge mcuboot header and tlvs into encrypted image 
    python img_helper.py merge --header-size %mcu_header_size% --sign-image .\sfw_1_sign.bin --enc-image .\sfw_1_enc.bin

) else (

    if not exist ".\sfw.bin" (
        echo Can't find file sfw.bin
        pause
    )

    :: File sfw start at 0x400. Insert 0x400 bytes at the beginning of sfw for tool image_enc.exe
    python img_helper.py paddingimage --pad-size %mcu_header_size% --input .\sfw.bin --output .\sfw_1.bin

    image_enc hw_eng=otfad ifile=.\sfw_1.bin ofile=.\sfw_1_enc.bin base_addr=0x60100000 %user_kek% %sfw1_otfad_arg% scramble=0x00000000 scramble_align=0x00
    
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


::***************************************************************************
:: Delete temp images
::***************************************************************************
del /Q .\ivt_sbl_nopadding_enc.bin .\ivt_sbl_nopadding.bin .\ivt_flashloader_nopadding.bin


@echo sbl.sb and sfw_1_enc.bin are the final signed and encrypted image, press any key to download them
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
blhost  -u 0x15A2,0x0073  -- get-property  1
blhost -u 0x15A2,0x0073  -- receive-sb-file .\sbl.sb

::***************************************************************************
:: Download SFW image
::***************************************************************************
blhost -t 2048000  -u 0x15A2,0x0073 -j -- flash-erase-region 0x60100000 0x80000 9
blhost             -u 0x15A2,0x0073    -- write-memory 0x60100000 .\sfw_1_enc.bin


pause

