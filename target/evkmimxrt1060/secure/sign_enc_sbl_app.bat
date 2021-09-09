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
:: Set KEK and encrypted region
:: User need to update these parameters!!!!!!!!!!
::***************************************************************************
set region0_key=region0_key=00112233445566778899aabbccddeeff
set region1_key=region1_key=00112233445566778899aabbccddeeff
set initial_region0_arg=region0_arg=1,[0x60001000,0x0000,0]
set demo1_region1_arg=region1_arg=1,[0x60101000,0x2000,0]

::***************************************************************************
:: Configure signing method RSA2048, ECDSAP256 or HAB
:: User need to specify one signing method!!!!!!!!!!
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
elftosb -f imx -V -c .\imx-flexspinor-flashloader-signed.bd  -o   .\ivt_flashloader.bin  .\flashloader.srec
del ivt_flashloader_nopadding.bin

::***************************************************************************
:: Parepre signed SBL
::***************************************************************************
elftosb.exe -f imx -V -c .\imx-flexspinor-normal-signed.bd       -o   .\ivt_sbl.bin  .\sbl.srec

image_enc hw_eng=bee ifile=.\ivt_sbl.bin ofile=.\ivt_sbl_enc.bin base_addr=0x60000000 ^
                %region0_key% %initial_region0_arg% ^
                %region1_key% %demo1_region1_arg% use_zero_key=1 is_boot_image=1

:: Delete encrypted image the first 1K bytes data.
python img_helper.py deleteheader --image .\ivt_sbl_enc.bin --size 1024

:: Generate sbl.sb file
elftosb.exe -f kinetis -V -c .\program_flexspinor_image_qspinor_xip_encrypt.bd -o   .\sbl.sb    .\ivt_sbl_enc.bin


::***************************************************************************
:: Prepare SFW1 image
::***************************************************************************
if %signing_type% == HAB (

    if not exist ".\sfw.srec" (
        echo Can't find file sfw.srec
        pause
    )

    :: Sign image
    elftosb -f imx -V -c .\imx-flexspinor-normal-sfw-signed.bd  -o .\ivt_sfw1.bin .\sfw.srec
    image_enc hw_eng=bee ifile=.\ivt_sfw1.bin ofile=.\sfw_1_enc.bin base_addr=0x60100000 ^
                    %region1_key% %demo1_region1_arg% use_zero_key=1 is_boot_image=0

    :: Add mcuboot header and tlv
    python %imgtool_path%\imgtool.py create --align 4  --version "1.0"  --header-size %mcu_header_size% --pad-header --slot-size 0x100000 --key-info .\ehdr1.bin .\ivt_sfw1_nopadding.bin .\sfw_1_sign.bin
    
    :: Merge mcuboot header and tlvs into encrypted image 
    python img_helper.py merge --header-size %mcu_header_size% --sign-image .\sfw_1_sign.bin --enc-image .\sfw_1_enc.bin

) else (

    if not exist ".\sfw.bin" (
        echo Can't find file sfw.bin
        pause
    )
    
    :: File sfw start at 0x400. Insert 0x400 bytes at the beginning of sfw for tool image_enc.exe
    python img_helper.py paddingimage --pad-size %mcu_header_size% --input .\sfw.bin --output .\sfw_1.bin

    image_enc.exe hw_eng=bee ifile=.\sfw_1.bin  ofile=.\sfw_1_enc.bin  base_addr=0x60100000 ^
                  %region1_key% %demo1_region1_arg% use_zero_key=1 is_boot_image=0
    
    :: Sign plain image with imgtool
    if %signing_type%==RSA2048 (
        python %imgtool_path%\imgtool.py sign --key %imgtool_path%\sign-rsa2048-priv.pem --align 4 --version "1.0" --header-size  %mcu_header_size% --pad-header --slot-size 0x100000 --max-sectors 32 --key-info .\ehdr1.bin  .\sfw.bin  .\sfw_1_rsa.bin
    ) else (
        python %imgtool_path%\imgtool.py sign --key %imgtool_path%\sign-ecdsap256-priv.pem --align 4 --version "1.0" --header-size  %mcu_header_size% --pad-header --slot-size 0x100000 --max-sectors 32 --key-info .\ehdr1.bin  .\sfw.bin  .\sfw_1_rsa.bin
    )
    
    :: Merge mcuboot header and tlvs into encrypted image 
    python img_helper.py merge --header-size %mcu_header_size% --sign-image .\sfw_1_rsa.bin --enc-image .\sfw_1_enc.bin

)

@echo sbl.sb and sfw_1_enc.bin are the final signed and encrypted image, press any key to download them
pause

::***************************************************************************
:: Download signed flashloader
::***************************************************************************
sdphost -u 0x1FC9,0x0135 -- error-status
sdphost -u 0x1FC9,0x0135 -j -- write-file 0x20001000 .\ivt_flashloader.bin
sdphost -u 0x1fc9,0x0135 -- jump-address 0x20001000

sleep 1

::***************************************************************************
:: Download SBL image
::***************************************************************************
blhost  -u 0x15A2,0x0073 -- get-property 1
blhost  -u 0x15A2,0x0073 -- receive-sb-file  .\sbl.sb

::***************************************************************************
:: Download SFW1 image
::***************************************************************************
blhost -t 50000  -u 0x15A2,0x0073 -j -- flash-erase-region 0x60100000 0x80000 9
blhost           -u 0x15A2,0x0073    -- write-memory       0x60100000 .\sfw_1_enc.bin

pause