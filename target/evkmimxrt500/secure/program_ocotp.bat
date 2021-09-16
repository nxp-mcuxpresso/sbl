::***************************************************************************
:: This file show how to burnning ocotp by blhost
:: 
::***************************************************************************

::***************************************************************************
:: Set utility path
:: User need to update the path according MCUX_Provi position!!!!!!!!!!
::***************************************************************************
@echo off
SET "PATH=C:\nxp\MCUX_Provi_v3\bin\tools\blhost\win;%PATH%"
SET com_port=COM22,115200
@echo on

::***************************************************************************
:: Program RKT hash(RKTH) value
:: User need to update the RKTH value below!!!!!!!!!!
:: Below is an example of RKTH
:: RKTH: 8b8123193c27489fe835e104be046187dbc5507c310de41b469e68d5842decc0
::***************************************************************************
::blhost -p %com_port% -t 15000 -- efuse-program-once 0x78 1923818b
::blhost -p %com_port% -t 15000 -- efuse-program-once 0x79 9f48273c
::blhost -p %com_port% -t 15000 -- efuse-program-once 0x7A 04e135e8
::blhost -p %com_port% -t 15000 -- efuse-program-once 0x7B 876104be
::blhost -p %com_port% -t 15000 -- efuse-program-once 0x7C 7c50c5db
::blhost -p %com_port% -t 15000 -- efuse-program-once 0x7D 1be40d31
::blhost -p %com_port% -t 15000 -- efuse-program-once 0x7E d5689e46
::blhost -p %com_port% -t 15000 -- efuse-program-once 0x7F c0ec2d84


::***************************************************************************
:: Enable Secure boot
::***************************************************************************
::blhost -p %com_port% -t 15000 -- efuse-program-once 0x60 900000

::***************************************************************************
:: Read RKTH value back
::***************************************************************************
blhost -p %com_port% -t 15000 -- efuse-read-once 0x78
blhost -p %com_port% -t 15000 -- efuse-read-once 0x79
blhost -p %com_port% -t 15000 -- efuse-read-once 0x7a
blhost -p %com_port% -t 15000 -- efuse-read-once 0x7b
blhost -p %com_port% -t 15000 -- efuse-read-once 0x7c
blhost -p %com_port% -t 15000 -- efuse-read-once 0x7d
blhost -p %com_port% -t 15000 -- efuse-read-once 0x7e
blhost -p %com_port% -t 15000 -- efuse-read-once 0x7f


::!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
:: Below is the configuretion for image decryption
:: It doesn't need to configure these parameters if user don't encrypt image
:: i.e.
:: OTP_MASTER_KEY: 000102030405060708090a0b0c0d0e0f00112233445566778899aabbccddeeff
:: OTFAD KEK seed: 504d1862298daed564f26abf7feb723a
:: OTFAD KEK:      0102030405060708090a0b0c0d0e0f00
::!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

::***************************************************************************
:: Program OTP_MASTER_KEY
:: User need to update the key value below!!!!!!!!!!
:: i.e. 00112233445566778899aabbccddeeff
::***************************************************************************
::blhost -p %com_port% -t 15000 -- efuse-program-once 0x70 ccddeeff
::blhost -p %com_port% -t 15000 -- efuse-program-once 0x71 8899aabb
::blhost -p %com_port% -t 15000 -- efuse-program-once 0x72 44556677
::blhost -p %com_port% -t 15000 -- efuse-program-once 0x73 00112233
::blhost -p %com_port% -t 15000 -- efuse-program-once 0x74 0c0d0e0f
::blhost -p %com_port% -t 15000 -- efuse-program-once 0x75 08090a0b
::blhost -p %com_port% -t 15000 -- efuse-program-once 0x76 04050607
::blhost -p %com_port% -t 15000 -- efuse-program-once 0x77 00010203

::***************************************************************************
:: Configure OTFAD SEED
:: OTFAD KEK seed example: 504d1862298daed564f26abf7feb723a
::***************************************************************************
::blhost -p %com_port% -t 15000 -- efuse-program-once 0x6c 62184d50
::blhost -p %com_port% -t 15000 -- efuse-program-once 0x6d d5ae8d29
::blhost -p %com_port% -t 15000 -- efuse-program-once 0x6e bf6af264
::blhost -p %com_port% -t 15000 -- efuse-program-once 0x6f 3a72eb7f

::***************************************************************************
:: OTFAD_CFG
::***************************************************************************
::blhost -p %com_port% -t 15000 -- efuse-program-once 0x6a 00001000


pause