# Image Security

## Introduction

Secure Bootloader(SBL) project keeps and extends the MCUBOOT security function. 
It aims to support MCUBOOT-RSA, MCUBOOT-ECDSA, BootROM HAB signing method and 
encrypted boot(XIP) based on hardware.

MCUBOOT-RSA and MCUBOOT-ECDSA are legacy MCUBOOT signing method. It signs the 
image by computing hash over the image, and then signing that hash. Please refer
to MCUBOOT design document for the details.

BootROM Secure Boot and Encrypted Boot(XIP) are security features provided by BootROM 
or Hardware engine.

## BootROM Secure Boot
Secure boot provides guarantee that unauthorized code cannot be executed on a given
product. It involves the device's ROM always executing when coming out of reset. The
ROM will then examine the first user executable image resident in flash memory
to determine the authenticity of that code. If the code is authentic, then control is
transferred to it. This establishes a chain of trusted code from the ROM to the user boot
code.

### High Assurance Boot(HAB)
NXP i.MX RT 4 digits provide the High Assurance Boot (HAB), which is the high-assurance 
boot feature in the system boot ROM, detects and prevents the execution of unauthorized
software (malware) during the boot sequence.

HAB use the asymmetric cryptography to sign the image. The bootable image can be 
signed by CST tool. The tool generates the CSF data in the binary file format that
consists of command sequences and signatures based on given input command sequence
file (csf file).

Here is the HAB signed file format.
```
+--------------------+
| Image Vector Table |
+--------------------+
| Boot Data          |
+--------------------+
| Padding            |
+--------------------+
| Application Image  | <- Image is signed by tool(elftosb, cst)
+--------------------+
| CSF                | <- Command Sequence File(commands + SRK table + signature + certificates)
+--------------------+
```

The OEM use a utility provided by NXP to generate private key and corresponding 
public key pairs. Then the private key is used to encrypt the digest of the image 
which OEM want to release. This encryption generates a unique identifier for the 
image which is called a signature. The certification with public key is also attached
to the image. Before applying the application, the public key is used to decrypt 
the signature. The OEMs burn the digest (hash) of the public key to the eFuses of 
i.MX RT chips. Once burned, it cannot be modified. BootRom can verify public key
by this value. 
For how to sign image using tool elftosb and CST, please check AN12079 or AN12681.

Here is the final file format.
```
+--------------------+
| Image Header       |
+--------------------+
| Padding (Optinal)  |
+--------------------+
| Application Image  | <- Image is signed by tool(cst)
+--------------------+
| TLVs               |
+--------------------+
```

SBL can call HAB api to verify the signed application image. For device which 
doesn't list below, user can search it in related Reference Manual.

| Device         | Silicon revision | HAB API table address |
| -------------- | ---------------- | --------------------- |
| `i.MX RT1170`  |        All       |       0x00211C0C      |
| `i.MX RT1060`  |        All       |       0x00200300      |
| `i.MX RT1050`  |        All       |       0x00200300      |
| `i.MX RT1020`  |        All       |       0x002002C0      |
| `i.MX RT1015`  |        All       |       0x002002C0      |

### LPC55Sxx Secure boot
LPC55Sxx devices support booting of RSA signed images using RSASSA-PKCS1-v1_5 signature verification.
The boot code is signed with RSA private keys. The corresponding RSA public keys used for signature 
verification are contained in the signed image.
LPC55Sxx devices support 2048-bit or 4096-bit RSA keys and X.509 V3 certificates.

Image validation is a two-step process.
1. Validate and extracts the Image public Key from x509 certificate embedded in the image.
2. Uses Image_key (Public) to validate image signature.

The BootROM API skboot_authenticate is used to verify authenticity of an image.
Before running the application with this IAP API, the PFR region(CFPA and CMPA) should be configured.

PFR resides at the end of flash region and can be programmed through ROM in ISP mode.

LPC55Sxx stores configuration for the boot ROM in Protected Flash Region (PFR).

Here is the final file format.
```
+--------------------+
| Image Header       |
+--------------------+
| Padding (Optinal)  |
+--------------------+
| Application Image  | <- Image is signed by tool(elftosb)
+--------------------+
| TLVs               |
+--------------------+
```

Tools blhost, elftosb or elftosb-gui can be used to create signed image and configure PFR. Please 
see AN12283 for detailed step-by-step guide describing use of these tools.

## Encrypted XIP boot
i.MX RT series BootROM supports XIP on the Serial NOR flash device directly with
On-the-fly decryption feature (using AES) powered by BEE/OTFAD controller. 
The PRINCE is used for real-time encrypt/decrypt operation on LPC55Sxx on-chip flash contents.

### Encrypted XIP boot based on BEE
i.MX RT1060/1050/1020 supports XIP with on-the-fly FlexSPI(QSPI)Flash decryption
via Bus Encryption Engine (BEE). The BootROM supports two separate encrypted regions
using two separate AES Keys. One encrypted region can be used for SBL, another can 
be used for application. The image can be encrypted by AES-CTR-128 or AES-ECB-128.

Before doing Encrypted XIP, the BootROM needs to set the BEE controller correctly, the
configurable parameters are organized as Protection Region Descriptor Block (PRDB),
the entire PRDB is encrypted using AES-CBC-128 mode with the AES KEY and IV in a
Key Info Block (KIB). The KIB is encrypted as Encrypted KIB (EKIB) using the AES key
provisioned in eFUSE (SW_GP2) or derived from OTPMK(One-Time Programmable Master Key).
The BootROM decrypts KIB using AES ECB-128 mode, up to 2 EKIBs are supported, 
EKIB0 is located at offset 0x400 and KIB1 is located at offset 0x800.

Image key is AES KEY in key info. In this solution, we use SW_GP2 as KEK which used 
to encrypt the key info.

Tool image_enc.exe can be used to encrypt the image. It is a command-line host program 
that customer can used to verify the encrypted procedure.

### Encrypted XIP boot based on OTFAD
i.MX RT1170/1010 supports XIP with on-the-fly FlexSPI(QSPI)Flash decryption via 
On-the-Fly AES Decryption Module (OTFAD). The OTFAD supports up to 4 separate encrypted 
regions using separate AES keys.

Before booting Encrypted XIP, the BootROM must set the OTFAD module correctly, the configurable 
parameters are organized as KeyBlob. A KeyBlob contains encryption keys for OTFAD, and 
is always encrypted with a KEK. The KEK can be scrambled for each encryption region.
The entire KeyBlob is encrypted using AES-CTR-128 mode. KeyBlob is located at offset 0x0.

The KEK is stored in the OTP/EFUSE block. For RT1170, the KEK can be restored by the PUF, 
using the PUF key store as part of the Encrypted XIP image.

### Encrypted XIP boot based on PRINCE
LPC55Sxx supports on-the-fly encryption/decryption to/from internal flash through PRINCE.
Data stored in on-chip internal Flash could be encrypted in real time. 

LPC55Sxx supports 3 regions that allow multiple code images from independent encryption base to co-exist. 
Each PRINCE region has a secret-key supplied from on-chip SRAM PUF via secret-bus interface
(not SW accessible). PRINCE encryption algorithm does not add latency.

PRINCE keys are 128-bit symmetric key and are sourced from on-chip SRAM PUF via an internal 
hardware interface, without exposing the key on the system bus.

The PUF controller provides secure key storage without storing the key. It is done by
using the digital fingerprint of a device derived from SRAM. Instead of storing the key, a
key code is generated, which in combination with the digital fingerprint is used to
reconstruct PRINCE keys that are routed to the AES engine or for use by software.
These key codes are stored in PFR region of flash.

During the startup, the ROM checks if valid key store data structure is present in PFR. 
If so, the whole key store data structure is loaded into RAM and ROM issues PUF start procedure,
which initializes PUF and reconstruct original keys so that each key can be used if needed.

For key provisioning procedure, please see AN12283 and AN12527.

### Encrypt Image Format

For BootROM HAB signing method, it needs to pad image header to align the
appliation. Other signing methods may have this.

```
+--------------------+
| Image Header       |
+--------------------+
| Padding            |
+--------------------+
| Application Image  | <- Encrytped image and it may be signed
+--------------------+
| TLVs               |
+--------------------+
```
