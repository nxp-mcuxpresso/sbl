using System;
using Microsoft.Win32.SafeHandles;
using System.Runtime.InteropServices;
using System.Windows.Forms;

namespace Test_DFU
{
    sealed internal partial class WinUsbDevice
    {
        /* Requests*/
        public const int RQ_DFU_DETACH = 0;
        public const int RQ_DFU_DNLOAD = 1;
        public const int RQ_DFU_UPLOAD = 2;
        public const int RQ_DFU_GETSTATUS = 3;
        public const int RQ_DFU_CLRSTATUS = 4;
        public const int RQ_DFU_GETSTATE = 5;
        public const int RQ_DFU_ABORT = 6;
        public const int RQ_DFU_RESET = 0xff;

        /* DFU vendor request */
        public const int RQ_DFU_VENDOR_SETSUFFIX = 0;
        public const int RQ_DFU_VENDOR_GETSUFFIX = 1;

        /* Request types*/
        public const int RQT_UPLOAD = 161;
        public const int RQT_DOWNLOAD = 33;
        public const int RQT_VENDOR_UPLOAD = 192;
        public const int RQT_VENDOR_DNLOAD = 64;

        /* define statuses for requests */
        public const int STT_OK = 0;
        public const int STT_ERR_TARGET = 1;
        public const int STT_ERR_FILE = 2;
        public const int STT_ERR_WRITE = 3;
        public const int STT_ERR_ERASE = 4;
        public const int STT_ERR_CHECK_ERASE = 5;
        public const int STT_ERR_PROG = 6;
        public const int STT_ERR_VERIFY = 7;
        public const int STT_ERR_ADDRESS = 8;
        public const int STT_ERR_NOTDONE = 9;
        public const int STT_ERR_FIRMWARE = 10;
        public const int STT_ERR_VENDOR = 11;
        public const int STT_ERR_STTR = 12;
        public const int STT_ERR_POR = 13;
        public const int STT_ERR_UNKNOWN = 14;
        public const int STT_ERR_STALLEDPKT = 15;

        /* define states for requests */
        public const int ST_APP_IDLE = 0;
        public const int ST_APP_DETACH = 1;
        public const int ST_DFU_IDLE = 2;
        public const int ST_DFU_DNLOAD_SYNC = 3;
        public const int ST_DFU_DNBUSY = 4;
        public const int ST_DFU_DNLOAD_IDLE = 5;
        public const int ST_DFU_MANIFEST_SYNC = 6;
        public const int ST_DFU_MANIFEST = 7;
        public const int ST_DFU_MANIFEST_WAIT_RESET = 8;
        public const int ST_DFU_UPLOAD_IDLE = 9;
        public const int ST_DFU_ERROR = 10;

        /* define Results of processes */
        public const int RST_SUCCESSFUL = 0;
        public const int RST_FAIL_FILE_TOO_LARGE = 1;
        public const int RST_FAIL_CHIP_NOT_RESPOND = 2;
        /* define Sizes */
        public const int PACKET_SIZE = 8;
        public const int MAX_FIRMWARE_SIZE = 512;
        /* define Results Messanges of processes */
        public static string[] MessangeResults =
        {
           "   Successful!   ",
           " File too large or no data, please chose an other file!",
           " Chip not Responding!"
        };
        /* define strings describe states */
        public static string[] stringofstate =
        {
            "ST_APP_IDLE",
            "ST_APP_DETACH",
            "ST_DFU_IDLE",
            "ST_DFU_DNLOAD_SYNC",
            "ST_DFU_DNBUSY",
            "ST_DFU_DNLOAD_IDLE",
            "ST_DFU_MANIFEST_SYNC",
            "ST_DFU_MANIFEST",
            "ST_DFU_MANIFEST_WAIT_RESET",
            "ST_DFU_UPLOAD_IDLE",
            "ST_DFU_ERROR",
        };

        internal struct deviceInfo
        {
            internal SafeFileHandle deviceHandle;
            internal IntPtr winUsbHandle;
        }

        internal static deviceInfo DFU_device_info = new deviceInfo();
        /*******************************************************************************/
        /************ Functions are used to exchange data through control end-point  ***/
        /*******************************************************************************/
        public static  bool Getstatus(byte[] status)
        {
            bool a;
            uint uiTransmitted = 0;
            USB_SETUP_PACKET setupPacket= new USB_SETUP_PACKET(RQT_UPLOAD, RQ_DFU_GETSTATUS, 0, 0, 6); /* get status*/
            a = WinUsb_ControlTransfer(DFU_device_info.winUsbHandle, setupPacket, status, 6, ref uiTransmitted, IntPtr.Zero);
            return a;
        }
        public static  bool Abort_Request()
        {
            bool a;
            uint uiTransmitted = 0;
            byte[] buffer = new byte[64 ];
            USB_SETUP_PACKET setupPacket = new USB_SETUP_PACKET(RQT_DOWNLOAD, RQ_DFU_ABORT, 0, 0, 0); /*ABORT*/
            a = WinUsb_ControlTransfer(DFU_device_info.winUsbHandle, setupPacket, buffer, 0, ref uiTransmitted, IntPtr.Zero);
            return a;
        }
        public static bool ClrStatus_Request()
        {
            bool a;
            uint uiTransmitted = 0;
            byte[] buffer = new byte[64];
            USB_SETUP_PACKET setupPacket = new USB_SETUP_PACKET(RQT_DOWNLOAD, RQ_DFU_CLRSTATUS, 0, 0, 0); /*Clear status*/
            a = WinUsb_ControlTransfer(DFU_device_info.winUsbHandle, setupPacket, buffer, 0, ref uiTransmitted, IntPtr.Zero);
            return a;
        }
        public static bool Detach_Request()
        {
            bool a;
            uint uiTransmitted = 0;
            byte[] buffer = new byte[64];
            USB_SETUP_PACKET setupPacket = new USB_SETUP_PACKET(RQT_DOWNLOAD, RQ_DFU_DETACH,1000, 0, 0); /*Detach*/
            a = WinUsb_ControlTransfer(DFU_device_info.winUsbHandle, setupPacket, buffer, 0, ref uiTransmitted, IntPtr.Zero);
            return a;
        }
        public static bool SetSuffix_Request(byte[] dfuSuffix, ushort length)
        {
            bool a;
            uint uiTransmitted = 0;
            USB_SETUP_PACKET setupPacket = new USB_SETUP_PACKET(RQT_VENDOR_DNLOAD, RQ_DFU_VENDOR_SETSUFFIX, 0, 0, length); /*Detach*/
            a = WinUsb_ControlTransfer(DFU_device_info.winUsbHandle, setupPacket, dfuSuffix, length, ref uiTransmitted, IntPtr.Zero);
            return a;
        }
        public static bool GetSuffix_Request(byte[] dfuSuffix, ref uint length_received, ushort length)
        {
            bool a;
            USB_SETUP_PACKET setupPacket = new USB_SETUP_PACKET(RQT_VENDOR_UPLOAD, RQ_DFU_VENDOR_GETSUFFIX, 0, 0, length); /*Detach*/
            a = WinUsb_ControlTransfer(DFU_device_info.winUsbHandle, setupPacket, dfuSuffix, length, ref length_received, IntPtr.Zero);
            return a;
        }
        public static bool Reset_Request()
        {
            bool a;
            uint uiTransmitted = 0;
            byte[] buffer = new byte[64];
            USB_SETUP_PACKET setupPacket = new USB_SETUP_PACKET(RQT_DOWNLOAD, RQ_DFU_RESET, 0, 0, 0); /*Detach*/
            a = WinUsb_ControlTransfer(DFU_device_info.winUsbHandle, setupPacket, buffer, 0, ref uiTransmitted, IntPtr.Zero);
            return a;
        }
        public static  bool Getstate(byte[] state)
        {
            bool a;
            uint uiTransmitted = 0;
            USB_SETUP_PACKET setupPacket= new USB_SETUP_PACKET(RQT_UPLOAD, RQ_DFU_GETSTATE, 0, 0, 1); /* get state*/
            a = WinUsb_ControlTransfer(DFU_device_info.winUsbHandle, setupPacket, state, 1, ref uiTransmitted, IntPtr.Zero);
            return a;
        }
        public static  bool DownLoad_Request(byte[] datatosend, ushort length, ushort blocknum)
        {
            bool a;
            uint uiTransmitted = 0;
            USB_SETUP_PACKET setupPacket= new USB_SETUP_PACKET(RQT_DOWNLOAD, RQ_DFU_DNLOAD, blocknum, 0, length);
            a = WinUsb_ControlTransfer(DFU_device_info.winUsbHandle, setupPacket, datatosend , length , ref uiTransmitted, IntPtr.Zero);
            return a;
        }
        public static bool UpLoad_Request(byte[] dataReceived, ref uint length_received, ushort length, ushort blocknum)
        {
            bool a;
            USB_SETUP_PACKET setupPacket = new USB_SETUP_PACKET(RQT_UPLOAD, RQ_DFU_UPLOAD, blocknum, 0, length); /* request upload data*/
            a = WinUsb_ControlTransfer(DFU_device_info.winUsbHandle, setupPacket, dataReceived, length, ref  length_received, IntPtr.Zero);
            return a;
        }
        public static bool DFU_GetDescriptor(byte[] buffer, UInt32 type, UInt32 bufferLength, ref UInt32 refLength)
        {
            bool error;
            error = WinUsb_GetDescriptor(DFU_device_info.winUsbHandle, type, 2, 0, buffer, bufferLength, ref refLength);
            return error;
        }
        /*******************************************************************************/
        /**************************** Device task functions  ****************************/
        /*******************************************************************************/
        ///  <summary>
        ///  Reset pipe
        ///  </summary>
        ///
        internal void Busreset()
        {
            try
            {
                WinUsb_ResetPipe(DFU_device_info.winUsbHandle, 0);
            }
            catch (Exception ex)
            {

                MessageBox.Show(" Error: " + ex.ToString());
            }
        }
        ///  <summary>
        ///  Release device handler
        ///  </summary>
        ///
        internal void CloseDeviceHandle()
        {
            try
            {
                WinUsb_Free(DFU_device_info.winUsbHandle);

                if (!(DFU_device_info.deviceHandle == null))
                {
                    if (!(DFU_device_info.deviceHandle.IsInvalid))
                    {
                        DFU_device_info.deviceHandle.Close();
                    }
                }
            }
            catch (Exception ex)
            {

                MessageBox.Show(" Error: " + ex.ToString());
            }
        }
        ///  <summary>
        ///  Get Device handler
        ///  </summary>
        internal Boolean GetDeviceHandle(String devicePathName)
        {
            DFU_device_info.deviceHandle = FileIO.CreateFile
                (devicePathName,
                (FileIO.GENERIC_WRITE | FileIO.GENERIC_READ),
                FileIO.FILE_SHARE_READ | FileIO.FILE_SHARE_WRITE,
                IntPtr.Zero,
                FileIO.OPEN_EXISTING,
                FileIO.FILE_ATTRIBUTE_NORMAL | FileIO.FILE_FLAG_OVERLAPPED,
                0);

            if (!(DFU_device_info.deviceHandle.IsInvalid))
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        ///  <summary>
        ///  Initializes a device
        ///  </summary>
        internal Boolean InitializeDevice()
        {
            USB_INTERFACE_DESCRIPTOR ifaceDescriptor;
            USB_PIPE_INFORMATION pipeInfo;
            Boolean success;

            try
            {
                ifaceDescriptor.bLength = 0;
                ifaceDescriptor.bDescriptorType = 0;
                ifaceDescriptor.bInterfaceNumber = 0;
                ifaceDescriptor.bAlternateSetting = 0;
                ifaceDescriptor.bNumEndpoints = 0;
                ifaceDescriptor.bInterfaceClass = 0;
                ifaceDescriptor.bInterfaceSubClass = 0;
                ifaceDescriptor.bInterfaceProtocol = 0;
                ifaceDescriptor.iInterface = 0;
                pipeInfo.PipeType = 0;
                pipeInfo.PipeId = 0;
                pipeInfo.MaximumPacketSize = 0;
                pipeInfo.Interval = 0;

                success = WinUsb_Initialize
                (DFU_device_info.deviceHandle,
                ref DFU_device_info.winUsbHandle);
                if (success)
                {
                    success = WinUsb_QueryInterfaceSettings
                        (DFU_device_info.winUsbHandle,
                        0,
                        ref ifaceDescriptor);
                }
                 return success;
            }
            catch (Exception ex)
            {
                MessageBox.Show(" Error: " + ex.ToString());
                return false;
            }
        }
    }
}
