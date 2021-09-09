using System;
using Microsoft.Win32.SafeHandles;
using System.Runtime.InteropServices;

namespace Test_DFU
{
	sealed internal partial class WinUsbDevice
	{    
        internal enum USB_PIPE_TYPE
        {
            UsbdPipeTypeControl,
            UsbdPipeTypeIsochronous,
            UsbdPipeTypeBulk,
            UsbdPipeTypeInterrupt,
        }
        [StructLayout(LayoutKind.Sequential)]
		internal struct USB_INTERFACE_DESCRIPTOR
		{
			internal Byte bLength;
			internal Byte bDescriptorType;
			internal Byte bInterfaceNumber;
			internal Byte bAlternateSetting;
			internal Byte bNumEndpoints;
			internal Byte bInterfaceClass;
			internal Byte bInterfaceSubClass;
			internal Byte bInterfaceProtocol;
			internal Byte iInterface;
		}
        [StructLayout(LayoutKind.Sequential)]
		internal struct USB_PIPE_INFORMATION
		{
			internal USB_PIPE_TYPE PipeType;
			internal Byte PipeId;
			internal ushort MaximumPacketSize;
			internal Byte Interval;
		}
        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        internal  struct USB_SETUP_PACKET
        {
            public byte RequestType;
            public byte Request;
            public ushort Value;
            public ushort Index;
            public ushort Length;
            public USB_SETUP_PACKET(byte requestType, byte request, ushort value, ushort index, ushort length)
                {
                    this.RequestType = requestType;
                    this.Request = request;
                    this.Value = value;
                    this.Index = index;
                    this.Length = length;
                }
        }

		[DllImport("winusb.dll", SetLastError = true)]
		internal static extern Boolean WinUsb_ControlTransfer(IntPtr InterfaceHandle, USB_SETUP_PACKET SetupPacket, Byte[] Buffer, UInt32 BufferLength, ref UInt32 LengthTransferred, IntPtr Overlapped);

		[DllImport("winusb.dll", SetLastError = true)]
		internal static extern Boolean WinUsb_Free(IntPtr InterfaceHandle);

		[DllImport("winusb.dll", SetLastError = true)]
		internal static extern Boolean WinUsb_Initialize(SafeFileHandle DeviceHandle, ref IntPtr InterfaceHandle);

		[DllImport("winusb.dll", SetLastError = true)]
		internal static extern Boolean WinUsb_QueryDeviceInformation(IntPtr InterfaceHandle, UInt32 InformationType, ref UInt32 BufferLength, ref Byte Buffer);

		[DllImport("winusb.dll", SetLastError = true)]
		internal static extern Boolean WinUsb_QueryInterfaceSettings(IntPtr InterfaceHandle, Byte AlternateInterfaceNumber, ref USB_INTERFACE_DESCRIPTOR UsbAltInterfaceDescriptor);
        [DllImport("winusb.dll", SetLastError = true)]
        internal static extern Boolean WinUsb_ResetPipe(IntPtr InterfaceHandle, ushort PipeID);
        [DllImport("winusb.dll", SetLastError = true)]
        internal static extern Boolean WinUsb_GetDescriptor(
                                                    IntPtr InterfaceHandle,
                                                    UInt32 DescriptorType,
                                                    UInt32 Index,
                                                    UInt32 LanguageID,
                                                    Byte[] Buffer,
                                                    UInt32 BufferLength,
                                                    ref UInt32 LengthTransferred
                                                );


	}
}
