using System;
using System.Runtime.InteropServices;
using System.Windows.Forms;

namespace Test_DFU
{
	
	sealed internal partial class DeviceManagement
	{
		
		internal Boolean DeviceNameMatch(Message m, String mydevicePathName)
		{
			Int32 stringSize;

			try
			{
				DEV_BROADCAST_DEVICEINTERFACE_1 devBroadcastDeviceInterface = new DEV_BROADCAST_DEVICEINTERFACE_1();
				DEV_BROADCAST_HDR devBroadcastHeader = new DEV_BROADCAST_HDR();
				Marshal.PtrToStructure(m.LParam, devBroadcastHeader);

				if ((devBroadcastHeader.dbch_devicetype == DBT_DEVTYP_DEVICEINTERFACE))
				{
					stringSize = System.Convert.ToInt32((devBroadcastHeader.dbch_size - 32) / 2); 
					devBroadcastDeviceInterface.dbcc_name = new Char[stringSize + 1];
					Marshal.PtrToStructure(m.LParam, devBroadcastDeviceInterface);
					String DeviceNameString = new String(devBroadcastDeviceInterface.dbcc_name, 0, stringSize);
					if ((String.Compare(DeviceNameString, mydevicePathName, true) == 0))
					{
						return true;
					}
					else
					{
						return false;
					}
				}
			}
			catch (Exception ex)
			{
                MessageBox.Show(" Error: " + ex.ToString());
			}

			return false;
		}
	
		internal Boolean FindDeviceFromGuid(System.Guid myGuid, ref String devicePathName)
		{
			Int32 bufferSize = 0;
			IntPtr detailDataBuffer = IntPtr.Zero;
			Boolean deviceFound;
			IntPtr deviceInfoSet = new System.IntPtr();
			Boolean lastDevice = false;
			Int32 memberIndex = 0;
			SP_DEVICE_INTERFACE_DATA MyDeviceInterfaceData = new SP_DEVICE_INTERFACE_DATA();
			Boolean success;

			try
			{

				deviceInfoSet = SetupDiGetClassDevs(ref myGuid, IntPtr.Zero, IntPtr.Zero, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
				deviceFound = false;
				memberIndex = 0;
				MyDeviceInterfaceData.cbSize = Marshal.SizeOf(MyDeviceInterfaceData);
				
				do
				{
					success = SetupDiEnumDeviceInterfaces
						(deviceInfoSet,
						IntPtr.Zero,
						ref myGuid,
						memberIndex,
						ref MyDeviceInterfaceData);
					if (!success)
					{
						lastDevice = true;

					}
					else
					{
						success = SetupDiGetDeviceInterfaceDetail
							(deviceInfoSet,
							ref MyDeviceInterfaceData,
							IntPtr.Zero,
							0,
							ref bufferSize,
							IntPtr.Zero);
						detailDataBuffer = Marshal.AllocHGlobal(bufferSize);
						Marshal.WriteInt32(detailDataBuffer, (IntPtr.Size == 4) ? (4 + Marshal.SystemDefaultCharSize) : 8);
						success = SetupDiGetDeviceInterfaceDetail
							(deviceInfoSet,
							ref MyDeviceInterfaceData,
							detailDataBuffer,
							bufferSize,
							ref bufferSize,
							IntPtr.Zero);
						IntPtr pDevicePathName = new IntPtr(detailDataBuffer.ToInt32() + 4);
						devicePathName = Marshal.PtrToStringAuto(pDevicePathName);
						deviceFound = true;
					}
					memberIndex = memberIndex + 1;
				}
				while (!((lastDevice == true)));				

				return deviceFound;
			}
			catch (Exception ex)
			{
                MessageBox.Show(" Error: " + ex.ToString());
                return false;
			}
				finally
			{
				if (detailDataBuffer != IntPtr.Zero)
				{
					Marshal.FreeHGlobal(detailDataBuffer);
				}
				if (deviceInfoSet != IntPtr.Zero)
				{
					SetupDiDestroyDeviceInfoList(deviceInfoSet);
				}
			}

		}			

		internal Boolean RegisterForDeviceNotifications(String devicePathName, IntPtr formHandle, Guid classGuid, ref IntPtr deviceNotificationHandle)
		{

			DEV_BROADCAST_DEVICEINTERFACE devBroadcastDeviceInterface = new DEV_BROADCAST_DEVICEINTERFACE();
			IntPtr devBroadcastDeviceInterfaceBuffer = IntPtr.Zero; 
			Int32 size = 0;

			try
			{

				size = Marshal.SizeOf(devBroadcastDeviceInterface);
				devBroadcastDeviceInterface.dbcc_size = size;
				devBroadcastDeviceInterface.dbcc_devicetype = DBT_DEVTYP_DEVICEINTERFACE;
				devBroadcastDeviceInterface.dbcc_reserved = 0;
				devBroadcastDeviceInterface.dbcc_classguid = classGuid;
				devBroadcastDeviceInterfaceBuffer = Marshal.AllocHGlobal(size);
				Marshal.StructureToPtr(devBroadcastDeviceInterface, devBroadcastDeviceInterfaceBuffer, true);
				deviceNotificationHandle = RegisterDeviceNotification(formHandle, devBroadcastDeviceInterfaceBuffer, DEVICE_NOTIFY_WINDOW_HANDLE);
				Marshal.PtrToStructure(devBroadcastDeviceInterfaceBuffer, devBroadcastDeviceInterface);
				if ((deviceNotificationHandle.ToInt32() == IntPtr.Zero.ToInt32()))
				{
					return false;
				}
				else
				{
					return true;
				}
			}
			catch (Exception ex)
			{
                MessageBox.Show(" Error: " + ex.ToString());
                return false;
			}
			finally
			{
				if (devBroadcastDeviceInterfaceBuffer != IntPtr.Zero)
				{
					Marshal.FreeHGlobal(devBroadcastDeviceInterfaceBuffer);
				}
			}
		}
		internal void StopReceivingDeviceNotifications(IntPtr deviceNotificationHandle)
		{
			try
			{
				DeviceManagement.UnregisterDeviceNotification(deviceNotificationHandle);
			}
			catch (Exception ex)
			{
                MessageBox.Show(" Error: " + ex.ToString());
			}
		}
	}
}
