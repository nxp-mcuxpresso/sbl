using System;
using System.IO;
using System.Text;
using System.Windows.Forms;
using System.Threading;

namespace Test_DFU
{
    internal  partial class frmDFU_demo : Form
    {
        /*************************************************************************/
        /************************ Constants  *************************************/
        /*************************************************************************/
        /* Program State */
        private const int ST_IDLE = 0;
        private const int ST_DEVICE_FOUND = 1;
        private const int ST_DEVICE_OPENED = 2;
        private const int ST_FILE_OPENED = 3;
        private const int ST_DOWNLOADING = 4;
        private const int ST_FIRMWARE_UPLOADED = 5;
        /* define index of fields in Status  */
        private const int BSTATUS          =      0;
        private const int BWPOLLTIMEOUT    =      1;
        private const int BSTATE           =      4;
        private const int ISTRING          =      5;
        /* Operation Modes */
        ushort BLOCK_SIZE;
        /*************************************************************************/
        /************************ local variables and objects ********************/
        /*************************************************************************/
        private int programstate = ST_IDLE;
        private ushort  blocknum;                         /* Block Number*/
        private uint sizeofdata;                          /* size of total data*/
        private int bytesTransmitted;
        private string downloadFilePath = "";            /* download file path */
        private string fileuploadpath = "";              /* upload file path */
        private string FirmwareFilePath = "";              /* firmeare file path */
        private uint sizeOfFirmware;                          /* size of firmware */
        private byte[] firmware_down;
        private byte[] firmware_up;
        private byte[] FirmwareFile;
        private UInt32[] dfuCRCTableList;
        /*************************************************************************/
        /*** this GUIDS must be identical with the interface GUID in .inf files **/
        /*************************************************************************/
        private const String DFU_DEVICE_GUID_STRING = "{850F69DF-CA91-42ef-9C8A-275569B5DA89}";
        System.Guid DeviceGuid;
        private IntPtr DeviceNotificationHandle;
        private DeviceManagement DFU_DeviceManagement = new DeviceManagement();
        private String DevicePathName = ""; /* the device path name achieved by win usb */
        private WinUsbDevice DFU_Device = new WinUsbDevice();
        internal frmDFU_demo frmTest_DFU;
        /*************************************************************************/
        /************************* Functions *************************************/
        /*************************************************************************/

        public frmDFU_demo()
        {
            InitializeComponent();
        }
        #region STATIC Members

        /// <summary>
        /// Converts bytes into a hexidecimal string
        /// </summary>
        /// <param name="data">Bytes to converted to a a hex string.</param>
        private static StringBuilder GetHexString(byte[] data, int offset, uint length)
        {
            StringBuilder sb = new StringBuilder((int)length*3);
            for (int i = offset; i < (offset + length); i++)
            {
                sb.Append(data[i].ToString("X2") + " ");
            }
            return sb;
        }
        #endregion
        /****************************************************************************/
        /************************* Device management functions ***********************/
        /****************************************************************************/
        private void createCRCTableList()
        {
            UInt32 polynomial = 0xEDB88320U;
            dfuCRCTableList = new UInt32[256];
            for (UInt32 index = 0U; index < 256U; index++)
            {
                UInt32 crcElement = index;
                for (UInt16 i = 0; i < 8; i++)
                {
                    if ((crcElement & 1) > 0)
                    {
                        crcElement = (crcElement >> 1) ^ polynomial;
                    }
                    else
                    {
                        crcElement = (crcElement >> 1);
                    }
                }
                dfuCRCTableList[index] = crcElement;
            }
        }
        private UInt32 calculateCRC(UInt32 crc, byte[] buffer, UInt32 length)
        {
            UInt32 crcIndex = 0U;
            UInt32 crcReturn = crc;
            for (UInt32 i = 0U; i < length; i++)
            {
                crcIndex = (UInt32)((crcReturn & 0x000000FFU) ^ buffer[i]);
                crcReturn = (dfuCRCTableList[crcIndex] ^ (crcReturn >> 8));
            }
            return (crcReturn);
        }
        /// <summary>
        /// This function Scan device and open it. Return true on success
        /// </summary>
        /// <returns></returns>
        private Boolean deviceManager()
        {
            tsProgramstatus.Text = " Scanning device";
            Boolean error = false;
            Thread.Sleep(1000);
            Application.DoEvents();
            cboDevices.Items.Clear();
            createCRCTableList();
            if (ScanDevice())
            {
                if (openDevice())
                {
                    error = true;
                }
            }
            return error;
        }
        /// <summary>
        /// force device enter dfu mode
        /// </summary>
        private void enterDfuMode()
        {
            cmdenterdfu.Enabled = false;
            tsProgramstatus.Text = " Detaching device";
            WinUsbDevice.Detach_Request();
            Thread.Sleep(1);
            Application.DoEvents();
        }
        private Boolean ScanDevice()
        {
            Boolean deviceFound = false ;
            try
            {
                /*  Get GUID object from GUID string*/
                DeviceGuid =  new System.Guid(DFU_DEVICE_GUID_STRING);
                /* Fill an array with the device path names of all attached devices with matching GUIDs*/
                deviceFound = DFU_DeviceManagement.FindDeviceFromGuid(DeviceGuid,ref DevicePathName);
            }
            catch (Exception ex)
            {
                MessageBox.Show(" Error: " + ex.ToString() );
            }
            return deviceFound;
        }
        private void Startup()
        {
            try
            {
                DFU_Device = new WinUsbDevice();
            }
            catch (Exception ex)
            {
                MessageBox.Show(" Error: " + ex.ToString());
            }
            tsDevice.Text = "None";
            tsDevicestatus.Text = "Unknown";
            deviceManager();
        }
        private void closeDevice()
        {
            if (programstate >= ST_DEVICE_OPENED) /* device has been opened*/
            {
                cboDevices.Items.Clear();
                DFU_Device.CloseDeviceHandle();
                DFU_DeviceManagement.StopReceivingDeviceNotifications(DeviceNotificationHandle);
                panTransfer.Enabled = false;
                tsDevicestatus.Text = "Device Closed";
                programstate = ST_IDLE;
            }
        }
        private bool openDevice()
        {
            bool error = false;
            if (DFU_Device.GetDeviceHandle(DevicePathName))
            {
                /* The device was detected.*/
                DFU_DeviceManagement.RegisterForDeviceNotifications
                                                            (DevicePathName,
                                                            frmTest_DFU.Handle,
                                                            DeviceGuid,
                                                            ref DeviceNotificationHandle);
                if (DFU_Device.InitializeDevice())
                {
                    byte[] bytesToRead = new byte[0xFF];
                    uint bytestransfered = 0;
                    uint dfuProtocol = 0;
                    programstate = ST_DEVICE_OPENED;
                    error = true;
                    tsDevicestatus.Text = "Device opened";
                    /* Get configuration descriptor */
                    WinUsbDevice.DFU_GetDescriptor(bytesToRead, 2, 0xFF, ref bytestransfered);
                    BLOCK_SIZE = 0x40;
                    dfuProtocol = bytesToRead[0x10];
                    if (dfuProtocol == 1 /* DFU device is in runtime mode */)
                    {
                        /* Enable "Enter DFU" mode feature */
                        cmdenterdfu.Enabled = true;
                        /* Disable transfer feature */
                        panTransfer.Enabled = false;
                        /* Display the attached device information */
                        cboDevices.Items.Add("Device firmware upgrade - RUNTIME mode");
                        cboDevices.SelectedIndex = 0;
                        tsDevice.Text = "Device firmware upgrade - RUNTIME mode";
                    }
                    else if  (dfuProtocol == 2 /* DFU device is in DFU mode */)
                    {
                        /* Disable "Enter DFU" mode feature */
                        cmdenterdfu.Enabled = false;
                        /* Enable transfer feature */
                        panTransfer.Enabled = true;
                        cboDevices.Items.Add("Device firmware upgrade - DFU mode");
                        cboDevices.SelectedIndex = 0;
                        tsDevice.Text = "Device firmware upgrade - DFU mode";
                    }
                    else
                    {
                        error = false;
                        programstate = ST_IDLE;
                        tsDevicestatus.Text = "Unknown";
                    }
                    if (error)
                    {
                        /* get DFU function descriptor */
                        uint index = 0;
                        while (index < bytestransfered)
                        {
                            if (0x21 == bytesToRead[index + 1])
                            {
                                BLOCK_SIZE = (ushort)((ushort)(bytesToRead[index + 6] << 8) + (ushort)bytesToRead[index + 5]);
                                break;
                            }
                            else
                            {
                                index += bytesToRead[index];
                            }
                        }
                    }
                }
                else
                {
                    tsDevicestatus.Text = "Unknown";
                }
            }
            else
            {
                tsDevicestatus.Text = "Unknown";
            }
            return error;
        }

        ///  <summary>
        ///  Called when a WM_DEVICECHANGE message has arrived,
        ///  indicating that a device has been attached or removed.
        ///  </summary>
        ///
        ///  <param name="m"> A message with information about the device. </param>

        internal void DeviceChanging(Message m)
        {

            try
            {
                if (m.WParam.ToInt32() == DeviceManagement.DBT_DEVNODES_CHANGED)
                {
                    if ((tsDevicestatus.Text == "Unknown") || (tsDevicestatus.Text == "Device Closed"))
                    {
                        tsDevicestatus.Text = "Device plugged";
                    }
                }
                else if ((m.WParam.ToInt32() == DeviceManagement.DBT_DEVICEREMOVECOMPLETE))
                {
                    /* If WParam contains DBT_DEVICEREMOVAL, a device has been removed.*/
                    /* Find out if it's the device we're communicating with.*/
                    if (DFU_DeviceManagement.DeviceNameMatch(m, DevicePathName))
                    {
                        tsDevice.Text = "None";
                        tsDevicestatus.Text = "Device unplugged";
                        panTransfer.Enabled = false;
                        cmdenterdfu.Enabled = false;
                    }
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show(" Error:" + ex.ToString());
            }
        }

        ///  <summary>
        ///  Overrides WndProc to enable checking for and handling
        ///  WM_DEVICECHANGE messages.
        ///  </summary>
        ///
        ///  <param name="m"> A Windows message.
        ///  </param>
        ///
        protected override void WndProc(ref Message m)
        {
           {
               try
               {
                   /*The OnDeviceChange routine processes WM_DEVICECHANGE messages.*/
                   if (m.Msg == DeviceManagement.WM_DEVICECHANGE)
                   {
                       DeviceChanging(m);
                   }
                   /* Let the base form process the message.*/
                   base.WndProc(ref m);
               }
               catch (Exception ex)
               {
                   MessageBox.Show(" Error:" + ex.ToString());
               }
           }
        }
        /*************************************************************************/
        /************************* Upload and Download functions *****************/
        /*************************************************************************/
        private void downloadtask(byte[] buffer, uint length)
        {
            int results;
            programstate = ST_DOWNLOADING;
            cmdDownload.Enabled = false;
            cmdUpload.Enabled = false;
            butAddSuffix.Enabled = false;
            tsProgramstatus.Text = "downloading...";
            Thread.Sleep(1000);
            Application.DoEvents();
            results = Download_firmware(buffer, length);
            if (results == WinUsbDevice.RST_SUCCESSFUL)
            {
                tsProgramstatus.Text = "Download finished.";
                programstate = ST_DEVICE_OPENED;
                MessageBox.Show("The firmware has been upgraded!");
            }
            else
            {
                tsProgramstatus.Text = "Download error!";
                Thread.Sleep(500);
                Application.DoEvents();
            }
            cmdDownload.Enabled = true;
            cmdUpload.Enabled = true;
            butAddSuffix.Enabled = true;
        }
        ///  <summary>
        ///  this function copy 8 bytes from arr to arr_8.
        ///  </summary>
        ///  <param name="sizeofdata"> size of arr (bytes)
        ///  </param>
        ///  <param name="i"> number of bytes copied
        ///  </param>
        private void PrepareData(uint sizeofdata, int blocknum, byte[] arr_8, byte[] arr)
        {
            int i;
            if ((sizeofdata - blocknum * BLOCK_SIZE) >= BLOCK_SIZE)     /* prepare data to send */
                for (i = 0; i < BLOCK_SIZE; i++)
                {
                    arr_8[i] = arr[blocknum * BLOCK_SIZE + i];
                }
            else
            /* bytes remain to transmit is < 8 stuff 0xff at trail bytes*/
            {
                for (i = 0; i < sizeofdata - blocknum * BLOCK_SIZE; i++)
                {
                    arr_8[i] = arr[blocknum * BLOCK_SIZE + i];
                };
                for (i = (int)(sizeofdata - blocknum * BLOCK_SIZE); i < BLOCK_SIZE; i++)
                {
                    arr_8[i] = 0xff;
                };
            }
        }
        private int Download_firmware(byte[] firmware_buffer, uint firmware_length)
        {
            byte[] bytesToWrite = new byte[BLOCK_SIZE];
            byte[] buffer = new byte[BLOCK_SIZE];
            UInt32 length;
            Int32 time_out;
            UInt32 trying_times = 0; /* number of time the host trying to sent get status request */
            const UInt32 TRYING_TIME = 10000;
            /* Check firmware size */
            /* Firmware is too large */
            if (firmware_length <= 0)
            {
                return (WinUsbDevice.RST_FAIL_FILE_TOO_LARGE);
            }
            else
            {
                /*****************************************************************/
                WinUsbDevice.Getstate(buffer);                      /* get state */
                /*****************************************************************/
                if (buffer[0] == WinUsbDevice.ST_DFU_ERROR) /* if Error state clear it*/
                {
                    WinUsbDevice.ClrStatus_Request();
                    WinUsbDevice.Getstate(buffer);/* DFU IDLE*/
                }
                /*****************************************************************/
                /* Check Device state*/
                if ((buffer[0] != WinUsbDevice.ST_DFU_IDLE) && (buffer[0]!= WinUsbDevice.ST_DFU_DNLOAD_IDLE))
                {
                    return (WinUsbDevice.RST_FAIL_CHIP_NOT_RESPOND);
                }
                else
                {
                    if ((firmware_length < BLOCK_SIZE))
                    {
                        length = firmware_length;
                    }
                    else
                    {
                        length = BLOCK_SIZE;
                    }
                    /* prepare data to send */
                    PrepareData(firmware_length, blocknum, bytesToWrite, firmware_buffer);
                    /* Send first data block*/
                    if (!WinUsbDevice.DownLoad_Request(bytesToWrite, (ushort)length, blocknum))
                    {
                        return (WinUsbDevice.RST_FAIL_CHIP_NOT_RESPOND);
                    }
                    else
                    {
                        /*****************************************************************/
                        WinUsbDevice.Getstate(buffer);                      /* get state (SYNC)*/
                        /*****************************************************************/
                        /*****************************************************************/
                        WinUsbDevice.Getstatus(buffer);                     /* get status (BUSY)*/
                        /*****************************************************************/
                        bytesTransmitted += (int)length;
                        tsProgramstatus.Text = bytesTransmitted + " bytes written.";
                        progressTransfer.Value = bytesTransmitted;
                        blocknum++;
                        /* wait when the device erase the flash */
                        time_out = (buffer[BWPOLLTIMEOUT] + (buffer[BWPOLLTIMEOUT + 1] << 8) + (buffer[BWPOLLTIMEOUT + 2] << 16)) * 4;
                        Thread.Sleep(time_out);
                        Application.DoEvents();
                        while (buffer[4] != WinUsbDevice.ST_DFU_DNLOAD_IDLE)
                        {
                            WinUsbDevice.Getstatus(buffer);                     /* get status*/
                            trying_times++;
                            if (trying_times == TRYING_TIME)
                                break;
                        }
                        trying_times = 0;
                        /*****************************************************************/
                        WinUsbDevice.Getstate(buffer);                      /* get state (DN IDLE)*/
                        /*****************************************************************/
                        if (buffer[0] != WinUsbDevice.ST_DFU_DNLOAD_IDLE)
                        {
                            return (WinUsbDevice.RST_FAIL_CHIP_NOT_RESPOND);
                        }
                        else
                        {
                            /*****************************************************************/
                            /************************** Send Blocks **************************/
                            /*****************************************************************/
                            /*Data remain  >0 bytes*/
                            while ((firmware_length - blocknum * BLOCK_SIZE) > 0)
                            {
                                if ((firmware_length - blocknum * BLOCK_SIZE) < BLOCK_SIZE)
                                {
                                    length = (uint)(firmware_length - blocknum * BLOCK_SIZE);
                                } else {
                                    length = BLOCK_SIZE;
                                }
                                /* prepare data to send */
                                PrepareData(firmware_length, blocknum, bytesToWrite, firmware_buffer);
                                /* Send data */
                                if (!WinUsbDevice.DownLoad_Request(bytesToWrite, (ushort)length, blocknum))
                                {
                                    return (WinUsbDevice.RST_FAIL_CHIP_NOT_RESPOND);
                                }
                                else
                                {
                                    WinUsbDevice.Getstatus(buffer);                     /* get status*/
                                    bytesTransmitted += (int)length;
                                    progressTransfer.Value = bytesTransmitted;
                                    tsProgramstatus.Text = bytesTransmitted + " bytes written.";
                                    blocknum++;
                                    time_out = (buffer[BWPOLLTIMEOUT] + (buffer[BWPOLLTIMEOUT + 1] << 8) + (buffer[BWPOLLTIMEOUT + 2] << 16)) * 2;
                                    Thread.Sleep(time_out);
                                    Application.DoEvents();
                                    /*****************************************************************/
                                    while (buffer[4] != WinUsbDevice.ST_DFU_DNLOAD_IDLE)
                                    {
                                        WinUsbDevice.Getstatus(buffer);                     /* get status*/
                                        trying_times++;
                                        if (trying_times == TRYING_TIME)
                                            break;
                                    }
                                    trying_times = 0;
                                    WinUsbDevice.Getstate(buffer);                      /* get state */
                                    /*****************************************************************/
                                    if (buffer[0] != WinUsbDevice.ST_DFU_DNLOAD_IDLE)
                                    {
                                        return (WinUsbDevice.RST_FAIL_CHIP_NOT_RESPOND);
                                    }
                                }
                            }
                            /****************** send frame with length = 0*********************/
                            WinUsbDevice.DownLoad_Request(buffer, 0, blocknum);
                            WinUsbDevice.Getstate(buffer);        /* get state (Manifest sync)*/
                            /******************** Manifest device ***************************/
                            WinUsbDevice.Getstatus(buffer);                     /* get status*/
                            tsProgramstatus.Text = "Manifesting...";
                            time_out = (buffer[BWPOLLTIMEOUT] + (buffer[BWPOLLTIMEOUT + 1] << 8) + (buffer[BWPOLLTIMEOUT + 2] << 16)) * 2;
                            Thread.Sleep(time_out);
                            Application.DoEvents();
                            /*****************************************************************/
                            WinUsbDevice.Getstatus(buffer);                     /* get status*/
                            time_out = (buffer[BWPOLLTIMEOUT] + (buffer[BWPOLLTIMEOUT + 1] << 8) + (buffer[BWPOLLTIMEOUT + 2] << 16)) * 2;
                            Thread.Sleep(time_out);
                            Application.DoEvents();
                            WinUsbDevice.Getstate(buffer);          /* get state */
                            /*****************************************************************/
                            return WinUsbDevice.RST_SUCCESSFUL;
                        }
                    }
                }
            }
        }
        /// <summary>
        /// This function copies 8 bytes of data array to buffer array
        /// </summary>
        /// <param name="buffer"></param>
        /// <param name="offset"></param>
        /// <param name="count"></param>
        private void coppydata(ref byte[] buffer, byte[] data, uint offset, uint count)
        {
            for (uint i = 0; i < count; i++)
                buffer[offset + i] = data[i];
        }

        private int Upload_firmware()
        {
            firmware_up = new byte[WinUsbDevice.MAX_FIRMWARE_SIZE];
            tsProgramstatus.Text  = "uploading...";
            int results = WinUsbDevice.RST_SUCCESSFUL;
            textUpload.Text = saveFileDialog1.FileName.ToString();
            fileuploadpath = saveFileDialog1.FileName.ToString();
            cmdDownload.Enabled = false;
            cmdUpload.Enabled = false;
            butAddSuffix.Enabled = false;
            textDataReceived.Text = "";
            blocknum = 0;                               /* Block Number*/
            byte[] bytesToRead = new byte[256];
            byte[] readBuffer = new byte[WinUsbDevice.MAX_FIRMWARE_SIZE];
            byte[] buffer = new byte[BLOCK_SIZE];
            byte[] uploadSuffix = new byte[0xFF];
            uint uploadSuffixLength = 0;
            uint uiTransmitted = BLOCK_SIZE;
            uint bytestransfered = 0;
            clearData();
            /* Create upload file */
            FileStream fileStream = File.Open(fileuploadpath, FileMode.Create);
            fileStream.Close();
            //if (GetSuffix_Request(uploadSuffix, uploadSuffixLength, 0xFF))
            {
                /*****************************************************************/
                WinUsbDevice.Getstatus(buffer);                     /* get status*/
                WinUsbDevice.Getstate(buffer);                      /* get state */
                /*****************************************************************/
                if (buffer[0] == WinUsbDevice.ST_DFU_ERROR) /* if Error state clear it*/
                {
                    WinUsbDevice.ClrStatus_Request();
                    WinUsbDevice.Getstate(buffer);
                }
                if (buffer[0] == WinUsbDevice.ST_DFU_IDLE)
                {
                    /*****************************************************************/
                    UInt32 uploadCrcTemp = 0xFFFFFFFF;
                    bool isUploadFinished = true;
                    while ((uiTransmitted == BLOCK_SIZE))/* check short frame*/
                    {
                        WinUsbDevice.UpLoad_Request(bytesToRead, ref uiTransmitted, BLOCK_SIZE, blocknum);
                        bytestransfered += uiTransmitted;
                        tsProgramstatus.Text = bytestransfered + " bytes uploaded.";
                        uploadCrcTemp = calculateCRC(uploadCrcTemp, bytesToRead, uiTransmitted);
                        Writefile(fileuploadpath, bytesToRead, uiTransmitted);
                        displayData(bytesToRead, uiTransmitted);
                        blocknum++;
                        Thread.Sleep(10);
                        Application.DoEvents();
                    }
                    if (WinUsbDevice.GetSuffix_Request(uploadSuffix, ref uploadSuffixLength, 0xFF))
                    {
                        UInt32 upLoadDwCRC;
                        uploadCrcTemp = calculateCRC(uploadCrcTemp, uploadSuffix, uploadSuffixLength - 4);
                        upLoadDwCRC = (UInt32)((UInt32)(uploadSuffix[uploadSuffixLength - 1] << 24) +
                                         (UInt32)(uploadSuffix[uploadSuffixLength - 2] << 16) +
                                         (UInt32)(uploadSuffix[uploadSuffixLength - 3] << 8) +
                                         (UInt32)(uploadSuffix[uploadSuffixLength - 4]));
                        if (upLoadDwCRC != uploadCrcTemp)
                        {
                            isUploadFinished = false;
                        }
                        else
                        {
                            UInt32 dwCRC, blength, dfuSignature, bcdDFU, idVendor, idProduct, bcdDevice;
                            Writefile(fileuploadpath, uploadSuffix, uploadSuffixLength);
                            displayData(uploadSuffix, uploadSuffixLength);
                            dwCRC = (UInt32)((UInt32)(uploadSuffix[uploadSuffixLength - 1] << 24) +
                                    (UInt32)(uploadSuffix[uploadSuffixLength - 2] << 16) +
                                    (UInt32)(uploadSuffix[uploadSuffixLength - 3] << 8) +
                                    (UInt32)(uploadSuffix[uploadSuffixLength - 4]));
                            blength = (UInt32)uploadSuffix[uploadSuffixLength - 5];

                            dfuSignature = (UInt32)((UInt32)(uploadSuffix[uploadSuffixLength - 6] << 16) +
                                    (UInt32)(uploadSuffix[uploadSuffixLength - 7] << 8) +
                                    (UInt32)(uploadSuffix[uploadSuffixLength - 8]));
                            bcdDFU = (UInt32)((UInt32)(uploadSuffix[uploadSuffixLength - 9] << 8) +
                                    (UInt32)(uploadSuffix[uploadSuffixLength - 10]));
                            idVendor = (UInt32)((UInt32)(uploadSuffix[uploadSuffixLength - 11] << 8) +
                                    (UInt32)(uploadSuffix[uploadSuffixLength - 12]));
                            idProduct = (UInt32)((UInt32)(uploadSuffix[uploadSuffixLength - 13] << 8) +
                                    (UInt32)(uploadSuffix[uploadSuffixLength - 14]));
                            bcdDevice = (UInt32)((UInt32)(uploadSuffix[uploadSuffixLength - 15] << 8) +
                                    (UInt32)(uploadSuffix[uploadSuffixLength - 16]));
                            textdwCRC.Text = dwCRC.ToString("X2");
                            textbLength.Text = blength.ToString("X2");
                            textSignature.Text = dfuSignature.ToString("X2");
                            textbcdDFU.Text = bcdDFU.ToString("X2");
                            textidVendor.Text = idVendor.ToString("X2");
                            textidProduct.Text = idProduct.ToString("X2");
                            textbcdDevice1.Text = bcdDevice.ToString("X2");
                        }
                    }
                    if (isUploadFinished)
                    {
                        MessageBox.Show("The firmware has been uploaded", "Upload", MessageBoxButtons.OK);
                    }
                    else
                    {
                        MessageBox.Show("CRC checking error!", "Upload", MessageBoxButtons.OK, MessageBoxIcon.Error);
                    }
                    /* send last request to return device to DFU_IDLE state */
                }
                else
                {
                    MessageBox.Show(" Cannot upload the firmware!", " Upload ", MessageBoxButtons.OK, MessageBoxIcon.Error);
                }
                tsProgramstatus.Text  = "Upload finished.";
                cmdDownload.Enabled = true;
                cmdUpload.Enabled = true;
                butAddSuffix.Enabled = true;
            }
            return results;
        }
        /*************************************************************************/
        /************************* File manipulate functions *********************/
        /*************************************************************************/
        private void clearData()
        {
            textDataReceived.Clear();
        }
        private void displayData(byte[] buffer, uint length)
        {
            textDataReceived.AppendText(GetHexString(buffer, 0, length).ToString());
        }
        /// <summary>
        /// Read a file stream
        /// </summary>
        /// <param name="filePath"></param>
        /// <param name="buffer">data return</param>
        /// <param name="length">length of file</param>
        private void ReadFile(string filePath,ref byte[] buffer, ref uint length)
        {
            FileStream fileStream = new FileStream(filePath, FileMode.Open, FileAccess.Read);
            try
            {
                length =(uint) fileStream.Length;  // get file length
                buffer = new byte[length];            // create buffer
                int count;                            // actual number of bytes read
                int sum = 0;                          // total number of bytes read

                // read until Read method returns 0 (end of the stream has been reached)
                while ((count = fileStream.Read(buffer, sum, (int)(length - sum))) > 0)
                    sum += count;  // sum is a buffer offset for next reading
            }
            finally
            {
                fileStream.Close();
            }
        }
        /// <summary>
        /// write data in buffer to a file stream
        /// </summary>
        /// <param name="filePath"></param>
        /// <param name="buffer">data to write </param>
        /// <param name="length">length of data </param>
        private void Writefile(string filePath, byte[] buffer,uint length)
        {
            FileStream fileStream = File.Open(filePath, FileMode.Append);
            try
            {
                fileStream.Write(buffer, 0, (int)length);
            }
            finally
            {
                fileStream.Close();
            }
        }

        /*************************************************************************/
        /*************************  event handler functions **********************/
        /*************************************************************************/
        private void cmdUpload_Click(object sender, EventArgs e)
        {
            int results;
            saveFileDialog1.OverwritePrompt = true;
            if (saveFileDialog1.ShowDialog() == DialogResult.OK)
            {
                results = Upload_firmware();
            }
            else
            {
                MessageBox.Show("Please select an exist file or create a new file to save the uploaded data", "Upload error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void cmdDownload_Click(object sender, EventArgs e)
        {
            if (downloadFilePath == "")
            {
                MessageBox.Show("Please select firmware file to download!", "DownLoad", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
            else
            {
                /* read firmware file again */
                if (File.Exists(downloadFilePath))
                {
                    blocknum = 0;
                    bytesTransmitted = 0;
                    ReadFile(downloadFilePath, ref firmware_down, ref sizeofdata);
                    if (sizeofdata > 16)
                    {
                        if ((firmware_down[sizeofdata - 6] == 0x44) && (firmware_down[sizeofdata - 7] == 0x46) && (firmware_down[sizeofdata - 8] == 0x55))
                        {
                            /* get suffix length */
                            UInt32 dwCRC, blength, dfuSignature, bcdDFU, idVendor, idProduct, bcdDevice;
                            byte suffixLength = firmware_down[sizeofdata - 5];
                            byte[] suffixTemp = new byte[suffixLength];
                            for (uint i = 0; i < suffixLength; i++)
                            {
                                suffixTemp[i] = firmware_down[sizeofdata - suffixLength + i];
                            }
                            dwCRC = (UInt32)((UInt32)(suffixTemp[suffixLength - 1] << 24) +
                                    (UInt32)(suffixTemp[suffixLength - 2] << 16) +
                                    (UInt32)(suffixTemp[suffixLength - 3] << 8) +
                                    (UInt32)(suffixTemp[suffixLength - 4]));
                            blength = (UInt32)suffixTemp[suffixLength - 5];

                            dfuSignature = (UInt32)((UInt32)(suffixTemp[suffixLength - 6] << 16) +
                                    (UInt32)(suffixTemp[suffixLength - 7] << 8) +
                                    (UInt32)(suffixTemp[suffixLength - 8]));
                            bcdDFU = (UInt32)((UInt32)(suffixTemp[suffixLength - 9] << 8) +
                                    (UInt32)(suffixTemp[suffixLength - 10]));
                            idVendor = (UInt32)((UInt32)(suffixTemp[suffixLength - 11] << 8) +
                                    (UInt32)(suffixTemp[suffixLength - 12]));
                            idProduct = (UInt32)((UInt32)(suffixTemp[suffixLength - 13] << 8) +
                                    (UInt32)(suffixTemp[suffixLength - 14]));
                            bcdDevice = (UInt32)((UInt32)(suffixTemp[suffixLength - 15] << 8) +
                                    (UInt32)(suffixTemp[suffixLength - 16]));
                            textdwCRC.Text = dwCRC.ToString("X2");
                            textbLength.Text = blength.ToString("X2");
                            textSignature.Text = dfuSignature.ToString("X2");
                            textbcdDFU.Text = bcdDFU.ToString("X2");
                            textidVendor.Text = idVendor.ToString("X2");
                            textidProduct.Text = idProduct.ToString("X2");
                            textbcdDevice1.Text = bcdDevice.ToString("X2");
                            WinUsbDevice.SetSuffix_Request(suffixTemp, suffixLength);
                            Thread.Sleep(10);
                            downloadtask(firmware_down, sizeofdata - suffixLength);
                        }
                        else
                        {
                            MessageBox.Show("The PC tool does not support to download the firmware file which does not contain DFU suffix information", "DownLoad", MessageBoxButtons.OK, MessageBoxIcon.Error);
                        }
                    }
                    else
                    {
                        MessageBox.Show("The PC tool does not support to download the firmware file which does not contain DFU suffix information", "DownLoad", MessageBoxButtons.OK, MessageBoxIcon.Error);
                    }
                }
                else
                {
                    MessageBox.Show("The firmware file does not exist!", "DownLoad", MessageBoxButtons.OK, MessageBoxIcon.Error);
                }
            }
        }

        private void exitToolStripMenuItem_Click(object sender, EventArgs e)
        {
            closeDevice();
            Application.Exit();
        }
        private void frmtestDFU_Load(object sender, EventArgs e)
        {
            try
            {
                frmTest_DFU = this;
                Startup();
            }
            catch (Exception ex)
            {
                MessageBox.Show(" Error: " + ex.ToString());
            }
        }
        /// <summary>
        /// Function process the change of program status
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        void tsStatus_TextChanged(object sender, System.EventArgs e)
        {
            if (tsDevicestatus.Text == "Device unplugged")
            {
                closeDevice();
            }
            if (tsDevicestatus.Text == "Device plugged")
            {
                deviceManager();
            }
        }

        private void enterdfu_Click(object sender, EventArgs e)
        {
            enterDfuMode();
        }

        private void cboDevices_SelectedIndexChanged(object sender, EventArgs e)
        {

        }

        private void menuStrip1_ItemClicked(object sender, ToolStripItemClickedEventArgs e)
        {

        }

        private void grpDatareceived_Enter(object sender, EventArgs e)
        {

        }

        private void butFileSelect_Click(object sender, EventArgs e)
        {
            if (openFileDialog1.ShowDialog() == DialogResult.OK)
            {
                if (openFileDialog1.FileName != "")
                {
                    downloadFilePath = openFileDialog1.FileName;
                    textDownLoad.Text = downloadFilePath;
                    ReadFile(downloadFilePath, ref firmware_down, ref sizeofdata);
                    if (sizeofdata > 16)
                    {
                        if ((firmware_down[sizeofdata - 6] == 0x44) && (firmware_down[sizeofdata - 7] == 0x46) && (firmware_down[sizeofdata - 8] == 0x55))
                        {
                            /* get suffix length */
                            UInt32 dwCRC, blength, dfuSignature, bcdDFU, idVendor, idProduct, bcdDevice;
                            byte suffixLength = firmware_down[sizeofdata - 5];
                            byte[] suffixTemp = new byte[suffixLength];
                            for (uint i = 0; i < suffixLength; i++)
                            {
                                suffixTemp[i] = firmware_down[sizeofdata - suffixLength + i];
                            }
                            dwCRC = (UInt32)((UInt32)(suffixTemp[suffixLength - 1] << 24) +
                                    (UInt32)(suffixTemp[suffixLength - 2] << 16) +
                                    (UInt32)(suffixTemp[suffixLength - 3] << 8) +
                                    (UInt32)(suffixTemp[suffixLength - 4]));
                            blength = (UInt32)suffixTemp[suffixLength - 5];

                            dfuSignature = (UInt32)((UInt32)(suffixTemp[suffixLength - 6] << 16) +
                                    (UInt32)(suffixTemp[suffixLength - 7] << 8) +
                                    (UInt32)(suffixTemp[suffixLength - 8]));
                            bcdDFU = (UInt32)((UInt32)(suffixTemp[suffixLength - 9] << 8) +
                                    (UInt32)(suffixTemp[suffixLength - 10]));
                            idVendor = (UInt32)((UInt32)(suffixTemp[suffixLength - 11] << 8) +
                                    (UInt32)(suffixTemp[suffixLength - 12]));
                            idProduct = (UInt32)((UInt32)(suffixTemp[suffixLength - 13] << 8) +
                                    (UInt32)(suffixTemp[suffixLength - 14]));
                            bcdDevice = (UInt32)((UInt32)(suffixTemp[suffixLength - 15] << 8) +
                                    (UInt32)(suffixTemp[suffixLength - 16]));
                            textdwCRC.Text = dwCRC.ToString("X2");
                            textbLength.Text = blength.ToString("X2");
                            textSignature.Text = dfuSignature.ToString("X2");
                            textbcdDFU.Text = bcdDFU.ToString("X2");
                            textidVendor.Text = idVendor.ToString("X2");
                            textidProduct.Text = idProduct.ToString("X2");
                            textbcdDevice1.Text = bcdDevice.ToString("X2");
                            clearData();
                            displayData(firmware_down, sizeofdata);
                            progressTransfer.Minimum = 0;
                            progressTransfer.Maximum = (int)sizeofdata;
                            progressTransfer.Value = 0;
                            programstate = ST_FILE_OPENED;
                        }
                        else
                        {
                            MessageBox.Show("The PC tool does not support to download the firmware file which does not contain DFU suffix information", "DownLoad", MessageBoxButtons.OK, MessageBoxIcon.Error);
                        }
                    }
                    else
                    {
                        MessageBox.Show("The PC tool does not support to download the firmware file which does not contain DFU suffix information", "DownLoad", MessageBoxButtons.OK, MessageBoxIcon.Error);
                    }
                }
                else
                {
                    MessageBox.Show("Please select firmware file to download!", "DownLoad", MessageBoxButtons.OK, MessageBoxIcon.Error);
                }
            }
            else
            {
                MessageBox.Show("Please select firmware file to download!", "DownLoad", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void textVendorID_TextChanged(object sender, EventArgs e)
        {
            string Item = textVendorID.Text;
            int n = 0;
            if (!int.TryParse(Item, System.Globalization.NumberStyles.HexNumber, System.Globalization.NumberFormatInfo.CurrentInfo, out n) &&
              Item != String.Empty)
            {
                textVendorID.Text = Item.Remove(Item.Length - 1, 1);
                MessageBox.Show("The VendorID should be a Hex number", "Add Suffix", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void textProductID_TextChanged(object sender, EventArgs e)
        {
            string Item = textProductID.Text;
            int n = 0;
            if (!int.TryParse(Item, System.Globalization.NumberStyles.HexNumber, System.Globalization.NumberFormatInfo.CurrentInfo, out n) &&
              Item != String.Empty)
            {
                textProductID.Text = Item.Remove(Item.Length - 1, 1);
                MessageBox.Show("The ProductID should be a Hex number", "Add Suffix", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void textbcdDevice_TextChanged(object sender, EventArgs e)
        {
            string Item = textbcdDevice.Text;
            int n = 0;
            if (!int.TryParse(Item, System.Globalization.NumberStyles.HexNumber, System.Globalization.NumberFormatInfo.CurrentInfo, out n) &&
              Item != String.Empty)
            {
                textbcdDevice.Text = Item.Remove(Item.Length - 1, 1);
                MessageBox.Show("The release number of device should be a Hex number", "Add Suffix", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void butFileToAddSuffixPath_Click(object sender, EventArgs e)
        {
            if (openFileDialog1.ShowDialog() == DialogResult.OK)
            {
                if (openFileDialog1.FileName != "")
                {
                    FirmwareFilePath = openFileDialog1.FileName;
                    textFirmwareFilePath.Text = FirmwareFilePath;
                }
                else
                {
                    MessageBox.Show("Please select firmware file to add suffix!", "Add Suffix", MessageBoxButtons.OK, MessageBoxIcon.Error);
                }
            }
            else
            {
                MessageBox.Show("Please select firmware file to add suffix!", "Add Suffix", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void butAddSuffix_Click(object sender, EventArgs e)
        {
            if (FirmwareFilePath != "")
            {
                if (File.Exists(FirmwareFilePath))
                {
                    ReadFile(FirmwareFilePath, ref FirmwareFile, ref sizeOfFirmware);
                    bool isAdded = false;
                    if (sizeOfFirmware > 16)
                    {
                        if ((FirmwareFile[sizeOfFirmware - 6] == 0x44) && (FirmwareFile[sizeOfFirmware - 7] == 0x46) && (FirmwareFile[sizeOfFirmware - 8] == 0x55))
                        {
                            clearData();
                            displayData(FirmwareFile, sizeOfFirmware);
                            MessageBox.Show("This input file has already added suffix", "Add Suffix", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                            isAdded = true;
                        }
                    }
                    if (false == isAdded)
                    {
                        UInt32 crcTemp = 0xFFFFFFFF;
                        byte[] dfuSuffix = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x01, 0x55, 0x46, 0x44, 0x10, 0x00, 0x00, 0x00, 0x00 };
                        /* Update release version */
                        if (textbcdDevice.Text == "")
                        {
                            textbcdDevice.Text = "FFFF";
                        }
                        else
                        {
                            ushort dcdDevice = ushort.Parse(textbcdDevice.Text, System.Globalization.NumberStyles.HexNumber);
                            dfuSuffix[0] = (byte)(dcdDevice & 0x00FF);
                            dfuSuffix[1] = (byte)((dcdDevice >> 8) & 0xFF);
                        }

                        /* Update product ID */
                        if (textProductID.Text == "")
                        {
                            textProductID.Text = "FFFF";
                        }
                        else
                        {
                            ushort productID = ushort.Parse(textProductID.Text, System.Globalization.NumberStyles.HexNumber);
                            dfuSuffix[2] = (byte)(productID & 0x00FF);
                            dfuSuffix[3] = (byte)((productID >> 8) & 0xFF);
                        }

                        /* Update vendor ID */
                        if (textVendorID.Text == "")
                        {
                            textVendorID.Text = "FFFF";
                        }
                        else
                        {
                            ushort vendorID = ushort.Parse(textVendorID.Text, System.Globalization.NumberStyles.HexNumber);
                            dfuSuffix[4] = (byte)(vendorID & 0x00FF);
                            dfuSuffix[5] = (byte)((vendorID >> 8) & 0xFF);
                        }
                        crcTemp = calculateCRC(crcTemp, FirmwareFile, sizeOfFirmware);
                        crcTemp = calculateCRC(crcTemp, dfuSuffix, 12);
                        dfuSuffix[12] = (byte)(crcTemp & 0xFF);
                        dfuSuffix[13] = (byte)((crcTemp >> 8) & 0xFF);
                        dfuSuffix[14] = (byte)((crcTemp >> 16) & 0xFF);
                        dfuSuffix[15] = (byte)((crcTemp >> 24) & 0xFF);
                        Writefile(FirmwareFilePath, dfuSuffix, 16);
                        /* read file again */
                        ReadFile(FirmwareFilePath, ref FirmwareFile, ref sizeOfFirmware);
                        clearData();
                        displayData(FirmwareFile, sizeOfFirmware);
                    }
                }
                else
                {
                    MessageBox.Show("Please select firmware file to add suffix","Add Suffix",MessageBoxButtons.OK,MessageBoxIcon.Error);
                }
            }
            else
            {
                MessageBox.Show("Please select firmware file to add suffix","Add Suffix",MessageBoxButtons.OK,MessageBoxIcon.Error);
            }
        }

        private void groupBox2_Enter(object sender, EventArgs e)
        {

        }

        private void groupBox3_Enter(object sender, EventArgs e)
        {

        }
    }
}
