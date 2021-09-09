namespace Test_DFU
{
    partial class frmDFU_demo
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }



        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.stsDevice = new System.Windows.Forms.StatusStrip();
            this.tsDevicelable = new System.Windows.Forms.ToolStripStatusLabel();
            this.tsRefresh = new System.Windows.Forms.ToolStripStatusLabel();
            this.tsDevice = new System.Windows.Forms.ToolStripStatusLabel();
            this.tsStatuslable = new System.Windows.Forms.ToolStripStatusLabel();
            this.tsProgramstatus = new System.Windows.Forms.ToolStripStatusLabel();
            this.tsDevicestatus = new System.Windows.Forms.ToolStripStatusLabel();
            this.grpDevice = new System.Windows.Forms.GroupBox();
            this.cboDevices = new System.Windows.Forms.ComboBox();
            this.cmdenterdfu = new System.Windows.Forms.Button();
            this.panTransfer = new System.Windows.Forms.Panel();
            this.textbcdDevice = new System.Windows.Forms.TextBox();
            this.textProductID = new System.Windows.Forms.TextBox();
            this.textVendorID = new System.Windows.Forms.TextBox();
            this.label3 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.label1 = new System.Windows.Forms.Label();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.butFileToAddSuffixPath = new System.Windows.Forms.Button();
            this.textFirmwareFilePath = new System.Windows.Forms.TextBox();
            this.butAddSuffix = new System.Windows.Forms.Button();
            this.groupUpload = new System.Windows.Forms.GroupBox();
            this.button1 = new System.Windows.Forms.Button();
            this.textUpload = new System.Windows.Forms.TextBox();
            this.groupDownload = new System.Windows.Forms.GroupBox();
            this.butFileSelect = new System.Windows.Forms.Button();
            this.textDownLoad = new System.Windows.Forms.TextBox();
            this.grpDatareceived = new System.Windows.Forms.GroupBox();
            this.progressTransfer = new System.Windows.Forms.ProgressBar();
            this.textDataReceived = new System.Windows.Forms.TextBox();
            this.cmdUpload = new System.Windows.Forms.Button();
            this.cmdDownload = new System.Windows.Forms.Button();
            this.menuStrip1 = new System.Windows.Forms.MenuStrip();
            this.fileToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.exitToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.openFileDialog1 = new System.Windows.Forms.OpenFileDialog();
            this.saveFileDialog1 = new System.Windows.Forms.SaveFileDialog();
            this.folderBrowserDialog1 = new System.Windows.Forms.FolderBrowserDialog();
            this.groupBox2 = new System.Windows.Forms.GroupBox();
            this.label4 = new System.Windows.Forms.Label();
            this.label5 = new System.Windows.Forms.Label();
            this.label6 = new System.Windows.Forms.Label();
            this.label7 = new System.Windows.Forms.Label();
            this.label8 = new System.Windows.Forms.Label();
            this.label9 = new System.Windows.Forms.Label();
            this.label10 = new System.Windows.Forms.Label();
            this.textdwCRC = new System.Windows.Forms.TextBox();
            this.textbLength = new System.Windows.Forms.TextBox();
            this.textSignature = new System.Windows.Forms.TextBox();
            this.textbcdDFU = new System.Windows.Forms.TextBox();
            this.textidVendor = new System.Windows.Forms.TextBox();
            this.textidProduct = new System.Windows.Forms.TextBox();
            this.textbcdDevice1 = new System.Windows.Forms.TextBox();
            this.groupBox3 = new System.Windows.Forms.GroupBox();
            this.stsDevice.SuspendLayout();
            this.grpDevice.SuspendLayout();
            this.panTransfer.SuspendLayout();
            this.groupBox1.SuspendLayout();
            this.groupUpload.SuspendLayout();
            this.groupDownload.SuspendLayout();
            this.grpDatareceived.SuspendLayout();
            this.menuStrip1.SuspendLayout();
            this.groupBox2.SuspendLayout();
            this.groupBox3.SuspendLayout();
            this.SuspendLayout();
            // 
            // stsDevice
            // 
            this.stsDevice.Items.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.tsDevicelable,
            this.tsRefresh,
            this.tsDevice,
            this.tsStatuslable,
            this.tsProgramstatus,
            this.tsDevicestatus});
            this.stsDevice.Location = new System.Drawing.Point(0, 554);
            this.stsDevice.Name = "stsDevice";
            this.stsDevice.Size = new System.Drawing.Size(794, 22);
            this.stsDevice.TabIndex = 1;
            this.stsDevice.Text = "statusStrip1";
            // 
            // tsDevicelable
            // 
            this.tsDevicelable.Name = "tsDevicelable";
            this.tsDevicelable.Size = new System.Drawing.Size(45, 17);
            this.tsDevicelable.Text = "Device:";
            // 
            // tsRefresh
            // 
            this.tsRefresh.Name = "tsRefresh";
            this.tsRefresh.Size = new System.Drawing.Size(0, 17);
            // 
            // tsDevice
            // 
            this.tsDevice.AutoSize = false;
            this.tsDevice.Name = "tsDevice";
            this.tsDevice.Size = new System.Drawing.Size(250, 17);
            this.tsDevice.Text = "Unknown Device     ";
            this.tsDevice.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
            // 
            // tsStatuslable
            // 
            this.tsStatuslable.Name = "tsStatuslable";
            this.tsStatuslable.Size = new System.Drawing.Size(42, 17);
            this.tsStatuslable.Text = "Status:";
            this.tsStatuslable.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
            // 
            // tsProgramstatus
            // 
            this.tsProgramstatus.Name = "tsProgramstatus";
            this.tsProgramstatus.Size = new System.Drawing.Size(26, 17);
            this.tsProgramstatus.Text = "Idle";
            // 
            // tsDevicestatus
            // 
            this.tsDevicestatus.Name = "tsDevicestatus";
            this.tsDevicestatus.Size = new System.Drawing.Size(416, 17);
            this.tsDevicestatus.Spring = true;
            this.tsDevicestatus.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            this.tsDevicestatus.TextChanged += new System.EventHandler(this.tsStatus_TextChanged);
            // 
            // grpDevice
            // 
            this.grpDevice.Controls.Add(this.cboDevices);
            this.grpDevice.Controls.Add(this.cmdenterdfu);
            this.grpDevice.Dock = System.Windows.Forms.DockStyle.Top;
            this.grpDevice.Location = new System.Drawing.Point(0, 24);
            this.grpDevice.Name = "grpDevice";
            this.grpDevice.Size = new System.Drawing.Size(794, 50);
            this.grpDevice.TabIndex = 2;
            this.grpDevice.TabStop = false;
            this.grpDevice.Text = "USB Device";
            // 
            // cboDevices
            // 
            this.cboDevices.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.cboDevices.BackColor = System.Drawing.SystemColors.Info;
            this.cboDevices.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.cboDevices.FormattingEnabled = true;
            this.cboDevices.Location = new System.Drawing.Point(12, 19);
            this.cboDevices.Name = "cboDevices";
            this.cboDevices.Size = new System.Drawing.Size(638, 21);
            this.cboDevices.TabIndex = 0;
            this.cboDevices.SelectedIndexChanged += new System.EventHandler(this.cboDevices_SelectedIndexChanged);
            // 
            // cmdenterdfu
            // 
            this.cmdenterdfu.Cursor = System.Windows.Forms.Cursors.Default;
            this.cmdenterdfu.Enabled = false;
            this.cmdenterdfu.Location = new System.Drawing.Point(664, 12);
            this.cmdenterdfu.Name = "cmdenterdfu";
            this.cmdenterdfu.Size = new System.Drawing.Size(117, 32);
            this.cmdenterdfu.TabIndex = 5;
            this.cmdenterdfu.Text = "Enter DFU mode";
            this.cmdenterdfu.UseVisualStyleBackColor = true;
            this.cmdenterdfu.Click += new System.EventHandler(this.enterdfu_Click);
            // 
            // panTransfer
            // 
            this.panTransfer.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.panTransfer.Controls.Add(this.textbcdDevice);
            this.panTransfer.Controls.Add(this.textProductID);
            this.panTransfer.Controls.Add(this.textVendorID);
            this.panTransfer.Controls.Add(this.label3);
            this.panTransfer.Controls.Add(this.label2);
            this.panTransfer.Controls.Add(this.label1);
            this.panTransfer.Controls.Add(this.groupBox1);
            this.panTransfer.Controls.Add(this.butAddSuffix);
            this.panTransfer.Controls.Add(this.groupUpload);
            this.panTransfer.Controls.Add(this.groupDownload);
            this.panTransfer.Controls.Add(this.grpDatareceived);
            this.panTransfer.Controls.Add(this.cmdUpload);
            this.panTransfer.Controls.Add(this.cmdDownload);
            this.panTransfer.Enabled = false;
            this.panTransfer.Location = new System.Drawing.Point(0, 80);
            this.panTransfer.Name = "panTransfer";
            this.panTransfer.Size = new System.Drawing.Size(794, 469);
            this.panTransfer.TabIndex = 3;
            // 
            // textbcdDevice
            // 
            this.textbcdDevice.Location = new System.Drawing.Point(602, 192);
            this.textbcdDevice.MaxLength = 4;
            this.textbcdDevice.Name = "textbcdDevice";
            this.textbcdDevice.Size = new System.Drawing.Size(45, 20);
            this.textbcdDevice.TabIndex = 25;
            this.textbcdDevice.TextChanged += new System.EventHandler(this.textbcdDevice_TextChanged);
            // 
            // textProductID
            // 
            this.textProductID.Location = new System.Drawing.Point(426, 192);
            this.textProductID.MaxLength = 4;
            this.textProductID.Name = "textProductID";
            this.textProductID.Size = new System.Drawing.Size(45, 20);
            this.textProductID.TabIndex = 24;
            this.textProductID.TextChanged += new System.EventHandler(this.textProductID_TextChanged);
            // 
            // textVendorID
            // 
            this.textVendorID.Location = new System.Drawing.Point(236, 191);
            this.textVendorID.MaxLength = 4;
            this.textVendorID.Name = "textVendorID";
            this.textVendorID.Size = new System.Drawing.Size(45, 20);
            this.textVendorID.TabIndex = 23;
            this.textVendorID.TextChanged += new System.EventHandler(this.textVendorID_TextChanged);
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(512, 195);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(86, 13);
            this.label3.TabIndex = 22;
            this.label3.Text = "Release Number";
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(361, 196);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(55, 13);
            this.label2.TabIndex = 21;
            this.label2.Text = "ProductID";
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(182, 195);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(52, 13);
            this.label1.TabIndex = 20;
            this.label1.Text = "VendorID";
            // 
            // groupBox1
            // 
            this.groupBox1.Controls.Add(this.butFileToAddSuffixPath);
            this.groupBox1.Controls.Add(this.textFirmwareFilePath);
            this.groupBox1.Location = new System.Drawing.Point(179, 138);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Size = new System.Drawing.Size(602, 44);
            this.groupBox1.TabIndex = 19;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "File To Add Suffix";
            // 
            // butFileToAddSuffixPath
            // 
            this.butFileToAddSuffixPath.Location = new System.Drawing.Point(564, 18);
            this.butFileToAddSuffixPath.Name = "butFileToAddSuffixPath";
            this.butFileToAddSuffixPath.Size = new System.Drawing.Size(27, 20);
            this.butFileToAddSuffixPath.TabIndex = 1;
            this.butFileToAddSuffixPath.Text = "...";
            this.butFileToAddSuffixPath.UseVisualStyleBackColor = true;
            this.butFileToAddSuffixPath.Click += new System.EventHandler(this.butFileToAddSuffixPath_Click);
            // 
            // textFirmwareFilePath
            // 
            this.textFirmwareFilePath.Location = new System.Drawing.Point(13, 18);
            this.textFirmwareFilePath.Name = "textFirmwareFilePath";
            this.textFirmwareFilePath.ReadOnly = true;
            this.textFirmwareFilePath.Size = new System.Drawing.Size(533, 20);
            this.textFirmwareFilePath.TabIndex = 0;
            // 
            // butAddSuffix
            // 
            this.butAddSuffix.Location = new System.Drawing.Point(12, 138);
            this.butAddSuffix.Name = "butAddSuffix";
            this.butAddSuffix.Size = new System.Drawing.Size(155, 44);
            this.butAddSuffix.TabIndex = 18;
            this.butAddSuffix.Text = "Add DFU File Suffix";
            this.butAddSuffix.UseVisualStyleBackColor = true;
            this.butAddSuffix.Click += new System.EventHandler(this.butAddSuffix_Click);
            // 
            // groupUpload
            // 
            this.groupUpload.Controls.Add(this.button1);
            this.groupUpload.Controls.Add(this.textUpload);
            this.groupUpload.Location = new System.Drawing.Point(179, 72);
            this.groupUpload.Name = "groupUpload";
            this.groupUpload.Size = new System.Drawing.Size(602, 44);
            this.groupUpload.TabIndex = 17;
            this.groupUpload.TabStop = false;
            this.groupUpload.Text = "Uploaded Firmware File";
            // 
            // button1
            // 
            this.button1.Location = new System.Drawing.Point(124, 44);
            this.button1.Name = "button1";
            this.button1.Size = new System.Drawing.Size(75, 23);
            this.button1.TabIndex = 1;
            this.button1.Text = "button1";
            this.button1.UseVisualStyleBackColor = true;
            // 
            // textUpload
            // 
            this.textUpload.Location = new System.Drawing.Point(13, 19);
            this.textUpload.Name = "textUpload";
            this.textUpload.ReadOnly = true;
            this.textUpload.Size = new System.Drawing.Size(533, 20);
            this.textUpload.TabIndex = 0;
            // 
            // groupDownload
            // 
            this.groupDownload.Controls.Add(this.butFileSelect);
            this.groupDownload.Controls.Add(this.textDownLoad);
            this.groupDownload.Location = new System.Drawing.Point(179, 9);
            this.groupDownload.Name = "groupDownload";
            this.groupDownload.Size = new System.Drawing.Size(602, 44);
            this.groupDownload.TabIndex = 16;
            this.groupDownload.TabStop = false;
            this.groupDownload.Text = "Downloaded Firmware File";
            // 
            // butFileSelect
            // 
            this.butFileSelect.Location = new System.Drawing.Point(561, 17);
            this.butFileSelect.Name = "butFileSelect";
            this.butFileSelect.Size = new System.Drawing.Size(28, 20);
            this.butFileSelect.TabIndex = 1;
            this.butFileSelect.Text = "...";
            this.butFileSelect.UseVisualStyleBackColor = true;
            this.butFileSelect.Click += new System.EventHandler(this.butFileSelect_Click);
            // 
            // textDownLoad
            // 
            this.textDownLoad.Location = new System.Drawing.Point(13, 19);
            this.textDownLoad.Name = "textDownLoad";
            this.textDownLoad.ReadOnly = true;
            this.textDownLoad.Size = new System.Drawing.Size(533, 20);
            this.textDownLoad.TabIndex = 0;
            // 
            // grpDatareceived
            // 
            this.grpDatareceived.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.grpDatareceived.AutoSize = true;
            this.grpDatareceived.Controls.Add(this.groupBox3);
            this.grpDatareceived.Controls.Add(this.groupBox2);
            this.grpDatareceived.Controls.Add(this.progressTransfer);
            this.grpDatareceived.Location = new System.Drawing.Point(0, 218);
            this.grpDatareceived.Name = "grpDatareceived";
            this.grpDatareceived.Size = new System.Drawing.Size(788, 354);
            this.grpDatareceived.TabIndex = 3;
            this.grpDatareceived.TabStop = false;
            this.grpDatareceived.Text = "Data Transferred";
            this.grpDatareceived.Enter += new System.EventHandler(this.grpDatareceived_Enter);
            // 
            // progressTransfer
            // 
            this.progressTransfer.Location = new System.Drawing.Point(12, 222);
            this.progressTransfer.Name = "progressTransfer";
            this.progressTransfer.Size = new System.Drawing.Size(769, 23);
            this.progressTransfer.TabIndex = 4;
            // 
            // textDataReceived
            // 
            this.textDataReceived.HideSelection = false;
            this.textDataReceived.Location = new System.Drawing.Point(6, 18);
            this.textDataReceived.Multiline = true;
            this.textDataReceived.Name = "textDataReceived";
            this.textDataReceived.ReadOnly = true;
            this.textDataReceived.ScrollBars = System.Windows.Forms.ScrollBars.Both;
            this.textDataReceived.Size = new System.Drawing.Size(596, 168);
            this.textDataReceived.TabIndex = 3;
            // 
            // cmdUpload
            // 
            this.cmdUpload.Location = new System.Drawing.Point(12, 73);
            this.cmdUpload.Name = "cmdUpload";
            this.cmdUpload.Size = new System.Drawing.Size(155, 44);
            this.cmdUpload.TabIndex = 1;
            this.cmdUpload.Text = "Upload Firmware";
            this.cmdUpload.UseVisualStyleBackColor = true;
            this.cmdUpload.Click += new System.EventHandler(this.cmdUpload_Click);
            // 
            // cmdDownload
            // 
            this.cmdDownload.Location = new System.Drawing.Point(12, 9);
            this.cmdDownload.Name = "cmdDownload";
            this.cmdDownload.Size = new System.Drawing.Size(155, 44);
            this.cmdDownload.TabIndex = 0;
            this.cmdDownload.Text = "Download Firmware";
            this.cmdDownload.UseVisualStyleBackColor = true;
            this.cmdDownload.Click += new System.EventHandler(this.cmdDownload_Click);
            // 
            // menuStrip1
            // 
            this.menuStrip1.Items.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.fileToolStripMenuItem});
            this.menuStrip1.Location = new System.Drawing.Point(0, 0);
            this.menuStrip1.Name = "menuStrip1";
            this.menuStrip1.Size = new System.Drawing.Size(794, 24);
            this.menuStrip1.TabIndex = 4;
            this.menuStrip1.Text = "menuStrip1";
            this.menuStrip1.ItemClicked += new System.Windows.Forms.ToolStripItemClickedEventHandler(this.menuStrip1_ItemClicked);
            // 
            // fileToolStripMenuItem
            // 
            this.fileToolStripMenuItem.DropDownItems.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.exitToolStripMenuItem});
            this.fileToolStripMenuItem.Name = "fileToolStripMenuItem";
            this.fileToolStripMenuItem.Size = new System.Drawing.Size(37, 20);
            this.fileToolStripMenuItem.Text = "&File";
            // 
            // exitToolStripMenuItem
            // 
            this.exitToolStripMenuItem.Name = "exitToolStripMenuItem";
            this.exitToolStripMenuItem.Size = new System.Drawing.Size(92, 22);
            this.exitToolStripMenuItem.Text = "&Exit";
            this.exitToolStripMenuItem.Click += new System.EventHandler(this.exitToolStripMenuItem_Click);
            // 
            // openFileDialog1
            // 
            this.openFileDialog1.Filter = "Binary file (*.bin)|*.bin";
            // 
            // saveFileDialog1
            // 
            this.saveFileDialog1.Filter = "Hex file (*.hex)|*.hex|All file (*.*)|*.*";
            // 
            // groupBox2
            // 
            this.groupBox2.Controls.Add(this.textbcdDevice1);
            this.groupBox2.Controls.Add(this.textidProduct);
            this.groupBox2.Controls.Add(this.textidVendor);
            this.groupBox2.Controls.Add(this.textbcdDFU);
            this.groupBox2.Controls.Add(this.textSignature);
            this.groupBox2.Controls.Add(this.textbLength);
            this.groupBox2.Controls.Add(this.textdwCRC);
            this.groupBox2.Controls.Add(this.label10);
            this.groupBox2.Controls.Add(this.label9);
            this.groupBox2.Controls.Add(this.label8);
            this.groupBox2.Controls.Add(this.label7);
            this.groupBox2.Controls.Add(this.label6);
            this.groupBox2.Controls.Add(this.label5);
            this.groupBox2.Controls.Add(this.label4);
            this.groupBox2.Location = new System.Drawing.Point(15, 21);
            this.groupBox2.Name = "groupBox2";
            this.groupBox2.Size = new System.Drawing.Size(152, 195);
            this.groupBox2.TabIndex = 5;
            this.groupBox2.TabStop = false;
            this.groupBox2.Text = "DFU file suffix";
            this.groupBox2.Enter += new System.EventHandler(this.groupBox2_Enter);
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(7, 22);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(43, 13);
            this.label4.TabIndex = 0;
            this.label4.Text = "dwCRC";
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(7, 45);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(46, 13);
            this.label5.TabIndex = 1;
            this.label5.Text = "bLength";
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(7, 71);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(50, 13);
            this.label6.TabIndex = 2;
            this.label6.Text = "signature";
            // 
            // label7
            // 
            this.label7.AutoSize = true;
            this.label7.Location = new System.Drawing.Point(7, 97);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(47, 13);
            this.label7.TabIndex = 3;
            this.label7.Text = "bcdDFU";
            // 
            // label8
            // 
            this.label8.AutoSize = true;
            this.label8.Location = new System.Drawing.Point(7, 121);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(49, 13);
            this.label8.TabIndex = 4;
            this.label8.Text = "idVendor";
            // 
            // label9
            // 
            this.label9.AutoSize = true;
            this.label9.Location = new System.Drawing.Point(7, 147);
            this.label9.Name = "label9";
            this.label9.Size = new System.Drawing.Size(52, 13);
            this.label9.TabIndex = 5;
            this.label9.Text = "idProduct";
            // 
            // label10
            // 
            this.label10.AutoSize = true;
            this.label10.Location = new System.Drawing.Point(7, 173);
            this.label10.Name = "label10";
            this.label10.Size = new System.Drawing.Size(59, 13);
            this.label10.TabIndex = 6;
            this.label10.Text = "bcdDevice";
            // 
            // textdwCRC
            // 
            this.textdwCRC.Location = new System.Drawing.Point(71, 18);
            this.textdwCRC.Name = "textdwCRC";
            this.textdwCRC.ReadOnly = true;
            this.textdwCRC.Size = new System.Drawing.Size(70, 20);
            this.textdwCRC.TabIndex = 7;
            // 
            // textbLength
            // 
            this.textbLength.Location = new System.Drawing.Point(71, 43);
            this.textbLength.Name = "textbLength";
            this.textbLength.ReadOnly = true;
            this.textbLength.Size = new System.Drawing.Size(70, 20);
            this.textbLength.TabIndex = 8;
            // 
            // textSignature
            // 
            this.textSignature.Location = new System.Drawing.Point(71, 68);
            this.textSignature.Name = "textSignature";
            this.textSignature.ReadOnly = true;
            this.textSignature.Size = new System.Drawing.Size(70, 20);
            this.textSignature.TabIndex = 9;
            // 
            // textbcdDFU
            // 
            this.textbcdDFU.Location = new System.Drawing.Point(71, 93);
            this.textbcdDFU.Name = "textbcdDFU";
            this.textbcdDFU.ReadOnly = true;
            this.textbcdDFU.Size = new System.Drawing.Size(70, 20);
            this.textbcdDFU.TabIndex = 10;
            // 
            // textidVendor
            // 
            this.textidVendor.Location = new System.Drawing.Point(71, 118);
            this.textidVendor.Name = "textidVendor";
            this.textidVendor.ReadOnly = true;
            this.textidVendor.Size = new System.Drawing.Size(70, 20);
            this.textidVendor.TabIndex = 11;
            // 
            // textidProduct
            // 
            this.textidProduct.Location = new System.Drawing.Point(71, 143);
            this.textidProduct.Name = "textidProduct";
            this.textidProduct.ReadOnly = true;
            this.textidProduct.Size = new System.Drawing.Size(70, 20);
            this.textidProduct.TabIndex = 12;
            // 
            // textbcdDevice1
            // 
            this.textbcdDevice1.Location = new System.Drawing.Point(71, 168);
            this.textbcdDevice1.Name = "textbcdDevice1";
            this.textbcdDevice1.ReadOnly = true;
            this.textbcdDevice1.Size = new System.Drawing.Size(70, 20);
            this.textbcdDevice1.TabIndex = 13;
            // 
            // groupBox3
            // 
            this.groupBox3.Controls.Add(this.textDataReceived);
            this.groupBox3.Location = new System.Drawing.Point(173, 21);
            this.groupBox3.Name = "groupBox3";
            this.groupBox3.Size = new System.Drawing.Size(608, 195);
            this.groupBox3.TabIndex = 6;
            this.groupBox3.TabStop = false;
            this.groupBox3.Text = "Firmware Data";
            this.groupBox3.Enter += new System.EventHandler(this.groupBox3_Enter);
            // 
            // frmDFU_demo
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(794, 576);
            this.Controls.Add(this.panTransfer);
            this.Controls.Add(this.grpDevice);
            this.Controls.Add(this.stsDevice);
            this.Controls.Add(this.menuStrip1);
            this.FormBorderStyle = System.Windows.Forms.FormBorderStyle.FixedToolWindow;
            this.MainMenuStrip = this.menuStrip1;
            this.MaximizeBox = false;
            this.Name = "frmDFU_demo";
            this.ShowIcon = false;
            this.Text = "DFU Demo";
            this.Load += new System.EventHandler(this.frmtestDFU_Load);
            this.stsDevice.ResumeLayout(false);
            this.stsDevice.PerformLayout();
            this.grpDevice.ResumeLayout(false);
            this.panTransfer.ResumeLayout(false);
            this.panTransfer.PerformLayout();
            this.groupBox1.ResumeLayout(false);
            this.groupBox1.PerformLayout();
            this.groupUpload.ResumeLayout(false);
            this.groupUpload.PerformLayout();
            this.groupDownload.ResumeLayout(false);
            this.groupDownload.PerformLayout();
            this.grpDatareceived.ResumeLayout(false);
            this.menuStrip1.ResumeLayout(false);
            this.menuStrip1.PerformLayout();
            this.groupBox2.ResumeLayout(false);
            this.groupBox2.PerformLayout();
            this.groupBox3.ResumeLayout(false);
            this.groupBox3.PerformLayout();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        



        private System.Windows.Forms.StatusStrip stsDevice;
        private System.Windows.Forms.ToolStripStatusLabel tsDevicelable;
        private System.Windows.Forms.ToolStripStatusLabel tsDevice;
        private System.Windows.Forms.ToolStripStatusLabel tsRefresh;
        private System.Windows.Forms.GroupBox grpDevice;
        private System.Windows.Forms.ComboBox cboDevices;
        private System.Windows.Forms.Panel panTransfer;
        private System.Windows.Forms.GroupBox grpDatareceived;
        private System.Windows.Forms.Button cmdUpload;
        private System.Windows.Forms.Button cmdDownload;
        private System.Windows.Forms.ToolStripStatusLabel tsDevicestatus;
        private System.Windows.Forms.MenuStrip menuStrip1;
        private System.Windows.Forms.ToolStripMenuItem fileToolStripMenuItem;
        private System.Windows.Forms.ToolStripMenuItem exitToolStripMenuItem;
        private System.Windows.Forms.OpenFileDialog openFileDialog1;
        private System.Windows.Forms.SaveFileDialog saveFileDialog1;
        private System.Windows.Forms.TextBox textDataReceived;
        private System.Windows.Forms.FolderBrowserDialog folderBrowserDialog1;
        private System.Windows.Forms.ToolStripStatusLabel tsStatuslable;
        private System.Windows.Forms.ToolStripStatusLabel tsProgramstatus;
        private System.Windows.Forms.Button cmdenterdfu;
        private System.Windows.Forms.GroupBox groupDownload;
        private System.Windows.Forms.Button butFileSelect;
        private System.Windows.Forms.TextBox textDownLoad;
        private System.Windows.Forms.GroupBox groupUpload;
        private System.Windows.Forms.TextBox textUpload;
        private System.Windows.Forms.ProgressBar progressTransfer;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.Button butFileToAddSuffixPath;
        private System.Windows.Forms.TextBox textFirmwareFilePath;
        private System.Windows.Forms.Button butAddSuffix;
        private System.Windows.Forms.Button button1;
        private System.Windows.Forms.TextBox textbcdDevice;
        private System.Windows.Forms.TextBox textProductID;
        private System.Windows.Forms.TextBox textVendorID;
        private System.Windows.Forms.GroupBox groupBox2;
        private System.Windows.Forms.TextBox textbcdDevice1;
        private System.Windows.Forms.TextBox textidProduct;
        private System.Windows.Forms.TextBox textidVendor;
        private System.Windows.Forms.TextBox textbcdDFU;
        private System.Windows.Forms.TextBox textSignature;
        private System.Windows.Forms.TextBox textbLength;
        private System.Windows.Forms.TextBox textdwCRC;
        private System.Windows.Forms.Label label10;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.GroupBox groupBox3;
    }
}