namespace STM32_MotorControl_UI
{
    partial class Form1
    {
        /// <summary>
        ///  Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        ///  Clean up any resources being used.
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

        #region Windows Form Designer generated code

        /// <summary>
        ///  Required method for Designer support - do not modify
        ///  the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            grpConnection = new GroupBox();
            lblComPortTitle = new Label();
            cmbPorts = new ComboBox();
            lblBaudrateTitle = new Label();
            cmbBaudrate = new ComboBox();
            btnRefreshPorts = new Button();
            btnConnect = new Button();
            btnDisconnect = new Button();
            lblConnectionStatus = new Label();
            lblTemperature = new Label();
            lblPwm = new Label();
            lblEncoder = new Label();
            lblValvePosition = new Label();
            lblTargetValve = new Label();
            lblTempSource = new Label();
            lblSystemState = new Label();
            lblLastPacket = new Label();
            pbValvePosition = new ProgressBar();
            pbPwm = new ProgressBar();
            txtRawData = new TextBox();
            btnFakeData = new Button();
            nudTempOverride = new NumericUpDown();
            btnSendTempOverride = new Button();
            lblCommandStatus = new Label();
            ((System.ComponentModel.ISupportInitialize)nudTempOverride).BeginInit();
            SuspendLayout();
            // 
            // grpConnection
            // 
            grpConnection.Location = new Point(29, 23);
            grpConnection.Name = "grpConnection";
            grpConnection.Size = new Size(250, 125);
            grpConnection.TabIndex = 0;
            grpConnection.TabStop = false;
            grpConnection.Text = "STM32 Bağlantı";
            // 
            // lblComPortTitle
            // 
            lblComPortTitle.AutoSize = true;
            lblComPortTitle.BackColor = SystemColors.MenuHighlight;
            lblComPortTitle.Location = new Point(29, 162);
            lblComPortTitle.Name = "lblComPortTitle";
            lblComPortTitle.Size = new Size(75, 20);
            lblComPortTitle.TabIndex = 1;
            lblComPortTitle.Text = "COM Port:";
            // 
            // cmbPorts
            // 
            cmbPorts.FormattingEnabled = true;
            cmbPorts.Location = new Point(110, 159);
            cmbPorts.Name = "cmbPorts";
            cmbPorts.Size = new Size(151, 28);
            cmbPorts.TabIndex = 2;
            cmbPorts.SelectedIndexChanged += comboBox1_SelectedIndexChanged;
            // 
            // lblBaudrateTitle
            // 
            lblBaudrateTitle.AutoSize = true;
            lblBaudrateTitle.BackColor = SystemColors.MenuHighlight;
            lblBaudrateTitle.Location = new Point(29, 201);
            lblBaudrateTitle.Name = "lblBaudrateTitle";
            lblBaudrateTitle.Size = new Size(72, 20);
            lblBaudrateTitle.TabIndex = 3;
            lblBaudrateTitle.Text = "Baudrate:";
            // 
            // cmbBaudrate
            // 
            cmbBaudrate.FormattingEnabled = true;
            cmbBaudrate.Location = new Point(110, 198);
            cmbBaudrate.Name = "cmbBaudrate";
            cmbBaudrate.Size = new Size(151, 28);
            cmbBaudrate.TabIndex = 4;
            cmbBaudrate.Text = "115200";
            // 
            // btnRefreshPorts
            // 
            btnRefreshPorts.Location = new Point(29, 238);
            btnRefreshPorts.Name = "btnRefreshPorts";
            btnRefreshPorts.Size = new Size(65, 32);
            btnRefreshPorts.TabIndex = 5;
            btnRefreshPorts.Text = "Yenile";
            btnRefreshPorts.UseVisualStyleBackColor = true;
            // 
            // btnConnect
            // 
            btnConnect.Location = new Point(29, 276);
            btnConnect.Name = "btnConnect";
            btnConnect.Size = new Size(65, 32);
            btnConnect.TabIndex = 6;
            btnConnect.Text = "Bağlan";
            btnConnect.UseVisualStyleBackColor = true;
            // 
            // btnDisconnect
            // 
            btnDisconnect.Location = new Point(29, 314);
            btnDisconnect.Name = "btnDisconnect";
            btnDisconnect.Size = new Size(65, 32);
            btnDisconnect.TabIndex = 7;
            btnDisconnect.Text = "Kes";
            btnDisconnect.UseVisualStyleBackColor = true;
            // 
            // lblConnectionStatus
            // 
            lblConnectionStatus.AutoSize = true;
            lblConnectionStatus.Location = new Point(110, 244);
            lblConnectionStatus.Name = "lblConnectionStatus";
            lblConnectionStatus.Size = new Size(133, 20);
            lblConnectionStatus.TabIndex = 8;
            lblConnectionStatus.Text = "Durum: Bağlı değil";
            lblConnectionStatus.Click += lblConnectionStatus_Click;
            // 
            // lblTemperature
            // 
            lblTemperature.AutoSize = true;
            lblTemperature.Location = new Point(324, 24);
            lblTemperature.Name = "lblTemperature";
            lblTemperature.Size = new Size(96, 20);
            lblTemperature.TabIndex = 9;
            lblTemperature.Text = "Sıcaklık: -- °C";
            lblTemperature.Click += label1_Click;
            // 
            // lblPwm
            // 
            lblPwm.AutoSize = true;
            lblPwm.Location = new Point(324, 60);
            lblPwm.Name = "lblPwm";
            lblPwm.Size = new Size(79, 20);
            lblPwm.TabIndex = 10;
            lblPwm.Text = "PWM: -- %";
            // 
            // lblEncoder
            // 
            lblEncoder.AutoSize = true;
            lblEncoder.Location = new Point(324, 91);
            lblEncoder.Name = "lblEncoder";
            lblEncoder.Size = new Size(82, 20);
            lblEncoder.TabIndex = 11;
            lblEncoder.Text = "Encoder: --";
            // 
            // lblValvePosition
            // 
            lblValvePosition.AutoSize = true;
            lblValvePosition.Location = new Point(324, 120);
            lblValvePosition.Name = "lblValvePosition";
            lblValvePosition.Size = new Size(128, 20);
            lblValvePosition.TabIndex = 12;
            lblValvePosition.Text = "Valf Konumu: -- %";
            // 
            // lblTargetValve
            // 
            lblTargetValve.AutoSize = true;
            lblTargetValve.Location = new Point(324, 149);
            lblTargetValve.Name = "lblTargetValve";
            lblTargetValve.Size = new Size(114, 20);
            lblTargetValve.TabIndex = 13;
            lblTargetValve.Text = "Hedef Valf: -- %";
            // 
            // lblTempSource
            // 
            lblTempSource.AutoSize = true;
            lblTempSource.Location = new Point(324, 180);
            lblTempSource.Name = "lblTempSource";
            lblTempSource.Size = new Size(134, 20);
            lblTempSource.TabIndex = 14;
            lblTempSource.Text = "Sıcaklık Kaynağı: --";
            // 
            // lblSystemState
            // 
            lblSystemState.AutoSize = true;
            lblSystemState.Location = new Point(324, 217);
            lblSystemState.Name = "lblSystemState";
            lblSystemState.Size = new Size(73, 20);
            lblSystemState.TabIndex = 15;
            lblSystemState.Text = "Durum: --";
            // 
            // lblLastPacket
            // 
            lblLastPacket.AutoSize = true;
            lblLastPacket.Location = new Point(324, 251);
            lblLastPacket.Name = "lblLastPacket";
            lblLastPacket.Size = new Size(82, 20);
            lblLastPacket.TabIndex = 16;
            lblLastPacket.Text = "Son Veri: --";
            // 
            // pbValvePosition
            // 
            pbValvePosition.BackColor = SystemColors.Info;
            pbValvePosition.Location = new Point(455, 70);
            pbValvePosition.Name = "pbValvePosition";
            pbValvePosition.Size = new Size(109, 10);
            pbValvePosition.TabIndex = 17;
            // 
            // pbPwm
            // 
            pbPwm.BackColor = SystemColors.Info;
            pbPwm.Location = new Point(458, 130);
            pbPwm.Name = "pbPwm";
            pbPwm.Size = new Size(106, 10);
            pbPwm.TabIndex = 18;
            // 
            // txtRawData
            // 
            txtRawData.Location = new Point(461, 166);
            txtRawData.Multiline = true;
            txtRawData.Name = "txtRawData";
            txtRawData.ReadOnly = true;
            txtRawData.ScrollBars = ScrollBars.Vertical;
            txtRawData.Size = new Size(125, 34);
            txtRawData.TabIndex = 19;
            // 
            // btnFakeData
            // 
            btnFakeData.Location = new Point(383, 373);
            btnFakeData.Name = "btnFakeData";
            btnFakeData.Size = new Size(139, 29);
            btnFakeData.TabIndex = 20;
            btnFakeData.Text = "Sahte Veri Al";
            btnFakeData.UseVisualStyleBackColor = true;
            // 
            // nudTempOverride
            // 
            nudTempOverride.DecimalPlaces = 1;
            nudTempOverride.Increment = new decimal(new int[] { 5, 0, 0, 65536 });
            nudTempOverride.Location = new Point(716, 22);
            nudTempOverride.Maximum = new decimal(new int[] { 80, 0, 0, 0 });
            nudTempOverride.Name = "nudTempOverride";
            nudTempOverride.Size = new Size(150, 27);
            nudTempOverride.TabIndex = 21;
            nudTempOverride.Value = new decimal(new int[] { 36, 0, 0, 0 });
            nudTempOverride.ValueChanged += numericUpDown1_ValueChanged;
            // 
            // btnSendTempOverride
            // 
            btnSendTempOverride.Location = new Point(716, 60);
            btnSendTempOverride.Name = "btnSendTempOverride";
            btnSendTempOverride.Size = new Size(133, 51);
            btnSendTempOverride.TabIndex = 22;
            btnSendTempOverride.Text = "20 sn Test Sıcaklığı Gönder";
            btnSendTempOverride.UseVisualStyleBackColor = true;
            // 
            // lblCommandStatus
            // 
            lblCommandStatus.AutoSize = true;
            lblCommandStatus.Location = new Point(716, 120);
            lblCommandStatus.Name = "lblCommandStatus";
            lblCommandStatus.Size = new Size(101, 20);
            lblCommandStatus.TabIndex = 23;
            lblCommandStatus.Text = "Son Komut: --";
            // 
            // Form1
            // 
            AutoScaleDimensions = new SizeF(8F, 20F);
            AutoScaleMode = AutoScaleMode.Font;
            ClientSize = new Size(973, 456);
            Controls.Add(lblCommandStatus);
            Controls.Add(btnSendTempOverride);
            Controls.Add(nudTempOverride);
            Controls.Add(btnFakeData);
            Controls.Add(txtRawData);
            Controls.Add(pbPwm);
            Controls.Add(pbValvePosition);
            Controls.Add(lblLastPacket);
            Controls.Add(lblSystemState);
            Controls.Add(lblTempSource);
            Controls.Add(lblTargetValve);
            Controls.Add(lblValvePosition);
            Controls.Add(lblEncoder);
            Controls.Add(lblPwm);
            Controls.Add(lblTemperature);
            Controls.Add(lblConnectionStatus);
            Controls.Add(btnDisconnect);
            Controls.Add(btnConnect);
            Controls.Add(btnRefreshPorts);
            Controls.Add(cmbBaudrate);
            Controls.Add(lblBaudrateTitle);
            Controls.Add(cmbPorts);
            Controls.Add(lblComPortTitle);
            Controls.Add(grpConnection);
            Name = "Form1";
            Text = "Form1";
            ((System.ComponentModel.ISupportInitialize)nudTempOverride).EndInit();
            ResumeLayout(false);
            PerformLayout();
        }

        #endregion

        private GroupBox grpConnection;
        private Label lblComPortTitle;
        private ComboBox cmbPorts;
        private Label lblBaudrateTitle;
        private ComboBox cmbBaudrate;
        private Button btnRefreshPorts;
        private Button btnConnect;
        private Button btnDisconnect;
        private Label lblConnectionStatus;
        private Label lblTemperature;
        private Label lblPwm;
        private Label lblEncoder;
        private Label lblValvePosition;
        private Label lblTargetValve;
        private Label lblTempSource;
        private Label lblSystemState;
        private Label lblLastPacket;
        private ProgressBar pbValvePosition;
        private ProgressBar pbPwm;
        private TextBox txtRawData;
        private Button btnFakeData;
        private NumericUpDown nudTempOverride;
        private Button btnSendTempOverride;
        private Label lblCommandStatus;
    }
}
