using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.IO.Ports;
using System.Windows.Forms;

namespace STM32_MotorControl_UI
{
    public partial class Form1 : Form
    {
        private readonly SerialPort serialPort = new SerialPort();
        private int fakeDataIndex = 0;
        private DateTime nextTemperatureLogTime = DateTime.MinValue;
        private readonly string temperatureLogPath =
            @"C:\Users\madde\Desktop\YAZILIM\UI\log.txt";

        public Form1()
        {
            InitializeComponent();

            InitializeConnectionPanel();
            InitializeTelemetryPanel();
            InitializeCommandPanel();
            BindButtonEvents();

            serialPort.DataReceived += SerialPort_DataReceived;

            this.FormClosing += Form1_FormClosing;
        }

        private void InitializeConnectionPanel()
        {
            cmbPorts.DropDownStyle = ComboBoxStyle.DropDownList;
            cmbBaudrate.DropDownStyle = ComboBoxStyle.DropDownList;

            cmbBaudrate.Items.Clear();
            cmbBaudrate.Items.Add("9600");
            cmbBaudrate.Items.Add("57600");
            cmbBaudrate.Items.Add("115200");
            cmbBaudrate.Items.Add("230400");

            cmbBaudrate.SelectedItem = "115200";

            lblConnectionStatus.Text = "Durum: Bağlı değil";

            SetConnectionUiState(false);
            RefreshComPorts();
        }

        private void InitializeTelemetryPanel()
        {
            lblTemperature.Text = "Sıcaklık: -- °C";
            lblPwm.Text = "PWM: -- %";
            lblEncoder.Text = "Encoder: --";
            lblValvePosition.Text = "Valf Konumu: -- %";
            lblTargetValve.Text = "Hedef Valf: -- %";
            lblTempSource.Text = "Sıcaklık Kaynağı: --";
            lblSystemState.Text = "Durum: --";
            lblLastPacket.Text = "Son Veri: --";

            pbValvePosition.Minimum = 0;
            pbValvePosition.Maximum = 100;
            pbValvePosition.Value = 0;

            pbPwm.Minimum = 0;
            pbPwm.Maximum = 100;
            pbPwm.Value = 0;

            txtRawData.Multiline = true;
            txtRawData.ScrollBars = ScrollBars.Vertical;
            txtRawData.ReadOnly = true;
            txtRawData.Clear();
        }

        private void InitializeCommandPanel()
        {
            nudTempOverride.Minimum = 0;
            nudTempOverride.Maximum = 80;
            nudTempOverride.DecimalPlaces = 1;
            nudTempOverride.Increment = 0.5M;
            nudTempOverride.Value = 36.0M;

            lblCommandStatus.Text = "Son Komut: --";
        }

        private void BindButtonEvents()
        {
            btnRefreshPorts.Click -= btnRefreshPorts_Click;
            btnRefreshPorts.Click += btnRefreshPorts_Click;

            btnConnect.Click -= btnConnect_Click;
            btnConnect.Click += btnConnect_Click;

            btnDisconnect.Click -= btnDisconnect_Click;
            btnDisconnect.Click += btnDisconnect_Click;

            btnFakeData.Click -= btnFakeData_Click;
            btnFakeData.Click += btnFakeData_Click;

            btnSendTempOverride.Click -= btnSendTempOverride_Click;
            btnSendTempOverride.Click += btnSendTempOverride_Click;
        }

        private void SetConnectionUiState(bool isConnected)
        {
            cmbPorts.Enabled = !isConnected;
            cmbBaudrate.Enabled = !isConnected;
            btnRefreshPorts.Enabled = !isConnected;
            btnConnect.Enabled = !isConnected;
            btnDisconnect.Enabled = isConnected;
        }

        private void RefreshComPorts()
        {
            cmbPorts.Items.Clear();

            string[] ports = SerialPort.GetPortNames();
            Array.Sort(ports);

            foreach (string port in ports)
            {
                cmbPorts.Items.Add(port);
            }

            if (cmbPorts.Items.Count > 0)
            {
                cmbPorts.SelectedIndex = 0;
                lblConnectionStatus.Text = "Durum: COM port listelendi";
            }
            else
            {
                lblConnectionStatus.Text = "Durum: COM port bulunamadı";
            }
        }

        private void btnRefreshPorts_Click(object? sender, EventArgs e)
        {
            RefreshComPorts();
        }

        private void btnConnect_Click(object? sender, EventArgs e)
        {
            if (cmbPorts.SelectedItem == null)
            {
                MessageBox.Show("Lütfen bir COM port seç.");
                return;
            }

            if (cmbBaudrate.SelectedItem == null)
            {
                MessageBox.Show("Lütfen baudrate seç.");
                return;
            }

            try
            {
                if (serialPort.IsOpen)
                {
                    serialPort.Close();
                }

                serialPort.PortName = cmbPorts.SelectedItem.ToString();
                serialPort.BaudRate = int.Parse(cmbBaudrate.SelectedItem.ToString()!);
                serialPort.DataBits = 8;
                serialPort.Parity = Parity.None;
                serialPort.StopBits = StopBits.One;
                serialPort.Handshake = Handshake.None;

                serialPort.NewLine = "\n";
                serialPort.ReadTimeout = 1000;
                serialPort.WriteTimeout = 1000;

                serialPort.Open();
                serialPort.DiscardInBuffer();
                serialPort.DiscardOutBuffer();

                lblConnectionStatus.Text = "Durum: Bağlandı - " + serialPort.PortName;
                SetConnectionUiState(true);
            }
            catch (Exception ex)
            {
                lblConnectionStatus.Text = "Durum: Bağlantı hatası";
                SetConnectionUiState(false);

                MessageBox.Show("Bağlantı hatası:\n" + ex.Message);
            }
        }

        private void btnDisconnect_Click(object? sender, EventArgs e)
        {
            DisconnectSerialPort();
        }

        private void DisconnectSerialPort()
        {
            try
            {
                if (serialPort.IsOpen)
                {
                    serialPort.Close();
                }

                lblConnectionStatus.Text = "Durum: Bağlı değil";
                SetConnectionUiState(false);
            }
            catch (Exception ex)
            {
                MessageBox.Show("Bağlantı kesme hatası:\n" + ex.Message);
            }
        }

        private bool EnsureSerialPortOpen()
        {
            if (serialPort.IsOpen)
                return true;

            if (cmbPorts.SelectedItem == null || cmbBaudrate.SelectedItem == null)
            {
                lblConnectionStatus.Text = "Durum: Bağlantı yok";
                SetConnectionUiState(false);
                return false;
            }

            try
            {
                serialPort.PortName = cmbPorts.SelectedItem.ToString();
                serialPort.BaudRate = int.Parse(cmbBaudrate.SelectedItem.ToString()!);
                serialPort.DataBits = 8;
                serialPort.Parity = Parity.None;
                serialPort.StopBits = StopBits.One;
                serialPort.Handshake = Handshake.None;
                serialPort.NewLine = "\n";
                serialPort.ReadTimeout = 1000;
                serialPort.WriteTimeout = 1000;

                serialPort.Open();
                serialPort.DiscardInBuffer();
                serialPort.DiscardOutBuffer();

                lblConnectionStatus.Text = "Durum: Bağlandı - " + serialPort.PortName;
                SetConnectionUiState(true);
                return true;
            }
            catch (Exception ex)
            {
                lblConnectionStatus.Text = "Durum: Bağlantı hatası";
                lblCommandStatus.Text = "Son Komut: Gönderilemedi - bağlantı yok";
                SetConnectionUiState(false);
                AppendRawData("OPEN ERROR -> " + ex.Message);
                return false;
            }
        }

        private void SerialPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            try
            {
                string line = serialPort.ReadLine();
                line = line.Trim();

                if (string.IsNullOrWhiteSpace(line))
                    return;

                BeginInvoke(new Action(() =>
                {
                    ProcessReceivedLine(line, true);
                }));
            }
            catch (TimeoutException)
            {
                // Satır tamamlanmadan timeout olursa buraya düşebilir.
            }
            catch (InvalidOperationException)
            {
                // Port kapanırken veri gelirse oluşabilir. Kritik değil.
            }
            catch (Exception ex)
            {
                BeginInvoke(new Action(() =>
                {
                    lblConnectionStatus.Text = "Durum: UART okuma hatası";
                    MessageBox.Show("UART okuma hatası:\n" + ex.Message);
                }));
            }
        }

        private void ProcessReceivedLine(string line, bool writeTemperatureLog)
        {
            AppendRawData("RX -> " + line);

            Dictionary<string, string> data = ParseTelemetryLine(line);

            if (data.ContainsKey("TEMP"))
            {
                lblTemperature.Text = "Sıcaklık: " + data["TEMP"] + " °C";

                if (writeTemperatureLog)
                {
                    LogTemperatureIfDue(data["TEMP"]);
                }
            }

            if (data.ContainsKey("PWM"))
            {
                lblPwm.Text = "PWM: " + data["PWM"] + " %";

                if (TryParseInt(data["PWM"], out int pwmValue))
                {
                    SetProgressBarValue(pbPwm, pwmValue);
                }
            }

            if (data.ContainsKey("ENC"))
            {
                lblEncoder.Text = "Encoder: " + data["ENC"];
            }

            if (data.ContainsKey("POS"))
            {
                lblValvePosition.Text = "Valf Konumu: " + data["POS"] + " %";

                if (TryParseInt(data["POS"], out int valvePosition))
                {
                    SetProgressBarValue(pbValvePosition, valvePosition);
                }
            }

            if (data.ContainsKey("TARGET"))
            {
                lblTargetValve.Text = "Hedef Valf: " + data["TARGET"] + " %";
            }

            if (data.ContainsKey("SRC"))
            {
                lblTempSource.Text = "Sıcaklık Kaynağı: " + data["SRC"];
            }

            if (data.ContainsKey("STATE"))
            {
                lblSystemState.Text = "Durum: " + data["STATE"];
            }

            lblLastPacket.Text = "Son Veri: " + DateTime.Now.ToString("HH:mm:ss.fff");
        }

        private Dictionary<string, string> ParseTelemetryLine(string line)
        {
            Dictionary<string, string> data = new Dictionary<string, string>();

            string[] parts = line.Split(';');

            foreach (string part in parts)
            {
                string[] keyValue = part.Split('=', 2);

                if (keyValue.Length != 2)
                    continue;

                string key = keyValue[0].Trim().ToUpperInvariant();
                string value = keyValue[1].Trim();

                if (!data.ContainsKey(key))
                {
                    data.Add(key, value);
                }
            }

            return data;
        }

        private bool TryParseInt(string text, out int value)
        {
            return int.TryParse(text, NumberStyles.Integer, CultureInfo.InvariantCulture, out value);
        }

        private void SetProgressBarValue(ProgressBar progressBar, int value)
        {
            if (value < progressBar.Minimum)
                value = progressBar.Minimum;

            if (value > progressBar.Maximum)
                value = progressBar.Maximum;

            progressBar.Value = value;
        }

        private void AppendRawData(string line)
        {
            if (txtRawData.TextLength > 7000)
            {
                txtRawData.Clear();
            }

            txtRawData.AppendText(line + Environment.NewLine);
        }

        private void LogTemperatureIfDue(string temperatureText)
        {
            DateTime now = DateTime.Now;

            if (now < nextTemperatureLogTime)
                return;

            try
            {
                string? directory = Path.GetDirectoryName(temperatureLogPath);

                if (!string.IsNullOrWhiteSpace(directory))
                {
                    Directory.CreateDirectory(directory);
                }

                string logLine =
                    now.ToString("yyyy-MM-dd HH:mm:ss", CultureInfo.InvariantCulture) +
                    ";TEMP=" +
                    temperatureText +
                    Environment.NewLine;

                File.AppendAllText(temperatureLogPath, logLine);
                nextTemperatureLogTime = now.AddSeconds(1);
            }
            catch (Exception ex)
            {
                AppendRawData("LOG ERROR -> " + ex.Message);
                nextTemperatureLogTime = now.AddSeconds(1);
            }
        }

        private void btnFakeData_Click(object? sender, EventArgs e)
        {
            string[] fakeLines =
            {
                "TEMP=24.8;PWM=30;ENC=320;POS=80;TARGET=80;SRC=REAL;STATE=LOW_TEMP_OPENING",
                "TEMP=28.5;PWM=25;ENC=580;POS=60;TARGET=60;SRC=REAL;STATE=NORMAL",
                "TEMP=32.4;PWM=35;ENC=810;POS=40;TARGET=40;SRC=REAL;STATE=HOT_LIMITING",
                "TEMP=36.2;PWM=45;ENC=110;POS=8;TARGET=0;SRC=REAL;STATE=HIGH_TEMP_CLOSING",
                "TEMP=36.4;PWM=0;ENC=0;POS=0;TARGET=0;SRC=REAL;STATE=VALVE_CLOSED"
            };

            string selectedLine = fakeLines[fakeDataIndex];

            ProcessReceivedLine(selectedLine, false);

            fakeDataIndex++;

            if (fakeDataIndex >= fakeLines.Length)
            {
                fakeDataIndex = 0;
            }
        }

        private void btnSendTempOverride_Click(object? sender, EventArgs e)
        {
            decimal temperature = nudTempOverride.Value;
            int durationMs = 20000;

            string command = "TEMP_OVERRIDE=" +
                             temperature.ToString("0.0", CultureInfo.InvariantCulture) +
                             ";DUR=" +
                             durationMs;

            SendCommandToStm32(command);
        }

        private void SendCommandToStm32(string command)
        {
            if (string.IsNullOrWhiteSpace(command))
            {
                MessageBox.Show("Gönderilecek komut boş olamaz.");
                return;
            }

            try
            {
                if (!EnsureSerialPortOpen())
                {
                    lblCommandStatus.Text = "Son Komut: Gönderilemedi - bağlantı yok";
                    AppendRawData("TX TEST -> " + command);
                    return;
                }

                serialPort.Write(command + "\n");

                lblCommandStatus.Text = "Son Komut: Gönderildi - " + command;
                AppendRawData("TX -> " + command);
            }
            catch (Exception ex)
            {
                lblCommandStatus.Text = "Son Komut: Gönderme hatası";
                MessageBox.Show("Komut gönderme hatası:\n" + ex.Message);
            }
        }

        private void Form1_FormClosing(object? sender, FormClosingEventArgs e)
        {
            DisconnectSerialPort();
        }

        // Designer tarafında yanlışlıkla bağlı kalmış eventler olabilir.
        // Boş bırakıyoruz ki tasarım ekranı hata vermesin.

        private void button1_Click(object? sender, EventArgs e)
        {

        }

        private void comboBox1_SelectedIndexChanged(object? sender, EventArgs e)
        {

        }

        private void lblConnectionStatus_Click(object? sender, EventArgs e)
        {

        }

        private void label1_Click(object? sender, EventArgs e)
        {

        }
        private void numericUpDown1_ValueChanged(object? sender, EventArgs e)
        {

        }
    }
}
