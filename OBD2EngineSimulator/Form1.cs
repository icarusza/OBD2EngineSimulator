
using System;
using System.Drawing;
using System.IO.Ports;
using System.Windows.Forms;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Collections.Concurrent;
using System.Diagnostics;

namespace OBD2EngineSimulator
{
    public partial class Form1 : Form
    {
        private SerialPort serialPort;

        private System.Windows.Forms.Timer startupTimer;
        private System.Windows.Forms.Timer sendQueueTimer;
        private System.Windows.Forms.Timer simulationTimer;
        private System.Windows.Forms.Timer logFlushTimer;

        private readonly Stopwatch simulationStopwatch = new Stopwatch();

        private readonly Queue<string> pendingParameterOrder =
            new Queue<string>();

        private readonly HashSet<string> queuedParameters =
            new HashSet<string>();

        private readonly Dictionary<string, string> pendingParameterCommands =
            new Dictionary<string, string>();

        private readonly ConcurrentQueue<string> incomingLogMessages =
            new ConcurrentQueue<string>();

        private const int MaximumLogLines = 500;

        private ComboBox cmbPorts;
        private ComboBox cmbBaud;
        private ComboBox cmbProtocol;
        private ComboBox cmbPID;
        private ComboBox cmbTuneMode;

        private Button btnRefresh;
        private Button btnConnect;
        private Button btnSimulate;

        private Label lblStatus;
        private Label lblConnection;
        private Label lblSimulation;

        private TabControl tabParameters;
        private TextBox txtLog;

        private readonly Dictionary<string, NumericUpDown> numericControls =
            new Dictionary<string, NumericUpDown>();

        private readonly Dictionary<string, TrackBar> sliders =
            new Dictionary<string, TrackBar>();

        private readonly Dictionary<string, Label> valueLabels =
            new Dictionary<string, Label>();

        private readonly Dictionary<string, Label> lambdaDisplays =
            new Dictionary<string, Label>();

        private readonly Dictionary<string, ParameterInfo> parameters =
            new Dictionary<string, ParameterInfo>();

        private bool updatingControls = false;
        private bool simulationRunning = false;
        private double simulationTime = 0;

        // =====================================
        // CAN PROTOCOL INFORMATION
        // =====================================

        public class CanProtocolOption
        {
            public string Name { get; set; }
            public int Bitrate { get; set; }
            public bool Extended { get; set; }
            public string RequestID { get; set; }
            public string ResponseID { get; set; }

            public override string ToString()
            {
                return Name;
            }
        }

        private CanProtocolOption SelectedProtocol
        {
            get { return cmbProtocol.SelectedItem as CanProtocolOption; }
        }

        // =====================================
        // PARAMETER INFORMATION
        // =====================================

        private class ParameterInfo
        {
            public string Name;
            public string Command;
            public string PID;
            public string Unit;
            public decimal Minimum;
            public decimal Maximum;
            public decimal DefaultValue;
            public int DecimalPlaces;
            public int Scale;
        }

        private void SetupParameters()
        {
            AddParameter("RPM", "Engine RPM", "RPM", "0C",
                "RPM", 0, 12000, 850, 0, 1);

            AddParameter("MAP", "Manifold Absolute Pressure", "MAP", "0B",
                "kPa", 0, 170, 85, 0, 1);

            AddParameter("STFT", "Short-Term Fuel Trim - Bank 1", "STFT", "06",
                "%", -100, 99.2m, 3.1m, 1, 10);

            AddParameter("LTFT", "Long-Term Fuel Trim - Bank 1", "LTFT", "07",
                "%", -100, 99.2m, -2.3m, 1, 10);

            AddParameter("AFR", "Commanded / Target AFR", "AFR", "44",
                "AFR", 8, 20, 14.7m, 1, 10);

            AddParameter("MEASURED_AFR", "Simulated / Measured AFR",
                "MEASURED_AFR", "--", "AFR", 8, 20, 14.7m, 1, 10);

            AddParameter("ECT", "Engine Coolant Temperature", "ECT", "05",
                "°C", -40, 215, 90, 0, 1);

            AddParameter("IAT", "Intake Air Temperature", "IAT", "0F",
                "°C", -40, 215, 25, 0, 1);

            AddParameter("TPS", "Throttle Position", "TPS", "11",
                "%", 0, 100, 0, 1, 1);

            AddParameter("SPEED", "Vehicle Speed", "SPEED", "0D",
                "km/h", 0, 255, 0, 0, 1);

            AddParameter("LOAD", "Calculated Engine Load", "LOAD", "04",
                "%", 0, 100, 20, 1, 1);

            AddParameter("MAF", "Mass Air Flow", "MAF", "10",
                "g/s", 0, 254, 3.5m, 1, 10);

            AddParameter("TIMING", "Ignition Timing Advance", "TIMING", "0E",
                "°", -64, 63.5m, 10, 1, 10);
        }

        private void AddParameter(
            string key,
            string name,
            string command,
            string pid,
            string unit,
            decimal min,
            decimal max,
            decimal defaultValue,
            int decimals,
            int scale)
        {
            parameters[key] = new ParameterInfo
            {
                Name = name,
                Command = command,
                PID = pid,
                Unit = unit,
                Minimum = min,
                Maximum = max,
                DefaultValue = defaultValue,
                DecimalPlaces = decimals,
                Scale = scale
            };
        }

        // =====================================
        // FORM STARTUP
        // =====================================

        public Form1()
        {
            InitializeComponent();

            SetupParameters();
            BuildInterface();
            RefreshPorts();

            startupTimer = new System.Windows.Forms.Timer();
            startupTimer.Interval = 2000;
            startupTimer.Tick += StartupTimer_Tick;

            sendQueueTimer = new System.Windows.Forms.Timer();
            sendQueueTimer.Interval = 100;
            sendQueueTimer.Tick += SendQueueTimer_Tick;

            simulationTimer = new System.Windows.Forms.Timer();
            simulationTimer.Interval = 33;
            simulationTimer.Tick += SimulationTimer_Tick;

            logFlushTimer = new System.Windows.Forms.Timer();
            logFlushTimer.Interval = 100;
            logFlushTimer.Tick += LogFlushTimer_Tick;
            logFlushTimer.Start();
        }

        // =====================================
        // BUILD MAIN INTERFACE
        // =====================================

        private void BuildInterface()
        {
            Text = "OBD-II ECU Simulator";
            Size = new Size(1100, 850);
            MinimumSize = new Size(850, 650);
            StartPosition = FormStartPosition.CenterScreen;
            BackColor = Color.FromArgb(240, 243, 247);
            Font = new Font("Segoe UI", 9);

            TableLayoutPanel main = new TableLayoutPanel();
            main.Dock = DockStyle.Fill;
            main.ColumnCount = 1;
            main.RowCount = 4;
            main.Padding = new Padding(12);

            main.RowStyles.Add(new RowStyle(SizeType.Absolute, 65));
            main.RowStyles.Add(new RowStyle(SizeType.Absolute, 125));
            main.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
            main.RowStyles.Add(new RowStyle(SizeType.Absolute, 155));

            Controls.Add(main);

            Label title = new Label();
            title.Text = "OBD-II ECU SIMULATOR";
            title.Font = new Font("Segoe UI", 20, FontStyle.Bold);
            title.Dock = DockStyle.Fill;
            title.TextAlign = ContentAlignment.MiddleLeft;
            title.ForeColor = Color.FromArgb(30, 50, 75);

            main.Controls.Add(title, 0, 0);

            // CONNECTION PANEL

            TableLayoutPanel connection = new TableLayoutPanel();
            connection.Dock = DockStyle.Fill;
            connection.ColumnCount = 4;
            connection.RowCount = 2;
            connection.Padding = new Padding(8);
            connection.BackColor = Color.White;

            for (int i = 0; i < 4; i++)
                connection.ColumnStyles.Add(
                    new ColumnStyle(SizeType.Percent, 25));

            connection.RowStyles.Add(new RowStyle(SizeType.Percent, 65));
            connection.RowStyles.Add(new RowStyle(SizeType.Percent, 35));

            main.Controls.Add(connection, 0, 1);

            cmbPorts = new ComboBox();
            cmbPorts.Dock = DockStyle.Fill;
            cmbPorts.DropDownStyle = ComboBoxStyle.DropDownList;

            cmbBaud = new ComboBox();
            cmbBaud.Dock = DockStyle.Fill;
            cmbBaud.DropDownStyle = ComboBoxStyle.DropDownList;
            cmbBaud.Items.AddRange(new object[]
            {
                "9600", "19200", "38400", "57600", "115200"
            });
            cmbBaud.SelectedItem = "115200";

            cmbProtocol = new ComboBox();
            cmbProtocol.Dock = DockStyle.Fill;
            cmbProtocol.DropDownStyle = ComboBoxStyle.DropDownList;

            cmbProtocol.Items.AddRange(new object[]
            {
                new CanProtocolOption
                {
                    Name = "ISO 15765-4 CAN - 11-bit - 500 kbit/s",
                    Bitrate = 500,
                    Extended = false,
                    RequestID = "7DF",
                    ResponseID = "7E8"
                },
                new CanProtocolOption
                {
                    Name = "ISO 15765-4 CAN - 11-bit - 250 kbit/s",
                    Bitrate = 250,
                    Extended = false,
                    RequestID = "7DF",
                    ResponseID = "7E8"
                },
                new CanProtocolOption
                {
                    Name = "ISO 15765-4 CAN - 29-bit - 500 kbit/s",
                    Bitrate = 500,
                    Extended = true,
                    RequestID = "18DB33F1",
                    ResponseID = "18DAF110"
                },
                new CanProtocolOption
                {
                    Name = "ISO 15765-4 CAN - 29-bit - 250 kbit/s",
                    Bitrate = 250,
                    Extended = true,
                    RequestID = "18DB33F1",
                    ResponseID = "18DAF110"
                }
            });

            cmbProtocol.SelectedIndex = 2;

            btnRefresh = new Button();
            btnRefresh.Text = "Refresh COM";
            btnRefresh.Dock = DockStyle.Fill;
            btnRefresh.Click += btnRefresh_Click;

            btnConnect = new Button();
            btnConnect.Text = "Connect";
            btnConnect.Dock = DockStyle.Fill;
            btnConnect.BackColor = Color.FromArgb(35, 125, 75);
            btnConnect.ForeColor = Color.White;
            btnConnect.FlatStyle = FlatStyle.Flat;
            btnConnect.Click += BtnConnect_Click;

            lblStatus = new Label();
            lblStatus.Text = "DISCONNECTED";
            lblStatus.ForeColor = Color.Red;
            lblStatus.Dock = DockStyle.Fill;
            lblStatus.TextAlign = ContentAlignment.MiddleLeft;
            lblStatus.Font = new Font("Segoe UI", 10, FontStyle.Bold);

            AddConnectionField(connection, "COM PORT", cmbPorts, 0);
            AddConnectionField(connection, "SERIAL BAUD", cmbBaud, 1);
            AddConnectionField(connection, "CAN PROTOCOL", cmbProtocol, 2);
            AddConnectionField(connection, "CONNECTION", lblStatus, 3);

            connection.Controls.Add(btnRefresh, 0, 1);
            connection.Controls.Add(btnConnect, 1, 1);

            lblConnection = new Label();
            lblConnection.Text = "USB SERIAL";
            lblConnection.Dock = DockStyle.Fill;
            lblConnection.TextAlign = ContentAlignment.MiddleLeft;
            lblConnection.ForeColor = Color.DimGray;

            connection.Controls.Add(lblConnection, 2, 1);

            // PARAMETER AREA

            TableLayoutPanel parameterArea = new TableLayoutPanel();
            parameterArea.Dock = DockStyle.Fill;
            parameterArea.ColumnCount = 1;
            parameterArea.RowCount = 3;

            parameterArea.RowStyles.Add(
                new RowStyle(SizeType.Absolute, 45));
            parameterArea.RowStyles.Add(
                new RowStyle(SizeType.Absolute, 55));
            parameterArea.RowStyles.Add(
                new RowStyle(SizeType.Percent, 100));

            main.Controls.Add(parameterArea, 0, 2);

            // PARAMETER SELECTOR

            FlowLayoutPanel selectorPanel = new FlowLayoutPanel();
            selectorPanel.Dock = DockStyle.Fill;
            selectorPanel.BackColor = Color.White;
            selectorPanel.Padding = new Padding(10);
            selectorPanel.WrapContents = false;

            Label selectLabel = new Label();
            selectLabel.Text = "SELECT ECU PARAMETER:";
            selectLabel.AutoSize = true;
            selectLabel.Margin = new Padding(5, 8, 15, 0);
            selectLabel.Font = new Font("Segoe UI", 10, FontStyle.Bold);

            cmbPID = new ComboBox();
            cmbPID.Width = 350;
            cmbPID.DropDownStyle = ComboBoxStyle.DropDownList;
            cmbPID.Font = new Font("Segoe UI", 10);

            selectorPanel.Controls.Add(selectLabel);
            selectorPanel.Controls.Add(cmbPID);
            parameterArea.Controls.Add(selectorPanel, 0, 0);

            // SIMULATION PANEL

            FlowLayoutPanel simulationPanel = new FlowLayoutPanel();
            simulationPanel.Dock = DockStyle.Fill;
            simulationPanel.BackColor = Color.White;
            simulationPanel.Padding = new Padding(8);
            simulationPanel.WrapContents = false;

            Label tuneLabel = new Label();
            tuneLabel.Text = "TUNE MODE:";
            tuneLabel.AutoSize = true;
            tuneLabel.Font = new Font("Segoe UI", 9, FontStyle.Bold);
            tuneLabel.Margin = new Padding(5, 8, 8, 0);

            cmbTuneMode = new ComboBox();
            cmbTuneMode.DropDownStyle = ComboBoxStyle.DropDownList;
            cmbTuneMode.Width = 150;
            cmbTuneMode.Items.AddRange(new object[]
            {
                "Normal Tune",
                "Lean Tune",
                "Rich Tune",
                "Bad Tune"
            });
            cmbTuneMode.SelectedIndex = 0;

            btnSimulate = new Button();
            btnSimulate.Text = "SIMULATE";
            btnSimulate.Width = 120;
            btnSimulate.Height = 32;
            btnSimulate.BackColor = Color.FromArgb(35, 125, 75);
            btnSimulate.ForeColor = Color.White;
            btnSimulate.FlatStyle = FlatStyle.Flat;
            btnSimulate.Click += BtnSimulate_Click;

            lblSimulation = new Label();
            lblSimulation.Text = "SIMULATION STOPPED";
            lblSimulation.AutoSize = true;
            lblSimulation.Font = new Font("Segoe UI", 9, FontStyle.Bold);
            lblSimulation.ForeColor = Color.Gray;
            lblSimulation.Margin = new Padding(12, 8, 0, 0);

            simulationPanel.Controls.Add(tuneLabel);
            simulationPanel.Controls.Add(cmbTuneMode);
            simulationPanel.Controls.Add(btnSimulate);
            simulationPanel.Controls.Add(lblSimulation);

            parameterArea.Controls.Add(simulationPanel, 0, 1);

            // PARAMETER TABS

            tabParameters = new TabControl();
            tabParameters.Dock = DockStyle.Fill;
            tabParameters.Font = new Font("Segoe UI", 9);

            parameterArea.Controls.Add(tabParameters, 0, 2);

            BuildParameterPages();

            cmbPID.SelectedIndexChanged += CmbPID_SelectedIndexChanged;
            cmbProtocol.SelectedIndexChanged += cmbProtocol_SelectedIndexChanged;

            // SERIAL LOG

            GroupBox logGroup = new GroupBox();
            logGroup.Text = "LIVE SERIAL COMMUNICATION";
            logGroup.Dock = DockStyle.Fill;
            logGroup.Padding = new Padding(8);

            main.Controls.Add(logGroup, 0, 3);

            txtLog = new TextBox();
            txtLog.Dock = DockStyle.Fill;
            txtLog.Multiline = true;
            txtLog.ReadOnly = true;
            txtLog.ScrollBars = ScrollBars.Vertical;
            txtLog.Font = new Font("Consolas", 9);
            txtLog.BackColor = Color.FromArgb(25, 30, 38);
            txtLog.ForeColor = Color.LightGreen;

            logGroup.Controls.Add(txtLog);
        }

        private void AddConnectionField(
            TableLayoutPanel panel,
            string title,
            Control control,
            int column)
        {
            Panel field = new Panel();
            field.Dock = DockStyle.Fill;

            Label label = new Label();
            label.Text = title;
            label.Dock = DockStyle.Top;
            label.Height = 20;
            label.Font = new Font("Segoe UI", 8, FontStyle.Bold);
            label.ForeColor = Color.Gray;

            control.Dock = DockStyle.Bottom;
            control.Height = 28;

            field.Controls.Add(control);
            field.Controls.Add(label);

            panel.Controls.Add(field, column, 0);
        }

        // =====================================
        // PARAMETER PAGES
        // =====================================

        private void BuildParameterPages()
        {
            foreach (var item in parameters)
            {
                string key = item.Key;
                ParameterInfo info = item.Value;

                TabPage page = new TabPage(info.Name);
                page.BackColor = Color.White;

                TableLayoutPanel layout = new TableLayoutPanel();
                layout.Dock = DockStyle.Fill;
                layout.Padding = new Padding(30);
                layout.ColumnCount = 1;
                layout.RowCount =
                    key == "AFR" || key == "MEASURED_AFR" ? 7 : 6;

                layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 55));
                layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 35));
                layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 65));
                layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 55));
                layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 45));
                layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 40));

                if (key == "AFR" || key == "MEASURED_AFR")
                    layout.RowStyles.Add(
                        new RowStyle(SizeType.Absolute, 35));

                page.Controls.Add(layout);

                Label name = new Label();
                name.Text = info.Name;
                name.Font = new Font("Segoe UI", 17, FontStyle.Bold);
                name.Dock = DockStyle.Fill;
                name.TextAlign = ContentAlignment.MiddleLeft;

                Label pid = new Label();
                pid.Text = info.PID == "--"
                    ? "Simulated parameter"
                    : "Mode 01 - PID " + info.PID;
                pid.Font = new Font("Segoe UI", 10);
                pid.ForeColor = Color.Gray;
                pid.Dock = DockStyle.Fill;

                Label valueLabel = new Label();
                valueLabel.Text = FormatValue(info, info.DefaultValue);
                valueLabel.Font = new Font("Segoe UI", 22, FontStyle.Bold);
                valueLabel.ForeColor = Color.FromArgb(35, 125, 75);
                valueLabel.Dock = DockStyle.Fill;

                NumericUpDown numeric = new NumericUpDown();
                numeric.Minimum = info.Minimum;
                numeric.Maximum = info.Maximum;
                numeric.Value = info.DefaultValue;
                numeric.DecimalPlaces = info.DecimalPlaces;
                numeric.Increment = info.DecimalPlaces == 0 ? 1 : 0.1m;
                numeric.Width = 180;
                numeric.Font = new Font("Segoe UI", 16);
                numeric.ThousandsSeparator = true;

                TrackBar slider = new TrackBar();
                slider.Minimum = (int)(info.Minimum * info.Scale);
                slider.Maximum = (int)(info.Maximum * info.Scale);
                slider.Value = (int)(info.DefaultValue * info.Scale);
                slider.TickFrequency = Math.Max(
                    1, (slider.Maximum - slider.Minimum) / 20);
                slider.Dock = DockStyle.Fill;

                Label range = new Label();
                range.Text = "Range: " + info.Minimum + " to " +
                    info.Maximum + " " + info.Unit;
                range.ForeColor = Color.Gray;
                range.Dock = DockStyle.Fill;

                layout.Controls.Add(name, 0, 0);
                layout.Controls.Add(pid, 0, 1);
                layout.Controls.Add(valueLabel, 0, 2);
                layout.Controls.Add(slider, 0, 3);
                layout.Controls.Add(numeric, 0, 4);
                layout.Controls.Add(range, 0, 5);

                numericControls[key] = numeric;
                sliders[key] = slider;
                valueLabels[key] = valueLabel;

                if (key == "AFR" || key == "MEASURED_AFR")
                {
                    Label lambda = new Label();
                    lambda.Font = new Font(
                        "Segoe UI", 12, FontStyle.Bold);
                    lambda.ForeColor = Color.DarkBlue;
                    lambda.Dock = DockStyle.Fill;

                    layout.Controls.Add(lambda, 0, 6);
                    lambdaDisplays[key] = lambda;

                    UpdateLambdaDisplay(key, info.DefaultValue);
                }

                numeric.ValueChanged += (sender, e) =>
                {
                    if (updatingControls)
                        return;

                    updatingControls = true;
                    slider.Value = (int)(numeric.Value * info.Scale);
                    valueLabel.Text = FormatValue(info, numeric.Value);
                    updatingControls = false;

                    UpdateLambdaIfNeeded(key, numeric.Value);
                    SendParameter(key);
                };

                slider.ValueChanged += (sender, e) =>
                {
                    if (updatingControls)
                        return;

                    updatingControls = true;
                    numeric.Value = (decimal)slider.Value / info.Scale;
                    valueLabel.Text = FormatValue(info, numeric.Value);
                    updatingControls = false;

                    UpdateLambdaIfNeeded(key, numeric.Value);
                    SendParameter(key);
                };

                tabParameters.TabPages.Add(page);
                cmbPID.Items.Add(info.Name);
            }

            if (tabParameters.TabPages.Count > 0)
            {
                tabParameters.SelectedIndex = 0;
                cmbPID.SelectedIndex = 0;
            }
        }

        private string FormatValue(ParameterInfo info, decimal value)
        {
            return value.ToString(
                "F" + info.DecimalPlaces,
                CultureInfo.InvariantCulture) + " " + info.Unit;
        }

        // =====================================
        // SET VALUE PROGRAMMATICALLY
        // =====================================

        private void SetParameterValue(string key, decimal requestedValue)
        {
            if (!numericControls.ContainsKey(key))
                return;

            ParameterInfo info = parameters[key];

            decimal value = requestedValue;

            if (value < info.Minimum) value = info.Minimum;
            if (value > info.Maximum) value = info.Maximum;

            value = decimal.Round(
                value * info.Scale,
                0,
                MidpointRounding.AwayFromZero) / info.Scale;

            NumericUpDown numeric = numericControls[key];

            if (numeric.Value == value)
                return;

            updatingControls = true;

            numeric.Value = value;
            sliders[key].Value = (int)(value * info.Scale);
            valueLabels[key].Text = FormatValue(info, value);

            updatingControls = false;

            UpdateLambdaIfNeeded(key, value);
            SendParameter(key);
        }

        // =====================================
        // DROPDOWN PAGE SELECTION
        // =====================================

        private void CmbPID_SelectedIndexChanged(
            object sender,
            EventArgs e)
        {
            if (cmbPID.SelectedIndex < 0)
                return;

            tabParameters.SelectedIndex = cmbPID.SelectedIndex;

            Log("Selected parameter: " +
                cmbPID.SelectedItem.ToString());
        }

        // =====================================
        // LAMBDA DISPLAY
        // =====================================

        private void UpdateLambdaIfNeeded(string key, decimal value)
        {
            if (key == "AFR" || key == "MEASURED_AFR")
                UpdateLambdaDisplay(key, value);
        }

        private void UpdateLambdaDisplay(string key, decimal afr)
        {
            if (!lambdaDisplays.ContainsKey(key))
                return;

            decimal lambda = afr / 14.7m;

            lambdaDisplays[key].Text =
                "Lambda: " +
                lambda.ToString("F3", CultureInfo.InvariantCulture);
        }

        // =====================================
        // DRIVING SIMULATOR
        // =====================================

        private void BtnSimulate_Click(object sender, EventArgs e)
        {
            if (!simulationRunning)
            {
                if (serialPort == null || !serialPort.IsOpen)
                {
                    MessageBox.Show(
                        "Please connect to the Arduino before starting the simulation.",
                        "OBD-II Simulator",
                        MessageBoxButtons.OK,
                        MessageBoxIcon.Information);
                    return;
                }

                simulationTime = 0;
                simulationStopwatch.Restart();

                simulationRunning = true;

                btnSimulate.Text = "STOP";
                btnSimulate.BackColor = Color.FromArgb(180, 45, 45);

                lblSimulation.Text = "SIMULATION RUNNING";
                lblSimulation.ForeColor = Color.Green;

                cmbTuneMode.Enabled = false;

                Log("Driving simulation started.");
                Log("Tune mode: " + cmbTuneMode.SelectedItem);

                UpdateDrivingSimulation();

                simulationTimer.Start();
            }
            else
            {
                StopSimulation();
            }
        }

        private void StopSimulation()
        {
            if (simulationTimer != null)
                simulationTimer.Stop();

            simulationStopwatch.Stop();
            simulationRunning = false;

            if (btnSimulate != null)
            {
                btnSimulate.Text = "SIMULATE";
                btnSimulate.BackColor = Color.FromArgb(35, 125, 75);
            }

            if (cmbTuneMode != null)
                cmbTuneMode.Enabled = true;

            if (lblSimulation != null)
            {
                lblSimulation.Text = "SIMULATION STOPPED";
                lblSimulation.ForeColor = Color.Gray;
            }

            Log("Driving simulation stopped. Manual controls restored.");
        }

        private void SimulationTimer_Tick(object sender, EventArgs e)
        {
            if (!simulationRunning)
                return;

            simulationTime = simulationStopwatch.Elapsed.TotalSeconds % 60.0;

            UpdateDrivingSimulation();
        }

        private double SmoothStep(double value)
        {
            value = Math.Max(0, Math.Min(1, value));
            return value * value * (3.0 - 2.0 * value);
        }

        private double Lerp(double start, double end, double amount)
        {
            return start + (end - start) * amount;
        }

        private void UpdateDrivingSimulation()
        {
            double t = simulationTime;

            double rpm = 850;
            double speed = 0;
            double tps = 2;
            double map = 35;
            double load = 18;
            double timing = 18;
            double commandedAfr = 14.7;
            double maf = 3.5;

            // 0-8 seconds: idle

            if (t < 8)
            {
                rpm = 850 + 20 * Math.Sin(t * 1.2);
                speed = 0;
                tps = 2 + Math.Sin(t) * 0.5;
                map = 35 + Math.Sin(t * 0.8) * 2;
                load = 18 + Math.Sin(t) * 1.5;
                timing = 18 + Math.Sin(t * 0.5) * 2;
                commandedAfr = 14.7;
            }

            // 8-20 seconds: acceleration

            else if (t < 20)
            {
                double p = SmoothStep((t - 8) / 12.0);

                rpm = Lerp(850, 6500, p);
                speed = Lerp(0, 145, p);
                tps = Lerp(5, 85, p);
                map = Lerp(35, 160, p);
                load = Lerp(20, 95, p);
                timing = Lerp(18, 5, p);
                commandedAfr = Lerp(14.7, 12.5, p);
            }

            // 20-22 seconds: gear change

            else if (t < 22)
            {
                double p = SmoothStep((t - 20) / 2.0);

                rpm = Lerp(6500, 4300, p);
                speed = Lerp(145, 155, p);
                tps = Lerp(85, 45, p);
                map = Lerp(160, 85, p);
                load = Lerp(95, 55, p);
                timing = Lerp(5, 12, p);
                commandedAfr = Lerp(12.5, 13.2, p);
            }

            // 22-38 seconds: cruising

            else if (t < 38)
            {
                double wave = Math.Sin((t - 22) * 0.5);

                rpm = 3000 + wave * 180;
                speed = 110 + wave * 3;
                tps = 22 + wave * 2;
                map = 55 + wave * 4;
                load = 35 + wave * 3;
                timing = 18 + wave * 2;
                commandedAfr = 14.7;
            }

            // 38-50 seconds: deceleration

            else if (t < 50)
            {
                double p = SmoothStep((t - 38) / 12.0);

                rpm = Lerp(3000, 850, p);
                speed = Lerp(110, 0, p);
                tps = Lerp(15, 0, p);
                map = Lerp(45, 28, p);
                load = Lerp(25, 15, p);
                timing = Lerp(20, 12, p);
                commandedAfr = Lerp(14.7, 16.0, p);
            }

            // 50-60 seconds: idle

            else
            {
                double wave = Math.Sin((t - 50) * 0.8);

                rpm = 850 + wave * 15;
                speed = 0;
                tps = 1.5;
                map = 34 + wave * 1.5;
                load = 17 + wave;
                timing = 18 + wave;
                commandedAfr = 14.7;
            }

            // Airflow estimate

            maf = 2.0 + (rpm / 6500.0) * (load / 100.0) * 220.0;
            maf = Math.Max(0, Math.Min(254, maf));

            // TUNE MODES

            double stft;
            double ltft;
            double measuredAfr;

            string mode = cmbTuneMode.SelectedItem?.ToString()
                ?? "Normal Tune";

            double slowWave = Math.Sin(t * 0.65);
            double fastWave = Math.Sin(t * 1.7);

            if (mode == "Lean Tune")
            {
                stft = 25 + slowWave * 8;
                ltft = 12 + Math.Sin(t * 0.15) * 2;
                measuredAfr = commandedAfr + 1.8 + fastWave * 0.35;
            }
            else if (mode == "Rich Tune")
            {
                stft = -25 + slowWave * 8;
                ltft = -12 + Math.Sin(t * 0.15) * 2;
                measuredAfr = commandedAfr - 1.8 + fastWave * 0.35;
            }
            else if (mode == "Bad Tune")
            {
                stft = 30 * Math.Sin(t * 0.65);
                ltft = 18 * Math.Sin(t * 0.22);
                measuredAfr = commandedAfr +
                    2.2 * Math.Sin(t * 0.65) +
                    0.8 * Math.Sin(t * 1.7);
            }
            else
            {
                stft = 1.5 + slowWave * 2;
                ltft = -1.5 + Math.Sin(t * 0.15);
                measuredAfr = commandedAfr + fastWave * 0.08;
            }

            stft = Math.Max(-100, Math.Min(99.2, stft));
            ltft = Math.Max(-100, Math.Min(99.2, ltft));
            measuredAfr = Math.Max(8, Math.Min(20, measuredAfr));

            // APPLY ALL ENGINE PARAMETERS

            SetParameterValue("RPM", (decimal)rpm);
            SetParameterValue("MAP", (decimal)map);
            SetParameterValue("STFT", (decimal)stft);
            SetParameterValue("LTFT", (decimal)ltft);
            SetParameterValue("AFR", (decimal)commandedAfr);
            SetParameterValue("MEASURED_AFR", (decimal)measuredAfr);
            SetParameterValue("TPS", (decimal)tps);
            SetParameterValue("SPEED", (decimal)speed);
            SetParameterValue("LOAD", (decimal)load);
            SetParameterValue("MAF", (decimal)maf);
            SetParameterValue("TIMING", (decimal)timing);

            SetParameterValue("ECT", 90);
            SetParameterValue("IAT", 30);
        }

        // =====================================
        // COM PORT REFRESH
        // =====================================

        private void RefreshPorts()
        {
            string previousPort = cmbPorts.SelectedItem?.ToString();

            cmbPorts.Items.Clear();

            string[] ports = SerialPort.GetPortNames();
            Array.Sort(ports);

            cmbPorts.Items.AddRange(ports);

            if (previousPort != null &&
                cmbPorts.Items.Contains(previousPort))
                cmbPorts.SelectedItem = previousPort;
            else if (cmbPorts.Items.Count > 0)
                cmbPorts.SelectedIndex = 0;

            Log("COM ports refreshed. Found: " + ports.Length);
        }

        private void btnRefresh_Click(object sender, EventArgs e)
        {
            RefreshPorts();
        }

        // =====================================
        // CAN PROTOCOL SELECTION
        // =====================================

        private void cmbProtocol_SelectedIndexChanged(
            object sender,
            EventArgs e)
        {
            if (SelectedProtocol == null)
                return;

            Log("CAN protocol selected: " + SelectedProtocol.Name);
            Log("Request ID: " + SelectedProtocol.RequestID);
            Log("Response ID: " + SelectedProtocol.ResponseID);
        }

        // =====================================
        // CONNECT / DISCONNECT
        // =====================================

        private void BtnConnect_Click(object sender, EventArgs e)
        {
            if (serialPort != null && serialPort.IsOpen)
            {
                DisconnectArduino();
                return;
            }

            if (cmbPorts.SelectedItem == null)
            {
                MessageBox.Show("Please select a COM port.");
                return;
            }

            try
            {
                serialPort = new SerialPort(
                    cmbPorts.SelectedItem.ToString(),
                    int.Parse(cmbBaud.SelectedItem.ToString()),
                    Parity.None,
                    8,
                    StopBits.One
                );

                serialPort.NewLine = "\n";
                serialPort.WriteTimeout = 250;
                serialPort.ReadTimeout = 250;
                serialPort.DataReceived += SerialPort_DataReceived;

                serialPort.Open();

                btnConnect.Text = "Disconnect";
                lblStatus.Text = "CONNECTED";
                lblStatus.ForeColor = Color.Green;

                Log("Connected to " + serialPort.PortName);

                if (SelectedProtocol != null)
                    Log("Selected ECU protocol: " + SelectedProtocol.Name);

                startupTimer.Start();
            }
            catch (Exception ex)
            {
                Log("CONNECTION ERROR: " + ex.Message);
                MessageBox.Show("Connection failed: " + ex.Message);
                DisconnectArduino();
            }
        }

        private void StartupTimer_Tick(object sender, EventArgs e)
        {
            startupTimer.Stop();

            Log("Sending initial engine parameters...");

            foreach (string key in parameters.Keys)
                SendParameter(key);
        }

        private void DisconnectArduino()
        {
            StopSimulation();

            if (startupTimer != null)
                startupTimer.Stop();

            if (sendQueueTimer != null)
                sendQueueTimer.Stop();

            pendingParameterOrder.Clear();
            queuedParameters.Clear();
            pendingParameterCommands.Clear();

            try
            {
                if (serialPort != null)
                {
                    serialPort.DataReceived -= SerialPort_DataReceived;

                    if (serialPort.IsOpen)
                        serialPort.Close();

                    serialPort.Dispose();
                    serialPort = null;
                }
            }
            catch (Exception ex)
            {
                Log("DISCONNECT ERROR: " + ex.Message);
            }

            if (btnConnect != null)
                btnConnect.Text = "Connect";

            if (lblStatus != null)
            {
                lblStatus.Text = "DISCONNECTED";
                lblStatus.ForeColor = Color.Red;
            }

            Log("Disconnected.");
        }

        // =====================================
        // SEND PARAMETER TO ARDUINO
        // =====================================

        private void SendParameter(string key)
        {
            if (serialPort == null || !serialPort.IsOpen)
                return;

            if (!numericControls.ContainsKey(key))
                return;

            ParameterInfo info = parameters[key];
            decimal value = numericControls[key].Value;

            string command = info.Command + ":" +
                value.ToString(CultureInfo.InvariantCulture);

            // Replace any older pending value for this parameter.

            pendingParameterCommands[key] = command;

            if (queuedParameters.Add(key))
                pendingParameterOrder.Enqueue(key);

            if (!sendQueueTimer.Enabled)
                sendQueueTimer.Start();
        }

        private void SendQueueTimer_Tick(object sender, EventArgs e)
        {
            if (serialPort == null || !serialPort.IsOpen)
            {
                sendQueueTimer.Stop();
                pendingParameterOrder.Clear();
                queuedParameters.Clear();
                pendingParameterCommands.Clear();
                return;
            }

            if (pendingParameterOrder.Count == 0)
            {
                sendQueueTimer.Stop();
                return;
            }

            string key = pendingParameterOrder.Dequeue();
            queuedParameters.Remove(key);

            if (!pendingParameterCommands.TryGetValue(
                key, out string command))
                return;

            pendingParameterCommands.Remove(key);

            try
            {
                serialPort.WriteLine(command);
                Log("TX: " + command);
            }
            catch (Exception ex)
            {
                Log("SEND ERROR: " + ex.Message);
            }

            if (pendingParameterOrder.Count == 0)
                sendQueueTimer.Stop();
        }

        // =====================================
        // RECEIVE ARDUINO MESSAGES
        // =====================================

        private void SerialPort_DataReceived(
            object sender,
            SerialDataReceivedEventArgs e)
        {
            try
            {
                SerialPort port = sender as SerialPort;

                if (port == null || !port.IsOpen)
                    return;

                while (port.BytesToRead > 0)
                {
                    string message = port.ReadLine().Trim();

                    if (!string.IsNullOrWhiteSpace(message))
                        incomingLogMessages.Enqueue(
                            "ARDUINO RX: " + message);
                }
            }
            catch (TimeoutException)
            {
            }
            catch (InvalidOperationException)
            {
            }
            catch (Exception ex)
            {
                incomingLogMessages.Enqueue(
                    "RECEIVE ERROR: " + ex.Message);
            }
        }

        // =====================================
        // BATCHED LOG DISPLAY
        // =====================================

        private void LogFlushTimer_Tick(object sender, EventArgs e)
        {
            if (txtLog == null || txtLog.IsDisposed)
                return;

            List<string> batch = new List<string>();

            // Drain incoming messages in one UI update.

            while (batch.Count < 150 &&
                   incomingLogMessages.TryDequeue(out string message))
            {
                batch.Add(
                    DateTime.Now.ToString("HH:mm:ss.fff") +
                    "  " + message);
            }

            if (batch.Count > 0)
                AppendLogBatch(batch);
        }

        private void AppendLogBatch(List<string> lines)
        {
            if (txtLog == null || txtLog.IsDisposed)
                return;

            txtLog.AppendText(
                string.Join(Environment.NewLine, lines) +
                Environment.NewLine);

            if (txtLog.Lines.Length > MaximumLogLines)
            {
                string[] currentLines = txtLog.Lines;

                int removeCount =
                    currentLines.Length - MaximumLogLines;

                txtLog.Lines = currentLines
                    .Skip(removeCount)
                    .ToArray();
            }

            txtLog.SelectionStart = txtLog.TextLength;
            txtLog.ScrollToCaret();
        }

        private void Log(string message)
        {
            if (txtLog == null || txtLog.IsDisposed)
                return;

            string line =
                DateTime.Now.ToString("HH:mm:ss.fff") +
                "  " + message +
                Environment.NewLine;

            txtLog.AppendText(line);
        }

        // =====================================
        // APPLICATION SHUTDOWN
        // =====================================

        protected override void OnFormClosing(FormClosingEventArgs e)
        {
            StopSimulation();
            DisconnectArduino();

            if (startupTimer != null)
            {
                startupTimer.Stop();
                startupTimer.Dispose();
            }

            if (sendQueueTimer != null)
            {
                sendQueueTimer.Stop();
                sendQueueTimer.Dispose();
            }

            if (simulationTimer != null)
            {
                simulationTimer.Stop();
                simulationTimer.Dispose();
            }

            if (logFlushTimer != null)
            {
                logFlushTimer.Stop();
                logFlushTimer.Dispose();
            }

            base.OnFormClosing(e);
        }
    }
}
