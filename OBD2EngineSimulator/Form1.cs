using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Globalization;
using System.IO.Ports;
using System.Linq;
using System.Text.RegularExpressions;
using System.Windows.Forms;

namespace OBD2EngineSimulator
{
    public partial class Form1 : Form
    {
        private readonly ArduinoLink link = new ArduinoLink();
        private readonly EngineModel engineModel = new EngineModel();

        private System.Windows.Forms.Timer simulationTimer;
        private System.Windows.Forms.Timer logFlushTimer;

        private readonly Stopwatch simulationStopwatch = new Stopwatch();

        private readonly ConcurrentQueue<string> logQueue =
            new ConcurrentQueue<string>();

        private const int MaximumLogLines = 500;

        private ComboBox cmbPorts = null!;
        private ComboBox cmbBaud = null!;
        private ComboBox cmbProtocol = null!;
        private ComboBox cmbPID = null!;
        private ComboBox cmbTuneMode = null!;
        private ComboBox cmbLogLevel = null!;

        private Button btnRefresh = null!;
        private Button btnConnect = null!;
        private Button btnSimulate = null!;

        private Label lblStatus = null!;
        private Label lblConnection = null!;
        private Label lblHealth = null!;
        private Label lblSimulation = null!;

        private CheckBox chkIgnition = null!;
        private CheckBox chkTraffic = null!;
        private CheckBox chkLinkDependent = null!;
        private CheckBox chkAutoDtc = null!;

        private TabControl tabParameters = null!;
        private TextBox txtLog = null!;

        private readonly ToolTip toolTip = new ToolTip();

        private readonly Dictionary<string, NumericUpDown> numericControls =
            new Dictionary<string, NumericUpDown>();

        private readonly Dictionary<string, TrackBar> sliders =
            new Dictionary<string, TrackBar>();

        private readonly Dictionary<string, Label> valueLabels =
            new Dictionary<string, Label>();

        private readonly Dictionary<string, Label> lambdaDisplays =
            new Dictionary<string, Label>();

        private readonly Dictionary<string, Label> statusLabels =
            new Dictionary<string, Label>();

        private readonly Dictionary<string, ParameterInfo> parameters =
            ParameterDefinitions.All.ToDictionary(p => p.Key);

        // Diagnostics tab
        private static readonly string[] DtcKeys = { "DTC_S", "DTC_P", "DTC_X" };
        private static readonly string[] DtcListNames = { "Stored (confirmed)", "Pending", "Permanent" };
        private const int MaximumDtcsPerList = 8;

        private const string DefaultVin = "1M8GDM9AXKP042788";
        private const string DefaultCalibrationId = "OBDSIM-CAL-0001";
        private const string DefaultEcuName = "ECM-EngineControl";
        private const string DefaultEsn = "OBDSIM-ENG-00001";

        // Lifecycle view and fault injection (Diagnostics tab).
        private ListView lvDtcStatus = null!;
        private TextBox txtFaultCode = null!;
        private ComboBox cmbFaultSeverity = null!;
        private Label lblCycle = null!;

        // Vehicle & ECU tab.
        private TextBox txtEsn = null!;
        private Label lblOdometer = null!;
        private NumericUpDown numOdometer = null!;
        private CheckBox chkTcm = null!;
        private NumericUpDown numResponseDelay = null!;
        private Label lblEcuSettingsStatus = null!;

        // Fault states reported to the firmware (FAULT command): code -> 0/1/2.
        // Active faults are re-sent every 2 s so a lost command cannot leave
        // the firmware with a stale state.
        private readonly Dictionary<string, int> faultStates = new Dictionary<string, int>();
        private readonly HashSet<string> simulationFaults = new HashSet<string>();
        private int faultResendTicks = 0;

        private readonly List<string>[] dtcCodes =
        {
            new List<string>(), new List<string>(), new List<string>()
        };

        private readonly ListBox[] dtcListBoxes = new ListBox[3];
        private TextBox txtDtcCode = null!;
        private ComboBox cmbDtcType = null!;
        private Label lblMil = null!;
        private Label lblFreezeFrame = null!;
        private Label lblDtcStatus = null!;
        private TextBox txtVin = null!;
        private TextBox txtCalId = null!;
        private TextBox txtEcuName = null!;
        private Label lblIdentityStatus = null!;

        // Values reported by the Arduino (INFO / DTC? replies).
        private bool canInitOk = true;
        private string firmwareVersion = "";

        private bool updatingControls = false;
        private bool simulationRunning = false;

        // =====================================
        // CAN PROTOCOL INFORMATION
        // =====================================

        public class CanProtocolOption
        {
            public string Name { get; set; } = "";
            public int Mode { get; set; }
            public int Bitrate { get; set; }
            public bool Extended { get; set; }
            public string RequestID { get; set; } = "";
            public string ResponseID { get; set; } = "";

            public override string ToString()
            {
                return Name;
            }
        }

        private static readonly CanProtocolOption[] CanProtocols =
        {
            new CanProtocolOption
            {
                Name = "ISO 15765-4 CAN - 11-bit - 500 kbit/s",
                Mode = 6, Bitrate = 500, Extended = false,
                RequestID = "7DF / 7E0", ResponseID = "7E8"
            },
            new CanProtocolOption
            {
                Name = "ISO 15765-4 CAN - 11-bit - 250 kbit/s",
                Mode = 8, Bitrate = 250, Extended = false,
                RequestID = "7DF / 7E0", ResponseID = "7E8"
            },
            new CanProtocolOption
            {
                Name = "ISO 15765-4 CAN - 29-bit - 500 kbit/s",
                Mode = 7, Bitrate = 500, Extended = true,
                RequestID = "18DB33F1 / 18DA10F1", ResponseID = "18DAF110"
            },
            new CanProtocolOption
            {
                Name = "ISO 15765-4 CAN - 29-bit - 250 kbit/s",
                Mode = 9, Bitrate = 250, Extended = true,
                RequestID = "18DB33F1 / 18DA10F1", ResponseID = "18DAF110"
            }
        };

        private CanProtocolOption? SelectedProtocol
        {
            get { return cmbProtocol.SelectedItem as CanProtocolOption; }
        }

        private static string DescribeCanMode(string? mode)
        {
            CanProtocolOption? option = CanProtocols.FirstOrDefault(
                p => p.Mode.ToString(CultureInfo.InvariantCulture) == mode);

            return option == null
                ? "mode " + mode
                : (option.Extended ? "29-bit " : "11-bit ") + option.Bitrate + " kbit/s";
        }

        // =====================================
        // FORM STARTUP
        // =====================================

        public Form1()
        {
            InitializeComponent();

            RegisterLinkItems();
            BuildInterface();
            RefreshPorts();

            link.LogMessage += Link_LogMessage;
            link.ItemChanged += Link_ItemChanged;
            link.ItemAdopted += Link_ItemAdopted;
            link.DeviceStatus += Link_DeviceStatus;
            link.ConnectionLost += Link_ConnectionLost;

            simulationTimer = new System.Windows.Forms.Timer();
            simulationTimer.Interval = 33;
            simulationTimer.Tick += SimulationTimer_Tick;

            logFlushTimer = new System.Windows.Forms.Timer();
            logFlushTimer.Interval = 100;
            logFlushTimer.Tick += LogFlushTimer_Tick;
            logFlushTimer.Start();

            RefreshAllStatus();
        }

        // Every value and setting the Arduino must confirm. Lines are built in
        // this order: settings first (CAN mode, ignition), then engine values,
        // then DTC lists, so a freeze frame captured by a new stored DTC
        // contains the current engine values rather than older ones.
        private void RegisterLinkItems()
        {
            link.RegisterItem("CAN", false, 0, false, CanProtocols[0].Mode.ToString(CultureInfo.InvariantCulture));
            link.RegisterItem("IGN", false, 0, false, "1");
            link.RegisterItem("LOG", false, 0, false, "1");
            link.RegisterItem("RESPDELAY", false, 0, false, "0");

            // Stored in the Arduino's EEPROM: read on connect, sent only when edited.
            link.RegisterItem("TCM", false, 0, true, "1");
            link.RegisterItem("VIN", false, 0, true, DefaultVin);
            link.RegisterItem("CALID", false, 0, true, DefaultCalibrationId);
            link.RegisterItem("ECUNAME", false, 0, true, DefaultEcuName);
            link.RegisterItem("ESN", false, 0, true, DefaultEsn);

            foreach (ParameterInfo info in ParameterDefinitions.All)
            {
                link.RegisterItem(
                    info.Key,
                    true,
                    info.Tolerance,
                    false,
                    info.DefaultValue.ToString(CultureInfo.InvariantCulture));
            }

            foreach (string key in DtcKeys)
                link.RegisterItem(key, false, 0, true, "");
        }

        // =====================================
        // BUILD MAIN INTERFACE
        // =====================================

        private void BuildInterface()
        {
            Text = "OBD-II ECU Simulator";
            Size = new Size(1100, 900);
            MinimumSize = new Size(900, 700);
            StartPosition = FormStartPosition.CenterScreen;
            BackColor = Color.FromArgb(240, 243, 247);
            Font = new Font("Segoe UI", 9);

            TableLayoutPanel main = new TableLayoutPanel();
            main.Dock = DockStyle.Fill;
            main.ColumnCount = 1;
            main.RowCount = 4;
            main.Padding = new Padding(12);

            main.RowStyles.Add(new RowStyle(SizeType.Absolute, 65));
            main.RowStyles.Add(new RowStyle(SizeType.Absolute, 140));
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
            connection.RowCount = 3;
            connection.Padding = new Padding(8, 4, 8, 4);
            connection.BackColor = Color.White;

            for (int i = 0; i < 4; i++)
                connection.ColumnStyles.Add(
                    new ColumnStyle(SizeType.Percent, 25));

            connection.RowStyles.Add(new RowStyle(SizeType.Absolute, 52));
            connection.RowStyles.Add(new RowStyle(SizeType.Absolute, 36));
            connection.RowStyles.Add(new RowStyle(SizeType.Absolute, 36));

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
            cmbProtocol.Items.AddRange(CanProtocols);
            cmbProtocol.SelectedIndex = 0;

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
            lblConnection.Text = "Active CAN: not connected";
            lblConnection.Dock = DockStyle.Fill;
            lblConnection.TextAlign = ContentAlignment.MiddleLeft;
            lblConnection.ForeColor = Color.DimGray;

            connection.Controls.Add(lblConnection, 2, 1);

            chkIgnition = new CheckBox();
            chkIgnition.Text = "IGNITION ON";
            chkIgnition.Checked = true;
            chkIgnition.Dock = DockStyle.Fill;
            chkIgnition.Font = new Font("Segoe UI", 9, FontStyle.Bold);
            chkIgnition.CheckedChanged += ChkIgnition_CheckedChanged;
            toolTip.SetToolTip(chkIgnition,
                "When off, the simulated ECU does not answer any CAN request.");

            connection.Controls.Add(chkIgnition, 3, 1);

            FlowLayoutPanel logLevelPanel = new FlowLayoutPanel();
            logLevelPanel.Dock = DockStyle.Fill;
            logLevelPanel.WrapContents = false;
            logLevelPanel.Margin = new Padding(0);

            Label logLevelLabel = new Label();
            logLevelLabel.Text = "ARDUINO LOG:";
            logLevelLabel.AutoSize = true;
            logLevelLabel.Font = new Font("Segoe UI", 8, FontStyle.Bold);
            logLevelLabel.ForeColor = Color.Gray;
            logLevelLabel.Margin = new Padding(3, 8, 3, 0);

            cmbLogLevel = new ComboBox();
            cmbLogLevel.DropDownStyle = ComboBoxStyle.DropDownList;
            cmbLogLevel.Width = 150;
            cmbLogLevel.Items.AddRange(new object[]
            {
                "0 - Errors only",
                "1 - OBD requests",
                "2 - Every CAN frame"
            });
            cmbLogLevel.SelectedIndex = 1;
            cmbLogLevel.SelectedIndexChanged += CmbLogLevel_SelectedIndexChanged;
            toolTip.SetToolTip(cmbLogLevel,
                "Level 2 prints every CAN frame. The Arduino skips log lines that would block it " +
                "(counted as LOGDROP in INFO), so busy buses show gaps in the log.");

            logLevelPanel.Controls.Add(logLevelLabel);
            logLevelPanel.Controls.Add(cmbLogLevel);
            connection.Controls.Add(logLevelPanel, 0, 2);

            chkTraffic = new CheckBox();
            chkTraffic.Text = "Log serial traffic";
            chkTraffic.Dock = DockStyle.Fill;
            toolTip.SetToolTip(chkTraffic,
                "Show every command sent to the Arduino and every OK reply.");

            connection.Controls.Add(chkTraffic, 1, 2);

            lblHealth = new Label();
            lblHealth.Text = "Link: not connected";
            lblHealth.Dock = DockStyle.Fill;
            lblHealth.TextAlign = ContentAlignment.MiddleLeft;
            lblHealth.ForeColor = Color.DimGray;

            connection.Controls.Add(lblHealth, 2, 2);
            connection.SetColumnSpan(lblHealth, 2);

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
                "Bad Tune",
                "Severe Misfire"
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

            chkAutoDtc = new CheckBox();
            chkAutoDtc.Text = "Auto faults";
            chkAutoDtc.AutoSize = true;
            chkAutoDtc.Margin = new Padding(16, 7, 0, 0);
            toolTip.SetToolTip(chkAutoDtc,
                "During simulation, report faults to the ECU: fuel trims beyond ±20 % for 5 s (P0171/P0172), " +
                "Bad Tune (P0300), Severe Misfire (P0302, flashing MIL). The ECU makes them pending, then " +
                "confirmed after two drive cycles.");

            chkLinkDependent = new CheckBox();
            chkLinkDependent.Text = "Link MAF / load to RPM, MAP, IAT";
            chkLinkDependent.AutoSize = true;
            chkLinkDependent.Checked = true;
            chkLinkDependent.Margin = new Padding(16, 7, 0, 0);
            chkLinkDependent.CheckedChanged += ChkLinkDependent_CheckedChanged;
            toolTip.SetToolTip(chkLinkDependent,
                "In manual mode, recalculate mass air flow and engine load (2.0 L speed-density model) " +
                "whenever RPM, MAP or IAT change.");

            simulationPanel.Controls.Add(tuneLabel);
            simulationPanel.Controls.Add(cmbTuneMode);
            simulationPanel.Controls.Add(btnSimulate);
            simulationPanel.Controls.Add(lblSimulation);
            simulationPanel.Controls.Add(chkAutoDtc);
            simulationPanel.Controls.Add(chkLinkDependent);

            parameterArea.Controls.Add(simulationPanel, 0, 1);

            // PARAMETER TABS

            tabParameters = new TabControl();
            tabParameters.Dock = DockStyle.Fill;
            tabParameters.Font = new Font("Segoe UI", 9);

            parameterArea.Controls.Add(tabParameters, 0, 2);

            BuildParameterPages();
            BuildDiagnosticsPage();
            BuildVehiclePage();

            if (tabParameters.TabPages.Count > 0)
            {
                tabParameters.SelectedIndex = 0;
                cmbPID.SelectedIndex = 0;
            }

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
            foreach (ParameterInfo info in ParameterDefinitions.All)
            {
                string key = info.Key;
                bool hasLambda = key == "AFR" || key == "MEASURED_AFR";

                TabPage page = new TabPage(info.Name);
                page.BackColor = Color.White;

                TableLayoutPanel layout = new TableLayoutPanel();
                layout.Dock = DockStyle.Fill;
                layout.Padding = new Padding(30);
                layout.ColumnCount = 1;
                layout.RowCount = hasLambda ? 8 : 7;

                layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 55));
                layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 35));
                layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 65));
                layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 55));
                layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 45));
                layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 30));
                layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 30));

                if (hasLambda)
                    layout.RowStyles.Add(
                        new RowStyle(SizeType.Absolute, 35));

                page.Controls.Add(layout);

                Label name = new Label();
                name.Text = info.Name;
                name.Font = new Font("Segoe UI", 17, FontStyle.Bold);
                name.Dock = DockStyle.Fill;
                name.TextAlign = ContentAlignment.MiddleLeft;

                Label pid = new Label();
                pid.Text = "Mode 01 - PID " + info.PID;
                pid.Font = new Font("Segoe UI", 10);
                pid.ForeColor = Color.Gray;
                pid.Dock = DockStyle.Fill;

                if (key == "FUELSYS")
                    pid.Text += "  (1 = open loop cold, 2 = closed loop, 4 = open loop driving, 8 = open loop fault, 16 = closed loop fault)";

                Label valueLabel = new Label();
                valueLabel.Text = FormatValue(info, info.DefaultValue);
                valueLabel.Font = new Font("Segoe UI", 22, FontStyle.Bold);
                valueLabel.ForeColor = Color.FromArgb(35, 125, 75);
                valueLabel.Dock = DockStyle.Fill;

                NumericUpDown numeric = new NumericUpDown();
                numeric.Minimum = info.Minimum;
                numeric.Maximum = info.Maximum;
                numeric.DecimalPlaces = info.DecimalPlaces;
                numeric.Increment = info.DecimalPlaces == 0
                    ? 1
                    : 1m / (decimal)Math.Pow(10, info.DecimalPlaces);
                numeric.Value = info.DefaultValue;
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

                Label status = new Label();
                status.Dock = DockStyle.Fill;
                status.Font = new Font("Segoe UI", 10, FontStyle.Bold);
                status.TextAlign = ContentAlignment.MiddleLeft;

                layout.Controls.Add(name, 0, 0);
                layout.Controls.Add(pid, 0, 1);
                layout.Controls.Add(valueLabel, 0, 2);
                layout.Controls.Add(slider, 0, 3);
                layout.Controls.Add(numeric, 0, 4);
                layout.Controls.Add(range, 0, 5);
                layout.Controls.Add(status, 0, 6);

                numericControls[key] = numeric;
                sliders[key] = slider;
                valueLabels[key] = valueLabel;
                statusLabels[key] = status;

                if (hasLambda)
                {
                    Label lambda = new Label();
                    lambda.Font = new Font(
                        "Segoe UI", 12, FontStyle.Bold);
                    lambda.ForeColor = Color.DarkBlue;
                    lambda.Dock = DockStyle.Fill;

                    layout.Controls.Add(lambda, 0, 7);
                    lambdaDisplays[key] = lambda;

                    UpdateLambdaDisplay(key, info.DefaultValue);
                }

                numeric.ValueChanged += (sender, e) =>
                {
                    if (updatingControls)
                        return;

                    // Fuel system status allows only single-bit codes.
                    decimal snapped = SnapToAllowed(key, numeric.Value);

                    if (snapped != numeric.Value)
                    {
                        numeric.Value = snapped;  // raises ValueChanged again
                        return;
                    }

                    updatingControls = true;
                    slider.Value = (int)(numeric.Value * info.Scale);
                    valueLabel.Text = FormatValue(info, numeric.Value);
                    updatingControls = false;

                    UpdateLambdaIfNeeded(key, numeric.Value);
                    SendParameter(key);
                    UpdateDependentValuesIfNeeded(key);
                };

                slider.ValueChanged += (sender, e) =>
                {
                    if (updatingControls)
                        return;

                    updatingControls = true;
                    numeric.Value = SnapToAllowed(key, (decimal)slider.Value / info.Scale);
                    slider.Value = (int)(numeric.Value * info.Scale);
                    valueLabel.Text = FormatValue(info, numeric.Value);
                    updatingControls = false;

                    UpdateLambdaIfNeeded(key, numeric.Value);
                    SendParameter(key);
                    UpdateDependentValuesIfNeeded(key);
                };

                tabParameters.TabPages.Add(page);
                cmbPID.Items.Add(info.Name);
            }
        }

        // FUELSYS (PID 03) must be 0 or a single bit: 1, 2, 4, 8, 16. Other
        // values snap to the next code in the direction of the change, so the
        // arrows and the slider step 1 -> 2 -> 4 -> 8 -> 16 and back.
        private static readonly decimal[] FuelSystemCodes = { 0, 1, 2, 4, 8, 16 };
        private decimal lastFuelSystem = 2;

        private decimal SnapToAllowed(string key, decimal value)
        {
            if (key != "FUELSYS")
                return value;

            decimal snapped = value >= lastFuelSystem
                ? FuelSystemCodes.Where(code => code >= value).DefaultIfEmpty(16).Min()
                : FuelSystemCodes.Where(code => code <= value).DefaultIfEmpty(0).Max();

            lastFuelSystem = snapped;
            return snapped;
        }

        private string FormatValue(ParameterInfo info, decimal value)
        {
            return value.ToString(
                "F" + info.DecimalPlaces,
                CultureInfo.InvariantCulture) + " " + info.Unit;
        }

        // =====================================
        // DIAGNOSTICS PAGE (DTCs, MIL, VEHICLE INFORMATION)
        // =====================================

        private void BuildDiagnosticsPage()
        {
            TabPage page = new TabPage("Diagnostics");
            page.BackColor = Color.White;

            TableLayoutPanel layout = new TableLayoutPanel();
            layout.Dock = DockStyle.Fill;
            layout.Padding = new Padding(20);
            layout.ColumnCount = 3;
            layout.RowCount = 8;

            for (int i = 0; i < 3; i++)
                layout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 33.3f));

            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 40));   // MIL + freeze frame
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 24));   // list headers
            layout.RowStyles.Add(new RowStyle(SizeType.Percent, 40));    // lists
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 40));   // add / remove
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 26));   // DTC sync status
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 24));   // lifecycle header
            layout.RowStyles.Add(new RowStyle(SizeType.Percent, 60));    // lifecycle table
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 40));   // faults + drive cycle

            page.Controls.Add(layout);

            lblMil = new Label();
            lblMil.Dock = DockStyle.Fill;
            lblMil.Font = new Font("Segoe UI", 14, FontStyle.Bold);
            lblMil.TextAlign = ContentAlignment.MiddleLeft;

            lblFreezeFrame = new Label();
            lblFreezeFrame.Dock = DockStyle.Fill;
            lblFreezeFrame.TextAlign = ContentAlignment.MiddleLeft;
            lblFreezeFrame.ForeColor = Color.DimGray;
            lblFreezeFrame.Text = "Freeze frame: unknown";

            layout.Controls.Add(lblMil, 0, 0);
            layout.Controls.Add(lblFreezeFrame, 1, 0);
            layout.SetColumnSpan(lblFreezeFrame, 2);

            for (int i = 0; i < 3; i++)
            {
                Label header = new Label();
                header.Text = DtcListNames[i].ToUpperInvariant() + " DTCs (Mode " +
                    (i == 0 ? "03" : i == 1 ? "07" : "0A") + ")";
                header.Font = new Font("Segoe UI", 8, FontStyle.Bold);
                header.ForeColor = Color.Gray;
                header.Dock = DockStyle.Fill;

                ListBox list = new ListBox();
                list.Dock = DockStyle.Fill;
                list.Font = new Font("Consolas", 11);

                dtcListBoxes[i] = list;

                layout.Controls.Add(header, i, 1);
                layout.Controls.Add(list, i, 2);
            }

            FlowLayoutPanel editPanel = new FlowLayoutPanel();
            editPanel.Dock = DockStyle.Fill;
            editPanel.WrapContents = false;

            txtDtcCode = new TextBox();
            txtDtcCode.Width = 90;
            txtDtcCode.CharacterCasing = CharacterCasing.Upper;
            txtDtcCode.MaxLength = 5;
            txtDtcCode.PlaceholderText = "P0171";

            cmbDtcType = new ComboBox();
            cmbDtcType.DropDownStyle = ComboBoxStyle.DropDownList;
            cmbDtcType.Width = 140;
            cmbDtcType.Items.AddRange(DtcListNames);
            cmbDtcType.SelectedIndex = 0;

            Button btnAddDtc = new Button();
            btnAddDtc.Text = "Add";
            btnAddDtc.AutoSize = true;
            btnAddDtc.Click += (sender, e) => AddDtcFromInput();

            Button btnRemoveDtc = new Button();
            btnRemoveDtc.Text = "Remove selected";
            btnRemoveDtc.AutoSize = true;
            btnRemoveDtc.Click += (sender, e) => RemoveSelectedDtcs();

            Button btnClearDtc = new Button();
            btnClearDtc.Text = "Clear stored + pending";
            btnClearDtc.AutoSize = true;
            btnClearDtc.Click += (sender, e) => ClearDtcLists(includePermanent: false);
            toolTip.SetToolTip(btnClearDtc,
                "Like a scan tool's Mode 04 clear: permanent DTCs stay.");

            Button btnClearAllDtc = new Button();
            btnClearAllDtc.Text = "Clear all";
            btnClearAllDtc.AutoSize = true;
            btnClearAllDtc.Click += (sender, e) => ClearDtcLists(includePermanent: true);

            editPanel.Controls.Add(txtDtcCode);
            editPanel.Controls.Add(cmbDtcType);
            editPanel.Controls.Add(btnAddDtc);
            editPanel.Controls.Add(btnRemoveDtc);
            editPanel.Controls.Add(btnClearDtc);
            editPanel.Controls.Add(btnClearAllDtc);

            layout.Controls.Add(editPanel, 0, 3);
            layout.SetColumnSpan(editPanel, 3);

            lblDtcStatus = new Label();
            lblDtcStatus.Dock = DockStyle.Fill;
            lblDtcStatus.TextAlign = ContentAlignment.MiddleLeft;

            layout.Controls.Add(lblDtcStatus, 0, 4);
            layout.SetColumnSpan(lblDtcStatus, 3);

            // DTC lifecycle as the ECU sees it (DTCX in the DTC? reply).
            Label lifecycleHeader = new Label();
            lifecycleHeader.Text = "DTC STATUS IN THE ECU (pending → confirmed after 2 failing drive cycles, " +
                "MIL off after 3 passing cycles, erased after 40 warm-ups)";
            lifecycleHeader.Font = new Font("Segoe UI", 8, FontStyle.Bold);
            lifecycleHeader.ForeColor = Color.Gray;
            lifecycleHeader.Dock = DockStyle.Fill;

            layout.Controls.Add(lifecycleHeader, 0, 5);
            layout.SetColumnSpan(lifecycleHeader, 3);

            lvDtcStatus = new ListView();
            lvDtcStatus.Dock = DockStyle.Fill;
            lvDtcStatus.View = View.Details;
            lvDtcStatus.FullRowSelect = true;
            lvDtcStatus.Font = new Font("Consolas", 9);

            foreach (string column in new[]
            {
                "Code", "ECU", "Status", "Fault now", "Pending", "Confirmed", "MIL",
                "Permanent", "Failed cycles", "Passed cycles", "Warm-ups"
            })
            {
                lvDtcStatus.Columns.Add(column, -2);
            }

            layout.Controls.Add(lvDtcStatus, 0, 6);
            layout.SetColumnSpan(lvDtcStatus, 3);

            FlowLayoutPanel faultPanel = new FlowLayoutPanel();
            faultPanel.Dock = DockStyle.Fill;
            faultPanel.WrapContents = false;

            Label faultLabel = new Label();
            faultLabel.Text = "FAULT:";
            faultLabel.AutoSize = true;
            faultLabel.Font = new Font("Segoe UI", 8, FontStyle.Bold);
            faultLabel.ForeColor = Color.Gray;
            faultLabel.Margin = new Padding(3, 8, 3, 0);

            txtFaultCode = new TextBox();
            txtFaultCode.Width = 90;
            txtFaultCode.CharacterCasing = CharacterCasing.Upper;
            txtFaultCode.MaxLength = 5;
            txtFaultCode.PlaceholderText = "P0420";

            cmbFaultSeverity = new ComboBox();
            cmbFaultSeverity.DropDownStyle = ComboBoxStyle.DropDownList;
            cmbFaultSeverity.Width = 170;
            cmbFaultSeverity.Items.AddRange(new object[] { "Failing", "Severe (flashing MIL)" });
            cmbFaultSeverity.SelectedIndex = 0;

            Button btnFaultPresent = new Button();
            btnFaultPresent.Text = "Fault present";
            btnFaultPresent.AutoSize = true;
            btnFaultPresent.Click += (sender, e) => ReportFaultFromInput(present: true);
            toolTip.SetToolTip(btnFaultPresent,
                "Report a failing test result: the ECU sets the DTC pending now and confirms it " +
                "if it is still failing at the end of the next drive cycle.");

            Button btnFaultGone = new Button();
            btnFaultGone.Text = "Fault gone";
            btnFaultGone.AutoSize = true;
            btnFaultGone.Click += (sender, e) => ReportFaultFromInput(present: false);

            Button btnEndCycle = new Button();
            btnEndCycle.Text = "End drive cycle";
            btnEndCycle.AutoSize = true;
            btnEndCycle.Click += (sender, e) => EndDriveCycle();
            toolTip.SetToolTip(btnEndCycle,
                "Completes a drive cycle now (CYCLE:1). Turning the ignition off after the engine ran " +
                "at least 60 s also completes one.");

            lblCycle = new Label();
            lblCycle.AutoSize = true;
            lblCycle.Margin = new Padding(12, 8, 0, 0);
            lblCycle.ForeColor = Color.DimGray;

            faultPanel.Controls.Add(faultLabel);
            faultPanel.Controls.Add(txtFaultCode);
            faultPanel.Controls.Add(cmbFaultSeverity);
            faultPanel.Controls.Add(btnFaultPresent);
            faultPanel.Controls.Add(btnFaultGone);
            faultPanel.Controls.Add(btnEndCycle);
            faultPanel.Controls.Add(lblCycle);

            layout.Controls.Add(faultPanel, 0, 7);
            layout.SetColumnSpan(faultPanel, 3);

            tabParameters.TabPages.Add(page);
            cmbPID.Items.Add("Diagnostics (DTCs, MIL, faults, drive cycles)");
        }

        // =====================================
        // VEHICLE & ECU PAGE
        // =====================================

        private void BuildVehiclePage()
        {
            TabPage page = new TabPage("Vehicle & ECU");
            page.BackColor = Color.White;

            TableLayoutPanel layout = new TableLayoutPanel();
            layout.Dock = DockStyle.Fill;
            layout.Padding = new Padding(20);
            layout.ColumnCount = 4;
            layout.RowCount = 6;

            for (int i = 0; i < 4; i++)
                layout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 25f));

            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 70));   // identity fields
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 44));   // identity apply
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 44));   // odometer
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 44));   // TCM + response delay
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 44));   // factory reset
            layout.RowStyles.Add(new RowStyle(SizeType.Percent, 100));

            page.Controls.Add(layout);

            txtVin = AddIdentityField(layout, 0, 0, "VIN (17 characters, Mode 09 / 02)", DefaultVin, 17);
            txtCalId = AddIdentityField(layout, 1, 0, "CALIBRATION ID (max 16, Mode 09 / 04)", DefaultCalibrationId, 16);
            txtEcuName = AddIdentityField(layout, 2, 0, "ECU NAME (max 20, Mode 09 / 0A)", DefaultEcuName, 20);
            txtEsn = AddIdentityField(layout, 3, 0, "ENGINE SERIAL (max 16, Mode 09 / 0D)", DefaultEsn, 16);
            txtVin.CharacterCasing = CharacterCasing.Upper;

            FlowLayoutPanel identityPanel = NewRowPanel();

            Button btnApplyIdentity = new Button();
            btnApplyIdentity.Text = "Apply vehicle information";
            btnApplyIdentity.AutoSize = true;
            btnApplyIdentity.Click += (sender, e) => ApplyIdentity();

            lblIdentityStatus = new Label();
            lblIdentityStatus.AutoSize = true;
            lblIdentityStatus.Margin = new Padding(12, 8, 0, 0);

            identityPanel.Controls.Add(btnApplyIdentity);
            identityPanel.Controls.Add(lblIdentityStatus);
            AddRow(layout, identityPanel, 1);

            // Odometer (PID A6): kept by the ECU, set on request.
            FlowLayoutPanel odometerPanel = NewRowPanel();

            lblOdometer = new Label();
            lblOdometer.AutoSize = true;
            lblOdometer.Font = new Font("Segoe UI", 10, FontStyle.Bold);
            lblOdometer.Margin = new Padding(3, 7, 20, 0);
            lblOdometer.Text = "Odometer: unknown";

            numOdometer = new NumericUpDown();
            numOdometer.Width = 120;
            numOdometer.Maximum = 4294967;  // firmware keeps metres in 32 bits
            numOdometer.DecimalPlaces = 1;
            numOdometer.ThousandsSeparator = true;

            Button btnSetOdometer = new Button();
            btnSetOdometer.Text = "Set odometer (km)";
            btnSetOdometer.AutoSize = true;
            btnSetOdometer.Click += (sender, e) =>
                link.SendCommand("ODO:" + numOdometer.Value.ToString("0.0", CultureInfo.InvariantCulture));

            odometerPanel.Controls.Add(lblOdometer);
            odometerPanel.Controls.Add(numOdometer);
            odometerPanel.Controls.Add(btnSetOdometer);
            AddRow(layout, odometerPanel, 2);

            // ECU behaviour.
            FlowLayoutPanel ecuPanel = NewRowPanel();

            chkTcm = new CheckBox();
            chkTcm.Text = "Transmission ECU (TCM) present";
            chkTcm.AutoSize = true;
            chkTcm.Checked = true;
            chkTcm.Margin = new Padding(3, 7, 20, 0);
            chkTcm.CheckedChanged += (sender, e) =>
            {
                if (!updatingControls)
                    link.SetDesired("TCM", chkTcm.Checked ? "1" : "0");
            };
            toolTip.SetToolTip(chkTcm,
                "A second ECU on 7E9 / 18DAF118 that also answers broadcast requests " +
                "(speed, gear, its own DTCs, VIN). Stored in the Arduino's EEPROM.");

            Label delayLabel = new Label();
            delayLabel.Text = "Response delay (ms):";
            delayLabel.AutoSize = true;
            delayLabel.Margin = new Padding(3, 8, 3, 0);

            numResponseDelay = new NumericUpDown();
            numResponseDelay.Width = 80;
            numResponseDelay.Maximum = 5000;
            numResponseDelay.Increment = 10;
            numResponseDelay.ValueChanged += (sender, e) =>
                link.SetDesired("RESPDELAY", ((int)numResponseDelay.Value).ToString(CultureInfo.InvariantCulture));
            toolTip.SetToolTip(numResponseDelay,
                "Simulates a slow ECU. Above 40 ms the ECU first answers 'response pending' (7F xx 78), " +
                "repeated every 4 s.");

            lblEcuSettingsStatus = new Label();
            lblEcuSettingsStatus.AutoSize = true;
            lblEcuSettingsStatus.Margin = new Padding(12, 8, 0, 0);

            ecuPanel.Controls.Add(chkTcm);
            ecuPanel.Controls.Add(delayLabel);
            ecuPanel.Controls.Add(numResponseDelay);
            ecuPanel.Controls.Add(lblEcuSettingsStatus);
            AddRow(layout, ecuPanel, 3);

            FlowLayoutPanel resetPanel = NewRowPanel();

            Button btnFactoryReset = new Button();
            btnFactoryReset.Text = "Factory reset ECU memory…";
            btnFactoryReset.AutoSize = true;
            btnFactoryReset.Click += (sender, e) => FactoryReset();

            resetPanel.Controls.Add(btnFactoryReset);
            AddRow(layout, resetPanel, 4);

            tabParameters.TabPages.Add(page);
            cmbPID.Items.Add("Vehicle & ECU (VIN, odometer, TCM)");
        }

        private static FlowLayoutPanel NewRowPanel()
        {
            FlowLayoutPanel panel = new FlowLayoutPanel();
            panel.Dock = DockStyle.Fill;
            panel.WrapContents = false;
            return panel;
        }

        private static void AddRow(TableLayoutPanel layout, Control control, int row)
        {
            layout.Controls.Add(control, 0, row);
            layout.SetColumnSpan(control, layout.ColumnCount);
        }

        private void FactoryReset()
        {
            if (!link.IsOpen)
                return;

            DialogResult answer = MessageBox.Show(
                "Erase all DTCs, freeze frames, counters, the odometer and the vehicle information " +
                "stored in the Arduino's EEPROM? This cannot be undone.",
                "OBD-II Simulator", MessageBoxButtons.YesNo, MessageBoxIcon.Warning);

            if (answer != DialogResult.Yes)
                return;

            link.SendCommand("FACTORY:1");
            engineModel.ResetFaultTimers();
            Log("ECU memory reset to factory defaults.");
        }

        private TextBox AddIdentityField(TableLayoutPanel layout, int column, int row, string title, string value, int maxLength)
        {
            Panel field = new Panel();
            field.Dock = DockStyle.Fill;
            field.Padding = new Padding(0, 0, 10, 0);

            Label label = new Label();
            label.Text = title;
            label.Dock = DockStyle.Top;
            label.Height = 22;
            label.Font = new Font("Segoe UI", 8, FontStyle.Bold);
            label.ForeColor = Color.Gray;

            TextBox textBox = new TextBox();
            textBox.Dock = DockStyle.Top;
            textBox.Font = new Font("Consolas", 11);
            textBox.MaxLength = maxLength;
            textBox.Text = value;

            field.Controls.Add(textBox);
            field.Controls.Add(label);

            layout.Controls.Add(field, column, row);
            return textBox;
        }

        private static bool IsValidDtc(string code) =>
            Regex.IsMatch(code, "^[PCBU][0-3][0-9A-F]{3}$");

        private static bool IsValidText(string text, int maxLength) =>
            text.Length <= maxLength &&
            text.All(c => c >= 0x20 && c <= 0x7E && c != ';' && c != '#' && c != '|' && c != '=');

        private void AddDtcFromInput()
        {
            string code = txtDtcCode.Text.Trim().ToUpperInvariant();

            if (!IsValidDtc(code))
            {
                MessageBox.Show(
                    "Enter a DTC like P0171 (P, C, B or U, then 0-3, then three hex digits).",
                    "OBD-II Simulator", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }

            AddDtc(cmbDtcType.SelectedIndex, code);
            txtDtcCode.Clear();
        }

        private bool AddDtc(int listIndex, string code)
        {
            List<string> codes = dtcCodes[listIndex];

            if (codes.Contains(code))
                return false;

            if (codes.Count >= MaximumDtcsPerList)
            {
                Log("DTC list '" + DtcListNames[listIndex] + "' is full (" + MaximumDtcsPerList + " codes).");
                return false;
            }

            codes.Add(code);
            PushDtcList(listIndex);
            return true;
        }

        private void RemoveSelectedDtcs()
        {
            for (int i = 0; i < 3; i++)
            {
                if (dtcListBoxes[i].SelectedItem is string code && dtcCodes[i].Remove(code))
                    PushDtcList(i);
            }
        }

        // Sends DTC:CLEAR (or DTC:CLEAR:ALL), which behaves like Mode 04 and
        // also resets the freeze frame and "since cleared" counters.
        private void ClearDtcLists(bool includePermanent)
        {
            link.SendCommand(includePermanent ? "DTC:CLEAR:ALL" : "DTC:CLEAR");
            engineModel.ResetFaultTimers();

            for (int i = 0; i < 3; i++)
            {
                if (i == 2 && !includePermanent)
                    continue;

                if (dtcCodes[i].Count == 0)
                    continue;

                dtcCodes[i].Clear();
                PushDtcList(i);
            }
        }

        private void PushDtcList(int listIndex)
        {
            RefreshDtcListBox(listIndex);
            link.SetDesired(DtcKeys[listIndex], string.Join(",", dtcCodes[listIndex]));
            UpdateDtcStatus();
        }

        private void RefreshDtcListBox(int listIndex)
        {
            ListBox list = dtcListBoxes[listIndex];
            list.BeginUpdate();
            list.Items.Clear();

            foreach (string code in dtcCodes[listIndex])
                list.Items.Add(code);

            list.EndUpdate();
        }

        private void ApplyIdentity()
        {
            string vin = txtVin.Text.Trim().ToUpperInvariant();
            string calId = txtCalId.Text.Trim();
            string ecuName = txtEcuName.Text.Trim();
            string esn = txtEsn.Text.Trim();

            if (!Regex.IsMatch(vin, "^[A-HJ-NPR-Z0-9]{17}$"))
            {
                MessageBox.Show("The VIN must be 17 characters: digits and capital letters except I, O and Q.",
                    "OBD-II Simulator", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }

            if (!IsValidText(calId, 16) || !IsValidText(ecuName, 20) || !IsValidText(esn, 16))
            {
                MessageBox.Show("Calibration ID (max 16), ECU name (max 20) and engine serial (max 16) must be " +
                    "printable ASCII without ; # | or =.", "OBD-II Simulator", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }

            link.SetDesired("VIN", vin);
            link.SetDesired("CALID", calId);
            link.SetDesired("ECUNAME", ecuName);
            link.SetDesired("ESN", esn);
            UpdateIdentityStatus();
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

            if (key == "FUELSYS")
                lastFuelSystem = value;

            UpdateLambdaIfNeeded(key, value);
            SendParameter(key);
        }

        // =====================================
        // LINKED VALUES (manual mode)
        // =====================================

        private void UpdateDependentValuesIfNeeded(string key)
        {
            if (simulationRunning || !chkLinkDependent.Checked)
                return;

            if (key == "RPM" || key == "MAP" || key == "IAT")
                UpdateDependentValues();
        }

        private void UpdateDependentValues()
        {
            (double maf, double load) = EngineModel.ComputeAirflow(
                (double)numericControls["RPM"].Value,
                (double)numericControls["MAP"].Value,
                (double)numericControls["IAT"].Value);

            SetParameterValue("MAF", (decimal)maf);
            SetParameterValue("LOAD", (decimal)load);
        }

        private void ChkLinkDependent_CheckedChanged(object? sender, EventArgs e)
        {
            if (chkLinkDependent.Checked && !simulationRunning)
                UpdateDependentValues();
        }

        // =====================================
        // DROPDOWN PAGE SELECTION
        // =====================================

        private void CmbPID_SelectedIndexChanged(
            object? sender,
            EventArgs e)
        {
            if (cmbPID.SelectedIndex < 0)
                return;

            tabParameters.SelectedIndex = cmbPID.SelectedIndex;

            Log("Selected parameter: " +
                cmbPID.SelectedItem);
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

        private TuneMode SelectedTuneMode =>
            cmbTuneMode.SelectedIndex switch
            {
                1 => TuneMode.Lean,
                2 => TuneMode.Rich,
                3 => TuneMode.Bad,
                4 => TuneMode.Misfire,
                _ => TuneMode.Normal
            };

        private void BtnSimulate_Click(object? sender, EventArgs e)
        {
            if (!simulationRunning)
            {
                if (!link.IsOpen)
                {
                    MessageBox.Show(
                        "Please connect to the Arduino before starting the simulation.",
                        "OBD-II Simulator",
                        MessageBoxButtons.OK,
                        MessageBoxIcon.Information);
                    return;
                }

                // Continue from the current temperatures and fuel level.
                engineModel.Reset(
                    (double)numericControls["ECT"].Value,
                    (double)numericControls["OIL"].Value,
                    (double)numericControls["IAT"].Value,
                    (double)numericControls["FUEL"].Value,
                    (double)numericControls["AMBIENT"].Value,
                    (double)numericControls["BARO"].Value);

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
            if (!simulationRunning)
                return;

            simulationTimer.Stop();
            simulationStopwatch.Stop();
            simulationRunning = false;

            btnSimulate.Text = "SIMULATE";
            btnSimulate.BackColor = Color.FromArgb(35, 125, 75);

            cmbTuneMode.Enabled = true;

            lblSimulation.Text = "SIMULATION STOPPED";
            lblSimulation.ForeColor = Color.Gray;

            // Faults the simulation reported end with it.
            foreach (string code in simulationFaults)
                SetFault(code, 0);

            simulationFaults.Clear();

            Log("Driving simulation stopped. Manual controls restored.");
        }

        // =====================================
        // FAULTS AND DRIVE CYCLES
        // =====================================

        // Reports a live fault state to the firmware, which runs the DTC
        // lifecycle (pending, confirmed after two drive cycles, MIL off after
        // three passing cycles).
        private void SetFault(string code, int severity, bool force = false)
        {
            faultStates.TryGetValue(code, out int current);

            if (current == severity && !force)
                return;

            faultStates[code] = severity;
            link.SendCommand("FAULT:" + code + ":" + severity.ToString(CultureInfo.InvariantCulture));
            Log(severity == 0
                ? "Fault " + code + " gone (passing test reported)."
                : "Fault " + code + " present (severity " + severity + ").");
        }

        // Re-sends every known fault state, passed ones included, so a lost
        // FAULT command (disconnect, Arduino restart) cannot leave the
        // firmware with a stale state. The firmware only saves real changes.
        private void ResendFaultStates()
        {
            foreach (KeyValuePair<string, int> fault in faultStates)
                link.SendCommand("FAULT:" + fault.Key + ":" + fault.Value.ToString(CultureInfo.InvariantCulture));
        }

        private void ReportFaultFromInput(bool present)
        {
            string code = txtFaultCode.Text.Trim().ToUpperInvariant();

            if (!IsValidDtc(code))
            {
                MessageBox.Show(
                    "Enter a DTC like P0171 (P, C, B or U, then 0-3, then three hex digits).",
                    "OBD-II Simulator", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }

            // Always sent, even if the app already thinks so.
            SetFault(code, present ? cmbFaultSeverity.SelectedIndex + 1 : 0, force: true);
        }

        private void EndDriveCycle()
        {
            if (!link.IsOpen)
                return;

            link.SendCommand("CYCLE:1");
            Log("Drive cycle ended (failing codes advance, passing codes count towards MIL off).");
        }

        private void SimulationTimer_Tick(object? sender, EventArgs e)
        {
            if (!simulationRunning)
                return;

            UpdateDrivingSimulation();
        }

        private void UpdateDrivingSimulation()
        {
            EngineSnapshot snapshot = engineModel.Step(
                simulationStopwatch.Elapsed.TotalSeconds,
                SelectedTuneMode,
                chkAutoDtc.Checked);

            foreach (KeyValuePair<string, double> value in snapshot.Values)
                SetParameterValue(value.Key, (decimal)value.Value);

            foreach (KeyValuePair<string, int> fault in snapshot.Faults)
            {
                simulationFaults.Add(fault.Key);
                SetFault(fault.Key, fault.Value);
            }
        }

        // =====================================
        // COM PORT REFRESH
        // =====================================

        private void RefreshPorts()
        {
            string? previousPort = cmbPorts.SelectedItem?.ToString();

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

        private void btnRefresh_Click(object? sender, EventArgs e)
        {
            RefreshPorts();
        }

        // =====================================
        // ECU SETTINGS (CAN, IGNITION, LOG LEVEL)
        // =====================================

        private void cmbProtocol_SelectedIndexChanged(
            object? sender,
            EventArgs e)
        {
            CanProtocolOption? protocol = SelectedProtocol;

            if (protocol == null)
                return;

            Log("CAN protocol selected: " + protocol.Name +
                " (request " + protocol.RequestID + ", response " + protocol.ResponseID + ")");

            link.SetDesired("CAN", protocol.Mode.ToString(CultureInfo.InvariantCulture));
            UpdateProtocolLabel();
        }

        private void ChkIgnition_CheckedChanged(object? sender, EventArgs e)
        {
            link.SetDesired("IGN", chkIgnition.Checked ? "1" : "0");
            UpdateIgnitionLabel();
        }

        private void CmbLogLevel_SelectedIndexChanged(object? sender, EventArgs e)
        {
            link.SetDesired("LOG", cmbLogLevel.SelectedIndex.ToString(CultureInfo.InvariantCulture));
        }

        // =====================================
        // CONNECT / DISCONNECT
        // =====================================

        private void BtnConnect_Click(object? sender, EventArgs e)
        {
            if (link.IsOpen)
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
                string portName = cmbPorts.SelectedItem.ToString()!;
                int baud = int.Parse(cmbBaud.SelectedItem?.ToString() ?? "115200", CultureInfo.InvariantCulture);

                link.Open(portName, baud);

                btnConnect.Text = "Disconnect";
                lblStatus.Text = "CONNECTED";
                lblStatus.ForeColor = Color.Green;
                firmwareVersion = "";

                Log("Connected to " + portName + " at " + baud + " baud.");
                RefreshAllStatus();
            }
            catch (Exception ex)
            {
                Log("CONNECTION ERROR: " + ex.Message);
                MessageBox.Show("Connection failed: " + ex.Message);
                DisconnectArduino();
            }
        }

        private void DisconnectArduino()
        {
            StopSimulation();
            link.Close();
            SetDisconnectedUi();
            Log("Disconnected.");
        }

        private void SetDisconnectedUi()
        {
            btnConnect.Text = "Connect";
            lblStatus.Text = "DISCONNECTED";
            lblStatus.ForeColor = Color.Red;
            RefreshAllStatus();
        }

        // =====================================
        // SEND PARAMETER TO ARDUINO
        // =====================================

        private void SendParameter(string key)
        {
            if (!numericControls.ContainsKey(key))
                return;

            link.SetParameter(key, numericControls[key].Value);
            UpdateParameterStatus(key);
        }

        // =====================================
        // CONFIRMATION STATUS
        // =====================================

        private void Link_LogMessage(string message, bool isTraffic)
        {
            if (!isTraffic || chkTraffic.Checked)
                Log(message);

            // A scan tool cleared the DTCs: faults must be detected again.
            if (message.Contains("EVENT DTC CLEARED BY TESTER", StringComparison.Ordinal))
                engineModel.ResetFaultTimers();
        }

        private void Link_ItemChanged(LinkItem item)
        {
            if (parameters.ContainsKey(item.Key))
            {
                UpdateParameterStatus(item.Key);
                return;
            }

            switch (item.Key)
            {
                case "CAN":
                    UpdateProtocolLabel();
                    break;

                case "IGN":
                    UpdateIgnitionLabel();
                    break;

                case "VIN":
                case "CALID":
                case "ECUNAME":
                case "ESN":
                    UpdateIdentityStatus();
                    break;

                case "TCM":
                case "RESPDELAY":
                    UpdateEcuSettingsStatus();
                    break;

                case "DTC_S":
                case "DTC_P":
                case "DTC_X":
                    UpdateDtcStatus();
                    break;
            }
        }

        // A value stored on the Arduino differs from what the app showed
        // (read on connect, DTC lifecycle, scan-tool clear): show the ECU's value.
        private void Link_ItemAdopted(LinkItem item)
        {
            switch (item.Key)
            {
                case "VIN": txtVin.Text = item.Desired; return;
                case "CALID": txtCalId.Text = item.Desired; return;
                case "ECUNAME": txtEcuName.Text = item.Desired; return;
                case "ESN": txtEsn.Text = item.Desired; return;

                case "TCM":
                    updatingControls = true;
                    chkTcm.Checked = item.Desired == "1";
                    updatingControls = false;
                    UpdateEcuSettingsStatus();
                    return;
            }

            int index = Array.IndexOf(DtcKeys, item.Key);

            if (index < 0)
                return;

            dtcCodes[index].Clear();
            dtcCodes[index].AddRange(item.Desired.Split(',', StringSplitOptions.RemoveEmptyEntries));
            RefreshDtcListBox(index);
            UpdateDtcStatus();
        }

        private void Link_DeviceStatus(string name, string value)
        {
            switch (name)
            {
                case "FW":
                    if (firmwareVersion != value)
                    {
                        firmwareVersion = value;

                        if (link.IsOpen)
                            lblStatus.Text = "CONNECTED (FW " + value + ")";
                    }
                    break;

                case "CANOK":
                    canInitOk = value == "1";
                    UpdateProtocolLabel();
                    break;

                case "MIL":
                    lblMil.Text = value == "2" ? "MIL FLASHING" : value == "1" ? "MIL ON" : "MIL OFF";
                    lblMil.ForeColor = value == "2" ? Color.Firebrick
                        : value == "1" ? Color.DarkOrange
                        : Color.FromArgb(35, 125, 75);
                    break;

                case "ODO":
                    lblOdometer.Text = "Odometer: " + value + " km";
                    break;

                case "CYCLE":
                    lblCycle.Text = "Drive cycles since Arduino start: " + value;
                    break;

                case "DTCX":
                    ShowDtcLifecycle(value);
                    break;

                case "FF":
                    lblFreezeFrame.Text = value == "-"
                        ? "Freeze frame: none stored"
                        : "Freeze frame: captured when " + value + " was stored (Mode 02)";
                    break;
            }
        }

        private void Link_ConnectionLost()
        {
            StopSimulation();
            SetDisconnectedUi();
            MessageBox.Show("The connection to the Arduino was lost.", "OBD-II Simulator",
                MessageBoxButtons.OK, MessageBoxIcon.Warning);
        }

        private static string StatusSymbol(LinkItem item) => item.Status switch
        {
            ValueStatus.Confirmed => "✓",
            ValueStatus.Clamped => "⚠",
            ValueStatus.Failed => "✗",
            _ => "…"
        };

        private static Color StatusColor(ValueStatus status) => status switch
        {
            ValueStatus.Confirmed => Color.FromArgb(35, 125, 75),
            ValueStatus.Clamped => Color.DarkOrange,
            ValueStatus.Failed => Color.Firebrick,
            _ => Color.Gray
        };

        private void UpdateParameterStatus(string key)
        {
            if (!statusLabels.TryGetValue(key, out Label? label))
                return;

            LinkItem item = link.GetItem(key);
            ParameterInfo info = parameters[key];
            string confirmed = item.Confirmed + " " + info.Unit;

            if (!link.IsOpen)
            {
                label.Text = "Not connected - value is sent when the Arduino connects";
                label.ForeColor = Color.Gray;
                return;
            }

            label.Text = item.Status switch
            {
                ValueStatus.Confirmed => "✓ Confirmed by Arduino: " + confirmed,
                ValueStatus.Clamped => "⚠ Arduino applied " + confirmed + " (limited by firmware range)",
                ValueStatus.Failed => "✗ Not confirmed: " + item.FailureReason,
                _ => "… Waiting for Arduino confirmation" +
                     (item.Confirmed != null ? " (last confirmed " + confirmed + ")" : "")
            };

            label.ForeColor = StatusColor(item.Status);
        }

        private void UpdateProtocolLabel()
        {
            if (!link.IsOpen)
            {
                lblConnection.Text = "Active CAN: not connected";
                lblConnection.ForeColor = Color.DimGray;
                return;
            }

            LinkItem item = link.GetItem("CAN");

            if (item.Status == ValueStatus.Confirmed && !canInitOk)
            {
                lblConnection.Text = "CAN: " + DescribeCanMode(item.Confirmed) + " - MCP2515 init failed";
                lblConnection.ForeColor = Color.Firebrick;
                return;
            }

            lblConnection.Text = item.Status switch
            {
                ValueStatus.Confirmed => "Active CAN: " + DescribeCanMode(item.Confirmed) + " ✓",
                ValueStatus.Failed => "CAN change failed: " + item.FailureReason,
                _ => "Switching CAN to " + DescribeCanMode(item.Desired) + " …"
            };

            lblConnection.ForeColor = StatusColor(item.Status);
        }

        private void UpdateIgnitionLabel()
        {
            LinkItem item = link.GetItem("IGN");
            string state = chkIgnition.Checked ? "IGNITION ON" : "IGNITION OFF";

            chkIgnition.Text = link.IsOpen ? state + " " + StatusSymbol(item) : state;
            chkIgnition.ForeColor = link.IsOpen ? StatusColor(item.Status) : SystemColors.ControlText;
        }

        private void UpdateDtcStatus()
        {
            if (!link.IsOpen)
            {
                lblDtcStatus.Text = "DTC lists: not connected";
                lblDtcStatus.ForeColor = Color.Gray;
                return;
            }

            LinkItem[] items = DtcKeys.Select(link.GetItem).ToArray();

            lblDtcStatus.Text = "Arduino DTC lists:  " + string.Join("   ",
                items.Select((item, i) => DtcListNames[i] + " " + StatusSymbol(item) +
                    (item.Status == ValueStatus.Failed ? " (" + item.FailureReason + ")" : "")));

            ValueStatus worst = items.Max(item => item.Status == ValueStatus.Failed ? 3 :
                item.Status == ValueStatus.Clamped ? 2 : item.Status == ValueStatus.Pending ? 1 : 0) switch
            {
                3 => ValueStatus.Failed,
                2 => ValueStatus.Clamped,
                1 => ValueStatus.Pending,
                _ => ValueStatus.Confirmed
            };

            lblDtcStatus.ForeColor = StatusColor(worst);
        }

        private void UpdateIdentityStatus()
        {
            if (!link.IsOpen)
            {
                lblIdentityStatus.Text = "Not connected";
                lblIdentityStatus.ForeColor = Color.Gray;
                return;
            }

            LinkItem vin = link.GetItem("VIN");
            LinkItem calId = link.GetItem("CALID");
            LinkItem ecuName = link.GetItem("ECUNAME");
            LinkItem esn = link.GetItem("ESN");

            lblIdentityStatus.Text =
                "VIN " + StatusSymbol(vin) +
                "   Calibration ID " + StatusSymbol(calId) +
                "   ECU name " + StatusSymbol(ecuName) +
                "   Engine serial " + StatusSymbol(esn);

            bool allConfirmed = vin.Status == ValueStatus.Confirmed &&
                calId.Status == ValueStatus.Confirmed &&
                ecuName.Status == ValueStatus.Confirmed &&
                esn.Status == ValueStatus.Confirmed;

            lblIdentityStatus.ForeColor = allConfirmed
                ? StatusColor(ValueStatus.Confirmed)
                : Color.Gray;
        }

        private void UpdateEcuSettingsStatus()
        {
            if (!link.IsOpen)
            {
                lblEcuSettingsStatus.Text = "Not connected";
                lblEcuSettingsStatus.ForeColor = Color.Gray;
                return;
            }

            LinkItem tcm = link.GetItem("TCM");
            LinkItem delay = link.GetItem("RESPDELAY");

            lblEcuSettingsStatus.Text = "TCM " + StatusSymbol(tcm) + "   Response delay " + StatusSymbol(delay);
            lblEcuSettingsStatus.ForeColor =
                tcm.Status == ValueStatus.Confirmed && delay.Status == ValueStatus.Confirmed
                    ? StatusColor(ValueStatus.Confirmed)
                    : Color.Gray;
        }

        // DTCX=<code>:<status hex>:<failed cycles>:<passed cycles>:<warm-ups>:<permanent>:<E|T>,...
        private void ShowDtcLifecycle(string value)
        {
            lvDtcStatus.BeginUpdate();
            lvDtcStatus.Items.Clear();

            foreach (string entry in value.Split(',', StringSplitOptions.RemoveEmptyEntries))
            {
                string[] fields = entry.Split(':');

                if (fields.Length != 7 ||
                    !int.TryParse(fields[1], NumberStyles.HexNumber, CultureInfo.InvariantCulture, out int status))
                    continue;

                string Flag(int bit) => (status & bit) != 0 ? "yes" : "";

                ListViewItem row = new ListViewItem(fields[0]);
                row.SubItems.Add(fields[6] == "T" ? "TCM" : "ECM");
                row.SubItems.Add(fields[1]);
                row.SubItems.Add(Flag(0x01));
                row.SubItems.Add(Flag(0x04));
                row.SubItems.Add(Flag(0x08));
                row.SubItems.Add(Flag(0x80));
                row.SubItems.Add(fields[5] == "1" ? "yes" : "");
                row.SubItems.Add(fields[2]);
                row.SubItems.Add(fields[3]);
                row.SubItems.Add(fields[4]);
                lvDtcStatus.Items.Add(row);
            }

            lvDtcStatus.EndUpdate();
        }

        private void UpdateHealthLabel()
        {
            if (!link.IsOpen)
            {
                lblHealth.Text = "Link: not connected";
                lblHealth.ForeColor = Color.DimGray;
                return;
            }

            if (!link.IsReady)
            {
                lblHealth.Text = "Link: waiting for the Arduino...";
                lblHealth.ForeColor = Color.DimGray;
                return;
            }

            long? age = link.MillisecondsSinceLastReply;

            lblHealth.Text =
                "Link: " + (link.Responding ? "OK" : "NOT RESPONDING") +
                " | last reply " + (age.HasValue ? age.Value + " ms ago" : "never") +
                " | RTT " + link.LastRoundTripMs + " ms" +
                " | retries " + link.RetryCount +
                " | failed " + link.FailureCount;

            lblHealth.ForeColor = link.Responding
                ? Color.FromArgb(35, 125, 75)
                : Color.Firebrick;
        }

        private void RefreshAllStatus()
        {
            foreach (string key in statusLabels.Keys)
                UpdateParameterStatus(key);

            UpdateProtocolLabel();
            UpdateIgnitionLabel();
            UpdateDtcStatus();
            UpdateIdentityStatus();
            UpdateEcuSettingsStatus();
            UpdateHealthLabel();

            if (!link.IsOpen)
            {
                lblMil.Text = "MIL: unknown";
                lblMil.ForeColor = Color.Gray;
                lblFreezeFrame.Text = "Freeze frame: unknown";
                lblOdometer.Text = "Odometer: unknown";
                lblCycle.Text = "";
                lvDtcStatus.Items.Clear();
            }
        }

        // =====================================
        // BATCHED LOG DISPLAY
        // =====================================

        private void LogFlushTimer_Tick(object? sender, EventArgs e)
        {
            UpdateHealthLabel();

            // Every 2 s (20 ticks of 100 ms).
            if (++faultResendTicks >= 20)
            {
                faultResendTicks = 0;

                if (link.IsReady)
                    ResendFaultStates();
            }

            if (txtLog == null || txtLog.IsDisposed)
                return;

            List<string> batch = new List<string>();

            while (batch.Count < 150 &&
                   logQueue.TryDequeue(out string? message))
            {
                batch.Add(message);
            }

            if (batch.Count > 0)
                AppendLogBatch(batch);
        }

        private void AppendLogBatch(List<string> lines)
        {
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

        // All log output goes through the queue, so it is batched and trimmed.
        private void Log(string message)
        {
            logQueue.Enqueue(
                DateTime.Now.ToString("HH:mm:ss.fff") +
                "  " + message);
        }

        // =====================================
        // APPLICATION SHUTDOWN
        // =====================================

        protected override void OnFormClosing(FormClosingEventArgs e)
        {
            StopSimulation();

            simulationTimer.Stop();
            simulationTimer.Dispose();

            logFlushTimer.Stop();
            logFlushTimer.Dispose();

            link.Dispose();
            toolTip.Dispose();

            base.OnFormClosing(e);
        }
    }
}
