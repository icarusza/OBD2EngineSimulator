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
            link.RegisterItem("VIN", false, 0, false, DefaultVin);
            link.RegisterItem("CALID", false, 0, false, DefaultCalibrationId);
            link.RegisterItem("ECUNAME", false, 0, false, DefaultEcuName);

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

            chkAutoDtc = new CheckBox();
            chkAutoDtc.Text = "Auto DTCs";
            chkAutoDtc.AutoSize = true;
            chkAutoDtc.Margin = new Padding(16, 7, 0, 0);
            toolTip.SetToolTip(chkAutoDtc,
                "During simulation, fuel trims beyond ±20 % set P0171/P0172 (pending after 10 s, stored after 30 s). " +
                "Bad tune also sets P0300.");

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
                    numeric.Value = (decimal)slider.Value / info.Scale;
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
            layout.RowCount = 7;

            for (int i = 0; i < 3; i++)
                layout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 33.3f));

            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 40));   // MIL + freeze frame
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 24));   // list headers
            layout.RowStyles.Add(new RowStyle(SizeType.Percent, 100));   // lists
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 40));   // add / remove
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 26));   // DTC sync status
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 70));   // identity fields
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 40));   // identity apply

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

            txtVin = AddIdentityField(layout, 0, "VIN (17 characters, Mode 09 / 02)", DefaultVin, 17);
            txtCalId = AddIdentityField(layout, 1, "CALIBRATION ID (max 16, Mode 09 / 04)", DefaultCalibrationId, 16);
            txtEcuName = AddIdentityField(layout, 2, "ECU NAME (max 20, Mode 09 / 0A)", DefaultEcuName, 20);
            txtVin.CharacterCasing = CharacterCasing.Upper;

            FlowLayoutPanel identityPanel = new FlowLayoutPanel();
            identityPanel.Dock = DockStyle.Fill;
            identityPanel.WrapContents = false;

            Button btnApplyIdentity = new Button();
            btnApplyIdentity.Text = "Apply vehicle information";
            btnApplyIdentity.AutoSize = true;
            btnApplyIdentity.Click += (sender, e) => ApplyIdentity();

            lblIdentityStatus = new Label();
            lblIdentityStatus.AutoSize = true;
            lblIdentityStatus.Margin = new Padding(12, 8, 0, 0);

            identityPanel.Controls.Add(btnApplyIdentity);
            identityPanel.Controls.Add(lblIdentityStatus);

            layout.Controls.Add(identityPanel, 0, 6);
            layout.SetColumnSpan(identityPanel, 3);

            tabParameters.TabPages.Add(page);
            cmbPID.Items.Add("Diagnostics (DTCs, MIL, VIN)");
        }

        private TextBox AddIdentityField(TableLayoutPanel layout, int column, string title, string value, int maxLength)
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

            layout.Controls.Add(field, column, 5);
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

            if (!Regex.IsMatch(vin, "^[A-HJ-NPR-Z0-9]{17}$"))
            {
                MessageBox.Show("The VIN must be 17 characters: digits and capital letters except I, O and Q.",
                    "OBD-II Simulator", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }

            if (!IsValidText(calId, 16) || !IsValidText(ecuName, 20))
            {
                MessageBox.Show("Calibration ID (max 16) and ECU name (max 20) must be printable ASCII " +
                    "without ; # | or =.", "OBD-II Simulator", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }

            link.SetDesired("VIN", vin);
            link.SetDesired("CALID", calId);
            link.SetDesired("ECUNAME", ecuName);
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

            Log("Driving simulation stopped. Manual controls restored.");
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

            foreach (string code in snapshot.PendingDtcs)
            {
                if (AddDtc(1, code))
                    Log("Simulation set pending DTC " + code + ".");
            }

            foreach (string code in snapshot.StoredDtcs)
            {
                if (AddDtc(0, code))
                    Log("Simulation set stored DTC " + code + " (MIL on).");
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
                    UpdateIdentityStatus();
                    break;

                case "DTC_S":
                case "DTC_P":
                case "DTC_X":
                    UpdateDtcStatus();
                    break;
            }
        }

        // The Arduino's DTC lists changed outside the app (e.g. Mode 04 from
        // a scan tool): show what the ECU now has.
        private void Link_ItemAdopted(LinkItem item)
        {
            int index = Array.IndexOf(DtcKeys, item.Key);

            if (index < 0)
                return;

            dtcCodes[index].Clear();
            dtcCodes[index].AddRange(item.Desired.Split(',', StringSplitOptions.RemoveEmptyEntries));
            RefreshDtcListBox(index);
            UpdateDtcStatus();

            // A cleared fault must be detected again from scratch, not re-set
            // on the next simulation tick.
            engineModel.ResetFaultTimers();
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
                    bool milOn = value == "1";
                    lblMil.Text = milOn ? "MIL ON" : "MIL OFF";
                    lblMil.ForeColor = milOn ? Color.DarkOrange : Color.FromArgb(35, 125, 75);
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

            lblIdentityStatus.Text =
                "VIN " + StatusSymbol(vin) +
                "   Calibration ID " + StatusSymbol(calId) +
                "   ECU name " + StatusSymbol(ecuName);

            bool allConfirmed = vin.Status == ValueStatus.Confirmed &&
                calId.Status == ValueStatus.Confirmed &&
                ecuName.Status == ValueStatus.Confirmed;

            lblIdentityStatus.ForeColor = allConfirmed
                ? StatusColor(ValueStatus.Confirmed)
                : Color.Gray;
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
            UpdateHealthLabel();

            if (!link.IsOpen)
            {
                lblMil.Text = "MIL: unknown";
                lblMil.ForeColor = Color.Gray;
                lblFreezeFrame.Text = "Freeze frame: unknown";
            }
        }

        // =====================================
        // BATCHED LOG DISPLAY
        // =====================================

        private void LogFlushTimer_Tick(object? sender, EventArgs e)
        {
            UpdateHealthLabel();

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
