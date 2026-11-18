using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Text;

namespace OBD2EngineSimulator
{
    public enum ValueStatus
    {
        Pending,    // sent or queued, not yet echoed by the Arduino
        Confirmed,  // the Arduino echoed the desired value
        Clamped,    // the Arduino applied a different value (range limit)
        Failed      // rejected by the Arduino, or no reply after retries
    }

    // One value or setting kept in sync with the Arduino.
    public sealed class LinkItem
    {
        internal LinkItem(string key, bool isNumeric, decimal tolerance, bool deviceOwned, string desired)
        {
            Key = key;
            IsNumeric = isNumeric;
            Tolerance = tolerance;
            DeviceOwned = deviceOwned;
            Desired = desired;
        }

        public string Key { get; }
        public bool IsNumeric { get; }
        public decimal Tolerance { get; }

        // Stored on the Arduino (EEPROM): DTC lists, vehicle information, TCM.
        // On connect and on read-back the app takes the Arduino's value
        // instead of sending its own; only user edits are sent. Otherwise
        // connecting would overwrite persisted DTCs, and a scan tool's clear
        // would be undone.
        public bool DeviceOwned { get; }

        public string Desired { get; internal set; }
        public string? Confirmed { get; internal set; }
        public ValueStatus Status { get; internal set; } = ValueStatus.Pending;
        public string? FailureReason { get; internal set; }

        internal int Attempts;

        // The Arduino answered ERR for this value: do not resend it on
        // read-back until the user changes it.
        internal bool Rejected;
    }

    // Serial link to the Arduino firmware with two-way confirmation:
    // every line carries a sequence number, the Arduino echoes the values it
    // applied, unconfirmed values are retried, and a periodic read-back
    // (INFO, GET, DTC?) detects divergence and Arduino resets.
    public sealed class ArduinoLink : IDisposable
    {
        private const int PumpIntervalMs = 25;
        private const int FlushIntervalMs = 50;
        // One line at a time: the UNO has a 64-byte receive buffer and blocks
        // while printing a ~200-character echo, so a second line sent during
        // that time could overflow it. A full update (two lines) still
        // reaches the Arduino about 10 times per second.
        private const int MaxInFlight = 1;
        private const int ReplyTimeoutMs = 300;
        private const int MaxRetries = 3;
        private const int ReadBackIntervalMs = 2000;
        private const int HandshakeQueryDelayMs = 300;
        private const int HandshakeRetryMs = 1000;
        private const int HandshakeTimeoutMs = 4000;
        private const int MaxLineLength = 190;  // firmware buffer is 200

        private enum LinkState { Closed, WaitingForDevice, Ready }

        private sealed class InFlightLine
        {
            public ushort Sequence;
            public List<string> Keys = new List<string>();
            public string? Query;
            public long SentAtMs;
        }

        private readonly Dictionary<string, LinkItem> items = new Dictionary<string, LinkItem>();
        private readonly List<string> itemOrder = new List<string>();
        private readonly HashSet<string> dirty = new HashSet<string>();
        private readonly Queue<string> pendingQueries = new Queue<string>();
        private readonly Queue<string> pendingCommands = new Queue<string>();
        private readonly Dictionary<ushort, InFlightLine> inFlight = new Dictionary<ushort, InFlightLine>();

        // Recently timed-out lines, so a late reply is still interpreted
        // correctly (a late GET is a read-back, not a command echo).
        private readonly Dictionary<ushort, InFlightLine> expired = new Dictionary<ushort, InFlightLine>();
        private readonly Queue<ushort> expiredOrder = new Queue<ushort>();
        private const int MaxExpired = 16;

        private readonly ConcurrentQueue<string> receivedLines = new ConcurrentQueue<string>();
        private readonly StringBuilder receiveBuffer = new StringBuilder();
        private readonly Stopwatch clock = Stopwatch.StartNew();
        private readonly System.Windows.Forms.Timer pumpTimer;

        private SerialPort? port;
        private LinkState state = LinkState.Closed;
        private ushort nextSequence = 1;

        private long stateSinceMs;
        private long lastFlushMs;
        private long lastReadBackMs;
        private long lastHandshakeQueryMs;
        private long? lastReplyAtMs;
        private long? lastUptimeMs;

        public ArduinoLink()
        {
            pumpTimer = new System.Windows.Forms.Timer();
            pumpTimer.Interval = PumpIntervalMs;
            pumpTimer.Tick += (sender, e) => Pump();
        }

        // message, isTraffic (TX/OK lines, only shown when traffic logging is on)
        public event Action<string, bool>? LogMessage;
        public event Action<LinkItem>? ItemChanged;
        public event Action<LinkItem>? ItemAdopted;
        public event Action<string, string>? DeviceStatus;
        public event Action? ConnectionLost;

        public bool IsOpen => port != null && port.IsOpen;
        public bool IsReady => state == LinkState.Ready;

        // Link health.
        public bool Responding { get; private set; }
        public int LastRoundTripMs { get; private set; }
        public int RetryCount { get; private set; }
        public int FailureCount { get; private set; }

        public long? MillisecondsSinceLastReply =>
            lastReplyAtMs.HasValue ? clock.ElapsedMilliseconds - lastReplyAtMs.Value : (long?)null;

        public LinkItem GetItem(string key) => items[key];

        // =====================================
        // ITEMS
        // =====================================

        public void RegisterItem(string key, bool isNumeric, decimal tolerance, bool deviceOwned, string initialValue)
        {
            items[key] = new LinkItem(key, isNumeric, tolerance, deviceOwned, Normalize(key, initialValue));
            itemOrder.Add(key);

            if (!deviceOwned)
                dirty.Add(key);
        }

        public void SetParameter(string key, decimal value)
        {
            SetDesired(key, value.ToString("0.######", CultureInfo.InvariantCulture));
        }

        public void SetDesired(string key, string value)
        {
            LinkItem item = items[key];
            value = Normalize(key, value);

            if (item.Desired == value && item.Status != ValueStatus.Failed)
                return;

            item.Desired = value;
            item.Status = ValueStatus.Pending;
            item.FailureReason = null;
            item.Attempts = 0;
            item.Rejected = false;
            dirty.Add(key);

            ItemChanged?.Invoke(item);
        }

        private static bool IsDtcKey(string key) => key.StartsWith("DTC_", StringComparison.Ordinal);

        private static string Normalize(string key, string value)
        {
            if (!IsDtcKey(key))
                return value.Trim();

            return string.Join(",", value
                .Split(',', StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries)
                .Select(code => code.ToUpperInvariant())
                .Distinct()
                .OrderBy(code => code, StringComparer.Ordinal));
        }

        private static bool Matches(LinkItem item, string value)
        {
            if (item.IsNumeric)
            {
                return decimal.TryParse(value, NumberStyles.Float, CultureInfo.InvariantCulture, out decimal actual) &&
                       decimal.TryParse(item.Desired, NumberStyles.Float, CultureInfo.InvariantCulture, out decimal desired) &&
                       Math.Abs(actual - desired) <= item.Tolerance;
            }

            return string.Equals(Normalize(item.Key, value), item.Desired, StringComparison.Ordinal);
        }

        // Queues every app-owned item for sending; device-owned items wait for
        // the read-back that adopts the Arduino's stored value.
        private void MarkAllDirty()
        {
            foreach (string key in itemOrder)
            {
                LinkItem item = items[key];
                item.Status = ValueStatus.Pending;
                item.FailureReason = null;
                item.Attempts = 0;
                item.Rejected = false;

                if (item.DeviceOwned)
                    dirty.Remove(key);
                else
                    dirty.Add(key);

                ItemChanged?.Invoke(item);
            }
        }

        // =====================================
        // OPEN / CLOSE
        // =====================================

        public void Open(string portName, int baudRate)
        {
            Close();

            SerialPort newPort = new SerialPort(portName, baudRate, Parity.None, 8, StopBits.One);
            newPort.NewLine = "\n";
            newPort.WriteTimeout = 250;
            newPort.DataReceived += Port_DataReceived;

            try
            {
                newPort.Open();
            }
            catch
            {
                newPort.DataReceived -= Port_DataReceived;
                newPort.Dispose();
                throw;
            }

            port = newPort;

            lock (receiveBuffer)
                receiveBuffer.Clear();

            while (receivedLines.TryDequeue(out _)) { }

            inFlight.Clear();
            ClearExpired();
            pendingQueries.Clear();
            pendingCommands.Clear();
            lastUptimeMs = null;
            lastReplyAtMs = null;
            Responding = false;
            RetryCount = 0;
            FailureCount = 0;
            LastRoundTripMs = 0;

            SetState(LinkState.WaitingForDevice);
            lastHandshakeQueryMs = clock.ElapsedMilliseconds - HandshakeRetryMs + HandshakeQueryDelayMs;

            MarkAllDirty();
            pumpTimer.Start();

            Log("Waiting for the Arduino (READY or INFO reply)...");
        }

        public void Close()
        {
            pumpTimer.Stop();

            SerialPort? oldPort = port;
            port = null;

            if (oldPort != null)
            {
                oldPort.DataReceived -= Port_DataReceived;

                try
                {
                    if (oldPort.IsOpen)
                        oldPort.Close();
                }
                catch (Exception ex)
                {
                    Log("Error closing port: " + ex.Message);
                }

                oldPort.Dispose();
            }

            inFlight.Clear();
            ClearExpired();
            pendingQueries.Clear();
            pendingCommands.Clear();
            SetState(LinkState.Closed);
            Responding = false;
        }

        public void Dispose()
        {
            Close();
            pumpTimer.Dispose();
        }

        private void SetState(LinkState newState)
        {
            state = newState;
            stateSinceMs = clock.ElapsedMilliseconds;
        }

        private void HandleConnectionLost(string reason)
        {
            Log("Connection lost: " + reason);
            Close();
            ConnectionLost?.Invoke();
        }

        // =====================================
        // RECEIVE (serial thread)
        // =====================================

        private void Port_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            try
            {
                if (sender is not SerialPort source || !source.IsOpen)
                    return;

                string data = source.ReadExisting();

                lock (receiveBuffer)
                {
                    receiveBuffer.Append(data);

                    while (true)
                    {
                        string text = receiveBuffer.ToString();
                        int newline = text.IndexOf('\n');

                        if (newline < 0)
                            break;

                        string line = text.Substring(0, newline).Trim();
                        receiveBuffer.Remove(0, newline + 1);

                        if (line.Length > 0)
                            receivedLines.Enqueue(line);
                    }
                }
            }
            catch (Exception ex) when (ex is InvalidOperationException || ex is IOException || ex is TimeoutException)
            {
                // Port closing; the pump detects a closed port.
            }
        }

        // =====================================
        // PUMP (UI thread)
        // =====================================

        private void Pump()
        {
            if (port == null)
                return;

            if (!port.IsOpen)
            {
                HandleConnectionLost("port closed");
                return;
            }

            int handled = 0;

            while (handled++ < 200 && receivedLines.TryDequeue(out string? line))
            {
                HandleLine(line);

                if (port == null)
                    return;
            }

            long now = clock.ElapsedMilliseconds;

            CheckTimeouts(now);

            if (state == LinkState.WaitingForDevice)
            {
                if (now - lastHandshakeQueryMs >= HandshakeRetryMs &&
                    !inFlight.Values.Any(l => l.Query == "INFO"))
                {
                    lastHandshakeQueryMs = now;
                    SendQuery("INFO");
                }

                if (now - stateSinceMs >= HandshakeTimeoutMs)
                {
                    Log("WARNING: no READY or INFO reply from the Arduino. " +
                        "Check the port and that firmware 2.x is flashed. Sending values anyway.");
                    EnterReady();
                }

                return;
            }

            if (state != LinkState.Ready)
                return;

            if (now - lastReadBackMs >= ReadBackIntervalMs)
            {
                lastReadBackMs = now;
                QueueQuery("INFO");
                QueueQuery("GET");
                QueueQuery("DTC?");
            }

            if (now - lastFlushMs >= FlushIntervalMs)
            {
                lastFlushMs = now;
                Flush(now);
            }
        }

        private void EnterReady()
        {
            SetState(LinkState.Ready);
            lastReadBackMs = clock.ElapsedMilliseconds;
            MarkAllDirty();

            // Read the Arduino's stored DTCs and vehicle information first.
            QueueQuery("INFO");
            QueueQuery("DTC?");

            Log("Arduino link ready. Reading stored DTCs and vehicle information, sending settings and values.");
        }

        private void Resync(string reason)
        {
            Log("Arduino restart detected (" + reason + "). Restoring settings and values; DTCs and vehicle information come from its EEPROM.");
            inFlight.Clear();
            ClearExpired();
            pendingQueries.Clear();
            pendingCommands.Clear();
            lastUptimeMs = null;
            MarkAllDirty();
            QueueQuery("INFO");
            QueueQuery("DTC?");
        }

        // =====================================
        // SEND
        // =====================================

        private ushort NextSequence()
        {
            ushort sequence = nextSequence;
            nextSequence = nextSequence == ushort.MaxValue ? (ushort)1 : (ushort)(nextSequence + 1);
            return sequence;
        }

        // One-off command (e.g. DTC:CLEAR) whose effect is confirmed through
        // the items it changes; not retried on timeout. Ignored when closed,
        // since all items are re-sent on the next connect anyway.
        public void SendCommand(string command)
        {
            if (port != null)
                pendingCommands.Enqueue(command);
        }

        private void QueueQuery(string query)
        {
            if (pendingQueries.Contains(query) || inFlight.Values.Any(l => l.Query == query))
                return;

            pendingQueries.Enqueue(query);
        }

        private string BuildCommand(LinkItem item)
        {
            if (IsDtcKey(item.Key))
                return "DTC:SET:" + item.Key.Substring(4) + ":" + item.Desired;

            return item.Key + ":" + item.Desired;
        }

        private void Flush(long now)
        {
            while (port != null && inFlight.Count < MaxInFlight)
            {
                if (pendingQueries.Count > 0)
                {
                    SendQuery(pendingQueries.Dequeue());
                    continue;
                }

                if (pendingCommands.Count > 0)
                {
                    ushort commandSequence = NextSequence();
                    SendLine(new InFlightLine { Sequence = commandSequence },
                        "#" + commandSequence + " " + pendingCommands.Dequeue());
                    continue;
                }

                if (dirty.Count == 0)
                    break;

                ushort sequence = NextSequence();
                string prefix = "#" + sequence + " ";
                StringBuilder body = new StringBuilder();
                List<string> keys = new List<string>();

                foreach (string key in itemOrder)
                {
                    if (!dirty.Contains(key))
                        continue;

                    string command = BuildCommand(items[key]);

                    if (keys.Count > 0 && prefix.Length + body.Length + 1 + command.Length > MaxLineLength)
                        break;

                    if (body.Length > 0)
                        body.Append(';');

                    body.Append(command);
                    keys.Add(key);
                }

                foreach (string key in keys)
                    dirty.Remove(key);

                SendLine(new InFlightLine { Sequence = sequence, Keys = keys }, prefix + body);
            }
        }

        private void SendQuery(string query)
        {
            ushort sequence = NextSequence();
            SendLine(new InFlightLine { Sequence = sequence, Query = query }, "#" + sequence + " " + query);
        }

        private void SendLine(InFlightLine record, string text)
        {
            if (port == null)
                return;

            record.SentAtMs = clock.ElapsedMilliseconds;
            inFlight[record.Sequence] = record;

            try
            {
                port.Write(text + "\n");
                LogMessage?.Invoke("TX: " + text, true);
            }
            catch (TimeoutException)
            {
                // Stays in flight; the reply timeout retries it.
            }
            catch (Exception ex) when (ex is IOException || ex is InvalidOperationException || ex is UnauthorizedAccessException)
            {
                HandleConnectionLost(ex.Message);
            }
        }

        // =====================================
        // TIMEOUTS AND RETRIES
        // =====================================

        private void CheckTimeouts(long now)
        {
            foreach (InFlightLine record in inFlight.Values.ToList())
            {
                if (now - record.SentAtMs < ReplyTimeoutMs)
                    continue;

                inFlight.Remove(record.Sequence);
                RememberExpired(record);

                // Unanswered handshake queries while the Arduino boots are expected.
                if (state == LinkState.Ready)
                {
                    Responding = false;
                    RetryCount++;
                }

                RetryLine(record, "no reply");
            }
        }

        private void ClearExpired()
        {
            expired.Clear();
            expiredOrder.Clear();
        }

        private void RememberExpired(InFlightLine record)
        {
            expired[record.Sequence] = record;
            expiredOrder.Enqueue(record.Sequence);

            while (expiredOrder.Count > MaxExpired)
                expired.Remove(expiredOrder.Dequeue());
        }

        private void RetryLine(InFlightLine record, string reason)
        {
            foreach (string key in record.Keys)
            {
                LinkItem item = items[key];

                if (dirty.Contains(key) || item.Status == ValueStatus.Failed)
                    continue;

                item.Attempts++;

                if (item.Attempts > MaxRetries)
                {
                    item.Status = ValueStatus.Failed;
                    item.FailureReason = reason;
                    FailureCount++;
                    Log("No confirmation for " + key + " after " + MaxRetries + " retries (" + reason + ").");
                }
                else
                {
                    dirty.Add(key);
                }

                ItemChanged?.Invoke(item);
            }
        }

        // =====================================
        // REPLIES
        // =====================================

        private static void ParseHeader(string rest, out ushort? sequence, out string payload)
        {
            sequence = null;
            rest = rest.Trim();

            if (rest.StartsWith("#", StringComparison.Ordinal))
            {
                int space = rest.IndexOf(' ');
                string number = space < 0 ? rest.Substring(1) : rest.Substring(1, space - 1);

                if (ushort.TryParse(number, NumberStyles.None, CultureInfo.InvariantCulture, out ushort parsed))
                    sequence = parsed;

                rest = space < 0 ? "" : rest.Substring(space + 1).Trim();
            }

            payload = rest;
        }

        private static bool IsReply(string line, string keyword) =>
            line == keyword || line.StartsWith(keyword + " ", StringComparison.Ordinal);

        private void HandleLine(string line)
        {
            if (line == "READY")
            {
                if (state == LinkState.WaitingForDevice)
                {
                    Log("Arduino reported READY.");
                    EnterReady();
                }
                else if (state == LinkState.Ready)
                {
                    Resync("READY received");
                }

                return;
            }

            if (IsReply(line, "OK"))
            {
                HandleOk(line);
                return;
            }

            if (IsReply(line, "ERR"))
            {
                HandleError(line);
                return;
            }

            LogMessage?.Invoke("ARDUINO: " + line, false);
        }

        private void NoteReply()
        {
            lastReplyAtMs = clock.ElapsedMilliseconds;
            Responding = true;
        }

        private void HandleOk(string line)
        {
            LogMessage?.Invoke("RX: " + line, true);
            ParseHeader(line.Substring(2), out ushort? sequence, out string payload);

            InFlightLine? record = null;

            if (sequence.HasValue && inFlight.Remove(sequence.Value, out InFlightLine? found))
            {
                record = found;
                LastRoundTripMs = (int)(clock.ElapsedMilliseconds - record.SentAtMs);
            }
            else if (sequence.HasValue && expired.Remove(sequence.Value, out InFlightLine? late))
            {
                // Late reply to a timed-out line: its values were already
                // re-queued, so only use it to classify the reply.
                record = late;
            }

            NoteReply();

            // A reply that matches no line we sent (e.g. one sent before a
            // resync) cannot be classified as echo or read-back: ignore it.
            if (record == null)
                return;

            List<KeyValuePair<string, string>> values = new List<KeyValuePair<string, string>>();

            foreach (string part in payload.Split(';', StringSplitOptions.RemoveEmptyEntries))
            {
                int equals = part.IndexOf('=');

                if (equals > 0)
                    values.Add(new KeyValuePair<string, string>(part.Substring(0, equals).Trim(), part.Substring(equals + 1)));
            }

            bool readBack = record?.Query != null;

            if (record?.Query == "INFO")
            {
                string? uptimeText = values.FirstOrDefault(v => v.Key == "UP").Value;

                if (long.TryParse(uptimeText, NumberStyles.None, CultureInfo.InvariantCulture, out long uptime))
                {
                    if (state == LinkState.Ready && lastUptimeMs.HasValue && uptime < lastUptimeMs.Value)
                    {
                        Resync("uptime went back");
                        lastUptimeMs = uptime;
                        return;
                    }

                    lastUptimeMs = uptime;
                }

                if (state == LinkState.WaitingForDevice)
                {
                    string firmware = values.FirstOrDefault(v => v.Key == "FW").Value ?? "?";
                    Log("Arduino firmware " + firmware + " answered.");
                    EnterReady();
                }
            }

            foreach (KeyValuePair<string, string> value in values)
                ApplyEcho(value.Key, value.Value, readBack);
        }

        private void HandleError(string line)
        {
            LogMessage?.Invoke("ARDUINO ERROR: " + line, false);
            ParseHeader(line.Substring(3), out ushort? sequence, out string payload);
            NoteReply();

            string[] parts = payload.Split(' ', 2, StringSplitOptions.RemoveEmptyEntries);
            string reason = parts.Length > 0 ? parts[0] : "ERROR";
            string? name = parts.Length > 1 ? parts[1].Trim() : null;

            InFlightLine? record = null;

            if (sequence.HasValue && !inFlight.TryGetValue(sequence.Value, out record))
            {
                // Late ERR for a timed-out line: its values were re-queued and
                // will be answered again, so do not mark them failed.
                if (expired.ContainsKey(sequence.Value))
                    return;
            }

            if (name == null)
            {
                // The whole line was rejected; no OK follows.
                if (record != null)
                {
                    inFlight.Remove(record.Sequence);
                    RetryLine(record, reason);
                }

                return;
            }

            IEnumerable<string> keys = name == "DTC"
                ? (record?.Keys.Where(IsDtcKey) ?? Enumerable.Empty<string>())
                : (items.ContainsKey(name) ? new[] { name } : Enumerable.Empty<string>());

            foreach (string key in keys.ToList())
            {
                LinkItem item = items[key];
                item.Status = ValueStatus.Failed;
                item.FailureReason = "rejected by Arduino (" + reason + ")";
                item.Rejected = true;
                FailureCount++;
                ItemChanged?.Invoke(item);
            }
        }

        private void ApplyEcho(string name, string value, bool readBack)
        {
            if (name == "DTC")
            {
                // S:P0171,P0300|P:|X:
                foreach (string list in value.Split('|'))
                {
                    int colon = list.IndexOf(':');

                    if (colon != 1)
                        continue;

                    string key = "DTC_" + list[0];

                    if (items.ContainsKey(key))
                        ApplyConfirmed(items[key], list.Substring(2), readBack);
                }

                return;
            }

            if (items.TryGetValue(name, out LinkItem? item))
            {
                ApplyConfirmed(item, value, readBack);
                return;
            }

            DeviceStatus?.Invoke(name, value);
        }

        // Takes the Arduino's value for a device-owned item.
        private void Adopt(LinkItem item, string value)
        {
            bool changed = item.Desired != value;

            item.Desired = value;
            item.Status = ValueStatus.Confirmed;
            item.FailureReason = null;

            if (changed)
                ItemAdopted?.Invoke(item);
        }

        private void ApplyConfirmed(LinkItem item, string value, bool readBack)
        {
            value = Normalize(item.Key, value);
            item.Confirmed = value;

            bool newerValuePending = dirty.Contains(item.Key) ||
                inFlight.Values.Any(l => l.Keys.Contains(item.Key));

            bool matches = Matches(item, value);

            if (matches && !newerValuePending)
                item.Rejected = false;

            if (readBack)
            {
                if (newerValuePending || matches)
                {
                    if (!newerValuePending)
                    {
                        item.Status = ValueStatus.Confirmed;
                        item.FailureReason = null;
                    }
                }
                else if (item.DeviceOwned)
                {
                    // The Arduino's stored value wins, even after a rejected
                    // edit (it may have applied part of it).
                    item.Rejected = false;
                    Adopt(item, value);
                }
                else if (item.Status != ValueStatus.Clamped && !item.Rejected)
                {
                    Log("Read-back mismatch for " + item.Key + ": Arduino has '" + value +
                        "', app wants '" + item.Desired + "'. Resending.");
                    item.Status = ValueStatus.Pending;
                    item.Attempts = 0;
                    dirty.Add(item.Key);
                }
            }
            else
            {
                item.Attempts = 0;

                if (newerValuePending)
                {
                    item.Status = ValueStatus.Pending;
                }
                else if (item.Rejected && !matches)
                {
                    // Rejected by an ERR in the same line: stay Failed.
                }
                else if (!matches && item.DeviceOwned)
                {
                    // The Arduino changed it itself (e.g. a FAULT made a DTC pending).
                    Adopt(item, value);
                }
                else if (matches)
                {
                    item.Status = ValueStatus.Confirmed;
                    item.FailureReason = null;
                }
                else
                {
                    item.Status = ValueStatus.Clamped;
                }
            }

            ItemChanged?.Invoke(item);
        }

        private void Log(string message)
        {
            LogMessage?.Invoke(message, false);
        }
    }
}
