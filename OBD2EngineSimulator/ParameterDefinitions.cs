using System;
using System.Collections.Generic;
using System.Linq;

namespace OBD2EngineSimulator
{
    public sealed class ParameterInfo
    {
        public string Key { get; init; } = "";
        public string Name { get; init; } = "";
        public string Command { get; init; } = "";
        public string PID { get; init; } = "";
        public string Unit { get; init; } = "";
        public decimal Minimum { get; init; }
        public decimal Maximum { get; init; }
        public decimal DefaultValue { get; init; }

        // Must match the decimals in PARAM_DEFS (firmware EngineState.cpp):
        // the firmware rounds to this precision before echoing a value back.
        public int DecimalPlaces { get; init; }

        // Slider steps per unit.
        public int Scale { get; init; }

        // An echoed value counts as equal to the desired value when it is
        // within half of the last displayed digit (firmware rounding).
        public decimal Tolerance =>
            0.5m / (decimal)Math.Pow(10, DecimalPlaces) + 0.000001m;
    }

    public static class ParameterDefinitions
    {
        public static IReadOnlyList<ParameterInfo> All { get; } = new List<ParameterInfo>
        {
            Define("RPM", "Engine RPM", "0C", "RPM", 0, 12000, 850, 0, 1),
            Define("MAP", "Manifold Absolute Pressure", "0B", "kPa", 0, 170, 85, 0, 1),
            Define("STFT", "Short-Term Fuel Trim - Bank 1", "06", "%", -100, 99.2m, 3.1m, 1, 10),
            Define("LTFT", "Long-Term Fuel Trim - Bank 1", "07", "%", -100, 99.2m, -2.3m, 1, 10),
            Define("AFR", "Commanded / Target AFR", "44", "AFR", 8, 20, 14.7m, 1, 10),
            Define("MEASURED_AFR", "Simulated / Measured AFR", "24", "AFR", 8, 20, 14.7m, 1, 10),
            Define("ECT", "Engine Coolant Temperature", "05", "°C", -40, 215, 90, 0, 1),
            Define("IAT", "Intake Air Temperature", "0F", "°C", -40, 215, 25, 0, 1),
            Define("TPS", "Throttle Position", "11", "%", 0, 100, 0, 1, 1),
            Define("SPEED", "Vehicle Speed", "0D", "km/h", 0, 255, 0, 0, 1),
            Define("LOAD", "Calculated Engine Load", "04", "%", 0, 100, 20, 1, 1),
            Define("MAF", "Mass Air Flow", "10", "g/s", 0, 254, 3.5m, 1, 10),
            Define("TIMING", "Ignition Timing Advance", "0E", "°", -64, 63.5m, 10, 1, 10),
            Define("FUEL", "Fuel Tank Level", "2F", "%", 0, 100, 75, 1, 10),
            Define("BARO", "Barometric Pressure", "33", "kPa", 0, 255, 101, 0, 1),
            Define("VBATT", "Control Module Voltage", "42", "V", 0, 65.5m, 14.2m, 2, 100),
            Define("AMBIENT", "Ambient Air Temperature", "46", "°C", -40, 215, 20, 0, 1),
            Define("OIL", "Engine Oil Temperature", "5C", "°C", -40, 215, 90, 0, 1),
            Define("O2V", "O2 Sensor Voltage - B1S1", "24", "V", 0, 5, 0.45m, 3, 1000),
            Define("FUELSYS", "Fuel System Status", "03", "code", 0, 16, 2, 0, 1)
        };

        public static ParameterInfo Get(string key) =>
            All.First(p => p.Key == key);

        private static ParameterInfo Define(
            string key,
            string name,
            string pid,
            string unit,
            decimal min,
            decimal max,
            decimal defaultValue,
            int decimals,
            int scale)
        {
            return new ParameterInfo
            {
                Key = key,
                Name = name,
                Command = key,
                PID = pid,
                Unit = unit,
                Minimum = min,
                Maximum = max,
                DefaultValue = defaultValue,
                DecimalPlaces = decimals,
                Scale = scale
            };
        }
    }
}
