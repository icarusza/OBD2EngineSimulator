using System;
using System.Collections.Generic;

namespace OBD2EngineSimulator
{
    public enum TuneMode
    {
        Normal,
        Lean,
        Rich,
        Bad,
        Misfire
    }

    // Fuel system status codes (PID 03).
    public static class FuelSystemStatus
    {
        public const int OpenLoopCold = 1;
        public const int ClosedLoop = 2;
        public const int OpenLoopDriving = 4;
        public const int OpenLoopFault = 8;
        public const int ClosedLoopFault = 16;
    }

    public sealed class EngineSnapshot
    {
        public Dictionary<string, double> Values { get; } = new Dictionary<string, double>();

        // Fault code -> 0 passed, 1 failed, 2 severe (flashing MIL). Only
        // filled when auto faults are enabled.
        public Dictionary<string, int> Faults { get; } = new Dictionary<string, int>();
    }

    // 60-second drive cycle with values that depend on each other:
    // airflow and load from speed-density, temperatures that warm up,
    // fuel cut on overrun, and optional fuel-trim fault codes.
    public sealed class EngineModel
    {
        public const double DisplacementLitres = 2.0;
        public const double Stoichiometric = 14.7;

        private const double AirDensityStp = 1.184;      // kg/m³ at 25 °C, 101.3 kPa
        private const double GasConstantAir = 287.05;    // J/(kg·K)
        private const double FuelDensity = 740;          // g/L
        private const double TankLitres = 50;

        private double ambient;
        private double baro;
        private double coolant;
        private double oil;
        private double intake;
        private double fuelLevel;
        private double lastElapsed;

        private double leanSeconds;
        private double richSeconds;
        private double badTuneSeconds;

        public void Reset(double coolantC, double oilC, double intakeC, double fuelPercent, double ambientC, double baroKpa)
        {
            coolant = coolantC;
            oil = oilC;
            intake = intakeC;
            fuelLevel = fuelPercent;
            ambient = ambientC;
            baro = baroKpa;
            lastElapsed = 0;
            ResetFaultTimers();
        }

        // After DTCs are cleared, faults must be detected again from scratch.
        public void ResetFaultTimers()
        {
            leanSeconds = 0;
            richSeconds = 0;
            badTuneSeconds = 0;
            leanActive = false;
            richActive = false;
        }

        // =====================================
        // AIRFLOW (speed-density)
        // =====================================

        public static double VolumetricEfficiency(double rpm)
        {
            double x = Math.Max(0, Math.Min(1, rpm / 9000.0));
            return 0.75 + 0.15 * Math.Sin(Math.PI * x);
        }

        // MAF in g/s and calculated load (PID 04) in %.
        public static (double Maf, double Load) ComputeAirflow(double rpm, double mapKpa, double intakeC)
        {
            double displacement = DisplacementLitres / 1000.0;           // m³
            double intakesPerSecond = Math.Max(0, rpm) / 120.0;          // 4-stroke
            double density = mapKpa * 1000.0 / (GasConstantAir * (intakeC + 273.15));

            double maf = intakesPerSecond * displacement * VolumetricEfficiency(rpm) * density * 1000.0;

            // Load = actual airflow / theoretical airflow at STP and WOT.
            double peak = intakesPerSecond * displacement * AirDensityStp * 1000.0;
            double load = peak > 0 ? maf / peak * 100.0 : 0;

            return (Math.Min(254, maf), Math.Max(0, Math.Min(100, load)));
        }

        // =====================================
        // SIMULATION STEP
        // =====================================

        private static double SmoothStep(double value)
        {
            value = Math.Max(0, Math.Min(1, value));
            return value * value * (3.0 - 2.0 * value);
        }

        private static double Lerp(double start, double end, double amount) =>
            start + (end - start) * amount;

        private static double Approach(double current, double target, double seconds, double timeConstant) =>
            current + (target - current) * (1 - Math.Exp(-seconds / timeConstant));

        public EngineSnapshot Step(double elapsedSeconds, TuneMode mode, bool autoDtcs)
        {
            double dt = Math.Max(0, elapsedSeconds - lastElapsed);
            lastElapsed = elapsedSeconds;

            double t = elapsedSeconds % 60.0;

            double rpm, speed, tps, map, timing, commandedAfr;
            bool overrun = false;

            if (t < 8)
            {
                // Idle
                rpm = 850 + 20 * Math.Sin(t * 1.2);
                speed = 0;
                tps = 2 + Math.Sin(t) * 0.5;
                map = 35 + Math.Sin(t * 0.8) * 2;
                timing = 18 + Math.Sin(t * 0.5) * 2;
                commandedAfr = Stoichiometric;
            }
            else if (t < 20)
            {
                // Acceleration
                double p = SmoothStep((t - 8) / 12.0);
                rpm = Lerp(850, 6500, p);
                speed = Lerp(0, 145, p);
                tps = Lerp(5, 85, p);
                map = Lerp(35, 160, p);
                timing = Lerp(18, 5, p);
                commandedAfr = Lerp(Stoichiometric, 12.5, p);
            }
            else if (t < 22)
            {
                // Gear change
                double p = SmoothStep((t - 20) / 2.0);
                rpm = Lerp(6500, 4300, p);
                speed = Lerp(145, 155, p);
                tps = Lerp(85, 45, p);
                map = Lerp(160, 85, p);
                timing = Lerp(5, 12, p);
                commandedAfr = Lerp(12.5, 13.2, p);
            }
            else if (t < 38)
            {
                // Cruise
                double wave = Math.Sin((t - 22) * 0.5);
                rpm = 3000 + wave * 180;
                speed = 110 + wave * 3;
                tps = 22 + wave * 2;
                map = 55 + wave * 4;
                timing = 18 + wave * 2;
                commandedAfr = Stoichiometric;
            }
            else if (t < 50)
            {
                // Deceleration: throttle closes quickly, fuel is cut while
                // the engine is driven above 1200 rpm.
                double p = SmoothStep((t - 38) / 12.0);
                rpm = Lerp(3000, 850, p);
                speed = Lerp(110, 0, p);
                tps = Lerp(15, 0, SmoothStep((t - 38) / 2.0));
                map = Lerp(45, 28, p);
                timing = Lerp(20, 12, p);
                commandedAfr = Stoichiometric;
                overrun = tps < 1 && rpm > 1200;
            }
            else
            {
                // Idle
                double wave = Math.Sin((t - 50) * 0.8);
                rpm = 850 + wave * 15;
                speed = 0;
                tps = 1.5;
                map = 34 + wave * 1.5;
                timing = 18 + wave;
                commandedAfr = Stoichiometric;
            }

            // Temperatures
            coolant = Approach(coolant, 90, dt, 60);
            oil = Approach(oil, coolant > 60 ? coolant + 5 : coolant, dt, 150);
            double intakeTarget = ambient + 12 - Math.Min(speed, 120) / 120.0 * 8;
            intake = Approach(intake, intakeTarget, dt, 20);

            (double maf, double load) = ComputeAirflow(rpm, map, intake);

            // Fuel system status
            int fuelSystem;

            if (overrun || tps > 80)
                fuelSystem = FuelSystemStatus.OpenLoopDriving;
            else if (coolant < 40)
                fuelSystem = FuelSystemStatus.OpenLoopCold;
            else
                fuelSystem = FuelSystemStatus.ClosedLoop;

            // Tune modes
            double slowWave = Math.Sin(t * 0.65);
            double fastWave = Math.Sin(t * 1.7);
            double stft;
            double ltft;
            double measuredAfr;

            switch (mode)
            {
                case TuneMode.Lean:
                    stft = 25 + slowWave * 8;
                    ltft = 12 + Math.Sin(t * 0.15) * 2;
                    measuredAfr = commandedAfr + 1.8 + fastWave * 0.35;
                    break;

                case TuneMode.Rich:
                    stft = -25 + slowWave * 8;
                    ltft = -12 + Math.Sin(t * 0.15) * 2;
                    measuredAfr = commandedAfr - 1.8 + fastWave * 0.35;
                    break;

                case TuneMode.Bad:
                    stft = 30 * Math.Sin(t * 0.65);
                    ltft = 18 * Math.Sin(t * 0.22);
                    measuredAfr = commandedAfr + 2.2 * Math.Sin(t * 0.65) + 0.8 * Math.Sin(t * 1.7);
                    break;

                default:
                    stft = 1.5 + slowWave * 2;
                    ltft = -1.5 + Math.Sin(t * 0.15);
                    measuredAfr = commandedAfr + fastWave * 0.08;
                    break;
            }

            // Short-term trim only works in closed loop.
            if (fuelSystem != FuelSystemStatus.ClosedLoop)
                stft = 0;

            if (overrun)
                measuredAfr = 20;  // no fuel: O2 sensor reads full lean

            stft = Math.Max(-100, Math.Min(99.2, stft));
            ltft = Math.Max(-100, Math.Min(99.2, ltft));
            measuredAfr = Math.Max(8, Math.Min(20, measuredAfr));

            // O2 sensor voltage: high when rich, low when lean.
            double lambda = measuredAfr / Stoichiometric;
            double o2Voltage = Math.Max(0.02, Math.Min(0.95, 0.45 + 0.4 * Math.Tanh((1 - lambda) * 20)));

            // Fuel used
            if (!overrun)
            {
                double fuelGramsPerSecond = maf / measuredAfr;
                fuelLevel -= fuelGramsPerSecond * dt / FuelDensity / TankLitres * 100.0;
                fuelLevel = Math.Max(5, fuelLevel);
            }

            double battery = rpm >= 300 ? 14.2 + 0.05 * Math.Sin(t * 3) : 12.4;

            // Severe misfire: one cylinder misses, the engine runs rough and the
            // unburnt oxygen makes the O2 sensor read lean.
            if (mode == TuneMode.Misfire && !overrun)
            {
                rpm += 35 * Math.Sin(t * 9);
                measuredAfr = Math.Min(20, measuredAfr + 1.2);
                o2Voltage = Math.Max(0.02, o2Voltage - 0.25);
            }

            // Bank 2 runs slightly richer than bank 1.
            double stft2 = fuelSystem == FuelSystemStatus.ClosedLoop ? stft - 0.6 + Math.Sin(t * 0.9) : 0;
            double ltft2 = ltft + 0.8;

            // Direct injection: rail pressure rises with load (4 to 15 MPa).
            double railPressure = rpm >= 300 ? 4000 + load * 110 : 0;

            EngineSnapshot snapshot = new EngineSnapshot();
            Dictionary<string, double> v = snapshot.Values;

            v["RPM"] = rpm;
            v["MAP"] = map;
            v["STFT"] = stft;
            v["LTFT"] = ltft;
            v["AFR"] = commandedAfr;
            v["MEASURED_AFR"] = measuredAfr;
            v["ECT"] = coolant;
            v["IAT"] = intake;
            v["TPS"] = tps;
            v["SPEED"] = speed;
            v["LOAD"] = load;
            v["MAF"] = maf;
            v["TIMING"] = timing;
            v["FUEL"] = fuelLevel;
            v["BARO"] = baro;
            v["VBATT"] = battery;
            v["AMBIENT"] = ambient;
            v["OIL"] = oil;
            v["O2V"] = o2Voltage;
            v["FUELSYS"] = fuelSystem;
            v["STFT2"] = Math.Max(-100, Math.Min(99.2, stft2));
            v["LTFT2"] = Math.Max(-100, Math.Min(99.2, ltft2));
            v["RAILP"] = railPressure;
            v["GEAR"] = GearForSpeed(speed);

            if (autoDtcs)
                UpdateFaults(snapshot, dt, mode, stft + ltft, fuelSystem);

            return snapshot;
        }

        private static int GearForSpeed(double speedKmh)
        {
            double[] upshiftAt = { 0.5, 15, 30, 50, 70, 95, 125 };
            int gear = 0;

            foreach (double threshold in upshiftAt)
            {
                if (speedKmh >= threshold)
                    gear++;
            }

            return gear;
        }

        // =====================================
        // AUTOMATIC FAULTS
        // =====================================
        // The model only reports whether a fault is present (FAULT command);
        // the firmware turns that into pending/confirmed DTCs over drive
        // cycles, like a real ECU.
        //   P0171 / P0172: total fuel trim beyond +/-20 % for 5 s in closed loop
        //                  (cleared after 5 s back within limits).
        //   P0300: random misfire (Bad Tune), severity 1.
        //   P0302: cylinder 2 misfire (Misfire), severity 2 = flashing MIL.

        public const double FaultDebounceSeconds = 5;

        private bool leanActive;
        private bool richActive;

        private static bool Debounce(bool condition, bool active, ref double seconds, double dt)
        {
            seconds = condition != active ? seconds + dt : 0;

            if (seconds >= FaultDebounceSeconds)
            {
                seconds = 0;
                return condition;
            }

            return active;
        }

        private void UpdateFaults(EngineSnapshot snapshot, double dt, TuneMode mode, double totalTrim, int fuelSystem)
        {
            // Trim monitoring only runs in closed loop; open-loop phases
            // (wide-open throttle, overrun) pause it.
            if (fuelSystem == FuelSystemStatus.ClosedLoop)
            {
                leanActive = Debounce(totalTrim > 20, leanActive, ref leanSeconds, dt);
                richActive = Debounce(totalTrim < -20, richActive, ref richSeconds, dt);
            }

            badTuneSeconds += dt;

            snapshot.Faults["P0171"] = leanActive ? 1 : 0;
            snapshot.Faults["P0172"] = richActive ? 1 : 0;
            snapshot.Faults["P0300"] = mode == TuneMode.Bad && badTuneSeconds >= FaultDebounceSeconds ? 1 : 0;
            snapshot.Faults["P0302"] = mode == TuneMode.Misfire && badTuneSeconds >= FaultDebounceSeconds ? 2 : 0;
        }
    }
}
