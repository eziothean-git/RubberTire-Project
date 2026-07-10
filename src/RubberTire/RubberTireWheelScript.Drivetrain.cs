using System;
using System.Globalization;
using System.Text;
using UnityEngine;
using Modding;

public partial class RubberTireWheelScript
{
    public float driveTorque = 0f;

    public bool enableDriveBrake = true;
    public bool drivenWheel = true;
    public bool invertDriveTorque = false;
    public bool enableEngineCurve = true;
    public float enginePeakTorque = 450f;
    // Retained for loading old machine records; the AC-style torque LUT replaces
    // the generated constant-power plateau when enableEngineCurve is true.
    public float enginePeakPower = 180000f;
    public float enginePowerHoldRpm = 6000f;
    public float engineIdleRpm = 900f;
    public float engineRedlineRpm = 7500f;
    public float engineLimiterHysteresisRpm = 250f;
    public float engineRpmFilterTau = 0.05f;
    public float engineCoastTorque = 20f;
    public bool smoothEngineTorqueLut = true;
    public float finalDriveRatio = 10f;
    public float drivetrainEfficiency = 0.90f;
    public string engineTorqueLut = DefaultEngineTorqueLut;
    public bool enableGearbox = true;
    public float gearCount = 5f;
    public float gearRatio1 = 4.00f;
    public float gearRatio2 = 2.80f;
    public float gearRatio3 = 1.90f;
    public float gearRatio4 = 1.35f;
    public float gearRatio5 = 1.00f;
    public float gearRatio6 = 0.75f;
    public float gearRatio7 = 0.55f;
    public float gearRatio8 = 0.40f;
    public float reverseGearRatio = 3.50f;

    public bool enableRollingDamping = false;
    public float rollingDampingK = 0.25f;
    public bool useLoadSensitiveRollingResistance = false;
    public float rollingResistanceCoeff = 0.015f;

    public float maxBrakeTorque = 12000f;
    public float brakeDeadbandOmega = 0.5f;
    public float brakeHoldK = 2000f;
    public float throttleRise = 8f;
    public float throttleFall = 10f;
    public float brakeRise = 12f;
    public float brakeFall = 14f;

    private MKey uiKeyThrottle, uiKeyBrake, uiKeyReverse, uiKeyGearUp, uiKeyGearDown;

    private float throttle01;
    private float brake01;
    private int currentGear = 1;
    private bool engineLimiterCut;
    private float currentEngineRpm;
    private float currentEngineRawRpm;
    private float baseAngularDrag;
    private bool baseAngularDragCaptured;

    internal const string DefaultEngineTorqueLut =
        "0|290\n" +
        "1000|340\n" +
        "2000|410\n" +
        "3500|450\n" +
        "5000|430\n" +
        "6000|380\n" +
        "7000|300\n" +
        "7500|0";
    private const int MaximumEngineLutPoints = 32;
    private const float RpmPerRadPerSecond = 9.5492966f;
    private const float RadPerSecondPerRpm = 0.10471976f;
    private readonly float[] engineLutRpm = new float[MaximumEngineLutPoints];
    private readonly float[] engineLutTorque = new float[MaximumEngineLutPoints];
    private readonly float[] engineLutSlope = new float[MaximumEngineLutPoints];
    private readonly float[] engineLutSpan = new float[MaximumEngineLutPoints - 1];
    private readonly float[] engineLutSecant = new float[MaximumEngineLutPoints - 1];
    private int engineLutPointCount;
    private string parsedEngineTorqueLut;

    private void CreateDrivetrainKeyControls()
    {
        uiKeyThrottle = AddKey("Throttle Key", "kThr", KeyCode.T);
        uiKeyBrake = AddKey("Brake Key", "kBrk", KeyCode.G);
        uiKeyReverse = AddKey("Reverse Key", "kRev", KeyCode.R);
        uiKeyGearUp = AddKey("Shift Up Key", "kGUp", KeyCode.PageUp);
        uiKeyGearDown = AddKey("Shift Down Key", "kGDn", KeyCode.PageDown);
        UpdateDriveKeyVisibility();
    }

    internal void UpdateDriveKeyVisibility()
    {
        if (uiKeyThrottle != null) uiKeyThrottle.DisplayInMapper = drivenWheel;
        if (uiKeyReverse != null) uiKeyReverse.DisplayInMapper = drivenWheel;
        if (uiKeyGearUp != null) uiKeyGearUp.DisplayInMapper = drivenWheel;
        if (uiKeyGearDown != null) uiKeyGearDown.DisplayInMapper = drivenWheel;
        // Brake remains available for both driven and free-rolling wheels.
        if (uiKeyBrake != null) uiKeyBrake.DisplayInMapper = true;
    }

    // B1: wheel-axis PROPULSION torque self-applied this step.  The static
    // friction solver feeds this forward instead of waiting a step for the
    // disturbance estimator to learn it.  Brake/coast torque must never enter
    // this channel: doing so makes static friction cancel the brake itself.
    private float pendingDriveAxisTorque;
    private Vector3 pendingDriveAxisWorld = Vector3.up;

    private void ApplyDriveBrake()
    {
        float dt = Time.fixedDeltaTime;
        bool driveActive = enableDriveBrake && drivenWheel;
        bool heldThr = driveActive && uiKeyThrottle != null && uiKeyThrottle.IsHeld;
        bool heldBrk = enableDriveBrake && uiKeyBrake != null && uiKeyBrake.IsHeld;
        UpdateGearboxInput();

        float targetThr = heldThr ? 1f : 0f;
        float targetBrk = heldBrk ? 1f : 0f;
        float thrRate = targetThr > throttle01 ? Mathf.Max(0f, throttleRise) : Mathf.Max(0f, throttleFall);
        float brkRate = targetBrk > brake01 ? Mathf.Max(0f, brakeRise) : Mathf.Max(0f, brakeFall);
        throttle01 = Mathf.MoveTowards(throttle01, targetThr, thrRate * dt);
        brake01 = Mathf.MoveTowards(brake01, targetBrk, brkRate * dt);

        Vector3 driveAxis = GetDriveAxisWorld();
        float omegaAxisAbs = Vector3.Dot(Rigidbody.angularVelocity, driveAxis);
        // B4: engine RPM and braking act on wheel spin RELATIVE to the joint
        // parent, so chassis yaw/roll does not pollute the tachometer or the
        // brake hold logic.
        float omegaAxis = omegaAxisAbs;
        Rigidbody parentBody = GetJointParentBody();
        if (parentBody != null)
            omegaAxis -= Vector3.Dot(parentBody.angularVelocity, driveAxis);
        float totalRatio = GetCurrentTotalDriveRatio();
        float rawEngineRpm = drivenWheel
            ? Mathf.Abs(omegaAxis) * totalRatio * RpmPerRadPerSecond
            : 0f;
        currentEngineRawRpm = rawEngineRpm;
        float rpmFilter = Mathf.Max(0f, engineRpmFilterTau);
        float rpmAlpha = rpmFilter > 1e-5f
            ? 1f - Mathf.Exp(-dt / rpmFilter)
            : 1f;
        currentEngineRpm = Mathf.Lerp(currentEngineRpm, rawEngineRpm, rpmAlpha);
        // The wheel and gearbox kinematically define engine speed in this
        // simplified locked-clutch model. Filtering is telemetry only: using
        // delayed RPM for torque and the limiter pumps energy past redline.
        float engineRpm = rawEngineRpm;
        if (drivenWheel) UpdateEngineLimiter(engineRpm);
        else engineLimiterCut = false;
        float engineTorque = drivenWheel && enableEngineCurve
            ? EvaluateEngineTorque(Mathf.Max(engineIdleRpm, engineRpm))
            : (drivenWheel ? Mathf.Max(0f, enginePeakTorque) : 0f);
        if (engineLimiterCut) engineTorque = 0f;
        float transmissionScale = totalRatio * Mathf.Clamp01(drivetrainEfficiency);
        float engineTorqueAtWheel = engineTorque * transmissionScale;
        float tauDriveCmd = throttle01 * engineTorqueAtWheel;

        float driveSign = (Flipped ? -1f : 1f)
                        * (invertDriveTorque ? -1f : 1f)
                        * (currentGear == 0 ? -1f : 1f);
        // Brake input wins over throttle instead of making the two fight at
        // the axle while the inputs are ramping in opposite directions.
        tauDriveCmd *= driveSign * (1f - Mathf.Clamp01(brake01));

        // Only propulsion is subject to the spin pumping guard and contact
        // feed-forward. Braking and coast are dissipative wheel torques.
        float tauDrive = ClampSpinPumpingTorque(tauDriveCmd, omegaAxisAbs, driveAxis, dt);
        float tauCoast = 0f;
        if (drivenWheel
            && engineCoastTorque > 0f
            && (throttle01 < 0.999f || engineLimiterCut)
            && Mathf.Abs(omegaAxis) > Mathf.Max(1e-4f, brakeDeadbandOmega))
        {
            float coastBlend = engineLimiterCut ? 1f : 1f - throttle01;
            tauCoast = -Mathf.Sign(omegaAxis)
                * engineCoastTorque
                * transmissionScale
                * coastBlend;
        }

        float tauBrake = 0f;
        if (brake01 > 1e-4f)
        {
            float tauBMax = brake01 * maxBrakeTorque;
            if (Mathf.Abs(omegaAxis) > 1e-5f)
            {
                // Never allow the brake to reverse wheel spin in one physics
                // step. The old low-speed spring changed sign each frame,
                // producing axle chatter and repeatedly invalidating contact.
                float wheelInertia = GetInertiaAroundWorldAxis(driveAxis);
                float stopTorque = wheelInertia > 1e-6f
                    ? wheelInertia * Mathf.Abs(omegaAxis) / Mathf.Max(1e-5f, dt)
                    : tauBMax;
                tauBrake = -Mathf.Sign(omegaAxis) * Mathf.Min(tauBMax, stopTorque);
            }
        }

        float tauDissipative = tauCoast + tauBrake;
        if (tauDissipative * omegaAxis < 0f)
        {
            float wheelInertia = GetInertiaAroundWorldAxis(driveAxis);
            if (wheelInertia > 1e-6f)
            {
                float stopTorque = wheelInertia * Mathf.Abs(omegaAxis)
                                 / Mathf.Max(1e-5f, dt);
                tauDissipative = Mathf.Clamp(tauDissipative, -stopTorque, stopTorque);
            }
        }
        float tau = tauDrive + tauDissipative;
        pendingDriveAxisTorque = tauDrive;
        pendingDriveAxisWorld = driveAxis;
        if (Mathf.Abs(tau) > 1e-6f)
            Rigidbody.AddTorque(driveAxis * tau, ForceMode.Force);
    }

    // A5: torque that spins the wheel further toward the hard cap is limited in
    // velocity space so a single step can never push |omega| past the cap.
    // Torque opposing the current spin (braking) is dissipative and never limited.
    private float ClampSpinPumpingTorque(float tau, float omegaAxis, Vector3 axisWorld, float dt)
    {
        if (tau * omegaAxis <= 0f) return tau;

        float headroom = GetSpinCap() - Mathf.Abs(omegaAxis);
        if (headroom <= 0f) return 0f;

        float inertia = GetInertiaAroundWorldAxis(axisWorld);
        if (inertia <= 1e-6f) return tau;

        float maxTau = inertia * headroom / Mathf.Max(1e-5f, dt);
        return Mathf.Clamp(tau, -maxTau, maxTau);
    }

    internal float EvaluateEngineTorque(float engineRpm)
    {
        float rpm = Mathf.Max(0f, engineRpm);
        if (rpm >= Mathf.Max(1f, engineRedlineRpm)) return 0f;
        if (!EnsureEngineTorqueLut()) return Mathf.Max(0f, enginePeakTorque);
        if (rpm <= engineLutRpm[0]) return Mathf.Max(0f, engineLutTorque[0]);

        for (int i = 1; i < engineLutPointCount; i++)
        {
            if (rpm > engineLutRpm[i]) continue;
            float span = Mathf.Max(1e-4f, engineLutRpm[i] - engineLutRpm[i - 1]);
            float u = Mathf.Clamp01((rpm - engineLutRpm[i - 1]) / span);
            if (!smoothEngineTorqueLut)
                return Mathf.Max(0f, Mathf.Lerp(engineLutTorque[i - 1], engineLutTorque[i], u));

            float u2 = u * u;
            float u3 = u2 * u;
            float h00 = 2f * u3 - 3f * u2 + 1f;
            float h10 = u3 - 2f * u2 + u;
            float h01 = -2f * u3 + 3f * u2;
            float h11 = u3 - u2;
            float torque = h00 * engineLutTorque[i - 1]
                         + h10 * span * engineLutSlope[i - 1]
                         + h01 * engineLutTorque[i]
                         + h11 * span * engineLutSlope[i];
            // PCHIP should be shape preserving; this final clamp also guards
            // against float noise on extremely close RPM points.
            torque = Mathf.Clamp(torque,
                Mathf.Min(engineLutTorque[i - 1], engineLutTorque[i]),
                Mathf.Max(engineLutTorque[i - 1], engineLutTorque[i]));
            return Mathf.Max(0f, torque);
        }
        int last = engineLutPointCount - 1;
        float lastRpm = engineLutRpm[last];
        float lastTorque = Mathf.Max(0f, engineLutTorque[last]);
        float redline = Mathf.Max(lastRpm + 1e-4f, engineRedlineRpm);
        float tailU = Mathf.Clamp01((rpm - lastRpm) / (redline - lastRpm));
        return lastTorque * (1f - SmoothStep01(tailU));
    }

    // C5: single source of truth for the curve breakpoints; the factory UI
    // draws its markers from the same values the torque evaluation uses.
    internal void GetEngineCurveBreakpoints(out float baseRpm, out float powerHoldRpm, out float redlineRpm)
    {
        baseRpm = 0f;
        powerHoldRpm = 0f;
        redlineRpm = Mathf.Max(1f, engineRedlineRpm);
    }

    private void UpdateEngineLimiter(float rpm)
    {
        float limiter = Mathf.Max(1f, engineRedlineRpm);
        float resetRpm = limiter - Mathf.Max(1f, engineLimiterHysteresisRpm);
        if (rpm >= limiter) engineLimiterCut = true;
        else if (rpm <= resetRpm) engineLimiterCut = false;
    }

    private bool EnsureEngineTorqueLut()
    {
        string source = engineTorqueLut ?? String.Empty;
        if (String.Equals(source, parsedEngineTorqueLut, StringComparison.Ordinal))
            return engineLutPointCount >= 2;

        parsedEngineTorqueLut = source;
        engineLutPointCount = 0;
        string normalized = source.Replace("\r", "\n").Replace(';', '\n');
        string[] lines = normalized.Split(new char[] { '\n' }, StringSplitOptions.RemoveEmptyEntries);
        for (int i = 0; i < lines.Length && engineLutPointCount < MaximumEngineLutPoints; i++)
        {
            string line = lines[i].Trim();
            if (line.Length == 0 || line[0] == '#' || line[0] == ';') continue;
            int separator = line.IndexOf('|');
            if (separator <= 0 || separator >= line.Length - 1) continue;
            float rpm;
            float torque;
            if (!float.TryParse(line.Substring(0, separator).Trim(), NumberStyles.Float,
                    CultureInfo.InvariantCulture, out rpm)) continue;
            if (!float.TryParse(line.Substring(separator + 1).Trim(), NumberStyles.Float,
                    CultureInfo.InvariantCulture, out torque)) continue;
            if (rpm < 0f || torque < 0f) continue;

            bool replacedDuplicate = false;
            for (int p = 0; p < engineLutPointCount; p++)
            {
                if (Mathf.Abs(engineLutRpm[p] - rpm) > 0.01f) continue;
                engineLutTorque[p] = torque;
                replacedDuplicate = true;
                break;
            }
            if (replacedDuplicate) continue;

            int insert = engineLutPointCount;
            while (insert > 0 && rpm < engineLutRpm[insert - 1])
            {
                engineLutRpm[insert] = engineLutRpm[insert - 1];
                engineLutTorque[insert] = engineLutTorque[insert - 1];
                insert--;
            }
            engineLutRpm[insert] = rpm;
            engineLutTorque[insert] = torque;
            engineLutPointCount++;
        }
        RebuildEngineLutSlopes();
        return engineLutPointCount >= 2;
    }

    internal bool FactorySetEngineTorqueLut(string text, out string error)
    {
        string previous = engineTorqueLut;
        engineTorqueLut = text ?? String.Empty;
        parsedEngineTorqueLut = null;
        if (EnsureEngineTorqueLut())
        {
            error = engineLutPointCount + " points loaded";
            return true;
        }

        engineTorqueLut = previous;
        parsedEngineTorqueLut = null;
        EnsureEngineTorqueLut();
        error = "Need at least two valid RPM|Nm rows";
        return false;
    }

    internal string FactoryGetEngineTorqueLut() { return engineTorqueLut ?? String.Empty; }
    internal int FactoryEngineLutPointCount() { EnsureEngineTorqueLut(); return engineLutPointCount; }
    internal void FactoryEngineLutPoint(int index, out float rpm, out float torque)
    {
        EnsureEngineTorqueLut();
        if (index < 0 || index >= engineLutPointCount)
        {
            rpm = 0f;
            torque = 0f;
            return;
        }
        rpm = engineLutRpm[index];
        torque = engineLutTorque[index];
    }

    internal bool FactoryMoveEngineLutPoint(int index, float rpm, float torque)
    {
        if (!EnsureEngineTorqueLut() || index < 0 || index >= engineLutPointCount)
            return false;

        float lower = index > 0 ? engineLutRpm[index - 1] + 25f : 0f;
        float upper = index + 1 < engineLutPointCount
            ? engineLutRpm[index + 1] - 25f
            : Mathf.Max(lower, engineRedlineRpm);
        if (upper < lower) upper = lower;
        engineLutRpm[index] = Mathf.Clamp(rpm, lower, upper);
        engineLutTorque[index] = Mathf.Clamp(torque, 0f, 50000f);
        RebuildEngineTorqueLutText();
        return true;
    }

    internal bool FactoryAddEngineLutPoint(int selectedIndex, out int newIndex)
    {
        newIndex = -1;
        if (!EnsureEngineTorqueLut() || engineLutPointCount >= MaximumEngineLutPoints)
            return false;

        int left = 0;
        int right = 1;
        if (selectedIndex >= 0 && selectedIndex < engineLutPointCount)
        {
            if (selectedIndex + 1 < engineLutPointCount)
            {
                left = selectedIndex;
                right = selectedIndex + 1;
            }
            else
            {
                left = selectedIndex - 1;
                right = selectedIndex;
            }
        }
        else
        {
            float largestGap = -1f;
            for (int i = 0; i + 1 < engineLutPointCount; i++)
            {
                float gap = engineLutRpm[i + 1] - engineLutRpm[i];
                if (gap <= largestGap) continue;
                largestGap = gap;
                left = i;
                right = i + 1;
            }
        }

        float rpm = 0.5f * (engineLutRpm[left] + engineLutRpm[right]);
        float torque = EvaluateEngineTorque(rpm);
        newIndex = right;
        for (int i = engineLutPointCount; i > newIndex; i--)
        {
            engineLutRpm[i] = engineLutRpm[i - 1];
            engineLutTorque[i] = engineLutTorque[i - 1];
        }
        engineLutRpm[newIndex] = rpm;
        engineLutTorque[newIndex] = torque;
        engineLutPointCount++;
        RebuildEngineTorqueLutText();
        return true;
    }

    internal bool FactoryRemoveEngineLutPoint(int index)
    {
        if (!EnsureEngineTorqueLut() || engineLutPointCount <= 2
            || index < 0 || index >= engineLutPointCount) return false;
        for (int i = index; i + 1 < engineLutPointCount; i++)
        {
            engineLutRpm[i] = engineLutRpm[i + 1];
            engineLutTorque[i] = engineLutTorque[i + 1];
        }
        engineLutPointCount--;
        RebuildEngineTorqueLutText();
        return true;
    }

    private void RebuildEngineTorqueLutText()
    {
        RebuildEngineLutSlopes();
        StringBuilder builder = new StringBuilder(engineLutPointCount * 18);
        for (int i = 0; i < engineLutPointCount; i++)
        {
            if (i > 0) builder.Append('\n');
            builder.Append(engineLutRpm[i].ToString("0.##", CultureInfo.InvariantCulture));
            builder.Append('|');
            builder.Append(engineLutTorque[i].ToString("0.##", CultureInfo.InvariantCulture));
        }
        engineTorqueLut = builder.ToString();
        parsedEngineTorqueLut = engineTorqueLut;
    }

    private void RebuildEngineLutSlopes()
    {
        if (engineLutPointCount <= 0) return;
        if (engineLutPointCount == 1)
        {
            engineLutSlope[0] = 0f;
            return;
        }

        for (int i = 0; i + 1 < engineLutPointCount; i++)
        {
            engineLutSpan[i] = Mathf.Max(1e-4f, engineLutRpm[i + 1] - engineLutRpm[i]);
            engineLutSecant[i] = (engineLutTorque[i + 1] - engineLutTorque[i]) / engineLutSpan[i];
        }

        if (engineLutPointCount == 2)
        {
            engineLutSlope[0] = engineLutSecant[0];
            engineLutSlope[1] = engineLutSecant[0];
            return;
        }

        engineLutSlope[0] = ShapePreservingEndpointSlope(
            engineLutSpan[0], engineLutSpan[1], engineLutSecant[0], engineLutSecant[1]);
        for (int i = 1; i + 1 < engineLutPointCount; i++)
        {
            float before = engineLutSecant[i - 1];
            float after = engineLutSecant[i];
            if (before * after <= 0f)
            {
                engineLutSlope[i] = 0f;
                continue;
            }
            float w1 = 2f * engineLutSpan[i] + engineLutSpan[i - 1];
            float w2 = engineLutSpan[i] + 2f * engineLutSpan[i - 1];
            engineLutSlope[i] = (w1 + w2) / (w1 / before + w2 / after);
        }
        int last = engineLutPointCount - 1;
        engineLutSlope[last] = ShapePreservingEndpointSlope(
            engineLutSpan[last - 1], engineLutSpan[last - 2],
            engineLutSecant[last - 1], engineLutSecant[last - 2]);
    }

    private static float ShapePreservingEndpointSlope(
        float h0, float h1, float d0, float d1)
    {
        float slope = ((2f * h0 + h1) * d0 - h0 * d1) / Mathf.Max(1e-4f, h0 + h1);
        if (slope * d0 <= 0f) return 0f;
        if (d0 * d1 < 0f && Mathf.Abs(slope) > Mathf.Abs(3f * d0))
            return 3f * d0;
        return slope;
    }

    private float SmoothStep01(float value)
    {
        float u = Mathf.Clamp01(value);
        return u * u * (3f - 2f * u);
    }

    private void UpdateGearboxInput()
    {
        int count = GetGearCount();
        currentGear = ClampGear(currentGear, count);
        if (!enableDriveBrake || !drivenWheel) return;

        if (uiKeyReverse != null && uiKeyReverse.IsPressed)
        {
            currentGear = currentGear == 0 ? 1 : 0;
            return;
        }
        if (!enableGearbox) return;
        if (uiKeyGearUp != null && uiKeyGearUp.IsPressed)
            currentGear = currentGear == 0 ? 1 : ClampGear(currentGear + 1, count);
        if (uiKeyGearDown != null && uiKeyGearDown.IsPressed)
            currentGear = ClampGear(currentGear - 1, count);
    }

    private int GetGearCount()
    {
        int count = Mathf.RoundToInt(gearCount);
        if (count < 1) return 1;
        if (count > 8) return 8;
        return count;
    }

    private int ClampGear(int gear, int count)
    {
        if (count < 1) count = 1;
        if (gear < 0) return 0;
        if (gear > count) return count;
        return gear;
    }

    private float GetCurrentGearRatio()
    {
        if (currentGear == 0)
            return enableGearbox ? Mathf.Max(0.05f, reverseGearRatio) : 1f;
        if (!enableGearbox) return 1f;
        int count = GetGearCount();
        currentGear = ClampGear(currentGear, count);
        return Mathf.Max(0.05f, GetGearRatio(currentGear));
    }

    private float GetCurrentTotalDriveRatio()
    {
        return Mathf.Max(0.01f, GetCurrentGearRatio())
            * Mathf.Max(0.01f, finalDriveRatio);
    }

    internal float FactoryGearWheelOmegaLimit(int gear)
    {
        float ratio = (enableGearbox ? Mathf.Max(0.01f, GetGearRatio(gear)) : 1f)
            * Mathf.Max(0.01f, finalDriveRatio);
        return Mathf.Min(GetSpinCap(), Mathf.Max(1f, engineRedlineRpm) / (ratio * RpmPerRadPerSecond));
    }

    internal float FactoryCurrentTotalDriveRatio() { return GetCurrentTotalDriveRatio(); }
    internal bool FactoryLimiterCut() { return engineLimiterCut; }
    internal float FactoryCurrentRawEngineRpm() { return Mathf.Max(0f, currentEngineRawRpm); }

    private float GetGearRatio(int gear)
    {
        switch (gear)
        {
            case 0: return reverseGearRatio;
            case 1: return gearRatio1;
            case 2: return gearRatio2;
            case 3: return gearRatio3;
            case 4: return gearRatio4;
            case 5: return gearRatio5;
            case 6: return gearRatio6;
            case 7: return gearRatio7;
            case 8: return gearRatio8;
        }
        return gearRatio1;
    }

    private void ApplyRollingAngularDrag(bool hasContact)
    {
        if (!HasRigidbody) return;
        if (!baseAngularDragCaptured)
        {
            baseAngularDrag = Rigidbody.angularDrag;
            baseAngularDragCaptured = true;
        }

        float target = baseAngularDrag;
        if (enableRollingDamping && !useLoadSensitiveRollingResistance && rollingDampingK > 0f && hasContact)
            target = baseAngularDrag + rollingDampingK;
        if (Mathf.Abs(Rigidbody.angularDrag - target) > 1e-6f)
            Rigidbody.angularDrag = target;
    }

    private void ApplyLoadSensitiveRollingResistance(float normalLoad, float radius, Vector3 wheelAxisWorld)
    {
        if (!HasRigidbody) return;
        if (!enableRollingDamping || !useLoadSensitiveRollingResistance) return;
        if (normalLoad <= 1e-6f || radius <= 1e-6f || rollingDampingK <= 0f) return;
        if (wheelAxisWorld.sqrMagnitude < 1e-10f) return;

        wheelAxisWorld.Normalize();
        float omega = Vector3.Dot(Rigidbody.angularVelocity, wheelAxisWorld);
        if (Mathf.Abs(omega) <= 1e-5f) return;

        float tauViscous = -omega * rollingDampingK;
        float tauLimit = Mathf.Max(0f, rollingResistanceCoeff) * normalLoad * radius;
        if (tauLimit > 1e-6f)
            tauViscous = Mathf.Clamp(tauViscous, -tauLimit, tauLimit);
        if (Mathf.Abs(tauViscous) > 1e-6f)
            Rigidbody.AddTorque(wheelAxisWorld * tauViscous, ForceMode.Force);
    }
}
