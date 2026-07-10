using UnityEngine;
using Modding;

public partial class RubberTireWheelScript
{
    public float driveTorque = 0f;

    public bool enableDriveBrake = true;
    public bool invertDriveTorque = false;
    public bool enableEngineCurve = true;
    public float enginePeakTorque = 450f;
    public float enginePeakPower = 180000f;
    public float enginePowerHoldRpm = 6000f;
    public float engineRedlineRpm = 7500f;
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
    private float baseAngularDrag;
    private bool baseAngularDragCaptured;

    // Fixed global shape choice: enough launch torque for direct coupling,
    // without adding another per-engine tuning parameter.
    private const float EngineZeroSpeedTorqueRatio = 0.65f;
    private const float EngineTorqueRiseBaseFraction = 0.35f;
    private const float RpmPerRadPerSecond = 9.5492966f;
    private const float RadPerSecondPerRpm = 0.10471976f;

    private void CreateDrivetrainKeyControls()
    {
        uiKeyThrottle = AddKey("Throttle Key", "kThr", KeyCode.T);
        uiKeyBrake = AddKey("Brake Key", "kBrk", KeyCode.G);
        uiKeyReverse = AddKey("Reverse Key", "kRev", KeyCode.R);
        uiKeyGearUp = AddKey("Shift Up Key", "kGUp", KeyCode.PageUp);
        uiKeyGearDown = AddKey("Shift Down Key", "kGDn", KeyCode.PageDown);
    }

    // B1: wheel-axis torque self-applied this step (drive+brake+test hook).
    // The static friction solver feeds it forward instead of waiting a step
    // for the disturbance estimator to learn it.
    private float pendingDriveAxisTorque;
    private Vector3 pendingDriveAxisWorld = Vector3.up;

    private void ApplyDriveBrake()
    {
        float dt = Time.fixedDeltaTime;
        bool heldThr = enableDriveBrake && uiKeyThrottle != null && uiKeyThrottle.IsHeld;
        bool heldBrk = enableDriveBrake && uiKeyBrake != null && uiKeyBrake.IsHeld;
        bool heldRev = enableDriveBrake && uiKeyReverse != null && uiKeyReverse.IsHeld;
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
        float gearRatio = GetCurrentGearRatio();
        float engineRpm = Mathf.Abs(omegaAxis) * gearRatio * RpmPerRadPerSecond;
        float engineTorque = enableEngineCurve
            ? EvaluateEngineTorque(engineRpm)
            : Mathf.Max(0f, enginePeakTorque);
        float engineTorqueAtWheel = engineTorque * gearRatio;
        float tauDriveCmd = throttle01 * engineTorqueAtWheel;

        float driveSign = (Flipped ? -1f : 1f)
                        * (invertDriveTorque ? -1f : 1f)
                        * (heldRev ? -1f : 1f);
        tauDriveCmd *= driveSign;

        float tau = tauDriveCmd;
        if (brake01 > 1e-4f)
        {
            float tauBMax = brake01 * maxBrakeTorque;
            if (Mathf.Abs(omegaAxis) > Mathf.Max(1e-4f, brakeDeadbandOmega))
                tau += -Mathf.Sign(omegaAxis) * tauBMax;
            else
                tau += Mathf.Clamp(-omegaAxis * brakeHoldK, -tauBMax, tauBMax);
        }

        // A5: never pump the wheel past the spin cap; braking stays unlimited.
        // The cap guards PhysX/joint stability, so it uses ABSOLUTE spin.
        tau = ClampSpinPumpingTorque(tau, omegaAxisAbs, driveAxis, dt);
        pendingDriveAxisTorque = tau;
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
        float peakTorque = Mathf.Max(0f, enginePeakTorque);
        float peakPower = Mathf.Max(0f, enginePeakPower);
        if (peakTorque <= 1e-6f || peakPower <= 1e-6f) return 0f;

        // T and P are independent controls. Their physically required
        // crossover is derived rather than exposed as a redundant parameter.
        float baseRpm, powerHoldRpm, redlineRpm;
        GetEngineCurveBreakpoints(out baseRpm, out powerHoldRpm, out redlineRpm);
        float rpm = Mathf.Max(0f, engineRpm);
        if (rpm >= redlineRpm) return 0f;

        float torqueRiseRpm = Mathf.Max(1f, baseRpm * EngineTorqueRiseBaseFraction);
        if (rpm < torqueRiseRpm)
        {
            float u = SmoothStep01(rpm / torqueRiseRpm);
            return Mathf.Lerp(
                peakTorque * EngineZeroSpeedTorqueRatio,
                peakTorque,
                u);
        }

        if (rpm <= baseRpm)
            return peakTorque;

        float omega = Mathf.Max(1e-4f, rpm * RadPerSecondPerRpm);
        if (rpm <= powerHoldRpm)
            return peakPower / omega;

        float falloff = 1f - SmoothStep01(
            (rpm - powerHoldRpm)
            / Mathf.Max(1f, redlineRpm - powerHoldRpm));
        return peakPower * falloff / omega;
    }

    // C5: single source of truth for the curve breakpoints; the factory UI
    // draws its markers from the same values the torque evaluation uses.
    internal void GetEngineCurveBreakpoints(out float baseRpm, out float powerHoldRpm, out float redlineRpm)
    {
        float peakTorque = Mathf.Max(1e-6f, enginePeakTorque);
        float peakPower = Mathf.Max(0f, enginePeakPower);
        baseRpm = Mathf.Max(1f, peakPower / peakTorque * RpmPerRadPerSecond);
        powerHoldRpm = Mathf.Max(baseRpm, enginePowerHoldRpm);
        redlineRpm = Mathf.Max(powerHoldRpm + 1f, engineRedlineRpm);
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
        if (!enableDriveBrake || !enableGearbox) return;
        if (uiKeyGearUp != null && uiKeyGearUp.IsPressed)
            currentGear = ClampGear(currentGear + 1, count);
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
        if (gear < 1) return 1;
        if (gear > count) return count;
        return gear;
    }

    private float GetCurrentGearRatio()
    {
        if (!enableGearbox) return 1f;
        int count = GetGearCount();
        currentGear = ClampGear(currentGear, count);
        return Mathf.Max(0.05f, GetGearRatio(currentGear));
    }

    private float GetGearRatio(int gear)
    {
        switch (gear)
        {
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
