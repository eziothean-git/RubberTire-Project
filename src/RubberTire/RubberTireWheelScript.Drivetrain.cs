using UnityEngine;
using Modding;

public partial class RubberTireWheelScript
{
    public float driveTorque = 0f;

    public bool enableDriveBrake = true;
    public bool invertDriveTorque = false;
    public float maxDriveTorque = 8000f;
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

    public bool enablePowerLimit = true;
    public float maxDrivePower = 120000f;
    public float powerLimitOmegaEps = 1.0f;

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

    private MToggle uiDriveBrake, uiInvertDrive;
    private MKey uiKeyThrottle, uiKeyBrake, uiKeyReverse, uiKeyGearUp, uiKeyGearDown;
    private MSlider uiMaxDriveTorque, uiMaxBrakeTorque, uiBrakeDeadband, uiBrakeHoldK;
    private MToggle uiPowerLimit, uiGearbox;
    private MSlider uiMaxDrivePower, uiPowerOmegaEps, uiGearCount;
    private MSlider uiGearRatio1, uiGearRatio2, uiGearRatio3, uiGearRatio4;
    private MSlider uiGearRatio5, uiGearRatio6, uiGearRatio7, uiGearRatio8;
    private MToggle uiRollingDamp, uiLoadRollingResistance;
    private MSlider uiRollingDampK, uiRollingResistanceCoeff;
    private MSlider uiThrottleRise, uiThrottleFall, uiBrakeRise, uiBrakeFall;

    private float throttle01;
    private float brake01;
    private int currentGear = 1;
    private float baseAngularDrag;
    private bool baseAngularDragCaptured;

    private void CreateDrivetrainMapperControls()
    {
        uiDriveBrake = AddToggle("Drive/Brake (Keys)", "drv", enableDriveBrake);
        uiInvertDrive = AddToggle("Invert Drive Torque", "invDrv", invertDriveTorque);
        uiKeyThrottle = AddKey("Throttle Key", "kThr", KeyCode.T);
        uiKeyBrake = AddKey("Brake Key", "kBrk", KeyCode.G);
        uiKeyReverse = AddKey("Reverse Key", "kRev", KeyCode.R);

        uiGearbox = AddToggle("Gearbox", "gbx", enableGearbox);
        uiKeyGearUp = AddKey("Shift Up Key", "kGUp", KeyCode.PageUp);
        uiKeyGearDown = AddKey("Shift Down Key", "kGDn", KeyCode.PageDown);
        uiGearCount = AddSlider("Gear Count", "gCnt", gearCount, 1f, 8f);
        uiGearRatio1 = AddSlider("Gear 1 Ratio", "gR1", gearRatio1, 0.05f, 10f);
        uiGearRatio2 = AddSlider("Gear 2 Ratio", "gR2", gearRatio2, 0.05f, 10f);
        uiGearRatio3 = AddSlider("Gear 3 Ratio", "gR3", gearRatio3, 0.05f, 10f);
        uiGearRatio4 = AddSlider("Gear 4 Ratio", "gR4", gearRatio4, 0.05f, 10f);
        uiGearRatio5 = AddSlider("Gear 5 Ratio", "gR5", gearRatio5, 0.05f, 10f);
        uiGearRatio6 = AddSlider("Gear 6 Ratio", "gR6", gearRatio6, 0.05f, 10f);
        uiGearRatio7 = AddSlider("Gear 7 Ratio", "gR7", gearRatio7, 0.05f, 10f);
        uiGearRatio8 = AddSlider("Gear 8 Ratio", "gR8", gearRatio8, 0.05f, 10f);

        uiMaxDriveTorque = AddSlider("Max Drive Torque", "drvT", maxDriveTorque, 0f, 50000f);
        uiPowerLimit = AddToggle("Power Limit", "pLim", enablePowerLimit);
        uiMaxDrivePower = AddSlider("Max Drive Power (W)", "pMax", maxDrivePower, 0f, 500000f);
        uiPowerOmegaEps = AddSlider("Power Omega Eps", "pEps", powerLimitOmegaEps, 0.1f, 20f);

        uiRollingDamp = AddToggle("Rolling Resistance", "rDmp", enableRollingDamping);
        uiRollingDampK = AddSlider("Rolling Damping K", "rK", rollingDampingK, 0f, 5000f);
        uiLoadRollingResistance = AddToggle("ADV: Load Rolling Resist", "advRoll", useLoadSensitiveRollingResistance);
        uiRollingResistanceCoeff = AddSlider("ADV: Roll Resist Coeff", "rrC", rollingResistanceCoeff, 0f, 0.20f);

        uiMaxBrakeTorque = AddSlider("Max Brake Torque", "brkT", maxBrakeTorque, 0f, 80000f);
        uiBrakeDeadband = AddSlider("Brake Deadband (rad/s)", "brkDb", brakeDeadbandOmega, 0f, 10f);
        uiBrakeHoldK = AddSlider("Brake Hold K", "brkK", brakeHoldK, 0f, 20000f);
        uiThrottleRise = AddSlider("Throttle Rise", "thrUp", throttleRise, 0f, 40f);
        uiThrottleFall = AddSlider("Throttle Fall", "thrDn", throttleFall, 0f, 40f);
        uiBrakeRise = AddSlider("Brake Rise 1/s", "brkUp", brakeRise, 0f, 40f);
        uiBrakeFall = AddSlider("Brake Fall", "brkDn", brakeFall, 0f, 40f);
    }

    private void SyncDrivetrainParamsFromUI()
    {
        if (uiDriveBrake != null) enableDriveBrake = uiDriveBrake.IsActive;
        if (uiInvertDrive != null) invertDriveTorque = uiInvertDrive.IsActive;
        if (uiMaxDriveTorque != null) maxDriveTorque = uiMaxDriveTorque.Value;
        if (uiGearbox != null) enableGearbox = uiGearbox.IsActive;
        if (uiGearCount != null) gearCount = Mathf.Clamp(Mathf.Round(uiGearCount.Value), 1f, 8f);
        if (uiGearRatio1 != null) gearRatio1 = uiGearRatio1.Value;
        if (uiGearRatio2 != null) gearRatio2 = uiGearRatio2.Value;
        if (uiGearRatio3 != null) gearRatio3 = uiGearRatio3.Value;
        if (uiGearRatio4 != null) gearRatio4 = uiGearRatio4.Value;
        if (uiGearRatio5 != null) gearRatio5 = uiGearRatio5.Value;
        if (uiGearRatio6 != null) gearRatio6 = uiGearRatio6.Value;
        if (uiGearRatio7 != null) gearRatio7 = uiGearRatio7.Value;
        if (uiGearRatio8 != null) gearRatio8 = uiGearRatio8.Value;
        currentGear = ClampGear(currentGear, GetGearCount());
        if (uiPowerLimit != null) enablePowerLimit = uiPowerLimit.IsActive;
        if (uiMaxDrivePower != null) maxDrivePower = uiMaxDrivePower.Value;
        if (uiPowerOmegaEps != null) powerLimitOmegaEps = uiPowerOmegaEps.Value;

        if (uiRollingDamp != null) enableRollingDamping = uiRollingDamp.IsActive;
        if (uiRollingDampK != null) rollingDampingK = uiRollingDampK.Value;
        if (uiLoadRollingResistance != null) useLoadSensitiveRollingResistance = uiLoadRollingResistance.IsActive;
        if (uiRollingResistanceCoeff != null) rollingResistanceCoeff = uiRollingResistanceCoeff.Value;
        if (uiMaxBrakeTorque != null) maxBrakeTorque = uiMaxBrakeTorque.Value;
        if (uiBrakeDeadband != null) brakeDeadbandOmega = uiBrakeDeadband.Value;
        if (uiBrakeHoldK != null) brakeHoldK = uiBrakeHoldK.Value;
        if (uiThrottleRise != null) throttleRise = uiThrottleRise.Value;
        if (uiThrottleFall != null) throttleFall = uiThrottleFall.Value;
        if (uiBrakeRise != null) brakeRise = uiBrakeRise.Value;
        if (uiBrakeFall != null) brakeFall = uiBrakeFall.Value;
    }

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
        float omegaAxis = Vector3.Dot(Rigidbody.angularVelocity, driveAxis);
        float gearRatio = GetCurrentGearRatio();
        float engineTorqueAtWheel = maxDriveTorque * gearRatio;
        float tauDriveCmd = throttle01 * engineTorqueAtWheel;

        float driveSign = (Flipped ? -1f : 1f)
                        * (invertDriveTorque ? -1f : 1f)
                        * (heldRev ? -1f : 1f);
        tauDriveCmd *= driveSign;

        if (enablePowerLimit && maxDrivePower > 0f)
        {
            float absOmega = Mathf.Abs(omegaAxis);
            if (absOmega > Mathf.Max(1e-4f, powerLimitOmegaEps))
            {
                float tauMaxByPower = maxDrivePower / absOmega;
                float tauMax = Mathf.Min(Mathf.Abs(engineTorqueAtWheel), tauMaxByPower);
                tauDriveCmd = Mathf.Clamp(tauDriveCmd, -tauMax, tauMax);
            }
        }

        float tau = tauDriveCmd;
        if (brake01 > 1e-4f)
        {
            float tauBMax = brake01 * maxBrakeTorque;
            if (Mathf.Abs(omegaAxis) > Mathf.Max(1e-4f, brakeDeadbandOmega))
                tau += -Mathf.Sign(omegaAxis) * tauBMax;
            else
                tau += Mathf.Clamp(-omegaAxis * brakeHoldK, -tauBMax, tauBMax);
        }

        if (Mathf.Abs(tau) > 1e-6f)
            Rigidbody.AddTorque(driveAxis * tau, ForceMode.Force);
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
