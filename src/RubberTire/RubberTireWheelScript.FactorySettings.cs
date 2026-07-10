using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;
using UnityEngine;
using Modding;

internal sealed class RubberTireFactorySetting
{
    public string Tab;
    public string Key;
    public string Label;
    public bool IsToggle;
    public float Min;
    public float Max;
    public Func<float> GetFloat;
    public Action<float> SetFloat;
    public Func<bool> GetBool;
    public Action<bool> SetBool;
}

public partial class RubberTireWheelScript
{
    internal const string FactoryUiModGuid = "61d89dcf-88a2-4a16-8eb2-08aeed441f1d";

    private RubberTireConfigMapper factoryConfig;
    private bool factoryApplyingConfig;
    private string factoryLastAppliedConfig;

    private RubberTireFactorySetting FactoryFloat(
        string tab,
        string key,
        string label,
        float min,
        float max,
        Func<float> getter,
        Action<float> setter)
    {
        RubberTireFactorySetting setting = new RubberTireFactorySetting();
        setting.Tab = tab;
        setting.Key = key;
        setting.Label = label;
        setting.Min = min;
        setting.Max = max;
        setting.GetFloat = getter;
        setting.SetFloat = delegate(float value)
        {
            setter(Mathf.Clamp(value, min, max));
        };
        return setting;
    }

    private RubberTireFactorySetting FactoryBool(
        string tab,
        string key,
        string label,
        Func<bool> getter,
        Action<bool> setter)
    {
        RubberTireFactorySetting setting = new RubberTireFactorySetting();
        setting.Tab = tab;
        setting.Key = key;
        setting.Label = label;
        setting.IsToggle = true;
        setting.GetBool = getter;
        setting.SetBool = setter;
        return setting;
    }

    internal List<RubberTireFactorySetting> BuildFactorySettings()
    {
        List<RubberTireFactorySetting> s = new List<RubberTireFactorySetting>(80);

        // Engine and driveline. Key bindings deliberately stay in the stock mapper.
        s.Add(FactoryBool("Engine", "drv", "Drive / brake enabled", delegate { return enableDriveBrake; }, delegate(bool v) { enableDriveBrake = v; }));
        s.Add(FactoryBool("Engine", "invDrv", "Invert drive torque", delegate { return invertDriveTorque; }, delegate(bool v) { invertDriveTorque = v; }));
        s.Add(FactoryBool("Engine", "engCurve", "Torque / power curve", delegate { return enableEngineCurve; }, delegate(bool v) { enableEngineCurve = v; }));
        s.Add(FactoryFloat("Engine", "engT", "Peak torque (N m)", 0f, 50000f, delegate { return enginePeakTorque; }, delegate(float v) { enginePeakTorque = v; }));
        s.Add(FactoryFloat("Engine", "engP", "Peak power (W)", 0f, 2000000f, delegate { return enginePeakPower; }, delegate(float v) { enginePeakPower = v; }));
        s.Add(FactoryFloat("Engine", "engHold", "Power hold RPM", 100f, 20000f, delegate { return enginePowerHoldRpm; }, delegate(float v) { enginePowerHoldRpm = v; }));
        s.Add(FactoryFloat("Engine", "engRed", "Redline RPM", 200f, 25000f, delegate { return engineRedlineRpm; }, delegate(float v) { engineRedlineRpm = v; }));
        s.Add(FactoryBool("Engine", "gbx", "Gearbox", delegate { return enableGearbox; }, delegate(bool v) { enableGearbox = v; }));
        s.Add(FactoryFloat("Engine", "gCnt", "Gear count", 1f, 8f, delegate { return gearCount; }, delegate(float v) { gearCount = Mathf.Round(v); }));
        s.Add(FactoryFloat("Engine", "gR1", "Gear 1 ratio", 0.05f, 10f, delegate { return gearRatio1; }, delegate(float v) { gearRatio1 = v; }));
        s.Add(FactoryFloat("Engine", "gR2", "Gear 2 ratio", 0.05f, 10f, delegate { return gearRatio2; }, delegate(float v) { gearRatio2 = v; }));
        s.Add(FactoryFloat("Engine", "gR3", "Gear 3 ratio", 0.05f, 10f, delegate { return gearRatio3; }, delegate(float v) { gearRatio3 = v; }));
        s.Add(FactoryFloat("Engine", "gR4", "Gear 4 ratio", 0.05f, 10f, delegate { return gearRatio4; }, delegate(float v) { gearRatio4 = v; }));
        s.Add(FactoryFloat("Engine", "gR5", "Gear 5 ratio", 0.05f, 10f, delegate { return gearRatio5; }, delegate(float v) { gearRatio5 = v; }));
        s.Add(FactoryFloat("Engine", "gR6", "Gear 6 ratio", 0.05f, 10f, delegate { return gearRatio6; }, delegate(float v) { gearRatio6 = v; }));
        s.Add(FactoryFloat("Engine", "gR7", "Gear 7 ratio", 0.05f, 10f, delegate { return gearRatio7; }, delegate(float v) { gearRatio7 = v; }));
        s.Add(FactoryFloat("Engine", "gR8", "Gear 8 ratio", 0.05f, 10f, delegate { return gearRatio8; }, delegate(float v) { gearRatio8 = v; }));
        s.Add(FactoryFloat("Engine", "brkT", "Maximum brake torque", 0f, 80000f, delegate { return maxBrakeTorque; }, delegate(float v) { maxBrakeTorque = v; }));
        s.Add(FactoryFloat("Engine", "brkDb", "Brake deadband (rad/s)", 0f, 10f, delegate { return brakeDeadbandOmega; }, delegate(float v) { brakeDeadbandOmega = v; }));
        s.Add(FactoryFloat("Engine", "brkK", "Brake hold gain", 0f, 20000f, delegate { return brakeHoldK; }, delegate(float v) { brakeHoldK = v; }));
        s.Add(FactoryFloat("Engine", "thrUp", "Throttle rise (1/s)", 0f, 40f, delegate { return throttleRise; }, delegate(float v) { throttleRise = v; }));
        s.Add(FactoryFloat("Engine", "thrDn", "Throttle fall (1/s)", 0f, 40f, delegate { return throttleFall; }, delegate(float v) { throttleFall = v; }));
        s.Add(FactoryFloat("Engine", "brkUp", "Brake rise (1/s)", 0f, 40f, delegate { return brakeRise; }, delegate(float v) { brakeRise = v; }));
        s.Add(FactoryFloat("Engine", "brkDn", "Brake fall (1/s)", 0f, 40f, delegate { return brakeFall; }, delegate(float v) { brakeFall = v; }));
        s.Add(FactoryBool("Engine", "rDmp", "Rolling resistance", delegate { return enableRollingDamping; }, delegate(bool v) { enableRollingDamping = v; }));
        s.Add(FactoryFloat("Engine", "rK", "Rolling damping gain", 0f, 5000f, delegate { return rollingDampingK; }, delegate(float v) { rollingDampingK = v; }));
        s.Add(FactoryBool("Engine", "advRoll", "Load-sensitive rolling resistance", delegate { return useLoadSensitiveRollingResistance; }, delegate(bool v) { useLoadSensitiveRollingResistance = v; }));
        s.Add(FactoryFloat("Engine", "rrC", "Rolling resistance coefficient", 0f, 0.20f, delegate { return rollingResistanceCoeff; }, delegate(float v) { rollingResistanceCoeff = v; }));
        s.Add(FactoryFloat("Engine", "airDmp", "Free-spin air damping", 0f, 100f, delegate { return axleAirDampingK; }, delegate(float v) { axleAirDampingK = v; }));

        s.Add(FactoryBool("Tire", "tire", "Tire model", delegate { return enableTireModel; }, delegate(bool v) { enableTireModel = v; }));
        s.Add(FactoryFloat("Tire", "muS", "Static friction coefficient", 0f, 3f, delegate { return muStatic; }, delegate(float v) { muStatic = v; }));
        s.Add(FactoryFloat("Tire", "muK", "Kinetic friction coefficient", 0f, 3f, delegate { return muKinetic; }, delegate(float v) { muKinetic = v; }));
        s.Add(FactoryFloat("Tire", "vStatic", "Static transition speed (m/s)", 0.01f, 2f, delegate { return vStatic; }, delegate(float v) { vStatic = v; }));
        s.Add(FactoryFloat("Tire", "fTau", "Force filter time (s)", 0f, 0.20f, delegate { return forceFilterTau; }, delegate(float v) { forceFilterTau = v; }));
        s.Add(FactoryBool("Tire", "advLoad", "Single-pass load scaling", delegate { return enableSinglePassLoadScaling; }, delegate(bool v) { enableSinglePassLoadScaling = v; }));
        s.Add(FactoryBool("Tire", "advSlip", "Combined-slip ellipse", delegate { return enableCombinedSlipFriction; }, delegate(bool v) { enableCombinedSlipFriction = v; }));
        s.Add(FactoryFloat("Tire", "muLong", "Longitudinal grip scale", 0f, 3f, delegate { return longitudinalGripScale; }, delegate(float v) { longitudinalGripScale = v; }));
        s.Add(FactoryFloat("Tire", "muLat", "Lateral grip scale", 0f, 3f, delegate { return lateralGripScale; }, delegate(float v) { lateralGripScale = v; }));
        s.Add(FactoryBool("Tire", "mls", "Modern low-speed model", delegate { return enableModernLowSpeedTire; }, delegate(bool v) { enableModernLowSpeedTire = v; }));
        s.Add(FactoryFloat("Tire", "mlsRf", "Low-speed relaxation floor", 0.05f, 5f, delegate { return lowSpeedRelaxSpeedFloor; }, delegate(float v) { lowSpeedRelaxSpeedFloor = v; }));
        s.Add(FactoryFloat("Tire", "mlsCv", "Low-speed creep speed", 0.01f, 1f, delegate { return lowSpeedCreepSpeed; }, delegate(float v) { lowSpeedCreepSpeed = v; }));
        s.Add(FactoryFloat("Tire", "mlsCb", "Low-speed creep blend", 0.05f, 3f, delegate { return lowSpeedCreepBlend; }, delegate(float v) { lowSpeedCreepBlend = v; }));
        s.Add(FactoryFloat("Tire", "mlsC", "Low-speed shear damping", 0f, 10000f, delegate { return lowSpeedShearDampingC; }, delegate(float v) { lowSpeedShearDampingC = v; }));
        s.Add(FactoryBool("Tire", "relax", "Tire relaxation", delegate { return enableTireRelaxation; }, delegate(bool v) { enableTireRelaxation = v; }));
        s.Add(FactoryFloat("Tire", "relaxL", "Relaxation length (m)", 0.05f, 5f, delegate { return relaxLength; }, delegate(float v) { relaxLength = v; }));
        s.Add(FactoryFloat("Tire", "shK", "Shear stiffness (N/m)", 1000f, 200000f, delegate { return shearK; }, delegate(float v) { shearK = v; }));
        s.Add(FactoryFloat("Tire", "shC", "Shear damping (N s/m)", 0f, 5000f, delegate { return shearC; }, delegate(float v) { shearC = v; }));
        s.Add(FactoryFloat("Tire", "shMax", "Maximum shear displacement", 0.01f, 0.50f, delegate { return maxShearDisp; }, delegate(float v) { maxShearDisp = v; }));
        s.Add(FactoryBool("Tire", "decouple", "Decouple force / torque", delegate { return decoupleTireForceAndTorque; }, delegate(bool v) { decoupleTireForceAndTorque = v; }));
        s.Add(FactoryBool("Tire", "advDec", "Decoupled force application", delegate { return enableDecoupledTireForceApplication; }, delegate(bool v) { enableDecoupledTireForceApplication = v; }));

        s.Add(FactoryFloat("Support", "k", "Spring stiffness (N/m)", 0f, 200000f, delegate { return springK; }, delegate(float v) { springK = v; }));
        s.Add(FactoryFloat("Support", "c", "Damper coefficient", 0f, 5000f, delegate { return damperC; }, delegate(float v) { damperC = v; }));
        s.Add(FactoryBool("Support", "nStable", "Energy-safe support", delegate { return enableStableNormalSupport; }, delegate(bool v) { enableStableNormalSupport = v; }));
        s.Add(FactoryFloat("Support", "nErp", "Recovery gain", 0f, 1f, delegate { return normalSupportERP; }, delegate(float v) { normalSupportERP = v; }));
        s.Add(FactoryFloat("Support", "nDamp", "Compression stop fraction", 0f, 1f, delegate { return normalSupportVelDamping; }, delegate(float v) { normalSupportVelDamping = v; }));
        s.Add(FactoryFloat("Support", "nSlop", "Penetration slop", 0f, 0.05f, delegate { return normalSupportSlop; }, delegate(float v) { normalSupportSlop = v; }));
        s.Add(FactoryBool("Support", "advNReact", "Apply ground reaction", delegate { return enableNormalGroundReactionForces; }, delegate(bool v) { enableNormalGroundReactionForces = v; }));

        s.Add(FactoryBool("Contact", "tw-clip", "Tread-width clipping", delegate { return enableTreadWidthClip; }, delegate(bool v) { enableTreadWidthClip = v; }));
        s.Add(FactoryFloat("Contact", "tw", "Tread width", 0.05f, 5f, delegate { return treadWidth; }, delegate(float v) { treadWidth = v; }));
        s.Add(FactoryBool("Contact", "advRay", "Tread ray fan", delegate { return enableTreadRayFan; }, delegate(bool v) { enableTreadRayFan = v; }));
        s.Add(FactoryFloat("Contact", "rayN", "Tread ray count", 1f, 7f, delegate { return treadRayCount; }, delegate(float v) { treadRayCount = Mathf.RoundToInt(v); }));
        s.Add(FactoryFloat("Contact", "cpN", "Maximum contact points", 1f, 6f, delegate { return maxContactPoints; }, delegate(float v) { maxContactPoints = Mathf.RoundToInt(v); }));
        s.Add(FactoryBool("Contact", "gate", "Contact gate", delegate { return enableContactGate; }, delegate(bool v) { enableContactGate = v; }));
        s.Add(FactoryFloat("Contact", "gIn", "Gate fade-in frames", 1f, 20f, delegate { return gateFadeInFrames; }, delegate(float v) { gateFadeInFrames = Mathf.RoundToInt(v); }));
        s.Add(FactoryFloat("Contact", "gOut", "Gate fade-out frames", 1f, 20f, delegate { return gateFadeOutFrames; }, delegate(float v) { gateFadeOutFrames = Mathf.RoundToInt(v); }));
        s.Add(FactoryBool("Contact", "nF", "Normal filter", delegate { return enableNormalFilter; }, delegate(bool v) { enableNormalFilter = v; }));
        s.Add(FactoryFloat("Contact", "nFa", "Normal filter alpha", 0.01f, 1f, delegate { return normalFilterAlpha; }, delegate(float v) { normalFilterAlpha = v; }));
        s.Add(FactoryFloat("Contact", "maxW", "Maximum angular velocity", 10f, 1000f, delegate { return maxAngularVelocityLimit; }, delegate(float v) { maxAngularVelocityLimit = v; }));

        s.Add(FactoryBool("Visual", "dbg", "Debug drawing", delegate { return debugDraw; }, delegate(bool v) { debugDraw = v; }));
        s.Add(FactoryBool("Visual", "dbgF", "Draw tire forces", delegate { return debugVizTireForce; }, delegate(bool v) { debugVizTireForce = v; }));
        s.Add(FactoryBool("Visual", "dbgTA", "Draw tread and axis", delegate { return debugVizTreadAndAxis; }, delegate(bool v) { debugVizTreadAndAxis = v; }));
        s.Add(FactoryFloat("Visual", "lw", "Debug line width", 0.01f, 0.30f, delegate { return forceLineWidth; }, delegate(float v) { forceLineWidth = v; thinLineWidth = v * 0.6f; }));
        s.Add(FactoryFloat("Visual", "fs", "Debug force scale", 0.00001f, 0.01f, delegate { return forceToLength; }, delegate(float v) { forceToLength = v; }));
        return s;
    }

    private void CreateFactoryConfigMapper()
    {
        factoryConfig = new RubberTireConfigMapper(
            "Rubber Tire UIFactory data",
            "rtFactoryConfig",
            SerializeFactorySettings());
        factoryConfig.DisplayInMapper = false;
        factoryConfig.Changed += ApplyFactorySettings;
        AddCustom(factoryConfig);
        factoryLastAppliedConfig = factoryConfig.Value;
    }

    internal void FactoryPullSettings()
    {
        if (factoryConfig == null || factoryApplyingConfig) return;
        if (!String.Equals(factoryConfig.Value, factoryLastAppliedConfig, StringComparison.Ordinal))
            ApplyFactorySettings(factoryConfig.Value);
    }

    internal void FactoryCommitSettings()
    {
        if (factoryConfig == null || factoryApplyingConfig) return;
        string value = SerializeFactorySettings();
        if (!String.Equals(factoryConfig.Value, value, StringComparison.Ordinal))
            factoryConfig.Value = value;
    }

    private string SerializeFactorySettings()
    {
        StringBuilder result = new StringBuilder(2048);
        result.Append("format=2\n");
        List<RubberTireFactorySetting> settings = BuildFactorySettings();
        for (int i = 0; i < settings.Count; i++)
        {
            RubberTireFactorySetting setting = settings[i];
            result.Append(setting.Key).Append('=');
            if (setting.IsToggle)
                result.Append(setting.GetBool() ? '1' : '0');
            else
                result.Append(setting.GetFloat().ToString("R", CultureInfo.InvariantCulture));
            result.Append('\n');
        }
        return result.ToString();
    }

    private void ApplyFactorySettings(string serialized)
    {
        if (String.IsNullOrEmpty(serialized)) return;
        factoryApplyingConfig = true;
        try
        {
            Dictionary<string, string> values = new Dictionary<string, string>(StringComparer.Ordinal);
            string[] lines = serialized.Split(new char[] { '\n' }, StringSplitOptions.RemoveEmptyEntries);
            for (int i = 0; i < lines.Length; i++)
            {
                int separator = lines[i].IndexOf('=');
                if (separator <= 0) continue;
                values[lines[i].Substring(0, separator)] = lines[i].Substring(separator + 1);
            }

            List<RubberTireFactorySetting> settings = BuildFactorySettings();
            for (int i = 0; i < settings.Count; i++)
            {
                RubberTireFactorySetting setting = settings[i];
                string raw;
                if (!values.TryGetValue(setting.Key, out raw)) continue;
                if (setting.IsToggle)
                {
                    bool parsed = raw == "1" || String.Equals(raw, "true", StringComparison.OrdinalIgnoreCase);
                    setting.SetBool(parsed);
                }
                else
                {
                    float parsed;
                    if (float.TryParse(raw, NumberStyles.Float, CultureInfo.InvariantCulture, out parsed))
                        setting.SetFloat(parsed);
                }
            }
            currentGear = ClampGear(currentGear, GetGearCount());
            factoryLastAppliedConfig = serialized;
        }
        finally
        {
            factoryApplyingConfig = false;
        }
    }

    internal float FactoryEngineRedlineRpm()
    {
        float peakTorque = Mathf.Max(1e-6f, enginePeakTorque);
        float peakPower = Mathf.Max(0f, enginePeakPower);
        float baseRpm = peakPower / peakTorque * RpmPerRadPerSecond;
        float holdRpm = Mathf.Max(baseRpm, enginePowerHoldRpm);
        return Mathf.Max(holdRpm + 1f, engineRedlineRpm);
    }

    internal void FactoryEnginePoint(float rpm, out float torque, out float power)
    {
        torque = enableEngineCurve ? EvaluateEngineTorque(rpm) : Mathf.Max(0f, enginePeakTorque);
        power = torque * Mathf.Max(0f, rpm) * RadPerSecondPerRpm;
    }

    internal float FactoryCurrentEngineRpm()
    {
        if (!IsSimulating || !HasRigidbody) return -1f;
        float omega = Mathf.Abs(Vector3.Dot(Rigidbody.angularVelocity, GetDriveAxisWorld()));
        return omega * GetCurrentGearRatio() * RpmPerRadPerSecond;
    }

    internal void FactoryFrictionEllipse(bool kinetic, out float longitudinal, out float lateral)
    {
        float mu = Mathf.Max(0f, kinetic ? muKinetic : muStatic);
        longitudinal = mu * (enableCombinedSlipFriction ? Mathf.Max(0f, longitudinalGripScale) : 1f);
        lateral = mu * (enableCombinedSlipFriction ? Mathf.Max(0f, lateralGripScale) : 1f);
    }

    internal void FactoryCurrentFrictionPoint(out float longitudinal, out float lateral)
    {
        float load = Mathf.Max(1e-5f, factoryTireNormalLoad);
        longitudinal = factoryTireLongForce / load;
        lateral = factoryTireLatForce / load;
    }

    internal float FactorySupportForce(float penetration)
    {
        return Mathf.Clamp(
            Mathf.Max(0f, springK) * Mathf.Max(0f, penetration),
            0f,
            Mathf.Max(0f, maxNormalForce));
    }
}
