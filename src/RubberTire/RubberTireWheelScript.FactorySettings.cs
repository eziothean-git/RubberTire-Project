using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;
using UnityEngine;
using Modding;

internal sealed class RubberTireFactorySetting
{
    public string Tab;
    public string Group = "";
    public string Key;
    public string Label;
    public string Tooltip = "";
    public bool IsToggle;
    public bool Advanced;
    public bool Curved;
    public float Min;
    public float Max;
    public float DefaultFloat;
    public bool DefaultBool;
    public Func<float> GetFloat;
    public Action<float> SetFloat;
    public Func<bool> GetBool;
    public Action<bool> SetBool;
    public Func<bool> VisibleWhen;

    public RubberTireFactorySetting In(string group) { Group = group; return this; }
    public RubberTireFactorySetting Adv() { Advanced = true; return this; }
    public RubberTireFactorySetting Curve() { Curved = true; return this; }
    public RubberTireFactorySetting When(Func<bool> visibleWhen) { VisibleWhen = visibleWhen; return this; }
    public RubberTireFactorySetting Tip(string tooltip) { Tooltip = tooltip; return this; }
}

public partial class RubberTireWheelScript
{
    internal const string FactoryUiModGuid = "61d89dcf-88a2-4a16-8eb2-08aeed441f1d";

    private RubberTireConfigMapper factoryConfig;
    private bool factoryApplyingConfig;
    private string factoryLastAppliedConfig;
    private List<RubberTireFactorySetting> factorySettingsCache;

    // C3: metadata and delegates are immutable per block instance; build once.
    // Defaults are captured at first build (SafeAwake), before machine data
    // is applied, so they reflect the code defaults.
    internal List<RubberTireFactorySetting> GetFactorySettings()
    {
        if (factorySettingsCache == null) factorySettingsCache = BuildFactorySettings();
        return factorySettingsCache;
    }

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
        setting.DefaultFloat = getter();
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
        setting.DefaultBool = getter();
        setting.SetBool = setter;
        return setting;
    }

    private List<RubberTireFactorySetting> BuildFactorySettings()
    {
        List<RubberTireFactorySetting> s = new List<RubberTireFactorySetting>(96);

        Func<bool> whenDriven = delegate { return drivenWheel; };
        Func<bool> whenEngineCurve = delegate { return drivenWheel && enableEngineCurve; };
        Func<bool> whenConstantEngine = delegate { return drivenWheel && !enableEngineCurve; };
        Func<bool> whenGearbox = delegate { return drivenWheel && enableGearbox; };
        Func<bool> whenRolling = delegate { return enableRollingDamping; };
        Func<bool> whenCombinedSlip = delegate { return enableCombinedSlipFriction; };
        Func<bool> whenModernLowSpeed = delegate { return enableModernLowSpeedTire; };
        Func<bool> whenRelaxation = delegate { return enableTireRelaxation; };
        Func<bool> whenStableSupport = delegate { return enableStableNormalSupport; };
        Func<bool> whenTreadClip = delegate { return enableTreadWidthClip; };
        Func<bool> whenTreadRayFan = delegate { return enableTreadWidthClip && enableTreadRayFan; };
        Func<bool> whenContactGate = delegate { return enableContactGate; };
        Func<bool> whenNormalFilter = delegate { return enableNormalFilter; };
        Func<bool> whenLoadSensitivity = delegate { return muLoadSensitivity > 1e-4f; };

        // ===== Engine =====
        s.Add(FactoryBool("Engine", "drv", "Powertrain / brake controls", delegate { return enableDriveBrake; }, delegate(bool v) { enableDriveBrake = v; }).In("Mode"));
        s.Add(FactoryBool("Engine", "driven", "Driven wheel", delegate { return drivenWheel; }, delegate(bool v) { drivenWheel = v; UpdateDriveKeyVisibility(); }).In("Mode").Tip("Off removes propulsion, engine braking and shift controls; the service brake remains active"));
        s.Add(FactoryBool("Engine", "invDrv", "Invert drive torque", delegate { return invertDriveTorque; }, delegate(bool v) { invertDriveTorque = v; }).In("Engine").When(whenDriven));
        s.Add(FactoryBool("Engine", "engCurve", "AC-style torque LUT", delegate { return enableEngineCurve; }, delegate(bool v) { enableEngineCurve = v; }).In("Engine").When(whenDriven).Tip("On = interpolate RPM|Nm rows from the curve editor; off = constant torque"));
        s.Add(FactoryBool("Engine", "engSmooth", "Smooth LUT (PCHIP)", delegate { return smoothEngineTorqueLut; }, delegate(bool v) { smoothEngineTorqueLut = v; }).In("Engine").When(whenEngineCurve).Tip("Shape-preserving cubic interpolation: smooth derivatives without overshooting the key-point torques"));
        s.Add(FactoryFloat("Engine", "engT", "Constant torque (N m)", 0f, 50000f, delegate { return enginePeakTorque; }, delegate(float v) { enginePeakTorque = v; }).In("Engine").Curve().When(whenConstantEngine));
        s.Add(FactoryFloat("Engine", "engIdle", "Idle RPM", 0f, 10000f, delegate { return engineIdleRpm; }, delegate(float v) { engineIdleRpm = v; }).In("Engine").When(whenDriven));
        s.Add(FactoryFloat("Engine", "engRed", "Limiter RPM", 200f, 25000f, delegate { return engineRedlineRpm; }, delegate(float v) { engineRedlineRpm = v; }).In("Engine").When(whenDriven));
        s.Add(FactoryFloat("Engine", "engHys", "Limiter hysteresis RPM", 1f, 2000f, delegate { return engineLimiterHysteresisRpm; }, delegate(float v) { engineLimiterHysteresisRpm = v; }).In("Engine").Adv().When(whenDriven).Tip("Fuel resumes this far below the limiter"));
        s.Add(FactoryFloat("Engine", "engRpmF", "RPM filter time (s)", 0f, 0.25f, delegate { return engineRpmFilterTau; }, delegate(float v) { engineRpmFilterTau = v; }).In("Engine").Adv().When(whenDriven).Tip("Low-pass filter for wheel-coupled engine RPM and torque lookup"));
        s.Add(FactoryFloat("Engine", "engCoast", "Engine braking (N m)", 0f, 1000f, delegate { return engineCoastTorque; }, delegate(float v) { engineCoastTorque = v; }).In("Engine").Curve().When(whenDriven).Tip("Closed-throttle coast torque at the crank"));

        // ===== Gearbox =====
        s.Add(FactoryBool("Engine", "gbx", "Gearbox", delegate { return enableGearbox; }, delegate(bool v) { enableGearbox = v; }).In("Gearbox").When(whenDriven));
        s.Add(FactoryFloat("Engine", "gCnt", "Gear count", 1f, 8f, delegate { return gearCount; }, delegate(float v) { gearCount = Mathf.Round(v); }).In("Gearbox").When(whenGearbox));
        s.Add(FactoryFloat("Engine", "gRev", "Reverse ratio", 0.05f, 20f, delegate { return reverseGearRatio; }, delegate(float v) { reverseGearRatio = v; }).In("Gearbox").When(whenGearbox));
        s.Add(FactoryFloat("Engine", "gFinal", "Final drive ratio", 0.05f, 30f, delegate { return finalDriveRatio; }, delegate(float v) { finalDriveRatio = v; }).In("Gearbox").When(whenDriven).Tip("Multiplies every gear for both engine RPM and wheel torque; Besiege's large wheels typically need 6-15"));
        s.Add(FactoryFloat("Engine", "gEff", "Drivetrain efficiency", 0.1f, 1f, delegate { return drivetrainEfficiency; }, delegate(float v) { drivetrainEfficiency = v; }).In("Gearbox").Adv().When(whenDriven));
        s.Add(FactoryFloat("Engine", "gR1", "Gear 1 ratio", 0.05f, 20f, delegate { return gearRatio1; }, delegate(float v) { gearRatio1 = v; }).In("Gearbox").When(delegate { return drivenWheel && enableGearbox && GetGearCount() >= 1; }));
        s.Add(FactoryFloat("Engine", "gR2", "Gear 2 ratio", 0.05f, 20f, delegate { return gearRatio2; }, delegate(float v) { gearRatio2 = v; }).In("Gearbox").When(delegate { return drivenWheel && enableGearbox && GetGearCount() >= 2; }));
        s.Add(FactoryFloat("Engine", "gR3", "Gear 3 ratio", 0.05f, 20f, delegate { return gearRatio3; }, delegate(float v) { gearRatio3 = v; }).In("Gearbox").When(delegate { return drivenWheel && enableGearbox && GetGearCount() >= 3; }));
        s.Add(FactoryFloat("Engine", "gR4", "Gear 4 ratio", 0.05f, 20f, delegate { return gearRatio4; }, delegate(float v) { gearRatio4 = v; }).In("Gearbox").When(delegate { return drivenWheel && enableGearbox && GetGearCount() >= 4; }));
        s.Add(FactoryFloat("Engine", "gR5", "Gear 5 ratio", 0.05f, 20f, delegate { return gearRatio5; }, delegate(float v) { gearRatio5 = v; }).In("Gearbox").When(delegate { return drivenWheel && enableGearbox && GetGearCount() >= 5; }));
        s.Add(FactoryFloat("Engine", "gR6", "Gear 6 ratio", 0.05f, 20f, delegate { return gearRatio6; }, delegate(float v) { gearRatio6 = v; }).In("Gearbox").When(delegate { return drivenWheel && enableGearbox && GetGearCount() >= 6; }));
        s.Add(FactoryFloat("Engine", "gR7", "Gear 7 ratio", 0.05f, 20f, delegate { return gearRatio7; }, delegate(float v) { gearRatio7 = v; }).In("Gearbox").When(delegate { return drivenWheel && enableGearbox && GetGearCount() >= 7; }));
        s.Add(FactoryFloat("Engine", "gR8", "Gear 8 ratio", 0.05f, 20f, delegate { return gearRatio8; }, delegate(float v) { gearRatio8 = v; }).In("Gearbox").When(delegate { return drivenWheel && enableGearbox && GetGearCount() >= 8; }));

        // ===== Brakes =====
        s.Add(FactoryFloat("Engine", "brkT", "Maximum brake torque", 0f, 80000f, delegate { return maxBrakeTorque; }, delegate(float v) { maxBrakeTorque = v; }).In("Brakes").Curve());
        s.Add(FactoryFloat("Engine", "brkDb", "Coast cutoff (rad/s)", 0f, 10f, delegate { return brakeDeadbandOmega; }, delegate(float v) { brakeDeadbandOmega = v; }).In("Brakes").Adv().Tip("Below this speed engine braking stops; service braking uses a non-reversing inertia clamp"));

        // ===== Response =====
        s.Add(FactoryFloat("Engine", "thrUp", "Throttle rise (1/s)", 0f, 40f, delegate { return throttleRise; }, delegate(float v) { throttleRise = v; }).In("Response").Adv().When(whenDriven));
        s.Add(FactoryFloat("Engine", "thrDn", "Throttle fall (1/s)", 0f, 40f, delegate { return throttleFall; }, delegate(float v) { throttleFall = v; }).In("Response").Adv().When(whenDriven));
        s.Add(FactoryFloat("Engine", "brkUp", "Brake rise (1/s)", 0f, 40f, delegate { return brakeRise; }, delegate(float v) { brakeRise = v; }).In("Response").Adv());
        s.Add(FactoryFloat("Engine", "brkDn", "Brake fall (1/s)", 0f, 40f, delegate { return brakeFall; }, delegate(float v) { brakeFall = v; }).In("Response").Adv());

        // ===== Rolling =====
        s.Add(FactoryBool("Engine", "rDmp", "Rolling resistance", delegate { return enableRollingDamping; }, delegate(bool v) { enableRollingDamping = v; }).In("Rolling"));
        s.Add(FactoryFloat("Engine", "rrC", "Rolling resistance coefficient", 0f, 0.20f, delegate { return rollingResistanceCoeff; }, delegate(float v) { rollingResistanceCoeff = v; }).In("Rolling").When(whenRolling));
        s.Add(FactoryBool("Engine", "advRoll", "Load-sensitive rolling resistance", delegate { return useLoadSensitiveRollingResistance; }, delegate(bool v) { useLoadSensitiveRollingResistance = v; }).In("Rolling").Adv().When(whenRolling).Tip("Per-contact torque limited by load, instead of angularDrag"));
        s.Add(FactoryFloat("Engine", "rK", "Rolling damping gain", 0f, 5000f, delegate { return rollingDampingK; }, delegate(float v) { rollingDampingK = v; }).In("Rolling").Adv().Curve().When(whenRolling));
        s.Add(FactoryFloat("Engine", "airDmp", "Free-spin air damping", 0f, 100f, delegate { return axleAirDampingK; }, delegate(float v) { axleAirDampingK = v; }).In("Rolling").Adv().Tip("Bearing drag applied only while airborne"));

        // ===== Tire / Grip =====
        s.Add(FactoryBool("Tire", "tire", "Tire model", delegate { return enableTireModel; }, delegate(bool v) { enableTireModel = v; }).In("Grip"));
        s.Add(FactoryFloat("Tire", "muS", "Static friction coefficient", 0f, 3f, delegate { return muStatic; }, delegate(float v) { muStatic = v; }).In("Grip"));
        s.Add(FactoryFloat("Tire", "muK", "Kinetic friction coefficient", 0f, 3f, delegate { return muKinetic; }, delegate(float v) { muKinetic = v; }).In("Grip"));
        s.Add(FactoryBool("Tire", "advSlip", "Combined-slip ellipse", delegate { return enableCombinedSlipFriction; }, delegate(bool v) { enableCombinedSlipFriction = v; }).In("Grip").Tip("Separate longitudinal / lateral grip budgets"));
        s.Add(FactoryFloat("Tire", "muLong", "Longitudinal grip scale", 0f, 3f, delegate { return longitudinalGripScale; }, delegate(float v) { longitudinalGripScale = v; }).In("Grip").When(whenCombinedSlip));
        s.Add(FactoryFloat("Tire", "muLat", "Lateral grip scale", 0f, 3f, delegate { return lateralGripScale; }, delegate(float v) { lateralGripScale = v; }).In("Grip").When(whenCombinedSlip));
        s.Add(FactoryBool("Tire", "surfMu", "Surface material grip", delegate { return useSurfaceFriction; }, delegate(bool v) { useSurfaceFriction = v; }).In("Grip").Tip("Scale grip by the ground's physic material (ice, sand...)"));
        s.Add(FactoryFloat("Tire", "muLS", "Load sensitivity", 0f, 0.5f, delegate { return muLoadSensitivity; }, delegate(float v) { muLoadSensitivity = v; }).In("Grip").Adv().Tip("Grip loss exponent with load: mu*(Fref/Fn)^s"));
        s.Add(FactoryFloat("Tire", "muLRef", "Load reference (N)", 100f, 20000f, delegate { return muLoadReference; }, delegate(float v) { muLoadReference = v; }).In("Grip").Adv().Curve().When(whenLoadSensitivity));

        // ===== Tire / Low speed =====
        s.Add(FactoryBool("Tire", "mls", "Modern low-speed model", delegate { return enableModernLowSpeedTire; }, delegate(bool v) { enableModernLowSpeedTire = v; }).In("Low speed").Tip("Impulse-based static grip near standstill"));
        s.Add(FactoryFloat("Tire", "vStatic", "Static transition speed (m/s)", 0.01f, 2f, delegate { return vStatic; }, delegate(float v) { vStatic = v; }).In("Low speed").Adv().Tip("Below this slip speed the static coefficient applies"));
        s.Add(FactoryFloat("Tire", "mlsRf", "Low-speed relaxation floor", 0.05f, 5f, delegate { return lowSpeedRelaxSpeedFloor; }, delegate(float v) { lowSpeedRelaxSpeedFloor = v; }).In("Low speed").Adv().When(whenModernLowSpeed));
        s.Add(FactoryFloat("Tire", "mlsCv", "Low-speed creep speed", 0.01f, 1f, delegate { return lowSpeedCreepSpeed; }, delegate(float v) { lowSpeedCreepSpeed = v; }).In("Low speed").Adv().When(whenModernLowSpeed).Tip("Slip speed treated as full creep demand"));
        s.Add(FactoryFloat("Tire", "mlsCb", "Low-speed creep blend", 0.05f, 3f, delegate { return lowSpeedCreepBlend; }, delegate(float v) { lowSpeedCreepBlend = v; }).In("Low speed").Adv().When(whenModernLowSpeed).Tip("Creep force fades out above this slip speed"));
        s.Add(FactoryFloat("Tire", "mlsC", "Low-speed shear damping", 0f, 10000f, delegate { return lowSpeedShearDampingC; }, delegate(float v) { lowSpeedShearDampingC = v; }).In("Low speed").Adv().Curve().When(whenModernLowSpeed));

        // ===== Tire / Brush =====
        s.Add(FactoryBool("Tire", "relax", "Tire relaxation", delegate { return enableTireRelaxation; }, delegate(bool v) { enableTireRelaxation = v; }).In("Brush").Tip("Brush model: force builds with rolled distance"));
        s.Add(FactoryFloat("Tire", "relaxL", "Relaxation length (m)", 0.05f, 5f, delegate { return relaxLength; }, delegate(float v) { relaxLength = v; }).In("Brush").When(whenRelaxation).Tip("Distance rolled before the force fully builds"));
        s.Add(FactoryFloat("Tire", "shK", "Shear stiffness (N/m)", 1000f, 200000f, delegate { return shearK; }, delegate(float v) { shearK = v; }).In("Brush").Curve().When(whenRelaxation));
        s.Add(FactoryFloat("Tire", "shC", "Shear damping (N s/m)", 0f, 5000f, delegate { return shearC; }, delegate(float v) { shearC = v; }).In("Brush").Adv().Curve().When(whenRelaxation));
        s.Add(FactoryFloat("Tire", "shMax", "Maximum shear displacement", 0.01f, 0.50f, delegate { return maxShearDisp; }, delegate(float v) { maxShearDisp = v; }).In("Brush").Adv().When(whenRelaxation));
        s.Add(FactoryFloat("Tire", "fTau", "Force filter time (s)", 0f, 0.20f, delegate { return forceFilterTau; }, delegate(float v) { forceFilterTau = v; }).In("Brush").Adv().Tip("Low-pass on the brush force output"));

        // ===== Tire / Application =====
        s.Add(FactoryBool("Tire", "advLoad", "Single-pass load scaling", delegate { return enableSinglePassLoadScaling; }, delegate(bool v) { enableSinglePassLoadScaling = v; }).In("Application").Adv());
        s.Add(FactoryBool("Tire", "decouple", "Decouple force / torque", delegate { return decoupleTireForceAndTorque; }, delegate(bool v) { decoupleTireForceAndTorque = v; }).In("Application").Adv());
        s.Add(FactoryBool("Tire", "advDec", "Decoupled force application", delegate { return enableDecoupledTireForceApplication; }, delegate(bool v) { enableDecoupledTireForceApplication = v; }).In("Application").Adv());

        // ===== Support =====
        s.Add(FactoryFloat("Support", "k", "Spring stiffness (N/m)", 0f, 5000000f, delegate { return springK; }, delegate(float v) { springK = v; }).In("Spring").Curve().Tip("Physical spring branch; heavy machines may need values above 200,000 N/m"));
        s.Add(FactoryFloat("Support", "c", "Damper coefficient", 0f, 5000f, delegate { return damperC; }, delegate(float v) { damperC = v; }).In("Spring").Curve());
        s.Add(FactoryFloat("Support", "fnMax", "Maximum support force (N)", 0f, 5000000f, delegate { return maxNormalForce; }, delegate(float v) { maxNormalForce = v; }).In("Spring").Curve().Tip("Ceiling after spring and damper calculation; raise this with stiffness for very heavy machines"));
        s.Add(FactoryBool("Support", "nStable", "Energy-safe support", delegate { return enableStableNormalSupport; }, delegate(bool v) { enableStableNormalSupport = v; }).In("Solver").Tip("Velocity-space impulse caps: no pogo, mass-aware landings"));
        s.Add(FactoryFloat("Support", "nErp", "Recovery gain", 0f, 1f, delegate { return normalSupportERP; }, delegate(float v) { normalSupportERP = v; }).In("Solver").When(whenStableSupport).Tip("Fraction of penetration recovered per 0.1 s"));
        s.Add(FactoryFloat("Support", "nDamp", "Compression stop fraction", 0f, 1f, delegate { return normalSupportVelDamping; }, delegate(float v) { normalSupportVelDamping = v; }).In("Solver").When(whenStableSupport).Tip("Fraction of impact speed cancelled on touch-down"));
        s.Add(FactoryFloat("Support", "nSlop", "Penetration slop", 0f, 0.05f, delegate { return normalSupportSlop; }, delegate(float v) { normalSupportSlop = v; }).In("Solver").Adv().When(whenStableSupport));
        s.Add(FactoryBool("Support", "advNReact", "Apply ground reaction", delegate { return enableNormalGroundReactionForces; }, delegate(bool v) { enableNormalGroundReactionForces = v; }).In("Solver").Adv().Tip("Push dynamic ground bodies back"));

        // ===== Contact =====
        s.Add(FactoryBool("Contact", "tw-clip", "Tread-width clipping", delegate { return enableTreadWidthClip; }, delegate(bool v) { enableTreadWidthClip = v; }).In("Geometry"));
        s.Add(FactoryFloat("Contact", "tw", "Tread width", 0.05f, 5f, delegate { return treadWidth; }, delegate(float v) { treadWidth = v; }).In("Geometry").When(whenTreadClip));
        s.Add(FactoryBool("Contact", "rayAdaptive", "Adaptive radial budget", delegate { return enableAdaptiveRadialSampling; }, delegate(bool v) { enableAdaptiveRadialSampling = v; }).In("Geometry").Adv().Tip("Samples ground + cached surface + one rotating probe instead of every sector every physics step"));
        s.Add(FactoryFloat("Contact", "rayR", "Radial sector resolution", 1f, 12f, delegate { return radialRayCount; }, delegate(float v) { radialRayCount = Mathf.RoundToInt(v); }).In("Geometry").Adv().Tip("Angular resolution for walls / ceilings; adaptive mode keeps the per-step query budget small"));
        s.Add(FactoryBool("Contact", "advRay", "Tread ray fan", delegate { return enableTreadRayFan; }, delegate(bool v) { enableTreadRayFan = v; }).In("Geometry").Adv().When(whenTreadClip));
        s.Add(FactoryFloat("Contact", "rayN", "Tread ray count", 1f, 7f, delegate { return treadRayCount; }, delegate(float v) { treadRayCount = Mathf.RoundToInt(v); }).In("Geometry").Adv().When(whenTreadRayFan));
        s.Add(FactoryFloat("Contact", "cpN", "Maximum contact points", 1f, 6f, delegate { return maxContactPoints; }, delegate(float v) { maxContactPoints = Mathf.RoundToInt(v); }).In("Geometry").Adv());
        s.Add(FactoryBool("Contact", "gate", "Contact gate", delegate { return enableContactGate; }, delegate(bool v) { enableContactGate = v; }).In("Filtering").Adv().Tip("Fade spring support in/out on flickering contacts"));
        s.Add(FactoryFloat("Contact", "gIn", "Gate fade-in frames", 1f, 20f, delegate { return gateFadeInFrames; }, delegate(float v) { gateFadeInFrames = Mathf.RoundToInt(v); }).In("Filtering").Adv().When(whenContactGate));
        s.Add(FactoryFloat("Contact", "gOut", "Gate fade-out frames", 1f, 20f, delegate { return gateFadeOutFrames; }, delegate(float v) { gateFadeOutFrames = Mathf.RoundToInt(v); }).In("Filtering").Adv().When(whenContactGate));
        s.Add(FactoryBool("Contact", "nF", "Normal filter", delegate { return enableNormalFilter; }, delegate(bool v) { enableNormalFilter = v; }).In("Filtering").Adv());
        s.Add(FactoryFloat("Contact", "nFa", "Normal filter alpha", 0.01f, 1f, delegate { return normalFilterAlpha; }, delegate(float v) { normalFilterAlpha = v; }).In("Filtering").Adv().When(whenNormalFilter));
        s.Add(FactoryFloat("Contact", "maxW", "Maximum angular velocity", 10f, SpinHardCap, delegate { return maxAngularVelocityLimit; }, delegate(float v) { maxAngularVelocityLimit = v; }).In("Limits").Tip("Hard capped at 60 rad/s for joint solver stability"));

        // ===== Visual =====
        s.Add(FactoryBool("Visual", "dbg", "Debug drawing", delegate { return debugDraw; }, delegate(bool v) { debugDraw = v; }).In("Debug"));
        s.Add(FactoryBool("Visual", "dbgF", "Draw tire forces", delegate { return debugVizTireForce; }, delegate(bool v) { debugVizTireForce = v; }).In("Debug"));
        s.Add(FactoryBool("Visual", "dbgTA", "Draw tread and axis", delegate { return debugVizTreadAndAxis; }, delegate(bool v) { debugVizTreadAndAxis = v; }).In("Debug"));
        s.Add(FactoryFloat("Visual", "lw", "Debug line width", 0.01f, 0.30f, delegate { return forceLineWidth; }, delegate(float v) { forceLineWidth = v; thinLineWidth = v * 0.6f; }).In("Debug"));
        s.Add(FactoryFloat("Visual", "fs", "Debug force scale", 0.00001f, 0.01f, delegate { return forceToLength; }, delegate(float v) { forceToLength = v; }).In("Debug").Curve());
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
        if (String.Equals(factoryConfig.Value, value, StringComparison.Ordinal)) return;

        // E8: setting Value fires Changed -> ApplyFactorySettings, which would
        // re-parse the string we just built and run every setter again. The
        // flag suppresses that self-triggered round trip; external changes
        // (undo, paste, load) still apply normally.
        factoryApplyingConfig = true;
        try
        {
            factoryConfig.Value = value;
            factoryLastAppliedConfig = value;
        }
        finally
        {
            factoryApplyingConfig = false;
        }
    }

    private string SerializeFactorySettings()
    {
        StringBuilder result = new StringBuilder(2048);
        result.Append("format=2\n");
        List<RubberTireFactorySetting> settings = GetFactorySettings();
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
        result.Append("engLut64=")
            .Append(Convert.ToBase64String(Encoding.UTF8.GetBytes(FactoryGetEngineTorqueLut())))
            .Append('\n');
        return result.ToString();
    }

    private void ApplyFactorySettings(string serialized)
    {
        if (factoryApplyingConfig) return;
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

            List<RubberTireFactorySetting> settings = GetFactorySettings();
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
            string encodedLut;
            if (values.TryGetValue("engLut64", out encodedLut))
            {
                try
                {
                    string lut = Encoding.UTF8.GetString(Convert.FromBase64String(encodedLut));
                    string ignored;
                    FactorySetEngineTorqueLut(lut, out ignored);
                }
                catch (FormatException) { }
            }
            currentGear = ClampGear(currentGear, GetGearCount());
            UpdateDriveKeyVisibility();
            factoryLastAppliedConfig = serialized;
        }
        finally
        {
            factoryApplyingConfig = false;
        }
    }

    // E3: reset one tab to the code defaults captured at first build.
    internal void FactoryResetTab(string tab)
    {
        List<RubberTireFactorySetting> settings = GetFactorySettings();
        for (int i = 0; i < settings.Count; i++)
        {
            RubberTireFactorySetting setting = settings[i];
            if (!String.Equals(setting.Tab, tab, StringComparison.Ordinal)) continue;
            if (setting.IsToggle) setting.SetBool(setting.DefaultBool);
            else setting.SetFloat(setting.DefaultFloat);
        }
        if (String.Equals(tab, "Engine", StringComparison.Ordinal))
        {
            string ignored;
            FactorySetEngineTorqueLut(DefaultEngineTorqueLut, out ignored);
        }
        currentGear = ClampGear(currentGear, GetGearCount());
        FactoryCommitSettings();
    }

    // E3: copy the whole config record from another wheel. Setting Value fires
    // Changed on THIS block, which applies every field.
    internal void FactoryApplyConfigFrom(RubberTireWheelScript source)
    {
        if (source == null || source == this) return;
        if (factoryConfig == null || source.factoryConfig == null) return;
        source.FactoryCommitSettings();
        factoryConfig.Value = source.factoryConfig.Value;
    }

    internal void FactoryEnginePoint(float rpm, out float torque, out float power)
    {
        torque = enableEngineCurve ? EvaluateEngineTorque(rpm) : Mathf.Max(0f, enginePeakTorque);
        power = torque * Mathf.Max(0f, rpm) * RadPerSecondPerRpm;
    }

    internal float FactoryCurrentEngineRpm()
    {
        if (!IsSimulating || !HasRigidbody) return -1f;
        if (!drivenWheel) return 0f;
        return Mathf.Max(0f, currentEngineRpm);
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

    internal void FactoryTireSlipDiagnostics(
        out float longitudinalSlip, out float lateralSlip,
        out float lockBlend, out bool staticSolved)
    {
        longitudinalSlip = factoryTireLongSlip;
        lateralSlip = factoryTireSideSlip;
        lockBlend = factoryStaticLockBlend;
        staticSolved = factoryStaticConstraintSolved;
    }

    internal float FactorySupportForce(float penetration)
    {
        return Mathf.Clamp(
            Mathf.Max(0f, springK) * Mathf.Max(0f, penetration),
            0f,
            Mathf.Max(0f, maxNormalForce));
    }

    // E6: live telemetry for the workspace strip.
    internal int FactoryCurrentGear() { return currentGear; }
    internal string FactoryCurrentGearLabel() { return currentGear == 0 ? "R" : currentGear.ToString(CultureInfo.InvariantCulture); }
    internal bool FactoryIsDrivenWheel() { return drivenWheel; }
    internal float FactoryThrottle01() { return throttle01; }
    internal float FactoryBrake01() { return brake01; }

    internal void FactoryContactQueryDiagnostics(
        out int rays, out int rawHits, out int acceptedHits, out int saturated)
    {
        rays = lastRaycastQueryCount;
        rawHits = lastRaycastRawHitCount;
        acceptedHits = lastRaycastAcceptedHitCount;
        saturated = lastRaycastSaturatedCount;
    }

    internal void FactoryLiveContactSummary(out int contactCount, out float totalLoad)
    {
        contactCount = IsSimulating ? topSamples.Count : 0;
        totalLoad = factoryTireNormalLoad;
    }

    // E6: per-sample view for the Contact tab live list.
    internal int FactoryContactSampleCount()
    {
        return IsSimulating ? topSamples.Count : 0;
    }

    internal int FactoryFilteredOwnMachineHits()
    {
        return IsSimulating ? lastOwnMachineHitCount : 0;
    }

    internal void FactoryContactSample(int index, out float penetration, out float gate)
    {
        penetration = 0f;
        gate = 0f;
        if (index < 0 || index >= topSamples.Count) return;
        ContactSample sample = topSamples[index];
        penetration = sample.pen;
        ColliderContactState state;
        if (colStates.TryGetValue(sample.colId, out state) && state != null)
            gate = Mathf.Clamp01(state.gate);
    }
}
