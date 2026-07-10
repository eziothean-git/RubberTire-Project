using System.Collections.Generic;
using UnityEngine;
using Modding;

/// <summary>
/// RubberTireWheelScript
/// - 多接触点：Top-N colliders by penetration (default N=6)
/// - 每 collider 聚合：同 collider TopK hits 按穿透加权 -> 生成 1 个稳定接触样本
/// - 接触强度 gate：fade-in/out (按物理帧渐入/渐出) 抑制“忽有忽无”的能量注入
/// - 法向一阶滤波：对聚合法向做 Slerp 低通，alpha 可调（建议较小）
/// - Raycast 方向：固定使用“轮子径向朝地面”（gravity 投影到垂直轮轴平面）
/// - LayerMask：24/29 必选，0 可选
/// - 可视化暂不改
/// </summary>
public class RubberTireWheelScript : BlockScript
{
    // =========================
    // 物理参数（会被 UI 覆盖）
    // =========================
    public float springK = 3000f;          // 法向弹簧刚度
    public float damperC = 25f;            // 法向阻尼
    public float maxNormalForce = 200000f; // 单点法向力上限（多点时每点各自 clamp）
    public bool enableStableNormalSupport = true;  // true: velocity-level support for low physics rates
    public float normalSupportERP = 0.35f;          // penetration fraction corrected per fixed step
    public float normalSupportVelDamping = 1.0f;    // relative normal velocity damping gain
    public float normalSupportMassScale = 4.0f;     // virtual mass for connected contraptions
    public float normalSupportSlop = 0.005f;        // ignored penetration before ERP correction

    public bool enableTireModel = true;

    // =========================
    // 静/动摩擦 + 输出滤波
    // =========================
    public float muStatic = 1.40f;
    public float muKinetic = 1.20f;
    public float vStatic = 0.25f;
    public float forceFilterTau = 0.03f;

    // Upgrade gates. Single-pass load scaling fixes legacy double attenuation and defaults ON;
    // riskier tire model changes stay OFF until tuned in game.
    public bool enableSinglePassLoadScaling = true;  // true: apply gate/sample weight once before tire clamp/filter
    public bool enableCombinedSlipFriction = false;  // true: longitudinal/lateral friction ellipse
    public float longitudinalGripScale = 1.0f;
    public float lateralGripScale = 1.0f;

    // Modern low-speed tire stabilization: creep/static grip + wheel-axis damping.
    public bool enableModernLowSpeedTire = true;
    public float lowSpeedRelaxSpeedFloor = 1.0f; // m/s, prevents relaxation time from exploding near standstill
    public float lowSpeedCreepSpeed = 0.15f;     // m/s, slip speed that reaches static-friction creep demand
    public float lowSpeedCreepBlend = 0.75f;     // m/s, blend-out speed for creep branch
    public float lowSpeedShearDampingC = 1200f; // N*s/m, bristle damping used by modern low-speed branch
    public float axleSpinDampingK = 250f;        // N*m/(rad/s), contact wheel-axis damping
    public float axleAirDampingK = 2f;           // N*m/(rad/s), free-spin bearing damping

    public float vEps = 0.2f;

    // =========================
    // 切向松弛/形变（per-point 状态）
    // =========================
    public bool enableTireRelaxation = true;
    public float relaxLength = 0.8f;
    public float shearK = 50000f;
    public float shearC = 0f;
    public float maxShearDisp = 0.15f;
    public bool resetShearOnNoContact = true;

    // =========================
    // 反驱：力/扭矩分离
    // =========================
    public bool decoupleTireForceAndTorque = true;
    public bool enableDecoupledTireForceApplication = false; // true: linear tire force at COM + spin-axis torque only
    public bool enableNormalGroundReactionForces = false;     // true: also push dynamic hit rigidbodies with custom normal force

    // =========================
    // 踏面裁切（有限宽度）
    // =========================
    public bool enableTreadWidthClip = true;
    public float treadWidth = 1.0f;

    // =========================
    // 接触点获取
    // =========================
    public bool useRaycastContact = true;
    public float rayExtra = 0.6f;

    public bool enableTreadRayFan = false; // true: cast multiple rays across finite tread width
    public int treadRayCount = 3;
    // Dev: layer mask performance
    public bool includeLayer0 = false;

    // ====== Multi-point settings ======
    public int maxContactPoints = 6;          // Top-N colliders by penetration after aggregation
    public float contactKeyQuantize = 0.02f;  // m，命中点 local 量化网格（用于 per-point 状态 key）
    public int contactStateTTLSteps = 30;     // 固定步数未见就淘汰状态（避免字典无限长）

    // ====== per-collider aggregation settings ======
    public int perColliderTopK = 3;            // 同一 collider 取穿透 TopK 命中进行聚合
    public bool perColliderWeightByPen = true; // true: 权重与 pen 成正比；false: 平均
    public float minPenForContact = 1e-4f;     // 过滤极小穿透（减少噪声）

    // =========================
    // NEW: 接触强度 gate + 法向滤波（调试期暴露 UI）
    // =========================
    public bool enableContactGate = true;
    public int gateFadeInFrames = 5;   // 进入接触：多少物理帧从 0 拉到 1
    public int gateFadeOutFrames = 2;  // 丢失接触：多少物理帧从 1 衰到 0（建议更快）

    public bool enableNormalFilter = true;
    public float normalFilterAlpha = 0.20f; // Slerp alpha（建议 0.05~0.30）

    public int colliderStateTTLSteps = 60; // collider 状态 TTL（gate==0 且过期则删）

    // 轮轴方向（局部）——当没找到 trigger capsule 时才用这个回退
    public enum AxisLocal { X, Y, Z }
    public AxisLocal wheelAxisLocal = AxisLocal.Y;

    // 可选：测试用驱动扭矩（一般保持 0）
    public float driveTorque = 0f;

    // =========================
    // 角速度上限
    // =========================
    public float maxAngularVelocityLimit = 250f;

    // =========================
    // Drive/Brake（键盘）
    // =========================
    public bool enableDriveBrake = true;
    public bool invertDriveTorque = false;
    public float maxDriveTorque = 8000f;
    public bool enableGearbox = true;
    public float gearCount = 5f;            // rounded to int at runtime
    public float gearRatio1 = 4.00f;
    public float gearRatio2 = 2.80f;
    public float gearRatio3 = 1.90f;
    public float gearRatio4 = 1.35f;
    public float gearRatio5 = 1.00f;
    public float gearRatio6 = 0.75f;
    public float gearRatio7 = 0.55f;
    public float gearRatio8 = 0.40f;

    // =========================
    // 恒功率约束（Drive torque cap by power）
    // |tau_drive * omega| <= maxDrivePower
    // 说明：只限制“驱动扭矩”（throttle 部分），刹车扭矩不受该限制。
    // =========================
    public bool enablePowerLimit = true;
    public float maxDrivePower = 120000f;      // W
    public float powerLimitOmegaEps = 1.0f;    // rad/s，避免低速除零

    // =========================
    // 速度阻尼（滚动阻力 / 粘性阻尼）
    // Implemented via Rigidbody.angularDrag (isotropic)
    // =========================
    public bool enableRollingDamping = false;
    public float rollingDampingK = 0.25f;      // extra Rigidbody.angularDrag (1/s)
    public bool useLoadSensitiveRollingResistance = false; // true: torque around wheel axis, clamped by normal load
    public float rollingResistanceCoeff = 0.015f;           // tau cap = coeff * Fn * radius

    public float maxBrakeTorque = 12000f;
    public float brakeDeadbandOmega = 0.5f;
    public float brakeHoldK = 2000f;

    public float throttleRise = 8f;
    public float throttleFall = 10f;
    public float brakeRise = 12f;
    public float brakeFall = 14f;

    // =========================
    // Debug 可视化（不动）
    // =========================
    public bool debugDraw = true;
    public bool debugVizTireForce = true;
    public bool debugVizTreadAndAxis = true;

    public float thinLineWidth = 0.05f;
    public float forceLineWidth = 0.10f;

    public float normalArrowLen = 0.6f;
    public float forceToLength = 0.015f;
    public float arrowMinLen = 0.25f;
    public float arrowMaxLen = 4.0f;

    public int drawEveryFixedSteps = 1;
    public int treadRangeSegments = 28;
    public float treadAxisDebugLen = 2.0f;

    // =========================
    // UI（Mapper）句柄
    // =========================
    private MSlider uiK, uiC, uiMuS, uiMuK, uiVStatic, uiFTau, uiLineW, uiForceScale, uiTreadW;
    private MToggle uiDbg, uiTreadClip, uiEnableTire;

    private MToggle uiDbgForce, uiDbgTreadAxis;

    private MToggle uiRelax, uiDecouple;
    private MSlider uiRelaxL, uiShearK, uiShearC, uiMaxShear;

    private MToggle uiSinglePassLoad, uiCombinedSlip, uiModernLowSpeed, uiStableNormal, uiDecoupledApply, uiNormalGroundReaction;
    private MSlider uiLongGrip, uiLatGrip, uiLowRelaxFloor, uiLowCreepSpeed, uiLowCreepBlend, uiLowShearDamping, uiAxleSpinDamping, uiAxleAirDamping;
    private MSlider uiNormalERP, uiNormalVelDamping, uiNormalMassScale, uiNormalSlop;

    private MToggle uiDriveBrake;
    private MToggle uiInvertDrive;
    private MKey uiKeyThrottle, uiKeyBrake, uiKeyReverse, uiKeyGearUp, uiKeyGearDown;
    private MSlider uiMaxDriveTorque, uiMaxBrakeTorque, uiBrakeDeadband, uiBrakeHoldK;
    private MToggle uiPowerLimit, uiGearbox;
    private MSlider uiMaxDrivePower, uiPowerOmegaEps, uiGearCount;
    private MSlider uiGearRatio1, uiGearRatio2, uiGearRatio3, uiGearRatio4, uiGearRatio5, uiGearRatio6, uiGearRatio7, uiGearRatio8;
    private MToggle uiRollingDamp, uiLoadRollingResistance;
    private MSlider uiRollingDampK, uiRollingResistanceCoeff;
    private MSlider uiThrottleRise, uiThrottleFall, uiBrakeRise, uiBrakeFall;

    private MSlider uiMaxAngVel;

    private MToggle uiIncludeLayer0;

    private MSlider uiMaxContactPoints;
    private MToggle uiTreadRayFan;
    private MSlider uiTreadRayCount;

    // NEW UI
    private MToggle uiGateEnable;
    private MSlider uiGateInFrames, uiGateOutFrames;
    private MToggle uiNFilterEnable;
    private MSlider uiNFilterAlpha;

    // =========================
    // 运行时状态
    // =========================
    private readonly HashSet<Collider> contacts = new HashSet<Collider>();
    private CapsuleCollider treadTriggerCapsule;
    private int fixedStepCounter = 0;

    // 键盘油门/刹车状态（0~1）
    private float throttle01 = 0f;
    private float brake01 = 0f;
    private int currentGear = 1;

    // Rolling damping via Rigidbody.angularDrag (store original to restore)
    private float baseAngularDrag = 0f;
    private bool baseAngularDragCaptured = false;

    // Temp: count contact samples per attached rigidbody (for per-body weight)
    private readonly Dictionary<int, int> tmpRbSampleCounts = new Dictionary<int, int>(16);
    private readonly RaycastHit[] raycastHitBuffer = new RaycastHit[128];
    private readonly List<List<HitSample>> hitListPool = new List<List<HitSample>>(64);

    // =========================
    // Debug 可视化（多接触点 + 插值对齐）
    // - 在 FixedUpdate 里只“采样”要画的东西，并把数据存为 Rigidbody 本地空间
    // - 在 LateUpdate 里用当前 Rigidbody 的插值 Pose 还原到世界坐标绘制（避免抖动/偏移）
    // =========================

    private GameObject dbgRoot;

    private class DebugPointViz
    {
        public GameObject root;
        public GameObject sphere;
        public LineRenderer lrCenterToP;
        public LineRenderer lrNormal;
        public LineRenderer lrForceN;
        public LineRenderer lrForceT;
    }

    private readonly List<DebugPointViz> dbgPoints = new List<DebugPointViz>(8);

    private LineRenderer lrTreadAxis;
    private LineRenderer lrTreadRingA;
    private LineRenderer lrTreadRingB;

    private struct DebugContactLocalData
    {
        public bool active;
        public Vector3 pLocal;
        // 方向量保留为 world（不要用插值后的 Rigidbody.rotation 去还原方向），
        // 否则轮子自转时 rotation 插值包含 spin，会把“本应指向世界向上/沿地面”的向量也一起转走。
        public Vector3 nWorld;
        public float FnMag;
        public Vector3 FtWorld;
        public float FtMag;
    }

    private DebugContactLocalData[] dbgLocalContacts = new DebugContactLocalData[8];
    private int dbgLocalCount = 0;
    private bool dbgHasLocal = false;

    // 本帧（物理）采样到的轮心/踏面裁切信息（全部用 Rigidbody local space 存）
    private Vector3 dbgCenterLocal = Vector3.zero;
    private Vector3 dbgAxisLocal = Vector3.right;
    private float dbgHalfWWorld = 0f;
    private float dbgRadiusWorld = 1f;
    private bool dbgDoClip = false;

    // Raycast mask cache
    private int contactRayMask = ~0;

    // ====== per-point tire state cache ======
    private struct ContactKey
    {
        public int colliderId;
        public Vector3 localQ; // quantized local point
        public override int GetHashCode()
        {
            unchecked
            {
                int h = colliderId;
                h = h * 31 + localQ.x.GetHashCode();
                h = h * 31 + localQ.y.GetHashCode();
                h = h * 31 + localQ.z.GetHashCode();
                return h;
            }
        }
        public override bool Equals(object obj)
        {
            if (!(obj is ContactKey)) return false;
            var o = (ContactKey)obj;
            return colliderId == o.colliderId && localQ == o.localQ;
        }
    }

    private class TirePointState
    {
        public Vector3 shearDispWorld = Vector3.zero;
        public Vector3 FtireFiltered = Vector3.zero;
        public int lastSeenStep = 0;
    }

    private readonly Dictionary<ContactKey, TirePointState> pointStates = new Dictionary<ContactKey, TirePointState>(64);
    private readonly List<ContactKey> pointStatesToRemove = new List<ContactKey>(64);

    // ====== NEW: per-collider contact gate + normal filter state ======
    private class ColliderContactState
    {
        public float gate = 0f;               // 0..1
        public Vector3 nFiltered = Vector3.up;
        public int lastSeenStep = 0;
        public bool seenThisFrame = false;
    }

    private readonly Dictionary<int, ColliderContactState> colStates = new Dictionary<int, ColliderContactState>(64);
    private readonly List<int> colStatesToRemove = new List<int>(64);

    // Candidate contact sample
    private struct ContactSample
    {
        public Collider col;
        public int colId;
        public Vector3 p;
        public Vector3 n;   // aggregated raw normal (unit)
        public float pen;
        public float dist;
    }

    private readonly List<ContactSample> topSamples = new List<ContactSample>(8);

    // per-collider aggregation buffers
    private struct HitSample
    {
        public Collider col;
        public int colId;
        public Vector3 p;
        public Vector3 n;
        public float pen;
    }

    private readonly Dictionary<int, List<HitSample>> hitsByCol = new Dictionary<int, List<HitSample>>(64);

    // ----------- 生命周期 -----------

    public override void SafeAwake()
    {
        uiK = AddSlider("K (Spring)", "k", springK, 0f, 200000f);
        uiC = AddSlider("C (Damper)", "c", damperC, 0f, 5000f);
        uiStableNormal = AddToggle("Stable Normal Support", "nStable", enableStableNormalSupport);
        uiNormalERP = AddSlider("Normal ERP", "nErp", normalSupportERP, 0f, 1f);
        uiNormalVelDamping = AddSlider("Normal Vel Damping", "nDamp", normalSupportVelDamping, 0f, 2f);
        uiNormalMassScale = AddSlider("Normal Mass Scale", "nMass", normalSupportMassScale, 0.25f, 20f);
        uiNormalSlop = AddSlider("Normal Slop", "nSlop", normalSupportSlop, 0f, 0.05f);
        uiMuS = AddSlider("Mu Static", "muS", muStatic, 0f, 3f);
        uiMuK = AddSlider("Mu Kinetic", "muK", muKinetic, 0f, 3f);
        uiVStatic = AddSlider("vStatic (m/s)", "vStatic", vStatic, 0.01f, 2.0f);
        uiFTau = AddSlider("Force Filter Tau (s)", "fTau", forceFilterTau, 0f, 0.20f);

        uiSinglePassLoad = AddToggle("ADV: Single Load Scale", "advLoad", enableSinglePassLoadScaling);
        uiCombinedSlip = AddToggle("ADV: Combined Slip", "advSlip", enableCombinedSlipFriction);
        uiLongGrip = AddSlider("ADV: Long Grip Scale", "muLong", longitudinalGripScale, 0f, 3f);
        uiLatGrip = AddSlider("ADV: Lat Grip Scale", "muLat", lateralGripScale, 0f, 3f);
        uiModernLowSpeed = AddToggle("Modern Low Speed Tire", "mls", enableModernLowSpeedTire);
        uiLowRelaxFloor = AddSlider("LowSpeed Relax Floor", "mlsRf", lowSpeedRelaxSpeedFloor, 0.05f, 5.0f);
        uiLowCreepSpeed = AddSlider("LowSpeed Creep Speed", "mlsCv", lowSpeedCreepSpeed, 0.01f, 1.0f);
        uiLowCreepBlend = AddSlider("LowSpeed Creep Blend", "mlsCb", lowSpeedCreepBlend, 0.05f, 3.0f);
        uiLowShearDamping = AddSlider("LowSpeed Shear Damping", "mlsC", lowSpeedShearDampingC, 0f, 10000f);
        uiAxleSpinDamping = AddSlider("Axle Spin Damping", "axDmp", axleSpinDampingK, 0f, 5000f);
        uiAxleAirDamping = AddSlider("Axle Air Damping", "airDmp", axleAirDampingK, 0f, 100f);

        uiEnableTire = AddToggle("Tire Model", "tire", enableTireModel);

        uiRelax = AddToggle("Tire Relaxation", "relax", enableTireRelaxation);
        uiRelaxL = AddSlider("Relax Length", "relaxL", relaxLength, 0.05f, 5.0f);
        uiShearK = AddSlider("Shear K (N/m)", "shK", shearK, 1000f, 200000f);
        uiShearC = AddSlider("Shear C (N*s/m)", "shC", shearC, 0f, 5000f);
        uiMaxShear = AddSlider("Max Shear Disp", "shMax", maxShearDisp, 0.01f, 0.50f);

        uiDecouple = AddToggle("Decouple F/T", "decouple", decoupleTireForceAndTorque);
        uiDecoupledApply = AddToggle("ADV: Decoupled Apply", "advDec", enableDecoupledTireForceApplication);
        uiNormalGroundReaction = AddToggle("ADV: Normal Reaction", "advNReact", enableNormalGroundReactionForces);

        uiDbg = AddToggle("Debug Master", "dbg", debugDraw);
        uiDbgForce = AddToggle("DebugViz: Tire Force", "dbgF", debugVizTireForce);
        uiDbgTreadAxis = AddToggle("Debug: Tread+Axis", "dbgTA", debugVizTreadAndAxis);

        uiTreadClip = AddToggle("Tread Width Clip", "tw-clip", enableTreadWidthClip);
        uiTreadW = AddSlider("Tread Width", "tw", treadWidth, 0.05f, 5f);
        uiTreadRayFan = AddToggle("ADV: Tread Ray Fan", "advRay", enableTreadRayFan);
        uiTreadRayCount = AddSlider("ADV: Tread Ray Count", "rayN", treadRayCount, 1f, 7f);

        uiLineW = AddSlider("Line Width", "lw", forceLineWidth, 0.01f, 0.30f);
        uiForceScale = AddSlider("Force Scale", "fs", forceToLength, 0.00001f, 0.01f);

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

        uiMaxAngVel = AddSlider("Max Angular Vel (rad/s)", "maxW", maxAngularVelocityLimit, 10f, 1000f);

        uiIncludeLayer0 = AddToggle("Contact Ray: Include Layer 0", "ly0", includeLayer0);

        uiMaxContactPoints = AddSlider("Max Contact Points", "cpN", maxContactPoints, 1f, 6f);

        // NEW: gate + normal filter tuning UI
        uiGateEnable = AddToggle("Contact Gate", "gate", enableContactGate);
        uiGateInFrames = AddSlider("Gate FadeIn (frames)", "gIn", gateFadeInFrames, 1f, 20f);
        uiGateOutFrames = AddSlider("Gate FadeOut (frames)", "gOut", gateFadeOutFrames, 1f, 20f);

        uiNFilterEnable = AddToggle("Normal Filter", "nF", enableNormalFilter);
        uiNFilterAlpha = AddSlider("Normal Filter Alpha", "nFa", normalFilterAlpha, 0.01f, 1.0f);
    }

    public override void OnSimulateStart()
    {
        contacts.Clear();
        fixedStepCounter = 0;
        currentGear = 1;

        pointStates.Clear();
        colStates.Clear();

        Rigidbody.maxAngularVelocity = Mathf.Max(10f, maxAngularVelocityLimit);

        // Capture baseline angularDrag so we can add/remove extra rolling resistance without breaking other physics.
        baseAngularDrag = Rigidbody.angularDrag;
        baseAngularDragCaptured = true;

        treadTriggerCapsule = FindTreadTriggerCapsule();

        contactRayMask = BuildContactRayMask();

        if (ShowDebugVisuals && debugDraw)
            EnsureDebugObjects();
    }

    public override void OnSimulateStop()
    {
        contacts.Clear();
        pointStates.Clear();
        colStates.Clear();

        DestroyDebugObjects();
        treadTriggerCapsule = null;

        // Restore baseline angular drag if we modified it.
        if (HasRigidbody && baseAngularDragCaptured)
            Rigidbody.angularDrag = baseAngularDrag;
    }

    public override void OnSimulateTriggerEnter(Collider other)
    {
        if (other == null) return;
        if (!IsColliderInContactLayers(other)) return;
        contacts.Add(other);
    }

    public override void OnSimulateTriggerExit(Collider other)
    {
        if (other == null) return;
        contacts.Remove(other);
    }

    public override void SimulateFixedUpdateAlways()
    {
        fixedStepCounter++;

        if (!IsSimulating || !HasRigidbody) return;

        SyncParamsFromUI();

        contactRayMask = BuildContactRayMask();
        Rigidbody.maxAngularVelocity = Mathf.Max(10f, maxAngularVelocityLimit);

        // Legacy rolling damping uses angularDrag. The advanced mode restores angularDrag and applies load-based torque per contact.
        ApplyRollingAngularDrag(contacts.Count > 0 && !useLoadSensitiveRollingResistance);

        // ===== 0) Drive/Brake (remappable keys) =====
        {
            float dt = Time.fixedDeltaTime;

            bool heldThr = enableDriveBrake && uiKeyThrottle != null && uiKeyThrottle.IsHeld;
            bool heldBrk = enableDriveBrake && uiKeyBrake != null && uiKeyBrake.IsHeld;
            bool heldRev = enableDriveBrake && uiKeyReverse != null && uiKeyReverse.IsHeld;
            UpdateGearboxInput();

            float targetThr = heldThr ? 1f : 0f;
            float targetBrk = heldBrk ? 1f : 0f;

            float thrRate = (targetThr > throttle01) ? Mathf.Max(0f, throttleRise) : Mathf.Max(0f, throttleFall);
            float brkRate = (targetBrk > brake01) ? Mathf.Max(0f, brakeRise) : Mathf.Max(0f, brakeFall);
            throttle01 = Mathf.MoveTowards(throttle01, targetThr, thrRate * dt);
            brake01 = Mathf.MoveTowards(brake01, targetBrk, brkRate * dt);

            Vector3 aAxis = GetDriveAxisWorld();
            float omegaAxis = Vector3.Dot(Rigidbody.angularVelocity, aAxis);

            float gearRatio = GetCurrentGearRatio();
            float engineTorqueAtWheel = maxDriveTorque * gearRatio;
            float tauDrive = throttle01 * engineTorqueAtWheel;

            float flipSign = Flipped ? -1f : 1f;
            float userSign = invertDriveTorque ? -1f : 1f;
            float revSign = heldRev ? -1f : 1f;
            float driveSign = flipSign * userSign * revSign;

            // ---- Drive torque (throttle) ----
            float tauDriveCmd = tauDrive * driveSign;

            // 恒功率约束：|tau_drive * omega| <= maxDrivePower
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
                {
                    tau += -Mathf.Sign(omegaAxis) * tauBMax;
                }
                else
                {
                    float tauHold = -omegaAxis * brakeHoldK;
                    tau += Mathf.Clamp(tauHold, -tauBMax, tauBMax);
                }
            }


            if (Mathf.Abs(tau) > 1e-6f)
                Rigidbody.AddTorque(aAxis * tau, ForceMode.Force);
        }

        if (contacts.Count == 0)
        {
            ApplyAxleSpinStabilization(0f, 0f, GetDriveAxisWorld());
            if (resetShearOnNoContact) pointStates.Clear();
            // gate 状态也要衰减/清理（避免突然恢复时跳变）
            DecayAndCleanupColliderStates();
            HideDebugObjects();
            return;
        }

        // ===== 轮心/半径（世界）=====
        Vector3 center;
        float R;
        if (treadTriggerCapsule != null)
        {
            center = treadTriggerCapsule.transform.TransformPoint(treadTriggerCapsule.center);
            float s = MaxAbsComponent(treadTriggerCapsule.transform.lossyScale);
            R = treadTriggerCapsule.radius * s;
        }
        else
        {
            center = transform.position;
            R = 1.0f;
        }

        // ===== 轮轴（世界）=====
        Vector3 aAxisWheel = GetWheelAxisWorld().normalized;

        // ===== 射线方向：轮子径向朝地面（gravity 投影到垂直轮轴平面）=====
        Vector3 g = (Physics.gravity.sqrMagnitude > 1e-8f) ? Physics.gravity.normalized : Vector3.down;
        Vector3 downDir = ProjectOnPlane(g, aAxisWheel);
        if (downDir.sqrMagnitude < 1e-8f) downDir = -transform.up;
        else downDir.Normalize();

        // ===== 踏面裁切参数（世界）=====
        Vector3 axisWorld = Vector3.right;
        float halfW = 0f;
        bool doClip = enableTreadWidthClip && treadWidth > 0f;

        if (doClip)
        {
            if (treadTriggerCapsule != null)
            {
                Transform t = treadTriggerCapsule.transform;
                switch (treadTriggerCapsule.direction)
                {
                    case 0: axisWorld = t.right; break;
                    case 1: axisWorld = t.up; break;
                    case 2: axisWorld = t.forward; break;
                }

                Vector3 s = t.lossyScale;
                float axisScale =
                    (treadTriggerCapsule.direction == 0) ? Mathf.Abs(s.x) :
                    (treadTriggerCapsule.direction == 1) ? Mathf.Abs(s.y) :
                                                           Mathf.Abs(s.z);

                axisWorld.Normalize();
                halfW = 0.5f * treadWidth * axisScale;
            }
            else
            {
                axisWorld = aAxisWheel;
                halfW = 0.5f * treadWidth * GetAxisScaleWorld();
            }
        }

        // ===== 1) Gather aggregated contact samples, then take Top-N colliders =====
        int N = Mathf.Clamp(maxContactPoints, 1, 6);
        GatherTopContactSamples(center, downDir, R, doClip, axisWorld, halfW, N, topSamples);

        // 标记所有状态“本帧未见”
        MarkAllColliderStatesUnseen();

        if (topSamples.Count == 0)
        {
            // 没命中：全部衰减
            DecayAndCleanupColliderStates();
            dbgHasLocal = false;
            CleanupPointStates();
            return;
        }

        // ===== 2) Update per-collider gate + normal filter based on current contacts =====
        UpdateColliderStatesFromSamples(topSamples);

        // ===== 3) Apply forces per point =====
        float dtFixed = Time.fixedDeltaTime;

        // Debug：在 FixedUpdate 里采样“要画什么”，并用 Rigidbody local space 存下来。
        // LateUpdate 用插值后的 Pose 还原到世界坐标画出来。
        bool doDbgSample = ShowDebugVisuals && debugDraw && (fixedStepCounter % Mathf.Max(1, drawEveryFixedSteps) == 0);
        Quaternion rbInvRot = Quaternion.identity;
        Vector3 rbPos = Vector3.zero;
        if (doDbgSample)
        {
            rbPos = Rigidbody.position;
            rbInvRot = Quaternion.Inverse(Rigidbody.rotation);

            EnsureDebugLocalCapacity(topSamples.Count);
            dbgLocalCount = 0;
            dbgCenterLocal = rbInvRot * (center - rbPos);
            dbgAxisLocal = rbInvRot * axisWorld;
            dbgHalfWWorld = halfW;
            dbgRadiusWorld = R;
            dbgDoClip = doClip;
        }

        // ---- Per-rigidbody sample weighting ----
        // 修复：不要用 1/maxContactPoints 这种常数缩放；而是按“实际命中数”分配。
        // 规则：同一个 attachedRigidbody(或静态=0) 的所有接触点权重和为 1。
        tmpRbSampleCounts.Clear();
        for (int i = 0; i < topSamples.Count; i++)
        {
            var ss = topSamples[i];
            int rbId = 0;
            if (ss.col != null && ss.col.attachedRigidbody != null) rbId = ss.col.attachedRigidbody.GetInstanceID();
            int c;
            tmpRbSampleCounts.TryGetValue(rbId, out c);
            tmpRbSampleCounts[rbId] = c + 1;
        }

        float totalNormalLoadForAxle = 0f;

        for (int i = 0; i < topSamples.Count; i++)
        {
            var s = topSamples[i];

            float gate = 1f;
            Vector3 nUse = s.n;

            ColliderContactState cs;
            if (colStates.TryGetValue(s.colId, out cs) && cs != null)
            {
                if (enableContactGate) gate = Mathf.Clamp01(cs.gate);

                if (enableNormalFilter)
                {
                    nUse = cs.nFiltered;
                    // 强制与当前 raw normal 同向，避免 180° 翻转
                    if (Vector3.Dot(nUse, s.n) < 0f) nUse = -nUse;
                    if (nUse.sqrMagnitude > 1e-12f) nUse.Normalize();
                    else nUse = s.n;
                }
            }

            if (gate <= 1e-4f) continue;

            // per-rigidbody weight (sum=1 per rigidbody / static body)
            int rbId = 0;
            if (s.col != null && s.col.attachedRigidbody != null) rbId = s.col.attachedRigidbody.GetInstanceID();
            int rbCount = 1;
            tmpRbSampleCounts.TryGetValue(rbId, out rbCount);
            if (rbCount <= 0) rbCount = 1;
            float sampleWeight = 1f / (float)rbCount;

            Rigidbody groundRb = (s.col != null) ? s.col.attachedRigidbody : null;
            if (groundRb == Rigidbody) groundRb = null;

            // ---- Normal force ----
            // Legacy spring is explicit; the stable support branch adds a velocity-level ERP impulse so high
            // apparent support stiffness does not require raising k beyond what a 100 Hz integrator can handle.
            Vector3 vAtP = Rigidbody.GetPointVelocity(s.p);
            Vector3 vGroundN = (groundRb != null) ? groundRb.GetPointVelocity(s.p) : Vector3.zero;
            float vRelN = Vector3.Dot(vAtP - vGroundN, nUse); // positive means separating

            float FnRaw = springK * s.pen;

            if (damperC > 0f)
            {
                float compressionSpeed = -vRelN;
                if (compressionSpeed > 0f) FnRaw += damperC * compressionSpeed;
            }

            float FnStable = BuildStableNormalSupportForce(s.pen, vRelN, s.p, nUse, groundRb);
            if (FnStable > FnRaw) FnRaw = FnStable;

            FnRaw = Mathf.Clamp(FnRaw, 0f, maxNormalForce);

            // Normal load applied to this contact. Legacy tire force below can still apply the old extra gate/weight pass
            // unless ADV: Single Load Scale is enabled.
            float Fn = FnRaw * gate * sampleWeight;
            if (Fn <= 1e-6f) continue;

            Vector3 FnVec = Fn * nUse;
            Rigidbody.AddForceAtPosition(FnVec, s.p, ForceMode.Force);
            if (enableNormalGroundReactionForces && groundRb != null)
                groundRb.AddForceAtPosition(-FnVec, s.p, ForceMode.Force);

            totalNormalLoadForAxle += Fn;

            ApplyLoadSensitiveRollingResistance(Fn, R, aAxisWheel);

            // ---- Tire friction per point（可选：enableTireModel）----
            Vector3 Ftire = Vector3.zero;

            if (enableTireModel)
            {
                Vector3 vGround = Vector3.zero;
                if (groundRb != null) vGround = groundRb.GetPointVelocity(s.p);

                Vector3 vWheel = Rigidbody.GetPointVelocity(s.p);
                Vector3 vRel = vWheel - vGround;

                // 切向基：f=滚动/驱动方向，side=横向方向（合力圆/椭圆只在高级模式使用）
                Vector3 f = ProjectOnPlane(Vector3.Cross(nUse, aAxisWheel), nUse);
                if (f.sqrMagnitude < 1e-6f)
                    f = ProjectOnPlane(Vector3.Cross(nUse, transform.right), nUse);

                if (f.sqrMagnitude > 1e-6f)
                {
                    f.Normalize();

                    Vector3 side = ProjectOnPlane(aAxisWheel, nUse);
                    if (side.sqrMagnitude < 1e-6f)
                        side = ProjectOnPlane(Vector3.Cross(f, nUse), nUse);
                    if (side.sqrMagnitude > 1e-6f)
                    {
                        side.Normalize();
                        if (Vector3.Dot(side, aAxisWheel) < 0f) side = -side;
                    }
                    else
                    {
                        side = Vector3.zero;
                    }

                    Vector3 vSlip = ProjectOnPlane(vRel, nUse);
                    float vSlipMag = vSlip.magnitude;

                    if (enableTireRelaxation)
                    {
                        TirePointState st = GetOrCreatePointState(s.col, s.p);
                        st.lastSeenStep = fixedStepCounter;

                        if (enableSinglePassLoadScaling || enableCombinedSlipFriction || enableModernLowSpeedTire)
                            st.shearDispWorld = ProjectOnPlane(st.shearDispWorld, nUse);

                        float speedFloor = enableModernLowSpeedTire ? Mathf.Max(vEps, lowSpeedRelaxSpeedFloor) : vEps;
                        float speed = Mathf.Max(vSlipMag, speedFloor);
                        float T = Mathf.Max(1e-4f, relaxLength / speed);

                        // Modern low-speed branch: bristle leak fades out near zero slip, so static deflection can hold.
                        float leakScale = 1f;
                        if (enableModernLowSpeedTire)
                        {
                            float leakV = Mathf.Max(1e-4f, lowSpeedCreepSpeed);
                            leakScale = Mathf.Clamp01(vSlipMag / leakV);
                        }

                        Vector3 xDot = vSlip - st.shearDispWorld * (leakScale / T);
                        st.shearDispWorld += xDot * dtFixed;

                        if (maxShearDisp > 1e-5f)
                        {
                            float m = st.shearDispWorld.magnitude;
                            if (m > maxShearDisp) st.shearDispWorld = st.shearDispWorld * (maxShearDisp / m);
                        }

                        Vector3 Fraw = -shearK * st.shearDispWorld;
                        float shearDamping = shearC;
                        if (enableModernLowSpeedTire) shearDamping = Mathf.Max(shearDamping, lowSpeedShearDampingC);
                        if (shearDamping > 0f) Fraw += -shearDamping * xDot;

                        bool staticZone = vSlipMag < Mathf.Max(1e-3f, vStatic);

                        if (enableModernLowSpeedTire)
                        {
                            Vector3 Fcreep = BuildLowSpeedCreepForce(vSlip, f, side, Fn);
                            float blendV = Mathf.Max(1e-4f, lowSpeedCreepBlend);
                            float creepBlend = 1f - Mathf.Clamp01(vSlipMag / blendV);
                            creepBlend = creepBlend * creepBlend * (3f - 2f * creepBlend);
                            Fraw += Fcreep * creepBlend;
                        }

                        if (enableCombinedSlipFriction && side.sqrMagnitude > 1e-6f)
                        {
                            float longScale = Mathf.Max(0f, longitudinalGripScale);
                            float latScale = Mathf.Max(0f, lateralGripScale);
                            float scale = 1f;

                            if (staticZone)
                            {
                                scale = GetCombinedFrictionScale(Fraw, f, side, muStatic * longScale * Fn, muStatic * latScale * Fn);
                                if (scale < 0.9999f)
                                    scale = GetCombinedFrictionScale(Fraw, f, side, muKinetic * longScale * Fn, muKinetic * latScale * Fn);
                            }
                            else
                            {
                                scale = GetCombinedFrictionScale(Fraw, f, side, muKinetic * longScale * Fn, muKinetic * latScale * Fn);
                            }

                            if (scale < 0.9999f)
                            {
                                Fraw *= scale;
                                st.shearDispWorld *= scale;
                            }
                        }
                        else
                        {
                            float FmaxS = muStatic * Fn;   // 注意：Fn 已乘 gate/sampleWeight
                            float FmaxK = muKinetic * Fn;  // 注意：Fn 已乘 gate/sampleWeight
                            float Fmag = Fraw.magnitude;

                            if (staticZone)
                            {
                                if (Fmag > FmaxS && Fmag > 1e-6f)
                                {
                                    Fraw *= (FmaxK / Fmag);
                                    st.shearDispWorld = -Fraw / Mathf.Max(1e-6f, shearK);
                                }
                            }
                            else
                            {
                                if (Fmag > FmaxK && Fmag > 1e-6f)
                                {
                                    Fraw *= (FmaxK / Fmag);
                                    st.shearDispWorld = -Fraw / Mathf.Max(1e-6f, shearK);
                                }
                            }
                        }

                        if (forceFilterTau > 1e-5f)
                        {
                            float aFilt = dtFixed / (forceFilterTau + dtFixed);
                            st.FtireFiltered = Vector3.Lerp(st.FtireFiltered, Fraw, aFilt);
                            Ftire = st.FtireFiltered;
                        }
                        else
                        {
                            st.FtireFiltered = Fraw;
                            Ftire = Fraw;
                        }

                        if (enableSinglePassLoadScaling)
                        {
                            if (enableCombinedSlipFriction && side.sqrMagnitude > 1e-6f)
                            {
                                float postMu = staticZone ? muStatic : muKinetic;
                                Ftire = ClampCombinedTireForce(
                                    Ftire, f, side,
                                    postMu * Mathf.Max(0f, longitudinalGripScale) * Fn,
                                    postMu * Mathf.Max(0f, lateralGripScale) * Fn);
                            }
                            else
                            {
                                float postMu = staticZone ? muStatic : muKinetic;
                                Ftire = LimitVectorMagnitude(Ftire, postMu * Fn);
                            }
                        }
                    }
                    else
                    {
                        if (vSlipMag > 1e-5f)
                        {
                            if (enableCombinedSlipFriction && side.sqrMagnitude > 1e-6f)
                            {
                                Ftire = BuildCombinedKineticFriction(
                                    vSlip, f, side,
                                    muKinetic * Mathf.Max(0f, longitudinalGripScale) * Fn,
                                    muKinetic * Mathf.Max(0f, lateralGripScale) * Fn);
                            }
                            else
                            {
                                float FmaxK = muKinetic * Fn; // Fn 已乘 gate/sampleWeight
                                Ftire = -vSlip / vSlipMag * FmaxK;
                            }
                        }
                        else
                        {
                            Ftire = Vector3.zero;
                        }
                    }

                    // Legacy behavior: tire force was gated/weighted once through Fn, then again here.
                    // ADV: Single Load Scale disables this second pass for physically correct load accounting.
                    if (!enableSinglePassLoadScaling)
                    {
                        Ftire *= gate;
                        Ftire *= sampleWeight;
                    }

                    if (Ftire.sqrMagnitude > 1e-10f)
                    {
                        ApplyTireForce(Ftire, s.p, groundRb, aAxisWheel);
                    }
                }
            }

            // ---- Debug 采样（每个接触点分别画 Fn / Ft / normal / contact point）----
            if (doDbgSample && dbgLocalCount < dbgLocalContacts.Length)
            {
                DebugContactLocalData d;
                d.active = true;
                d.pLocal = rbInvRot * (s.p - rbPos);
                d.nWorld = nUse;
                d.FnMag = Fn;
                d.FtWorld = Ftire;
                d.FtMag = Ftire.magnitude;
                dbgLocalContacts[dbgLocalCount] = d;
                dbgLocalCount++;
            }

        }

        ApplyAxleSpinStabilization(totalNormalLoadForAxle, R, aAxisWheel);

        // Optional drive torque test hook
        if (Mathf.Abs(driveTorque) > 1e-6f)
        {
            float flipSign = Flipped ? -1f : 1f;
            Rigidbody.AddTorque(GetDriveAxisWorld() * (driveTorque * flipSign), ForceMode.Force);
        }

        // Debug：本帧是否更新了“本地空间”绘制数据
        if (doDbgSample)
        {
            dbgHasLocal = (dbgLocalCount > 0);
        }

        // 本帧结束：对未见 collider 衰减 gate，并清理
        DecayAndCleanupColliderStates();
        CleanupPointStates();
    }

    public override void SimulateLateUpdateAlways()
    {
        // 只负责渲染：用当前 Rigidbody 的插值 Pose，把 FixedUpdate 缓存的 local 数据还原到世界坐标
        if (!IsSimulating || !HasRigidbody)
        {
            dbgHasLocal = false;
            HideDebugObjects();
            return;
        }

        // Besiege 内置开关 + 我们自己的总开关
        if (!ShowDebugVisuals || !debugDraw)
        {
            HideDebugObjects();
            return;
        }

        if (!dbgHasLocal || dbgLocalCount <= 0)
        {
            HideDebugObjects();
            return;
        }

        EnsureDebugObjects();
        EnsureDebugPointPool(Mathf.Max(dbgLocalCount, Mathf.Clamp(maxContactPoints, 1, 12)));
        ShowDebugObjects();

        Vector3 rbPos = Rigidbody.position;
        Quaternion rbRot = Rigidbody.rotation;

        Vector3 center = rbPos + rbRot * dbgCenterLocal;
        Vector3 axisWorld = rbRot * dbgAxisLocal;
        if (axisWorld.sqrMagnitude > 1e-12f) axisWorld.Normalize();
        else axisWorld = GetWheelAxisWorld();

        bool showForceViz = debugVizTireForce;

        // points
        for (int i = 0; i < dbgPoints.Count; i++)
        {
            bool active = (i < dbgLocalCount) && dbgLocalContacts[i].active;
            DebugPointViz pv = dbgPoints[i];
            if (pv == null || pv.root == null) continue;
            pv.root.SetActive(active);
            if (!active) continue;

            DebugContactLocalData d = dbgLocalContacts[i];

            Vector3 p = rbPos + rbRot * d.pLocal;
            Vector3 n = d.nWorld;
            if (n.sqrMagnitude > 1e-12f) n.Normalize();
            else n = Vector3.up;

            if (pv.sphere != null) pv.sphere.transform.position = p;

            // center -> point
            if (pv.lrCenterToP != null)
            {
                pv.lrCenterToP.enabled = true;
                SetLine(pv.lrCenterToP, center, p);
            }

            // normal dir
            if (pv.lrNormal != null)
            {
                pv.lrNormal.enabled = true;
                SetLine(pv.lrNormal, p, p + n * normalArrowLen);
            }

            // normal force (support)
            if (pv.lrForceN != null)
            {
                pv.lrForceN.enabled = true;
                float fnLen = Mathf.Clamp(d.FnMag * forceToLength, arrowMinLen, arrowMaxLen);
                SetLine(pv.lrForceN, p, p + n * fnLen);
            }

            // tangential force (friction)
            if (pv.lrForceT != null)
            {
                if (showForceViz && d.FtMag > 1e-6f)
                {
                    Vector3 ftW = d.FtWorld;
                    float m = ftW.magnitude;
                    if (m > 1e-6f)
                    {
                        Vector3 tDir = ftW / m;
                        float ftLen = Mathf.Clamp(d.FtMag * forceToLength, arrowMinLen, arrowMaxLen);
                        pv.lrForceT.enabled = true;
                        SetLine(pv.lrForceT, p, p + tDir * ftLen);
                    }
                    else
                    {
                        pv.lrForceT.enabled = false;
                    }
                }
                else
                {
                    pv.lrForceT.enabled = false;
                }
            }
        }

        // tread / axis clip viz (单份)
        if (debugVizTreadAndAxis && dbgDoClip)
        {
            SetLRVisible(lrTreadAxis, true);
            SetLRVisible(lrTreadRingA, true);
            SetLRVisible(lrTreadRingB, true);
            DrawTreadRangeDebug(center, axisWorld, dbgHalfWWorld, dbgRadiusWorld);
        }
        else
        {
            SetLRVisible(lrTreadAxis, false);
            SetLRVisible(lrTreadRingA, false);
            SetLRVisible(lrTreadRingB, false);
        }

        ApplyLineWidthsIfReady();
    }



    // =========================
    // Gearbox
    // =========================
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

    // =========================
    // Stable normal support
    // =========================
    private float BuildStableNormalSupportForce(float penetration, float relativeNormalVelocity, Vector3 point, Vector3 normal, Rigidbody groundRb)
    {
        if (!enableStableNormalSupport) return 0f;
        if (!HasRigidbody) return 0f;
        if (normal.sqrMagnitude < 1e-10f) return 0f;

        float dt = Mathf.Max(1e-5f, Time.fixedDeltaTime);
        float pen = Mathf.Max(0f, penetration - Mathf.Max(0f, normalSupportSlop));

        // Baumgarte/ERP target: correct only a fraction of penetration per fixed step.
        // This is a velocity target, not a larger explicit k, so it remains stable at 100 Hz.
        float targetVN = pen * Mathf.Clamp01(normalSupportERP) / dt;
        float dv = targetVN - relativeNormalVelocity * Mathf.Max(0f, normalSupportVelDamping);
        if (dv <= 1e-5f) return 0f;

        float effectiveMass = GetEffectiveNormalMass(point, normal, groundRb);
        effectiveMass *= Mathf.Max(0.01f, normalSupportMassScale);

        return effectiveMass * dv / dt;
    }

    private float GetEffectiveNormalMass(Vector3 point, Vector3 normal, Rigidbody groundRb)
    {
        float invMass = GetPointInverseMassAlongAxis(Rigidbody, point, normal);
        if (groundRb != null) invMass += GetPointInverseMassAlongAxis(groundRb, point, normal);

        if (invMass > 1e-6f) return 1f / invMass;
        if (HasRigidbody && Rigidbody.mass > 1e-6f) return Rigidbody.mass;
        return 1f;
    }

    private float GetPointInverseMassAlongAxis(Rigidbody rb, Vector3 point, Vector3 axisWorld)
    {
        if (rb == null) return 0f;
        if (rb.isKinematic) return 0f;
        if (axisWorld.sqrMagnitude < 1e-10f) return 0f;

        axisWorld.Normalize();

        float invMass = 0f;
        if (rb.mass > 1e-6f) invMass = 1f / rb.mass;

        Vector3 r = point - rb.worldCenterOfMass;
        Vector3 rn = Vector3.Cross(r, axisWorld);
        Vector3 invIrn = MultiplyWorldInverseInertia(rb, rn);
        float angular = Vector3.Dot(Vector3.Cross(invIrn, r), axisWorld);
        if (angular < 0f) angular = 0f;

        return invMass + angular;
    }

    private Vector3 MultiplyWorldInverseInertia(Rigidbody rb, Vector3 v)
    {
        if (rb == null) return Vector3.zero;

        Quaternion principalToWorld = rb.rotation * rb.inertiaTensorRotation;
        Vector3 vp = Quaternion.Inverse(principalToWorld) * v;
        Vector3 I = rb.inertiaTensor;

        vp.x = (I.x > 1e-6f) ? vp.x / I.x : 0f;
        vp.y = (I.y > 1e-6f) ? vp.y / I.y : 0f;
        vp.z = (I.z > 1e-6f) ? vp.z / I.z : 0f;

        return principalToWorld * vp;
    }
    // =========================
    // Rolling damping: legacy angularDrag, or ADV load-sensitive wheel-axis torque
    // =========================
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

        // Only set when changed to reduce churn
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

    private void ApplyTireForce(Vector3 force, Vector3 point, Rigidbody groundRb, Vector3 wheelAxisWorld)
    {
        if (force.sqrMagnitude <= 1e-12f) return;

        bool useDecoupled = enableDecoupledTireForceApplication && decoupleTireForceAndTorque;
        if (useDecoupled)
        {
            Rigidbody.AddForce(force, ForceMode.Force);

            if (wheelAxisWorld.sqrMagnitude > 1e-10f)
            {
                wheelAxisWorld.Normalize();
                Vector3 tau = Vector3.Cross(point - Rigidbody.worldCenterOfMass, force);
                float spinTau = Vector3.Dot(tau, wheelAxisWorld);
                if (Mathf.Abs(spinTau) > 1e-6f)
                    Rigidbody.AddTorque(wheelAxisWorld * spinTau, ForceMode.Force);
            }
        }
        else
        {
            Rigidbody.AddForceAtPosition(force, point, ForceMode.Force);
        }

        // Preserve legacy tire reaction behavior: tangential custom tire force always pushes dynamic ground back.
        if (groundRb != null)
            groundRb.AddForceAtPosition(-force, point, ForceMode.Force);
    }

    private Vector3 LimitVectorMagnitude(Vector3 v, float maxMagnitude)
    {
        if (maxMagnitude <= 0f) return Vector3.zero;
        float sqr = v.sqrMagnitude;
        float maxSqr = maxMagnitude * maxMagnitude;
        if (sqr <= maxSqr || sqr <= 1e-12f) return v;
        return v * (maxMagnitude / Mathf.Sqrt(sqr));
    }

    private float GetCombinedFrictionScale(Vector3 force, Vector3 forward, Vector3 side, float maxLong, float maxSide)
    {
        if (force.sqrMagnitude <= 1e-12f) return 1f;

        float fx = Vector3.Dot(force, forward);
        float fy = Vector3.Dot(force, side);
        float usage = 0f;

        if (maxLong > 1e-6f) usage += (fx * fx) / (maxLong * maxLong);
        else if (Mathf.Abs(fx) > 1e-6f) return 0f;

        if (maxSide > 1e-6f) usage += (fy * fy) / (maxSide * maxSide);
        else if (Mathf.Abs(fy) > 1e-6f) return 0f;

        if (usage <= 1f) return 1f;
        return 1f / Mathf.Sqrt(usage);
    }

    private Vector3 ClampCombinedTireForce(Vector3 force, Vector3 forward, Vector3 side, float maxLong, float maxSide)
    {
        float fx = Vector3.Dot(force, forward);
        float fy = Vector3.Dot(force, side);
        Vector3 tangentForce = forward * fx + side * fy;
        float scale = GetCombinedFrictionScale(tangentForce, forward, side, maxLong, maxSide);
        return tangentForce * scale;
    }

    private Vector3 BuildCombinedKineticFriction(Vector3 vSlip, Vector3 forward, Vector3 side, float maxLong, float maxSide)
    {
        Vector3 demand = Vector3.zero;

        float vx = Vector3.Dot(vSlip, forward);
        if (Mathf.Abs(vx) > 1e-5f && maxLong > 0f)
            demand += -Mathf.Sign(vx) * maxLong * forward;

        float vy = Vector3.Dot(vSlip, side);
        if (Mathf.Abs(vy) > 1e-5f && maxSide > 0f)
            demand += -Mathf.Sign(vy) * maxSide * side;

        return ClampCombinedTireForce(demand, forward, side, maxLong, maxSide);
    }

    private Vector3 BuildLowSpeedCreepForce(Vector3 vSlip, Vector3 forward, Vector3 side, float normalLoad)
    {
        if (!enableModernLowSpeedTire) return Vector3.zero;
        if (normalLoad <= 1e-6f) return Vector3.zero;

        float creepV = Mathf.Max(1e-4f, lowSpeedCreepSpeed);

        if (enableCombinedSlipFriction && side.sqrMagnitude > 1e-6f)
        {
            float maxLong = muStatic * Mathf.Max(0f, longitudinalGripScale) * normalLoad;
            float maxSide = muStatic * Mathf.Max(0f, lateralGripScale) * normalLoad;

            Vector3 demand = Vector3.zero;
            float vx = Vector3.Dot(vSlip, forward);
            if (Mathf.Abs(vx) > 1e-6f && maxLong > 0f)
                demand += -Mathf.Clamp(vx / creepV, -1f, 1f) * maxLong * forward;

            float vy = Vector3.Dot(vSlip, side);
            if (Mathf.Abs(vy) > 1e-6f && maxSide > 0f)
                demand += -Mathf.Clamp(vy / creepV, -1f, 1f) * maxSide * side;

            return ClampCombinedTireForce(demand, forward, side, maxLong, maxSide);
        }

        float vMag = vSlip.magnitude;
        if (vMag <= 1e-6f) return Vector3.zero;

        float maxForce = muStatic * normalLoad;
        float demandMag = maxForce * Mathf.Clamp01(vMag / creepV);
        return -vSlip / vMag * demandMag;
    }

    private void ApplyAxleSpinStabilization(float normalLoad, float radius, Vector3 wheelAxisWorld)
    {
        if (!enableModernLowSpeedTire) return;
        if (!HasRigidbody) return;
        if (wheelAxisWorld.sqrMagnitude < 1e-10f) return;

        wheelAxisWorld.Normalize();

        float omega = Vector3.Dot(Rigidbody.angularVelocity, wheelAxisWorld);
        if (Mathf.Abs(omega) <= 1e-5f) return;

        bool hasLoad = normalLoad > 1e-6f && radius > 1e-6f;
        float dampingK = hasLoad ? axleSpinDampingK : axleAirDampingK;
        if (dampingK <= 0f) return;

        float tau = -omega * dampingK;
        if (hasLoad)
        {
            float tauLimit = Mathf.Max(0f, muStatic) * normalLoad * radius;
            if (tauLimit > 1e-6f)
                tau = Mathf.Clamp(tau, -tauLimit, tauLimit);
        }

        float inertia = GetInertiaAroundWorldAxis(wheelAxisWorld);
        float dt = Mathf.Max(1e-5f, Time.fixedDeltaTime);
        if (inertia > 1e-6f)
        {
            float stopTorque = Mathf.Abs(omega) * inertia / dt;
            if (stopTorque > 1e-6f)
                tau = Mathf.Clamp(tau, -stopTorque, stopTorque);
        }

        if (Mathf.Abs(tau) > 1e-6f)
            Rigidbody.AddTorque(wheelAxisWorld * tau, ForceMode.Force);
    }

    private float GetInertiaAroundWorldAxis(Vector3 axisWorld)
    {
        if (!HasRigidbody) return 0f;
        if (axisWorld.sqrMagnitude < 1e-10f) return 0f;
        axisWorld.Normalize();

        Quaternion principalToWorld = Rigidbody.rotation * Rigidbody.inertiaTensorRotation;
        Vector3 axisPrincipal = Quaternion.Inverse(principalToWorld) * axisWorld;
        if (axisPrincipal.sqrMagnitude > 1e-10f) axisPrincipal.Normalize();

        Vector3 I = Rigidbody.inertiaTensor;
        float ix = Mathf.Max(0f, I.x);
        float iy = Mathf.Max(0f, I.y);
        float iz = Mathf.Max(0f, I.z);

        return ix * axisPrincipal.x * axisPrincipal.x
             + iy * axisPrincipal.y * axisPrincipal.y
             + iz * axisPrincipal.z * axisPrincipal.z;
    }

    private void RecycleHitLists()
    {
        foreach (var kv in hitsByCol)
        {
            List<HitSample> list = kv.Value;
            if (list == null) continue;
            list.Clear();
            hitListPool.Add(list);
        }
        hitsByCol.Clear();
    }

    private List<HitSample> GetHitListFromPool()
    {
        int last = hitListPool.Count - 1;
        if (last >= 0)
        {
            List<HitSample> list = hitListPool[last];
            hitListPool.RemoveAt(last);
            return list;
        }
        return new List<HitSample>(8);
    }

    // =========================
    // Contact gather (per collider aggregation)
    // =========================
    private void GatherTopContactSamples(
        Vector3 center, Vector3 downDir, float R,
        bool doClip, Vector3 axisWorldUnit, float halfWWorld,
        int N, List<ContactSample> outTop)
    {
        outTop.Clear();
        RecycleHitLists();

        if (!useRaycastContact) return;

        float maxDist = R + Mathf.Max(0f, rayExtra);

        int rayCount = 1;
        bool useRayFan = enableTreadRayFan && doClip && halfWWorld > 1e-4f && axisWorldUnit.sqrMagnitude > 1e-8f;
        if (useRayFan)
        {
            axisWorldUnit.Normalize();
            rayCount = Mathf.Clamp(treadRayCount, 1, 7);
            if (rayCount > 1 && (rayCount % 2) == 0) rayCount = Mathf.Min(7, rayCount + 1);
        }

        for (int r = 0; r < rayCount; r++)
        {
            float u = 0f;
            if (rayCount > 1) u = -1f + (2f * (float)r) / (float)(rayCount - 1);

            Vector3 rayOrigin = useRayFan ? center + axisWorldUnit * (u * halfWWorld) : center;
            int hitCount = Physics.RaycastNonAlloc(
                rayOrigin,
                downDir,
                raycastHitBuffer,
                maxDist,
                contactRayMask,
                QueryTriggerInteraction.Ignore
            );

            if (hitCount <= 0) continue;
            if (hitCount > raycastHitBuffer.Length) hitCount = raycastHitBuffer.Length;

            // collect hits and group by collider
            for (int i = 0; i < hitCount; i++)
            {
                RaycastHit hit = raycastHitBuffer[i];
                Collider c = hit.collider;
                if (c == null) continue;

                if (!contacts.Contains(c)) continue;
                if (!IsColliderInContactLayers(c)) continue;

                Vector3 p = hit.point;
                Vector3 n = hit.normal;
                if (n.sqrMagnitude < 1e-12f) continue;
                n.Normalize();

                if (doClip && !IsWithinTreadWidth(p, center, axisWorldUnit, halfWWorld))
                    continue;

                float dist = hit.distance;
                if (dist <= 1e-6f) dist = Vector3.Distance(rayOrigin, p);
                float pen = R - dist;
                if (pen <= minPenForContact) continue;

                int id = c.GetInstanceID();

                HitSample hs;
                hs.col = c;
                hs.colId = id;
                hs.p = p;
                hs.n = n;
                hs.pen = pen;

                List<HitSample> list;
                if (!hitsByCol.TryGetValue(id, out list))
                {
                    list = GetHitListFromPool();
                    hitsByCol.Add(id, list);
                }
                list.Add(hs);
            }
        }

        if (hitsByCol.Count == 0) return;

        int K = Mathf.Clamp(perColliderTopK, 1, 16);

        foreach (var kv in hitsByCol)
        {
            List<HitSample> list = kv.Value;
            if (list == null || list.Count == 0) continue;

            list.Sort((a, b) => b.pen.CompareTo(a.pen));
            int take = Mathf.Min(K, list.Count);

            float wsum = 0f;
            Vector3 pAcc = Vector3.zero;
            Vector3 nAcc = Vector3.zero;
            float penAcc = 0f;

            float penMax = list[0].pen;

            for (int i = 0; i < take; i++)
            {
                float w = perColliderWeightByPen ? Mathf.Max(0f, list[i].pen) : 1f;
                wsum += w;
                pAcc += list[i].p * w;
                nAcc += list[i].n * w;
                penAcc += list[i].pen * w;
            }

            if (wsum <= 1e-8f)
                continue;

            Vector3 pAgg = pAcc / wsum;
            Vector3 nAgg = nAcc;
            if (nAgg.sqrMagnitude > 1e-12f) nAgg.Normalize();
            else nAgg = list[0].n;

            float distAgg = Vector3.Distance(center, pAgg);
            float penAgg;
            if (useRayFan && rayCount > 1)
                penAgg = penAcc / wsum;
            else
                penAgg = R - distAgg;

            penAgg = Mathf.Clamp(penAgg, 0f, penMax);

            if (penAgg <= minPenForContact) continue;

            ContactSample agg;
            agg.col = list[0].col;
            agg.colId = list[0].colId;
            agg.p = pAgg;
            agg.n = nAgg;
            agg.pen = penAgg;
            agg.dist = R - penAgg;

            outTop.Add(agg);
        }

        if (outTop.Count == 0) return;

        outTop.Sort((a, b) => b.pen.CompareTo(a.pen));
        if (outTop.Count > N)
            outTop.RemoveRange(N, outTop.Count - N);
    }

    // =========================
    // Collider gate + normal filter state update
    // =========================
    private void MarkAllColliderStatesUnseen()
    {
        foreach (var kv in colStates)
        {
            if (kv.Value != null) kv.Value.seenThisFrame = false;
        }
    }

    private void UpdateColliderStatesFromSamples(List<ContactSample> samplesNow)
    {
        int inFrames = Mathf.Max(1, gateFadeInFrames);
        int outFrames = Mathf.Max(1, gateFadeOutFrames);

        float gateUp = 1f / (float)inFrames;
        float gateDn = 1f / (float)outFrames;

        float alpha = Mathf.Clamp01(normalFilterAlpha);

        for (int i = 0; i < samplesNow.Count; i++)
        {
            ContactSample s = samplesNow[i];
            int id = s.colId;

            ColliderContactState st;
            if (!colStates.TryGetValue(id, out st) || st == null)
            {
                st = new ColliderContactState();
                st.gate = 0f;
                st.nFiltered = s.n;
                st.lastSeenStep = fixedStepCounter;
                st.seenThisFrame = true;
                colStates[id] = st;
            }

            st.seenThisFrame = true;
            st.lastSeenStep = fixedStepCounter;

            if (enableContactGate)
                st.gate = Mathf.Clamp01(st.gate + gateUp);
            else
                st.gate = 1f;

            if (enableNormalFilter)
            {
                // 初始化保护
                if (st.nFiltered.sqrMagnitude < 1e-10f) st.nFiltered = s.n;

                // 防止 slerp 走“长弧”导致翻转：先保证同向
                Vector3 nTarget = s.n;
                if (Vector3.Dot(st.nFiltered, nTarget) < 0f) nTarget = -nTarget;

                st.nFiltered = Vector3.Slerp(st.nFiltered, nTarget, alpha);
                if (st.nFiltered.sqrMagnitude > 1e-12f) st.nFiltered.Normalize();
                else st.nFiltered = nTarget;
            }
            else
            {
                st.nFiltered = s.n;
            }
        }

        // 未出现的在 DecayAndCleanupColliderStates() 里统一衰减
    }

    private void DecayAndCleanupColliderStates()
    {
        if (colStates.Count == 0) return;

        int outFrames = Mathf.Max(1, gateFadeOutFrames);
        float gateDn = 1f / (float)outFrames;

        colStatesToRemove.Clear();

        foreach (var kv in colStates)
        {
            int id = kv.Key;
            ColliderContactState st = kv.Value;
            if (st == null) { colStatesToRemove.Add(id); continue; }

            if (!st.seenThisFrame)
            {
                if (enableContactGate)
                    st.gate = Mathf.Clamp01(st.gate - gateDn);
                else
                    st.gate = 0f; // gate 关闭时未见就直接清零
            }

            // gate=0 且过 TTL：清理
            int ttl = Mathf.Max(1, colliderStateTTLSteps);
            if (st.gate <= 1e-5f && (fixedStepCounter - st.lastSeenStep) > ttl)
            {
                colStatesToRemove.Add(id);
            }
        }

        for (int i = 0; i < colStatesToRemove.Count; i++)
            colStates.Remove(colStatesToRemove[i]);
    }

    // =========================
    // Per-point state helpers
    // =========================
    private TirePointState GetOrCreatePointState(Collider col, Vector3 worldPoint)
    {
        ContactKey k = MakeContactKey(col, worldPoint);
        TirePointState st;
        if (!pointStates.TryGetValue(k, out st))
        {
            st = new TirePointState();
            st.lastSeenStep = fixedStepCounter;
            pointStates.Add(k, st);
        }
        return st;
    }

    private ContactKey MakeContactKey(Collider col, Vector3 worldPoint)
    {
        int id = (col != null) ? col.GetInstanceID() : 0;
        Vector3 lp = (col != null) ? col.transform.InverseTransformPoint(worldPoint) : worldPoint;

        float q = Mathf.Max(1e-4f, contactKeyQuantize);
        lp.x = Mathf.Round(lp.x / q) * q;
        lp.y = Mathf.Round(lp.y / q) * q;
        lp.z = Mathf.Round(lp.z / q) * q;

        ContactKey k;
        k.colliderId = id;
        k.localQ = lp;
        return k;
    }

    private void CleanupPointStates()
    {
        if (pointStates.Count == 0) return;

        int ttl = Mathf.Max(1, contactStateTTLSteps);
        int threshold = fixedStepCounter - ttl;

        pointStatesToRemove.Clear();
        foreach (var kv in pointStates)
        {
            if (kv.Value == null) { pointStatesToRemove.Add(kv.Key); continue; }
            if (kv.Value.lastSeenStep < threshold) pointStatesToRemove.Add(kv.Key);
        }

        for (int i = 0; i < pointStatesToRemove.Count; i++)
            pointStates.Remove(pointStatesToRemove[i]);
    }

    // =========================
    // UI 同步
    // =========================
    private void SyncParamsFromUI()
    {
        if (uiK != null) springK = uiK.Value;
        if (uiC != null) damperC = uiC.Value;
        if (uiStableNormal != null) enableStableNormalSupport = uiStableNormal.IsActive;
        if (uiNormalERP != null) normalSupportERP = uiNormalERP.Value;
        if (uiNormalVelDamping != null) normalSupportVelDamping = uiNormalVelDamping.Value;
        if (uiNormalMassScale != null) normalSupportMassScale = uiNormalMassScale.Value;
        if (uiNormalSlop != null) normalSupportSlop = uiNormalSlop.Value;
        if (uiMuS != null) muStatic = uiMuS.Value;
        if (uiMuK != null) muKinetic = uiMuK.Value;
        if (uiVStatic != null) vStatic = uiVStatic.Value;
        if (uiFTau != null) forceFilterTau = uiFTau.Value;

        if (uiSinglePassLoad != null) enableSinglePassLoadScaling = uiSinglePassLoad.IsActive;
        if (uiCombinedSlip != null) enableCombinedSlipFriction = uiCombinedSlip.IsActive;
        if (uiLongGrip != null) longitudinalGripScale = uiLongGrip.Value;
        if (uiLatGrip != null) lateralGripScale = uiLatGrip.Value;
        if (uiModernLowSpeed != null) enableModernLowSpeedTire = uiModernLowSpeed.IsActive;
        if (uiLowRelaxFloor != null) lowSpeedRelaxSpeedFloor = uiLowRelaxFloor.Value;
        if (uiLowCreepSpeed != null) lowSpeedCreepSpeed = uiLowCreepSpeed.Value;
        if (uiLowCreepBlend != null) lowSpeedCreepBlend = uiLowCreepBlend.Value;
        if (uiLowShearDamping != null) lowSpeedShearDampingC = uiLowShearDamping.Value;
        if (uiAxleSpinDamping != null) axleSpinDampingK = uiAxleSpinDamping.Value;
        if (uiAxleAirDamping != null) axleAirDampingK = uiAxleAirDamping.Value;

        if (uiEnableTire != null) enableTireModel = uiEnableTire.IsActive;

        if (uiRelax != null) enableTireRelaxation = uiRelax.IsActive;
        if (uiRelaxL != null) relaxLength = uiRelaxL.Value;
        if (uiShearK != null) shearK = uiShearK.Value;
        if (uiShearC != null) shearC = uiShearC.Value;
        if (uiMaxShear != null) maxShearDisp = uiMaxShear.Value;

        if (uiDecouple != null) decoupleTireForceAndTorque = uiDecouple.IsActive;
        if (uiDecoupledApply != null) enableDecoupledTireForceApplication = uiDecoupledApply.IsActive;
        if (uiNormalGroundReaction != null) enableNormalGroundReactionForces = uiNormalGroundReaction.IsActive;

        if (uiDbg != null) debugDraw = uiDbg.IsActive;
        if (uiDbgForce != null) debugVizTireForce = uiDbgForce.IsActive;
        if (uiDbgTreadAxis != null) debugVizTreadAndAxis = uiDbgTreadAxis.IsActive;

        if (uiTreadClip != null) enableTreadWidthClip = uiTreadClip.IsActive;
        if (uiTreadW != null) treadWidth = uiTreadW.Value;
        if (uiTreadRayFan != null) enableTreadRayFan = uiTreadRayFan.IsActive;
        if (uiTreadRayCount != null) treadRayCount = Mathf.RoundToInt(uiTreadRayCount.Value);

        if (uiLineW != null)
        {
            float w = uiLineW.Value;
            forceLineWidth = w;
            thinLineWidth = w * 0.6f;
        }

        if (uiForceScale != null) forceToLength = uiForceScale.Value;

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

        if (uiMaxAngVel != null) maxAngularVelocityLimit = uiMaxAngVel.Value;

        if (uiIncludeLayer0 != null) includeLayer0 = uiIncludeLayer0.IsActive;

        if (uiMaxContactPoints != null) maxContactPoints = Mathf.RoundToInt(uiMaxContactPoints.Value);

        // NEW: gate + normal filter UI
        if (uiGateEnable != null) enableContactGate = uiGateEnable.IsActive;
        if (uiGateInFrames != null) gateFadeInFrames = Mathf.RoundToInt(uiGateInFrames.Value);
        if (uiGateOutFrames != null) gateFadeOutFrames = Mathf.RoundToInt(uiGateOutFrames.Value);

        if (uiNFilterEnable != null) enableNormalFilter = uiNFilterEnable.IsActive;
        if (uiNFilterAlpha != null) normalFilterAlpha = uiNFilterAlpha.Value;
    }

    private void ApplyLineWidthsIfReady()
    {
        for (int i = 0; i < dbgPoints.Count; i++)
        {
            DebugPointViz p = dbgPoints[i];
            if (p == null) continue;
            if (p.lrCenterToP != null) p.lrCenterToP.SetWidth(thinLineWidth, thinLineWidth);
            if (p.lrNormal != null) p.lrNormal.SetWidth(thinLineWidth, thinLineWidth);
            if (p.lrForceN != null) p.lrForceN.SetWidth(forceLineWidth, forceLineWidth);
            if (p.lrForceT != null) p.lrForceT.SetWidth(forceLineWidth, forceLineWidth);
        }
        if (lrTreadAxis != null) lrTreadAxis.SetWidth(thinLineWidth, thinLineWidth);
        if (lrTreadRingA != null) lrTreadRingA.SetWidth(thinLineWidth, thinLineWidth);
        if (lrTreadRingB != null) lrTreadRingB.SetWidth(thinLineWidth, thinLineWidth);
    }

    // =========================
    // Layer mask helpers
    // =========================
    private int BuildContactRayMask()
    {
        int mask = (1 << 24) | (1 << 29);
        if (includeLayer0) mask |= (1 << 0);
        return mask;
    }

    private bool IsColliderInContactLayers(Collider c)
    {
        if (c == null) return false;
        int lay = c.gameObject.layer;
        int bit = 1 << lay;
        int mask = BuildContactRayMask();
        return (bit & mask) != 0;
    }

    // =========================
    // 轮轴方向（世界）
    // 默认：优先使用 trigger capsule 的轴；没有则回退 wheelAxisLocal
    // =========================
    private Vector3 GetWheelAxisWorld()
    {
        if (treadTriggerCapsule != null)
        {
            Transform t = treadTriggerCapsule.transform;
            Vector3 axisWorld;
            switch (treadTriggerCapsule.direction) // 0=X 1=Y 2=Z
            {
                case 0: axisWorld = t.right; break;
                case 1: axisWorld = t.up; break;
                default: axisWorld = t.forward; break;
            }
            if (axisWorld.sqrMagnitude > 1e-10f) return axisWorld.normalized;
        }

        switch (wheelAxisLocal)
        {
            case AxisLocal.X: return transform.right;
            case AxisLocal.Y: return transform.up;
            case AxisLocal.Z: return transform.forward;
        }
        return transform.up;
    }

    private Vector3 GetDriveAxisWorld()
    {
        Vector3 a = GetWheelAxisWorld();
        return (a.sqrMagnitude > 1e-10f) ? a.normalized : transform.up;
    }

    private float GetAxisScaleWorld()
    {
        Vector3 s = transform.lossyScale;
        switch (wheelAxisLocal)
        {
            case AxisLocal.X: return Mathf.Abs(s.x);
            case AxisLocal.Y: return Mathf.Abs(s.y);
            case AxisLocal.Z: return Mathf.Abs(s.z);
        }
        return 1f;
    }

    private bool IsWithinTreadWidth(Vector3 pointWorld, Vector3 centerWorld, Vector3 axisWorldUnit, float halfWWorld)
    {
        float s = Vector3.Dot(pointWorld - centerWorld, axisWorldUnit);
        return Mathf.Abs(s) <= halfWWorld;
    }

    private Vector3 ProjectOnPlane(Vector3 v, Vector3 n)
    {
        return v - Vector3.Dot(v, n) * n;
    }

    // =========================
    // Debug：显示踏面裁切范围（两侧圆环 + 轮轴线）——不动
    // =========================
    private void DrawTreadRangeDebug(Vector3 center, Vector3 axisWorldUnit, float halfW, float radius)
    {
        if (radius <= 1e-5f) return;
        if (axisWorldUnit.sqrMagnitude < 1e-8f) return;
        axisWorldUnit.Normalize();

        if (lrTreadAxis == null || lrTreadRingA == null || lrTreadRingB == null) return;

        float axisLen = Mathf.Max(radius * 0.5f, radius * treadAxisDebugLen);
        SetLine(lrTreadAxis, center - axisWorldUnit * axisLen, center + axisWorldUnit * axisLen);

        Vector3 u = Vector3.Cross(axisWorldUnit, Vector3.up);
        if (u.sqrMagnitude < 1e-6f) u = Vector3.Cross(axisWorldUnit, Vector3.right);
        u.Normalize();
        Vector3 v = Vector3.Cross(axisWorldUnit, u).normalized;

        int seg = Mathf.Clamp(treadRangeSegments, 8, 128);
        int count = seg + 1;
        lrTreadRingA.SetVertexCount(count);
        lrTreadRingB.SetVertexCount(count);

        Vector3 c0 = center + axisWorldUnit * halfW;
        Vector3 c1 = center - axisWorldUnit * halfW;
        SetCircleLine(lrTreadRingA, c0, u, v, radius, seg);
        SetCircleLine(lrTreadRingB, c1, u, v, radius, seg);
    }

    private void SetCircleLine(LineRenderer lr, Vector3 center, Vector3 u, Vector3 v, float r, int segments)
    {
        if (lr == null) return;
        for (int i = 0; i <= segments; i++)
        {
            float a = (Mathf.PI * 2f) * ((float)i / (float)segments);
            Vector3 p = center + (Mathf.Cos(a) * u + Mathf.Sin(a) * v) * r;
            lr.SetPosition(i, p);
        }
    }

    // =========================
    // 找到踏面 trigger capsule
    // =========================
    private CapsuleCollider FindTreadTriggerCapsule()
    {
        Collider[] cols = GetComponentsInChildren<Collider>(true);
        for (int i = 0; i < cols.Length; i++)
        {
            CapsuleCollider cc = cols[i] as CapsuleCollider;
            if (cc == null) continue;
            if (!cc.isTrigger) continue;
            return cc;
        }
        return null;
    }

    private float MaxAbsComponent(Vector3 v)
    {
        float ax = Mathf.Abs(v.x);
        float ay = Mathf.Abs(v.y);
        float az = Mathf.Abs(v.z);
        return Mathf.Max(ax, Mathf.Max(ay, az));
    }

    // =========================
    // Debug 可视化：对象创建/销毁
    // =========================
    private void EnsureDebugObjects()
    {
        if (dbgRoot == null)
        {
            dbgRoot = new GameObject("RubberTire_Debug");
            if (MainVis != null) dbgRoot.transform.SetParent(MainVis, false);
            else dbgRoot.transform.SetParent(transform, false);
        }

        // 踏面/轴线（单份）
        if (lrTreadAxis == null) lrTreadAxis = CreateLine(dbgRoot.transform, "L_tread_axis", Color.yellow, thinLineWidth);

        int seg = Mathf.Clamp(treadRangeSegments, 8, 128);
        if (lrTreadRingA == null) lrTreadRingA = CreatePolyline(dbgRoot.transform, "L_tread_ringA", Color.cyan, thinLineWidth, seg + 1);
        if (lrTreadRingB == null) lrTreadRingB = CreatePolyline(dbgRoot.transform, "L_tread_ringB", Color.cyan, thinLineWidth, seg + 1);

        // 多接触点可视化池
        EnsureDebugPointPool(Mathf.Clamp(maxContactPoints, 1, 12));

        HideDebugObjects();
    }

    private void EnsureDebugLocalCapacity(int desired)
    {
        int need = Mathf.Max(1, desired);
        if (dbgLocalContacts == null) dbgLocalContacts = new DebugContactLocalData[Mathf.Max(8, need)];
        if (dbgLocalContacts.Length >= need) return;

        int newCap = dbgLocalContacts.Length;
        while (newCap < need) newCap *= 2;
        if (newCap < 8) newCap = 8;
        var n = new DebugContactLocalData[newCap];
        for (int i = 0; i < dbgLocalContacts.Length; i++) n[i] = dbgLocalContacts[i];
        dbgLocalContacts = n;
    }

    private void EnsureDebugPointPool(int needed)
    {
        int need = Mathf.Max(0, needed);
        if (need <= dbgPoints.Count) return;
        if (dbgRoot == null) return;

        while (dbgPoints.Count < need)
        {
            int idx = dbgPoints.Count;
            dbgPoints.Add(CreateDebugPointViz(idx));
        }
    }

    private DebugPointViz CreateDebugPointViz(int index)
    {
        DebugPointViz p = new DebugPointViz();

        p.root = new GameObject("P_" + index);
        p.root.transform.SetParent(dbgRoot.transform, false);

        // contact point
        p.sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        p.sphere.name = "Contact";
        p.sphere.transform.SetParent(p.root.transform, false);
        p.sphere.transform.localScale = Vector3.one * 0.10f;

        var c = p.sphere.GetComponent<Collider>();
        if (c != null) c.enabled = false;

        var r = p.sphere.GetComponent<Renderer>();
        if (r != null) r.material = CreateColorMaterial(Color.yellow);

        // lines
        p.lrCenterToP = CreateLine(p.root.transform, "L_center_p", Color.white, thinLineWidth);
        p.lrNormal = CreateLine(p.root.transform, "L_normal", Color.green, thinLineWidth);
        p.lrForceN = CreateLine(p.root.transform, "L_forceN", Color.red, forceLineWidth);
        p.lrForceT = CreateLine(p.root.transform, "L_forceT", Color.blue, forceLineWidth);

        p.root.SetActive(false);
        return p;
    }

    private Material CreateColorMaterial(Color color)
    {
        Shader sh = Shader.Find("Particles/Additive");
        if (sh == null) sh = Shader.Find("Particles/Alpha Blended");
        if (sh == null) sh = Shader.Find("Unlit/Color");
        if (sh == null) sh = Shader.Find("Diffuse");

        var mat = new Material(sh);
        mat.mainTexture = Texture2D.whiteTexture;

        if (mat.HasProperty("_TintColor")) mat.SetColor("_TintColor", color);
        if (mat.HasProperty("_Color")) mat.SetColor("_Color", color);

        mat.color = color;
        return mat;
    }

    private LineRenderer CreateLine(Transform parent, string name, Color color, float width)
    {
        if (parent == null) parent = dbgRoot != null ? dbgRoot.transform : transform;

        var go = new GameObject(name);
        go.transform.SetParent(parent, false);

        var lr = go.AddComponent<LineRenderer>();
        lr.useWorldSpace = true;

        lr.SetVertexCount(2);
        lr.SetWidth(width, width);
        lr.SetColors(color, color);
        lr.material = CreateColorMaterial(color);

        lr.SetPosition(0, Vector3.zero);
        lr.SetPosition(1, Vector3.zero);
        return lr;
    }

    private LineRenderer CreateLine(string name, Color color, float width)
    {
        return CreateLine(dbgRoot != null ? dbgRoot.transform : transform, name, color, width);
    }

    private LineRenderer CreatePolyline(Transform parent, string name, Color color, float width, int vertexCount)
    {
        if (parent == null) parent = dbgRoot != null ? dbgRoot.transform : transform;

        var go = new GameObject(name);
        go.transform.SetParent(parent, false);

        var lr = go.AddComponent<LineRenderer>();
        lr.useWorldSpace = true;

        int vc = Mathf.Max(2, vertexCount);
        lr.SetVertexCount(vc);
        lr.SetWidth(width, width);
        lr.SetColors(color, color);
        lr.material = CreateColorMaterial(color);

        for (int i = 0; i < vc; i++) lr.SetPosition(i, Vector3.zero);
        return lr;
    }

    private LineRenderer CreatePolyline(string name, Color color, float width, int vertexCount)
    {
        return CreatePolyline(dbgRoot != null ? dbgRoot.transform : transform, name, color, width, vertexCount);
    }

    private void SetLine(LineRenderer lr, Vector3 a, Vector3 b)
    {
        if (lr == null) return;
        lr.SetPosition(0, a);
        lr.SetPosition(1, b);
    }

    private void SetLRVisible(LineRenderer lr, bool visible)
    {
        if (lr == null) return;
        lr.enabled = visible;
    }

    private void ShowDebugObjects()
    {
        if (dbgRoot != null) dbgRoot.SetActive(true);
    }

    private void HideDebugObjects()
    {
        if (dbgRoot != null) dbgRoot.SetActive(false);
    }

    private void DestroyDebugObjects()
    {
        if (dbgRoot != null)
        {
            GameObject.Destroy(dbgRoot);
            dbgRoot = null;
            lrTreadAxis = null;
            lrTreadRingA = null;
            lrTreadRingB = null;
            dbgPoints.Clear();
            dbgHasLocal = false;
            dbgLocalCount = 0;
        }
    }
}
