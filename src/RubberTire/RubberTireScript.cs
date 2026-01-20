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

    public bool enableTireModel = true;

    // =========================
    // 静/动摩擦 + 输出滤波
    // =========================
    public float muStatic = 1.40f;
    public float muKinetic = 1.20f;
    public float vStatic = 0.25f;
    public float forceFilterTau = 0.03f;

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
    // tau_damp = -rollingDampingK * omega
    // =========================
    public bool enableRollingDamping = false;
    public float rollingDampingK = 50f;        // N*m*s

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

    private MToggle uiDriveBrake;
    private MToggle uiInvertDrive;
    private MKey uiKeyThrottle, uiKeyBrake, uiKeyReverse;
    private MSlider uiMaxDriveTorque, uiMaxBrakeTorque, uiBrakeDeadband, uiBrakeHoldK;
    private MToggle uiPowerLimit;
    private MSlider uiMaxDrivePower, uiPowerOmegaEps;
    private MToggle uiRollingDamp;
    private MSlider uiRollingDampK;
    private MSlider uiThrottleRise, uiThrottleFall, uiBrakeRise, uiBrakeFall;

    private MSlider uiMaxAngVel;

    private MToggle uiIncludeLayer0;

    private MSlider uiMaxContactPoints;

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
        uiK = AddSlider("K (Spring)", "k", springK, 0f, 20000f);
        uiC = AddSlider("C (Damper)", "c", damperC, 0f, 200f);
        uiMuS = AddSlider("Mu Static", "muS", muStatic, 0f, 3f);
        uiMuK = AddSlider("Mu Kinetic", "muK", muKinetic, 0f, 3f);
        uiVStatic = AddSlider("vStatic (m/s)", "vStatic", vStatic, 0.01f, 2.0f);
        uiFTau = AddSlider("Force Filter Tau (s)", "fTau", forceFilterTau, 0f, 0.20f);

        uiEnableTire = AddToggle("Tire Model", "tire", enableTireModel);

        uiRelax = AddToggle("Tire Relaxation", "relax", enableTireRelaxation);
        uiRelaxL = AddSlider("Relax Length", "relaxL", relaxLength, 0.05f, 5.0f);
        uiShearK = AddSlider("Shear K (N/m)", "shK", shearK, 1000f, 200000f);
        uiShearC = AddSlider("Shear C (N*s/m)", "shC", shearC, 0f, 5000f);
        uiMaxShear = AddSlider("Max Shear Disp", "shMax", maxShearDisp, 0.01f, 0.50f);

        uiDecouple = AddToggle("Decouple F/T", "decouple", decoupleTireForceAndTorque);

        uiDbg = AddToggle("Debug Master", "dbg", debugDraw);
        uiDbgForce = AddToggle("DebugViz: Tire Force", "dbgF", debugVizTireForce);
        uiDbgTreadAxis = AddToggle("Debug: Tread+Axis", "dbgTA", debugVizTreadAndAxis);

        uiTreadClip = AddToggle("Tread Width Clip", "tw-clip", enableTreadWidthClip);
        uiTreadW = AddSlider("Tread Width", "tw", treadWidth, 0.05f, 5f);

        uiLineW = AddSlider("Line Width", "lw", forceLineWidth, 0.01f, 0.30f);
        uiForceScale = AddSlider("Force Scale", "fs", forceToLength, 0.00001f, 0.01f);

        uiDriveBrake = AddToggle("Drive/Brake (Keys)", "drv", enableDriveBrake);
        uiInvertDrive = AddToggle("Invert Drive Torque", "invDrv", invertDriveTorque);

        uiKeyThrottle = AddKey("Throttle Key", "kThr", KeyCode.T);
        uiKeyBrake = AddKey("Brake Key", "kBrk", KeyCode.G);
        uiKeyReverse = AddKey("Reverse Key", "kRev", KeyCode.R);

        uiMaxDriveTorque = AddSlider("Max Drive Torque", "drvT", maxDriveTorque, 0f, 50000f);
        uiPowerLimit = AddToggle("Power Limit", "pLim", enablePowerLimit);
        uiMaxDrivePower = AddSlider("Max Drive Power (W)", "pMax", maxDrivePower, 0f, 500000f);
        uiPowerOmegaEps = AddSlider("Power Omega Eps", "pEps", powerLimitOmegaEps, 0.1f, 20f);

        uiRollingDamp = AddToggle("Rolling Damping", "rDmp", enableRollingDamping);
        uiRollingDampK = AddSlider("Rolling Damping K", "rK", rollingDampingK, 0f, 5000f);

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

        pointStates.Clear();
        colStates.Clear();

        Rigidbody.maxAngularVelocity = Mathf.Max(10f, maxAngularVelocityLimit);
        Rigidbody.angularDrag = 0.05f;

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

        // ===== 0) Drive/Brake (remappable keys) =====
        {
            float dt = Time.fixedDeltaTime;

            bool heldThr = enableDriveBrake && uiKeyThrottle != null && uiKeyThrottle.IsHeld;
            bool heldBrk = enableDriveBrake && uiKeyBrake != null && uiKeyBrake.IsHeld;
            bool heldRev = enableDriveBrake && uiKeyReverse != null && uiKeyReverse.IsHeld;

            float targetThr = heldThr ? 1f : 0f;
            float targetBrk = heldBrk ? 1f : 0f;

            float thrRate = (targetThr > throttle01) ? Mathf.Max(0f, throttleRise) : Mathf.Max(0f, throttleFall);
            float brkRate = (targetBrk > brake01) ? Mathf.Max(0f, brakeRise) : Mathf.Max(0f, brakeFall);
            throttle01 = Mathf.MoveTowards(throttle01, targetThr, thrRate * dt);
            brake01 = Mathf.MoveTowards(brake01, targetBrk, brkRate * dt);

            Vector3 aAxis = GetDriveAxisWorld();
            float omegaAxis = Vector3.Dot(Rigidbody.angularVelocity, aAxis);

            float tauDrive = throttle01 * maxDriveTorque;

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
                    float tauMax = Mathf.Min(Mathf.Abs(maxDriveTorque), tauMaxByPower);
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

            // ---- Rolling viscous damping (simple rolling resistance / jitter damper) ----
            // 建议只在有接触时启用，避免空中“空气阻尼”影响你测试；需要的话你也可以改成始终启用。
            if (enableRollingDamping && rollingDampingK > 0f && contacts.Count > 0)
            {
                tau += -omegaAxis * rollingDampingK;
            }

            if (Mathf.Abs(tau) > 1e-6f)
                Rigidbody.AddTorque(aAxis * tau, ForceMode.Force);
        }

        if (contacts.Count == 0)
        {
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

            // ---- Normal force ----
            float Fn = springK * s.pen;

            if (damperC > 0f)
            {
                Vector3 vAtP = Rigidbody.GetPointVelocity(s.p);
                float vN = Vector3.Dot(vAtP, nUse);
                float compressionSpeed = -vN;
                if (compressionSpeed > 0f) Fn += damperC * compressionSpeed;
            }

            Fn = Mathf.Clamp(Fn, 0f, maxNormalForce);
            Fn *= gate;
            if (Fn <= 1e-6f) continue;

            Vector3 FnVec = Fn * nUse;
            Rigidbody.AddForceAtPosition(FnVec, s.p, ForceMode.Force);

            // ---- Tire friction per point（可选：enableTireModel）----
            Vector3 Ftire = Vector3.zero;
            Rigidbody groundRb = null;

            if (enableTireModel)
            {
                groundRb = (s.col != null) ? s.col.attachedRigidbody : null;

                Vector3 vGround = Vector3.zero;
                if (groundRb != null) vGround = groundRb.GetPointVelocity(s.p);

                Vector3 vWheel = Rigidbody.GetPointVelocity(s.p);
                Vector3 vRel = vWheel - vGround;

                // 切向基（沿“滚动前进”方向）
                Vector3 f = ProjectOnPlane(Vector3.Cross(nUse, aAxisWheel), nUse);
                if (f.sqrMagnitude < 1e-6f)
                    f = ProjectOnPlane(Vector3.Cross(nUse, transform.right), nUse);

                if (f.sqrMagnitude > 1e-6f)
                {
                    f.Normalize();

                    Vector3 vSlip = ProjectOnPlane(vRel, nUse);
                    float vSlipMag = vSlip.magnitude;

                    if (enableTireRelaxation)
                    {
                        TirePointState st = GetOrCreatePointState(s.col, s.p);
                        st.lastSeenStep = fixedStepCounter;

                        float speed = Mathf.Max(vSlipMag, vEps);
                        float T = Mathf.Max(1e-4f, relaxLength / speed);

                        Vector3 xDot = vSlip - st.shearDispWorld / T;
                        st.shearDispWorld += xDot * dtFixed;

                        if (maxShearDisp > 1e-5f)
                        {
                            float m = st.shearDispWorld.magnitude;
                            if (m > maxShearDisp) st.shearDispWorld = st.shearDispWorld * (maxShearDisp / m);
                        }

                        Vector3 Fraw = -shearK * st.shearDispWorld;
                        if (shearC > 0f) Fraw += -shearC * xDot;

                        float FmaxS = muStatic * Fn;   // 注意：Fn 已乘 gate
                        float FmaxK = muKinetic * Fn;  // 注意：Fn 已乘 gate
                        float Fmag = Fraw.magnitude;

                        if (vSlipMag < Mathf.Max(1e-3f, vStatic))
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
                    }
                    else
                    {
                        float FmaxK = muKinetic * Fn; // Fn 已乘 gate
                        if (vSlipMag > 1e-5f)
                            Ftire = -vSlip / vSlipMag * FmaxK;
                        else
                            Ftire = Vector3.zero;
                    }

                    // gate 同样乘到切向（即使 Fn 已 gate，乘一次不会错，只会更“渐变”）
                    Ftire *= gate;

                    if (Ftire.sqrMagnitude > 1e-10f)
                    {
                        Rigidbody.AddForceAtPosition(Ftire, s.p, ForceMode.Force);
                        if (groundRb != null) groundRb.AddForceAtPosition(-Ftire, s.p, ForceMode.Force);
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
    // Contact gather (per collider aggregation)
    // =========================
    private void GatherTopContactSamples(
        Vector3 center, Vector3 downDir, float R,
        bool doClip, Vector3 axisWorldUnit, float halfWWorld,
        int N, List<ContactSample> outTop)
    {
        outTop.Clear();
        hitsByCol.Clear();

        if (!useRaycastContact) return;

        float maxDist = R + Mathf.Max(0f, rayExtra);

        RaycastHit[] hits = Physics.RaycastAll(
            center,
            downDir,
            maxDist,
            contactRayMask,
            QueryTriggerInteraction.Ignore
        );

        if (hits == null || hits.Length == 0) return;

        // collect hits and group by collider
        for (int i = 0; i < hits.Length; i++)
        {
            Collider c = hits[i].collider;
            if (c == null) continue;

            if (!contacts.Contains(c)) continue;
            if (!IsColliderInContactLayers(c)) continue;

            Vector3 p = hits[i].point;
            Vector3 n = hits[i].normal;
            if (n.sqrMagnitude < 1e-12f) continue;
            n.Normalize();

            if (doClip && !IsWithinTreadWidth(p, center, axisWorldUnit, halfWWorld))
                continue;

            float dist = Vector3.Distance(center, p);
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
                list = new List<HitSample>(8);
                hitsByCol.Add(id, list);
            }
            list.Add(hs);
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

            float penMax = list[0].pen;

            for (int i = 0; i < take; i++)
            {
                float w = perColliderWeightByPen ? Mathf.Max(0f, list[i].pen) : 1f;
                wsum += w;
                pAcc += list[i].p * w;
                nAcc += list[i].n * w;
            }

            if (wsum <= 1e-8f)
                continue;

            Vector3 pAgg = pAcc / wsum;
            Vector3 nAgg = nAcc;
            if (nAgg.sqrMagnitude > 1e-12f) nAgg.Normalize();
            else nAgg = list[0].n;

            float distAgg = Vector3.Distance(center, pAgg);
            float penAgg = R - distAgg;
            penAgg = Mathf.Clamp(penAgg, 0f, penMax);

            if (penAgg <= minPenForContact) continue;

            ContactSample agg;
            agg.col = list[0].col;
            agg.colId = list[0].colId;
            agg.p = pAgg;
            agg.n = nAgg;
            agg.pen = penAgg;
            agg.dist = distAgg;

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
        if (uiMuS != null) muStatic = uiMuS.Value;
        if (uiMuK != null) muKinetic = uiMuK.Value;
        if (uiVStatic != null) vStatic = uiVStatic.Value;
        if (uiFTau != null) forceFilterTau = uiFTau.Value;

        if (uiEnableTire != null) enableTireModel = uiEnableTire.IsActive;

        if (uiRelax != null) enableTireRelaxation = uiRelax.IsActive;
        if (uiRelaxL != null) relaxLength = uiRelaxL.Value;
        if (uiShearK != null) shearK = uiShearK.Value;
        if (uiShearC != null) shearC = uiShearC.Value;
        if (uiMaxShear != null) maxShearDisp = uiMaxShear.Value;

        if (uiDecouple != null) decoupleTireForceAndTorque = uiDecouple.IsActive;

        if (uiDbg != null) debugDraw = uiDbg.IsActive;
        if (uiDbgForce != null) debugVizTireForce = uiDbgForce.IsActive;
        if (uiDbgTreadAxis != null) debugVizTreadAndAxis = uiDbgTreadAxis.IsActive;

        if (uiTreadClip != null) enableTreadWidthClip = uiTreadClip.IsActive;
        if (uiTreadW != null) treadWidth = uiTreadW.Value;

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
        if (uiPowerLimit != null) enablePowerLimit = uiPowerLimit.IsActive;
        if (uiMaxDrivePower != null) maxDrivePower = uiMaxDrivePower.Value;
        if (uiPowerOmegaEps != null) powerLimitOmegaEps = uiPowerOmegaEps.Value;

        if (uiRollingDamp != null) enableRollingDamping = uiRollingDamp.IsActive;
        if (uiRollingDampK != null) rollingDampingK = uiRollingDampK.Value;
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
