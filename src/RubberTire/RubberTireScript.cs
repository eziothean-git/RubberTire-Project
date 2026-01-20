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

    // Debug 对象（不动）
    private GameObject dbgRoot;
    private LineRenderer lrCenterToP;
    private LineRenderer lrNormal;
    private LineRenderer lrForceN;
    private LineRenderer lrForceT;

    private LineRenderer lrTreadAxis;
    private LineRenderer lrTreadRingA;
    private LineRenderer lrTreadRingB;
    private GameObject dbgPointSphere;

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

            float tau = tauDrive * driveSign;

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
            HideDebugObjects();
            CleanupPointStates();
            return;
        }

        // ===== 2) Update per-collider gate + normal filter based on current contacts =====
        UpdateColliderStatesFromSamples(topSamples);

        // ===== 3) Apply forces per point =====
        float dtFixed = Time.fixedDeltaTime;

        // For debug, keep the deepest point (topSamples[0]) as “best”
        ContactSample best = topSamples[0];

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

            // ---- Tire friction per point ----
            if (!enableTireModel) continue;

            Rigidbody groundRb = (s.col != null) ? s.col.attachedRigidbody : null;

            Vector3 vGround = Vector3.zero;
            if (groundRb != null) vGround = groundRb.GetPointVelocity(s.p);

            Vector3 vWheel = Rigidbody.GetPointVelocity(s.p);
            Vector3 vRel = vWheel - vGround;

            Vector3 f = ProjectOnPlane(Vector3.Cross(nUse, aAxisWheel), nUse);
            if (f.sqrMagnitude < 1e-6f)
                f = ProjectOnPlane(Vector3.Cross(nUse, transform.right), nUse);

            if (f.sqrMagnitude < 1e-6f) continue;
            f.Normalize();

            Vector3 vSlip = ProjectOnPlane(vRel, nUse);
            float vSlipMag = vSlip.magnitude;

            Vector3 Ftire = Vector3.zero;

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

        // Optional drive torque test hook
        if (Mathf.Abs(driveTorque) > 1e-6f)
        {
            float flipSign = Flipped ? -1f : 1f;
            Rigidbody.AddTorque(GetDriveAxisWorld() * (driveTorque * flipSign), ForceMode.Force);
        }

        // ===== Debug: still render deepest point only (保持轻量，不动) =====
        if (ShowDebugVisuals && debugDraw && (fixedStepCounter % Mathf.Max(1, drawEveryFixedSteps) == 0))
        {
            EnsureDebugObjects();
            ShowDebugObjects();

            bool showForceViz = debugVizTireForce;
            bool showTreadAxisViz = debugVizTreadAndAxis;

            SetLRVisible(lrCenterToP, true);
            SetLRVisible(lrNormal, true);
            SetLRVisible(lrForceN, true);
            SetLRVisible(lrForceT, showForceViz);

            if (dbgPointSphere != null) dbgPointSphere.SetActive(true);

            // debug 使用过滤后的法向 & gate 后的 Fn（更贴近真实施加的）
            float gateBest = 1f;
            Vector3 nBest = best.n;
            ColliderContactState csBest;
            if (colStates.TryGetValue(best.colId, out csBest) && csBest != null)
            {
                if (enableContactGate) gateBest = Mathf.Clamp01(csBest.gate);
                if (enableNormalFilter)
                {
                    nBest = csBest.nFiltered;
                    if (Vector3.Dot(nBest, best.n) < 0f) nBest = -nBest;
                    if (nBest.sqrMagnitude > 1e-12f) nBest.Normalize();
                    else nBest = best.n;
                }
            }

            float FnBest = Mathf.Clamp(springK * best.pen, 0f, maxNormalForce) * gateBest;
            float nLen = Mathf.Clamp(FnBest * forceToLength, arrowMinLen, arrowMaxLen);

            SetLine(lrCenterToP, center, best.p);
            SetLine(lrNormal, best.p, best.p + nBest * normalArrowLen);
            SetLine(lrForceN, best.p, best.p + nBest * nLen);

            if (dbgPointSphere != null) dbgPointSphere.transform.position = best.p;

            if (showForceViz)
            {
                SetLine(lrForceT, best.p, best.p);
            }

            if (showTreadAxisViz && doClip)
            {
                SetLRVisible(lrTreadAxis, true);
                SetLRVisible(lrTreadRingA, true);
                SetLRVisible(lrTreadRingB, true);
                DrawTreadRangeDebug(center, axisWorld, halfW, R);
            }
            else
            {
                SetLRVisible(lrTreadAxis, false);
                SetLRVisible(lrTreadRingA, false);
                SetLRVisible(lrTreadRingB, false);
            }

            ApplyLineWidthsIfReady();
        }
        else
        {
            HideDebugObjects();
        }

        // 本帧结束：对未见 collider 衰减 gate，并清理
        DecayAndCleanupColliderStates();
        CleanupPointStates();
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
        if (lrCenterToP != null) lrCenterToP.SetWidth(thinLineWidth, thinLineWidth);
        if (lrNormal != null) lrNormal.SetWidth(thinLineWidth, thinLineWidth);
        if (lrForceN != null) lrForceN.SetWidth(forceLineWidth, forceLineWidth);
        if (lrForceT != null) lrForceT.SetWidth(forceLineWidth, forceLineWidth);
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
    // Debug 可视化：对象创建/销毁 ——不动
    // =========================
    private void EnsureDebugObjects()
    {
        if (dbgRoot != null) return;

        dbgRoot = new GameObject("RubberTire_Debug");
        if (MainVis != null) dbgRoot.transform.SetParent(MainVis, false);
        else dbgRoot.transform.SetParent(transform, false);

        lrCenterToP = CreateLine("L_center_p", Color.white, thinLineWidth);
        lrNormal = CreateLine("L_normal", Color.green, thinLineWidth);
        lrForceN = CreateLine("L_forceN", Color.red, forceLineWidth);
        lrForceT = CreateLine("L_forceT", Color.blue, forceLineWidth);

        lrTreadAxis = CreateLine("L_tread_axis", Color.yellow, thinLineWidth);
        int seg = Mathf.Clamp(treadRangeSegments, 8, 128);
        lrTreadRingA = CreatePolyline("L_tread_ringA", Color.cyan, thinLineWidth, seg + 1);
        lrTreadRingB = CreatePolyline("L_tread_ringB", Color.cyan, thinLineWidth, seg + 1);

        dbgPointSphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        dbgPointSphere.name = "P_contact";
        dbgPointSphere.transform.SetParent(dbgRoot.transform, false);
        dbgPointSphere.transform.localScale = Vector3.one * 0.10f;

        var c = dbgPointSphere.GetComponent<Collider>();
        if (c != null) c.enabled = false;

        var r = dbgPointSphere.GetComponent<Renderer>();
        if (r != null) r.material = CreateColorMaterial(Color.yellow);

        HideDebugObjects();
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

    private LineRenderer CreateLine(string name, Color color, float width)
    {
        var go = new GameObject(name);
        go.transform.SetParent(dbgRoot.transform, false);

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

    private LineRenderer CreatePolyline(string name, Color color, float width, int vertexCount)
    {
        var go = new GameObject(name);
        go.transform.SetParent(dbgRoot.transform, false);

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
            lrCenterToP = null;
            lrNormal = null;
            lrForceN = null;
            lrForceT = null;
            lrTreadAxis = null;
            lrTreadRingA = null;
            lrTreadRingB = null;
            dbgPointSphere = null;
        }
    }
}
