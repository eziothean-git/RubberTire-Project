using System;
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
/// - Contact ray：查询所有 layer，射线命中本身就是接触候选
/// - 可视化暂不改
/// </summary>
public partial class RubberTireWheelScript : BlockScript
{
    internal static readonly List<RubberTireWheelScript> SimulatingInstances =
        new List<RubberTireWheelScript>(16);
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

    // A1: radial contact directions around the wheel circle (1 = legacy
    // gravity-down ray only). Walls, ceilings and loop tracks need > 1.
    public int radialRayCount = 8;
    public bool enableAdaptiveRadialSampling = true;

    // ====== Multi-point settings ======
    public int maxContactPoints = 6;          // Top-N colliders by penetration after aggregation
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

    // =========================
    // 角速度上限
    // =========================
    // A5: solver-stability policy. Above ~0.5 rad of rotation per fixed step the
    // block joint cannot hold axle alignment and precession diverges ("wobble").
    // Hard cap, drive torque gating and wobble damping all share this bound.
    internal const float SpinHardCap = 60f;
    public float maxAngularVelocityLimit = SpinHardCap;

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
    // 运行时状态
    // =========================
    private readonly HashSet<Collider> contacts = new HashSet<Collider>();
    private CapsuleCollider treadTriggerCapsule;
    private int fixedStepCounter = 0;

    // B3: raycast contact result of the previous fixed step (rolling damping gate).
    private bool lastStepHadRaycastContact;
    private int consecutiveNoContactSteps;
    private const int ShearResetNoContactSteps = 3;

    // A5: immediate joint parent (steering knuckle, suspension arm, chassis...).
    private Rigidbody jointParentBody;
    private bool jointParentCached;
    private Transform ownSimulationRoot;
    private readonly HashSet<Collider> ownMachineColliders = new HashSet<Collider>();
    private int lastOwnMachineHitCount;
    private bool lastGatherWasMultiRay;
    private int adaptiveRadialCursor = 1;
    private int lastSuccessfulRadialSector = -1;
    private readonly int[] radialQuerySectors = new int[16];
    private int lastRaycastQueryCount;
    private int lastRaycastRawHitCount;
    private int lastRaycastAcceptedHitCount;
    private int lastRaycastSaturatedCount;

    // Physics queries must use the Rigidbody pose. With interpolation enabled,
    // reading a child Transform can return a render pose between fixed steps;
    // using that point with AddForceAtPosition produces a rotating lever-arm bias.
    private bool treadPhysicsGeometryCached;
    private Vector3 treadCenterBodyLocal;
    private Vector3 treadAxisBodyLocal = Vector3.up;
    private float treadRadiusWorld = 1f;
    private float treadAxisScaleWorld = 1f;

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
        // Contact samples and forces are world-space physics data. Keeping the
        // point world-space is just as important as keeping the directions there:
        // a tyre-local point would rotate around the axle during interpolation.
        public Vector3 pWorld;
        public Vector3 nWorld;
        public float FnMag;
        public Vector3 FtWorld;
        public float FtMag;
    }

    private DebugContactLocalData[] dbgLocalContacts = new DebugContactLocalData[8];
    private int dbgLocalCount = 0;
    private bool dbgHasLocal = false;

    // Fixed-step world-space debug snapshot. It intentionally does not follow
    // the wheel's render interpolation or spin after the force was submitted.
    private Vector3 dbgCenterWorld = Vector3.zero;
    private Vector3 dbgAxisWorld = Vector3.right;
    private float dbgHalfWWorld = 0f;
    private float dbgRadiusWorld = 1f;
    private bool dbgDoClip = false;

    // Raycast mask cache
    private int contactRayMask = ~0;

    // ====== per-point tire state cache ======
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

    // C1: cached comparers; inline lambdas allocate a Comparison<T> per call.
    private static readonly Comparison<HitSample> HitSamplePenDescending =
        delegate(HitSample a, HitSample b) { return b.pen.CompareTo(a.pen); };
    private static readonly Comparison<ContactSample> ContactSamplePenDescending =
        delegate(ContactSample a, ContactSample b) { return b.pen.CompareTo(a.pen); };

    private readonly Dictionary<int, List<HitSample>> hitsByCol = new Dictionary<int, List<HitSample>>(64);

    // ----------- 生命周期 -----------

    public override void SafeAwake()
    {
        CreateDrivetrainKeyControls();
        CreateFactoryConfigMapper();
    }

    public override void OnSimulateStart()
    {
        if (!SimulatingInstances.Contains(this)) SimulatingInstances.Add(this);
        contacts.Clear();
        fixedStepCounter = 0;
        currentGear = 1;
        throttle01 = 0f;
        brake01 = 0f;
        engineLimiterCut = false;
        currentEngineRpm = 0f;
        adaptiveRadialCursor = 1;
        lastSuccessfulRadialSector = -1;
        lastRaycastQueryCount = 0;
        lastRaycastRawHitCount = 0;
        lastRaycastAcceptedHitCount = 0;
        lastRaycastSaturatedCount = 0;
        lastStepHadRaycastContact = false;
        consecutiveNoContactSteps = 0;
        jointParentCached = false;
        jointParentBody = null;

        pointStates.Clear();
        colStates.Clear();

        Rigidbody.maxAngularVelocity = GetSpinCap();

        // Capture baseline angularDrag so we can add/remove extra rolling resistance without breaking other physics.
        baseAngularDrag = Rigidbody.angularDrag;
        baseAngularDragCaptured = true;

        treadTriggerCapsule = FindTreadTriggerCapsule();
        CacheTreadPhysicsGeometry();
        CacheOwnMachineColliders();

        contactRayMask = BuildContactRayMask();

        if (ShowDebugVisuals && debugDraw)
            EnsureDebugObjects();
    }

    public override void OnSimulateStop()
    {
        SimulatingInstances.Remove(this);
        contacts.Clear();
        pointStates.Clear();
        colStates.Clear();
        throttle01 = 0f;
        brake01 = 0f;
        engineLimiterCut = false;
        currentEngineRpm = 0f;
        lastStepHadRaycastContact = false;
        consecutiveNoContactSteps = 0;
        jointParentCached = false;
        jointParentBody = null;

        DestroyDebugObjects();
        treadTriggerCapsule = null;
        treadPhysicsGeometryCached = false;
        ownSimulationRoot = null;
        ownMachineColliders.Clear();

        // Restore baseline angular drag if we modified it.
        if (HasRigidbody && baseAngularDragCaptured)
            Rigidbody.angularDrag = baseAngularDrag;
    }

    public override void OnSimulateTriggerEnter(Collider other)
    {
        if (other == null) return;
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

        FactoryPullSettings();
        contactRayMask = BuildContactRayMask();
        Rigidbody.maxAngularVelocity = GetSpinCap();

        // Legacy rolling damping uses angularDrag. The advanced mode restores angularDrag and applies load-based torque per contact.
        // B3: gated by the previous step's raycast contact; the trigger set is no longer part of the contact model.
        ApplyRollingAngularDrag(lastStepHadRaycastContact && !useLoadSensitiveRollingResistance);

        // ===== 0) Drive/Brake (remappable keys) =====
        ApplyDriveBrake();
        ApplyAxleWobbleDamping(GetDriveAxisWorld());

        // Constant test torque hook. Applied before contact processing so the
        // static-friction feed-forward (B1) sees it in the same step.
        if (drivenWheel && Mathf.Abs(driveTorque) > 1e-6f)
        {
            float hookFlipSign = Flipped ? -1f : 1f;
            Vector3 hookAxis = GetDriveAxisWorld();
            float hookTau = ClampSpinPumpingTorque(
                driveTorque * hookFlipSign,
                Vector3.Dot(Rigidbody.angularVelocity, hookAxis),
                hookAxis,
                Time.fixedDeltaTime);
            if (Mathf.Abs(hookTau) > 1e-6f)
            {
                pendingDriveAxisTorque += hookTau;
                Rigidbody.AddTorque(hookAxis * hookTau, ForceMode.Force);
            }
        }
        MarkAllColliderStatesUnseen();

        // ===== 轮心/半径（世界）=====
        Vector3 center;
        float R;
        if (treadPhysicsGeometryCached)
        {
            center = Rigidbody.position + Rigidbody.rotation * treadCenterBodyLocal;
            R = treadRadiusWorld;
        }
        else
        {
            center = Rigidbody.position;
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
            if (treadPhysicsGeometryCached)
            {
                axisWorld = Rigidbody.rotation * treadAxisBodyLocal;
                axisWorld.Normalize();
                halfW = 0.5f * treadWidth * treadAxisScaleWorld;
            }
            else if (treadTriggerCapsule != null)
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
        GatherTopContactSamples(center, downDir, R, doClip, axisWorld, halfW, aAxisWheel, N, topSamples);
        lastStepHadRaycastContact = topSamples.Count > 0;
        ResetLateralPatchAccumulators();

        if (topSamples.Count == 0)
        {
            consecutiveNoContactSteps++;
            // 没命中：全部衰减
            ApplyAxleSpinStabilization(0f, 0f, GetDriveAxisWorld());
            // A single adaptive-ray miss must not erase the relaxation/filter
            // state. That rebuilt force from zero every other step and turned
            // longitudinal grip into a weak high-frequency pulse train.
            if (resetShearOnNoContact
                && consecutiveNoContactSteps >= ShearResetNoContactSteps)
                pointStates.Clear();
            DecayAndCleanupColliderStates();
            dbgHasLocal = false;
            CleanupPointStates();
            HideDebugObjects();
            return;
        }

        consecutiveNoContactSteps = 0;

        // ===== 2) Update per-collider gate + normal filter based on current contacts =====
        UpdateColliderStatesFromSamples(topSamples);

        // ===== 3) Apply forces per point =====
        float dtFixed = Time.fixedDeltaTime;

        // Debug: snapshot the exact world point used by AddForceAtPosition.
        // Re-applying the interpolated wheel rotation would move a stationary
        // ground contact around the axle and make the force look mis-injected.
        bool doDbgSample = ShowDebugVisuals && debugDraw && (fixedStepCounter % Mathf.Max(1, drawEveryFixedSteps) == 0);
        if (doDbgSample)
        {
            EnsureDebugLocalCapacity(topSamples.Count);
            dbgLocalCount = 0;
            dbgCenterWorld = center;
            dbgAxisWorld = axisWorld;
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

            float Fn = EvaluateAndApplySupportForce(s, nUse, groundRb, gate, sampleWeight);
            if (Fn <= 1e-6f) continue;

            totalNormalLoadForAxle += Fn;

            ApplyLoadSensitiveRollingResistance(Fn, R, aAxisWheel);
            int lateralPatchIndex = AccumulateLateralPatch(
                s,
                nUse,
                groundRb,
                Fn,
                gate * sampleWeight);

            // ---- Debug 采样（每个接触点分别画 Fn / Ft / normal / contact point）----
            if (doDbgSample && dbgLocalCount < dbgLocalContacts.Length)
            {
                DebugContactLocalData d;
                d.active = true;
                d.pWorld = s.p;
                d.nWorld = nUse;
                d.FnMag = Fn;
                d.FtWorld = Vector3.zero;
                d.FtMag = 0f;
                dbgLocalContacts[dbgLocalCount] = d;
                AttachLateralPatchDebugIndex(lateralPatchIndex, dbgLocalCount);
                dbgLocalCount++;
            }

        }

        ApplyAccumulatedLateralPatches(aAxisWheel, dtFixed, doDbgSample);

        ApplyAxleSpinStabilization(totalNormalLoadForAxle, R, aAxisWheel);

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
        // Render the fixed-step world-space force snapshot. Contact points belong
        // to the contacted surface, not to the tyre's spinning local frame.
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

        Vector3 center = dbgCenterWorld;
        Vector3 axisWorld = dbgAxisWorld;
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

            Vector3 p = d.pWorld;
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
        Vector3 wheelAxisUnit, int N, List<ContactSample> outTop)
    {
        outTop.Clear();
        RecycleHitLists();
        lastOwnMachineHitCount = 0;
        lastRaycastQueryCount = 0;
        lastRaycastRawHitCount = 0;
        lastRaycastAcceptedHitCount = 0;
        lastRaycastSaturatedCount = 0;

        if (!useRaycastContact) return;

        // A2: back the origin off along -rayDir so the ray still starts outside
        // the ground when the wheel centre itself has penetrated the surface.
        float backoff = 0.5f * R;
        float maxDist = R + backoff + Mathf.Max(0f, rayExtra);

        int fanCount = 1;
        bool useRayFan = enableTreadRayFan && doClip && halfWWorld > 1e-4f && axisWorldUnit.sqrMagnitude > 1e-8f;
        if (useRayFan)
        {
            axisWorldUnit.Normalize();
            fanCount = Mathf.Clamp(treadRayCount, 1, 7);
            if (fanCount > 1 && (fanCount % 2) == 0) fanCount = Mathf.Min(7, fanCount + 1);
        }

        // A1: radial directions in the wheel plane, rotated around the axle.
        int dirCount = Mathf.Clamp(radialRayCount, 1, 12);
        Vector3 wheelAxis = wheelAxisUnit.sqrMagnitude > 1e-10f
            ? wheelAxisUnit.normalized
            : GetWheelAxisWorld();

        // D3: one extra ray along the in-plane velocity when a single step
        // covers a meaningful fraction of the radius, so thin obstacles between
        // radial directions are still seen along the approach path.
        Vector3 velDir = Vector3.zero;
        float dtStep = Mathf.Max(1e-5f, Time.fixedDeltaTime);
        Vector3 velocity = Rigidbody.velocity;
        if (velocity.magnitude * dtStep > 0.25f * R)
        {
            velDir = ProjectOnPlane(velocity, wheelAxis);
            if (velDir.sqrMagnitude > 1e-8f) velDir.Normalize();
            else velDir = Vector3.zero;
        }
        bool hasVelDir = velDir.sqrMagnitude > 0.5f;

        int sectorQueryCount = 0;
        if (!enableAdaptiveRadialSampling)
        {
            for (int sector = 0; sector < dirCount; sector++)
                AddRadialQuerySector(sector, radialQuerySectors, ref sectorQueryCount);
        }
        else
        {
            // Ground/gravity direction is always sampled. Keep the last useful
            // non-ground sector (wall, ceiling or loop) hot, then rotate a tiny
            // exploration budget through the remaining resolution sectors.
            AddRadialQuerySector(0, radialQuerySectors, ref sectorQueryCount);
            if (lastSuccessfulRadialSector > 0 && lastSuccessfulRadialSector < dirCount)
                AddRadialQuerySector(lastSuccessfulRadialSector, radialQuerySectors, ref sectorQueryCount);

            if (dirCount > 1)
            {
                int probesWanted = lastStepHadRaycastContact ? 1 : 2;
                int probesAdded = 0;
                int attempts = 0;
                while (probesAdded < probesWanted && attempts < dirCount * 2)
                {
                    if (adaptiveRadialCursor <= 0 || adaptiveRadialCursor >= dirCount)
                        adaptiveRadialCursor = 1;
                    int candidate = adaptiveRadialCursor++;
                    int before = sectorQueryCount;
                    AddRadialQuerySector(candidate, radialQuerySectors, ref sectorQueryCount);
                    if (sectorQueryCount > before) probesAdded++;
                    attempts++;
                }
            }
        }

        int totalDirectionQueries = sectorQueryCount + (hasVelDir ? 1 : 0);
        lastGatherWasMultiRay = totalDirectionQueries * fanCount > 1;

        // A4: the joint parent is part of this machine's axle assembly, never ground.
        Rigidbody parentRb = GetJointParentBody();

        int bestNonGroundSector = -1;
        float bestNonGroundPenetration = -1f;
        for (int d = 0; d < totalDirectionQueries; d++)
        {
            int sector = d < sectorQueryCount ? radialQuerySectors[d] : -1;
            Vector3 rayDir = sector >= 0
                ? Quaternion.AngleAxis((360f * sector) / dirCount, wheelAxis) * downDir
                : velDir;
            float directionBestPenetration = -1f;

            for (int r = 0; r < fanCount; r++)
            {
                float u = 0f;
                if (fanCount > 1) u = -1f + (2f * (float)r) / (float)(fanCount - 1);

                Vector3 rayOrigin = useRayFan ? center + axisWorldUnit * (u * halfWWorld) : center;
                rayOrigin -= rayDir * backoff;
                int hitCount = Physics.RaycastNonAlloc(
                    rayOrigin,
                    rayDir,
                    raycastHitBuffer,
                    maxDist,
                    contactRayMask,
                    QueryTriggerInteraction.Ignore
                );
                lastRaycastQueryCount++;
                lastRaycastRawHitCount += Mathf.Max(0, hitCount);
                if (hitCount >= raycastHitBuffer.Length) lastRaycastSaturatedCount++;

                if (hitCount <= 0) continue;
                if (hitCount > raycastHitBuffer.Length) hitCount = raycastHitBuffer.Length;

                // collect hits and group by collider
                for (int i = 0; i < hitCount; i++)
                {
                    RaycastHit hit = raycastHitBuffer[i];
                    Collider c = hit.collider;
                    if (c == null) continue;

                    if (IsOwnMachineCollider(c, parentRb))
                    {
                        lastOwnMachineHitCount++;
                        continue;
                    }

                    Vector3 p = hit.point;
                    Vector3 n = hit.normal;
                    if (n.sqrMagnitude < 1e-12f) continue;
                    n.Normalize();

                    if (doClip && !IsWithinTreadWidth(p, center, axisWorldUnit, halfWWorld))
                        continue;

                    // Defensive: RaycastHit.distance can be 0 when the origin sits on
                    // the collider surface; the backed-off origin makes that likelier.
                    float hitDist = hit.distance;
                    if (hitDist <= 1e-6f) hitDist = Vector3.Distance(rayOrigin, p);
                    float dist = hitDist - backoff;
                    float pen = R - dist;
                    if (pen > R) pen = R;
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
                    lastRaycastAcceptedHitCount++;
                    if (pen > directionBestPenetration) directionBestPenetration = pen;
                }
            }

            if (sector > 0 && directionBestPenetration > bestNonGroundPenetration)
            {
                bestNonGroundPenetration = directionBestPenetration;
                bestNonGroundSector = sector;
            }
        }

        if (enableAdaptiveRadialSampling)
            lastSuccessfulRadialSector = bestNonGroundSector;

        if (hitsByCol.Count == 0) return;

        int K = Mathf.Clamp(perColliderTopK, 1, 16);

        foreach (var kv in hitsByCol)
        {
            List<HitSample> list = kv.Value;
            if (list == null || list.Count == 0) continue;

            list.Sort(HitSamplePenDescending);
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
            // Weighted penetration average whenever several rays contributed;
            // |center - pAgg| under-reports depth for blended multi-ray points.
            if (lastGatherWasMultiRay)
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

        outTop.Sort(ContactSamplePenDescending);
        if (outTop.Count > N)
            outTop.RemoveRange(N, outTop.Count - N);
    }

    private static void AddRadialQuerySector(int sector, int[] sectors, ref int count)
    {
        if (sectors == null || count >= sectors.Length || sector < 0) return;
        for (int i = 0; i < count; i++)
            if (sectors[i] == sector) return;
        sectors[count++] = sector;
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
        float gateUp = 1f / (float)inFrames;

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
    // Per lateral-patch state helpers
    // =========================
    private TirePointState GetOrCreatePatchState(
        int patchKey,
        Vector3 worldPoint,
        Vector3 worldNormal)
    {
        TirePointState st;
        if (!pointStates.TryGetValue(patchKey, out st))
        {
            st = new TirePointState();
            pointStates.Add(patchKey, st);
        }

        bool reset = false;
        if (st.lastSeenStep > 0)
        {
            if ((worldPoint - st.lastPointWorld).sqrMagnitude
                > StaticStateMaxPointJump * StaticStateMaxPointJump)
                reset = true;
            if (Vector3.Dot(st.lastNormalWorld, worldNormal)
                < StaticStateMinNormalDot)
                reset = true;
        }

        if (reset)
        {
            st.shearDispWorld = Vector3.zero;
            st.FtireFiltered = Vector3.zero;
            ResetStaticConstraintHistory(st);
        }

        if (st.lastSeenStep > 0
            && fixedStepCounter - st.lastSeenStep > 1)
            ResetStaticConstraintHistory(st);

        st.lastPointWorld = worldPoint;
        st.lastNormalWorld = worldNormal;
        st.lastSeenStep = fixedStepCounter;
        return st;
    }

    private void CleanupPointStates()
    {
        if (pointStates.Count == 0) return;

        int ttl = Mathf.Max(1, contactStateTTLSteps);
        int threshold = fixedStepCounter - ttl;

        pointStatesToRemove.Clear();
        foreach (KeyValuePair<int, TirePointState> kv in pointStates)
        {
            if (kv.Value == null) { pointStatesToRemove.Add(kv.Key); continue; }
            if (kv.Value.lastSeenStep < threshold) pointStatesToRemove.Add(kv.Key);
        }

        for (int i = 0; i < pointStatesToRemove.Count; i++)
            pointStates.Remove(pointStatesToRemove[i]);
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
    // Contact query mask
    // =========================
    private int BuildContactRayMask()
    {
        // Historical versions queried only Besiege ground layers 24/29 and
        // optionally layer 0. That was also used as an accidental self-filter,
        // but it missed valid surfaces on layer pairs disabled for trigger
        // callbacks. Raycasts do not use the trigger collision-pair matrix, so
        // keep all layers queryable and filter this machine by identity instead.
        return ~0;
    }

    // =========================
    // 轮轴方向（世界）
    // 默认：优先使用 trigger capsule 的轴；没有则回退 wheelAxisLocal
    // =========================
    private Vector3 GetWheelAxisWorld()
    {
        if (treadPhysicsGeometryCached && HasRigidbody)
        {
            Vector3 axisWorld = Rigidbody.rotation * treadAxisBodyLocal;
            if (axisWorld.sqrMagnitude > 1e-10f) return axisWorld.normalized;
        }

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

    private bool IsOwnMachineCollider(Collider candidate, Rigidbody directParent)
    {
        if (candidate == null) return true;

        Rigidbody candidateBody = candidate.attachedRigidbody;
        if (candidateBody == Rigidbody) return true;
        if (directParent != null && candidateBody == directParent) return true;
        if (ownMachineColliders.Contains(candidate)) return true;

        // Eight-direction contact rays can see suspension, bodywork and other
        // blocks that the legacy downward ray never crossed. Treating those as
        // ground creates equal-and-opposite forces inside one machine: the VIZ
        // arrows look valid, but the machine receives no net support or grip.
        Transform machineRoot = ownSimulationRoot;
        Transform candidateTransform = candidate.transform;
        return machineRoot != null
            && candidateTransform != null
            && (candidateTransform == machineRoot
                || candidateTransform.IsChildOf(machineRoot));
    }

    private void CacheOwnMachineColliders()
    {
        ownMachineColliders.Clear();
        ownSimulationRoot = Machine != null ? Machine.SimulationMachine : null;

        // Prefer the public modding API's simulation block list. It remains
        // accurate when Besiege reparents individual rigidbody clusters and a
        // simple IsChildOf(machineRoot) test would no longer be sufficient.
        if (Machine != null && Machine.SimulationBlocks != null)
        {
            for (int i = 0; i < Machine.SimulationBlocks.Count; i++)
            {
                Modding.Blocks.Block block = Machine.SimulationBlocks[i];
                if (block == null || block.GameObject == null) continue;
                Collider[] blockColliders = block.GameObject.GetComponentsInChildren<Collider>(true);
                for (int c = 0; c < blockColliders.Length; c++)
                {
                    if (blockColliders[c] != null)
                        ownMachineColliders.Add(blockColliders[c]);
                }
            }
        }

        // Fallback for loader variants that have not populated SimulationBlocks
        // when OnSimulateStart runs.
        if (ownSimulationRoot != null)
        {
            Collider[] rootedColliders = ownSimulationRoot.GetComponentsInChildren<Collider>(true);
            for (int i = 0; i < rootedColliders.Length; i++)
            {
                if (rootedColliders[i] != null)
                    ownMachineColliders.Add(rootedColliders[i]);
            }
        }
    }

    private Vector3 GetDriveAxisWorld()
    {
        Vector3 a = GetWheelAxisWorld();
        return (a.sqrMagnitude > 1e-10f) ? a.normalized : transform.up;
    }

    private float GetSpinCap()
    {
        return Mathf.Min(Mathf.Max(10f, maxAngularVelocityLimit), SpinHardCap);
    }

    // A5: immediate joint parent for relative angular velocity. Cached per
    // simulation; Besiege builds the simulation joints at simulation start.
    private Rigidbody GetJointParentBody()
    {
        if (!jointParentCached)
        {
            jointParentCached = true;
            Joint joint = GetComponent<Joint>();
            jointParentBody = (joint != null && joint.connectedBody != Rigidbody)
                ? joint.connectedBody
                : null;
        }
        return jointParentBody;
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

    private void CacheTreadPhysicsGeometry()
    {
        treadPhysicsGeometryCached = false;
        if (!HasRigidbody || treadTriggerCapsule == null) return;

        Transform triggerTransform = treadTriggerCapsule.transform;
        Vector3 centerWorld = triggerTransform.TransformPoint(treadTriggerCapsule.center);
        Vector3 axisWorld;
        switch (treadTriggerCapsule.direction)
        {
            case 0: axisWorld = triggerTransform.right; break;
            case 1: axisWorld = triggerTransform.up; break;
            default: axisWorld = triggerTransform.forward; break;
        }
        if (axisWorld.sqrMagnitude < 1e-10f) return;

        Quaternion bodyInverse = Quaternion.Inverse(Rigidbody.rotation);
        treadCenterBodyLocal = bodyInverse * (centerWorld - Rigidbody.position);
        treadAxisBodyLocal = (bodyInverse * axisWorld).normalized;
        Vector3 triggerScale = triggerTransform.lossyScale;
        treadRadiusWorld = treadTriggerCapsule.radius * MaxAbsComponent(triggerScale);
        switch (treadTriggerCapsule.direction)
        {
            case 0: treadAxisScaleWorld = Mathf.Abs(triggerScale.x); break;
            case 1: treadAxisScaleWorld = Mathf.Abs(triggerScale.y); break;
            default: treadAxisScaleWorld = Mathf.Abs(triggerScale.z); break;
        }
        treadPhysicsGeometryCached = treadRadiusWorld > 1e-5f;
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
        bool createdRoot = (dbgRoot == null);
        if (createdRoot)
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

        // C2: start hidden only on first creation. Hiding on every call made
        // SimulateLateUpdateAlways toggle the whole hierarchy off/on each frame.
        if (createdRoot) HideDebugObjects();
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
