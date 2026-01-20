using System.Collections.Generic;
using UnityEngine;
using Modding;

/// <summary>
/// RubberTireWheelScript
/// - 单接触点（pen 最大）
/// - 法向弹簧阻尼 + 轮胎摩擦（Bristle/静-动摩擦，基于接触点相对速度）
/// - 轮胎切向松弛/形变：剪切位移状态 + 弹簧-阻尼（低通）
/// - 反驱友好：切向力/轴向扭矩分离（AddForce + AddTorque）
/// - Debug 可视化（旧版 LineRenderer API：SetVertexCount/SetWidth/SetColors）
/// - 游戏内 UI 调参
/// </summary>
public class RubberTireWheelScript : BlockScript
{
    // =========================
    // 物理参数（会被 UI 覆盖）
    // =========================
    public float springK = 3000f;          // 法向弹簧刚度
    public float damperC = 25f;            // 法向阻尼
    public float maxNormalForce = 200000f; // 法向力上限

    public bool enableTireModel = true;    // 是否启用轮胎摩擦模型
    // =========================
    // 静/动摩擦 + 输出滤波（解决低速抖动/低速粘滞）
    // =========================
    public float muStatic = 1.40f;         // 静摩擦峰值系数 μs（应 >= μk）
    public float muKinetic = 1.20f;        // 动摩擦系数 μk
    public float vStatic = 0.25f;          // 静摩擦锁定速度阈值（m/s），越大越“稳”但更粘
    public float forceFilterTau = 0.03f;   // 输出切向力滤波时间常数（s），0 表示关闭

    public float vEps = 0.2f;              // 速度防除零
    // =========================
    // 切向松弛/形变（低通）
    // =========================
    public bool enableTireRelaxation = true;  // 是否启用松弛
    public float relaxLength = 0.8f;         // 松弛长度（世界单位，约等于轮胎形变建立的距离）
    public float shearK = 50000f;             // 剪切刚度（N/m）
    public float shearC = 0f;                 // 剪切阻尼（N*s/m），0 表示只用 relaxLength/speed 的一阶耗散
    public float maxShearDisp = 0.15f;        // 形变位移上限（m）
    public bool resetShearOnNoContact = true; // 无接触时是否清零形变

    // =========================
    // 反驱：力/扭矩分离
    // =========================
    public bool decoupleTireForceAndTorque = true;

    // =========================
    // 踏面裁切（把球形触发域裁成“有限宽度圆柱体”）
    // =========================
    public bool enableTreadWidthClip = true;
    public float treadWidth = 1.0f;        // 踏面总宽（世界单位；会乘以轴向缩放）

    // =========================
    // 接触点获取
    // =========================
    public bool useRaycastContact = true;
    public float rayExtra = 0.6f;
    public bool useWorldDown = true;

    // 轮轴方向（局部）
    public enum AxisLocal { X, Y, Z }
    // 注意：你已验证 BasePoint Motion 的自由旋转轴是 Y，因此这里默认用 Y。
    // 若你的模型/碰撞体确实绕其它轴转，再改这里（或打开 useTriggerCapsuleAxisForWheelAxis）。
    public AxisLocal wheelAxisLocal = AxisLocal.Y;

    // 可选：如果你确认“以 trigger capsule 轴为准”且不会与 BasePoint Motion 冲突，可打开它
    public bool useTriggerCapsuleAxisForWheelAxis = false;

    // 可选：测试用驱动扭矩（一般保持 0）
    public float driveTorque = 0f;

    // =========================
    // 角速度上限（防止甩飞/打坏连接点）
    // Unity 默认 maxAngularVelocity 往往很小（~7 rad/s），你已验证需要放大。
    // 但若完全放开，抓地不足时会空转到极限转速，冲击连接点。
    // =========================
    public float maxAngularVelocityLimit = 250f; // rad/s

    // =========================
    // 简易驱动/制动（可重绑定按键）：先让模型能跑
    // - 直接对轮子刚体绕轮轴施加扭矩（AddTorque）
    // - 通过 AddKey 暴露 3 个按键：油门 / 刹车 / 倒车（可在 Mapper 里改键）
    // =========================
    public bool enableDriveBrake = true;

    // 若发现“油门方向反了”，可勾选反向。
    public bool invertDriveTorque = false;
    public float maxDriveTorque = 8000f;   // N*m（按你轮胎/质量缩放）
    public float maxBrakeTorque = 12000f; // N*m
    public float brakeDeadbandOmega = 0.5f; // rad/s，低速进入“锁止”模式
    public float brakeHoldK = 2000f;        // N*m per (rad/s)，低速锁止阻尼

    // 输入平滑（避免突变）
    public float throttleRise = 8f; // 1/s
    public float throttleFall = 10f; // 1/s
    public float brakeRise = 12f; // 1/s
    public float brakeFall = 14f; // 1/s

    // =========================
    // Debug 可视化（会被 UI 调）
    // =========================
    public bool debugDraw = true;

    // DebugViz：轮胎力（Ftire）
    public bool debugVizTireForce = true;

    // 踏面范围与轴心/接触指示（与 DebugViz 独立）
    public bool debugVizTreadAndAxis = true;

    public float thinLineWidth = 0.05f;    // 细线宽
    public float forceLineWidth = 0.10f;   // 力线宽

    public float normalArrowLen = 0.6f;

    public float forceToLength = 0.015f;   // 力->长度映射
    public float arrowMinLen = 0.25f;
    public float arrowMaxLen = 4.0f;

    public int drawEveryFixedSteps = 1;    // 每几帧画一次
    // Debug：踏面裁切范围可视化
    public int treadRangeSegments = 28;
    public float treadAxisDebugLen = 2.0f;

    // =========================
    // UI（Mapper）句柄
    // =========================
    private MSlider uiK, uiC, uiMuS, uiMuK, uiVStatic, uiFTau, uiLineW, uiForceScale, uiTreadW;
    private MToggle uiDbg, uiTreadClip, uiEnableTire;

    // Debug sub-toggles
    private MToggle uiDbgForce, uiDbgTreadAxis;

    private MToggle uiRelax, uiDecouple, uiUseTrigAxis;
    private MSlider uiRelaxL, uiShearK, uiShearC, uiMaxShear;

    // Drive/Brake UI（可选，只调参数，不映射按键）
    private MToggle uiDriveBrake;
    private MToggle uiInvertDrive;
    private MKey uiKeyThrottle, uiKeyBrake, uiKeyReverse;
    private MSlider uiMaxDriveTorque, uiMaxBrakeTorque, uiBrakeDeadband, uiBrakeHoldK;
    private MSlider uiThrottleRise, uiThrottleFall, uiBrakeRise, uiBrakeFall;

    // Angular velocity limit UI
    private MSlider uiMaxAngVel;

    // =========================
    // 运行时状态
    // =========================
    private readonly HashSet<Collider> contacts = new HashSet<Collider>();
    private CapsuleCollider treadTriggerCapsule;
    private int fixedStepCounter = 0;

    // 切向剪切形变状态（世界坐标）
    private Vector3 shearDispWorld = Vector3.zero;

    // 切向力输出滤波状态（世界坐标）
    private Vector3 FtireFiltered = Vector3.zero;

    // 键盘油门/刹车状态（0~1）
    private float throttle01 = 0f;
    private float brake01 = 0f;

    // Debug 对象（保持旧版实现）
    private GameObject dbgRoot;
    private LineRenderer lrCenterToP;
    private LineRenderer lrNormal;
    private LineRenderer lrForceN;
    private LineRenderer lrForceT;

    // Debug：踏面范围（轴线 + 两侧圆环）
    private LineRenderer lrTreadAxis;
    private LineRenderer lrTreadRingA;
    private LineRenderer lrTreadRingB;
    private GameObject dbgPointSphere;

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
        uiUseTrigAxis = AddToggle("Use Trigger Axis", "trigAxis", useTriggerCapsuleAxisForWheelAxis);

        uiDbg = AddToggle("Debug Master", "dbg", debugDraw);
        uiDbgForce = AddToggle("DebugViz: Tire Force", "dbgF", debugVizTireForce);
        uiDbgTreadAxis = AddToggle("Debug: Tread+Axis", "dbgTA", debugVizTreadAndAxis);

        uiTreadClip = AddToggle("Tread Width Clip", "tw-clip", enableTreadWidthClip);
        uiTreadW = AddSlider("Tread Width", "tw", treadWidth, 0.05f, 5f);

        uiLineW = AddSlider("Line Width", "lw", forceLineWidth, 0.01f, 0.30f);
        uiForceScale = AddSlider("Force Scale", "fs", forceToLength, 0.00001f, 0.01f);

        // -------- Drive/Brake (keyboard) parameter UI --------
        uiDriveBrake = AddToggle("Drive/Brake (Keys)", "drv", enableDriveBrake);
        uiInvertDrive = AddToggle("Invert Drive Torque", "invDrv", invertDriveTorque);

        // 可重绑定按键（每个轮子实例独立）
        // 默认：T 油门，G 刹车，R 倒车
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
        uiBrakeFall = AddSlider("Brake Fall 1/s", "brkDn", brakeFall, 0f, 40f);

        // Angular velocity limit (rad/s)
        uiMaxAngVel = AddSlider("Max Angular Vel (rad/s)", "maxW", maxAngularVelocityLimit, 10f, 1000f);
    }

    public override void OnSimulateStart()
    {
        contacts.Clear();
        fixedStepCounter = 0;
        shearDispWorld = Vector3.zero;
        FtireFiltered = Vector3.zero;
        // Apply angular velocity ceiling (prevents free-spinning to extreme rpm when traction is low)
        Rigidbody.maxAngularVelocity = Mathf.Max(10f, maxAngularVelocityLimit);
        Rigidbody.angularDrag = 0.05f; 

        treadTriggerCapsule = FindTreadTriggerCapsule();

        if (ShowDebugVisuals && debugDraw)
            EnsureDebugObjects();
    }

    public override void OnSimulateStop()
    {
        contacts.Clear();
        DestroyDebugObjects();
        treadTriggerCapsule = null;
        shearDispWorld = Vector3.zero;
        FtireFiltered = Vector3.zero;
    }

    public override void OnSimulateTriggerEnter(Collider other)
    {
        if (other != null) contacts.Add(other);
    }

    public override void OnSimulateTriggerExit(Collider other)
    {
        if (other != null) contacts.Remove(other);
    }

    public override void SimulateFixedUpdateAlways()
    {
        fixedStepCounter++;

        if (!IsSimulating || !HasRigidbody) return;

        SyncParamsFromUI();

        // 角速度上限：避免抓地不足时空转到极限转速、冲击连接点。
        // 同时也防止 Unity/其它系统把它改回默认小值。
        Rigidbody.maxAngularVelocity = Mathf.Max(10f, maxAngularVelocityLimit);

        // ===== 0) Drive/Brake (remappable keys) =====
        // 目标：先让轮子能在游戏里被“油门/刹车”驱动起来。
        // 重要：这里不改你的轮胎摩擦模型，只是对刚体绕轮轴施加额外扭矩。
        // 注：若你后续要做真实发动机/传动比/ABS，可以直接替换这里的 tauDrive/tauBrake 计算。
        {
            float dt = Time.fixedDeltaTime;

            // 读取按键（AddKey -> MKey，可在 Mapper 里改键）
            bool heldThr = enableDriveBrake && uiKeyThrottle != null && uiKeyThrottle.IsHeld;
            bool heldBrk = enableDriveBrake && uiKeyBrake != null && uiKeyBrake.IsHeld;
            bool heldRev = enableDriveBrake && uiKeyReverse != null && uiKeyReverse.IsHeld;

            float targetThr = heldThr ? 1f : 0f;
            float targetBrk = heldBrk ? 1f : 0f;

            // 一阶平滑（避免扭矩突变）
            float thrRate = (targetThr > throttle01) ? Mathf.Max(0f, throttleRise) : Mathf.Max(0f, throttleFall);
            float brkRate = (targetBrk > brake01) ? Mathf.Max(0f, brakeRise) : Mathf.Max(0f, brakeFall);
            throttle01 = Mathf.MoveTowards(throttle01, targetThr, thrRate * dt);
            brake01 = Mathf.MoveTowards(brake01, targetBrk, brkRate * dt);

            // 轮轴（世界）
            // 关键：用于“驱动/刹车”扭矩的轴必须与轮子实际自由旋转轴一致。
            // 最稳是直接使用踏面 trigger capsule 的轴（由 XML 的 Capsule direction + Rotation 决定）。
            Vector3 aAxis = GetDriveAxisWorld();
            float omegaAxis = Vector3.Dot(Rigidbody.angularVelocity, aAxis);

            // 驱动扭矩（油门）
            float tauDrive = throttle01 * maxDriveTorque;

            // 建造态 Flip（F 反转）：用于左右镜像轮子方向统一。
            // XML 已启用 <CanFlip>true</CanFlip> 时，BlockScript 会维护 Flipped 状态。
            // 这里仅作用于“驱动/制动扭矩”的符号，不影响摩擦/接触模型。
            float flipSign = Flipped ? -1f : 1f;
            float userSign = invertDriveTorque ? -1f : 1f;
            float revSign = heldRev ? -1f : 1f;
            float driveSign = flipSign * userSign * revSign;

            // 合成扭矩：驱动（带方向）+ 制动（始终反向）
            float tau = tauDrive * driveSign;

            // 制动扭矩：高速为恒定反向，低速进入“锁止”避免符号翻转抖动
            if (brake01 > 1e-4f)
            {
                float tauBMax = brake01 * maxBrakeTorque;
                if (Mathf.Abs(omegaAxis) > Mathf.Max(1e-4f, brakeDeadbandOmega))
                {
                    tau += -Mathf.Sign(omegaAxis) * tauBMax;
                }
                else
                {
                    // 低速锁止：把 omegaAxis 拉向 0（等价于粘性制动）
                    float tauHold = -omegaAxis * brakeHoldK;
                    tau += Mathf.Clamp(tauHold, -tauBMax, tauBMax);
                }
            }

            if (Mathf.Abs(tau) > 1e-6f)
                Rigidbody.AddTorque(aAxis * tau, ForceMode.Force);
        }

        if (contacts.Count == 0)
        {
            if (resetShearOnNoContact) shearDispWorld = Vector3.zero;
            if (resetShearOnNoContact) FtireFiltered = Vector3.zero;
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

        Vector3 downDir = useWorldDown ? Vector3.down : -transform.up;

        // ===== 踏面裁切参数（世界）=====
        Vector3 axisWorld = Vector3.right;
        float halfW = 0f;
        bool doClip = enableTreadWidthClip && treadWidth > 0f;

        if (doClip)
        {
            if (treadTriggerCapsule != null)
            {
                Transform t = treadTriggerCapsule.transform;
                switch (treadTriggerCapsule.direction) // 0=X 1=Y 2=Z
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
                axisWorld = GetWheelAxisWorld().normalized;
                halfW = 0.5f * treadWidth * GetAxisScaleWorld();
            }
        }

        // ===== 单接触点：选 penetration 最大的那个 =====
        bool hasBest = false;
        Vector3 pBest = Vector3.zero;
        Vector3 nBest = Vector3.up;
        float penBest = -1f;
        Collider colBest = null;

        foreach (var col in contacts)
        {
            if (col == null) continue;

            Vector3 p, n;
            if (!TryGetContact(col, center, downDir, R, out p, out n)) continue;

            if (doClip && !IsWithinTreadWidth(p, center, axisWorld, halfW))
                continue;

            float pen = R - Vector3.Distance(center, p);
            if (pen <= 0f) continue;

            if (pen > penBest)
            {
                penBest = pen;
                pBest = p;
                nBest = n;
                colBest = col;
                hasBest = true;
            }
        }

        if (!hasBest)
        {
            HideDebugObjects();
            return;
        }

        // ===== 1) 法向力 =====
        float Fn = springK * penBest;

        if (damperC > 0f)
        {
            Vector3 vAtP = Rigidbody.GetPointVelocity(pBest);
            float vN = Vector3.Dot(vAtP, nBest);
            float compressionSpeed = -vN;
            if (compressionSpeed > 0f) Fn += damperC * compressionSpeed;
        }

        Fn = Mathf.Clamp(Fn, 0f, maxNormalForce);
        Vector3 FnVec = Fn * nBest;
        Rigidbody.AddForceAtPosition(FnVec, pBest, ForceMode.Force);

        // ===== 2) 轮胎摩擦模型 =====
        Vector3 Ftire = Vector3.zero;

        if (enableTireModel && Fn > 0f)
        {
            Rigidbody groundRb = (colBest != null) ? colBest.attachedRigidbody : null;

            Vector3 vGround = Vector3.zero;
            if (groundRb != null) vGround = groundRb.GetPointVelocity(pBest);

            Vector3 vWheel = Rigidbody.GetPointVelocity(pBest);
            Vector3 vRel = vWheel - vGround;

            Vector3 a = GetWheelAxisWorld().normalized;

            // 纵向/侧向基向量（对镜像更稳定）
            Vector3 f = ProjectOnPlane(Vector3.Cross(nBest, a), nBest);
            if (f.sqrMagnitude < 1e-6f)
                f = ProjectOnPlane(Vector3.Cross(nBest, transform.right), nBest);

            if (f.sqrMagnitude < 1e-6f)
            {
                Ftire = Vector3.zero;
            }
            else
            {
                f.Normalize();
                Vector3 sdir = Vector3.Cross(nBest, f);
                if (sdir.sqrMagnitude > 1e-6f) sdir.Normalize();            // ---------- 2.1) Bristle 静/动摩擦（方案1：完全基于接触点切向相对速度） ----------
                // vRel = vWheel(pBest) - vGround(pBest)
                // vSlip = ProjectOnPlane(vRel, nBest)
                // 形变状态 shearDispWorld 积分 vSlip，并用弹簧/阻尼输出摩擦力（对 wheel 的作用方向为 -shearK*x - shearC*xDot）

                // 接触点切向相对速度 = 滑移速度（核心：不再使用 ωR，因此不受“压陷导致有效半径变化”的系统性误差影响）
                Vector3 vSlip = ProjectOnPlane(vRel, nBest);
                float vSlipMag = vSlip.magnitude;

                // Bristle/松弛
                Vector3 Fraw;
                float dt = Time.fixedDeltaTime;

                if (enableTireRelaxation)
                {
                    // 松弛长度模型：T = L / |vT|，这里用切向相对速度的大小近似（避免静止时 T 过小）
                    float speed = Mathf.Max(vSlipMag, vEps);
                    float T = Mathf.Max(1e-4f, relaxLength / speed);

                    // 一阶松弛：xDot = vSlip - x/T
                    Vector3 xDot = vSlip - shearDispWorld / T;
                    shearDispWorld += xDot * dt;

                    // 限制形变位移（物理上是最大剪切形变）
                    if (maxShearDisp > 1e-5f)
                    {
                        float m = shearDispWorld.magnitude;
                        if (m > maxShearDisp) shearDispWorld = shearDispWorld * (maxShearDisp / m);
                    }

                    // 摩擦力方向：对 wheel 施加“反向弹力”
                    Fraw = -shearK * shearDispWorld;
                    if (shearC > 0f) Fraw += -shearC * xDot;

                    // 静/动摩擦上限
                    float FmaxS = muStatic * Fn;
                    float FmaxK = muKinetic * Fn;
                    float Fmag = Fraw.magnitude;

                    if (vSlipMag < Mathf.Max(1e-3f, vStatic))
                    {
                        // 静摩擦锁定区：只要没超过静摩擦上限，就按 bristle 输出（避免低速翻转震荡）
                        if (Fmag > FmaxS && Fmag > 1e-6f)
                        {
                            // 超过静摩擦极限 -> 进入滑移：限制到动摩擦上限
                            Fraw *= (FmaxK / Fmag);

                            // 关键：把形变重投影到饱和值，避免下一帧立刻反弹
                            shearDispWorld = -Fraw / Mathf.Max(1e-6f, shearK);
                        }
                    }
                    else
                    {
                        // 明显滑移：动摩擦上限
                        if (Fmag > FmaxK && Fmag > 1e-6f)
                        {
                            Fraw *= (FmaxK / Fmag);
                            shearDispWorld = -Fraw / Mathf.Max(1e-6f, shearK);
                        }
                    }

                    // 输出力轻滤波（抑制接触点离散跳动）
                    if (forceFilterTau > 1e-5f)
                    {
                        float aFilt = dt / (forceFilterTau + dt);
                        FtireFiltered = Vector3.Lerp(FtireFiltered, Fraw, aFilt);
                        Fraw = FtireFiltered;
                    }
                    else
                    {
                        FtireFiltered = Fraw;
                    }
                }
                else
                {
                    // 关闭松弛时：直接库仑动摩擦（仍基于接触点切向相对速度）
                    float FmaxK = muKinetic * Fn;
                    if (vSlipMag > 1e-5f)
                        Fraw = -vSlip / vSlipMag * FmaxK;
                    else
                        Fraw = Vector3.zero;

                    // 同步状态
                    FtireFiltered = Fraw;
                    shearDispWorld = Vector3.zero;
                }

                Ftire = Fraw;
// ---------- 2.4) 反驱：接触点施力（闭合） ----------
                if (decoupleTireForceAndTorque)
                {
                    // 力施加在接触点：自然产生 r×F，实现反驱闭合
                    Rigidbody.AddForceAtPosition(Ftire, pBest, ForceMode.Force);
                    if (groundRb != null)
                        groundRb.AddForceAtPosition(-Ftire, pBest, ForceMode.Force);

                    // 驱动扭矩仍然可以单独沿轴施加（相当于发动机输入）
                    if (Mathf.Abs(driveTorque) > 1e-6f)
                    {
                        float flipSign = Flipped ? -1f : 1f;
                        Rigidbody.AddTorque(GetDriveAxisWorld() * (driveTorque * flipSign), ForceMode.Force);
                    }
                }
                else
                {
                    Rigidbody.AddForceAtPosition(Ftire, pBest, ForceMode.Force);
                    if (groundRb != null) groundRb.AddForceAtPosition(-Ftire, pBest, ForceMode.Force);

                    if (Mathf.Abs(driveTorque) > 1e-6f)
                    {
                        float flipSign = Flipped ? -1f : 1f;
                        Rigidbody.AddTorque(GetDriveAxisWorld() * (driveTorque * flipSign), ForceMode.Force);
                    }
                }
            }
        }

        // ===== Debug 可视化（保持旧版绘制实现）=====
        if (ShowDebugVisuals && debugDraw && (fixedStepCounter % Mathf.Max(1, drawEveryFixedSteps) == 0))
        {
            EnsureDebugObjects();
            ShowDebugObjects();

            // 新逻辑：
            // - DebugViz = 轮胎力 Ftire（独立开关）
            // - 踏面/轴线范围 = 独立开关
            // - 接触点/法向/法向力箭头：作为“基础接触信息”，只要 Debug Master 开着就显示
            //   （你关掉踏面/轴线后仍需要看到接触点与法向力）。
            bool showForceViz = debugVizTireForce;
            bool showTreadAxisViz = debugVizTreadAndAxis;

            // legacy：若旧开关 debugVizTreadAndAxis 被打开，在 SyncParamsFromUI() 里已映射为：
            // showGeomViz=true 且 showForceViz=false。

            // 基础接触信息：默认始终显示（由 Debug Master 控制）
            SetLRVisible(lrCenterToP, true);
            SetLRVisible(lrNormal, true);
            SetLRVisible(lrForceN, true);

            // 轮胎力：独立开关
            SetLRVisible(lrForceT, showForceViz);

            // 接触点球：若无接触则隐藏
            if (dbgPointSphere != null) dbgPointSphere.SetActive(hasBest);

            if (hasBest)
            {
                SetLine(lrCenterToP, center, pBest);
                SetLine(lrNormal, pBest, pBest + nBest * normalArrowLen);

                float nLen = Mathf.Clamp(Fn * forceToLength, arrowMinLen, arrowMaxLen);
                SetLine(lrForceN, pBest, pBest + nBest * nLen);

                if (dbgPointSphere != null) dbgPointSphere.transform.position = pBest;
            }
            else
            {
                // 无接触：收起线段，避免显示上一次的残影
                SetLine(lrCenterToP, center, center);
                SetLine(lrNormal, center, center);
                SetLine(lrForceN, center, center);
                if (showForceViz) SetLine(lrForceT, center, center);
            }

            if (showForceViz)
            {
                float tMag = Ftire.magnitude;
                if (tMag > 1e-3f)
                {
                    Vector3 tDir = Ftire / tMag;
                    float tLen = Mathf.Clamp(tMag * forceToLength, arrowMinLen, arrowMaxLen);
                    SetLine(lrForceT, pBest, pBest + tDir * tLen);
                }
                else
                {
                    SetLine(lrForceT, pBest, pBest);
                }
            }

            // 踏面范围/轴线（仅在启用裁切时绘制）
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
        if (uiUseTrigAxis != null) useTriggerCapsuleAxisForWheelAxis = uiUseTrigAxis.IsActive;

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

        // Drive/Brake params
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
    // 接触点求解：Raycast 命中指定 collider
    // =========================
    private bool TryGetContact(Collider col, Vector3 center, Vector3 downDir, float R, out Vector3 p, out Vector3 n)
    {
        p = Vector3.zero;
        n = Vector3.up;

        if (useRaycastContact)
        {
            float maxDist = R + Mathf.Max(0f, rayExtra);
            RaycastHit[] hits = Physics.RaycastAll(center, downDir, maxDist, ~0);

            bool found = false;
            float bestDist = float.MaxValue;
            RaycastHit bestHit = new RaycastHit();

            for (int i = 0; i < hits.Length; i++)
            {
                if (hits[i].collider != col) continue;
                if (hits[i].distance < bestDist)
                {
                    bestDist = hits[i].distance;
                    bestHit = hits[i];
                    found = true;
                }
            }
            if (!found) return false;

            p = bestHit.point;
            n = bestHit.normal.normalized;
            return true;
        }
        else
        {
            Bounds b = col.bounds;
            p = ClosestPointOnAABB(b, center);
            Vector3 nn = center - p;
            float nnMag = nn.magnitude;
            if (nnMag < 1e-6f) return false;
            n = nn / nnMag;
            return true;
        }
    }

    // =========================
    // 轮轴方向（世界）
    // =========================
    private Vector3 GetWheelAxisWorld()
    {
        if (useTriggerCapsuleAxisForWheelAxis && treadTriggerCapsule != null)
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
        return transform.right;
    }

    private Vector3 GetDriveAxisWorld()
    {
        // ✅ 最稳：用踏面 trigger capsule 的轴
        if (treadTriggerCapsule != null)
        {
            Transform t = treadTriggerCapsule.transform;
            Vector3 axis;
            switch (treadTriggerCapsule.direction) // 0=X 1=Y 2=Z
            {
                case 0: axis = t.right; break;
                case 1: axis = t.up; break;
                default: axis = t.forward; break;
            }
            if (axis.sqrMagnitude > 1e-10f) return axis.normalized;
        }

        // 兜底：回退到原 GetWheelAxisWorld（但建议你尽量让上面生效）
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
    // Debug：显示踏面裁切范围（两侧圆环 + 轮轴线）——保持旧版
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

    private Vector3 ClosestPointOnAABB(Bounds b, Vector3 point)
    {
        Vector3 q;
        q.x = Mathf.Clamp(point.x, b.min.x, b.max.x);
        q.y = Mathf.Clamp(point.y, b.min.y, b.max.y);
        q.z = Mathf.Clamp(point.z, b.min.z, b.max.z);
        return q;
    }

    // =========================
    // Debug 可视化：对象创建/销毁 ——保持旧版（古董 API）
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
