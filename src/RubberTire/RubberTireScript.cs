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
    public AxisLocal wheelAxisLocal = AxisLocal.X;

    // 可选：如果你确认“以 trigger capsule 轴为准”且不会与 BasePoint Motion 冲突，可打开它
    public bool useTriggerCapsuleAxisForWheelAxis = false;

    // 可选：测试用驱动扭矩（一般保持 0）
    public float driveTorque = 0f;

    // =========================
    // Debug 可视化（会被 UI 调）
    // =========================
    public bool debugDraw = true;

    public float thinLineWidth = 0.05f;    // 细线宽
    public float forceLineWidth = 0.10f;   // 力线宽

    public float normalArrowLen = 0.6f;

    public float forceToLength = 0.015f;   // 力->长度映射
    public float arrowMinLen = 0.25f;
    public float arrowMaxLen = 4.0f;

    public int drawEveryFixedSteps = 1;    // 每几帧画一次

    // Debug：踏面裁切范围可视化
    public bool debugDrawTreadRange = true;
    public bool treadVizOnly = false;      // 仅显示踏面范围与轴线
    public int treadRangeSegments = 28;
    public float treadAxisDebugLen = 2.0f;

    // =========================
    // UI（Mapper）句柄
    // =========================
    private MSlider uiK, uiC, uiMuS, uiMuK, uiVStatic, uiFTau, uiLineW, uiForceScale, uiTreadW;
    private MToggle uiDbg, uiTreadClip, uiTreadVizOnly, uiEnableTire;

    private MToggle uiRelax, uiDecouple, uiUseTrigAxis;
    private MSlider uiRelaxL, uiShearK, uiShearC, uiMaxShear;

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

        uiDbg = AddToggle("Debug Draw", "dbg", debugDraw);

        uiTreadClip = AddToggle("Tread Width Clip", "tw-clip", enableTreadWidthClip);
        uiTreadW = AddSlider("Tread Width", "tw", treadWidth, 0.05f, 5f);

        uiTreadVizOnly = AddToggle("Tread Viz Only", "treadVizOnly", treadVizOnly);

        uiLineW = AddSlider("Line Width", "lw", forceLineWidth, 0.01f, 0.30f);
        uiForceScale = AddSlider("Force Scale", "fs", forceToLength, 0.00001f, 0.01f);
    }

    public override void OnSimulateStart()
    {
        contacts.Clear();
        fixedStepCounter = 0;
        shearDispWorld = Vector3.zero;
        FtireFiltered = Vector3.zero;

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

        //蜘蛛社我草你妈的为什么这个不暴露在XML里要手动覆盖
        if (Rigidbody.maxAngularVelocity < 50f)
            Rigidbody.maxAngularVelocity = 1000f;

        if (!IsSimulating || !HasRigidbody) return;

        SyncParamsFromUI();

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

            // IMPORTANT (Scheme 1, robust):
            // Avoid using Rigidbody.GetPointVelocity(pBest) directly as "wheel surface velocity",
            // because Rigidbody.angularVelocity may contain non-wheel components (pitch/yaw of the block),
            // which can create artificial slip at higher speeds and look like a speed limiter.
            // Instead, reconstruct the wheel surface velocity at the contact point using:
            //   - hub linear velocity at wheel center
            //   - spin about the wheel axis only
            //   - effective radius vector to the contact point (projected to be perpendicular to the axis)

            Vector3 a = GetWheelAxisWorld().normalized;

            Vector3 vHub = Rigidbody.GetPointVelocity(center);

            // effective lever arm from axis to contact point (perpendicular to axis)
            Vector3 r0 = pBest - center;
            Vector3 rPerp = r0 - a * Vector3.Dot(r0, a);

            // keep only wheel spin component about axis
            Vector3 omegaAxis = a * Vector3.Dot(Rigidbody.angularVelocity, a);
            Vector3 vSpin = Vector3.Cross(omegaAxis, rPerp);

            Vector3 vWheelSurface = vHub + vSpin;
            Vector3 vRel = vWheelSurface - vGround;

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
                        Rigidbody.AddTorque(a * driveTorque, ForceMode.Force);
                }
                else
                {
                    Rigidbody.AddForceAtPosition(Ftire, pBest, ForceMode.Force);
                    if (groundRb != null) groundRb.AddForceAtPosition(-Ftire, pBest, ForceMode.Force);

                    if (Mathf.Abs(driveTorque) > 1e-6f)
                        Rigidbody.AddTorque(a * driveTorque, ForceMode.Force);
                }
            }
        }

        // ===== Debug 可视化（保持旧版绘制实现）=====
        if (ShowDebugVisuals && debugDraw && (fixedStepCounter % Mathf.Max(1, drawEveryFixedSteps) == 0))
        {
            EnsureDebugObjects();
            ShowDebugObjects();

            if (treadVizOnly)
            {
                SetLRVisible(lrCenterToP, false);
                SetLRVisible(lrNormal, false);
                SetLRVisible(lrForceN, false);
                SetLRVisible(lrForceT, false);
                if (dbgPointSphere != null) dbgPointSphere.SetActive(false);
            }
            else
            {
                SetLRVisible(lrCenterToP, true);
                SetLRVisible(lrNormal, true);
                SetLRVisible(lrForceN, true);
                SetLRVisible(lrForceT, true);
                if (dbgPointSphere != null) dbgPointSphere.SetActive(true);
            }

            if (!treadVizOnly)
            {
                SetLine(lrCenterToP, center, pBest);
                SetLine(lrNormal, pBest, pBest + nBest * normalArrowLen);

                float nLen = Mathf.Clamp(Fn * forceToLength, arrowMinLen, arrowMaxLen);
                SetLine(lrForceN, pBest, pBest + nBest * nLen);

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

                if (dbgPointSphere != null) dbgPointSphere.transform.position = pBest;
            }

            if (doClip && debugDrawTreadRange)
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

        if (uiTreadClip != null) enableTreadWidthClip = uiTreadClip.IsActive;
        if (uiTreadW != null) treadWidth = uiTreadW.Value;

        if (uiTreadVizOnly != null) treadVizOnly = uiTreadVizOnly.IsActive;

        if (uiLineW != null)
        {
            float w = uiLineW.Value;
            forceLineWidth = w;
            thinLineWidth = w * 0.6f;
        }

        if (uiForceScale != null) forceToLength = uiForceScale.Value;
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
