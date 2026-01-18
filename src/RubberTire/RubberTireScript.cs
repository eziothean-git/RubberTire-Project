using System.Collections.Generic;
using UnityEngine;
using Modding;

/// <summary>
/// RubberTireWheelScript
/// - 单接触点（pen 最大）
/// - 法向弹簧阻尼 + 轮胎摩擦（kappa/alpha -> Fx/Fy -> 摩擦圆）
/// - Debug 可视化（点/线）
/// - 游戏内 UI 调参（K/C/mu/Cx/Cy/线宽/力缩放/Debug 开关/踏面裁切/仅踏面可视化）
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

    public float mu = 1.25f;               // 峰值摩擦系数
    public float Cx = 20000f;              // 纵向滑移刚度（N）
    public float Cy = 25000f;              // 侧偏刚度（N/rad）

    public float vEps = 0.2f;              // 速度防除零
    public float vBlend = 0.5f;            // 低速衰减：V/(V+vBlend)

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

    // 可选：测试用驱动扭矩（一般保持 0，使用 Spinning 模块驱动）
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
    private MSlider uiK, uiC, uiMu, uiCx, uiCy, uiLineW, uiForceScale, uiTreadW;
    private MToggle uiDbg, uiTreadClip, uiTreadVizOnly, uiEnableTire;

    // =========================
    // 运行时状态
    // =========================
    private readonly HashSet<Collider> contacts = new HashSet<Collider>();
    private CapsuleCollider treadTriggerCapsule;
    private int fixedStepCounter = 0;

    // Debug 对象
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
        // ✅ 关键修复：把 UI 句柄真正创建出来（否则 uiK/uiMu... 全是 null，UI 改了也不影响脚本）
        // 如果你选择用 XML <MapperTypes> 来定义脚本 UI，请把这一段注释掉，避免重复。
        uiK = AddSlider("K (Spring)", "k", springK, 0f, 20000f);
        uiC = AddSlider("C (Damper)", "c", damperC, 0f, 200f);

        uiMu = AddSlider("Mu", "mu", mu, 0f, 3f);
        uiCx = AddSlider("Cx", "cx", Cx, 0f, 80000f);
        uiCy = AddSlider("Cy", "cy", Cy, 0f, 80000f);

        uiEnableTire = AddToggle("Tire Model", "tire", enableTireModel);

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

        // 找到本 block 的踏面 trigger capsule，用它来定义轮心/半径
        treadTriggerCapsule = FindTreadTriggerCapsule();

        if (ShowDebugVisuals && debugDraw)
            EnsureDebugObjects();
    }

    public override void OnSimulateStop()
    {
        contacts.Clear();
        DestroyDebugObjects();
        treadTriggerCapsule = null;
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

        // 每个物理帧先同步 UI
        SyncParamsFromUI();

        if (contacts.Count == 0)
        {
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

        // 可选：施加驱动扭矩（一般为 0）
        if (driveTorque != 0f)
        {
            Vector3 aAxis = GetWheelAxisWorld().normalized;
            Rigidbody.AddTorque(aAxis * driveTorque, ForceMode.Force);
        }

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

            Vector3 omegaVec = Rigidbody.angularVelocity;
            Vector3 f0 = Vector3.Cross(omegaVec, -nBest);
            Vector3 f = ProjectOnPlane(f0, nBest);

            if (f.sqrMagnitude < 1e-6f)
                f = ProjectOnPlane(Vector3.Cross(nBest, a), nBest);

            if (f.sqrMagnitude < 1e-6f)
            {
                Ftire = Vector3.zero;
            }
            else
            {
                f.Normalize();

                Vector3 sdir = Vector3.Cross(nBest, f);
                if (sdir.sqrMagnitude > 1e-6f) sdir.Normalize();

                Vector3 vT = ProjectOnPlane(vRel, nBest);

                float Vx = Vector3.Dot(vT, f);
                float Vy = Vector3.Dot(vT, sdir);

                float omega = Vector3.Dot(omegaVec, a);
                float Vw = omega * R;

                float V = Mathf.Max(vT.magnitude, vEps);

                float kappa = (Vw - Vx) / V;
                float alpha = Mathf.Atan2(Vy, Mathf.Abs(Vx) + vEps);

                float Fx0 = Cx * kappa;
                float Fy0 = -Cy * alpha;

                float Fmax = mu * Fn;
                float Fmag0 = Mathf.Sqrt(Fx0 * Fx0 + Fy0 * Fy0);

                float Fx = Fx0, Fy = Fy0;
                if (Fmag0 > Fmax && Fmag0 > 1e-6f)
                {
                    float scale = Fmax / Fmag0;
                    Fx *= scale;
                    Fy *= scale;
                }

                float w = V / (V + Mathf.Max(1e-4f, vBlend));
                Fx *= w;
                Fy *= w;

                Ftire = Fx * f + Fy * sdir;

                Rigidbody.AddForceAtPosition(Ftire, pBest, ForceMode.Force);
                if (groundRb != null) groundRb.AddForceAtPosition(-Ftire, pBest, ForceMode.Force);
            }
        }

        // ===== Debug 可视化 =====
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

        if (uiMu != null) mu = uiMu.Value;
        if (uiCx != null) Cx = uiCx.Value;
        if (uiCy != null) Cy = uiCy.Value;

        if (uiEnableTire != null) enableTireModel = uiEnableTire.IsActive;

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
    // Debug：显示踏面裁切范围（两侧圆环 + 轮轴线）
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
    // Debug 可视化：对象创建/销毁
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
