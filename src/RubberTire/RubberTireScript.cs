using System.Collections.Generic;
using UnityEngine;
using Modding;

/// <summary>
/// RubberTireWheelScript
/// 0.1 目标：
/// - 单接触点（pen 最大）
/// - 法向弹簧阻尼 + 轮胎摩擦（kappa/alpha -> Fx/Fy -> 摩擦圆）
/// - Debug 可视化（点/线）
/// - 游戏内 UI 调参（K/C/mu/Cx/Cy/线宽/力缩放/Debug 开关）
/// </summary>
public class RubberTireWheelScript : BlockScript
{
    // =========================
    // 物理参数（会被 UI 覆盖）
    // =========================
    public float springK = 3000f;          // 法向弹簧刚度
    public float damperC = 25f;            // 法向阻尼
    public float maxNormalForce = 200000f; // 法向力上限

    public bool enableTireModel = true;    // 是否启用轮胎摩擦模型（真实模型）

    public float mu = 1.1f;                // 峰值摩擦系数
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

    // 轮轴方向（局部），你可以按模型实际改
    public enum AxisLocal { X, Y, Z }
    public AxisLocal wheelAxisLocal = AxisLocal.X;

    // 可选：测试用驱动扭矩（一般保持 0，使用 Spinning 模块驱动）
    public float driveTorque = 0f;

    // =========================
    // Debug 可视化（会被 UI 调）
    // =========================
    public bool debugDraw = true;

    public float thinLineWidth = 0.05f;    // 细线宽（轮心->接触点、法向）
    public float forceLineWidth = 0.10f;   // 力线宽（法向力、切向力）

    public float normalArrowLen = 0.6f;

    public float forceToLength = 0.0008f;  // 力->长度映射
    public float arrowMinLen = 0.25f;
    public float arrowMaxLen = 4.0f;

    public int drawEveryFixedSteps = 1;    // 每几帧画一次，调性能用

    // =========================
    // Debug：踏面裁切范围可视化
    // =========================
    public bool debugDrawTreadRange = true;  // 显示踏面宽度范围（两侧圆环 + 轮轴线）
    public int treadRangeSegments = 28;      // 圆环分段数
    public float treadAxisDebugLen = 2.0f;   // 轮轴线长度（乘以 R 再显示）

    // =========================
    // UI（Mapper）句柄
    // =========================
    private MSlider uiK, uiC, uiMu, uiCx, uiCy, uiLineW, uiForceScale;
    private MToggle uiDbg;
    private MToggle uiTreadClip;
    private MSlider uiTreadW;

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
    private GameObject dbgPointSphere;

    // ----------- 生命周期 -----------

    public override void SafeAwake()
    {
        // 用官方 Mapper UI 暴露参数（如果你已经在 XML 定义了同 key，可能会出现重复项；不想重复就删掉这段）
        //uiK = AddSlider("
        // k", "K (Spring)", springK, 0f, 20000f);
        //uiC = AddSlider("c", "C (Damper)", damperC, 0f, 200f);

        //uiMu = AddSlider("mu", "Mu", mu, 0f, 3f);
        //uiCx = AddSlider("cx", "Cx", Cx, 0f, 80000f);
        //uiCy = AddSlider("cy", "Cy", Cy, 0f, 80000f);

        //uiDbg = AddToggle("dbg", "Debug Draw", debugDraw);

        // 踏面裁切：把触发球/胶囊裁成“有限宽度”踏面
        //uiTreadClip = AddToggle("tw-clip", "Tread Width Clip", enableTreadWidthClip);
        //uiTreadW = AddSlider("tw", "Tread Width", treadWidth, 0.05f, 5f);

        //uiLineW = AddSlider("lw", "Line Width", forceLineWidth, 0.01f, 0.30f);
        //uiForceScale = AddSlider("fs", "Force Scale", forceToLength, 0.00001f, 0.01f);
    }

    public override void OnSimulateStart()
    {
        contacts.Clear();
        fixedStepCounter = 0;

        // 找到本 block 的踏面 trigger capsule，用它来定义轮心/半径（解决 XML offset）
        treadTriggerCapsule = FindTreadTriggerCapsule();

        // 如果需要画 debug，先创建对象
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

        // 踏面裁切参数（世界）
        Vector3 axisWorld = Vector3.right;
    float halfW = 0f;
    bool doClip = enableTreadWidthClip && treadWidth > 0f;

    if (doClip)
    {
        // 1) 轴方向：优先用触发胶囊自己的 direction（最可靠）
        if (treadTriggerCapsule != null)
        {
            Transform t = treadTriggerCapsule.transform;
            switch (treadTriggerCapsule.direction) // 0=X 1=Y 2=Z
            {
                case 0: axisWorld = t.right;   break;
                case 1: axisWorld = t.up;      break;
                case 2: axisWorld = t.forward; break;
            }

            // 2) 宽度缩放：也用胶囊自己的缩放（避免父子层级不一致）
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
            // fallback：没有胶囊时才用脚本 transform
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

            // 裁切：只有接触点落在踏面宽度范围内才参与计算
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

        // ===== 2) 轮胎摩擦模型（kappa/alpha -> Fx/Fy -> 摩擦圆）=====
        Vector3 Ftire = Vector3.zero;

        if (enableTireModel && Fn > 0f)
        {
            Rigidbody groundRb = (colBest != null) ? colBest.attachedRigidbody : null;

            Vector3 vGround = Vector3.zero;
            if (groundRb != null) vGround = groundRb.GetPointVelocity(pBest);

            Vector3 vWheel = Rigidbody.GetPointVelocity(pBest);
            Vector3 vRel = vWheel - vGround;

            // 轮轴（世界）
            Vector3 a = GetWheelAxisWorld().normalized;

            // 滚动方向 f：优先用“瞬时滚动方向”增加鲁棒性
            Vector3 omegaVec = Rigidbody.angularVelocity;
            Vector3 f0 = Vector3.Cross(omegaVec, -nBest);
            Vector3 f = ProjectOnPlane(f0, nBest);

            if (f.sqrMagnitude < 1e-6f)
            {
                // 回退：f = n × a
                f = ProjectOnPlane(Vector3.Cross(nBest, a), nBest);
            }
            if (f.sqrMagnitude < 1e-6f)
            {
                // 再回退：不计算
                f = Vector3.zero;
            }
            else f.Normalize();

            // 侧向方向 s：尽量与 f、n 正交
            Vector3 sdir = Vector3.Cross(nBest, f);
            if (sdir.sqrMagnitude > 1e-6f) sdir.Normalize();

            if (f == Vector3.zero || sdir == Vector3.zero)
            {
                // 几何退化时，不算轮胎力
                Ftire = Vector3.zero;
            }
            else
            {
                // 切平面相对速度
                Vector3 vT = ProjectOnPlane(vRel, nBest);

                float Vx = Vector3.Dot(vT, f);
                float Vy = Vector3.Dot(vT, sdir);

                // 轮周速度 Vw = omega_axis * R
                float omega = Vector3.Dot(omegaVec, a);
                float Vw = omega * R;

                // 速度尺度：用切向速度大小，避免 Vx≈0 导致 κ 坍塌
                float V = Mathf.Max(vT.magnitude, vEps);

                float kappa = (Vw - Vx) / V;
                float alpha = Mathf.Atan2(Vy, Mathf.Abs(Vx) + vEps);

                float Fx0 = Cx * kappa;
                float Fy0 = -Cy * alpha;

                // 摩擦圆
                float Fmax = mu * Fn;
                float Fmag0 = Mathf.Sqrt(Fx0 * Fx0 + Fy0 * Fy0);

                float Fx = Fx0, Fy = Fy0;
                if (Fmag0 > Fmax && Fmag0 > 1e-6f)
                {
                    float scale = Fmax / Fmag0;
                    Fx *= scale;
                    Fy *= scale;
                }

                // 低速衰减
                float w = V / (V + Mathf.Max(1e-4f, vBlend));
                Fx *= w;
                Fy *= w;

                Ftire = Fx * f + Fy * sdir;

                // 施加到轮子 + 反作用到地面刚体（如果有）
                Rigidbody.AddForceAtPosition(Ftire, pBest, ForceMode.Force);
                if (groundRb != null) groundRb.AddForceAtPosition(-Ftire, pBest, ForceMode.Force);
            }
        }

        // ===== Debug 可视化（先保证可见，颜色后续再精修）=====
        if (ShowDebugVisuals && debugDraw &&
            (fixedStepCounter % Mathf.Max(1, drawEveryFixedSteps) == 0))
        {
            EnsureDebugObjects();
            ShowDebugObjects();

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

            // 踏面宽度范围：两侧圆环 + 轮轴线（用于检查裁切方向/宽度是否正确）
            if (doClip && debugDrawTreadRange)
            {
                DrawTreadRangeDebug(center, axisWorld, halfW, R);
            }

            // 线宽可能被 UI 改了，需要实时同步
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
        // 注意：MSlider/MToggle 在不同版本 API 命名可能不同；
        // 你目前能编译说明 .Value / .IsActive 是可用的。
        if (uiK != null) springK = uiK.Value;
        if (uiC != null) damperC = uiC.Value;

        if (uiMu != null) mu = uiMu.Value;
        if (uiCx != null) Cx = uiCx.Value;
        if (uiCy != null) Cy = uiCy.Value;

        if (uiDbg != null) debugDraw = uiDbg.IsActive;

        if (uiTreadClip != null) enableTreadWidthClip = uiTreadClip.IsActive;
        if (uiTreadW != null) treadWidth = uiTreadW.Value;

        if (uiTreadClip != null) enableTreadWidthClip = uiTreadClip.IsActive;
        if (uiTreadW != null) treadWidth = uiTreadW.Value;

        if (uiLineW != null)
        {
            // 你拖动一个滑条同时控制“细线/力线”
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
            // fallback：AABB 最近点（不推荐）
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

    // 轮轴方向上的缩放（用于把 treadWidth 从“未缩放值”映射到世界单位）
    // 说明：半径 R 你用的是 MaxAbsComponent(lossyScale)，这里我们只取轴向分量更合理。
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

    // 踏面宽度裁切：把球形触发域裁成“有限宽度圆柱体”（只做宽度裁切，不做半径裁切）
    private bool IsWithinTreadWidth(Vector3 pointWorld, Vector3 centerWorld, Vector3 axisWorldUnit, float halfWWorld)
    {
        // axisWorldUnit 需要是单位向量
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

        // 轴线（黄色）
        float axisLen = Mathf.Max(radius * 0.5f, radius * treadAxisDebugLen);
        Debug.DrawLine(center - axisWorldUnit * axisLen, center + axisWorldUnit * axisLen, Color.yellow);

        // 计算圆环所在平面的基底 u/v（都与 axis 正交）
        Vector3 u = Vector3.Cross(axisWorldUnit, Vector3.up);
        if (u.sqrMagnitude < 1e-6f) u = Vector3.Cross(axisWorldUnit, Vector3.right);
        u.Normalize();
        Vector3 v = Vector3.Cross(axisWorldUnit, u).normalized;

        // 两侧边界圆环（青色）：center ± axis * halfW
        Vector3 c0 = center + axisWorldUnit * halfW;
        Vector3 c1 = center - axisWorldUnit * halfW;
        int seg = Mathf.Clamp(treadRangeSegments, 8, 128);
        DrawCircleDebug(c0, u, v, radius, seg, Color.cyan);
        DrawCircleDebug(c1, u, v, radius, seg, Color.cyan);
    }

    private void DrawCircleDebug(Vector3 center, Vector3 u, Vector3 v, float r, int segments, Color color)
    {
        float step = (Mathf.PI * 2f) / segments;
        Vector3 prev = center + u * r;
        for (int i = 1; i <= segments; i++)
        {
            float a = step * i;
            Vector3 p = center + (Mathf.Cos(a) * u + Mathf.Sin(a) * v) * r;
            Debug.DrawLine(prev, p, color);
            prev = p;
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

        // 尽力区分颜色（若你的环境仍吞色，至少线宽能看清）
        lrCenterToP = CreateLine("L_center_p", Color.white, thinLineWidth);
        lrNormal    = CreateLine("L_normal",   Color.green, thinLineWidth);
        lrForceN    = CreateLine("L_forceN",   Color.red,   forceLineWidth);
        lrForceT    = CreateLine("L_forceT",   Color.blue,  forceLineWidth);

        dbgPointSphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        dbgPointSphere.name = "P_contact";
        dbgPointSphere.transform.SetParent(dbgRoot.transform, false);
        dbgPointSphere.transform.localScale = Vector3.one * 0.10f;

        var c = dbgPointSphere.GetComponent<Collider>();
        if (c != null) c.enabled = false;

        var r = dbgPointSphere.GetComponent<Renderer>();
        if (r != null)
        {
            r.material = CreateColorMaterial(Color.yellow);
        }

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

    // ✅ 关键：Particles 系列多用 _TintColor
    if (mat.HasProperty("_TintColor")) mat.SetColor("_TintColor", color);
    if (mat.HasProperty("_Color"))     mat.SetColor("_Color", color);

    mat.color = color; // 兜底
    return mat;
}


    private LineRenderer CreateLine(string name, Color color, float width)
    {
        var go = new GameObject(name);
        go.transform.SetParent(dbgRoot.transform, false);

        var lr = go.AddComponent<LineRenderer>();
        lr.useWorldSpace = true;

        // 老 Unity 的 LineRenderer API
        lr.SetVertexCount(2);
        lr.SetWidth(width, width);

        // ✅ 关键：老版本用 SetColors
        lr.SetColors(color, color);

        // 材质仍然给（用于透明/发光等），但不再依赖 mat.color 作为唯一上色手段
        lr.material = CreateColorMaterial(color);

        lr.SetPosition(0, Vector3.zero);
        lr.SetPosition(1, Vector3.zero);
        return lr;
    }


    private void SetLine(LineRenderer lr, Vector3 a, Vector3 b)
    {
        if (lr == null) return;
        lr.SetPosition(0, a);
        lr.SetPosition(1, b);
    }

    private void MakeOrthoBasis(Vector3 axisUnit, out Vector3 u, out Vector3 v)
    {
        // 找一个不与 axis 平行的参考向量
        Vector3 refVec = (Mathf.Abs(axisUnit.y) < 0.9f) ? Vector3.up : Vector3.right;
        u = Vector3.Cross(axisUnit, refVec);
        float um = u.magnitude;
        if (um < 1e-6f)
        {
            refVec = Vector3.forward;
            u = Vector3.Cross(axisUnit, refVec);
            um = u.magnitude;
        }
        u /= Mathf.Max(1e-6f, um);
        v = Vector3.Cross(axisUnit, u);
        float vm = v.magnitude;
        v /= Mathf.Max(1e-6f, vm);
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
            dbgPointSphere = null;
        }
    }
}
