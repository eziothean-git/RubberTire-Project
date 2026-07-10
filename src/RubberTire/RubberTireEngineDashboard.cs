using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Besiege.UI;

internal sealed class RubberTireEngineDashboardRow
{
    public RubberTireWheelScript Target;
    public Text Identity;
    public Text Telemetry;
    public Text BrakeOnly;
    public RubberTireCurveGraphic Chart;
}

/// <summary>
/// Simulation-only distributed-drive monitor. F10 toggles a scrollable row for
/// every simulated tyre instance; clicking a row pins that wheel to the F9 lab.
/// </summary>
public sealed class RubberTireEngineDashboard : MonoBehaviour
{
    private GameObject root;
    private GameObject compactRoot;
    private Text compactText;
    private RectTransform content;
    private bool open;
    private bool readyRequested;
    private bool hadRoot;
    private float nextScanTime;
    private float nextTelemetryTime;
    private RubberTireFactoryUIController lab;

    private readonly List<RubberTireWheelScript> wheels = new List<RubberTireWheelScript>(16);
    private readonly List<RubberTireEngineDashboardRow> rows = new List<RubberTireEngineDashboardRow>(16);

    private static readonly Color Panel = new Color(0.045f, 0.055f, 0.065f, 0.97f);
    private static readonly Color Row = new Color(0.085f, 0.10f, 0.115f, 0.96f);
    private static readonly Color Accent = new Color(0.20f, 0.78f, 0.88f, 1f);
    private static readonly Color Muted = new Color(0.58f, 0.64f, 0.68f, 1f);

    private void Update()
    {
        if (lab == null) lab = GetComponent<RubberTireFactoryUIController>();
        if (root == null)
        {
            if (hadRoot)
            {
                hadRoot = false;
                readyRequested = false;
                rows.Clear();
                wheels.Clear();
                content = null;
                compactRoot = null;
                compactText = null;
            }
            TryBuildUI();
        }
        else hadRoot = true;

        if (Input.GetKeyDown(KeyCode.F10))
        {
            open = !open;
            nextScanTime = 0f;
        }
        if (root == null) return;

        if (Time.unscaledTime >= nextScanTime)
        {
            nextScanTime = Time.unscaledTime + 1.5f;
            ScanWheels();
        }

        bool hasWheels = wheels.Count > 0;
        bool fullVisible = open && hasWheels;
        bool compactVisible = !open && hasWheels;
        if (root.activeSelf != fullVisible) root.SetActive(fullVisible);
        if (compactRoot != null && compactRoot.activeSelf != compactVisible)
            compactRoot.SetActive(compactVisible);
        if (!hasWheels || Time.unscaledTime < nextTelemetryTime) return;
        nextTelemetryTime = Time.unscaledTime + 0.10f;
        RefreshTelemetry();
    }

    private void TryBuildUI()
    {
        if (Make.Instance == null) return;
        if (Make.ScreenCanvas != null)
        {
            BuildUI();
            return;
        }
        if (readyRequested) return;
        readyRequested = true;
        Make.OnReady("RubberTireDriveMonitor", BuildUI);
    }

    private void BuildUI()
    {
        if (root != null) return;
        root = Make.Prefab("UIFactory3", "Panel", Make.ScreenCanvas.transform);
        root.name = "Rubber Tire Distributed Drive Monitor";
        RectTransform rootRect = root.GetComponent<RectTransform>();
        rootRect.anchorMin = new Vector2(0f, 0.5f);
        rootRect.anchorMax = new Vector2(0f, 0.5f);
        rootRect.pivot = new Vector2(0f, 0.5f);
        rootRect.anchoredPosition = new Vector2(18f, 0f);
        rootRect.sizeDelta = new Vector2(640f, 640f);
        Image panelImage = root.GetComponent<Image>();
        if (panelImage != null) panelImage.color = Panel;

        Canvas canvas = root.GetComponent<Canvas>();
        if (canvas == null) canvas = root.AddComponent<Canvas>();
        canvas.overrideSorting = true;
        canvas.sortingOrder = 29990;
        if (root.GetComponent<GraphicRaycaster>() == null) root.AddComponent<GraphicRaycaster>();
        CanvasGroup group = root.GetComponent<CanvasGroup>();
        if (group == null) group = root.AddComponent<CanvasGroup>();
        group.interactable = true;
        group.blocksRaycasts = true;
        RubberTireUiCameraGuard guard = root.AddComponent<RubberTireUiCameraGuard>();
        guard.Controller = lab;

        CreateText(root.transform, "DISTRIBUTED DRIVE MONITOR", 21, FontStyle.Bold,
            new Vector2(18f, -12f), new Vector2(390f, 32f), TextAnchor.MiddleLeft, Color.white);
        CreateText(root.transform, "F10 toggle  |  click row -> F9 lab target", 12, FontStyle.Normal,
            new Vector2(344f, -14f), new Vector2(278f, 28f), TextAnchor.MiddleRight, Muted);

        RectTransform viewport = CreateRectObject("Drive Monitor Viewport", root.transform,
            new Vector2(18f, -54f), new Vector2(604f, 566f));
        Image viewportImage = viewport.gameObject.AddComponent<Image>();
        viewportImage.color = new Color(0.02f, 0.025f, 0.03f, 0.82f);
        Mask mask = viewport.gameObject.AddComponent<Mask>();
        mask.showMaskGraphic = true;

        content = CreateRectObject("Drive Monitor Content", viewport, Vector2.zero, new Vector2(586f, 0f));
        content.anchorMin = new Vector2(0f, 1f);
        content.anchorMax = new Vector2(0f, 1f);
        content.pivot = new Vector2(0f, 1f);
        content.anchoredPosition = new Vector2(6f, -6f);
        VerticalLayoutGroup layout = content.gameObject.AddComponent<VerticalLayoutGroup>();
        layout.spacing = 6f;
        layout.childAlignment = TextAnchor.UpperLeft;
        layout.childForceExpandHeight = false;
        layout.childForceExpandWidth = true;
        ContentSizeFitter fitter = content.gameObject.AddComponent<ContentSizeFitter>();
        fitter.verticalFit = ContentSizeFitter.FitMode.PreferredSize;

        ScrollRect scroll = viewport.gameObject.AddComponent<ScrollRect>();
        scroll.content = content;
        scroll.viewport = viewport;
        scroll.horizontal = false;
        scroll.vertical = true;
        scroll.movementType = ScrollRect.MovementType.Clamped;
        scroll.scrollSensitivity = 32f;

        compactRoot = Make.Prefab("UIFactory3", "Panel", Make.ScreenCanvas.transform);
        compactRoot.name = "Rubber Tire Compact Drive Monitor";
        RectTransform compactRect = compactRoot.GetComponent<RectTransform>();
        compactRect.anchorMin = new Vector2(0f, 1f);
        compactRect.anchorMax = new Vector2(0f, 1f);
        compactRect.pivot = new Vector2(0f, 1f);
        compactRect.anchoredPosition = new Vector2(18f, -108f);
        compactRect.sizeDelta = new Vector2(330f, 56f);
        Image compactImage = compactRoot.GetComponent<Image>();
        if (compactImage != null)
        {
            compactImage.color = new Color(0.035f, 0.045f, 0.055f, 0.90f);
            compactImage.raycastTarget = false;
        }
        Canvas compactCanvas = compactRoot.GetComponent<Canvas>();
        if (compactCanvas == null) compactCanvas = compactRoot.AddComponent<Canvas>();
        compactCanvas.overrideSorting = true;
        compactCanvas.sortingOrder = 29980;
        CanvasGroup compactGroup = compactRoot.GetComponent<CanvasGroup>();
        if (compactGroup == null) compactGroup = compactRoot.AddComponent<CanvasGroup>();
        compactGroup.interactable = false;
        compactGroup.blocksRaycasts = false;
        CreateText(compactRoot.transform, "DRIVE  |  F10 EXPAND", 12, FontStyle.Bold,
            new Vector2(10f, -4f), new Vector2(310f, 20f), TextAnchor.MiddleLeft, Accent);
        compactText = CreateText(compactRoot.transform, "", 13, FontStyle.Bold,
            new Vector2(10f, -25f), new Vector2(310f, 24f), TextAnchor.UpperLeft, Color.white);
        root.SetActive(false);
        compactRoot.SetActive(false);
    }

    private void ScanWheels()
    {
        List<RubberTireWheelScript> active = new List<RubberTireWheelScript>(
            RubberTireWheelScript.SimulatingInstances.Count);
        for (int i = RubberTireWheelScript.SimulatingInstances.Count - 1; i >= 0; i--)
        {
            RubberTireWheelScript wheel = RubberTireWheelScript.SimulatingInstances[i];
            if (wheel == null || !wheel.IsSimulating)
            {
                RubberTireWheelScript.SimulatingInstances.RemoveAt(i);
                continue;
            }
            active.Add(wheel);
        }
        active.Sort(delegate(RubberTireWheelScript a, RubberTireWheelScript b)
        {
            return a.GetInstanceID().CompareTo(b.GetInstanceID());
        });

        bool changed = active.Count != wheels.Count;
        if (!changed)
            for (int i = 0; i < active.Count; i++)
                if (active[i] != wheels[i]) { changed = true; break; }
        if (!changed) return;

        wheels.Clear();
        wheels.AddRange(active);
        RebuildRows();
    }

    private void RebuildRows()
    {
        if (content == null) return;
        for (int i = content.childCount - 1; i >= 0; i--)
            Destroy(content.GetChild(i).gameObject);
        rows.Clear();

        for (int i = 0; i < wheels.Count; i++)
        {
            RubberTireWheelScript wheel = wheels[i];
            RectTransform rect = CreateRectObject("Drive Wheel " + (i + 1), content,
                Vector2.zero, new Vector2(586f, 112f));
            LayoutElement element = rect.gameObject.AddComponent<LayoutElement>();
            element.preferredHeight = 112f;
            Image background = rect.gameObject.AddComponent<Image>();
            background.color = Row;
            Button select = rect.gameObject.AddComponent<Button>();
            select.targetGraphic = background;
            RubberTireWheelScript captured = wheel;
            select.onClick.AddListener(delegate
            {
                if (lab != null) lab.PinSimulationTarget(captured);
            });

            RubberTireEngineDashboardRow row = new RubberTireEngineDashboardRow();
            row.Target = wheel;
            row.Identity = CreateText(rect, "", 14, FontStyle.Bold,
                new Vector2(10f, -6f), new Vector2(168f, 24f), TextAnchor.MiddleLeft, Accent);
            row.Telemetry = CreateText(rect, "", 12, FontStyle.Normal,
                new Vector2(10f, -31f), new Vector2(168f, 70f), TextAnchor.UpperLeft, Color.white);
            row.BrakeOnly = CreateText(rect, "BRAKE ONLY", 16, FontStyle.Bold,
                new Vector2(190f, -38f), new Vector2(380f, 34f), TextAnchor.MiddleCenter, Muted);

            RectTransform chartRect = CreateRectObject("Mini Engine Curve", rect,
                new Vector2(190f, -8f), new Vector2(380f, 96f));
            row.Chart = chartRect.gameObject.AddComponent<RubberTireCurveGraphic>();
            row.Chart.Target = wheel;
            row.Chart.Kind = wheel.FactoryIsDrivenWheel() ? RubberTireChartKind.Engine : RubberTireChartKind.None;
            row.Chart.raycastTarget = false;
            rows.Add(row);
        }
        RefreshTelemetry();
    }

    private void RefreshTelemetry()
    {
        string compact = "";
        for (int i = 0; i < rows.Count; i++)
        {
            RubberTireEngineDashboardRow row = rows[i];
            RubberTireWheelScript wheel = row.Target;
            if (wheel == null) continue;
            bool driven = wheel.FactoryIsDrivenWheel();
            Vector3 position = wheel.transform.position;
            float driveTorque;
            float gripTorque;
            float driveGripRatio = wheel.FactoryDriveGripRatio(
                out driveTorque, out gripTorque);
            row.Identity.text = "W" + (i + 1).ToString("00")
                + (driven ? "  DRIVE" : "  FREE / BRAKE");
            row.Telemetry.text = driven
                ? "GEAR " + wheel.FactoryCurrentGearLabel()
                    + "   RPM " + Mathf.Max(0f, wheel.FactoryCurrentEngineRpm()).ToString("0")
                    + "\nTHR " + Mathf.RoundToInt(wheel.FactoryThrottle01() * 100f) + "%"
                    + "   BRK " + Mathf.RoundToInt(wheel.FactoryBrake01() * 100f) + "%"
                    + (wheel.FactoryLimiterCut() ? "  LIMIT" : "")
                    + "\nGRIP x" + driveGripRatio.ToString("0.0")
                    + "   axle/cap " + driveTorque.ToString("0")
                    + "/" + gripTorque.ToString("0")
                : "BRK " + Mathf.RoundToInt(wheel.FactoryBrake01() * 100f) + "%"
                    + "\nworld x/z " + position.x.ToString("0.0") + " / " + position.z.ToString("0.0")
                    + "\nno propulsion torque";
            row.BrakeOnly.gameObject.SetActive(!driven);
            row.Chart.Kind = driven ? RubberTireChartKind.Engine : RubberTireChartKind.None;
            row.Chart.SetVerticesDirty();

            if (compact.Length > 0) compact += "\n";
            compact += "W" + (i + 1).ToString("00") + "   ";
            if (driven)
            {
                compact += "[" + wheel.FactoryCurrentGearLabel() + "]   "
                    + Mathf.Max(0f, wheel.FactoryCurrentEngineRpm()).ToString("0").PadLeft(5)
                    + " RPM";
                if (wheel.FactoryLimiterCut()) compact += "  LIMIT";
            }
            else
            {
                compact += "[FREE]   BRK "
                    + Mathf.RoundToInt(wheel.FactoryBrake01() * 100f) + "%";
            }
        }
        if (compactText != null)
        {
            compactText.text = compact;
            float height = Mathf.Clamp(32f + rows.Count * 20f, 52f, 320f);
            RectTransform compactRect = compactRoot.GetComponent<RectTransform>();
            compactRect.sizeDelta = new Vector2(330f, height);
            compactText.rectTransform.sizeDelta = new Vector2(310f, height - 28f);
        }
    }

    private static RectTransform CreateRectObject(string name, Transform parent, Vector2 topLeft, Vector2 size)
    {
        GameObject go = new GameObject(name, typeof(RectTransform));
        go.layer = parent.gameObject.layer;
        RectTransform rect = go.GetComponent<RectTransform>();
        rect.SetParent(parent, false);
        SetTopLeftRect(rect, topLeft, size);
        return rect;
    }

    private static void SetTopLeftRect(RectTransform rect, Vector2 topLeft, Vector2 size)
    {
        rect.anchorMin = new Vector2(0f, 1f);
        rect.anchorMax = new Vector2(0f, 1f);
        rect.pivot = new Vector2(0f, 1f);
        rect.anchoredPosition = topLeft;
        rect.sizeDelta = size;
    }

    private static Text CreateText(Transform parent, string value, int fontSize, FontStyle style,
        Vector2 topLeft, Vector2 size, TextAnchor alignment, Color color)
    {
        GameObject go = new GameObject("T", typeof(RectTransform), typeof(CanvasRenderer), typeof(Text));
        go.layer = parent.gameObject.layer;
        RectTransform rect = go.GetComponent<RectTransform>();
        rect.SetParent(parent, false);
        SetTopLeftRect(rect, topLeft, size);
        Text text = go.GetComponent<Text>();
        text.font = Make.Font;
        text.fontSize = fontSize;
        text.fontStyle = style;
        text.alignment = alignment;
        text.color = color;
        text.text = value;
        text.raycastTarget = false;
        return text;
    }

    private void OnDestroy()
    {
        if (root != null) Destroy(root);
        if (compactRoot != null) Destroy(compactRoot);
    }
}
