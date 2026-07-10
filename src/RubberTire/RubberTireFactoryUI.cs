using System;
using System.Collections.Generic;
using System.Globalization;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.EventSystems;
using UnityEngine.UI;
using Besiege.UI;

public enum RubberTireChartKind
{
    Engine,
    Tire,
    Support,
    None
}

internal sealed class RubberTireFloatBinding
{
    public RubberTireFactorySetting Setting;
    public InputField Input;
}

internal sealed class RubberTireToggleBinding
{
    public RubberTireFactorySetting Setting;
    public Toggle Toggle;
}

public sealed class RubberTireFactoryUIController : MonoBehaviour
{
    private static readonly string[] Tabs =
        { "Engine", "Tire", "Support", "Contact", "Visual" };

    private GameObject root;
    private RectTransform settingsContent;
    private Text chartTitle;
    private Text chartLegend;
    private RubberTireCurveGraphic chart;
    private RubberTireWheelScript target;
    private string activeTab = "Engine";
    private bool readyRequested;
    private float nextRefreshTime;

    private readonly List<Button> tabButtons = new List<Button>(8);
    private readonly List<RubberTireFloatBinding> floatBindings =
        new List<RubberTireFloatBinding>(64);
    private readonly List<RubberTireToggleBinding> toggleBindings =
        new List<RubberTireToggleBinding>(32);

    private static readonly Color PanelColor = new Color(0.055f, 0.065f, 0.075f, 0.97f);
    private static readonly Color RowColor = new Color(0.10f, 0.115f, 0.13f, 0.92f);
    private static readonly Color AccentColor = new Color(0.20f, 0.78f, 0.88f, 1f);
    private static readonly Color MutedColor = new Color(0.58f, 0.64f, 0.68f, 1f);

    private void Update()
    {
        if (root == null)
        {
            TryBuildUI();
            return;
        }

        RubberTireWheelScript selected = FindSelectedWheel();
        if (selected != target)
        {
            target = selected;
            if (target != null)
            {
                target.FactoryPullSettings();
                RebuildSettings();
            }
        }

        bool visible = target != null;
        if (root.activeSelf != visible) root.SetActive(visible);
        if (!visible) return;

        target.FactoryPullSettings();
        chart.Target = target;
        chart.SetVerticesDirty();

        if (Time.unscaledTime >= nextRefreshTime)
        {
            nextRefreshTime = Time.unscaledTime + 0.20f;
            RefreshBindings();
        }
    }

    private void TryBuildUI()
    {
        if (readyRequested) return;
        if (Make.Instance == null || Make.ScreenCanvas == null) return;

        readyRequested = true;
        Make.OnReady("UIFactory3", BuildUI);
    }

    private void BuildUI()
    {
        root = Make.Prefab("UIFactory3", "Panel", Make.ScreenCanvas.transform);
        root.name = "Rubber Tire Lab";
        RectTransform rootRect = root.GetComponent<RectTransform>();
        rootRect.anchorMin = new Vector2(1f, 0.5f);
        rootRect.anchorMax = new Vector2(1f, 0.5f);
        rootRect.pivot = new Vector2(1f, 0.5f);
        rootRect.anchoredPosition = new Vector2(-18f, 0f);
        rootRect.sizeDelta = new Vector2(880f, 640f);
        Image rootImage = root.GetComponent<Image>();
        if (rootImage != null) rootImage.color = PanelColor;

        CreateText(root.transform, "RUBBER TIRE LAB", 22, FontStyle.Bold,
            new Vector2(18f, -12f), new Vector2(500f, 34f), TextAnchor.MiddleLeft, Color.white);
        CreateText(root.transform, "UIFactory workspace · one native machine-data record",
            12, FontStyle.Normal, new Vector2(450f, -14f), new Vector2(408f, 30f),
            TextAnchor.MiddleRight, MutedColor);

        for (int i = 0; i < Tabs.Length; i++)
        {
            string tab = Tabs[i];
            Button button = CreateButton(
                root.transform,
                tab,
                new Vector2(18f + i * 136f, -54f),
                new Vector2(128f, 34f));
            string capturedTab = tab;
            button.onClick.AddListener(delegate { SelectTab(capturedTab); });
            tabButtons.Add(button);
        }

        RectTransform viewport = CreateRectObject(
            "Settings Viewport",
            root.transform,
            new Vector2(18f, -102f),
            new Vector2(330f, 514f));
        Image viewportImage = viewport.gameObject.AddComponent<Image>();
        viewportImage.color = new Color(0.025f, 0.03f, 0.035f, 0.75f);
        Mask mask = viewport.gameObject.AddComponent<Mask>();
        mask.showMaskGraphic = true;

        settingsContent = CreateRectObject(
            "Settings Content",
            viewport,
            Vector2.zero,
            new Vector2(313f, 0f));
        settingsContent.anchorMin = new Vector2(0f, 1f);
        settingsContent.anchorMax = new Vector2(0f, 1f);
        settingsContent.pivot = new Vector2(0f, 1f);
        settingsContent.anchoredPosition = new Vector2(6f, -6f);

        VerticalLayoutGroup layout = settingsContent.gameObject.AddComponent<VerticalLayoutGroup>();
        layout.padding = new RectOffset(0, 0, 0, 0);
        layout.spacing = 5f;
        layout.childAlignment = TextAnchor.UpperLeft;
        layout.childForceExpandHeight = false;
        layout.childForceExpandWidth = true;
        ContentSizeFitter fitter = settingsContent.gameObject.AddComponent<ContentSizeFitter>();
        fitter.verticalFit = ContentSizeFitter.FitMode.PreferredSize;

        ScrollRect scroll = viewport.gameObject.AddComponent<ScrollRect>();
        scroll.content = settingsContent;
        scroll.viewport = viewport;
        scroll.horizontal = false;
        scroll.vertical = true;
        scroll.movementType = ScrollRect.MovementType.Clamped;
        scroll.scrollSensitivity = 28f;

        RectTransform chartPanel = CreateRectObject(
            "Chart Panel",
            root.transform,
            new Vector2(366f, -102f),
            new Vector2(496f, 514f));
        Image chartBackground = chartPanel.gameObject.AddComponent<Image>();
        chartBackground.color = new Color(0.022f, 0.027f, 0.032f, 0.92f);

        chartTitle = CreateText(chartPanel, "", 18, FontStyle.Bold,
            new Vector2(16f, -10f), new Vector2(464f, 28f), TextAnchor.MiddleLeft, Color.white);
        chartLegend = CreateText(chartPanel, "", 12, FontStyle.Normal,
            new Vector2(16f, -40f), new Vector2(464f, 40f), TextAnchor.UpperLeft, MutedColor);

        RectTransform chartRect = CreateRectObject(
            "Curve Graphic",
            chartPanel,
            new Vector2(18f, -88f),
            new Vector2(460f, 402f));
        chart = chartRect.gameObject.AddComponent<RubberTireCurveGraphic>();
        chart.raycastTarget = false;

        root.SetActive(false);
        SelectTab(activeTab);
    }

    private RubberTireWheelScript FindSelectedWheel()
    {
        if (!BlockMapper.IsOpen || BlockMapper.CurrentInstance == null) return null;
        if (BlockMapper.CurrentInstance.Block == null) return null;
        return BlockMapper.CurrentInstance.Block.GetComponent<RubberTireWheelScript>();
    }

    private void SelectTab(string tab)
    {
        activeTab = tab;
        for (int i = 0; i < tabButtons.Count; i++)
        {
            Image image = tabButtons[i].GetComponent<Image>();
            if (image != null)
                image.color = Tabs[i] == activeTab
                    ? new Color(0.12f, 0.42f, 0.48f, 1f)
                    : new Color(0.12f, 0.14f, 0.16f, 1f);
        }

        if (target != null) RebuildSettings();
        UpdateChartMode();
    }

    private void UpdateChartMode()
    {
        if (chart == null) return;
        if (activeTab == "Engine")
        {
            chart.Kind = RubberTireChartKind.Engine;
            chartTitle.text = "ENGINE TORQUE / POWER";
            chartLegend.text = "CYAN  torque     ORANGE  power     WHITE  live RPM";
        }
        else if (activeTab == "Tire")
        {
            chart.Kind = RubberTireChartKind.Tire;
            chartTitle.text = "COMBINED FRICTION ELLIPSE";
            chartLegend.text = "CYAN  static limit     ORANGE  kinetic limit     WHITE  live Fx/Fn, Fy/Fn\nHorizontal: longitudinal force. Vertical: lateral force. Both use one common scale.";
        }
        else if (activeTab == "Support")
        {
            chart.Kind = RubberTireChartKind.Support;
            chartTitle.text = "SUPPORT FORCE / PENETRATION";
            chartLegend.text = "CYAN  spring branch at zero normal velocity\nRebound damping and the mass-aware recovery cap act dynamically.";
        }
        else
        {
            chart.Kind = RubberTireChartKind.None;
            chartTitle.text = activeTab == "Contact" ? "CONTACT PIPELINE" : "DEBUG VISUALS";
            chartLegend.text = activeTab == "Contact"
                ? "All-layer ray query → per-collider hit aggregation → point support → body patch friction"
                : "These controls affect diagnostics only; physical parameters live in the other pages.";
        }
        chart.SetVerticesDirty();
    }

    private void RebuildSettings()
    {
        if (settingsContent == null || target == null) return;

        for (int i = settingsContent.childCount - 1; i >= 0; i--)
            Destroy(settingsContent.GetChild(i).gameObject);
        floatBindings.Clear();
        toggleBindings.Clear();

        List<RubberTireFactorySetting> settings = target.BuildFactorySettings();
        for (int i = 0; i < settings.Count; i++)
        {
            RubberTireFactorySetting setting = settings[i];
            if (setting.Tab != activeTab) continue;
            if (setting.IsToggle) CreateToggleRow(setting);
            else CreateFloatRow(setting);
        }

        UpdateChartMode();
        Canvas.ForceUpdateCanvases();
    }

    private void CreateFloatRow(RubberTireFactorySetting setting)
    {
        RectTransform row = CreateRectObject(
            setting.Label,
            settingsContent,
            Vector2.zero,
            new Vector2(313f, 38f));
        LayoutElement element = row.gameObject.AddComponent<LayoutElement>();
        element.preferredHeight = 38f;
        Image bg = row.gameObject.AddComponent<Image>();
        bg.color = RowColor;

        CreateText(row, setting.Label, 12, FontStyle.Normal,
            new Vector2(10f, -1f), new Vector2(205f, 36f), TextAnchor.MiddleLeft, Color.white);

        GameObject inputObject = Make.Prefab("UIFactory3", "Input Field", row);
        RectTransform inputRect = inputObject.GetComponent<RectTransform>();
        SetTopLeftRect(inputRect, new Vector2(220f, -5f), new Vector2(83f, 28f));
        InputField input = inputObject.GetComponent<InputField>();
        input.contentType = InputField.ContentType.DecimalNumber;
        input.text = FormatValue(setting.GetFloat());

        RubberTireFactorySetting captured = setting;
        input.onEndEdit.AddListener(delegate(string text)
        {
            float value;
            if (TryParseValue(text, out value))
                captured.SetFloat(Mathf.Clamp(value, captured.Min, captured.Max));
            input.text = FormatValue(captured.GetFloat());
            OnSettingChanged();
        });

        RubberTireFloatBinding binding = new RubberTireFloatBinding();
        binding.Setting = setting;
        binding.Input = input;
        floatBindings.Add(binding);
    }

    private void CreateToggleRow(RubberTireFactorySetting setting)
    {
        GameObject toggleObject = Make.Prefab("UIFactory3", "Text Toggle", settingsContent);
        toggleObject.name = setting.Label;
        RectTransform rect = toggleObject.GetComponent<RectTransform>();
        rect.sizeDelta = new Vector2(313f, 38f);
        LayoutElement element = toggleObject.GetComponent<LayoutElement>();
        if (element == null) element = toggleObject.AddComponent<LayoutElement>();
        element.preferredHeight = 38f;
        Image bg = toggleObject.GetComponent<Image>();
        if (bg != null) bg.color = RowColor;

        Text label = toggleObject.GetComponentInChildren<Text>();
        if (label != null) label.text = setting.Label;
        Toggle toggle = toggleObject.GetComponent<Toggle>();
        toggle.isOn = setting.GetBool();

        RubberTireFactorySetting captured = setting;
        toggle.onValueChanged.AddListener(delegate(bool value)
        {
            captured.SetBool(value);
            OnSettingChanged();
        });

        RubberTireToggleBinding binding = new RubberTireToggleBinding();
        binding.Setting = setting;
        binding.Toggle = toggle;
        toggleBindings.Add(binding);
    }

    private void OnSettingChanged()
    {
        if (target != null) target.FactoryCommitSettings();
        if (chart != null) chart.SetVerticesDirty();
    }

    private void RefreshBindings()
    {
        for (int i = 0; i < floatBindings.Count; i++)
        {
            RubberTireFloatBinding binding = floatBindings[i];
            float value = binding.Setting.GetFloat();
            if (!InputHasFocus(binding.Input))
                binding.Input.text = FormatValue(value);
        }

        for (int i = 0; i < toggleBindings.Count; i++)
        {
            RubberTireToggleBinding binding = toggleBindings[i];
            bool value = binding.Setting.GetBool();
            if (binding.Toggle.isOn != value) binding.Toggle.isOn = value;
        }
    }

    private static bool InputHasFocus(InputField input)
    {
        return input != null
            && EventSystem.current != null
            && EventSystem.current.currentSelectedGameObject == input.gameObject;
    }

    private static bool TryParseValue(string text, out float value)
    {
        if (float.TryParse(text, NumberStyles.Float, CultureInfo.InvariantCulture, out value))
            return true;
        return float.TryParse(text, out value);
    }

    private static string FormatValue(float value)
    {
        float magnitude = Mathf.Abs(value);
        string format = magnitude >= 10000f ? "0"
            : magnitude >= 100f ? "0.0"
            : magnitude >= 1f ? "0.###"
            : "0.#####";
        return value.ToString(format, CultureInfo.InvariantCulture);
    }

    private static RectTransform CreateRectObject(
        string name,
        Transform parent,
        Vector2 topLeft,
        Vector2 size)
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

    private static Text CreateText(
        Transform parent,
        string value,
        int fontSize,
        FontStyle style,
        Vector2 topLeft,
        Vector2 size,
        TextAnchor alignment,
        Color color)
    {
        GameObject go = new GameObject(value, typeof(RectTransform), typeof(CanvasRenderer), typeof(Text));
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

    private static Button CreateButton(
        Transform parent,
        string label,
        Vector2 topLeft,
        Vector2 size)
    {
        GameObject buttonObject = Make.Prefab("UIFactory3", "Text Button", parent);
        buttonObject.name = label;
        SetTopLeftRect(buttonObject.GetComponent<RectTransform>(), topLeft, size);
        Text text = buttonObject.GetComponentInChildren<Text>();
        if (text != null) text.text = label;
        return buttonObject.GetComponent<Button>();
    }

    private void OnDestroy()
    {
        if (root != null) Destroy(root);
    }
}

public sealed class RubberTireCurveGraphic : MaskableGraphic
{
    public RubberTireWheelScript Target;
    public RubberTireChartKind Kind;

    private static readonly Color GridColor = new Color(0.28f, 0.32f, 0.35f, 0.36f);
    private static readonly Color Cyan = new Color(0.20f, 0.84f, 0.94f, 1f);
    private static readonly Color Orange = new Color(1f, 0.58f, 0.18f, 1f);
    private static readonly Color White = new Color(0.94f, 0.96f, 0.98f, 0.9f);

    protected override void OnPopulateMesh(VertexHelper vh)
    {
        vh.Clear();
        Rect r = rectTransform.rect;
        if (r.width <= 1f || r.height <= 1f) return;

        if (Target == null || Kind == RubberTireChartKind.None) return;

        if (Kind == RubberTireChartKind.Tire) DrawTire(vh, r);
        else
        {
            DrawGrid(vh, r);
            if (Kind == RubberTireChartKind.Engine) DrawEngine(vh, r);
            else if (Kind == RubberTireChartKind.Support) DrawSupport(vh, r);
        }
    }

    private void DrawGrid(VertexHelper vh, Rect r)
    {
        for (int i = 0; i <= 5; i++)
        {
            float u = i / 5f;
            AddLine(vh,
                new Vector2(Mathf.Lerp(r.xMin, r.xMax, u), r.yMin),
                new Vector2(Mathf.Lerp(r.xMin, r.xMax, u), r.yMax),
                1f, GridColor);
            AddLine(vh,
                new Vector2(r.xMin, Mathf.Lerp(r.yMin, r.yMax, u)),
                new Vector2(r.xMax, Mathf.Lerp(r.yMin, r.yMax, u)),
                1f, GridColor);
        }
        AddLine(vh, new Vector2(r.xMin, r.yMin), new Vector2(r.xMax, r.yMin), 2f, White);
        AddLine(vh, new Vector2(r.xMin, r.yMin), new Vector2(r.xMin, r.yMax), 2f, White);
    }

    private void DrawEngine(VertexHelper vh, Rect r)
    {
        float maxRpm = Mathf.Max(1f, Target.FactoryEngineRedlineRpm());
        const int samples = 96;
        float maxTorque = 1e-4f;
        float maxPower = 1e-4f;
        for (int i = 0; i <= samples; i++)
        {
            float rpm = maxRpm * i / samples;
            float torque, power;
            Target.FactoryEnginePoint(rpm, out torque, out power);
            maxTorque = Mathf.Max(maxTorque, torque);
            maxPower = Mathf.Max(maxPower, power);
        }

        Vector2 previousTorque = Vector2.zero;
        Vector2 previousPower = Vector2.zero;
        for (int i = 0; i <= samples; i++)
        {
            float u = i / (float)samples;
            float rpm = maxRpm * u;
            float torque, power;
            Target.FactoryEnginePoint(rpm, out torque, out power);
            Vector2 torquePoint = Plot(r, u, torque / maxTorque);
            Vector2 powerPoint = Plot(r, u, power / maxPower);
            if (i > 0)
            {
                AddLine(vh, previousTorque, torquePoint, 3f, Cyan);
                AddLine(vh, previousPower, powerPoint, 3f, Orange);
            }
            previousTorque = torquePoint;
            previousPower = powerPoint;
        }

        float liveRpm = Target.FactoryCurrentEngineRpm();
        if (liveRpm >= 0f)
        {
            float u = Mathf.Clamp01(liveRpm / maxRpm);
            AddLine(vh, Plot(r, u, 0f), Plot(r, u, 1f), 2f, White);
        }
    }

    private void DrawTire(VertexHelper vh, Rect r)
    {
        float staticLong, staticLat, kineticLong, kineticLat;
        Target.FactoryFrictionEllipse(false, out staticLong, out staticLat);
        Target.FactoryFrictionEllipse(true, out kineticLong, out kineticLat);
        float scale = Mathf.Max(0.25f,
            Mathf.Max(Mathf.Max(staticLong, staticLat), Mathf.Max(kineticLong, kineticLat))) * 1.12f;

        Vector2 center = r.center;
        float halfSize = Mathf.Min(r.width, r.height) * 0.46f;
        AddLine(vh, center + Vector2.left * halfSize, center + Vector2.right * halfSize, 1.5f, GridColor);
        AddLine(vh, center + Vector2.down * halfSize, center + Vector2.up * halfSize, 1.5f, GridColor);
        DrawEllipse(vh, center, halfSize * 0.5f, halfSize * 0.5f, 1f, GridColor);
        DrawEllipse(vh, center, halfSize, halfSize, 1f, GridColor);

        DrawEllipse(vh, center,
            halfSize * staticLong / scale,
            halfSize * staticLat / scale,
            3f, Cyan);
        DrawEllipse(vh, center,
            halfSize * kineticLong / scale,
            halfSize * kineticLat / scale,
            3f, Orange);

        float liveLong, liveLat;
        Target.FactoryCurrentFrictionPoint(out liveLong, out liveLat);
        Vector2 live = center + new Vector2(liveLong, liveLat) * (halfSize / scale);
        Vector2 delta = live - center;
        if (delta.magnitude > halfSize) live = center + delta.normalized * halfSize;
        AddLine(vh, center, live, 2f, White);
        AddLine(vh, live + new Vector2(-5f, 0f), live + new Vector2(5f, 0f), 2f, White);
        AddLine(vh, live + new Vector2(0f, -5f), live + new Vector2(0f, 5f), 2f, White);
    }

    private static void DrawEllipse(
        VertexHelper vh,
        Vector2 center,
        float radiusX,
        float radiusY,
        float width,
        Color color)
    {
        if (radiusX <= 1e-4f || radiusY <= 1e-4f) return;
        const int samples = 96;
        Vector2 previous = center + new Vector2(radiusX, 0f);
        for (int i = 1; i <= samples; i++)
        {
            float angle = Mathf.PI * 2f * i / samples;
            Vector2 point = center + new Vector2(
                Mathf.Cos(angle) * radiusX,
                Mathf.Sin(angle) * radiusY);
            AddLine(vh, previous, point, width, color);
            previous = point;
        }
    }

    private void DrawSupport(VertexHelper vh, Rect r)
    {
        const float maxPenetration = 0.25f;
        const int samples = 80;
        float maxForce = Mathf.Max(1f, Target.FactorySupportForce(maxPenetration));
        Vector2 previous = Vector2.zero;
        for (int i = 0; i <= samples; i++)
        {
            float u = i / (float)samples;
            float force = Target.FactorySupportForce(maxPenetration * u);
            Vector2 point = Plot(r, u, force / maxForce);
            if (i > 0) AddLine(vh, previous, point, 3f, Cyan);
            previous = point;
        }
    }

    private static Vector2 Plot(Rect r, float x, float y)
    {
        return new Vector2(
            Mathf.Lerp(r.xMin, r.xMax, Mathf.Clamp01(x)),
            Mathf.Lerp(r.yMin, r.yMax, Mathf.Clamp01(y)));
    }

    private static void AddLine(
        VertexHelper vh,
        Vector2 a,
        Vector2 b,
        float width,
        Color color)
    {
        Vector2 direction = b - a;
        if (direction.sqrMagnitude <= 1e-8f) return;
        direction.Normalize();
        Vector2 normal = new Vector2(-direction.y, direction.x) * (width * 0.5f);

        int index = vh.currentVertCount;
        AddVertex(vh, a - normal, color);
        AddVertex(vh, a + normal, color);
        AddVertex(vh, b + normal, color);
        AddVertex(vh, b - normal, color);
        vh.AddTriangle(index, index + 1, index + 2);
        vh.AddTriangle(index, index + 2, index + 3);
    }

    private static void AddVertex(VertexHelper vh, Vector2 position, Color color)
    {
        UIVertex vertex = UIVertex.simpleVert;
        vertex.position = position;
        vertex.color = color;
        vh.AddVert(vertex);
    }
}
