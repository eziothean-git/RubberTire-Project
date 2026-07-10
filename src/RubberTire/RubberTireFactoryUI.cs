// The base game defines a global-namespace Slider type that shadows
// UnityEngine.UI.Slider; alias explicitly.
using UISlider = UnityEngine.UI.Slider;
using System;
using System.Collections.Generic;
using System.Globalization;
using UnityEngine;
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

internal sealed class RubberTireSettingRow
{
    public RubberTireFactorySetting Setting;
    public GameObject Root;
    public InputField Input;
    public UISlider Slider;
    public Toggle Toggle;
    public float LastShown = float.NaN;
    public bool Suppress;
    public string GroupKey;
}

internal sealed class RubberTireGroupHeader
{
    public string Group;
    public string GroupKey;
    public GameObject Root;
    public Text Label;
    public bool AnyVisible;
}

// Hover helper implementing ONLY enter/exit: a full EventTrigger would also
// swallow scroll/drag events and break the settings ScrollRect.
internal sealed class RubberTireRowHover : MonoBehaviour, IPointerEnterHandler, IPointerExitHandler
{
    internal RubberTireFactoryUIController Controller;
    internal RubberTireSettingRow Row;

    public void OnPointerEnter(PointerEventData eventData)
    {
        if (Controller != null && Row != null) Controller.ShowTooltip(Row.Setting);
    }

    public void OnPointerExit(PointerEventData eventData)
    {
        if (Controller != null) Controller.ShowTooltip(null);
    }
}

// The game cameras poll Input.GetAxis("Mouse ScrollWheel") directly, so merely
// consuming an EventSystem scroll event cannot stop world zoom.  Tell the
// controller when the pointer is anywhere over the lab and temporarily mute
// only the cameras' zoom sensitivity.
internal sealed class RubberTireUiCameraGuard : MonoBehaviour,
    IPointerEnterHandler, IPointerExitHandler, IScrollHandler
{
    internal RubberTireFactoryUIController Controller;

    public void OnPointerEnter(PointerEventData eventData)
    {
        if (Controller != null) Controller.SetPointerOverLab(true);
    }

    public void OnPointerExit(PointerEventData eventData)
    {
        if (Controller != null) Controller.SetPointerOverLab(false);
    }

    public void OnScroll(PointerEventData eventData)
    {
        // Child ScrollRects still receive and process their own scroll event.
        // Use() prevents unrelated event-driven consumers from seeing it.
        if (eventData != null) eventData.Use();
    }

    private void OnDisable()
    {
        if (Controller != null) Controller.SetPointerOverLab(false);
    }
}

public sealed class RubberTireFactoryUIController : MonoBehaviour
{
    private static readonly string[] Tabs =
        { "Engine", "Tire", "Support", "Contact", "Visual" };

    private const string DefaultTooltip =
        "Hover a row for details. F9 opens this panel during simulation (edits there tune live physics but are not saved to the machine).";

    private GameObject root;
    private RectTransform settingsContent;
    private ScrollRect settingsScroll;
    private Text chartTitle;
    private Text chartLegend;
    private Text chartLive;
    private Text contactLive;
    private Text chartAxisLeft;
    private Text chartAxisRight;
    private Text chartAxisX;
    private Text tooltipLabel;
    private Text advancedButtonText;
    private Text groupButtonText;
    private Button engineLutToggleButton;
    private Button engineAddPointButton;
    private Button engineDeletePointButton;
    private Text engineLutToggleText;
    private GameObject engineLutEditorRoot;
    private InputField engineLutInput;
    private Text engineLutStatus;
    private bool engineLutEditorVisible;
    private string focusedGroupKey;
    private bool pointerOverLab;
    private OrbitCamera[] guardedOrbitCameras;
    private float[] guardedOrbitZoomSpeeds;
    private MouseOrbit[] guardedMouseOrbits;
    private float[] guardedMouseOrbitScrollSpeeds;
    private ScrollToZoom[] guardedScrollZooms;
    private float[] guardedScrollZoomSpeeds;
    private RubberTireCurveGraphic chart;
    private RubberTireWheelScript target;
    private RubberTireWheelScript simTarget;
    private string activeTab = "Engine";
    private bool readyRequested;
    private bool hadRoot;
    private bool simPanelOpen;
    private bool commitPending;
    private float nextRefreshTime;
    private float nextChartDirtyTime;
    private float nextTargetScanTime;

    // Session-wide UI preferences.
    private static bool showAdvanced;
    private static readonly HashSet<string> collapsedGroups = new HashSet<string>();

    private readonly List<Button> tabButtons = new List<Button>(8);
    private readonly List<RubberTireSettingRow> rows = new List<RubberTireSettingRow>(96);
    private readonly List<RubberTireGroupHeader> headers = new List<RubberTireGroupHeader>(24);
    private readonly Dictionary<string, RubberTireGroupHeader> headersByKey =
        new Dictionary<string, RubberTireGroupHeader>(24, StringComparer.Ordinal);

    private static readonly Color PanelColor = new Color(0.055f, 0.065f, 0.075f, 0.97f);
    private static readonly Color RowColor = new Color(0.10f, 0.115f, 0.13f, 0.92f);
    private static readonly Color AccentColor = new Color(0.20f, 0.78f, 0.88f, 1f);
    private static readonly Color MutedColor = new Color(0.58f, 0.64f, 0.68f, 1f);

    private void Update()
    {
        if (root == null)
        {
            if (hadRoot)
            {
                // E7: the UIFactory canvas (and our panel with it) died on a
                // scene change. Drop stale references and allow a rebuild.
                hadRoot = false;
                readyRequested = false;
                ClearUiReferences();
            }
            TryBuildUI();
            return;
        }
        hadRoot = true;

        if (Input.GetKeyDown(KeyCode.F9))
        {
            simPanelOpen = !simPanelOpen;
            nextTargetScanTime = 0f;
        }

        RubberTireWheelScript selected = ResolveTarget();
        if (selected != target)
        {
            if (target != null && commitPending)
            {
                commitPending = false;
                target.FactoryCommitSettings();
            }
            target = selected;
            if (target != null)
            {
                target.FactoryPullSettings();
                RebindRows();
                chart.Target = target;
                chart.SelectEnginePoint(-1);
                UpdateChartMode();
                RefreshBindings();
                ApplyRowVisibility();
                UpdateChartTexts();
                if (engineLutEditorVisible && engineLutInput != null)
                    engineLutInput.text = target.FactoryGetEngineTorqueLut();
            }
        }

        bool visible = target != null;
        if (root.activeSelf != visible) root.SetActive(visible);
        if (!visible) return;

        if (Time.unscaledTime >= nextRefreshTime)
        {
            nextRefreshTime = Time.unscaledTime + 0.20f;
            target.FactoryPullSettings();
            if (commitPending)
            {
                commitPending = false;
                target.FactoryCommitSettings();
            }
            RefreshBindings();
            ApplyRowVisibility();
            UpdateChartTexts();
        }

        // E8: the chart mesh is rebuilt only when parameters change or, during
        // simulation, on a 10 Hz tick for the live work point.
        if (target.IsSimulating
            && chart.Kind != RubberTireChartKind.None
            && Time.unscaledTime >= nextChartDirtyTime)
        {
            nextChartDirtyTime = Time.unscaledTime + 0.10f;
            chart.SetVerticesDirty();
        }
    }

    private void TryBuildUI()
    {
        if (Make.Instance == null) return;

        // E7: when the canvas already exists OnReady may have fired long ago
        // and re-registering would never call back. Build directly.
        if (Make.ScreenCanvas != null)
        {
            BuildUI();
            return;
        }

        if (readyRequested) return;
        readyRequested = true;
        Make.OnReady("UIFactory3", BuildUI);
    }

    private void ClearUiReferences()
    {
        target = null;
        simTarget = null;
        chart = null;
        settingsContent = null;
        settingsScroll = null;
        chartTitle = null;
        chartLegend = null;
        chartLive = null;
        contactLive = null;
        chartAxisLeft = null;
        chartAxisRight = null;
        chartAxisX = null;
        tooltipLabel = null;
        advancedButtonText = null;
        groupButtonText = null;
        engineLutToggleButton = null;
        engineAddPointButton = null;
        engineDeletePointButton = null;
        engineLutToggleText = null;
        engineLutEditorRoot = null;
        engineLutInput = null;
        engineLutStatus = null;
        engineLutEditorVisible = false;
        focusedGroupKey = null;
        RestoreCameraZoom();
        tabButtons.Clear();
        rows.Clear();
        headers.Clear();
        headersByKey.Clear();
    }

    private void BuildUI()
    {
        if (root != null) return;

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

        // Block Mapper owns a full-screen modal raycast layer. Give the lab its
        // own top-level canvas/raycaster so controls rendered above that layer
        // also receive pointer input above it.
        Canvas labCanvas = root.GetComponent<Canvas>();
        if (labCanvas == null) labCanvas = root.AddComponent<Canvas>();
        labCanvas.overrideSorting = true;
        labCanvas.sortingOrder = 30000;
        if (root.GetComponent<GraphicRaycaster>() == null)
            root.AddComponent<GraphicRaycaster>();
        CanvasGroup labGroup = root.GetComponent<CanvasGroup>();
        if (labGroup == null) labGroup = root.AddComponent<CanvasGroup>();
        labGroup.interactable = true;
        labGroup.blocksRaycasts = true;
        RubberTireUiCameraGuard cameraGuard = root.AddComponent<RubberTireUiCameraGuard>();
        cameraGuard.Controller = this;
        root.transform.SetAsLastSibling();

        CreateText(root.transform, "RUBBER TIRE LAB", 22, FontStyle.Bold,
            new Vector2(18f, -12f), new Vector2(500f, 34f), TextAnchor.MiddleLeft, Color.white);
        CreateText(root.transform, "UIFactory workspace  |  F9 = live panel in simulation",
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
            new Vector2(330f, 408f));
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

        settingsScroll = viewport.gameObject.AddComponent<ScrollRect>();
        settingsScroll.content = settingsContent;
        settingsScroll.viewport = viewport;
        settingsScroll.horizontal = false;
        settingsScroll.vertical = true;
        settingsScroll.movementType = ScrollRect.MovementType.Clamped;
        settingsScroll.scrollSensitivity = 28f;

        Button groupButton = CreateButton(root.transform, "Group: All",
            new Vector2(18f, -518f), new Vector2(330f, 26f));
        groupButtonText = groupButton.GetComponentInChildren<Text>();
        groupButton.onClick.AddListener(CycleFocusedGroup);

        // E3/E4: workspace actions under the settings list.
        Button resetButton = CreateButton(root.transform, "Reset Tab",
            new Vector2(18f, -550f), new Vector2(102f, 26f));
        resetButton.onClick.AddListener(delegate { ResetActiveTab(); });

        Button applyAllButton = CreateButton(root.transform, "Apply to All",
            new Vector2(126f, -550f), new Vector2(110f, 26f));
        applyAllButton.onClick.AddListener(delegate { ApplyToAllWheels(); });

        Button advancedButton = CreateButton(root.transform, "Adv: Off",
            new Vector2(242f, -550f), new Vector2(106f, 26f));
        advancedButtonText = advancedButton.GetComponentInChildren<Text>();
        advancedButton.onClick.AddListener(delegate
        {
            showAdvanced = !showAdvanced;
            UpdateAdvancedButton();
            ApplyRowVisibility();
        });
        UpdateAdvancedButton();

        tooltipLabel = CreateText(root.transform, DefaultTooltip, 11, FontStyle.Normal,
            new Vector2(18f, -582f), new Vector2(330f, 50f), TextAnchor.UpperLeft, MutedColor);

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
            new Vector2(16f, -40f), new Vector2(464f, 32f), TextAnchor.UpperLeft, MutedColor);

        RectTransform chartRect = CreateRectObject(
            "Curve Graphic",
            chartPanel,
            new Vector2(18f, -84f),
            new Vector2(460f, 386f));
        chart = chartRect.gameObject.AddComponent<RubberTireCurveGraphic>();
        chart.raycastTarget = false;
        chart.OnEngineCurveEdited = OnEngineCurveDragged;

        // E6: numeric context for the mesh-only chart.
        chartAxisLeft = CreateText(chartPanel, "", 11, FontStyle.Normal,
            new Vector2(18f, -72f), new Vector2(230f, 16f), TextAnchor.MiddleLeft, AccentColor);
        chartAxisRight = CreateText(chartPanel, "", 11, FontStyle.Normal,
            new Vector2(248f, -72f), new Vector2(230f, 16f), TextAnchor.MiddleRight,
            new Color(1f, 0.58f, 0.18f, 1f));
        chartAxisX = CreateText(chartPanel, "", 11, FontStyle.Normal,
            new Vector2(18f, -470f), new Vector2(460f, 16f), TextAnchor.MiddleRight, MutedColor);

        chartLive = CreateText(chartPanel, "", 13, FontStyle.Bold,
            new Vector2(16f, -488f), new Vector2(464f, 22f), TextAnchor.MiddleLeft, Color.white);

        contactLive = CreateText(chartPanel, "", 12, FontStyle.Normal,
            new Vector2(18f, -96f), new Vector2(440f, 360f), TextAnchor.UpperLeft, Color.white);

        engineLutToggleButton = CreateButton(chartPanel, "Edit RPM|Nm",
            new Vector2(368f, -8f), new Vector2(110f, 28f));
        engineLutToggleText = engineLutToggleButton.GetComponentInChildren<Text>();
        engineLutToggleButton.onClick.AddListener(delegate
        {
            SetEngineLutEditorVisible(!engineLutEditorVisible);
        });
        engineAddPointButton = CreateButton(chartPanel, "+ Point",
            new Vector2(244f, -8f), new Vector2(58f, 28f));
        engineAddPointButton.onClick.AddListener(AddEngineLutPoint);
        engineDeletePointButton = CreateButton(chartPanel, "- Point",
            new Vector2(306f, -8f), new Vector2(58f, 28f));
        engineDeletePointButton.onClick.AddListener(RemoveEngineLutPoint);
        CreateEngineLutEditor(chartPanel);

        root.SetActive(false);
        SelectTab(activeTab);
    }

    private RubberTireWheelScript ResolveTarget()
    {
        if (BlockMapper.IsOpen && BlockMapper.CurrentInstance != null
            && BlockMapper.CurrentInstance.Block != null)
        {
            return BlockMapper.CurrentInstance.Block.GetComponent<RubberTireWheelScript>();
        }

        // E5: simulation-time live panel. Build-mode data does not simulate, so
        // pick a simulating wheel instance instead.
        if (!simPanelOpen) return null;
        if (simTarget != null && simTarget.IsSimulating) return simTarget;
        if (Time.unscaledTime >= nextTargetScanTime)
        {
            nextTargetScanTime = Time.unscaledTime + 1f;
            simTarget = FindSimulatingWheel();
        }
        return simTarget;
    }

    private RubberTireWheelScript FindSimulatingWheel()
    {
        for (int i = RubberTireWheelScript.SimulatingInstances.Count - 1; i >= 0; i--)
        {
            RubberTireWheelScript wheel = RubberTireWheelScript.SimulatingInstances[i];
            if (wheel == null || !wheel.IsSimulating)
            {
                RubberTireWheelScript.SimulatingInstances.RemoveAt(i);
                continue;
            }
            return wheel;
        }
        return null;
    }

    internal void PinSimulationTarget(RubberTireWheelScript wheel)
    {
        if (wheel == null || !wheel.IsSimulating) return;
        simTarget = wheel;
        nextTargetScanTime = Time.unscaledTime + 1f;
    }

    private void CreateEngineLutEditor(Transform parent)
    {
        RectTransform editor = CreateRectObject("Engine LUT Editor", parent,
            new Vector2(18f, -78f), new Vector2(460f, 398f));
        engineLutEditorRoot = editor.gameObject;
        Image background = editor.gameObject.AddComponent<Image>();
        background.color = new Color(0.025f, 0.03f, 0.035f, 0.995f);

        CreateText(editor, "Assetto Corsa style: one RPM|TORQUE_NM point per line",
            12, FontStyle.Normal, new Vector2(10f, -8f), new Vector2(440f, 22f),
            TextAnchor.MiddleLeft, MutedColor);
        engineLutInput = CreateMultilineInput(editor,
            new Vector2(10f, -36f), new Vector2(440f, 288f));

        Button apply = CreateButton(editor, "Apply LUT",
            new Vector2(10f, -334f), new Vector2(110f, 28f));
        apply.onClick.AddListener(ApplyEngineLutEditor);
        Button defaults = CreateButton(editor, "Default",
            new Vector2(128f, -334f), new Vector2(100f, 28f));
        defaults.onClick.AddListener(delegate
        {
            if (engineLutInput != null) engineLutInput.text = RubberTireWheelScript.DefaultEngineTorqueLut;
            ApplyEngineLutEditor();
        });
        Button close = CreateButton(editor, "Curve",
            new Vector2(236f, -334f), new Vector2(96f, 28f));
        close.onClick.AddListener(delegate { SetEngineLutEditorVisible(false); });
        engineLutStatus = CreateText(editor, "", 11, FontStyle.Normal,
            new Vector2(10f, -368f), new Vector2(440f, 22f),
            TextAnchor.MiddleLeft, AccentColor);
        engineLutEditorRoot.SetActive(false);
    }

    private void SetEngineLutEditorVisible(bool visible)
    {
        engineLutEditorVisible = visible && activeTab == "Engine"
            && target != null && target.FactoryIsDrivenWheel();
        if (engineLutEditorRoot != null) engineLutEditorRoot.SetActive(engineLutEditorVisible);
        if (chart != null) chart.raycastTarget = activeTab == "Engine"
            && target != null && target.FactoryIsDrivenWheel() && !engineLutEditorVisible;
        if (engineLutToggleText != null)
            engineLutToggleText.text = engineLutEditorVisible ? "Curve" : "Edit RPM|Nm";
        if (engineLutEditorVisible && engineLutInput != null && target != null)
        {
            engineLutInput.text = target.FactoryGetEngineTorqueLut();
            if (engineLutStatus != null) engineLutStatus.text = "";
        }
    }

    private void ApplyEngineLutEditor()
    {
        if (target == null || engineLutInput == null) return;
        string status;
        bool valid = target.FactorySetEngineTorqueLut(engineLutInput.text, out status);
        if (engineLutStatus != null)
        {
            engineLutStatus.color = valid ? AccentColor : new Color(1f, 0.35f, 0.28f, 1f);
            engineLutStatus.text = status;
        }
        if (!valid) return;
        if (chart != null) chart.SelectEnginePoint(-1);
        OnSettingChanged();
        UpdateChartTexts();
        if (chart != null) chart.SetVerticesDirty();
    }

    private void AddEngineLutPoint()
    {
        if (target == null || chart == null) return;
        int inserted;
        if (!target.FactoryAddEngineLutPoint(chart.SelectedEnginePoint, out inserted)) return;
        chart.SelectEnginePoint(inserted);
        if (engineLutInput != null) engineLutInput.text = target.FactoryGetEngineTorqueLut();
        OnSettingChanged();
        UpdateChartTexts();
    }

    private void RemoveEngineLutPoint()
    {
        if (target == null || chart == null || chart.SelectedEnginePoint < 0) return;
        int removed = chart.SelectedEnginePoint;
        if (!target.FactoryRemoveEngineLutPoint(removed)) return;
        int remaining = target.FactoryEngineLutPointCount();
        chart.SelectEnginePoint(Mathf.Clamp(removed, 0, remaining - 1));
        if (engineLutInput != null) engineLutInput.text = target.FactoryGetEngineTorqueLut();
        OnSettingChanged();
        UpdateChartTexts();
    }

    private void SelectTab(string tab)
    {
        activeTab = tab;
        focusedGroupKey = null;
        UpdateGroupButton();
        if (settingsScroll != null) settingsScroll.verticalNormalizedPosition = 1f;
        for (int i = 0; i < tabButtons.Count; i++)
        {
            Image image = tabButtons[i].GetComponent<Image>();
            if (image != null)
                image.color = Tabs[i] == activeTab
                    ? new Color(0.12f, 0.42f, 0.48f, 1f)
                    : new Color(0.12f, 0.14f, 0.16f, 1f);
        }

        ApplyRowVisibility();
        UpdateChartMode();
        UpdateChartTexts();
    }

    private void UpdateChartMode()
    {
        if (chart == null) return;
        bool drivenEngine = activeTab == "Engine"
            && target != null && target.FactoryIsDrivenWheel();
        if (engineLutToggleButton != null)
            engineLutToggleButton.gameObject.SetActive(drivenEngine);
        if (engineAddPointButton != null)
            engineAddPointButton.gameObject.SetActive(drivenEngine);
        if (engineDeletePointButton != null)
            engineDeletePointButton.gameObject.SetActive(drivenEngine);
        if (!drivenEngine) SetEngineLutEditorVisible(false);
        if (drivenEngine)
        {
            chart.Kind = RubberTireChartKind.Engine;
            chartTitle.text = "ENGINE TORQUE / POWER";
            chartLegend.text = "CYAN RPM|Nm LUT (drag white points)    ORANGE power    WHITE live RPM";
        }
        else if (activeTab == "Engine")
        {
            chart.Kind = RubberTireChartKind.None;
            chartTitle.text = "NON-DRIVEN WHEEL / BRAKE";
            chartLegend.text = "Propulsion, engine braking and shift controls are disabled; service braking remains active.";
        }
        else if (activeTab == "Tire")
        {
            chart.Kind = RubberTireChartKind.Tire;
            chartTitle.text = "COMBINED FRICTION ELLIPSE";
            chartLegend.text = "CYAN static limit    ORANGE kinetic limit    WHITE live Fx/Fn, Fy/Fn";
        }
        else if (activeTab == "Support")
        {
            chart.Kind = RubberTireChartKind.Support;
            chartTitle.text = "SUPPORT FORCE / PENETRATION";
            chartLegend.text = "CYAN spring branch at zero normal velocity";
        }
        else
        {
            chart.Kind = RubberTireChartKind.None;
            chartTitle.text = activeTab == "Contact" ? "CONTACT PIPELINE" : "DEBUG VISUALS";
            chartLegend.text = activeTab == "Contact"
                ? "Radial all-layer rays -> per-collider aggregation -> point support -> body patch friction"
                : "These controls affect diagnostics only; physical parameters live in the other pages.";
        }
        if (contactLive != null && activeTab != "Contact") contactLive.text = "";
        chart.raycastTarget = drivenEngine && !engineLutEditorVisible;
        chart.SetVerticesDirty();
    }

    // =========================
    // Rows
    // =========================

    private void RebindRows()
    {
        List<RubberTireFactorySetting> settings = target.GetFactorySettings();
        if (rows.Count != settings.Count)
        {
            BuildRows(settings);
            return;
        }

        // Every wheel builds an identical settings layout; rows are reused and
        // only the delegate targets change.
        for (int i = 0; i < rows.Count; i++)
        {
            rows[i].Setting = settings[i];
            rows[i].LastShown = float.NaN;
        }
    }

    private void BuildRows(List<RubberTireFactorySetting> settings)
    {
        for (int i = settingsContent.childCount - 1; i >= 0; i--)
            Destroy(settingsContent.GetChild(i).gameObject);
        rows.Clear();
        headers.Clear();
        headersByKey.Clear();

        string previousKey = null;
        for (int i = 0; i < settings.Count; i++)
        {
            RubberTireFactorySetting setting = settings[i];
            string groupKey = setting.Tab + "/" + setting.Group;
            if (!String.Equals(groupKey, previousKey, StringComparison.Ordinal))
            {
                previousKey = groupKey;
                if (!headersByKey.ContainsKey(groupKey))
                    CreateGroupHeader(setting.Group, groupKey);
            }
            rows.Add(setting.IsToggle
                ? CreateToggleRow(setting, groupKey)
                : CreateFloatRow(setting, groupKey));
        }
        UpdateGroupButton();
    }

    private void CreateGroupHeader(string group, string groupKey)
    {
        RubberTireGroupHeader header = new RubberTireGroupHeader();
        header.Group = group;
        header.GroupKey = groupKey;

        RectTransform rect = CreateRectObject("H_" + groupKey, settingsContent,
            Vector2.zero, new Vector2(313f, 22f));
        header.Root = rect.gameObject;
        LayoutElement element = rect.gameObject.AddComponent<LayoutElement>();
        element.preferredHeight = 22f;
        Image bg = rect.gameObject.AddComponent<Image>();
        bg.color = new Color(0.07f, 0.09f, 0.11f, 1f);
        Button button = rect.gameObject.AddComponent<Button>();
        button.targetGraphic = bg;
        header.Label = CreateText(rect, "- " + group.ToUpperInvariant(), 11, FontStyle.Bold,
            new Vector2(8f, -2f), new Vector2(297f, 18f), TextAnchor.MiddleLeft, AccentColor);

        string capturedKey = groupKey;
        button.onClick.AddListener(delegate
        {
            if (!collapsedGroups.Remove(capturedKey)) collapsedGroups.Add(capturedKey);
            ApplyRowVisibility();
        });

        headers.Add(header);
        headersByKey.Add(groupKey, header);
    }

    private RubberTireSettingRow CreateFloatRow(RubberTireFactorySetting setting, string groupKey)
    {
        RubberTireSettingRow row = new RubberTireSettingRow();
        row.Setting = setting;
        row.GroupKey = groupKey;

        RectTransform rect = CreateRectObject(setting.Key, settingsContent,
            Vector2.zero, new Vector2(313f, 52f));
        row.Root = rect.gameObject;
        LayoutElement element = rect.gameObject.AddComponent<LayoutElement>();
        element.preferredHeight = 52f;
        Image bg = rect.gameObject.AddComponent<Image>();
        bg.color = RowColor;

        CreateText(rect, setting.Label, 11, FontStyle.Normal,
            new Vector2(10f, -2f), new Vector2(210f, 22f), TextAnchor.MiddleLeft, Color.white);

        row.Input = CreateNumericInput(rect, new Vector2(224f, -3f), new Vector2(81f, 22f));
        row.Input.text = FormatValue(setting.GetFloat());

        row.Slider = CreateSlider(rect, new Vector2(10f, -31f), new Vector2(295f, 16f));
        row.Suppress = true;
        row.Slider.value = ValueToSlider(setting, setting.GetFloat());
        row.Suppress = false;

        RubberTireSettingRow captured = row;
        row.Slider.onValueChanged.AddListener(delegate(float u)
        {
            if (captured.Suppress) return;
            float value = SliderToValue(captured.Setting, u);
            if (!ValueDiffers(captured.Setting.GetFloat(), value)) return;
            captured.Setting.SetFloat(value);
            captured.LastShown = captured.Setting.GetFloat();
            if (captured.Input != null && !InputHasFocus(captured.Input))
                captured.Input.text = FormatValue(captured.LastShown);
            OnSettingChanged();
        });

        row.Input.onEndEdit.AddListener(delegate(string text)
        {
            float parsed;
            if (TryParseValue(text, out parsed))
            {
                float clamped = Mathf.Clamp(parsed, captured.Setting.Min, captured.Setting.Max);
                // E8: write back only when the value truly differs, so the
                // rounded display text can never degrade stored precision.
                if (ValueDiffers(captured.Setting.GetFloat(), clamped))
                {
                    captured.Setting.SetFloat(clamped);
                    OnSettingChanged();
                }
            }
            captured.LastShown = float.NaN;
            RefreshRow(captured);
        });

        AddHoverTooltip(rect.gameObject, row);
        return row;
    }

    private RubberTireSettingRow CreateToggleRow(RubberTireFactorySetting setting, string groupKey)
    {
        RubberTireSettingRow row = new RubberTireSettingRow();
        row.Setting = setting;
        row.GroupKey = groupKey;

        GameObject toggleObject = Make.Prefab("UIFactory3", "Text Toggle", settingsContent);
        toggleObject.name = setting.Key;
        row.Root = toggleObject;
        RectTransform rect = toggleObject.GetComponent<RectTransform>();
        rect.sizeDelta = new Vector2(313f, 34f);
        LayoutElement element = toggleObject.GetComponent<LayoutElement>();
        if (element == null) element = toggleObject.AddComponent<LayoutElement>();
        element.preferredHeight = 34f;
        Image bg = toggleObject.GetComponent<Image>();
        if (bg != null) bg.color = RowColor;

        Text label = toggleObject.GetComponentInChildren<Text>();
        if (label != null) label.text = setting.Label;
        row.Toggle = toggleObject.GetComponent<Toggle>();
        row.Toggle.isOn = setting.GetBool();

        RubberTireSettingRow captured = row;
        row.Toggle.onValueChanged.AddListener(delegate(bool value)
        {
            if (captured.Suppress) return;
            if (captured.Setting.GetBool() == value) return;
            captured.Setting.SetBool(value);
            OnSettingChanged();
            // Dependent rows (When conditions) may have flipped.
            ApplyRowVisibility();
        });

        AddHoverTooltip(toggleObject, row);
        return row;
    }

    private void AddHoverTooltip(GameObject rowObject, RubberTireSettingRow row)
    {
        RubberTireRowHover hover = rowObject.AddComponent<RubberTireRowHover>();
        hover.Controller = this;
        hover.Row = row;
    }

    internal void ShowTooltip(RubberTireFactorySetting setting)
    {
        if (tooltipLabel == null) return;
        if (setting == null)
        {
            tooltipLabel.text = DefaultTooltip;
            return;
        }

        string text = setting.Tooltip;
        if (!setting.IsToggle)
        {
            string range = "Range " + FormatValue(setting.Min) + " - " + FormatValue(setting.Max)
                         + "  |  default " + FormatValue(setting.DefaultFloat);
            text = String.IsNullOrEmpty(text) ? range : text + "\n" + range;
        }
        else if (String.IsNullOrEmpty(text))
        {
            text = setting.Label + "  |  default " + (setting.DefaultBool ? "on" : "off");
        }
        tooltipLabel.text = text;
    }

    private void OnSettingChanged()
    {
        // Commit is batched onto the 0.2 s tick (and flushed on target switch)
        // so slider drags do not serialize the whole record per event.
        commitPending = true;
        UpdateChartMode();
        if (chart != null) chart.SetVerticesDirty();
    }

    private void OnEngineCurveDragged()
    {
        if (target == null) return;
        if (engineLutInput != null)
            engineLutInput.text = target.FactoryGetEngineTorqueLut();
        if (engineLutStatus != null)
        {
            engineLutStatus.color = AccentColor;
            engineLutStatus.text = "Curve point moved";
        }
        OnSettingChanged();
        UpdateChartTexts();
    }

    private void CycleFocusedGroup()
    {
        List<string> keys = new List<string>(8);
        for (int i = 0; i < headers.Count; i++)
        {
            string key = headers[i].GroupKey;
            if (key.StartsWith(activeTab + "/", StringComparison.Ordinal)) keys.Add(key);
        }

        if (keys.Count == 0) focusedGroupKey = null;
        else if (String.IsNullOrEmpty(focusedGroupKey)) focusedGroupKey = keys[0];
        else
        {
            int index = keys.IndexOf(focusedGroupKey);
            focusedGroupKey = index >= 0 && index + 1 < keys.Count ? keys[index + 1] : null;
        }
        UpdateGroupButton();
        ApplyRowVisibility();
        if (settingsScroll != null) settingsScroll.verticalNormalizedPosition = 1f;
    }

    private void UpdateGroupButton()
    {
        if (groupButtonText == null) return;
        if (String.IsNullOrEmpty(focusedGroupKey))
        {
            groupButtonText.text = "Group: All  >";
            return;
        }
        RubberTireGroupHeader header;
        groupButtonText.text = headersByKey.TryGetValue(focusedGroupKey, out header)
            ? "Group: " + header.Group + "  >"
            : "Group: All  >";
    }

    private void ResetActiveTab()
    {
        if (target == null) return;
        target.FactoryResetTab(activeTab);
        commitPending = false;
        ForceRefresh();
    }

    private void ApplyToAllWheels()
    {
        if (target == null) return;
        if (commitPending)
        {
            commitPending = false;
            target.FactoryCommitSettings();
        }

        RubberTireWheelScript[] wheels = UnityEngine.Object.FindObjectsOfType<RubberTireWheelScript>();
        Transform machineRoot = target.transform.root;
        for (int i = 0; i < wheels.Length; i++)
        {
            RubberTireWheelScript wheel = wheels[i];
            if (wheel == null || wheel == target) continue;
            if (wheel.transform.root != machineRoot) continue;
            wheel.FactoryApplyConfigFrom(target);
        }
    }

    private void UpdateAdvancedButton()
    {
        if (advancedButtonText != null)
            advancedButtonText.text = showAdvanced ? "Adv: On" : "Adv: Off";
    }

    private void ForceRefresh()
    {
        for (int i = 0; i < rows.Count; i++) rows[i].LastShown = float.NaN;
        RefreshBindings();
        ApplyRowVisibility();
        if (chart != null) chart.SetVerticesDirty();
        UpdateChartTexts();
    }

    private void RefreshBindings()
    {
        for (int i = 0; i < rows.Count; i++) RefreshRow(rows[i]);
    }

    private void RefreshRow(RubberTireSettingRow row)
    {
        if (row.Setting.IsToggle)
        {
            if (row.Toggle == null) return;
            bool value = row.Setting.GetBool();
            if (row.Toggle.isOn != value)
            {
                row.Suppress = true;
                row.Toggle.isOn = value;
                row.Suppress = false;
            }
            return;
        }

        float current = row.Setting.GetFloat();
        float epsilon = Mathf.Max(1e-6f, Mathf.Abs(current) * 1e-5f);
        // NaN in LastShown (forced refresh) falls through the comparison.
        if (Mathf.Abs(row.LastShown - current) <= epsilon) return;
        row.LastShown = current;
        if (row.Input != null && !InputHasFocus(row.Input))
            row.Input.text = FormatValue(current);
        if (row.Slider != null)
        {
            row.Suppress = true;
            row.Slider.value = ValueToSlider(row.Setting, current);
            row.Suppress = false;
        }
    }

    private void ApplyRowVisibility()
    {
        if (rows.Count == 0) return;

        for (int i = 0; i < headers.Count; i++) headers[i].AnyVisible = false;
        bool anyRowPassesFocus = false;

        for (int i = 0; i < rows.Count; i++)
        {
            RubberTireSettingRow row = rows[i];
            RubberTireFactorySetting setting = row.Setting;
            bool pass = String.Equals(setting.Tab, activeTab, StringComparison.Ordinal)
                && (String.IsNullOrEmpty(focusedGroupKey)
                    || String.Equals(row.GroupKey, focusedGroupKey, StringComparison.Ordinal))
                && (!setting.Advanced || showAdvanced)
                && (setting.VisibleWhen == null || setting.VisibleWhen());
            if (pass)
            {
                anyRowPassesFocus = true;
                RubberTireGroupHeader groupHeader;
                if (headersByKey.TryGetValue(row.GroupKey, out groupHeader))
                    groupHeader.AnyVisible = true;
            }
            bool shown = pass && !collapsedGroups.Contains(row.GroupKey);
            if (row.Root != null && row.Root.activeSelf != shown)
                row.Root.SetActive(shown);
        }

        if (!String.IsNullOrEmpty(focusedGroupKey) && !anyRowPassesFocus)
        {
            focusedGroupKey = null;
            UpdateGroupButton();
            ApplyRowVisibility();
            return;
        }

        for (int i = 0; i < headers.Count; i++)
        {
            RubberTireGroupHeader header = headers[i];
            if (header.Root == null) continue;
            if (header.Root.activeSelf != header.AnyVisible)
                header.Root.SetActive(header.AnyVisible);
            if (header.AnyVisible && header.Label != null)
            {
                string text = (collapsedGroups.Contains(header.GroupKey) ? "+ " : "- ")
                            + header.Group.ToUpperInvariant();
                if (header.Label.text != text) header.Label.text = text;
            }
        }
    }

    // =========================
    // Chart context texts (E6)
    // =========================

    private void UpdateChartTexts()
    {
        if (target == null || chart == null || chartAxisLeft == null) return;

        if (chart.Kind == RubberTireChartKind.Engine)
        {
            float baseRpm, holdRpm, redlineRpm;
            target.GetEngineCurveBreakpoints(out baseRpm, out holdRpm, out redlineRpm);
            float maxTorque = 1e-4f;
            float maxPower = 1e-4f;
            const int samples = 48;
            for (int i = 0; i <= samples; i++)
            {
                float torque, power;
                target.FactoryEnginePoint(redlineRpm * i / samples, out torque, out power);
                if (torque > maxTorque) maxTorque = torque;
                if (power > maxPower) maxPower = power;
            }
            chartAxisLeft.text = "T max " + maxTorque.ToString("0") + " Nm";
            chartAxisRight.text = "P max " + (maxPower / 1000f).ToString("0.#") + " kW";
            chartAxisX.text = "0 - " + redlineRpm.ToString("0")
                + " RPM  |  final " + target.FactoryCurrentTotalDriveRatio().ToString("0.###")
                + " total ratio in current gear";
        }
        else if (chart.Kind == RubberTireChartKind.Support)
        {
            chartAxisLeft.text = "F max " + target.FactorySupportForce(0.25f).ToString("0") + " N";
            chartAxisRight.text = "";
            chartAxisX.text = "penetration 0 - 0.25 m";
        }
        else if (chart.Kind == RubberTireChartKind.Tire)
        {
            float staticLong, staticLat, kineticLong, kineticLat;
            target.FactoryFrictionEllipse(false, out staticLong, out staticLat);
            target.FactoryFrictionEllipse(true, out kineticLong, out kineticLat);
            chartAxisLeft.text = "static " + staticLong.ToString("0.##") + " / " + staticLat.ToString("0.##");
            chartAxisRight.text = "kinetic " + kineticLong.ToString("0.##") + " / " + kineticLat.ToString("0.##");
            chartAxisX.text = "x longitudinal, y lateral";
        }
        else
        {
            chartAxisLeft.text = "";
            chartAxisRight.text = "";
            chartAxisX.text = "";
        }

        UpdateLiveStrip();
    }

    private void UpdateLiveStrip()
    {
        if (chartLive == null) return;
        if (!target.IsSimulating)
        {
            chartLive.text = simPanelOpen ? "not simulating" : "";
            if (contactLive != null && activeTab == "Contact") contactLive.text = "";
            return;
        }

        if (chart.Kind == RubberTireChartKind.Engine)
        {
            chartLive.text = "GEAR " + target.FactoryCurrentGearLabel()
                + "   THR " + Mathf.RoundToInt(target.FactoryThrottle01() * 100f) + "%"
                + "   BRK " + Mathf.RoundToInt(target.FactoryBrake01() * 100f) + "%"
                + "   RPM " + Mathf.Max(0f, target.FactoryCurrentEngineRpm()).ToString("0")
                + (target.FactoryLimiterCut() ? "   LIMIT" : "")
                + "   gear cap "
                + target.FactoryGearWheelOmegaLimit(target.FactoryCurrentGear()).ToString("0.0")
                + " rad/s";
        }
        else if (chart.Kind == RubberTireChartKind.Tire)
        {
            float longitudinal, lateral;
            target.FactoryCurrentFrictionPoint(out longitudinal, out lateral);
            float longSlip, sideSlip, lockBlend;
            bool staticSolved;
            target.FactoryTireSlipDiagnostics(
                out longSlip, out sideSlip, out lockBlend, out staticSolved);
            int contactCount;
            float totalLoad;
            target.FactoryLiveContactSummary(out contactCount, out totalLoad);
            chartLive.text = "LONG Fx/Fn " + longitudinal.ToString("0.00")
                + "   LAT Fy/Fn " + lateral.ToString("0.00")
                + "   slip L/S " + longSlip.ToString("0.00") + "/" + sideSlip.ToString("0.00")
                + "   lock " + Mathf.RoundToInt(lockBlend * 100f) + "%"
                + (staticSolved ? " STICK" : "")
                + "   load " + totalLoad.ToString("0") + " N";
        }
        else
        {
            int contactCount;
            float totalLoad;
            target.FactoryLiveContactSummary(out contactCount, out totalLoad);
            chartLive.text = "contacts " + contactCount + "   load " + totalLoad.ToString("0") + " N";
        }

        if (contactLive != null && activeTab == "Contact")
        {
            int count = target.FactoryContactSampleCount();
            int rays, rawHits, acceptedHits, saturated;
            target.FactoryContactQueryDiagnostics(out rays, out rawHits, out acceptedHits, out saturated);
            string text = "live samples: " + count
                + "   self hits filtered: " + target.FactoryFilteredOwnMachineHits()
                + "\nrays " + rays + "   raw hits " + rawHits
                + "   accepted " + acceptedHits
                + (saturated > 0 ? "   SATURATED x" + saturated : "");
            for (int i = 0; i < count; i++)
            {
                float pen, gate;
                target.FactoryContactSample(i, out pen, out gate);
                text += "\n#" + i + "  pen " + pen.ToString("0.000") + " m   gate " + gate.ToString("0.00");
            }
            contactLive.text = text;
        }
    }

    // =========================
    // Slider mapping (E1)
    // =========================

    private static float SliderToValue(RubberTireFactorySetting setting, float u)
    {
        u = Mathf.Clamp01(u);
        // Cubic response gives fine control near the low end of wide ranges
        // while still reaching Max; works with Min = 0 where log scales fail.
        if (setting.Curved) u = u * u * u;
        return setting.Min + (setting.Max - setting.Min) * u;
    }

    private static float ValueToSlider(RubberTireFactorySetting setting, float value)
    {
        float span = setting.Max - setting.Min;
        if (span <= 1e-8f) return 0f;
        float u = Mathf.Clamp01((value - setting.Min) / span);
        if (setting.Curved) u = Mathf.Pow(u, 1f / 3f);
        return u;
    }

    private static bool ValueDiffers(float a, float b)
    {
        return Mathf.Abs(a - b) > Mathf.Max(1e-6f, Mathf.Abs(a) * 1e-5f);
    }

    private UISlider CreateSlider(Transform parent, Vector2 topLeft, Vector2 size)
    {
        RectTransform rect = CreateRectObject("Slider", parent, topLeft, size);

        // Put the raycast target on the same object as Slider. Previously only
        // the 4 px track and 10 px handle were graphics; in a ScrollRect the
        // remaining visible row sent the drag to scrolling/modal UI instead.
        Image hitArea = rect.gameObject.AddComponent<Image>();
        hitArea.color = new Color(1f, 1f, 1f, 0.001f);
        hitArea.raycastTarget = true;

        GameObject trackObject = new GameObject("Track", typeof(RectTransform), typeof(CanvasRenderer), typeof(Image));
        trackObject.layer = rect.gameObject.layer;
        RectTransform trackRect = trackObject.GetComponent<RectTransform>();
        trackRect.SetParent(rect, false);
        trackRect.anchorMin = new Vector2(0f, 0.5f);
        trackRect.anchorMax = new Vector2(1f, 0.5f);
        trackRect.pivot = new Vector2(0.5f, 0.5f);
        trackRect.anchoredPosition = Vector2.zero;
        trackRect.sizeDelta = new Vector2(0f, 4f);
        Image trackImage = trackObject.GetComponent<Image>();
        trackImage.color = new Color(0.16f, 0.19f, 0.22f, 1f);
        trackImage.raycastTarget = false;

        GameObject handleArea = new GameObject("Handle Slide Area", typeof(RectTransform));
        handleArea.layer = rect.gameObject.layer;
        RectTransform handleAreaRect = handleArea.GetComponent<RectTransform>();
        handleAreaRect.SetParent(rect, false);
        handleAreaRect.anchorMin = Vector2.zero;
        handleAreaRect.anchorMax = Vector2.one;
        handleAreaRect.sizeDelta = new Vector2(-10f, 0f);
        handleAreaRect.anchoredPosition = Vector2.zero;

        GameObject handleObject = new GameObject("Handle", typeof(RectTransform), typeof(CanvasRenderer), typeof(Image));
        handleObject.layer = rect.gameObject.layer;
        RectTransform handleRect = handleObject.GetComponent<RectTransform>();
        handleRect.SetParent(handleAreaRect, false);
        handleRect.sizeDelta = new Vector2(10f, 14f);
        Image handleImage = handleObject.GetComponent<Image>();
        handleImage.color = AccentColor;
        handleImage.raycastTarget = true;

        UISlider slider = rect.gameObject.AddComponent<UISlider>();
        slider.targetGraphic = handleImage;
        slider.handleRect = handleRect;
        slider.minValue = 0f;
        slider.maxValue = 1f;
        slider.wholeNumbers = false;
        slider.direction = UISlider.Direction.LeftToRight;
        slider.interactable = true;
        Navigation navigation = slider.navigation;
        navigation.mode = Navigation.Mode.None;
        slider.navigation = navigation;
        return slider;
    }

    private InputField CreateNumericInput(Transform parent, Vector2 topLeft, Vector2 size)
    {
        RectTransform rect = CreateRectObject("Numeric Input", parent, topLeft, size);
        Image background = rect.gameObject.AddComponent<Image>();
        background.color = new Color(0.025f, 0.03f, 0.035f, 1f);
        background.raycastTarget = true;

        Text valueText = CreateText(rect, "", 11, FontStyle.Normal,
            new Vector2(5f, -1f), new Vector2(size.x - 10f, size.y - 2f),
            TextAnchor.MiddleRight, Color.white);
        valueText.name = "Text";
        valueText.supportRichText = false;
        valueText.horizontalOverflow = HorizontalWrapMode.Overflow;
        valueText.verticalOverflow = VerticalWrapMode.Truncate;

        Text placeholder = CreateText(rect, "-", 11, FontStyle.Normal,
            new Vector2(5f, -1f), new Vector2(size.x - 10f, size.y - 2f),
            TextAnchor.MiddleRight, MutedColor);
        placeholder.name = "Placeholder";

        InputField input = rect.gameObject.AddComponent<InputField>();
        input.targetGraphic = background;
        input.textComponent = valueText;
        input.placeholder = placeholder;
        input.contentType = InputField.ContentType.DecimalNumber;
        input.lineType = InputField.LineType.SingleLine;
        input.interactable = true;
        Navigation navigation = input.navigation;
        navigation.mode = Navigation.Mode.None;
        input.navigation = navigation;
        return input;
    }

    private InputField CreateMultilineInput(Transform parent, Vector2 topLeft, Vector2 size)
    {
        RectTransform rect = CreateRectObject("LUT Text", parent, topLeft, size);
        Image background = rect.gameObject.AddComponent<Image>();
        background.color = new Color(0.01f, 0.012f, 0.015f, 1f);
        background.raycastTarget = true;

        Text valueText = CreateText(rect, "", 13, FontStyle.Normal,
            new Vector2(8f, -6f), new Vector2(size.x - 16f, size.y - 12f),
            TextAnchor.UpperLeft, Color.white);
        valueText.name = "Text";
        valueText.supportRichText = false;
        valueText.horizontalOverflow = HorizontalWrapMode.Wrap;
        valueText.verticalOverflow = VerticalWrapMode.Truncate;

        InputField input = rect.gameObject.AddComponent<InputField>();
        input.targetGraphic = background;
        input.textComponent = valueText;
        input.contentType = InputField.ContentType.Standard;
        input.lineType = InputField.LineType.MultiLineNewline;
        input.interactable = true;
        Navigation navigation = input.navigation;
        navigation.mode = Navigation.Mode.None;
        input.navigation = navigation;
        return input;
    }

    // =========================
    // Shared helpers
    // =========================

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

    internal void SetPointerOverLab(bool over)
    {
        if (pointerOverLab == over) return;
        pointerOverLab = over;
        if (over) MuteCameraZoom();
        else RestoreCameraZoom();
    }

    private void MuteCameraZoom()
    {
        RestoreCameraZoom();
        pointerOverLab = true;

        guardedOrbitCameras = UnityEngine.Object.FindObjectsOfType<OrbitCamera>();
        guardedOrbitZoomSpeeds = new float[guardedOrbitCameras.Length];
        for (int i = 0; i < guardedOrbitCameras.Length; i++)
        {
            if (guardedOrbitCameras[i] == null) continue;
            guardedOrbitZoomSpeeds[i] = guardedOrbitCameras[i].zoomSpeed;
            guardedOrbitCameras[i].zoomSpeed = 0f;
        }

        guardedMouseOrbits = UnityEngine.Object.FindObjectsOfType<MouseOrbit>();
        guardedMouseOrbitScrollSpeeds = new float[guardedMouseOrbits.Length];
        for (int i = 0; i < guardedMouseOrbits.Length; i++)
        {
            if (guardedMouseOrbits[i] == null) continue;
            guardedMouseOrbitScrollSpeeds[i] = guardedMouseOrbits[i].scrollSensitivityScaler;
            guardedMouseOrbits[i].scrollSensitivityScaler = 0f;
        }

        guardedScrollZooms = UnityEngine.Object.FindObjectsOfType<ScrollToZoom>();
        guardedScrollZoomSpeeds = new float[guardedScrollZooms.Length];
        for (int i = 0; i < guardedScrollZooms.Length; i++)
        {
            if (guardedScrollZooms[i] == null) continue;
            guardedScrollZoomSpeeds[i] = guardedScrollZooms[i].speed;
            guardedScrollZooms[i].speed = 0f;
        }
    }

    private void RestoreCameraZoom()
    {
        pointerOverLab = false;
        if (guardedOrbitCameras != null && guardedOrbitZoomSpeeds != null)
        {
            int count = Mathf.Min(guardedOrbitCameras.Length, guardedOrbitZoomSpeeds.Length);
            for (int i = 0; i < count; i++)
                if (guardedOrbitCameras[i] != null)
                    guardedOrbitCameras[i].zoomSpeed = guardedOrbitZoomSpeeds[i];
        }
        if (guardedMouseOrbits != null && guardedMouseOrbitScrollSpeeds != null)
        {
            int count = Mathf.Min(guardedMouseOrbits.Length, guardedMouseOrbitScrollSpeeds.Length);
            for (int i = 0; i < count; i++)
                if (guardedMouseOrbits[i] != null)
                    guardedMouseOrbits[i].scrollSensitivityScaler = guardedMouseOrbitScrollSpeeds[i];
        }
        if (guardedScrollZooms != null && guardedScrollZoomSpeeds != null)
        {
            int count = Mathf.Min(guardedScrollZooms.Length, guardedScrollZoomSpeeds.Length);
            for (int i = 0; i < count; i++)
                if (guardedScrollZooms[i] != null)
                    guardedScrollZooms[i].speed = guardedScrollZoomSpeeds[i];
        }
        guardedOrbitCameras = null;
        guardedOrbitZoomSpeeds = null;
        guardedMouseOrbits = null;
        guardedMouseOrbitScrollSpeeds = null;
        guardedScrollZooms = null;
        guardedScrollZoomSpeeds = null;
    }

    private void OnDestroy()
    {
        RestoreCameraZoom();
        if (root != null) Destroy(root);
    }
}

public sealed class RubberTireCurveGraphic : MaskableGraphic,
    IPointerDownHandler, IBeginDragHandler, IDragHandler, IEndDragHandler
{
    public RubberTireWheelScript Target;
    public RubberTireChartKind Kind;
    public Action OnEngineCurveEdited;
    private int selectedEnginePoint = -1;
    public int SelectedEnginePoint { get { return selectedEnginePoint; } }

    private int draggedPoint = -1;
    private float engineEditorMaxRpm = 1f;
    private float engineEditorTorqueScale = 1f;

    private static readonly Color GridColor = new Color(0.28f, 0.32f, 0.35f, 0.36f);
    private static readonly Color MarkerColor = new Color(0.55f, 0.62f, 0.68f, 0.55f);
    private static readonly Color Cyan = new Color(0.20f, 0.84f, 0.94f, 1f);
    private static readonly Color Orange = new Color(1f, 0.58f, 0.18f, 1f);
    private static readonly Color White = new Color(0.94f, 0.96f, 0.98f, 0.9f);
    private static readonly Color Selected = new Color(1f, 0.86f, 0.20f, 1f);

    public void SelectEnginePoint(int index)
    {
        selectedEnginePoint = index;
        SetVerticesDirty();
    }

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
        // C5/E6: breakpoints come from the same source the physics uses.
        float baseRpm, holdRpm, redlineRpm;
        Target.GetEngineCurveBreakpoints(out baseRpm, out holdRpm, out redlineRpm);
        float maxRpm = Mathf.Max(1f, redlineRpm);

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
        engineEditorMaxRpm = maxRpm;
        // Keep some vertical headroom so the highest key can be dragged upward
        // as well as down. Releasing and dragging again expands it further.
        engineEditorTorqueScale = Mathf.Max(1f, maxTorque * 1.15f);

        if (baseRpm > 0f)
            AddLine(vh, Plot(r, baseRpm / maxRpm, 0f), Plot(r, baseRpm / maxRpm, 1f), 1f, MarkerColor);
        if (holdRpm > 0f)
            AddLine(vh, Plot(r, holdRpm / maxRpm, 0f), Plot(r, holdRpm / maxRpm, 1f), 1f, MarkerColor);

        Vector2 previousTorque = Vector2.zero;
        Vector2 previousPower = Vector2.zero;
        for (int i = 0; i <= samples; i++)
        {
            float u = i / (float)samples;
            float rpm = maxRpm * u;
            float torque, power;
            Target.FactoryEnginePoint(rpm, out torque, out power);
            Vector2 torquePoint = Plot(r, u, torque / engineEditorTorqueScale);
            Vector2 powerPoint = Plot(r, u, power / maxPower);
            if (i > 0)
            {
                AddLine(vh, previousTorque, torquePoint, 3f, Cyan);
                AddLine(vh, previousPower, powerPoint, 3f, Orange);
            }
            previousTorque = torquePoint;
            previousPower = powerPoint;
        }

        int pointCount = Target.FactoryEngineLutPointCount();
        for (int i = 0; i < pointCount; i++)
        {
            float pointRpm;
            float pointTorque;
            Target.FactoryEngineLutPoint(i, out pointRpm, out pointTorque);
            Vector2 point = Plot(r,
                Mathf.Clamp01(pointRpm / maxRpm),
                Mathf.Clamp01(pointTorque / engineEditorTorqueScale));
            float marker = i == selectedEnginePoint ? 7f : 4f;
            Color markerColor = i == selectedEnginePoint ? Selected : White;
            AddLine(vh, point + new Vector2(-marker, 0f), point + new Vector2(marker, 0f), 2f, markerColor);
            AddLine(vh, point + new Vector2(0f, -marker), point + new Vector2(0f, marker), 2f, markerColor);
        }

        float liveRpm = Target.FactoryCurrentEngineRpm();
        if (liveRpm >= 0f)
        {
            float u = Mathf.Clamp01(liveRpm / maxRpm);
            AddLine(vh, Plot(r, u, 0f), Plot(r, u, 1f), 2f, White);
        }
    }

    public void OnPointerDown(PointerEventData eventData)
    {
        if (eventData == null || Target == null || Kind != RubberTireChartKind.Engine) return;
        Vector2 local;
        if (!RectTransformUtility.ScreenPointToLocalPointInRectangle(
                rectTransform, eventData.position, eventData.pressEventCamera, out local)) return;

        Rect r = rectTransform.rect;
        int best = -1;
        float bestDistance = 18f * 18f;
        int count = Target.FactoryEngineLutPointCount();
        for (int i = 0; i < count; i++)
        {
            float rpm;
            float torque;
            Target.FactoryEngineLutPoint(i, out rpm, out torque);
            Vector2 point = Plot(r,
                rpm / Mathf.Max(1f, engineEditorMaxRpm),
                torque / Mathf.Max(1f, engineEditorTorqueScale));
            float distance = (point - local).sqrMagnitude;
            if (distance > bestDistance) continue;
            best = i;
            bestDistance = distance;
        }
        draggedPoint = best;
        selectedEnginePoint = best;
        SetVerticesDirty();
        if (draggedPoint >= 0) eventData.Use();
    }

    public void OnBeginDrag(PointerEventData eventData)
    {
        if (draggedPoint >= 0 && eventData != null) eventData.Use();
    }

    public void OnDrag(PointerEventData eventData)
    {
        if (draggedPoint < 0 || eventData == null || Target == null) return;
        Vector2 local;
        if (!RectTransformUtility.ScreenPointToLocalPointInRectangle(
                rectTransform, eventData.position, eventData.pressEventCamera, out local)) return;
        Rect r = rectTransform.rect;
        float u = Mathf.Clamp01((local.x - r.xMin) / Mathf.Max(1f, r.width));
        float v = Mathf.Clamp01((local.y - r.yMin) / Mathf.Max(1f, r.height));
        if (Target.FactoryMoveEngineLutPoint(
                draggedPoint, u * engineEditorMaxRpm, v * engineEditorTorqueScale))
        {
            SetVerticesDirty();
            if (OnEngineCurveEdited != null) OnEngineCurveEdited();
        }
        eventData.Use();
    }

    public void OnEndDrag(PointerEventData eventData)
    {
        draggedPoint = -1;
        if (eventData != null) eventData.Use();
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
