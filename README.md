# RubberTire

RubberTire 是一个 Besiege 轮胎 Mod，用独立的接触、支撑、摩擦和动力模型替代原版轮子的主要物理行为。目标是在 Besiege 50/100 Hz、迭代式物理解算器的限制下，提供更可信的低速抓地、滑行、联合滑移和动力响应。

当前版本：`0.3.0`

## 功能

- 聚合多射线接触，查询所有碰撞层，不再依赖 trigger contact mask。
- 支撑与切向摩擦分离求解。
- 低速静摩擦冲量约束，并保留动态刷毛/松弛模型。
- 纵向与横向联合摩擦椭圆。
- 按有效质量限制回弹速度的支撑冲量，避免轻轮持续注入能量和 pogo 弹跳。
- 扭矩、功率、恒功率区间、红线转速和多挡变速箱动力模型。
- UIFactory 参数工作区，包括发动机曲线、摩擦椭圆实时工作点和支撑曲线。
- 原版 Block Mapper 只保留按键绑定；物理参数保存在单个不可见的自定义机器数据项中。

## 依赖

- Besiege
- [UI Factory 3](https://steamcommunity.com/sharedfiles/filedetails/?id=2913469777) `3.4.0` 或兼容版本

UIFactory 是参数编辑界面的必需前置。缺少它时物理代码仍可加载，但无法编辑轮胎参数。

## 安装

将仓库中的 `RubberTire/` 目录复制到：

```text
Besiege/Besiege_Data/Mods/RubberTire/
```

确认 UI Factory 已启用后完整重启 Besiege。程序集变更不能通过 XML 热重载生效。

## 项目结构

```text
RubberTire/                                            可直接安装的 Mod 包
src/RubberTire/RubberTireScript.cs                     接触采集、生命周期和调试显示
src/RubberTire/RubberTireWheelScript.Support.cs        法向支撑和能量安全回弹约束
src/RubberTire/RubberTireWheelScript.Lateral.cs        摩擦、低速静态约束和联合滑移
src/RubberTire/RubberTireWheelScript.Drivetrain.cs     发动机、变速箱、制动和滚阻
src/RubberTire/RubberTireWheelScript.FactorySettings.cs  UIFactory 参数绑定和机器内配置序列化
src/RubberTire/RubberTireFactoryUI.cs                  UIFactory 工作区和图表
```

## 构建

工程目标为 .NET Framework 3.5/x86，并直接引用 Besiege 与 UIFactory 的程序集。构建前设置：

- `BESIEGE_GAME_ASSEMBLIES`：`Besiege_Data/Managed/`
- `BESIEGE_UNITY_ASSEMBLIES`：Unity 程序集目录；通常同样指向 `Besiege_Data/Managed/`
- `UIFACTORY_ASSEMBLIES`：包含 `Besiege.UI.dll` 和 `Besiege.UI.Bridge.dll` 的目录

Windows 上可使用 Visual Studio/MSBuild。Linux 上需要提供 .NET Framework 3.5 reference assemblies，或使用 Roslyn `csc` 直接引用游戏自带程序集。构建产物应复制为 `RubberTire/RubberTire.dll`。

## 配置兼容性

`0.3.0` 删除了旧版散落在 XML/C# 中的 Mapper sliders/toggles，改用单个版本化配置记录。旧机器可以继续加载方块，但旧版逐项 Mapper 参数不会自动迁移，首次使用应在 UIFactory 中重新检查轮胎参数。
