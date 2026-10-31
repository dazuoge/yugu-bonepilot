# 驭骨 · BonePilot

Blender 插件：按集合导航控制器，快速旋转/移动，逐段撤销，让动画操作跟手。

面向 **Blender 5.2+**（Rigify 与 AniMate 类骨架均可使用）。

## 功能

- **IK/FK 一键切换**：选中 IK 或 FK 控制器，按一下键切到另一边，自动收起离开的那套、把选择挪到新的控制器上。
  - Rigify 骨架：有 `rig_id`，直接调用自带 snap，pole / stretch 全对，与自带按钮结果差异 0.000000 m；
  - AniMate 类骨架：控制器骨上带 `ik_fk_switch`，矩阵快照切换，手臂/腿切换位移误差 ≤ 0.1 mm。
- **全键位自定义**：所有动作的键位都能自己绑。偏好里一张表，点键名直接按想要的键——鼠标中键 / 侧键 / 滚轮方向都认，旋转、移动模式里的滚轮也可分别绑定。
- **切集合只留一个**：从一个控制器集合跳到另一个，上一次进入的集合整组隐藏，屏幕上永远只有一套控制器。饼菜单槽位可选骨架原本自带的全部集合（含空集合）。
- **逐段撤销**：操作按段落拆分，撤销不再一撤到底。

## 安装

1. 下载本仓库（Code → Download ZIP，或 `git clone`）；
2. 打开 Blender → `Edit → Preferences → Add-ons → Install`；
3. 选择 `controller_navigator.py`（或打包后的 zip）；
4. 勾选启用「驭骨 · BonePilot」。

## 使用

进入 **Pose Mode**，在 **N 面板 → Nav** 找到插件面板。

具体键位与偏好设置说明见 [CHANGELOG.md](CHANGELOG.md) 中各版本的功能说明。

## 版本历史

见 [CHANGELOG.md](CHANGELOG.md)。

## 许可证

[MIT](LICENSE)
