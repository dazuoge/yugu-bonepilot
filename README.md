# 驭骨 · BonePilot

Blender 插件：按集合导航控制器，快速旋转/移动，逐段撤销，让动画操作跟手。

面向 **Blender 5.2+**（Rigify / AniMate / ARP 类骨架均可使用）。

当前版本：**v5.2.42**（2026-11-05）

## 下载

- **安装包（推荐）**：[Releases → 最新版](https://github.com/dazuoge/yugu-bonepilot/releases/latest)，文件名 `yugu-bonepilot-v5.2.42.zip`
- **单文件**：[`controller_navigator.py`](controller_navigator.py)（与安装包内文件逐字节一致，md5 `612731feb2a0403245ca063bd008566f`）

## 安装

1. Blender → `Edit → Preferences → Add-ons` → 右上角 `⌄` → **Install from Disk…**
2. 选 `yugu-bonepilot-v5.2.42.zip`（或直接选 `controller_navigator.py`）
3. 勾选启用「驭骨 · BonePilot」
4. 已经装过旧版的：覆盖后按 `F3 → Reload Scripts`，或重启 Blender —— 否则跑的还是内存里的旧代码

安装完进入 **Pose Mode**，在 **N 面板 → Nav** 找到插件面板。

## 功能

- **IK/FK 一键切换**：选中 IK 或 FK 控制器，按一下键切到另一边，自动收起离开的那套、把选择挪到新的控制器上。
  - Rigify 骨架：有 `rig_id`，直接调用自带 snap，pole / stretch 全对，与自带按钮结果差异 0.000000 m；
  - AniMate 类骨架：控制器骨上带 `ik_fk_switch`，矩阵快照切换，手臂/腿切换位移误差 ≤ 0.1 mm。
  - 弹出面板只列**真开关**：没被任何驱动器引用的残留属性自动过滤（Ju_Fufu 实测 5 行 → 4 行）。
- **三层饼菜单**：同一手势配修饰键弹不同饼，按使用频率分层。
  - 主饼（`Alt+左键`）身体核心 8 格：头 / 双臂 / 双腿 / 躯干 / 双手指；
  - 细节饼（`Shift+Alt+左键`）眼、耳、发束、指节这类分节细节；
  - 附属饼（`Ctrl+Alt+左键`）装备、衣服、尾巴、道具。
- **一键配置饼菜单**：读骨架原生集合，自动归类进三层饼（同一部位的 IK/FK 合成一格，同一格不同组按可信度仲裁，空集合与纯符号横幅不占位）。
- **全键位自定义**：所有动作的键位都能自己绑。偏好里一张表，点键名直接按想要的键 —— 鼠标中键 / 侧键 / 滚轮方向都认，旋转、移动模式里的滚轮也可分别绑定。
- **切集合只留一个**：从一个控制器集合跳到另一个，上一次进入的集合整组隐藏，屏幕上永远只有一套控制器。饼菜单槽位可选骨架原本自带的全部集合（含空集合）。
- **逐段撤销**：操作按段落拆分，撤销不再一撤到底。

## 版本历史

见 [CHANGELOG.md](CHANGELOG.md)。

## 许可证

[MIT](LICENSE)
