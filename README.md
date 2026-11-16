# 驭骨 · BonePilot

> **当前分支 `arp-bip-v2.7` = 官方 v5.2.45 + ARP BIP 扩展 v3.2（实验分支）**
>
> 给 ARP 骨架加了「拖 FK 手控骨、手臂像 3ds Max Biped 一样 IK 跟随、结果仍写回 FK」
> 的能力，并让 BIP 拖动时也只显示当前控制器。`main` 分支仍是纯净的 v5.2.45，
> [Releases 最新正式版](https://github.com/dazuoge/yugu-bonepilot/releases/latest)
> 也不受影响。扩展源码与说明见 [`bip/`](bip/README.md)。

Blender 插件：按集合导航控制器，快速旋转/移动，逐段撤销，让动画操作跟手。

面向 **Blender 5.2+**（Rigify / AniMate / ARP 类骨架均可使用）；本分支的 BIP 扩展在 **Blender 5.3** 上实测。

当前版本：**v5.2.45 + ARP BIP v3.2**（2026-11-16，本分支）

## 下载

- **打包下载（推荐）**：[Releases → `bip-v3.2`](https://github.com/dazuoge/yugu-bonepilot/releases/tag/bip-v3.2)
  - `yugu-bonepilot-bip-v3.2.zip` —— 单文件可拖入安装版（包内 `controller_navigator_v4971.py`）
  - `yugu-bonepilot-bip-v3.2-source.zip` —— 扩展源码、幂等植入器、全套回归与 v1.6~v3.1 回退备份
  - 该 Release 为 **预发布**，`/releases/latest` 仍指向正式版 v5.2.45。
  - `bip-v3.1` 及更早版本已过时：驭骨自带的 IK/FK 快速切换（I 键 / O 面板）
    走自算矩阵、会写坏 FK 动画，请用 v3.2。
- **本分支整合版（推荐）**：[`controller_navigator.py`](controller_navigator.py)
  —— 已内置 ARP BIP v3.2，可直接按下方步骤安装；本地文件 md5 `2687e7e7b9a5d4b7776e0c9c6f7b23d5`
  （CRLF 换行；经 GitHub 下载时行尾可能被规范化，不影响使用）。
- **官方稳定版 v5.2.45**：切到 `main` 分支，或到
  [Releases → 最新版](https://github.com/dazuoge/yugu-bonepilot/releases/latest)
  下载 `yugu-bonepilot-v5.2.45.zip`。

## 安装

1. Blender → `Edit → Preferences → Add-ons` → 右上角 `⌄` → **Install from Disk…**
2. 选本分支的 `controller_navigator.py`（官方稳定版则是 `yugu-bonepilot-v5.2.45.zip`）
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

## ARP BIP 拖手（本分支新增）

- **FK 保持 FK**：拖动 ARP 的手部 FK 控制器，上臂 / 前臂 IK 式跟随，但结果写回
  `c_arm_fk` / `c_forearm_fk` 的旋转；不留 IK 约束，`ik_fk_switch` 不变。
- **自动理顺旋转曲线**：多帧画弧时自动消除四元数 `q/-q`、欧拉多解分支造成的
  中间帧暴跳，只换数值表示、关键帧姿态逐帧零改变。
- **运动路径实时跟手**：拖动中直接写当前帧路径点，松手同步全量重算，与原生
  移动模式的跟手速度一致。
- **BIP 也独显**：拖动时只显示正在操作的手控骨，松手 / 取消 / 切骨逐骨还原。
- **前臂铰链 / 近伸直 / 往外拉直**：锁轴投影、解析二连杆与步长钳位。

- **拖动结束必定还原控制器**（v2.8）：修掉「拖完还得按一次 Alt H」。
  补齐 Blender 外部取消时的 `cancel()`、交还失败改为直接还原、开拖前脏状态自愈、
  0.5 s 定时守卫兜底，四条实测残留路径全部封死。

- **撤回后不再残留隐藏控制器**（v2.9）：修掉「BIP 移动完按一下 Ctrl+Z 又只剩手掌」。
  `undo_post` / `redo_post` 即时守卫 + 常驻定时器补装 + `persistent` 回调，
  只还原显隐，姿态与关键帧撤销完全不受影响；残留判据重做，不再依赖隐藏骨比例。

- **旋转模式下控件不再闪烁**（v3.0）：修掉「手掌控制器进旋转模式，其他控制器
  先隐藏、轻微一动又全部亮出来」。守卫现在先判断显隐归谁管，只处理 BIP 自己
  造成的残留；正常收尾即消费兜底授权，v2.9 的撤回修复完全保留。

- **ARP 原生 IK/FK 切换不再毁掉 FK 动画**（v3.1）：做过 BIP 拖动的手臂
  切到 IK 再切回 FK，插件会保护 FK 旋转曲线逐帧还原；如果确实在 IK 下
  摆过姿势，则完整保留 ARP 的匹配结果，只自动理顺旋转分支。

- **IK/FK 快速切换改走 ARP 原生**（v3.2）：按 I 键或 O 面板切换时，
  插件不再自算矩阵摆控制器，而是调用 AutoRig Pro 原生的
  `pose.arp_switch_snap`，并跳过会污染曲线的二次全量插帧。
  Rigify / AniMate 等有自己 snap 的骨架仍走原逻辑。

仅限 ARP 骨架，Rigify / AniMate 行为不变。详细说明、边界与实测数据见
[`bip/README.md`](bip/README.md)。

## 版本历史

见 [CHANGELOG.md](CHANGELOG.md)。

## 许可证

[MIT](LICENSE)
