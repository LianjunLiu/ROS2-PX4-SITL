# 提交改动到 GitHub 指南

> 本仓库是「伞仓」：自身只存文档、配置与**子模块指针**，代码分散在 10 个子模块里。所以「提交改动」要先判断改动落在哪一层，再走对应的流程。版本 v1 · 状态 有效。

## 1. 三层 git 结构

```text
ROS2-PX4-SITL/  (伞仓 → ROS2-PX4-SITL，分支 main)
├── PX4-Autopilot             (fork → ROS2-PX4-Autopilot，分支 ROS2)
│   └── Tools/simulation/gz   (嵌套 fork → ROS2-PX4-gazebo-models，分支 ROS2)
├── src/px4_msgs              (fork → ROS2-px4_msgs，分支 ROS2)
├── src/FAST-LIVO2            (fork → ROS2-FAST-LIVO2，分支 ROS2)
├── src/livox_ros_driver2     (fork → ROS2-livox_ros_driver2，分支 ROS2)
├── src/px4_ros_com           (上游 PX4，不改)
├── src/rpg_vikit             (上游，不改)
├── Livox-SDK2                (上游，不改)
├── Sophus                    (上游，不改)
├── Dynamic_World_Generator   (上游，不改)
└── Micro-XRCE-DDS-Agent      (上游，不改)
```

核心概念：**伞仓对每个子模块只记一个 commit SHA（gitlink）**，不记文件内容。你在子模块里提交了新东西，伞仓看到的只是「这个子模块指向了新 SHA」；必须再在伞仓里 `git add` 那个子模块路径并提交，别人才会拉到新版本。

## 2. 先判断改动在哪一层

改文件前先问一句「这个文件属于哪个 git 仓库」：

```bash
cd /home/liu/Desktop/ROS2-PX4-SITL
git rev-parse --show-toplevel   # 在某个目录下跑，看它属于哪个仓库
```

- 结果是 `/home/liu/Desktop/ROS2-PX4-SITL` → 伞仓自己的文件 → **情况 A**
- 结果是 `/home/liu/Desktop/ROS2-PX4-SITL/PX4-Autopilot`（或 `src/px4_msgs`、`src/FAST-LIVO2`、`src/livox_ros_driver2`）→ fork 子模块 → **情况 B**
- 结果是 `.../PX4-Autopilot/Tools/simulation/gz` → 嵌套子模块 → **情况 C**
- 结果落在其余 6 个上游子模块里 → **情况 D（一般别改）**

## 3. 情况 A：改伞仓自己的文件

适用于：`README.md`、`LICENSE`、`.gitmodules`、`.gitignore`、`Docs/`、`src/x500_plus/`。

```bash
cd /home/liu/Desktop/ROS2-PX4-SITL
# 编辑文件…

git add <文件或目录>
git commit -m "描述做了什么"
git push origin main
```

## 4. 情况 B：改 fork 子模块（PX4-Autopilot / px4_msgs / FAST-LIVO2 / livox_ros_driver2）

以 `PX4-Autopilot` 为例，其余三个同理（把路径和仓库名换掉）。

```bash
# ① 先进子模块，确认在 ROS2 分支
cd /home/liu/Desktop/ROS2-PX4-SITL/PX4-Autopilot
git checkout ROS2

# ② 编辑、提交、推到 fork 的 ROS2 分支
git add <文件>
git commit -m "描述做了什么"
git push origin ROS2

# ③ 回到伞仓，更新 gitlink
cd /home/liu/Desktop/ROS2-PX4-SITL
git add PX4-Autopilot              # 记录子模块的新 SHA
git commit -m "更新 PX4-Autopilot 子模块到 <简述>"
git push origin main
```

> **为什么 ③ 不可少**：fork 上推了新提交，但伞仓里记录的还是旧 SHA。不执行 ③，别人 `git submodule update` 拉到的还是旧代码；而且伞仓 `git status` 会一直显示 `modified: PX4-Autopilot (new commits)`。

## 5. 情况 C：改嵌套子模块（Tools/simulation/gz）

嵌套 = 要一层层从内往外更新，共三跳：

```bash
# ① 最内层：gazebo-models fork
cd /home/liu/Desktop/ROS2-PX4-SITL/PX4-Autopilot/Tools/simulation/gz
git checkout ROS2
git add <文件>
git commit -m "描述做了什么"
git push origin ROS2

# ② 中间层：PX4-Autopilot 里的 gitlink
cd /home/liu/Desktop/ROS2-PX4-SITL/PX4-Autopilot
git add Tools/simulation/gz
git commit -m "更新 gz 子模块到 <简述>"
git push origin ROS2

# ③ 最外层：伞仓里的 PX4-Autopilot gitlink
cd /home/liu/Desktop/ROS2-PX4-SITL
git add PX4-Autopilot
git commit -m "更新 PX4-Autopilot 子模块（含 gz）"
git push origin main
```

## 6. 情况 D：上游子模块（一般别改）

这 6 个是别人的仓库、你没有写权限：`src/px4_ros_com`、`src/rpg_vikit`、`Livox-SDK2`、`Sophus`、`Dynamic_World_Generator`、`Micro-XRCE-DDS-Agent`。

- **不改**：就这么用，伞仓指针钉在上游某 commit 上即可。
- **确实要改**：先自己 fork 一份，然后把 `.gitmodules` 里该子模块的 `url` 改成你的 fork 地址，`branch` 写上你的分支，再 `git submodule sync` + 走情况 B/C 流程。改完记得把 `.gitmodules` 一起提交。

## 7. 常用检查命令

```bash
cd /home/liu/Desktop/ROS2-PX4-SITL

git status              # 伞仓自己的改动 + 哪些子模块有「新提交」
git submodule status    # 每个子模块实际 HEAD 是否对齐伞仓记录（前缀 - / + 的含义见下）

# 看某个子模块有没有未推送的提交
git -C PX4-Autopilot status
git -C PX4-Autopilot log origin/ROS2..HEAD   # 有输出 = 还有没推到 fork 的提交
```

`git submodule status` 前缀含义：

| 前缀 | 含义 |
|---|---|
| `-` | 子模块未初始化（本地没拉代码）；伞仓 `git submodule update --init --recursive` 可解决 |
| 空格 | 已初始化且对齐 |
| `+` | 子模块 HEAD 与伞仓记录的 SHA 不一致（你改了但还没在伞仓里 `git add`） |

## 8. 坑与注意事项

1. **构建命令**：ROS2 工作区永远用 `colcon build --base-paths src`（裸 `colcon build` 会连带编译 PX4 固件）。固件用 `cd PX4-Autopilot && make px4_sitl`。
2. **构建产物**：`build/`、`install/`、`log/` 已被伞仓 `.gitignore` 忽略，不用管。
3. **子模块显示 dirty（`?`）不碍事**：例如 `src/rpg_vikit` 里 `colcon` 生成的 `bin/`、`lib/` 是未跟踪文件，会让子模块显示脏，但**不影响 gitlink**、也**不要提交**它们。
4. **fork 推到 `ROS2` 分支，不是 `main`**：四个 fork 的默认改动分支是 `ROS2`；伞仓推 `main`。
5. **别在子模块里推错远程**：fork 子模块的 `origin` 指向你自己的 fork（如 `ROS2-PX4-Autopilot`），不是上游 PX4。推之前可 `git remote -v` 确认。
6. **`.gitmodules` 里的 `branch` 字段**：只对 `git submodule update --remote` 生效，普通 `git submodule update` 用钉死的 SHA，不受影响。
7. **改了 fork 但伞仓忘了更新 gitlink**：别人拉下来还是旧代码。每次改完子模块，回到伞仓 `git status` 看一眼有没有 `(new commits)`，有就 `git add` + 提交。

## 9. 一条命令看全局状态

```bash
cd /home/liu/Desktop/ROS2-PX4-SITL
git status -s && echo '--- 子模块 ---' && git submodule status
```

出现 `+` 前缀的、或 `modified: xxx (new commits)` 的，就是「子模块推了、伞仓还没更新 gitlink」的待办项。
