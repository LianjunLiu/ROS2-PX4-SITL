# 攻击原语库（Layer A）

> `AttackPrimitive` 是攻击注入三层架构中的纯数学内核（Layer A），回答「如何把一个干净的标量真值篡改成另一个值，或决定丢弃它」；13 种原语覆盖完整性、可用性、时间序列三类；关联（系统设计 §3）（管理器分册 §6）。版本 v5 · 状态 已实现 · 代码 `attack/AttackPrimitive.hpp` / `attack/AttackPrimitive.cpp`。

---

## 1. 定位

`AttackPrimitive` 是攻击注入三层架构中的 Layer A（系统设计 §3），实现原语对单个标量真值的篡改；原语 / 真值 / 通道的统一定义见系统设计 §2。

| 性质 | 说明 |
|---|---|
| 零依赖 | 仅依赖标准库 `<cstddef>`、`<cstdint>`、`<cmath>`，不含任何 PX4 / Gazebo / uORB 头文件 |
| 纯逻辑 | 无 I/O、无消息、无定时器 |
| 无自有可变状态 | `AttackLibrary` 均为静态函数；需要记忆的状态放在 `ChannelState`（由调用方持有） |

---

## 2. 结构

Layer A 由常量、两个枚举（`PrimitiveType`、`Channel`）与三个结构体（`AttackSpec`、`ChannelState`、`AttackResult`）构成。

### 2.1 常量

```cpp
constexpr size_t kNumChannels = 10;          // 通道数（GPS 6 + 电机 4）
constexpr size_t kTimeSeriesCapacity = 4096; // 每通道时间序列缓冲容量
```

`kTimeSeriesCapacity` 决定 `DELAY` / `REPLAY` 的最大回看（§6.4）。

### 2.2 枚举

`PrimitiveType` 枚举定义攻击形状：

```cpp
enum class PrimitiveType : uint8_t {
    NONE = 0,  BIAS, SPOOF, NOISE, SCALING, DRIFT, OSCILLATION,
    RANDOM_WALK, QUANTIZE, CLAMP, FREEZE, DROP, DELAY, REPLAY,
};
```

> **注**：数值是 `attack_command` / `attack_status` 携带的线协议编号，一经发布不可改动顺序或数值，否则旧日志对不上新代码；因此 `DELAY` / `REPLAY` 追加为 12 / 13，而非插入中间。底层类型为 `uint8_t`（1 字节，范围 0–255）；`enum class` 与整数互转须显式 `static_cast`。

### 2.3 通道

`Channel` 枚举是通道的编号词汇表（定义见系统设计 §2）：

```cpp
enum class Channel : uint8_t {
    GPS_LAT = 0, GPS_LON, GPS_ALT, GPS_VEL_N, GPS_VEL_E, GPS_VEL_D,  // 0..5
    MOTOR_0 = 6, MOTOR_1, MOTOR_2, MOTOR_3,                          // 6..9
    NUM_CHANNELS = 10,
};
```

| 下标 | 通道 | 物理含义 |
|---|---|---|
| 0 | `GPS_LAT` | GPS 纬度 [deg] |
| 1 | `GPS_LON` | GPS 经度 [deg] |
| 2 | `GPS_ALT` | GPS 高度 [m] |
| 3 | `GPS_VEL_N` | GPS 速度北向 [m/s] |
| 4 | `GPS_VEL_E` | GPS 速度东向 [m/s] |
| 5 | `GPS_VEL_D` | GPS 速度地向 [m/s] |
| 6 | `MOTOR_0` | 电机 0 转速 [rpm] |
| 7 | `MOTOR_1` | 电机 1 转速 [rpm] |
| 8 | `MOTOR_2` | 电机 2 转速 [rpm] |
| 9 | `MOTOR_3` | 电机 3 转速 [rpm] |

> **注**：`Channel` 是无语义编号，Layer A 只关心「第 0 号通道」，其物理含义由注入点（Layer B）赋予。

### 2.4 配置

`AttackSpec` 是单个通道的可序列化攻击配置：

```cpp
struct AttackSpec {
    PrimitiveType type{PrimitiveType::NONE};  // 攻击形状（NONE = 关闭）
    double a{0.0}, b{0.0}, c{0.0}, d{0.0};    // 原语参数（含义随 type 变，§4）
    uint64_t start_us{0};                     // 生效起点（绝对 hrt 微秒）
    uint64_t end_us{0};                       // 生效终点；0 = 无终点
    uint32_t seed{0};                         // 随机原语种子；0 = 每通道默认
};
```

| 字段 | 含义 | 谁写入 |
|---|---|---|
| `type` | 攻击形状 | 管理器从 `attack_command.type` 解析 |
| `a b c d` | 原语参数，含义随 `type` 而定 | 管理器从 `attack_command.param[4]` 解析 |
| `start_us` | 生效起点（绝对时间） | 管理器解析 `now + t0_us` |
| `end_us` | 生效终点；`0` = 无限 | 管理器解析 `start + t1_us`（t1=0 则 0） |
| `seed` | 随机种子 | 管理器从 `attack_command.seed` 解析 |

> **注**：`AttackSpec` 是纯值——可序列化、可随消息走、可进 ulog；运行时会变的量全部在 `ChannelState`（§2.5）。

### 2.5 状态

`ChannelState` 是每通道的运行时状态，由管理器持有、不序列化；其中的**环形缓冲**承载通道历史数据流：

```cpp
struct ChannelState {
    // 点变换记忆
    double   last_value{0.0};  // 上次「通过」的值（FREEZE / RANDOM_WALK 用）
    bool     has_last{false};  // last_value 是否有效
    uint32_t rng_state{0};     // PRNG 状态，由管理器 set() 播种

    // 历史环形缓冲（DELAY / REPLAY 回看数据源）
    uint64_t hist_time[kTimeSeriesCapacity]{};   // 采样时间戳 [µs]
    double   hist_value[kTimeSeriesCapacity]{};  // 采样值，与 hist_time 同序
    size_t   hist_head{0};                       // 下一个写入格
    size_t   hist_count{0};                      // 有效条目数（≤ capacity）

    // REPLAY 循环快照
    uint64_t replay_offset[kTimeSeriesCapacity]{};  // 相对 (start − a) 的偏移 [µs]
    double   replay_value[kTimeSeriesCapacity]{};   // 快照值
    size_t   replay_count{0};                       // 快照有效条目数
    bool     replay_captured{false};                // 快照是否已截取
};
```

三组字段的性质与生命周期：

| 组 | 字段 | 服务对象 | 性质 |
|---|---|---|---|
| 点变换记忆 | `last_value` / `has_last` / `rng_state` | `FREEZE` / `RANDOM_WALK` / `NOISE` / `DROP` | 本次攻击的状态 |
| 历史环形缓冲 | `hist_time` / `hist_value` / `hist_head` / `hist_count` | `DELAY` / `REPLAY` 的回看数据源 | 通道的数据流 |
| `REPLAY` 快照 | `replay_offset` / `replay_value` / `replay_count` / `replay_captured` | `REPLAY` 循环回放 | 本次攻击的状态 |

各字段换攻击时的重置项由（管理器分册 §6）给出。

### 2.6 结果

`AttackResult` 是一次 `apply()` 的输出：

```cpp
struct AttackResult {
    bool   pass{true};   // false => 丢弃本条消息（仅 DROP 会产生 false）
    double value{0.0};   // 篡改后的值；仅 pass == true 时有效
};
```

`pass == false` 时 `value` 被忽略；注入点据此决定如何丢弃（系统设计 §5.3）。

---

## 3. 接口

```cpp
class AttackLibrary {
public:
    static AttackResult apply(double truth, const AttackSpec &spec,
                              uint64_t now_us, ChannelState &state);
    static bool active(const AttackSpec &spec, uint64_t now_us);
};
```

### 3.1 应用

`apply()` 把一次攻击施加到单个标量真值上：

| 参数 | 类型 | 含义 |
|---|---|---|
| `truth` | `double` | 注入点传来的干净真值 |
| `spec` | `const AttackSpec &` | 本通道攻击配置（只读） |
| `now_us` | `uint64_t` | 当前时间，微秒 |
| `state` | `ChannelState &` | 本通道运行时状态（会被推进） |
| 返回 | `AttackResult` | `pass == false` 仅由 `DROP` 产生 |

执行顺序：

```text
1. record_history(state, now_us, truth)   ← 无条件：先记干净真值（预滚）
2. 若 !active(spec, now_us)：               ← 未激活：直通，但刷新 last_value
       last_value = truth; has_last = true; return {true, truth}
3. result = transform(type, truth, ...)     ← 已激活：套原语
4. 若 result.pass：last_value = result.value; has_last = true
5. return result
```

第 1 步即使无攻击也在记录，保证 `DELAY` / `REPLAY` 激活前已有历史可回看；第 2 步刷新 `last_value`，保证 `FREEZE` / `RANDOM_WALK` 激活时从当前值起步。

### 3.2 生效

`active()` 返回 `true` 当且仅当同时满足：

1. `type != NONE`；
2. `now_us >= start_us`；
3. `end_us == 0`（无终点）或 `now_us < end_us`。

即判断 `now_us ∈ [start_us, end_us)`（半开区间，含起点、不含终点）。

---

## 4. 原语

13 种原语按**完整性**、**可用性**、**时间序列**三类列于下表；表中 `v` 为篡改值、`truth` 为真值、`a b c d` 为参数、`t` 为攻击启动后经过的秒数（`elapsed_s = (now − start) / 1e6`）、`N(0,1)` 为标准正态。

| 类别 | 原语 | 公式 | 参数 | 状态 | 说明 |
|---|---|---|---|---|---|
| 完整性 | `BIAS` | `v = truth + a` | `a` = 偏置 | 无 | 最基础的加性篡改 |
| 完整性 | `SPOOF` | `v = a` | `a` = 目标值 | 无 | 无视 `truth`，直接输出 `a`（假坐标 / 锁转速） |
| 完整性 | `NOISE` | `v = truth + a·N(0,1)` | `a` = σ（标准差） | rng | `a` 是标准差，不是幅度 |
| 完整性 | `SCALING` | `v = truth · a` | `a` = 系数 | 无 | `a = 0` 恒为 0，不是直通 |
| 完整性 | `DRIFT` | `v = truth + a·t` | `a` = 斜率（单位/秒） | 无 | 随时间线性漂移 |
| 完整性 | `OSCILLATION` | `v = truth + a·sin(b·t + c)` | `a`=振幅，`b`=角频率(rad/s)，`c`=相位(rad) | 无 | 正弦振荡叠加 |
| 完整性 | `RANDOM_WALK` | `v = v_last + a·N(0,1)` | `a` = 步长 σ | last + rng | 无界累积，长时间发散 |
| 完整性 | `QUANTIZE` | `v = round(truth/a)·a` | `a` = 量化步长 | 无 | `a ≈ 0` 时直通（防除零） |
| 完整性 | `CLAMP` | `v = clamp(truth, a, b)` | `a` = min，`b` = max | 无 | 要求 `a ≤ b` |
| 可用性 | `FREEZE` | `v = v_last` | 无 | last | 冻结在上次通过的值；无历史时退化为 `truth` |
| 可用性 | `DROP` | 概率 `a` 丢弃本条 | `a` = 丢包概率 [0,1] | rng | `a` 不建议大于 0.9（§8） |
| 时间序列 | `DELAY` | `out(t) = in(t − a)` | `a` = 延迟秒 | 历史 | 真实但滞后 `a` 秒 |
| 时间序列 | `REPLAY` | `out(t) = seg[(t − start) mod a]` | `a` = 片段长度秒 | 历史 + 快照 | 循环回放开始前 `a` 秒 |

> **注**：三类攻击的定位——完整性篡改数值（飞控读到错的值）；可用性破坏数据流（飞控读不到或读到旧值）；时间序列把历史数据错位（飞控读到真但过期的值）。`DROP` 产生 `pass == false`，其余原语均 `pass == true`。

---

## 5. 随机

### 5.1 生成

**伪随机数生成器**由三个确定性函数构成：

| 函数 | 作用 | 说明 |
|---|---|---|
| `xorshift32(state)` | 32 位 PRNG，原地推进 `state` | 状态为 0 时用固定种子 `0x9E3779B9` 自愈 |
| `uniform(state)` | `[0, 1)` 均匀分布 | 取高 24 位 / 2²⁴ |
| `gaussian(state)` | 标准正态 N(0,1) | Box-Muller 变换；`u1 < 1e-12` 时钳到 `1e-12` 防 `log(0)` |

### 5.2 种子

`rng_state` 由管理器 `set()` 播种（`default_seed` 公式见管理器分册 §5.2）：

```cpp
state.rng_state = (spec.seed != 0) ? spec.seed : default_seed(idx);
```

| `seed` | 行为 | 可复现 |
|---|---|---|
| `0`（默认） | 每通道确定性默认种子 `default_seed(idx)`（管理器分册 §5.2） | 是 |
| 非 0 | 直接作为 PRNG 种子 | 同 seed 同序列 |

> **注**：默认逐通道不同的原因——若所有通道共享同一默认种子，两个通道同时用同一随机原语会得到完全相同的噪声序列，例如 GPS 纬度/经度同时 `NOISE` 会漂成一条直线对角线，而非真正的 2D 随机游走。逐通道确定性默认种子（`default_seed(idx)`，管理器分册 §5.2）保证默认即可复现、且通道间不相关。

只有 3 种原语用到 `rng_state`：`NOISE`、`RANDOM_WALK`、`DROP`；其余 10 种与 `seed` 无关。

---

## 6. 时序

### 6.1 机制

其余 11 种是**点变换**——输出只依赖当前这一个 `truth`；`DELAY` / `REPLAY` 是**时间序列变换**——输出依赖过去一段时间的数据，因此 `ChannelState` 额外维护两块：

1. 历史环形缓冲（`hist_*`）：每次 `apply()` 无条件记录干净 `truth`（预滚），攻击激活时已有过去数据可回看；
2. `REPLAY` 快照（`replay_*`）：从历史里冻结一段出来单独存，避免回放中段被 live 历史覆盖。

### 6.2 延迟

```text
apply() 先记历史 → 已激活 → DELAY 分支：
    target = now − a·1e6
    history_lookup(state, target, v)   // 找「时间戳 ≤ target 的最新一条」
    return v                            // 历史为空时 v 保持 truth（兜底）
```

`history_lookup` 兜底：历史为空 → 返回 `false`（`out` 不动）；`target` 早于最旧采样 → 返回最旧一条。

### 6.3 重放

```text
第一次激活（replay_captured == false）：
    replay_capture() 截取 [start − a, start) 历史段 → 快照，置 captured = true

之后每次：
    phase = (now − start) % (a·1e6)
    在快照里找「offset ≤ phase 的最后一条」→ 输出其值
```

`replay_capture` 的边界约定：段区间 `[start − a, start)` 左闭右开（含 `start − a`、不含 `start`）；`replay_offset[i] = t − (start − a)` 存相对偏移，便于与 `phase` 直接比较。

### 6.4 容量

| 通道 | 采样率 | 4096 条 ≈ |
|---|---|---|
| GPS | ~5 Hz | ~13.6 分钟 |
| 电机 | ~400 Hz | ~10 秒 |

`a` 超出窗口时：`DELAY` 退化为现存最旧值；`REPLAY` 片段截断到最近 4096 条。

---

## 7. 生命周期

Layer A 的状态字段在换攻击时的处置由 Layer C 的 `set()` 决定，精确列表由（管理器分册 §6）给出。核心原则：

> **注**：历史属于通道（数据），不属于攻击（配置）。

- `last_value` / `has_last` / `rng_state` / `replay_*` 是本次攻击的状态，换攻击时重置；
- `hist_*` 是通道的数据流，跨命令持久——换攻击不中断数据流，`DELAY` / `REPLAY` 才有预滚可用。

---

## 8. 限制

| # | 事项 | 说明 |
|---|---|---|
| 1 | `DROP` 的 `a` 不建议大于 0.9 | `a` 是每次独立（IID）丢包概率；`a → 1` 时电机通道退化为 `FREEZE`（丢包后 hold last，几乎每拍都丢）。连续丢 k 拍概率为 `a^k`；电机 400 Hz、40 拍（100 ms）处：`a=0.5` → 10⁻¹³、`a=0.9` → 1.5%、`a=0.99` → 67%。建议 `a ∈ [0.01, 0.9]`；研究持续丢帧需先补 ESC 失步建模（系统设计 §9） |
| 2 | `DELAY` / `REPLAY` 回看受容量限制 | §6.4 |
| 3 | 数值边界 | 见下表 |
| 4 | 时间单位两套 | `now_us` / `start_us` / `end_us` / 历史时间戳是微秒；`DELAY` / `REPLAY` 的 `a` 是秒；`DRIFT` / `OSCILLATION` 的 `t` 由 `elapsed_s()` 转成秒 |

数值边界细表：

| 原语 | 边界情形 | 结果 |
|---|---|---|
| `SCALING` | `a = 0` | 值恒为 0（GPS 归零 / 电机停转），不是直通 |
| `QUANTIZE` | `a = 0`（`fabs(a) ≤ 1e-12`） | 直通（防除零，安全） |
| `CLAMP` | `a > b` | 行为异常（min > max），应保证 `a ≤ b` |
| `NOISE` / `RANDOM_WALK` | `a` 是 σ | 不是幅度；`RANDOM_WALK` 无界，长时间发散 |
| `FREEZE` | 首次激活前无历史 | 退化为当前 `truth` |

---

## 9. 结论

`AttackPrimitive` 是「把一个干净标量篡改成目标值（或丢弃）」的纯数学内核：13 种原语覆盖完整性、可用性、时间序列三类；点变换无状态或只靠 `last_value` / `rng_state`，时间序列靠 `hist_*` 历史环形缓冲与 `replay_*` 快照；确定性 PRNG（xorshift32 + Box-Muller）保证同 seed 可复现；所有状态由调用方传入，库本身零依赖、可独立单测。
