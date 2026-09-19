# 攻击管理器（Layer C）

> 本文档说明 Layer C 的 `AttackManager`：攻击注入的路由中枢，负责把 `attack_command` 解析为每通道攻击配置并发布 `attack_status`。关联文档：（原语分册 §5）（系统设计 §2）。版本 v5 · 状态 已实现 · 代码 `attack/AttackManager.hpp` / `attack/AttackManager.cpp`。

## 1. 定位

`AttackManager` 是攻击注入三层架构中的 Layer C，亦即本系统的**路由中枢**，粘合 Layer A（纯数学）与 Layer B（注入点）。职责：

| 职责 | 说明 |
|---|---|
| 路由表 | 维护 `Channel → AttackSpec + ChannelState` 两张表（每通道一份） |
| 接收指令 | 订阅 uORB `attack_command`，把**相对时间窗**解析成**绝对时间** |
| 应用攻击 | 注入点每次调 `apply()`，查表后交给 Layer A 篡改并写回状态 |
| 发布状态 | 10 Hz 发布 `attack_status`，供 logger 与 ROS2 侧监控 |
| 线程安全 | pthread 互斥锁保护所有表访问 |

## 2. 结构

`AttackManager` 持有两张等长数组与三个句柄：

```cpp
class AttackManager {
    AttackSpec   _specs[kNumChannels]{};   // 配置表：每通道攻击配置（默认全 NONE）
    ChannelState _state[kNumChannels]{};   // 状态表：每通道运行时状态

    uORB::Subscription                 _cmd_sub{ORB_ID(attack_command)};
    uORB::Publication<attack_status_s> _status_pub{ORB_ID(attack_status)};

    pthread_mutex_t _lock{};           // 保护所有表访问
    bool _lock_initialized{false};     // 是否已 init（析构时据此决定是否 destroy）
    uint64_t _last_status_us{0};       // 上次发 status 的 hrt 时间（10 Hz 节流）
};
```

| 成员 | 作用 |
|---|---|
| `_specs[10]` | **配置表**：每通道当前挂载的攻击（`type = NONE` 即直通） |
| `_state[10]` | **状态表**：每通道运行时状态（last / rng / 历史 / 快照） |
| `_cmd_sub` | 订阅 `attack_command`（指令输入） |
| `_status_pub` | 发布 `attack_status`（状态输出） |
| `_lock` | 互斥锁，串行化所有表访问 |
| `_last_status_us` | status 发布的 10 Hz 节流计数器 |

> **注**：`_specs` 是配置（纯值、可序列化、进 ulog），`_state` 是运行时状态（会变、不序列化）；两者一一对应、同一下标，由 `index_of(ch)` 建立 `Channel` 到数组下标的映射。

## 3. 生命周期

| 项 | 说明 |
|---|---|
| 构造 | `= default`，成员各自默认初始化 |
| 析构 | 仅在 `_lock_initialized` 时 `pthread_mutex_destroy(&_lock)` |
| 复制 / 移动 | 显式 `delete`（持有互斥锁与 uORB 句柄，复制会破坏唯一性） |
| `init()` | `pthread_mutex_init` 初始化锁；提前 `_status_pub.advertise()`；成功返回 `true`，锁初始化失败返回 `false` |

> **注**：`init()` 即提前 advertise，让订阅者（logger / DDS 桥）一开始就能看到话题，避免「logger 想订阅、话题还没人发布」的时序问题——这是 logger 稳定记录 `attack_status` 的前提。

## 4. 接口

### 4.1 设置

`set(Channel ch, const AttackSpec &spec)` 挂载或整体替换某通道的攻击：

```cpp
void set(Channel ch, const AttackSpec &spec)
{
    const size_t idx = index_of(ch);
    if (idx >= kNumChannels) { return; }   // 非法通道：直接返回

    pthread_mutex_lock(&_lock);
    _specs[idx] = spec;                    // 整体覆盖旧配置
    // 重置运行时状态（§6）
    ChannelState &state = _state[idx];
    state.last_value = 0.0;
    state.has_last = false;
    state.rng_state = (spec.seed != 0) ? spec.seed : default_seed(idx);
    state.replay_captured = false;
    state.replay_count = 0;
    pthread_mutex_unlock(&_lock);
}
```

### 4.2 清除

`clear(Channel ch)` 移除攻击，等价于 `set(ch, AttackSpec{})`（`type = NONE`，即直通），是「发 `type = NONE` 关闭攻击」的底层实现。

### 4.3 应用

`apply(Channel ch, double &value, uint64_t now_us)` 对传入的干净值应用攻击：

```cpp
bool apply(Channel ch, double &value, uint64_t now_us)
{
    const size_t idx = index_of(ch);
    if (idx >= kNumChannels) { return true; }   // 未知通道：直通不改动

    pthread_mutex_lock(&_lock);
    const AttackResult result = AttackLibrary::apply(value, _specs[idx], now_us, _state[idx]);
    pthread_mutex_unlock(&_lock);

    if (result.pass) { value = result.value; }   // 通过：写回篡改值
    return result.pass;                          // false = 丢弃（DROP）
}
```

| 项 | 说明 |
|---|---|
| 参数 | `value` 为引用（in/out）：进去是干净真值 `truth`，出来是（可能的）篡改值 |
| 返回 | `true` 用 `value`；`false` 丢弃（仅 DROP） |
| 未知通道 | 返回 `true`、不改动 `value`（安全兜底） |
| 加锁范围 | 只包住 `AttackLibrary::apply`；`value` 写回在解锁后（`value` 是注入点局部变量，非共享，无竞态） |

### 4.4 更新

`update()` 是周期驱动，由 `GZBridge::Run()` 每 10 ms 调一次（约 100 Hz）：

```cpp
void update()   // 由 GZBridge::Run() 每 10 ms 调一次（~100 Hz）
{
    const uint64_t now = hrt_absolute_time();

    attack_command_s cmd{};
    if (_cmd_sub.update(&cmd)) { on_command(cmd, now); }      // 有新指令 → 处理

    if (now - _last_status_us >= STATUS_PERIOD_US) {          // 10 Hz 节流
        publish_status(now);
        _last_status_us = now;
    }
}
```

| 步骤 | 说明 |
|---|---|
| 收指令 | `_cmd_sub.update()` 拿最新一条；两次轮询间若有多条，最新者胜 |
| 发状态 | 距上次 ≥ 100 ms（`STATUS_PERIOD_US`）才发，10 Hz |

## 5. 内部

### 5.1 解析

`make_spec(cmd, now_us)` 把指令解析为 `AttackSpec`，将相对时间窗转为绝对时间：

```cpp
AttackSpec spec{};
spec.type     = static_cast<PrimitiveType>(cmd.type);
spec.a        = cmd.param[0];   spec.b = cmd.param[1];
spec.c        = cmd.param[2];   spec.d = cmd.param[3];
spec.seed     = cmd.seed;

spec.start_us = now_us + cmd.t0_us;                                  // 相对延迟 → 绝对起点
spec.end_us   = (cmd.t1_us != 0) ? spec.start_us + cmd.t1_us : 0;   // 相对时长 → 绝对终点
return spec;
```

> **注**：ROS2 侧消息用相对偏移（`t0_us` 延迟、`t1_us` 时长），管理器在收到指令当下解析成绝对 hrt 时间（`start_us` / `end_us`），由 Layer A 直接比较。故「延迟 1 秒开始」发 `t0_us: 1000000` 即可，ROS2 侧无需自算绝对时间。

### 5.2 播种

`default_seed(idx)` 给出每通道确定性默认种子：

```cpp
uint32_t default_seed(size_t idx)
{
    constexpr uint32_t kBase = 0x9E3779B9u;
    return kBase + static_cast<uint32_t>(idx);
}
```

`seed == 0` 时调用（§6）。默认种子可复现且逐通道不同，避免多通道共享同一条噪声序列（原语分册 §5）。

### 5.3 发布

`publish_status(now_us)` 快照当前配置表并发布：

```cpp
attack_status_s status{};
status.timestamp = now_us;
for (size_t ch = 0; ch < kNumChannels; ch++) {
    status.active[ch]  = AttackLibrary::active(_specs[ch], now_us) ? 1 : 0;
    status.type[ch]    = static_cast<uint8_t>(_specs[ch].type);
    status.param_a[ch] = _specs[ch].a;   status.param_b[ch] = _specs[ch].b;
    status.param_c[ch] = _specs[ch].c;   status.param_d[ch] = _specs[ch].d;
}
_status_pub.publish(status);
```

> **注**：`active` 调 `AttackLibrary::active()` 判时间窗，故能正确反映「命令已到但还没到 `start_us`」的中间态。

## 6. 状态

换新攻击时 `set()` 对 `ChannelState` 的处置：

| 字段 | 处置 | 原因 |
|---|---|---|
| `last_value` | 重置为 0 | 本次攻击的状态 |
| `has_last` | 重置为 false | 同上 |
| `rng_state` | 重新播种：`seed != 0 ? seed : default_seed(idx)` | 保证随机原语可复现 |
| `replay_captured` | 重置为 false | 新攻击需重新截取 REPLAY 快照 |
| `replay_count` | 重置为 0 | 同上 |
| `hist_*`（历史环形缓冲） | 不清空 | 历史是通道的数据流，跨命令持久 |

种子规则：

| `spec.seed` | `rng_state` | 可复现 |
|---|---|---|
| `0`（默认） | `default_seed(idx)`，每通道固定 | 是 |
| 非零 | 直接用该值 | 是，同 seed 同序列 |

> **注**：种子只影响 3 种随机原语（`NOISE` / `RANDOM_WALK` / `DROP`），其余 10 种与 seed 无关。

## 7. 线程

GPS 注入跑在 gz-transport 回调线程，电机注入与指令处理跑在 rate_ctrl 工作队列，故所有表访问由 pthread 互斥锁保护。

| 方法 | 加锁 |
|---|---|
| `set` / `clear` | `pthread_mutex_lock` / `pthread_mutex_unlock` |
| `apply` | 包住 `AttackLibrary::apply` |
| `publish_status` | 包住遍历 `_specs` 的循环 |
| `on_command` | 经 `set` 间接加锁 |

## 8. 限制

| # | 事项 | 说明 |
|---|---|---|
| 1 | 最新者胜 | `update()` 只取最新一条指令；两次轮询间（约 10 ms）同通道多条指令只有最后一条生效。要顺序执行多步攻击，需在 ROS2 侧按时间错开。 |
| 2 | 时间窗半开 | `active` 判 `now ∈ [start_us, end_us)`：含起点、不含终点。`t1_us` 刚到终点那一刻攻击已结束。 |
| 3 | 快照精度 | `attack_status` 只能还原攻击窗口级时间线，不能逐拍还原丢了哪条消息；逐拍随机事件靠 seed 重放复现。 |
| 4 | 越界兜底 | `apply()` 对越界通道返回 `true`（直通）；`on_command()` 对越界 channel 直接丢弃。两者保证越界不崩溃、不污染合法通道。 |

## 9. 结论

`AttackManager` 是攻击注入的路由中枢：持有配置表 `_specs` 与状态表 `_state`，订阅 `attack_command` 把 ROS2 的相对时间窗解析成绝对 hrt 时间后整体覆盖到对应通道；注入点每次 `apply()` 查表交给 Layer A 篡改，10 Hz 发布 `attack_status` 供日志与监控；所有表访问由互斥锁串行化。它是三层架构中把纯数学（A）与物理注入点（B）粘合起来的那一层。
