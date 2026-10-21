# 自定义 HID：KWS 模型管理与实时动作配置

本目录包含 PC 与设备通信的实现。USB 描述符位于 `../usb_driver/`，
KWS、音频及灯环模块只保留必要的调用接口。

## 1. 功能与约定

- USB 复合设备：原有 UAC2 音频 + 键盘 HID + 独立厂商控制 HID。
- `class_index=0` 固定为 `silence`，`1` 固定为 `unknown`，两者不能绑定动作。
- `2..class_count-1` 为当前模型的关键词。索引是唯一的关键词句柄；文本只作注释。
- 每次模型更新必须上传完整 `.espdl` 和完整类别表，不支持关键词/模型局部更新。
- 灯效、键盘映射及每个关键词的动作开关独立实时配置，模型包不包含绑定。
- 每次上电/复位均将动作总开关、灯效开关和键盘开关置为 false，包括恢复已保存配置和启动回退；颜色、键码和时长保留。
- UART0 发送 `a/A` 开启全部关键词动作及灯效（有键码才开启键盘），`x/X` 关闭；无需回车。也可用 HID `SET_BINDING` 逐词开启。
- 关键词禁用只屏蔽灯效和键盘，不改变推理、分类竞争、消抖、冷却或主机识别事件。
- 没有全局键盘开关。每个关键词分别具有动作总开关、灯效开关、键盘开关。
- 首版上限为 34 类（2 个背景类 + 32 个关键词）；这不保证所有模型都满足实时性能预算。

文件：

| 文件 | 职责 |
|---|---|
| `custom_protocol.[ch]` | 64 字节帧、小端读写、CRC32、命令与状态码 |
| `custom_control.c` | 命令 FIFO、应答/事件发送、JSON 快照、更新状态机 |
| `custom_hid.c` | TinyUSB HID 回调及键盘/控制接口分发 |
| `custom_state.c` | RAM 类别表和绑定、版本检查、协作暂停屏障 |
| `custom_storage.c` | A/B 槽、完整性验证、配置与活动槽持久化 |
| `tools/kws_control.py` | HIDAPI 参考客户端及独立协议编解码 |
| `tools/kws_filter_gui.py` | KWS 判决参数配置窗口 |

## 2. USB 参数

| 项目 | 值 |
|---|---|
| VID / PID | `0x303A / 0x8003`（原 PID `0x8002` 加一） |
| 控制接口编号 / TinyUSB HID instance | `6 / 1` |
| Usage Page / Usage | `0xFF00 / 0x01` |
| Subclass / Protocol | `0 / 0` |
| Interrupt OUT / IN | `0x05 / 0x85` |
| 报告长度 | Input、Output 均为 64 字节 |
| Report ID | 无 |
| 轮询 | FS `bInterval=1`，HS `bInterval=4`，均为 1 ms |
| 键盘接口 | 接口 5，HID instance 0，IN `0x84`，保持 8 字节报告 |
| 序列号 | 芯片默认 MAC 的 12 位大写十六进制文本 |

PC 必须按 VID/PID + Usage Page/Usage 选择控制接口，不能打开键盘接口。
HIDAPI `write()` 的参数为 **`0x00 + 64 字节帧`**，长度 65；`read()` 返回 64 字节帧。
前置 `0x00` 是主机 API 的无 Report ID 占位，不属于 CRC 或设备帧。

## 3. 帧编码

所有多字节整数使用小端。禁止直接传输本机 C 结构体。

| 偏移 | 长度 | 字段 |
|---:|---:|---|
| 0 | 2 | ASCII `KC`，即 `4B 43` |
| 2 | 1 | 协议版本，固定 1 |
| 3 | 1 | 类型：0 请求、1 应答、2 事件 |
| 4 | 2 | `command` |
| 6 | 2 | `sequence` |
| 8 | 4 | `transaction`：更新事务或读取快照 ID；普通命令为 0 |
| 12 | 4 | `offset`：对象内字节偏移，普通命令为 0 |
| 16 | 2 | `payload_length`，0～40 |
| 18 | 2 | `status`，请求填 0 |
| 20 | 40 | Payload，未使用的尾部必须填 0 |
| 60 | 4 | 前 60 字节的 CRC32 |

CRC 为 CRC-32/ISO-HDLC：反射多项式 `0xEDB88320`，初始值和最终异或均为
`0xFFFFFFFF`；与 Python `zlib.crc32()` 一致。SHA-256 使用原始 32 字节摘要，
不是 64 字节十六进制字符串。

## 4. 调度、应答与重试

USB 回调只复制报告并零等待入队，不能进行 Flash、JSON、模型加载或外设操作。
命令 FIFO 深度 16，控制任务优先级 1，发送任务优先级 2。
命令应答和主动事件使用不同队列，应答优先。

普通命令入队后延迟执行，**没有额外的入队成功应答**；收到 `OK` 才表示执行完成。
`SAVE_CONFIG` 返回 `ACCEPTED`，表示已经固定待保存的配置快照，尚未写入 Flash。
查询、读取类别表、修改绑定只访问 RAM，不暂停、不静音、不重置音频或模型。

HID 操作 UART 日志默认关闭。串口发送 `u/U` 切换并确认 `HID logs ON/OFF`；
状态只保存在 RAM，上电/复位后恢复关闭，`SAVE_CONFIG` 不保存这个调试开关。
开启后在控制任务中输出反馈，包含命令名、序号、状态码、事务及必要参数。
`SET_BINDING` 记录类别索引、flags、RGB、modifier、6 个键码、灯效/按键时长和当前 revision；
`ACTIVATE` 记录目标槽（A/B/embedded），`OBJECT_BEGIN` 记录对象编号和字节数。
`SAVE_CONFIG` 的 `ACCEPTED` 仅表示受理，Flash 保存完成后另输出
`HID SAVE_CONFIG complete status=OK(0) ...`；失败输出 `FLASH_ERROR`。
相同帧重试的操作日志标记 `replay=1`，不代表重复执行或再次保存。
成功的 `READ_CHUNK`、`READ_END`、`OBJECT_DATA` 不逐帧打印，错误仍输出，避免大量分块刷屏。
开启后的反馈不受 `l/L` 普通日志开关或 USB 开流自动静默影响，但遵守 Capture2 UART 隔离实验开关。
关闭 HID 串口日志也不会影响 HID 应答、错误状态、保存完成事件或命令执行；不补发关闭期间的历史日志。
USB 接收回调仍只入队，不格式化或输出串口；这些反馈不会从音频/KWS 推理任务发送。

PC 使用逐请求确认：同一时刻只等待一个命令；序号逐次递增，16 位自然回绕。
每次打开 HID 句柄后，先发送 `OPEN_SESSION=0x0004`：Payload 为主机随机生成的
16 字节会话 nonce，transaction 和 offset 为 0；成功应答原样回显 nonce。
相同 nonce 重试不会重置会话；不同 nonce 清除旧重放历史、读取快照和事件订阅，
不修改模型、绑定或音频状态。更新事务进行中拒绝其他会话，返回 `BUSY`。
主机必须校验 nonce 回显，避免接收上一进程遗留的握手应答。参考客户端自动握手。
超时重发完全相同的帧，不能更改序号或数据。建议每次超时 15 秒，最多 3 次；
任务繁忙、首次擦除、模型加载可能比普通命令慢。

设备缓存当前会话最近 16 个已执行请求及应答。相同序号和相同帧返回原应答；
相同序号而内容不同返回 `CONFLICT`。因此重复写块、提交、激活不会再次执行。
缓存范围以外的旧帧不能重放；主机不要同时运行两个写客户端。

FIFO 满时尽力应答 `QUEUE_FULL`；应答队列也满时主机会超时，需要原帧重试。
USB 重新挂载/断线会清除重放历史、读取快照和订阅，更新事务自动中止。
设备重启以 `boot_id` 变化识别。关闭 PC 进程未必触发 USB 断线，更新事务另有
60 秒无操作超时，会自动退出维护状态；不支持断电续传。

## 5. 信息读取：冻结的 UTF-8 JSON 快照

读取对象编号：

| 编号 | 对象 | 主要内容 |
|---:|---|---|
| 1 | info | 唯一设备 ID、固件版本、项目名、协议、VID/PID |
| 2 | caps | 类别/模型/清单/标签容量，命令队列、键盘按键上限，前端 ABI，灯环及运行时 KWS 参数能力 |
| 3 | status | 当前模型、槽状态、KWS 运行状态、判决参数、音频接口、配置及更新进度 |
| 4 | classes | 当前模型完整类别索引与文本，包括 0、1 |
| 5 | bindings | 当前模型的完整实时动作绑定，不包括背景类 |
| 6 | stats | 推理帧数、耗时、错误、重置、丢样本、命令/事件丢弃计数 |

每份快照包含 `boot_id`、`model_epoch`、`config_revision`。
`model_epoch` 在模型选择/重载及进入更新维护状态时变化，用来隔离旧识别上下文和旧动作。
`class_count` 包含背景类，`keyword_count=class_count-2`，
`enabled_keyword_count` 只统计动作总开关打开的关键词。
类别表与绑定对象携带当前模型包的 `package_sha256`。

| 命令 | 码 | 请求 | 成功应答 |
|---|---:|---|---|
| GET_INFO | `0x0001` | 空 | 开始对象 1 快照 |
| GET_CAPS | `0x0002` | 空 | 开始对象 2 快照 |
| GET_STATUS | `0x0003` | 空 | 开始对象 3 快照 |
| READ_BEGIN | `0x0010` | `object:u16` | 开始指定对象快照 |
| READ_CHUNK | `0x0011` | 空；帧中填快照 ID、offset | 最多 40 字节 JSON 原始字节 |
| READ_END | `0x0012` | 空；帧中填快照 ID | 释放快照 |

开始快照的应答在帧 `transaction` 返回新快照 ID，Payload 为：
`total_bytes:u32, boot_id:u32, model_epoch:u32, config_revision:u32`。
从 offset 0 连续读取至 `total_bytes`，然后 UTF-8 解码并解析 JSON。
UTF-8 字符可能跨帧，不能逐帧解码。offset 恰好等于总长度返回空块，超过总长度报错。
同一连接只保留一份快照，新 READ_BEGIN/GET_* 会替换上一份。
快照内容冻结，后续模型变化不会改变正在读取的字节；写配置仍需版本校验。

`status` 的核心结构示例：

```json
{
  "boot_id": 1234,
  "model_epoch": 3,
  "config_revision": 12,
  "active_model": {
    "source": "slot_a",
    "package_sha256": "64位十六进制摘要",
    "model_bytes": 138064,
    "class_count": 8,
    "keyword_count": 6,
    "enabled_keyword_count": 4
  },
  "runtime": {"state": "ready", "last_error": 0, "config_dirty": true, "save_pending": false},
  "kws_filter": {"revision": 1, "keyword_threshold_percent": 80, "cooldown_ms": 500, "stable_frames": 3, "need_vad_checked": false},
  "audio": {"stream_mask": 6, "capture_output_mode": "normal", "processing_state": "running"},
  "model_update": {"state": 0, "transaction": 0, "target_slot": 0, "received_bytes": 0, "total_bytes": 0, "last_error": 0},
  "slots": [{"slot": 0, "state": "valid", "selected": true}],
  "queued_commands": 0,
  "dropped_commands": 0,
  "dropped_events": 0
}
```

`stream_mask`：bit0 播放、bit1 原始录音、bit2 Capture2。
`runtime.state`：`stopped/loading/warming_up/ready/error`。
`model_update.state`：0 空闲、1 接收中、2 已提交候选包、3 失败/中止。
槽为 `valid/receiving/empty_or_invalid`。`valid` 表示存储完整，不表示已经加载或通过推理自检。
维护期 `capture_output_mode=maintenance_silence`，处理状态显示 `paused`（包括暂停过渡）。
更新进度字节数是当前上传对象的进度，不是两个对象的合计。
`last_error` 在 runtime 中是 ESP-IDF 错误码，在 model_update 中是本协议状态码。

## 6. 实时绑定

`SET_BINDING=0x0020`，transaction=0，Payload 严格 29 字节：

| Payload 偏移 | 类型 | 含义 |
|---:|---|---|
| 0 | u32 | boot_id |
| 4 | u32 | model_epoch |
| 8 | u32 | expected_config_revision |
| 12 | u16 | class_index，必须 >=2 |
| 14 | u8 | bit0 actions_enabled；bit1 led.enabled；bit2 keyboard.enabled；其余为 0 |
| 15 | 3×u8 | R、G、B |
| 18 | u8 | HID modifier 位掩码 |
| 19 | 6×u8 | 同时按下的 HID keyboard usages，空位为 0 |
| 25 | u16 | 灯效持续 ms，0 使用灯环默认值，上限 60000 |
| 27 | u16 | 按键保持 ms，0 表示按下传输完成后立即释放，上限 1000 |

一次命令整体替换一个类别的配置，成功 Payload 返回新的 `config_revision:u32`。
boot/epoch 不符返回 `MODEL_CHANGED`；revision 不符返回 `CONFLICT`，重新读取后再改。
键值只能是 0 或 `0x04..0xDF`，不能重复；键盘启用时至少有一个非零键值。
modifier 使用标准 HID modifier 位，字符含义取决于主机键盘布局。
首版灯效为现有灯环 `flash` 提示，尚不支持下发任意灯效程序。

实际触发条件分别为：`actions_enabled && led.enabled`、
`actions_enabled && keyboard.enabled`。改绑定不重置模型、消抖和冷却。
该关键词旧版待执行动作失效，其他关键词的待执行动作不受这次编辑影响；
重新启用不补发历史事件，已经开始的灯效可以结束，
已按下按键必须释放。键盘驱动在断线后优先补发全零释放，再允许新按键。
若等待键盘释放屏障超时返回 BUSY，RAM 配置可能已经更新；需读取 revision 确认，
不能假定 BUSY 一定没有修改。

`SAVE_CONFIG=0x0021`，Payload：`boot_id:u32, model_epoch:u32, expected_revision:u32`。
返回 ACCEPTED 后由优先级 1 的控制任务写 Flash，低于音频处理任务。
播放、原始录音和 Capture2 接口开启均不阻塞保存；不再等待音频接口关闭。
不为保存配置主动静音、暂停或重置音频处理。串口保存日志显示
`save_priority` 和 `stream_mask`，例如三路均开启时为 `save_priority=1 stream_mask=0x07`。
保存的是请求执行时固定的快照，之后的实时编辑会继续保持 dirty。
完成后发送 SAVE 事件，也可轮询 `save_pending/config_dirty`。保存失败保留 RAM 配置。
待保存时不允许开始模型更新或激活，避免保存到错误模型。

串口 `a/A`、`x/X` 与 HID 命令串行执行，不自动保存；保存过的开启状态也不会在下次启动自动启用。
若启动时屏蔽了保存配置中的开启标志，RAM 与 Flash 不同，`config_dirty` 为 true。
串口关闭使旧版排队动作失效并等待按键释放；无键码的关键词只开启灯效，键盘保持禁用。
模型更新进行中串口开关返回 BUSY，命令队列满时提示重试；识别与事件不受动作开关影响。

### KWS 判决参数（运行中可改并立即持久化）

`status.kws_filter` 返回当前有效值及独立的 `revision`。`caps.supports_runtime_kws_filter=true` 表示支持此命令。
`SET_KWS_FILTER=0x0023`，transaction=0、offset=0，Payload 严格 17 字节，所有整数小端：

| Payload 偏移 | 类型 | 字段与范围 |
|---:|---|---|
| 0 | u32 | `boot_id`，来自 status |
| 4 | u32 | `expected_filter_revision`，来自 `status.kws_filter.revision` |
| 8 | u16 | `keyword_threshold_percent`：0～100 |
| 10 | u32 | `cooldown_ms`：1～60000 |
| 14 | u16 | `stable_frames`：1～1000，每帧 10 ms |
| 16 | u8 | `need_vad_checked`：0=false，1=true |

成功应答为 `OK`，Payload `filter_revision:u32`；此时新值已写入 `kws_cfg` NVS 并应用于下一次 KWS 判决。
相同值不重复写 Flash。断电重启后读取保存值；若无有效保存值，使用 menuconfig 编译默认值。
与模型槽及动作绑定分别保存，切换模型不清除这些参数。更改参数时清除当前候选确认计数，
预热进度、同词去重及已开始的冷却继续保持。写入失败返回 `FLASH_ERROR`，运行值不改变；
旧 boot 返回 `MODEL_CHANGED`，旧 filter revision 返回 `CONFLICT`，非法值返回 `OUT_OF_RANGE`，
模型更新事务中返回 `BUSY`。HID 重试沿用同帧同序号的重放规则。
此命令不使用 `SAVE_CONFIG`，后者只保存动作绑定；`config_dirty` 也只反映动作绑定。

Windows 上位机界面：

```powershell
python -m pip install hidapi
python src/custom/tools/kws_filter_gui.py
# 多设备时：python src/custom/tools/kws_filter_gui.py --serial 设备序列号
```

界面启动时读取设备当前值，点击“Save to device”后在写入并读回成功时显示 `Saved to NVS`。
命令行也可使用 `python src/custom/tools/kws_control.py filter 80 500 3 false`。

## 7. 模型更新包

清单为无 BOM、无 NUL 的 UTF-8 JSON，最大 8192 字节，不包含动作绑定：

```json
{
  "format_version": 1,
  "frontend_profile": "mfcc40_16k_int16_state6_v1",
  "classes": [
    {"class_index": 0, "text": "silence"},
    {"class_index": 1, "text": "unknown"},
    {"class_index": 2, "text": "开灯"}
  ]
}
```

classes 必须按连续索引从 0 排到 C-1，C 为 3～34；文本非空、最长 64 个 UTF-8 字节。
背景类文本必须严格为 `silence` 和 `unknown`。仅变更文本不会改变识别能力。
模型必须是未加密、单模型 **EDL2** `.espdl`，最大 960 KiB；不接受 PDL 多模型包。
前端/模型 ABI 固定为 16 kHz、40 维 MFCC、INT16 输入输出、现有 6 对状态张量和名称，
算子必须已注册。类别表长度必须与真实 logits 元素数一致。
改变前端、状态接口或所需算子需要固件升级。

整体包身份为 `SHA256(原始 espdl 字节 || 原始 manifest 字节)`。
清单的空白、顺序等字节变化也会改变包身份；不要在传输过程中重新格式化清单。

| 命令 | 码 | 请求 Payload | 成功应答 |
|---|---:|---|---|
| UPDATE_BEGIN | `0x0030` | 空，transaction=0 | 帧返回事务 ID，Payload `target_slot:u8` |
| OBJECT_BEGIN | `0x0031` | `object:u16, bytes:u32, sha256:32 bytes` | 空 |
| OBJECT_DATA | `0x0032` | 1～40 字节；帧 offset 为对象偏移 | `next_expected_offset:u32` |
| OBJECT_END | `0x0033` | 空 | 校验完成 |
| UPDATE_COMMIT | `0x0034` | 空 | 候选包持久化完成，尚未激活 |
| UPDATE_ABORT | `0x0035` | 空 | 中止并恢复音频 |
| ACTIVATE | `0x0036` | transaction=0，`slot:u8`：0=A、1=B、255=内嵌兜底 | 加载、自检及活动记录保存完成 |

除了 BEGIN/ACTIVATE，更新命令必须携带本次事务 ID。
顺序：UPDATE_BEGIN → manifest OBJECT_BEGIN(0x100)/DATA/END →
model OBJECT_BEGIN(0x101)/DATA/END → UPDATE_COMMIT → 可选 ACTIVATE。
不允许跳过对象、在对象未完成时提交、覆盖活动槽或写任意 Flash 地址。
DATA 必须顺序传输，最后一个块可小于 40 字节。
OBJECT_BEGIN 中的长度不含帧头、填充、CRC；SHA256 只覆盖对象的实际字节。
校验失败时建议中止整个事务并重来。

下载完成不需要整机 reset。ACTIVATE 暂停处理任务，释放旧模型、加载候选模型，
检查真实 I/O、量化类型、状态配对，执行 3 帧零输入冒烟测试并重置状态。
成功后保存完整活动模型/配置记录，恢复处理并重新预热；预热中不触发动作。
失败重新加载原模型及 RAM 配置。激活当前已选槽返回 INVALID_STATE。

新模型包的绑定默认全部禁用。若目标槽保存过**同一整体包摘要**的配置，
回滚/重新选择该模型时恢复匹配配置；不会按相同索引或相同文本跨模型继承。
内嵌兜底模型首次使用保留原 6 个工厂快捷键和颜色，但动作、灯效和键盘开关均默认禁用。

## 8. 音频维护与 Flash 并发

UPDATE_BEGIN 开始录音静音，两个录音端点持续按名义采样率发送零 PCM；
接口、采样率、USB 枚举不变。DSP、AFE feed/fetch、NS、DOA、KWS、原始采集任务
在安全边界协作暂停。播放仍接收、消费并输出静音，反馈端点继续工作。
暂停等待最多 3 秒，超时不会开始擦写。

提交、中止、超时或断线后清除积压音频与状态，恢复任务。
USB 录音在 FIFO 重新达到启动水位之前继续发零包，避免恢复时发送空包。
Capture2 在维护后先积累 96 ms 新数据，以覆盖 AFE 重置后的起始突发和调度抖动；
正常开流预填和连续运行目标仍为 48 ms，恢复后逐步回到该目标。
ACTIVATE 也使用同一维护流程，不改变 USB 配置。

**工程默认启用 `CONFIG_SPIRAM_XIP_FROM_PSRAM=y`。** 当前 ESP-IDF 的 Flash 擦写
会在存在永久 Flash mmap 时关闭缓存，单纯暂停 DSP 无法保障 USB 零包连续性。
因此应用指令/rodata 在启动时迁入 PSRAM；AFE 的 VAD 资产和独立 KWS 模型也复制到
PSRAM 使用，不建立长期 Flash mmap。这样 Flash 擦写路径可保持缓存和 USB 服务运行。
代价是增加 PSRAM 占用，并要求板卡 PSRAM 可用。后续模块不能随意添加长期 Flash mmap。
完整模型包放在 PSRAM 不代表所有运行张量也在 PSRAM。KWS 仍将热点权重复制到内部
SRAM，MFCC history/FFT 显式使用 SRAM；工作张量按总空闲量保留 64 KiB 系统余量、
最大连续块保留 4 KiB 分配余量后计算 SRAM 预算，每次激活重新计算。XIP 不会消除
PSRAM 的 Cache miss 和总线竞争成本，需结合 `kws_model` 内存日志与 KWS 耗时验收。
预算在模型图及张量元数据加载后计算，并在规划前实际分配 SRAM arena，防止规划临时
内存抢占该连续块；最终张量复用 arena，由模型销毁释放。工程通过
`kws/espdl_budget.cmake` 生成局部 ESP-DL 源码副本，为 KWS 启用该路径，其他模型
沿用原预算。若 SRAM arena 分配失败，本次加载改用 PSRAM 并明确记录警告。
既有工程 sdkconfig 若关闭了 XIP，CMake 会报错，需要开启后重编译。
ESP-DL 当前版本在 XIP 模式下会对内部 SRAM 查询外部 MMU 并触发断言。
`kws/kws_memory_compat.cpp` 通过链接包装地址分类函数，先识别 TCM、内部 SRAM
和 PSRAM，再查询其他地址的 MMU；AFE 的预编译库与 KWS 共用此修复。
仍需板上验证擦除期间双录音的 1-ms 包连续性、播放反馈和恢复噪声；主机测试不能替代总线测试。

## 9. 分区与掉电语义

| 分区 | 偏移 | 大小 |
|---|---:|---:|
| 原 AFE `model` | `0x410000` | 512 KiB，保留 |
| `kws_a` | `0x490000` | 1 MiB |
| `kws_b` | `0x590000` | 1 MiB |
| `kws_cfg` | `0x690000` | 64 KiB NVS |

每槽前 960 KiB 存 EDL2 模型，后 64 KiB 中首个扇区存带版本/CRC 的类别元数据与摘要。
`data/spiffs` 只表示分区类型兼容，槽不挂载文件系统。
新事务先擦除候选槽元数据，防止不完整模型被视为有效。
提交完整性通过后才写有效元数据，启动时再次验证模型 SHA256。

NVS 保存两个交替的完整活动快照，包含槽、整体包摘要、全部绑定、递增 generation 和 CRC；
启动选择最新且能匹配完整模型的记录。每槽另存匹配配置供回滚使用。
下载/试加载中掉电不会更改原活动记录；只有新模型加载自检成功后才切换记录。
外部模型无有效记录时使用内嵌兜底，外部模型加载失败也尝试内嵌兜底。
NVS 初始化失败不会自动擦除用户配置，控制查询仍可用，写入报错。

首次安装必须烧录新分区表与应用，例如 `idf.py -p COMx flash`；仅 app-flash 不会添加分区。
常规完整 flash 不写 KWS 槽和 kws_cfg，不自动擦除已有配置；erase-flash 会删除这些数据。

## 10. 主动事件

`SUBSCRIBE=0x0022`，Payload 为一个字节：0 关闭、1 开启关键词事件。
关键词事件 `command=0x8001`、type=2，Payload 31 字节：

```text
boot_id:u32, model_epoch:u32, config_revision:u32, event_sequence:u32,
timestamp_us:u64, class_index:u16, actions_enabled:u8, dropped_events:u32
```

事件反映模型消抖后的识别结果，禁用动作不会停止事件；不承诺灯效或键盘已经成功执行。
队列满时丢弃并计数，不阻塞模型，不要求主机逐条 ACK。
帧 sequence 为事件计数的低 16 位，完整计数在 Payload 中。

SAVE 事件 `command=0x8002`，Payload 为 `model_epoch:u32, saved_revision:u32`，
status 为 OK 或 FLASH_ERROR，不依赖关键词订阅；主机也可通过状态轮询判断。

## 11. 状态码

| 值 | 名称 | 含义 |
|---:|---|---|
| 0 | OK | 执行成功 |
| 1 | ACCEPTED | 配置保存已排队，未持久化 |
| 2 | BUSY | 与维护/保存冲突或暂停/释放屏障超时 |
| 3 | BAD_VERSION | Magic 或协议版本不符 |
| 4 | BAD_LENGTH | 长度、请求类型、保留位或填充不合法 |
| 5 | BAD_CRC | 帧损坏 |
| 6 | INVALID_STATE | 事务/对象顺序错误 |
| 7 | OUT_OF_RANGE | 索引、长度或参数超限 |
| 8 | OFFSET_MISMATCH | 不连续的写偏移 |
| 9 | HASH_MISMATCH | 对象 SHA256 不符 |
| 10 | MODEL_INCOMPATIBLE | 清单/EDL2/前端不兼容 |
| 11 | LOAD_FAILED | 激活或活动配置提交失败，已尝试回滚 |
| 12 | NO_SPACE | 无法分配快照等资源 |
| 13 | FLASH_ERROR | Flash/NVS 失败 |
| 14 | CONFLICT | 序号内容冲突或配置 revision 过期 |
| 15 | NOT_FOUND | 无快照、槽、分区或对象 |
| 16 | MODEL_CHANGED | boot/epoch 过期 |
| 17 | QUEUE_FULL | 命令 FIFO 满 |
| 18 | BAD_COMMAND | 未支持的命令 |

模型语义兼容还需要在激活阶段验证，下载完整性通过不代表模型可以运行。

## 12. PC 参考工具

```powershell
python -m pip install hidapi
python src/custom/tools/kws_control.py get info
python src/custom/tools/kws_control.py get status
python src/custom/tools/kws_control.py get classes
python src/custom/tools/kws_control.py get bindings
python src/custom/tools/kws_control.py bind binding.json
python src/custom/tools/kws_control.py save
python src/custom/tools/kws_control.py download model.espdl manifest.json --activate
python src/custom/tools/kws_control.py activate a
python src/custom/tools/kws_control.py activate factory
```

多台设备添加全局参数 `--serial 设备序列号`。binding.json 是单个类别的完整配置：

```json
{
  "class_index": 2,
  "actions_enabled": true,
  "led": {"enabled": true, "effect": "flash", "rgb": [0,255,0], "duration_ms": 600},
  "keyboard": {"enabled": true, "modifier": 1, "keycodes": [6], "hold_ms": 30}
}
```

上例发送 Left Ctrl + C；停用该词仅需将 actions_enabled 改为 false。
修改后先读取设备状态再用当前 boot/epoch/revision 写入；工具会自动完成这些步骤。

`examples/manifest.json` 与仓库当前的 8 类 `.espdl` 匹配，`examples/binding.json`
演示 class 2 的绿色提示和 Ctrl+C。上传仓库模型后仍需单独写入绑定，不会继承内嵌模型动作。

## 13. 验证

```text
idf.py build
cmake -S src/custom/tests -B build-custom-tests
cmake --build build-custom-tests
ctest --test-dir build-custom-tests --output-on-failure
python src/custom/tests/test_client.py
python src/custom/tests/test_hid_logging.py --cc <主机 C 编译器>
python src/custom/tests/test_audio_maintenance.py --cxx <主机 C++ 编译器>
python src/custom/tests/test_capture_resume.py --cxx <主机 C++ 编译器>
python src/kws/tests/test_memory_compat.py --cxx <主机 C++ 编译器>
python src/tests/test_keyword_actions.py --cc <主机 C 编译器>
```

CMake 主机测试覆盖帧/CRC/非法 UTF-8、模型版本隔离、实时动作开关；
Windows 的 GCC/Clang 还运行真实存储模块的 NOR/NVS 模拟测试，使用 BCrypt SHA256，
覆盖跨 4-KiB 分块、只下载不激活、提交前掉电、活动模型损坏回退及哈希错误。
主机客户端测试不需要 hidapi 或 USB 硬件。
另外应运行现有 KWS、AFE/NS 输入、录音包长、USB DMA/PTI 和日志回归。
上板验收需验证新枚举、真实模型下载/激活、双录音维护零包、断线/掉电恢复和识别性能。

### Windows 实板自动验证

串口用于复位和日志；还需要连接开发板独立的 USB HS 数据口。
`tools/board_validate.py` 使用 WinMM 播放 16-kHz 双声道激励，并以独占 WASAPI 录制 Capture2，
从 `--source` 中选取 yes/no/up/down/left/right 各两段 WAV，循环播放。
设备需启用 USB 播放参考 KWS 输入和 Capture2 的 1-kHz 诊断正弦波。

```powershell
python -m pip install numpy hidapi pyserial sounddevice
python src/custom/tools/board_validate.py smoke --port COM6 --reset --output .codex-tmp/board-smoke
python src/custom/tools/board_validate.py audio --port COM6 --reset --source E:/Sony/mini_speech_commands --output .codex-tmp/board-audio
python src/custom/tools/board_validate.py keyboard --port COM6 --reset --output .codex-tmp/board-keyboard
python src/custom/tools/board_validate.py update --port COM6 --reset --capture-raw --output .codex-tmp/board-update
python src/custom/tools/board_validate.py update --port COM6 --reset --capture-raw --all-logs --steady-seconds 35 --output .codex-tmp/board-performance
python src/custom/tools/board_recovery_validate.py --port COM6 --output .codex-tmp/board-recovery
```

`audio` 对比空闲运行、连续读取信息及实时编辑绑定阶段；`keyboard` 增加 class 2 的
F24 开启、禁用和重新启用测试，低级键盘钩子只截获测试用 F24，并记录按下/释放。
测试临时关闭其他关键词键盘映射，结束后恢复原 RAM 绑定，不调用 SAVE_CONFIG。
`update` 下载仓库模型及完整清单、显式激活，再切回测试前的活动模型。
切回前停止测试语音播放，避免恢复原模型的键盘绑定后发送测试词的快捷键。
`--capture-raw` 另外通过 WDM-KS 采集 8 通道/24-kHz 原始录音，保存为 `capture1.wav`。
`--all-logs` 保留开流期间性能日志；`--steady-seconds` 延长启动、激活和恢复后的
观察阶段，比较完整的 10 秒性能窗口时应排除跨越下载/激活暂停的窗口。
`update --reload-cycles 9` 在播放及双录音持续开启时额外轮换内嵌模型与有效 A/B 槽，
每次激活后观察 4 秒并检查 ready 状态、模型来源与 boot_id。
`update` 会将下载、激活和恢复后的 warming_up → ready 单独记录为恢复阶段；
正常波形观察从 ready 后开始，恢复静音的起止边沿不计作正常流突变。
`maintenance` 验证进入维护再中止；`--quick` 缩短查询及基线阶段，`--all-logs` 打开流内诊断。
恢复测试要求初始使用内嵌模型，并已有一个有效下载槽，会覆盖测试板的候选槽；
验证下载中硬件复位、类别数不兼容时回滚、错误 SHA 拒绝、另一槽成功激活，最后恢复初始模型和 RAM 绑定。
该脚本通过串口 EN/RTS 复位测试，不能替代实际切断电源。
播放及录音接口保持开启时也会保存，写入以低于音频处理的优先级执行。
`--skip-save` 可单独运行模型恢复部分。
验收时应分别检查 ACCEPTED、save_pending 和 config_dirty，确认数据已经落盘。
不要在这些测试运行时同时开启其他控制客户端。

`tools/board_save_validate.py` 用于验证开流保存及复位读回；`--expect-deferred` 仅用于复现旧固件的
播放等待问题。正常模式覆盖播放、Capture2、双录音、三路全开时的连续两次保存，检查完成事件及音频回调，
最后恢复现场映射并复位核对参数及动作默认禁用。先释放串口监视器，并备份现场 status/bindings。
2026-10-20 的实板复现、下载及闭环结果见 [开流保存验证](SAVE_STREAM_VALIDATION.md)。

每次运行保存 UART、JSON 状态/事件/阶段时间和 `capture2.wav`；
正弦波按 1-kHz/16-kHz 二阶递推检查突变，并区分维护静音与正常流阶段。
正常阶段还检查信号幅度，防止把接近全零的录音误判为连续正弦波。
共享 WinMM 录音路径在本机对恒定正弦波施加了抑制，约 0.3 秒后降到接近静音；
独占 WASAPI 实测保留 ±12000 的设备 PCM，验收应使用这一路径。
status 和 stats 的 `usb` 对象同时提供累计录音空包、异常零包、失败包、端点背压、
短写及播放欠载计数，用于交叉检查波形。波形检测无法独立证明 USB 总线上完全没有丢包。
本机实测结果、修复和未覆盖项见 [实板验证记录](HARDWARE_VALIDATION.md)。
后续内存分配修复、前后性能对比和反复激活验证见 [SRAM 优化验证](SRAM_VALIDATION.md)。
