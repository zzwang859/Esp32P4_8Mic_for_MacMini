# ESP32-P4 USB Audio HS

每次上电或复位后，关键词灯效和按键均默认禁用，即使此前保存过开启状态。
串口发送 `a/A` 开启全部关键词动作，`x/X` 关闭；也可通过控制 HID 逐词开启。
开启后，关键词确认会按绑定颜色闪烁灯环 0.6 秒，并发送绑定快捷键：预置
`yes/no/up/down/left/right → y/n/u/d/l/r`，静音和 unknown 不触发动作。
颜色、组合键和每个词的动作开关可通过新增控制 HID 实时配置。
HID 操作串口日志默认关闭，发送 `u/U` 切换；开启后包含绑定修改、保存完成和模型切换结果，独立于普通日志开关。
协议、完整模型更新和 PC 工具见 [`src/custom/README.md`](src/custom/README.md)。

完整 ESP-IDF 工程。应用源码和 menuconfig 定义位于 [`src`](src/README.md)。

USB 按双通道处理后录音优先的顺序枚举，随后为播放与原始八通道录音；
接口表及 Linux 检查方法见 [USB 接口顺序与 Linux 录音](src/README.md#usb-接口顺序与-linux-录音)。

## 串口命令汇总

UART0：115200 bit/s，TX GPIO37、RX GPIO38。直接发送单个字符，无需回车；表中大小写别名等效。

| 字符 | 操作及默认状态 |
|---|---|
| `0` / `i` / `I` | 灯环默认模式 0，启动时使用 |
| `1` / `c` / `C` | 灯环对话模式 1 |
| `2` / `k` / `K` | 手动触发关键词灯效测试，RGB(191,64,223)，使用默认持续时间 |
| `4` / `f` / `F` | 灯环故障模式 4 |
| `6` | 键盘测试：发送 Ctrl+C 并释放 |
| `7` | 键盘测试：发送 Ctrl+V 并释放 |
| `8` | 键盘测试：发送小写字母 a 并释放 |
| `a` / `A` | 开启全部关键词动作及灯效，有键码的关键词同时开启键盘 |
| `x` / `X` | 关闭全部关键词动作、灯效和键盘；每次上电/复位默认关闭 |
| `u` / `U` | 切换 HID 操作串口日志；默认关闭，重启恢复关闭，确认 `HID logs ON/OFF` |
| `l` / `L` | 切换普通运行日志；首次关闭，再次打开，重启恢复 menuconfig 默认策略 |
| `p` / `P` | 切换每个 10-ms KWS 推理帧的原始 logits CSV；默认关闭，重启不保留 |
| `e` / `E` | 打开 DOA 结果日志，仍受普通日志开关控制 |
| `d` / `D` | 关闭 DOA 结果日志，默认关闭 |
| `?` / `h` / `H` | 打印命令帮助，普通日志关闭时先用 `l/L` 打开 |

`u/U`、`p/P` 与 `l/L` 相互独立，不控制关键词识别；HID 日志关闭也不会影响上位机应答、保存或模型操作。
开关确认不受普通日志关闭影响，但 Capture2 UART 隔离实验会抑制全部串口输出。
灯环模式、`6/7/8` 测试和帮助依赖 `CONFIG_WS2812_LED_RING_ENABLE`；其余命令也支持不编译灯环的音频控制台。
空格、Tab 和换行无需发送，灯环控制台会忽略它们。完整说明见 [串口命令与控制台](src/README.md#串口命令与控制台)。

KWS 默认接入真实 AFE 经 NsCore 降噪后的输出，也可在 menuconfig → `AFE keyword recognition`
→ `KWS input source` 选择进入 AEC 参考端的单声道 USB 播放流。
推理代码和可替换模型统一位于 [`src/kws`](src/kws/README.md)。
支持关键词消抖、后台串口结果和 RTF 统计；权重替换、性能日志策略见该目录说明。
UART0 发送单字符 `l` / `L` 可切换非 KWS 日志：首次关闭普通信息、警告、错误及 SDK 标准输出，
仅保留 KWS 识别结果；再次发送打开普通日志（覆盖 USB 开流自动静默）。每次切换都会显示一条 ON/OFF 确认。
无需回车，重启后恢复默认策略。
串口 `p` / `P` 独立切换每个 10-ms 推理帧的原始 logits，默认关闭；紧凑 CSV 为
`KL,seq,sil,unk,yes,no,up,down,left,right`，浮点固定显示2位小数，不经过 softmax/消抖。
建议先用 `l` 关闭普通日志，再用 `p` 打开原始帧；该追踪不受 `l` 或 USB 静默影响。
推理只复制数据零等待入队，格式化/串口发送在后台。`KD,n` 表示本次追踪累计丢失 n 帧，
仅指调试输出丢失，不阻塞音频或模型。再次 `p` 关闭并显示发送/丢失计数。
KWS 使用全部当前模型类别的 argmax 和 softmax，1 秒预热；默认同一关键词连续 3 帧且每帧概率达到 80% 后触发，
配合 500-ms 全局冷却及同词去重。menuconfig 顶层 `KWS-Filter` 可配置门限、冷却和连续帧数。
新增 `Need-VAD-Checked` 的 `No` / `Yes` 选项，默认 `No` 保持原判决；`Yes` 要求连续确认帧期间 VAD 全部为高，
中途低/未知则重新计数。menuconfig 值作为编译默认值；四项均可通过
`python src/custom/tools/kws_filter_gui.py` 运行中修改并立即保存到 NVS，协议见
[自定义 HID 说明](src/custom/README.md#kws-判决参数运行中可改并立即持久化)，详细时序见 [KWS 说明](src/kws/README.md#消抖和串口结果)。

目标是 ESP32-P4 USB 2.0 High-Speed 端口上的 UAC 2.0 声卡：8 通道
24 kHz/16-bit 原始录音、双通道 16 kHz/16-bit 音频与 VAD 录音、双通道
16 kHz/16-bit 放音、异步反馈，以及可配置的 TDM/I2S 后端。播放音量和静音由 PC 端处理。

片上独立 Full-Speed USB 接口保持默认 PHY 映射，固件不再交换 GPIO24/25 与
GPIO26/27 的 FS PHY。

编译前请运行 `idf.py menuconfig` 配置实际板级 I2S GPIO；未配置时固件会安全停止。

Capture2 枚举为 16 kHz/16-bit 双声道，左声道默认输出 Espressif AFE 再经 NsCore 降噪后的音频。
AFE 启用 AEC、VADNet `vadnet1_medium` 和 AGC，AFE 自带 NS 保持关闭；新增 NsCore 独立运行在 core1、优先级 4。
menuconfig → `Microphone array processing` → `Post-AFE NsCore minimum gain (%, lower = stronger; 100 = bypass)`
配置原算法的 `denoiseBound`，默认 25%（0.25），越小降噪越强，100% 为精确 PCM 旁路。
右声道默认输出同步 VAD 标志，也可选择复制左声道或输出平滑 DOA；元数据跟随 NS 新增的 96 点/6 ms 名义延迟。
移植说明见 [`nscore`](src/audio_processing/nscore/README.md)。USB PID 为 `0x8003`（新增控制 HID 后由 `0x8002` 加 1）。
AFE 使用单麦克风：`MR` 为一路 DAS 麦克风加一路 AEC 播放参考，AFE 输出为单声道。
VAD 标志可通过固件接口 `audio_pipeline_get_vad_state()` 读取，不对连续音频或 KWS 输入做门控。
数据路径和诊断模式见 [应用组件说明](src/README.md)。

VADNet 从 `model` 分区加载（`0x410000`，512 KiB），构建生成 `build/srmodels/srmodels.bin`。
首次升级需运行 `idf.py -p <串口> flash`，同时写入分区表、应用和模型；只烧录应用会缺少模型。
启动日志应显示 `microphone=1 reference=1 (MR), output=1; VAD model=vadnet1_medium`。

## Capture2 录音模式配置

四种模式均通过配置切换，无需修改代码。运行 `idf.py menuconfig`，进入
`Microphone array processing -> Capture2 left channel source`：

| 模式 | 菜单选项 | 配置项 |
| --- | --- | --- |
| AEC 直接输出 | Espressif AFE output | `CONFIG_ARRAY_CAPTURE2_AEC_OUTPUT` |
| 信号发生器录音测试（1 kHz 正弦波） | Replace Capture2 AEC output with a 1-kHz sine table | `CONFIG_ARRAY_CAPTURE2_SINE_TEST` |
| NsCore 后正弦波对照（保持正常输出任务及元数据） | Replace Capture2 audio after NsCore with a 1-kHz sine table | `CONFIG_ARRAY_CAPTURE2_POST_NS_SINE_TEST` |
| USB 播放回环录音测试 | Loop USB playback reference back to Capture2 | `CONFIG_ARRAY_CAPTURE2_PLAYBACK_TEST` |

四项互斥，只能选择一种；工程默认配置为 `CONFIG_ARRAY_CAPTURE2_AEC_OUTPUT=y`。
四个选项只决定左声道内容，逐帧按 `L,R` 交错发送。右声道可在同一菜单的
`Capture2 right channel source` 独立选择：

| 右声道选项 | 输出 |
| --- | --- |
| Copy left channel audio | 逐样本复制所选左声道 |
| VAD flag（默认） | 对应音频块的 VAD 标志 |
| Smoothed DOA used by DAS | DAS 当前使用的平滑角度，PCM16=`round(角度×100−18000)` |

DOA 解码为 `(PCM16 + 18000) / 100.0` 度，0°/180° 分别对应 −18000/0。
正常录音的 DOA 在 AFE 输出入降噪队列时取快照，跟随 NS 延迟后与左声道一起进入 FIFO；不回溯补偿 AFE 内部延迟。
VAD 模式下：
语音 `+1` 编码为 `+32767`（正满幅），非语音 `-1` 编码为 `-32768`（负满幅），未知为 `0`。
正常 AFE 输出模式下，同一 fetch 帧的音频与 VAD 先进入降噪队列，降噪并补偿 NS 延迟后成对入 FIFO，随 USB 缓冲、分包一起移动。
标志保留 VAD 判定延迟（默认最短语音 128 ms、最短静音 1000 ms，可在 menuconfig 的 `Microphone array processing` 菜单调整），不回溯修改前面的音频。
两种诊断源仍保持原左声道数据路径；VAD 右声道记录生成该音频块时的最新麦克风 VAD。
新增 NsCore 后正弦波对照使用正常 NS 发布任务、160 点节奏及延迟后的元数据，
KWS 仍接收真实配置输入；用于排查正常录音特有的单点双声道零值，
诊断计数和对照方法见 [应用组件说明](src/README.md#单点双声道零值的后续排查)。
排查 core1 打印干扰时，可在 `ESP32-P4 USB audio` 开启
`CONFIG_AUDIO_DIAGNOSTICS_CAPTURE2_SUSPEND_UART` 做对照：录音接口开启期间暂停全部普通
串口输出（含 KWS 结果、logits 和命令确认），关闭后恢复；默认关闭，识别仍持续运行。
DOA 使用 SRP-PHAT 为 DAS 提供方位，仅在 VAD=1 时沿最短角路径平滑转向，默认限速 30°/秒；静音或未知状态冻结当前方位。速度可在 menuconfig 的 `Microphone array processing` 菜单配置，通过既有固件接口与串口日志读取平滑后的方位。
保存配置后运行 `idf.py build` 重新编译，再运行 `idf.py -p <串口> flash` 烧录生效。
这些选项属于编译时配置，暂不支持运行时切换。选择 `Espressif AFE output` 可恢复正常 AEC 录音。

## Capture2 录音噪点修复

排查中将 AEC 输出临时替换为连续正弦波，确认 DSP 源数据和 FIFO 正常。
随机整包缺失对应 DWC2 等时 IN 传输失败；单独启用 USB DMA 仍有失败，
工程随后加入构建时 TinyUSB DMA/PTI 补丁，随机缺包消失。剩余噪点与
实时 DSP 任务每 10 秒打印四行串口统计同步；录音期间暂停这组统计后，
用户上板确认当前诊断固件录音已无噪点，独立的 CPU 使用率日志仍保留。

补丁只作用于构建生成的 TinyUSB 源文件，托管组件原文件不修改。
完整证据、计数含义和恢复实际 AEC 输出的步骤见
[应用组件说明](src/README.md#capture2-录音噪点排查结论2026-10)。

后续单点双声道零值已在 COM6 复现：设备 FIFO 无零帧和欠载，关闭串口仍有噪点；
证据指向瞬时水位导致 USB 短/长包频繁波动，触发主机块尾补零。Capture2 现采用平均水位和分数包长校正，
实板对照及验证范围见 [闭环记录](src/README.md#com6-实板闭环单点零值的包长控制修复)。

当前固件还将 DSP 统计及 USB 状态移入后台日志路径，并覆盖 I2S/RMT 驱动的常规 ESP 日志。
默认任一路 USB 播放或录音开启时静音普通诊断，异常异步限频报告；配置入口和队列策略见
[音频链路的串口日志隔离](src/README.md#音频链路的串口日志隔离)。
