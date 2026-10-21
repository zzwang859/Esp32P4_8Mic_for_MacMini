# ESP32-P4 UAC 2.0 高速 USB 声卡

新增厂商自定义 HID 控制接口，PID 为 `0x8003`。上位机可读取结构化设备/模型信息、
实时配置各关键词灯效和快捷键、独立禁用关键词动作，以及完整更新 KWS 模型 A/B 槽。
普通命令通过 FIFO 在后台处理；更新维护期 USB 录音输出零 PCM。
通信协议、分区和参考 PC 工具见 [`custom/README.md`](custom/README.md)。
工程启用 PSRAM XIP，完整 VAD/KWS 模型包复制到 PSRAM，避免模型擦写关闭缓存影响 USB。
KWS 热点权重、MFCC 工作区及当前模型工作张量仍使用 SRAM；张量区在规划前实际预留，
防止热切换时因碎片化退回 PSRAM。实测及适用范围见 [`custom/SRAM_VALIDATION.md`](custom/SRAM_VALIDATION.md)。
首次安装需要完整烧录新分区表。

AFE 经 NsCore 后的输出默认接入独立 KWS 推理任务。menuconfig → `AFE keyword recognition`
→ `KWS input source` 也可选择送入 AFE 的 16-kHz 单声道 USB 播放参考；
取样位于 AEC 前，包含参考延迟，不经过 NsCore，与 Capture2 选源独立。
模型替换、消抖、RTF 与后台日志策略见
[`kws/README.md`](kws/README.md)。

本目录是工程的 ESP-IDF 应用组件，功能如下：

- 原录音设备：24 kHz、16-bit PCM、8 通道，`ch0..ch7` 交错排列；
- 第二录音设备：16 kHz、16-bit PCM、双通道，左声道为所选音频，右声道可选复制左声道、VAD 标志或平滑 DOA；
- 放音：16 kHz、16-bit PCM、2 通道，左右声道交错排列；
- UAC 2.0 High-Speed，异步播放反馈端点；
- 同一 USB 复合设备还枚举 HID 键盘和厂商控制 HID；
- 播放音量和静音由 PC 端处理，设备不枚举播放 Feature Unit，USB PCM 直接写入 I2S；
- 8-slot MSB TDM 录音输入和标准双声道 Philips I2S 放音输出；
- 原始录音软件 FIFO 容量 50 ms、播放软件 FIFO 容量 40 ms，处理链路使用独立缓冲及任务。

## DOA、DAS 和回声消除

`audio_processing/array_dsp.cpp` 实现固定 SRP-PHAT 定位和 DAS 波束形成的单精度
连续处理：24 kHz、256 点 Hann STFT、128 点 hop、
360 个方位、约 500–4000 Hz 的 DOA 分析频段。阵列半径默认 56 mm，通道方位沿用
`157.5、202.5、247.5、292.5、337.5、22.5、67.5、112.5°`。
0° 指向 +X，角度沿逆时针增加。SRP-PHAT 互谱使用约 170 ms 的指数平均。
DAS 对全部频点使用 `w_m(f) = a_m(f) / 8`，按 `sum(conj(w_m) * X_m)` 合成音频；
参考本地 pyroomacoustics 的单目标远场、无距离衰减 DAS，以阵列中心为相位参考。
仅方位改变时更新权重，不进行协方差矩阵求解。OLA 和重采样状态跨帧保留，
不做整块截断拼接；固定一 hop 的处理延迟及 AEC 参考延迟计算沿用原链路。

STFT 前将原始 PCM 转成浮点，并按通道乘以
`10^((common_base_gain_db + channel_trim_gain_db[ch]) / 20)`。
系数直接从 `src/audio_processing/gain_cali.json` 在 CMake 构建时生成；
公共增益叠加各通道 trim，具体数值以 JSON 为准。修改 JSON 后重新构建会自动更新
固件系数，端侧运行时不读取 PC 文件。校准值保持浮点，不截断回 16-bit；DC 去除、DOA PHAT 统计和 DAS
均使用校准后的数据及跨帧历史。原始 USB 8 通道设备仍上传未校准 PCM，避免 PC
Python 再应用校准时重复增益；播放参考不应用麦克风校准。

DOA 固定为 SRP-PHAT，结果日志默认关闭。打开日志后，UART0 每 500 ms 报告方位、有效标志、谱峰锐度及计算耗时。
发送单个字符即可控制日志，无需回车：`e` 打开 DOA 结果日志，`d` 关闭日志，大小写均可。
已移除算法切换命令 `s`、`m`、`t`。
关闭日志不影响 DOA 控制。关闭 LED 功能后仍保留 DOA 串口命令。
仅在 AFE VAD 为 `SPEECH=1` 时估计并接受有效 DOA 目标；VAD 为静音或未知时冻结当前转向。
DAS 每个 128-sample hop 沿最短角距离逐步接近目标，跨 0°/360° 时也选择较短路径；
恰好相差 180° 时选择角度增加方向。默认最大速度 30°/秒，可在 menuconfig →
`Microphone array processing` → `DOA rotation speed limit (deg/s)` 配置，范围 1–360。
暂停或静音的时间不累积成恢复时的角度跳变。无有效 DOA 时继续使用上一有效目标，
但仅在 VAD 为语音时允许转向；固件接口、DOA 日志及 DAS 使用同一个平滑后的方位。

处理链路为：

```text
PCMD3180 8ch/24k -> STFT -> DAS  -> continuous OLA -> FIR 24k:16k
                                                           |
PC volume/mute -> USB stereo/16k -> I2S playback -> (L+R)/2 -> 16-kHz delay
                                                           |
                                         Espressif AFE SR: AEC(FD_HIGH_PERF), VAD, AGC
                                                           |
                                         core1 NsCore: 16k mono, 160-point step
                                                           |
                                         KWS / Capture2 L: NS audio, R: selected
```

上图的 STFT 输入已完成浮点通道校准。24→16 kHz 降采样发生在 AFE 之前。
AFE 使用 `AFE_TYPE_SR`、`AEC_MODE_FD_HIGH_PERF`，关闭 NS，启用 WebRTC AGC；
AFE 后新增从 STM32 例程移植的 NsCore，独立任务运行在 core1、优先级 4。
正常 Capture2 和 KWS 均使用该任务的降噪后音频；原正弦波/播放参考诊断源继续保持独立。
在 `Microphone array processing` → `Post-AFE NsCore minimum gain (%, lower = stronger; 100 = bypass)`
配置 `denoiseBound = 百分比 / 100`，范围 1–100%，默认 25%，越小降噪越强，100% 精确旁路。
NS 每 160 点/10 ms 处理一次，活动滤波新增 96 点/6 ms 名义 OLA 延迟，另有不足一帧的重分帧等待。
VAD/DOA 元数据随该延迟移动；全局 AFE VAD 和 DOA 门控仍在降噪前。
8 块 PSRAM 输入队列满时丢弃新块并计数，不阻塞 AFE；消费者遇到缺口时复位 NS 和 KWS。
后台每 10 秒按现有静默策略打印 NS CPU、平均/最坏帧耗时、丢样本和复位次数。
来源、平台改动和验证见 [NsCore 移植说明](audio_processing/nscore/README.md)。
同时启用 VADNet `vadnet1_medium`（`VAD_MODE_1`，默认最短语音 128 ms、最短静音 1000 ms、缓存 128 ms）。
前两个时长可在 menuconfig → `Microphone array processing` 配置：
`AFE VAD minimum speech duration (ms)`（33–2000 ms，默认 128）及
`AFE VAD minimum non-speech duration (ms)`（65–5000 ms，默认 1000）。
例如可将语音时长改为 180 ms、非语音时长改为 500 ms；实际判定受 AFE 算法帧粒度影响。
缓存 delay 保持 128 ms，修改语音时长后 AFE 环形缓冲容量自动重算；重新编译、烧录后生效。
AFE 的 `MR` 输入为一路 DAS 麦克风和一路 AEC 播放参考，不是双麦克风；处理后仅输出一路音频。
`model` 分区位于 `0x410000`、大小 512 KiB，ESP-SR 构建时根据
`CONFIG_SR_VADN_VADNET1_MEDIUM=y` 生成 `build/srmodels/srmodels.bin`。
初始化先加载该分区、选取 `vadnet1_medium`，再创建单麦 AFE；缺失或未烧录模型时明确报错。
`audio_pipeline_get_vad_state()` 提供非阻塞、ISR-safe 的最新检测状态：
`AUDIO_PIPELINE_VAD_UNKNOWN=-1`（尚未取得有效 AFE 帧或 fetch 失败）、
`AUDIO_PIPELINE_VAD_SILENCE=0`（噪声/静音）、`AUDIO_PIPELINE_VAD_SPEECH=1`（语音）。
状态随 AFE fetch 更新，带有算法和最短持续时间造成的判定延迟。
VAD 仅发布标志；USB 音频和 KWS 输入持续传递，不根据 VAD 丢帧，也不重复插入 VAD 缓存。
处理结果进入 Capture2 的左声道，右声道默认填入对应音频帧的 VAD 标志。AGC 压缩增益由
`CONFIG_ARRAY_AGC_GAIN_DB` 配置。实际消回声效果及 CPU 占用需上板测量。

当前 ESP-SR 2.5.1 的 VAD 历史长度为 `vad_min_speech_ms + vad_delay_ms`，默认 256 ms。
在 512 点/32 ms 算法帧下默认占 8 帧，AFE 环形缓冲另外保留 4 帧处理余量，共 12 帧；
容量随配置自动调整，例如最短语音 180 ms 时配置 14 帧。
原 4 帧配置在启动后积累 VAD 历史时会让 feed 的空闲空间检查持续失败，出现
`AFE(FEED) is full` 与 `AFE is empty` 交替报警。增加的是存储容量，不要求预填满 12 帧，
不改变左声道处理算法或 Capture2 的启动水位。启动日志打印实际帧长、容量及历史时长；
若未来库的帧长导致容量不足，初始化会明确报错。

可选诊断构建启用 `CONFIG_ARRAY_CAPTURE2_PLAYBACK_TEST=y`：用 USB 播放参考数据
进行播放到 Capture2 的数字回环。取数点是 AFE 的参考输入，
即实际写入 I2S 的双通道 PCM 经 `(L+R)/2` 合成的 16 kHz、16-bit 单声道数据，包含原有
参考延迟及播放缺包补零，不应用麦克风校准增益。AFE 仍处理输入；
Capture2 沿用现有输出 FIFO 和 USB 上传路径，Capture1 的原始八通道数据不受影响。
四种录音模式均通过 `idf.py menuconfig` 中的
`Microphone array processing -> Capture2 left channel source` 配置，无需修改代码：

| 模式 | 菜单选项 | 配置项 |
| --- | --- | --- |
| AEC 直接输出 | Espressif AFE output | `CONFIG_ARRAY_CAPTURE2_AEC_OUTPUT` |
| 信号发生器录音测试（1 kHz 正弦波） | Replace Capture2 AEC output with a 1-kHz sine table | `CONFIG_ARRAY_CAPTURE2_SINE_TEST` |
| NsCore 后正弦波对照 | Replace Capture2 audio after NsCore with a 1-kHz sine table | `CONFIG_ARRAY_CAPTURE2_POST_NS_SINE_TEST` |
| USB 播放回环录音测试 | Loop USB playback reference back to Capture2 | `CONFIG_ARRAY_CAPTURE2_PLAYBACK_TEST` |

四项互斥，只能选择一种，工程默认启用 AFE 输出。保存配置后需运行 `idf.py build`
重新编译，再运行 `idf.py -p <串口> flash` 烧录生效，暂不支持运行时切换。
选择 `Espressif AFE output` 并重新编译、烧录即可恢复正常 AEC 录音。

Capture2 的四个音频选项只改变左声道。右声道可在 menuconfig →
`Microphone array processing` → `Capture2 right channel source` 独立配置：

| 选项 | 右声道数据 |
| --- | --- |
| Copy left channel audio | 逐样本复制最终选定的左声道，包括正弦波/播放参考诊断源 |
| VAD flag | 默认；输出对应块的语音、非语音或未知标志 |
| Smoothed DOA used by DAS | 当前 DAS 实际使用的 VAD 门控、限速后的角度，非原始 SRP-PHAT 目标 |

DOA 使用 PCM16=`round(角度×100−18000)` 编码，解码角度为 `(PCM16+18000)/100.0` 度；
例如 0°/90°/180°/270° 对应 −18000/−9000/0/9000。正常录音在 AFE 输出入 NS 队列时取快照，
跟随 NS 的 96 点名义延迟移动；诊断源在音频块发布时取快照。元数据与左声道共同排队，
不会用 USB 读取时的新角度重标旧音频，也不回溯补偿 AFE 内部延迟。
这些都是编译时选项，修改后需重新编译、烧录。

USB 枚举名称为 `Stereo Capture`，PID 为 `0x8003`；
格式为 16 kHz、16-bit、双声道，逐帧 `L,R` 交错，声道位置声明为 Front Left / Front Right。
选择 VAD 模式时，右声道将语音 `+1` 编码为 `+32767`，非语音 `-1` 编码为 `-32768`，未知编码为 `0`。
归一化浮点 PCM 对应约 `+1.0/-1.0`，可按右声道正负判断语音/非语音。
正常 AFE 输出模式下，每次 fetch 的音频与该次 `vad_state` 配对进入 NS，降噪并补偿元数据延迟后进入 FIFO，
USB 只读取完整的双声道帧；即使一个 USB 包跨越状态切换点，也保留逐样本配对关系。
这是 AFE 的帧级判定，仍有 VAD 的起止判定延迟，不回溯补标前面的音频。
正弦波和播放参考诊断模式保持原左声道生成路径；选择 VAD 时，右声道为生成该块时最新的麦克风 VAD，
不能用于判断诊断信号本身是否为语音。KWS 与 LED RMS 仍只使用音频，不接收满幅 VAD 标志。
DOA 继续为 DAS 提供转向，固件接口和串口日志保持可用。

构建后可运行 `python src/usb_driver/tests/test_capture2_stereo.py --cxx <主机C++编译器>`，
验证左右声道排列、包内 VAD 切换、15/16/17 帧打包、缓冲背压及最终 ELF 中的 FS/HS 双声道描述符和 PID。
附加 `--objdump <riscv32-esp-elf-objdump>` 可检查 USB 使用的 FIFO 读取函数不含浮点指令。
`python src/audio_processing/tests/test_capture2_vad.py --cxx <主机C++编译器>` 验证真实 AFE fetch
路径的三种右声道选择、配对快照、左右声道共同排队、原音频/RMS/KWS、ISR 读取和重新开流清队列。

回环测试时，PC 向设备播放音频并录制 Capture2 左声道，比较时需对齐参考及 USB 缓冲延迟。
左右声道相同可直接比较；只播放一个声道时回环幅度减半，左右反相时会相互抵消。
停止播放后，已有缓冲和参考延迟排空，回环输出应变为静音。
AEC 参考取自实际写入 I2S 的 16-kHz 音频，包含 PC 端音量/静音处理结果和播放缺包补零；停播时继续
送静音，避免 DAC 播放尾部残留旧音频。参考在 16-kHz 域自动补偿一个 hop 的 OLA
延迟及麦克风 24→16 kHz FIR 群延迟。`Microphone array processing` 菜单可配置阵列半径、额外参考延迟
`CONFIG_ARRAY_AEC_REFERENCE_DELAY_MS`（0–200 ms）和 AGC 压缩增益。
单参考 Espressif AFE 使用左右声道平均；互相独立或反相的立体声播放无法保证充分消回声，
通话场景建议左右播放相同的参考信号。声学路径及 DMA 延迟需上板校准。

core 1 执行 DAS、OLA、24→16 kHz 重采样（优先级 6）、AFE 后 NsCore（优先级 4）及 DOA（优先级 3）；
core 0 执行 AFE 内部处理和 feed/fetch（优先级 5）、KWS 单核推理（优先级 4）。
NS 和 DOA 可被实时 DSP 抢占，KWS 可被 AFE 抢占。I2S 采集/播放持续运行，USB 回调仅取
处理结果。DSP 任务只更新 hop 最大耗时、输入/回声处理/USB 等待次数及参考欠载/溢出计数；
独立后台任务每 10 秒读取统计并生成报告。默认在任一路 USB 播放或录音开启期间暂停普通诊断输出，
包括 DSP、CPU 使用率、DOA 结果及控制台信息；处理和 CPU 统计采样继续运行。
静音期间保留 DSP/AEC 最大耗时，所有接口关闭后的下一次后台报告会包含这些峰值。
一个 hop 的预算是 5333 us。未打开 Capture2 录音时，AEC 仍持续更新但其输出不排入 USB 缓冲；打开时清除旧缓冲，
防止上传之前录音的旧片段。
Capture2 异步 IN 端点根据缓冲水位发送每毫秒 15/16/17 个双声道帧（60/64/68 字节），补偿 I2S 与 USB
时钟漂移；录音启动门槛和平均水位目标为 48 ms，通过 128-ms 水位滤波缓慢调节，
为 AFE 帧的调度抖动留出余量。
诊断日志另报 Capture2 实际完成的 USB 传输包数、零长度包、异常长度包、
应用读数不足、全零源数据包及应用 FIFO 最低水位；零长度包计数从首个有效数据包后开始。
`failed` 表示 DWC2 等时 IN 传输未完成，不表示 DSP 输出了零样本。当前构建通过
`src/CMakeLists.txt` 对 TinyUSB 0.19 的 DWC2 源文件加入 DMA/PTI 时序处理补丁；
PTI 会屏蔽旧的未完成中断，因此仍需以 PC 录音结果确认是否消除了缺包。

### 音频 FIFO 与延迟水位

下表的容量是最大存储空间，不代表正常运行时必须等待这么长时间。
毫秒数按各队列实际 PCM 格式计算；Capture2 的应用输出 FIFO 按音频/VAD 双声道帧存储。

| 队列 | 容量 | 交付节奏或运行水位 |
| --- | --- | --- |
| I2S RX DMA | 8 × 4 ms = 32 ms，12288 B | 每 4 ms 交付一批；原为 8 × 10 ms |
| `input_stream`：采集 → DAS | 96000 B，250 ms，8ch/24k | 满足 128 帧 hop 后计算，约 5.33 ms；不预填满 |
| `afe_stream`：DAS/参考 → AFE feed | 12800 B，200 ms，2ch/16k | 读够一个 AFE feed 块即处理；不预填满 |
| AFE → NsCore 输入队列 | 8 块 PSRAM，每块最多 1024 点，当前 AFE 块为 512 点 | 零等待入队，满时丢新块；NS 每 160 点处理，缺口时复位 NS/KWS |
| AFE 内部 ring buffer | 随 VAD 语音时长自动调整，默认 12 个算法帧，当前帧长 32 ms | 默认 VAD 历史占 8 帧（最短语音 128 ms + delay 128 ms），另留 4 帧处理余量；启动时检查实际 fetch 帧长并打印容量 |
| `output_stream`：所选音频/VAD → Capture2 | 8192 B，128 ms，stereo/16k | 启动/平均目标 48 ms，模型维护后恢复预填 96 ms；128-ms 水位滤波、分数包长校正 |
| `s_mic_stream[0]`：原始录音 → Capture1 | 19200 B，50 ms，8ch/24k | 启动/低水位 4 ms，高水位 12 ms；原高水位 40 ms |
| TinyUSB Capture1 IN FIFO | 2048 B，约 5.33 ms | 回调按包补充，通常约一个 USB 包 |
| TinyUSB Capture2 IN FIFO | 512 B，8 ms，stereo/16k | 回调按包补充，通常约一个 USB 包 |
| TinyUSB playback OUT FIFO | 2176 B，34 ms，stereo/16k | 接收回调立即排入播放软件 FIFO，无预填水位 |
| `s_spk_stream`：USB → 播放任务 | 2560 B，40 ms，stereo/16k | 启动/重新预填 4 ms，反馈目标 8 ms；原目标 10 ms |
| I2S TX DMA | 4 × 2 ms = 8 ms，512 B | 保留原有描述符和调度余量 |
| `reference_fifo`：实际 I2S 写入 → AEC 参考 | 8000 B，250 ms，mono/16k | DSP 连续消费，不主动预填；保留余量避免参考丢样 |
| AEC 参考对齐延迟线 `ref_delay` | 当前 102 samples，6.375 ms | OLA 与 FIR 补偿，加额外参考延迟配置；不能作为普通 FIFO 随意缩短 |

`USB_AUDIO_MIC2_RING_BYTES` 虽仍有宏定义，但当前没有创建对应的第二个 `s_mic_stream`；
Capture2 实际从 `output_stream` 读取，不能把两者重复计入延迟。
USB 控制器/I2S 外设另有硬件 FIFO，由驱动管理，未设置应用层的预填阈值。

水位常量集中在 `usb_driver/usb_audio_config.h`。这轮优先减少正常排队与采集交付周期，
保留软件 FIFO 容量、AFE 内部缓冲、算法 hop、AEC 对齐延迟和 TX DMA 余量。
当前 AFE 实测按 512 点/32 ms 交付，正常 NsCore 再按 160 点发布。Capture2 启动门槛为 48 ms，
原始 feed 诊断源通常等待两个块（64 ms），NS 发布路径以实际累积帧数为准。
当前先对 FIFO 水位作约 128 ms 的整数低通，再向 48 ms 平均目标缓慢调节；
分数累加器将调节量分散到单个 15/17 帧包，最大校正速度约 ±3906 ppm。
不再把 32-ms 算法块和调度抖动直接转换成连续短/长包。水位低于一包可发送的完整帧数时，
停止发送并重新预填，避免长期贴着空队列运行。
播放欠载仍补零并重新预填，所有 USB 回调保持完整帧读写和整数运算。

此前延迟优化曾将 Capture2 低水位降低 6 ms（现已被平均水位控制替代）、采集批次最长等待降低 6 ms，
播放软件队列目标少 2 ms；这些量分属不同路径和阶段，不能直接相加当成实测端到端改善。
PC 音频栈和 AFE 算法仍有自身延迟。后台日志在流关闭后可查看 Capture2 最低水位、
零包/短读，以及播放欠载/丢字节/I2S 错误计数。

### Capture2 录音噪点排查结论（2026-10）

#### 单点双声道零值的后续排查

如果噪点是某一个采样时刻的 `L=0,R=0`，即使右声道是 DOA/VAD 也同步发生，
应优先检查双声道配对后的缓冲、USB 传输和主机补零。
这与下文已处理的整包缺失是不同的现象，目前尚未确认单点零值的根因。
`TX zero` 统计零长度包，`source zero` 统计左声道整包静音，两者均不能排除单点零帧。
VAD 未知状态编码为 0；DOA 恰好 180° 也编码为 0，不能将所有零值直接视为损坏。

原 `ARRAY_CAPTURE2_SINE_TEST` 由 core0 的 AFE feed 任务按 feed 块发布；正常录音由
core1 的 NsCore 任务按 160 点发布。原正弦波正常只能证明该诊断模式下录音正常，
并非只替换了正常录音的采样数值。
新增左声道选项 `Replace Capture2 audio after NsCore with a 1-kHz sine table`
（`CONFIG_ARRAY_CAPTURE2_POST_NS_SINE_TEST`）保留正常 fetch、NS 队列、降噪处理、
160 点发布任务及延迟后的元数据，仅在发布 USB 音频前替换左声道；KWS 输入仍为真实数据。
与原正弦波模式比较，可区分输出任务/节奏差异与音频内容的影响。

后台增加 `capture2 lifetime frames queued/read=... zero-pairs queued/read=...`
及当前录音 `session packets 15/16/17=...` 计数。`zero-pairs` 对每一个 `L=0,R=0`
完整双声道帧计数，分别位于成功写入应用 FIFO 后和 USB 从应用 FIFO 读出后。
计数操作不打印日志，统计仍遵守录音期间静默策略，停止录音、等待后台统计后查看。
它们是启动以来累计值，应比较同一次实验前后的增量；开启录音时丢弃的旧 FIFO 数据不算 USB 读取，
停止后 FIFO 也可能留有尾帧，不能要求两个累计帧数严格相等。

- 发布端已经有异常零帧：检查配对缓冲及元数据生成路径。
- USB 读出端新增零帧、发布端没有：检查应用 FIFO 存储和并发访问。
- 两端在稳定非零元数据区间均没有零帧，而 PC 文件出现：继续检查 TinyUSB FIFO、DMA、
  USB 包长/时序及主机音频栈；仅凭这些计数还不能区分这几段。

`15/16/17` 帧分别对应合法的 `60/64/68` 字节包，不表示发生了丢包。
但连续短/长包不能用来追赶算法块产出抖动，否则可能扰动主机接收节奏；当前已改用平均水位控制。
新诊断选项默认不启用，复测时保持相同右声道源和录音软件。

#### core1 调度与打印隔离

进一步对照反馈是：core1 越忙，同期零值越多；NsCore 后替换正弦波仍异常，
NsCore 前按相同样本量替换正弦波正常。这支持优先检查 NsCore 的内容相关耗时及其
输出调度间隙，但还不能单独证明打印任务是阻塞源。

| 任务 | CPU / 优先级 | 与 NsCore 的关系 |
| --- | --- | --- |
| `array_das` | core1 / 6 | 可抢占 NsCore，处理 8 通道 STFT、DAS、重采样 |
| `post_afe_ns` | core1 / 4 | 每 160 点处理降噪，再提交 KWS 和 Capture2 |
| `array_doa` | core1 / 3 | 低于 NsCore，计算定位 |
| `led_ring` / `led_input` | 无固定核 / 默认 3 | 可能在 core1 执行；LED 浮点计算可能使任务固定在运行它的核 |
| `cpu1_probe` | core1 / 2 | 每 10 秒读取一次空闲任务计数，不打印 |
| `audio_diag` | core1 / 1 | 日志格式化和 UART 输出，低于 NsCore |
| `custom_control` | 无固定核 / 1 | 处理 HID 和保存配置，低于音频处理任务；录音开流不阻塞配置写入 |
| `array_stats` / `cpu_usage` | core0 / 1、2 | 后台统计格式化，实际 UART 输出交给 `audio_diag` |

当前 NsCore 路径不调用 `printf`/`ESP_LOG`，也没有获取 UART 的 TX 互斥锁。
因此不能仅因低优先级打印任务在 UART 上阻塞，就认定高优先级 NsCore 同步等待它。
普通日志原先仍经 `vprintf` 进入 UART VFS，逐字节获取驱动信号量；现已统一采用
与 logits 一样的整行 UART 提交，已格式化日志直接发送，不再重复格式化。
该改动减少驱动调用次数，但是否消除录音欠载仍需实板对照。

在 `ESP32-P4 USB audio` 中新增
`Suspend all console UART output during Capture2 (diagnostic)`
（`CONFIG_AUDIO_DIAGNOSTICS_CAPTURE2_SUSPEND_UART`，默认关闭）。
开启后，Capture2 接口开启期间跳过常规日志、警告/错误、KWS 结果、logits 及命令确认输出，
并拦截 stdout/stderr；识别和命令处理仍运行。`l/p` 开关不能覆盖这个隔离设置。
已在发送中的一条输出可能完成，panic/ROM 直出不属于普通日志路径。
关闭 Capture2 接口后恢复新日志，不重放被抑制的消息。

复测时保留实际 AEC/NS 音源和相同负载，只切换这个选项。结束后关闭全部 USB 音频接口，
等待下一次后台统计（普通日志若被 `l` 关闭，先用 `l` 打开），比较
`NS avg/max`、`dropped/resets`、`USB null mono`、`source min` 和零帧计数：
若只在启用隔离时恢复正常，才有证据将串口活动列为关键触发条件；若仍异常，
应继续检查高优先级 DAS 占用及 NS 运算/发布间隙。不要把 `NS max` 当作纯计算时间，
它使用墙钟计时，包含抢占；`NS CPU` 才是任务运行时间占比的估计。

#### COM6 实板闭环：单点零值的包长控制修复

在 ESP32-P4 v1.0、COM6 上自动复位/烧录，使用 Windows WinMM 同步播放
`test.wav` 并录制 16-kHz 双声道 Capture2。左声道选择 NsCore 后正弦波，右声道为 DOA。
复现文件中右声道稳定约 -18000（主机共享音频路径存在约 ±1 LSB 的量化/抖动），
零值故障是两路同时插入约 1～4 个零采样，前后正弦波序列仍连续；全部出现在主机
160 帧/10-ms 块尾。检测时同时检查两路近零值和正弦波递推连续性，排除正常过零点。

| 包长策略 / 日志 | 录音时长 | 零值插入处数 / 总帧数 |
| --- | ---: | ---: |
| 原瞬时水位控制，全部日志及原始 logits | 41.08 s | 5 / 9 |
| 原瞬时水位控制，关闭原始 logits | 41.06 s | 5 / 7 |
| 原瞬时水位控制，普通日志与 logits 均关闭 | 41.07 s | 4 / 9 |
| 平均水位控制，全部日志及原始 logits | 41.07 s | 0 / 0 |
| 平均水位控制，全部日志及原始 logits，仅录音长测 | 120.00 s | 0 / 0 |
| 平均水位控制，恢复正常 AEC/NsCore，关闭测试日志，同步播放/录音 | 41.08 s | 0 / 0 |

前三轮设备端 `zero-pairs queued/read=0/0`，NS `dropped/resets=0/0`，USB 零包和短读均为 0，
最低应用 FIFO 水位仍有约 184～208 点。第三轮在录音区间串口字节数保持不变，仍能复现。
这排除了本轮复现由打印阻塞或 NS 输出 FIFO 实际耗尽直接造成的解释。

原 Capture2 每 1 ms 根据瞬时 FIFO 水位立即选择 15、16 或 17 帧。
AFE 的 512 点批次和 NS 的 160 点输出节奏，使其频繁、成串发送短/长包；
原策略约 22% 的包为 15/17 帧。该轮改成约 128 ms 的整数低通水位，当时目标为 40 ms，
以约两秒的响应时间缓慢校正；分数累加器分散单帧校正，幅度上限约 ±3906 ppm。
仍保持原 PCM/DOA/VAD 配对，不重采样、不丢样、不补假音频，应用 FIFO 背压时不消耗校正相位。
实板对照中 15/17 帧包比例降至不足 1%，首轮录音的零值插入及正弦波断点均消失。
后续 HID 模型更新实测中，双录音并发恢复的低水位余量不足，目标已提高到 48 ms，增加 8 ms 缓冲余量。

证据指向包长抖动触发主机音频路径的 10-ms 块尾补零；没有进行 USB 总线抓包，
因此不将具体插入零值的驱动层级视为已单独证实。上述结论针对单点零值，
与后文历史上的 DWC2 整包缺失及同步统计打印问题分别验证。

120 秒正弦波长测包含 192 万帧，递推连续性检测无断点；最终正常路径复测中 NS/KWS
丢样、复位及错误计数均为 0，播放欠载、丢字节及 I2S 错误均为 0。
正常路径这轮左声道为约 5562 的近直流输出、DOA 为约 -18000，验证范围是同步零值插入和链路连续性，
不据此声称语音音质或降噪效果已完成评估。开流前 100 ms 预填阶段不纳入零值检测。
另一次 148 秒双工压力尝试遇到主机播放完成超时，未保存完整录音，不计入通过结果。
最终已烧录正常 AEC/NsCore + DOA 配置，并复位恢复默认日志策略，释放 COM6。

主机回归还覆盖 512 点直接发布和 512→160 点重分帧/处理间隙、0～4 ms 抖动、
±1000 ppm 源时钟漂移，各模拟两分钟且无应用 FIFO 欠载，验证真正时钟漂移仍能跟踪。
录音、UART、事件和零值分析保存在 `.codex-tmp/ns-zero-*`，该目录不纳入 Git。

本次排查用 `CONFIG_ARRAY_CAPTURE2_SINE_TEST=y` 将 AEC 输出替换为连续 1 kHz 正弦波，
但保留原来的 DSP/AEC 运算和按块产出节奏（当前 AFE 实测为 512 点/32 ms，以启动日志为准）。Capture1 的 8 通道原始录音正常，
可作为 TDM 采集及 USB 上传的对照。定位时需要分别检查 DSP 输出 FIFO、TinyUSB
端点 FIFO、DWC2 传输完成结果和 PC 录音文件，不能只看 FIFO 是否下溢。

1. 最初的随机整包缺失不是 DSP 生成了零样本：Capture2 源数据非零、FIFO 有余量，
   `scheduled=0`、源短读和端点短写均为 0；`TX zero` 与 `failed` 同步增长。
   `failed` 来自 DWC2 的等时 IN 未完成分支，位置在 USB 端点传输阶段。
2. 单独启用 USB DMA 后，`failed` 仍会增加。当前工程同时启用 DMA 和 PTI；
   `src/CMakeLists.txt` 在构建时生成带补丁的 TinyUSB DWC2 源文件，托管组件原文件
   不需要修改。PTI 后随机整包零点消失。PTI 会屏蔽旧的未完成中断，因而
   `failed=0` 只能作为辅助信息，最终仍以 PC 录音检查为准。
3. 剩余噪点约每 10 秒出现一次，与实时 `dsp_task` 打印的四行统计同步。
   115200 baud 串口输出会占用 DSP 产出时间。录音期间停止这四行周期日志，
   保留独立的 CPU 使用率日志；用户再次录音确认噪点消失，说明周期性 DSP
   串口统计是剩余噪点的关键触发条件，CPU 使用率统计不是本次的主要原因。

录音停止后 `TX packets` 保持不变是正常现象；最低 FIFO 水位只在新一次录音开始时
重置，避免停止后的 `min=0` 被误判为录音期间下溢。`ref zero` 统计的是放音参考
缺样，与 Capture2 USB 包是否为零是两回事。上述结论来自正弦波诊断；当前配置已切换为
USB 播放参考回环。恢复实际 AEC 录音时，应选择 `CONFIG_ARRAY_CAPTURE2_AEC_OUTPUT=y`
并重新构建、复测。

工程默认启用 PSRAM、4 MiB 应用分区及 512 KiB VADNet 模型分区，分区表为 `src/partitions.csv`。
PCMD3180 不应答时，I2C 探测最多等待
100 ms，串口报错后继续 USB、播放、LED/HID；录音补零，DOA 为无效状态。
其他 PCMD3180 配置错误也不会触发 `ESP_ERROR_CHECK`。PSRAM 缺失或处理框架
初始化失败时会报告错误，保留 USB 原始设备，单通道输出静音。

## 仅播放时断流的回归修复（2026-10）

针对“开录音时反复播放/停止正常，关闭录音后单开播放会卡住”的反馈，继续保留录音的
DMA/PTI 和 IN 回调顺序补丁，并补齐播放 OUT 路径：

- TinyUSB 0.19 在线性接收缓冲路径中，对 `tu_fifo_write_n(..., 0)` 的返回值使用
  `TU_VERIFY`。合法零长度 OUT 包会因此提前返回，下一次接收未挂接，后续播放数据无法进入设备。
  现在零长度包跳过 FIFO 写入，继续挂接 OUT 接收并调用应用回调。
- PTI 模式下，ISO OUT 的 `XFRC` 也可能表示丢包。检测 `iso_packet_drop` 后以零字节失败完成，
  由音频驱动继续接收；不再把旧缓冲作为有效播放数据，也不执行可能下溢的残余长度相减。
- 首次播放反馈只在打开对应 OUT 接口时提交。切换同一音频功能的录音接口时，不再改写、
  重复提交已经在 DMA 传输中的反馈缓冲。

补丁仍只作用于构建生成的 TinyUSB 副本。构建后可运行
`python src/usb_driver/tests/test_usb_audio_patches.py --cc <主机C编译器>`，测试直接抽取生成文件中的
接收函数，以模拟 FIFO/DMA 验证零长度包后续接收、连续空包、PTI 丢包、正常短包及状态事件过滤。
使用 `--source-dir managed_components/espressif__tinyusb/src` 可对比未打补丁源码的零长度包失败行为。

上述缺陷已通过源码定位；PC 卡住现象是否完全消失仍需上板验证。复测需覆盖冷启动后只播放、
反复播放/停止、播放中打开/关闭两路录音，以及 Capture2 长时间录音有无空包噪点。

### 播放异常日志

USB 回环录音中约每秒一次的噪点与播放任务每秒打印的长统计日志周期吻合。
现已移除这条定时日志：播放任务持续供给 I2S 和 AEC 参考，统计计数仍保留在内存中。
只有播放队列溢出丢数据、播放队列欠载补零、I2S 写入失败或短写时才触发播放异常日志。
正常预缓冲、停播补零及接口关闭时丢弃的尾部数据不作为异常报警。
异常由独立的低优先级任务打印，USB 接收回调和播放任务只发送通知；
同类异常会合并，每类日志最多约每秒一次，无新异常时不输出。
启动/停止接口事件和 CPU 使用率日志受下述音频运行静音配置控制。
修复效果需上板以相同音源复测，并检查播放期间 `ref zero` 是否继续增长。

### 音频链路的串口日志隔离

`audio_diagnostics.c` 在启动音频任务前安装 ESP 日志输出钩子。应用及 I2S/RMT 驱动的常规
ESP 日志写入固定容量的后台队列，入队等待时间为 0；后台 UART 工作由 core1、优先级1 的
`audio_diag` 完成，控制台也可直接发送开关确认。原始 logits 采用两位小数的快速整数格式化；
普通日志、识别结果和 logits 均整行提交 UART 驱动，避免逐字符信号量开销；AFE/KWS 不等待串口。
队列最多保存 24 条消息，每条文本最多 255 字节（超长文本截断并补换行）。队列满时丢弃新日志，
不会同步回退到串口或阻塞音频任务；累计丢弃量保留在内存，并在接口全部关闭后报告。
后台警告/错误输出还按每条至少 100 ms 的间隔限速，防止底层重复错误占满串口。
当前日志钩子按工程的 ESP-IDF Log V1 文本配置工作，仍保留有限的消息格式化和队列复制开销。

- DSP 的统计报告使用独立后台任务；实时波束形成任务不格式化、打印统计。
- USB 播放/录音启停、挂载和恢复回调只投递静态事件，后台完成格式化。
- LED/RMT 帧错误在应用层按每秒最多一条报告，保留累计帧错误数，并移除一处重复报错。
- panic、断言及早期启动日志不经过这条普通后台日志队列。

运行 `idf.py menuconfig`，进入 `ESP32-P4 USB audio`，配置
`Mute informational diagnostics during USB playback/capture`
（`CONFIG_AUDIO_DIAGNOSTICS_QUIET_WHILE_STREAMING`，默认开启）。
判断条件覆盖 USB 播放、Capture1、Capture2：只要任一路接口处于开启状态，便暂停普通信息输出，
控制台命令和 HID 动作仍执行，真实异常仍通过后台日志报告。停播但接口尚未关闭时仍保持静音。
关闭该选项后，普通信息恢复为异步输出；DOA 结果仍需串口 `e` 命令单独开启。
切换此配置后需重新编译、烧录。

后台统计的 `log dropped` 为累计队列满丢弃的消息数。可运行以下主机回归测试：

```text
python src/tests/test_audio_diagnostics.py --cc <主机C编译器>
```

测试使用实际日志模块，覆盖静音开/关、三个流交叠启停、ISR 投递、
满队列不阻塞、累计计数、超长日志截断。调度时延和噪点改善仍需上板验证。

## USB 端口

固件使用 ESP32-P4 的 High-Speed DWC 控制器（TinyUSB RHPORT1）和内部 UTMI
PHY，并启用 USB DMA。音频设备连接到板上的 USB 2.0 High-Speed 端口。

独立的 Full-Speed DWC 控制器（TinyUSB RHPORT0）不由本应用初始化，FS PHY
选择寄存器也不修改。由于本应用将 GPIO27 配置为 PCMD3180 的 I2C SCL，运行时
不能同时启用包含 GPIO27 的 FS 备用引脚映射；这不影响使用内部 UTMI PHY 的
High-Speed USB 音频接口。USB Serial/JTAG 调试接口不会被音频应用交换到另一组
FS 引脚。

## 外设 GPIO 分配

| 外设 | 控制器及工作模式 | 信号 | GPIO | ESP32-P4 方向 |
| --- | --- | --- | ---: | --- |
| PCMD3180 录音 | I2S0，TDM 主机，24 kHz、8 slots、16-bit | MCLK | 20 | 输出 |
| PCMD3180 录音 | I2S0，TDM 主机 | BCLK | 21 | 输出 |
| PCMD3180 录音 | I2S0，TDM 主机 | FSYNC/WS，50% 占空比 | 22 | 输出 |
| PCMD3180 录音 | I2S0，TDM 主机 | SDIN | 23 | 输入 |
| PCMD3180 控制 | I2C0 主机，100 kHz，7-bit 地址 `0x4C` | SCL | 27 | 双向、开漏 |
| PCMD3180 控制 | I2C0 主机，100 kHz，7-bit 地址 `0x4C` | SDA | 28 | 双向、开漏 |
| 立体声播放 | I2S1 主机，16 kHz、16-bit | MCLK | 32 | 输出 |
| 立体声播放 | I2S1 主机 | BCLK | 33 | 输出 |
| 立体声播放 | I2S1 主机 | LRCK/WS | 46 | 输出 |
| 立体声播放 | I2S1 主机 | DOUT | 47 | 输出 |
| WS2812 灯链 | RMT TX + GDMA，800 kbit/s | DIN（从物理编号 0 注入） | 26 | 输出 |
| 灯效切换按键 | 内部上拉，按下接地，40 ms 去抖 | KEY | 35 | 输入 |
| 调试控制台 | UART0，115200 bit/s | TX | 37 | 输出 |
| 调试控制台 | UART0，115200 bit/s | RX | 38 | 输入 |
| USB Audio HS | DWC High-Speed + 内部 UTMI PHY | USB D+/D- | 不占用普通 GPIO | 双向 |

PCMD3180 的 SCL 和 SDA 在软件中启用了内部弱上拉，硬件设计仍建议分别配置合适的
外部上拉电阻。当前 VS Code Monitor 使用板载 CH343 USB-UART 对应的 `COM5`，
通过 UART0 的 GPIO37/38 输出和接收控制台数据。GPIO24、GPIO25 当前未由本应用
分配；GPIO26 用于 WS2812 数据输出。

## LED 灯链驱动与逻辑分组

50 颗 WS2812 按物理编号 `0..49` 串联，数据从编号 0 的 `DIN` 注入。物理传输仍然
每帧依次发送全部 50 颗灯的数据，软件仅在帧缓冲填充阶段做以下逻辑分组：

- A 组共 8 颗：`0、1、11、17、23、29、35、41`。A 组不参与圆环动画；
- B 组共 42 颗：除 A 组以外的物理灯珠。灯效模块以连续逻辑编号 `0..41` 计算动画，
  发送前再映射到对应物理编号；
- 顶层模式 0、1、2 下 A 组保持熄灭；故障模式 4 下 A 组以 50% 亮度显示红色，
  B 组保持熄灭；
- 顶层模式 0、1、2 的 B 组颜色受 `CONFIG_WS2812_LED_RING_BRIGHTNESS` 全局亮度
  限制，默认值为 20%；故障模式的 50% 亮度不受该全局限制影响。

底层使用 ESP32-P4 RMT TX + GDMA，在 GPIO26 输出 GRB 数据，默认 100 FPS。工程要求
`CONFIG_WS2812_LED_COUNT=50`，编译期会校验该值，避免物理映射与帧长度不一致。
上电固定进入顶层模式 0。

## LED 顶层模式

| 模式 | 名称 | 显示行为 |
| --- | --- | --- |
| 0 | 默认模式 | B 组显示绿色呼吸底色；处理后录音的 RMS 电平驱动红色分量，左右声道平均后的实际放音 RMS 电平驱动蓝色分量。上电默认进入此模式。 |
| 1 | 对话模式 | B 组显示蓝色呼吸底色；处理后录音驱动红色分量，左右声道平均后的放音数据驱动绿色分量。 |
| 2 | 关键词触发 | B 组以调用方传入的颜色按 100 ms 周期闪烁，默认持续 1 秒。它是暂态模式；可指定结束后的模式，未指定时返回触发前的模式。 |
| 4 | 故障模式 | A 组 8 颗灯以 50% 亮度显示红色，B 组全部熄灭。 |

模式 0 和模式 1 中，呼吸底色、处理后录音 RMS 对应的红色分量以及放音 RMS 对应的蓝色或
绿色分量，都会在发送前统一乘以 `CONFIG_WS2812_LED_RING_BRIGHTNESS` 全局亮度：

```text
最终发送分量 = 映射后的 RGB 分量 × 全局亮度 / 255
```

当前全局亮度为 20%，即音频电平映射到最大值 255 时，实际发送的对应颜色分量约为
51。模式 2 的指定颜色也受这一全局亮度限制。模式 4 是例外：A 组红灯固定使用约
50% 亮度，不受全局 20% 配置限制。

处理后录音与放音电平使用 RMS dBFS。电平小于或等于
`CONFIG_WS2812_LED_AUDIO_THRESHOLD_DB` 时，对应音频颜色分量映射为 0；阈值到
0 dBFS 之间线性映射至 `0..255`。默认阈值为 -45 dBFS，也可在运行时调用
`led_ring_set_audio_threshold_db()` 修改。音频源超过 150 ms 未更新时按静音处理。

呼吸底色的亮度系数为
`0.04 + 0.96 * ((sin(2πt/period - π/2) + 1) / 2)^2`，默认周期为 6 秒。
模式 2 的默认持续时间由 `CONFIG_WS2812_LED_KEYWORD_DURATION_MS` 配置，当前为
1000 ms。颜色不使用固定配置宏，必须在每次调用 `led_ring_trigger_keyword()` 时传入，
因此不同关键词可以分别使用不同 RGB 颜色。可选参数还可以逐次覆盖持续时间并指定
下一模式；不提供可选参数时使用默认 1 秒并返回触发前的模式。

GPIO35 按键启用内部上拉，按键另一端应接 GND。每次有效按下按
`模式0 → 模式1 → 模式4 → 模式0` 循环；暂态模式 2 不进入按键循环。

## 串口命令与控制台
UART0 控制台为 115200 bit/s，直接发送单个字符即可切换，无需回车：

| 字符 | 操作 | 默认状态 / 说明 |
| --- | --- | --- |
| `0` / `i` / `I` | 进入灯环默认模式 0 | 启动模式 |
| `1` / `c` / `C` | 进入灯环对话模式 1 | 手动切换 |
| `2` / `k` / `K` | 以 `RGB(191,64,223)` 手动触发关键词模式 2 | 使用灯环默认时长，结束后恢复原模式 |
| `4` / `f` / `F` | 进入灯环故障模式 4 | 手动切换 |
| `6` | 键盘测试：Ctrl+C | 按下后释放，不依赖关键词动作开关 |
| `7` | 键盘测试：Ctrl+V | 按下后释放，不依赖关键词动作开关 |
| `8` | 键盘测试：小写字母 a | 按下后释放，不依赖关键词动作开关 |
| `a` / `A` | 开启全部关键词动作及灯效，有键位映射的词同时开启键盘 | RAM 配置，不自动保存 |
| `x` / `X` | 关闭全部关键词动作、灯效和键盘 | 每次上电/复位默认关闭 |
| `u` / `U` | 切换 HID 操作串口日志 | 默认关闭；确认 `HID logs ON/OFF`，重启不保留 |
| `l` / `L` | 切换普通运行日志 | 首次关闭，再次打开；重启恢复 menuconfig 默认策略 |
| `p` / `P` | 切换每个 10-ms KWS 推理帧的原始 logits CSV | 默认关闭；重启不保留 |
| `e` / `E` | 打开 DOA 结果日志 | 仍受普通日志开关控制 |
| `d` / `D` | 关闭 DOA 结果日志 | 默认关闭，不影响 VAD 门控转向 |
| `?` / `h` / `H` | 重新打印命令帮助 | 普通日志关闭时，先用 `l/L` 打开 |

表中字母命令的大小写别名等效，空格、Tab 和换行会被灯环控制台忽略。
灯环模式、`6/7/8` 测试和帮助依赖 `CONFIG_WS2812_LED_RING_ENABLE`；关闭灯环编译时，
`a/x/u/l/p/e/d` 仍由音频控制台处理。

每次上电或复位均保留已保存的颜色、键码和时长，但将所有关键词的
`actions_enabled`、`led.enabled`、`keyboard.enabled` 置为 false；启动回退到内嵌模型时同样禁用。
`a/A`、`x/X` 与 HID 配置命令在同一控制队列执行，成功后打印 `KWS actions ON/OFF`；
模型更新进行中返回 BUSY，队列满时提示重试。无键码的关键词不会被串口命令启用键盘。
HID 上位机可用 `SET_BINDING` 按词开启对应开关，协议不变；每次命令整体替换一个词的绑定。
串口开关仅修改 RAM，不自动保存；即使通过 HID 保存开启状态，下次启动仍默认禁用。
识别、串口识别结果及 HID 识别事件继续工作。手动 `2/k` 灯效测试和 `6/7/8` 键盘测试仍可使用。

`l` 首次关闭运行期普通日志，包括信息、警告、错误、队列丢日志汇总及 SDK 的标准输出/错误。
仅 `kws: keyword class= ...` 识别结果继续输出（内嵌/下载模型格式相同，不包含 label）；KWS 性能统计也会关闭。
HID 上位机操作串口反馈默认关闭，发送 `u/U` 开启，再次发送关闭；每次都会确认 `HID logs ON/OFF`。
该状态只保存在 RAM，重启恢复关闭，不受 `SAVE_CONFIG` 影响；HID 日志关闭不会停止命令执行、HID 应答或保存完成事件。
`u/U` 独立于 `l/L` 和 `p/P`，开启后关闭普通日志或 USB 开流时仍可见（Capture2 UART 隔离实验除外）。
日志包含命令、状态及绑定/模型参数；`SAVE_CONFIG` 分别报告受理和实际保存完成。
配置保存由优先级 1 的后台控制任务执行，播放、原始录音及 Capture2 开流均不阻塞写入。
保存日志中的 `save_priority=1` 表示后台任务优先级，`stream_mask` 仅显示三路接口状态，不作为保存门槛。
成功的数据分块传输不逐帧打印，失败和重放会按协议状态反馈；完整说明见 [控制 HID 协议](custom/README.md#4-调度应答与重试)。
每次 `l/L` 都单独确认 `non-KWS logs OFF; KWS results only` 或 `non-KWS logs ON`；
确认通过控制台输出，不占日志队列，即使队列满也不挤掉 KWS 结果。其余已排队的非 KWS 日志会被丢弃。
再次发送 `l` 打开普通日志，
覆盖 `AUDIO_DIAGNOSTICS_QUIET_WHILE_STREAMING` 的自动静默策略；DOA 的 `e/d` 开关仍独立生效。
该状态仅保存在 RAM，重启恢复 menuconfig 的默认静默策略。日志开关不停止音频、VAD、DOA、NS 或 KWS。
`p/P` 是独立的显式调试开关：打开后即使 `l` 已关闭普通日志、USB 正在开流，仍输出原始 logits。
内嵌模型格式为 `KL,seq,sil,unk,yes,no,up,down,left,right`；只在打开时打印这一行列名。
下载模型按当前实际类别数输出，类别名称通过控制 HID 的 classes 表查询。
后续每行以 `KL,帧序号,8个原始数值` 输出，数值固定显示2位小数，不做 softmax，
覆盖预热、背景及重复抑制期间所有成功推理帧。帧序号来自 KWS 输入队列，每块 160 点/10 ms；
它不是串口发送时间，模型失败或输入/调试队列丢帧时可跳号。
推理任务仅复制 8 个 float 并零等待入队，独立的 32 帧调试队列不占用关键词结果队列。
后台优先发送普通结果，再发送原始帧；使用任务通知唤醒，无需在推理任务打印或等待 UART。
每行使用512字节缓冲，异常大数导致整行超长时丢弃该调试帧，不发送截断的 CSV。
建议用 `l` 关掉其他日志。队列过载同样丢弃新调试帧，至多每秒用 `KD,n` 报告累计丢帧数；
这些丢失不会阻塞模型或导致音频丢样。再次 `p` 会停止并清除待发送追踪，显示 `KL OFF tx=... drop=...`。
原始追踪默认关闭，开关重启后不保留，也不会修改消抖机制。
当前判决使用8类 softmax 概率：默认同一关键词连续3帧且每帧概率至少80%才触发，
触发结果成功入队后全局冷却500 ms；冷却期间不累计关键词帧，结束后重新确认。
保留1秒预热和同词去重，silence/unknown 各自连续3帧更新去重状态，背景不使用关键词概率门限。
menuconfig 顶层 `KWS-Filter` 可设置概率门限、全局冷却时间和连续帧数。
同一菜单新增 `Need-VAD-Checked`（`No` / `Yes`，默认 `No`）：
`No` 保持上述逻辑；`Yes` 还要求连续确认的每个 10-ms 关键词帧全程 VAD 为高。
任一帧 VAD 低或未知就清空关键词确认计数，随后须重新累计完整帧数；背景类仍按原规则更新去重。
VAD 与音频一起排队；NsCore 使用对齐后的 VAD，USB 参考输入使用提交时最新的 AFE VAD。
menuconfig 设置作为编译默认值；运行中可通过 `custom/tools/kws_filter_gui.py` 修改四项判决参数，
设备立即持久化，详见 [HID 协议](custom/README.md#kws-判决参数运行中可改并立即持久化)及 [KWS 判决说明](kws/README.md#消抖和串口结果)。
`up/down` 参与判决，串口仍只输出关键词；`p/P` 保留原始 logits，不改成概率输出。
普通 `ESP_LOG` 在生产和发送阶段双重过滤，SDK 的 `printf`/`puts` 等经 stdout/stderr 的输出由
libc `_write_r` 链接包装过滤；其他文件描述符的读写不受影响。KWS 结果由后台任务直接写出完整行，
不经过共享 stdio 缓冲。ROM 启动、panic 等绕开普通日志/标准输出的底层信息不属于此运行期控制通道。

在日志打开时，按键或串口切换成功后，控制台会输出当前模式。串口和按键共享同一顶层模式状态。

## 配置与编译

工程已经包含顶层 `CMakeLists.txt`、`src/sdkconfig.defaults`、依赖锁文件和 VS Code
ESP-IDF 设置。先在 `ESP32-P4 USB audio` 菜单填写板级 I2S 引脚和主从模式：

```text
idf.py set-target esp32p4
idf.py menuconfig
idf.py build
```

Kconfig 中的 I2S GPIO 初始默认值为 `-1`，本工程的 `sdkconfig.defaults` 已按上表
设置板级引脚；若删除这些设置，程序会在启动阶段报告配置错误。已使用 ESP-IDF v6.1.0 和
`espressif/tinyusb` 0.19.0~3 构建；音频处理使用 `espressif/esp-sr` 2.5.1。
顶层 CMake 自动加载 `src/sdkconfig.defaults`。已有 `sdkconfig` 需确认 PSRAM、
性能优化和 `CONFIG_PARTITION_TABLE_CUSTOM_FILENAME="src/partitions.csv"` 已启用；
首次烧录这版固件使用完整 `idf.py -p <串口> flash`，同时更新分区表、启动程序、应用和 VADNet 模型。
不能只烧录应用：旧分区表没有 `model` 分区。当前 `model` 分区只打包 `vadnet1_medium`，
占约 282 KiB；KWS 权重继续独立嵌入应用。成功启动时应看到
`AFE channels: microphone=1 reference=1 (MR), output=1; VAD model=vadnet1_medium`。

PC 算法验证：

```text
cmake -S src/audio_processing/tests -B build-tests
cmake --build build-tests --config Release
ctest --test-dir build-tests -C Release --output-on-failure
```

测试覆盖七个方位的 SRP-PHAT、静音检测、流式 DAS 与双精度 PC C++ 的稳态 OLA
对比（含 JSON 增益）、DAS 的 1/8 归一化及转向更新、通道校准后的 PHAT 与不等增益阵列的 DOA、24:16 kHz
重采样的样本数量、直流增益及 11 kHz 抗混叠。
双精度对比基准位于 `audio_processing/tests/reference`，仅由主机测试编译，
不进入固件。双精度参考覆盖 SRP-PHAT 和全频段 DAS，以及 OLA 边缘归一化。
端侧源码、校准数据、配置模板、分区表和主机测试均在 `src` 下；固件与测试构建
都不依赖原 PC Python 工程目录，可独立保留或移除该目录。

## 音频硬件假设

当前采集后端匹配 PCMD3180：ESP32-P4 输出 24 kHz TDM 时钟，PCMD3180 作为
从机输出 MSB 对齐、16-bit、8 个连续时隙的数据；播放 DAC/codec 接受 Philips
I2S、16 kHz、16-bit 双声道。录音和放音使用不同 I2S 控制器，分别配置为主机。

当前 `USB_AUDIO_MIC_TEST_TONE_ENABLED=0`，8 通道录音设备使用 PCMD3180 的 TDM
数据，处理后录音设备输出 DAS + Espressif AFE 音频。若将该宏改为 `1`，原始 8 通道
端点生成 1 kHz 测试正弦波，跳过 TDM / DSP 输入，单通道处理设备输出静音。

USB 数据方向以主机为视角：8 路录音是设备到主机（IN），2 路放音是主机到设备
（OUT）。上板后仍需验证 USB 枚举、TDM 时隙顺序、I2S 极性以及长时间时钟漂移。

串口控制台键盘测试命令（115200 baud）：

- `6`：发送 `Ctrl+C`（包含按键释放报告）
- `7`：发送 `Ctrl+V`（包含按键释放报告）
- `8`：发送字符 `a`（包含按键释放报告）
