# AFE 关键词识别

从 `E:\esp32-p4-onnx\esp32p4_ds_tc_resnet\components\kws_port` 迁入 FP32 MFCC
前端、ESP-DL 神经网络后端、6 对流式反馈状态及优化转置。固定使用同一份 W16A16
权重。直接依赖 ESP-DL 3.3.11，与参考工程一致。

算子注册及优化转置的安装放在独立的 `kws_modules.cpp`，避免修改
`kws_port.cpp` 的模型加载或 SRAM 预算时重新编译全部算子。首次构建、清理后
构建或算子相关头文件变化时，注册文件仍需较长编译时间。CMake 生成的
TinyUSB / ESP-DL 补丁源码仅在内容变化时写入，避免无关的重新配置触发编译和链接。
命令行使用 `idf.py --ccache build` 复用编译缓存；VS Code ESP-IDF 扩展需开启
工作区设置 `idf.enableCCache`，扩展默认关闭此选项。

## 输入和调度

`afe_fetch_task` 在每次成功 fetch 后复制真实 AFE 的 16 kHz、PCM16 单声道输出到
8 块 PSRAM 队列；core1 的 `post_afe_ns` 任务按 160 点 / 10 ms 重分帧并运行 NsCore。
默认由该任务向 KWS 喂入降噪后的真实麦克风 PCM，USB 关闭时也持续工作。
模型输入将 PCM16 乘以 65536 转成参考接口所需的 PCM32。
降噪强度/100% 旁路在 `Microphone array processing` 菜单配置，默认最小频谱增益 25%。

menuconfig → `AFE keyword recognition` → `KWS input source` 可选择输入：

| 选项 | KWS 音频及唯一生产者 |
| --- | --- |
| Post-AFE NsCore output (microphone) | 默认；core1 `post_afe_ns` 的降噪后麦克风流 |
| USB playback mono at AEC reference input | core0 `afe_feed_task` 送入 AFE 前复制参考通道 |

USB 选项直接取交错 `MR` 中的 `R`，是实际 I2S 播放 `(L+R)/2` 的 16 kHz、PCM16 单声道，
已经包含 DAS/重采样对应的补偿和 `Additional playback reference delay`，缺少播放时连续补零。
该信号不经过麦克风 AEC/AGC 或 NsCore，也不再次消费参考 FIFO；只复制至原 KWS 队列。
输入源与 Capture2 左/右声道选择独立，AFE、NsCore、扬声器和录音链路继续运行。
此选项为编译时选择，保存后重新编译、烧录；启动日志显示实际 KWS 输入源。

AFE 只执行复制和零等待入队，不调用降噪/模型或等待这两个任务。KWS 的 32 帧 PSRAM 队列最多缓存
320 ms；满时丢弃新模型帧，不影响原录音链路。每帧序号让消费者在真正遇到缺口时
重置 MFCC、神经网络状态和消抖；NS 输入队列溢出或 AFE fetch 异常先按顺序复位
NS 的自适应/OLA 状态和未满帧；仅在 KWS 选择 NsCore 源时通知 KWS 制造序号缺口。
选择 USB 参考时，NsCore 丢帧或 AFE fetch 异常不会重置独立的参考输入 KWS。
不清空正在被推理任务使用的队列，不把缺口两侧音频拼成一帧。

推理任务 `kws_infer` 固定 CPU0、优先级 4、8 KiB 栈，使用单核推理，低于同核 AFE（5）、
USB/I2S（6/7）。CPU1 执行 DAS/OLA/重采样（6）、AFE 后 NsCore（4）与 DOA（3）。
每次完成一帧后检查经过时间（含 AFE 抢占）；距离上次主动暂停达到 20 ms 时阻塞 2 ticks，
在当前 1-kHz tick 下至少留下约 1 ms 调度窗口，避免追赶积压时反复错过 IDLE0。
主动暂停只在帧边界执行，帧内仍可被更高优先级任务抢占；不依靠增大看门狗超时或主动代替 idle 喂狗。
启动发生在音频、USB、
LED 分配后。当前张量内部 SRAM 预算默认最多 256 KiB，还会
按内部总空闲量减去 64 KiB 系统余量、最大连续空闲块减去 4 KiB 分配余量的
较小值自动限制，并向下对齐到 16 字节；其余张量用 PSRAM。
系统余量不再从单个连续块扣除，避免热切换后总空闲量充足、但最大块不足 64 KiB
时预算错误归零。每次加载（包括 A/B 激活和回退）都会重新计算预算。
预算在 ESP-DL 完成模型图/算子和张量元数据加载后测量，并立即占用 SRAM arena；
后续规划临时内存不能抢走这个连续块，最终张量分配复用该 arena，由模型销毁统一释放。
`espdl_budget.cmake` 在构建目录生成带延后预算及 arena 复用的 ESP-DL 源码副本；
只有当前任务正在构造 KWS 模型时启用该路径，其他模型及其他任务沿用原参数。
不修改 managed component；依赖源码布局改变时构建会报错，需审核补丁。
实际预算会记录在启动日志中，可在 `AFE keyword recognition` 菜单调整。

KWS 上下文（含 MFCC history/FFT，大小由加载日志给出）显式使用内部 SRAM，绕过普通
malloc 的 PSRAM 大小阈值。分配失败时返回 `ESP_ERR_NO_MEM`，不回退 PSRAM；错误与
销毁路径都使用匹配的释放函数。激活张量分配完成后，尝试将 PSRAM 中的模型参数复制到
16 字节对齐的内部 SRAM，保留至少 64 KiB 内部空闲内存；分配失败的参数继续使用原位置。
当前模型可迁移约 45 KiB，启动日志 `parameter promotion` 和 `model parameter_copy`
显示实际迁移量。权重数值和推理优先级不变；模型销毁时释放对应 SRAM 副本。

完成固件构建后，可用本机 C++ 编译器验证实际生成的 ESP-DL 分配函数（含 arena 复用、
分配失败回退和释放）：

```text
python src/kws/tests/test_tensor_arena.py --cxx zig c++
```

## 消抖和串口结果

排查判决问题时可发送 `p/P` 打开逐帧原始输出（再次发送关闭，默认关闭），格式：

```text
KL,seq,sil,unk,yes,no,up,down,left,right
KL,1234,-0.52,1.03,-1.10,0.20,2.40,-0.80,0.70,-1.20
```

按模型类别0–7排列，固定显示2位小数；仅串口显示舍入，推理和判决继续使用完整精度。
这些是 `kws_process()` 的原始输出，
没有 softmax、阈值、预热或消抖过滤。每个成功的 10-ms 推理帧均尝试输出，
失败帧不输出未初始化 logits。`seq` 是模型输入块序号，不是串口到达时间。
推理只零等待复制至独立 32 帧队列，后台格式化/发送，关键词结果使用原队列并优先处理。
常规 binary32 logits 使用整数路径精确舍入到两位小数，与 `%.2f` 的最近偶数舍入一致，
保留负零；异常大幅值回退原格式化路径。UART 控制台整行提交驱动，保留配置的换行格式，
避免逐字符获取驱动信号量。日志任务位于 core1、优先级1，低于 DAS、NsCore 和 DOA。
该调试开关独立于 `l/L` 和 USB 自动静默，推荐 `l` 关闭普通日志后再 `p` 开追踪。
115200 下带宽余量较小，拥堵时丢弃调试帧并显示 `KD,n`（累计丢失）；关闭时显示
`KL OFF tx=... drop=...`。序号可跳变，调试帧丢失不触发模型复位，也不改变识别条件。
关闭 raw 后清空待发调试队列，推理任务跳过调试提交，后台跳过该队列读取、数值格式化和
丢行统计分支；不再为 raw 唤醒日志任务，只保留必要的开关读取。关闭时已开始的一帧可能
完成收尾，已提交 UART 的字节会继续发送。KWS 推理及关键词判决仍正常执行。

当前判决采用连续帧确认、softmax 概率门限、全局冷却和同词去重：

- 当前模型的所有类别参与 top-1 和 softmax，包含 silence、unknown 和关键词类。
  使用 `exp(logit[i] - max_logit)` 防止指数溢出，门限作用于 top-1 关键词概率。
- 初始化或音频缺口后预热100个有效帧（1秒音频），预热期间也累计合格候选。
- 同一关键词连续N帧、每帧概率均达到门限才可触发，默认N=3、门限80%。
  换类、低于门限或无效数值都会中断连续计数。
- menuconfig → `KWS-Filter` → `Need-VAD-Checked` 提供 `No` / `Yes`，默认 `No`。
  `No` 保留现有判决；`Yes` 要求上述 N 个连续关键词帧的 VAD 全部为 speech/high。
  其中任一帧为 silence/unknown/低电平都会清空关键词确认计数，从下一个合格帧重新计满 N 帧。
  例如 N=3 时，`高、高、低、高、高、高` 只能在最后一帧完成 VAD 连续确认。
  概率门限、预热、冷却和同词去重仍同时生效；silence/unknown 类别本身的去重确认不要求 VAD 为高。
- 关键词动作事件成功入队后启动全局冷却，默认500 ms，期间所有关键词均不能触发，
  也不累计关键词确认帧，冷却结束后重新计帧。入队失败不启动冷却，下一匹配帧可重试。
- 保留同词去重：持续同一词不重复输出。silence/unknown 各自连续N帧更新去重状态，
  不合并计数、不使用关键词概率门限、不输出关键词日志。
  背景可在冷却期间更新状态，但不能缩短冷却；冷却结束且背景已稳定确认后，同词可再次触发。
- 连续帧数按10-ms音频块累计。冷却使用单调墙钟，从动作事件成功入队后计时，
  不因处理积压而压缩，也不因输入缺口或推理错误重置而提前解除。

softmax 属于识别判决，与 raw 日志开关无关；`p/P` 仍输出未经 softmax 的原始 logits。
20-ms调度窗口及2-tick暂停策略继续保留；VAD 只在选择 `Yes` 时参与关键词确认。

menuconfig 顶层 `KWS-Filter` 提供：

| 参数 | 默认值 | 范围及含义 |
| --- | --- | --- |
| Keyword probability threshold (%) | 80 | 0～100；80即0.80，连续确认的每帧均须达标 |
| Global cooldown after a keyword trigger (ms) | 500 | 1～60000 ms |
| Consecutive keyword frames to trigger (10 ms/frame) | 3 | 1～1000；默认3帧对应30 ms音频，背景沿用该计数 |
| Need-VAD-Checked | No | Yes：每个连续关键词确认帧都必须具有全程高 VAD；低或未知会重新计数 |

对应 `CONFIG_KWS_FILTER_KEYWORD_THRESHOLD_PERCENT`、`CONFIG_KWS_FILTER_COOLDOWN_MS`
和 `CONFIG_KWS_FILTER_STABLE_FRAMES`。这些是首次启动或无有效 NVS 记录时的默认值；
修改编译默认值需要重新编译、烧录。
VAD 选项对应 `CONFIG_KWS_FILTER_NEED_VAD_CHECKED_NO` / `CONFIG_KWS_FILTER_NEED_VAD_CHECKED_YES`，
它也是编译默认值。四项均可在运行中通过 [自定义 HID 的 SET_KWS_FILTER](../custom/README.md#kws-判决参数运行中可改并立即持久化)
或 `src/custom/tools/kws_filter_gui.py` 修改；写入 NVS 成功后立即生效，掉电保留，不需再发 `SAVE_CONFIG`。

VAD 随 160 点音频块进入 KWS 队列，推理不读取处理积压之后的全局 VAD。
NsCore 输入采用已补偿 96 点 OLA 延迟的逐样本 VAD；一个 10-ms 块中只要存在低或未知标记，该块就不合格。
USB 播放参考输入绕过 AFE fetch，使用提交参考音频时最新发布的 AFE VAD 快照；该快照并非播放源自身的 VAD，
`Yes` 时即使播放参考能识别出关键词，AFE VAD 未报告语音仍不会触发。
两种选择均持续输入和推理，不裁剪 PCM、不改变 Capture2 传输，也不改变 `p/P` 原始 logits 输出。

输出示例（时间戳仅示意）：

```text
I (12345) kws: keyword class= 2
```

内嵌模型和 HID 下载模型的触发日志统一只输出类别索引，不包含 label。
内嵌模型类别顺序为 `0=silence, 1=unknown, 2=yes, 3=no, 4=up, 5=down, 6=left, 7=right`；
HID 更新模型的 2 及以上类别以配套完整类别表为准。推理使用当前模型的实际类别数量。
推理任务确认关键词后零等待提交给独立的 `keyword_actions` 队列并开始冷却、去重；
动作被禁用或外设队列满不会改变识别判决，也不等待 LED、USB 或串口。
识别日志仍通过 `audio_diagnostics` 后台格式化和发送；日志静音、串口隔离或日志队列满
不影响已经接收的灯效与快捷键，也不会因为日志失败而重复发送按键。
未迁入参考工程的 USB 输入、录音脉冲、控制台或直接 `printf/fflush` 路径。

## 关键词颜色与快捷键

内嵌兜底模型的初始映射位于 [`../custom/custom_state.c`](../custom/custom_state.c)，如下；
活动映射由控制 HID 实时维护：

每次上电或复位后，全部关键词的动作、灯效和键盘开关均为禁用，包括恢复已保存配置时。
下表仅为预置颜色和键位。UART0 发送 `a/A` 开启全部动作及灯效（有键位才开启键盘），
`x/X` 关闭；也可通过控制 HID 的 `SET_BINDING` 按词开启。保存开启状态后重启仍默认禁用。

| 关键词 | RGB | 颜色 | 默认按键 |
| --- | --- | --- | --- |
| yes | 0, 255, 0 | 绿色 | y |
| no | 255, 0, 0 | 红色 | n |
| up | 255, 255, 0 | 黄色 | u |
| down | 0, 0, 255 | 蓝色 | d |
| left | 0, 255, 255 | 青色 | l |
| right | 255, 0, 255 | 品红 | r |

silence 和 unknown 只参与原有消抖/重新使能，不触发灯效或按键。
每次确认关键词后，后台任务同时向现有 LED、HID 工作队列提交动作：B 组灯环按绑定颜色
闪烁 600 ms（100-ms 周期、50% 占空比），之后恢复被打断的模式；新关键词立即换色并重新计时。
此处显式使用 600 ms，不受通用关键词灯效默认时长配置影响；仍遵守全局 LED 亮度配置。
HID 发送无 Shift 的小写字母键，并在按下报告完成后发送全零释放报告。

HID modifier 位掩码和最多六个同时按下的 keycode 可通过控制接口实时修改，无需重新编译。
每个关键词独立配置动作总开关、灯效开关和键盘开关；禁用动作不改变识别或主机事件。
完整模型更新同时下发类别索引和注释文本，灯效/键盘配置独立管理；
新模型包默认禁用动作，不按旧索引继承绑定。详见 [`../custom/README.md`](../custom/README.md)。

主机回归：`python src/tests/test_keyword_actions.py --cc <C 编译器>`，覆盖实时映射、
禁用时旧动作取消、按下/释放、维护状态和队列饱和；另有 custom 协议和状态测试。

## RTF

每帧记录完整 `kws_process()` 墙钟耗时，包含 FP32 MFCC、输入量化、网络推理、
logits 转换、反馈更新和任务被抢占时间；不含等待输入、PCM 转换、消抖或日志。

```text
RTF = 累计成功处理耗时 / (成功处理帧数 × 10000 us)
```

`RTF < 1` 表示此统计窗口内平均处理速度快于音频输入；同时检查最大耗时和丢样量。
综合链路是否实时还要看队列丢样、USB 计数及实际录音。新增负载下的 RTF 必须上板实测，
参考例程的旧测量值不能代表本工程。

沿用现有优先级 1 的音频统计任务，约每 10 秒输出：

```text
kws: perf chunks=... avg=... ms max=... ms RTF=... dropped=... resets=... errors=...
kws: cpu task_avg=... ms task_RTF=... core0_share=...% interval=... s (whole-task estimate)
audio_dsp: DAS task CPU=...% runtime=... ms interval=... s core=1 priority=6
```

`CONFIG_AUDIO_DIAGNOSTICS_QUIET_WHILE_STREAMING=y` 时普通性能日志也静音，统计持续
累积，所有 USB 音频接口关闭后下一次统计输出包含整个静音期间的数据。需要在 USB
开流时查看周期 RTF，可在 menuconfig 关闭此项；UART 发送优先级仍为 1。
也可发送串口字符 `l` / `L`，在“仅 KWS 识别结果”和“普通日志打开”之间切换。
后者覆盖开流静默策略，前者连警告、错误和 KWS 性能统计也屏蔽；两种状态都保留关键词结果。
`chunks/avg/max/RTF` 为自上次允许输出以来的窗口，`dropped/resets/errors` 为启动以来
累计值。`kws_stream_get_stats()` 提供完整生命周期计数，供调试器或其他模块读取。
错误帧不计入成功帧的 RTF，会增加 errors、重置状态并重新预热。

`task_RTF = 本窗口 KWS 任务实际运行时间 / (成功帧数 × 10 ms)`，`core0_share`
是该任务占单核时间的百分比。CPU 运行时间通过现有 FreeRTOS 微秒计数器在后台采样，
不对每帧调用调度器接口。USB 静音期间仍每 10 秒采样并累计 CPU 时间，处理 32-bit
计数器回绕。它覆盖整个任务（包括转换、队列操作、状态重置及失败推理），不是单独
神经网络耗时；窗口边界可能落后一个任务运行片段，ISR 时间也可能计入任务。
`task_avg` 和 `task_RTF` 因此用于窗口级判断，首个窗口还包含启动边界。

启动时会记录内部堆空闲量/最大连续块、实际张量预算、MFCC 上下文及 history/FFT
所在内存区域、模型 variable/parameter/parameter_copy 的 SRAM/PSRAM/flash 字节数。
`parameter_copy` 是 ESP-DL 另行复制的参数，分析时不要简单把各项当作独立内存池相加。

## 采集调度与内存诊断

保持当前 W16A16 权重、任务优先级和内存预算，先做基线测试。关闭 PC 的 USB 音频
播放/录音接口，使普通诊断可见；I2S、DAS、AFE 和 KWS 仍持续运行。退出占用目标
串口的监视器后，在 ESP-IDF Python 环境执行：

```text
idf.py build
python src/kws/tools/profile_kws.py --port COM4 --flash --seconds 65 --output .codex-tmp/kws-profile-baseline
```

首次升级到 VADNet 版本先执行完整 `idf.py -p <串口> flash`，安装分区表和 VAD 模型。
之后脚本只烧录应用区，保存启动和运行日志，以及 JSON/Markdown 统计。
烧录后使用软件 watchdog 复位并立即打开串口；不烧录时尝试 RTS/EN 复位。
串口号按实际板卡调整。若 UART 转接器未接自动下载信号，先手动进入 BOOT 下载模式，
再增加 `--before no-reset`。去掉 `--flash` 可采集已烧录固件，但未接 RTS/EN 时需手动复位。
已有日志可离线整理：

```text
python src/kws/tools/profile_kws.py --input-log .codex-tmp/kws-profile-baseline.log --output .codex-tmp/kws-profile-summary
```

舍弃第一个窗口，观察后续至少 3 个完整窗口：墙钟 RTF 远高于任务 CPU RTF，支持
抢占/阻塞时间占比较高；任务 CPU RTF 本身也较高，则需要结合内存位置排查计算与
访存成本。SRAM 位置本身不能证明性能因果，必要时再做单因素对比。
采集与打印均不改变原任务优先级，UART 仍由优先级 1 后台任务发送。

也可以手动烧录 `build/esp32p4_usb_audio_hs.bin` 并复制串口日志，不必运行采集脚本。
115200 baud，从复位开始保留 `kws_model` 内存日志，USB 音频接口关闭时连续记录约
60～65 秒，至少包含 4 组 `kws: perf`、`kws: cpu` 和 `DAS task CPU`。
先依据这些数据区分调度等待和任务执行成本，再决定是否需要改变内存或调度设置。

上下文已显式固定在内部 SRAM。手动烧录后先确认日志出现
`MFCC context=... bytes region=SRAM history=SRAM fft=SRAM ... forced_internal=1`，
并检查每次激活后 `model variable` 的 PSRAM 是否仍为 0，以及
`feedback state tensors: SRAM=12 PSRAM=0 total=12`。PSRAM XIP 只改变代码和只读数据
的执行/存放位置，不要求热点权重或工作张量放在 PSRAM。当前已将波束形成改为 DAS、DOA 固定
为 SRP-PHAT。剔除首个窗口，再与原波束形成链路的任务 CPU RTF 约 0.63～0.64、
墙钟 RTF 约 1.8 比较；效果以实板数据为准。

## 替换同结构权重

所有需要随模型维护的文件集中在 `src/kws/model/`：

| 文件 | 用途 |
| --- | --- |
| `ds_tc_resnet_backend_w16a16_esp32p4.espdl` | W16A16 神经网络权重 |
| `kws_frontend_data.h` | FP32 窗函数、Mel、DCT 和归一化表 |
| `model_config.hpp` | 输入输出名、状态配对、类别文本、采样率和消抖参数 |

仅权重改变且结构、类别顺序、MFCC 前端与 W16A16 量化精度保持一致时，覆盖
`ds_tc_resnet_backend_w16a16_esp32p4.espdl`，再运行 `idf.py build` 和
`idf.py -p <串口> flash`。模型嵌入应用固件，
内嵌 KWS 权重作为出厂兜底；HID 更新使用独立 `kws_a/kws_b` 分区，AFE 的 VADNet 保留在 `model` 分区。
CMake 跟踪文件变化，以固定符号嵌入，替换 KWS 权重无需改推理代码。
不要使用包含量化音频前端的旧完整模型。

初始化会校验特征数 40、类别数与当前完整类别表一致、量化 dtype、6 对反馈张量的元素数和数据指针。
模型的特征提取定义、类别顺序或状态名称发生变化时，需要同步这里的表和代码。

## 验证

主机测试可独立运行：

```text
cmake -S src/kws/tests -B build-kws-tests
cmake --build build-kws-tests --config Release
ctest --test-dir build-kws-tests -C Release --output-on-failure
```

覆盖 AFE 512 点分帧、跨块连续性、丢帧序号、异常后的残帧丢弃、预热边界、
8类参与概率计算、概率边界与数值平移不变性、连续确认、重复抑制、稳定背景、结果队列失败重试，
以及全局冷却边界、冷却期不积帧、重置不绕过冷却和零冷却配置，
同时检查 INT8/INT16 转置边界。运行时测试直接执行真实
`kws_stream.cpp`，模拟队列和模型，检查非阻塞发送、PCM 满量程转换、丢帧/错误重置、
RTF 计数、静音时保留统计和初始化失败后的资源释放；慢推理/USB 输入运行时用例覆盖
持续积压、AFE 抢占和推理错误时的按时间暂停，以及 tick 边界的最短调度窗口。
现有 `src/tests/test_audio_diagnostics.py`
覆盖 USB 静音时关键词仍通过后台线程输出，以及日志满队列不阻塞。
`src/audio_processing/tests/test_kws_input.py` 执行真实 AFE feed 任务，覆盖两种 KWS 源与
三种 Capture2 左声道组合、分次读取、参考 PCM 保真、停播补零及 AFE 输入不被修改；
`test_post_ns_pipeline.py` 同时验证两种 KWS 源的生产者和重置隔离。

本次已通过 ESP-IDF v6.1.0 默认 INT16 固件构建、上述五项 CTest、日志框架两种静音
配置，以及既有 Capture2 数据包/描述符/时钟偏差模拟和 TinyUSB DMA/PTI 回归。
权重及 MFCC 参数与参考工程 SHA-256 一致。尚未烧录，合并链路的实际 RTF、内存占用、
识别准确率及录音连续性仍需实板确认。
