# 实板验证记录：2026-10-19

已完成修改、编译、COM6 自动复位烧录和 USB 实板回归。
接通 USB HS 数据链路后，枚举与控制通信正常。

后续 SRAM 分配优化及当前固件的复测结果见 [SRAM 优化验证](SRAM_VALIDATION.md)。
播放/录音开流时的配置保存、连续下发及复位读回见 [开流保存闭环](SAVE_STREAM_VALIDATION.md)。

HID 串口日志开关已于 2026-10-20 通过 COM6 下载和实板验证：上电默认关闭，
`u/U` 可反复切换，普通日志 `l/L` 关闭后 HID 日志仍受独立开关控制。
关闭时 HID 查询、错误应答、SAVE 完成事件及实际持久化均正常；开启时可见操作、错误及保存完成日志。
在开关打开时硬件复位，确认重启恢复关闭，现场绑定参数和 A/B 槽保持不变，无 UART 故障。
证据：`.codex-tmp/hid-log-switch-live/summary.json`、`uart.log`、`flash.log`。

## 首轮验收固件和环境

- ESP32-P4 v1.0，16 MiB Flash，32 MiB PSRAM，CPU 360 MHz，ESP-IDF v6.1。
- 应用：`build/esp32p4_usb_audio_hs.bin`；完整分区已安装，烧录哈希校验通过。
- 应用 SHA256：`68b6f1b9d83f2317a49dbf7fc6b757373b1f3a8fa78c25cb736d9781af4a3ce3`。
- VID/PID `303A:8003`，控制 HID Usage Page/Usage `FF00:0001`，键盘为独立 HID 接口。
- KWS 输入为 USB 播放参考，Capture2 左声道为 NsCore 后替换的 1-kHz 正弦波。
- 激励来自 `E:/Sony/mini_speech_commands`，每个 yes/no/up/down/left/right 各两段 WAV 循环播放。
- Capture2 使用独占 WASAPI，8 通道原始录音使用 WDM-KS，播放使用 WinMM。

## 实板发现及修复

1. XIP 下 ESP-DL 对内部 SRAM 查询外部 MMU，导致 AFE 初始化断言。
   新增 `kws/kws_memory_compat.cpp` 链接包装，先识别本地内存，再查询映射地址；不改 managed component。
2. 关闭 PC HID 句柄不会使 USB 断线，旧序号重放缓存会影响新客户端。
   新增随机 nonce 的 OPEN_SESSION，清除旧会话快照、订阅和重放历史。
3. PCMD3180 未应答时使用软件静音时钟；原逻辑恢复后追赶暂停期间的期限，形成样本洪泛和看门狗。
   现于维护恢复时重建时钟期限，不补发暂停期间的音频。
4. 40-ms Capture2 目标在一轮双录音恢复测试中出现 6 次源欠载。
   预填及平均目标提高到 48 ms，增加 8 ms 余量；最终完整回归未再出现欠载或断点。
5. 本机 WinMM 共享录音把恒定正弦波在约 0.3 秒内压到接近静音。
   独占 WASAPI 保留 ±12000 PCM，验收工具因此改为独占输入，并检查信号幅度。

## 最终双录音和模型更新

证据：`.codex-tmp/custom-board/update-final/summary.json`，26 项自动检查全部通过。

| 项目 | 实测结果 |
|---|---|
| 并发接口 | USB 播放、8ch/24-kHz Capture1、2ch/16-kHz Capture2、自定义 HID |
| 两路录音 | 均为 68.0 s，无主机输入溢出报告 |
| 查询及修改阶段 | 15.36 s，204 次显式 JSON 对象读取，修改绑定 34 次 |
| 完整模型 + 清单下载 | 138064 B + 419 B，9.328 s |
| 下载维护期间 | 两路继续收包；Capture2 稳态区间为全零 PCM |
| 下载后活动模型 | 保持原模型，候选槽有效 |
| 激活候选槽命令 | 0.500 s，boot_id 不变 |
| 切回内嵌模型命令 | 0.375 s，boot_id 不变 |
| 正常阶段 Capture2 | ±12000，RMS 约 8485，二阶递推突变数为 0 |
| Capture2 完成包数 | 67916 |
| 录音源欠载、零长度包、失败包、异常包长 | 均为 0 |
| FIFO 背压、短写、播放欠载/丢字节/I2S 错误 | 均为 0 |
| KWS 丢样本、重置、错误 | 均为 0 |
| 看门狗、断言、崩溃 | 未观察到 |

维护边缘有预期的静音阶跃。正常阶段检查排除首尾 0.3 秒，维护稳态单独要求全零。
激活命令耗时不包含全部识别预热，主机仍应轮询 runtime 状态。
波形为同目录 `validation.png`，录音为 `capture1.wav`、`capture2.wav`。

## 关键词禁用与键盘

证据：`.codex-tmp/custom-board/keyboard-1/summary.json`。使用被钩子截获的 F24，避免交给桌面应用。

| 28 秒观察阶段 | class 2 识别事件 | F24 按下 | F24 释放 |
|---|---:|---:|---:|
| 动作启用 | 1 | 1 | 1 |
| 动作禁用 | 1 | 0 | 0 |
| 重新启用 | 1 | 1 | 1 |

禁用没有停止识别或识别事件。该轮录音 115.7 秒，正弦波突变为 0。
灯效颜色、时长及开关完成配置与读回验证，没有对物理灯光进行光学自动测量。
该轮使用采集时钟和水位修复前的版本，动作代码之后未改动。

## 故障恢复及保存

证据：`.codex-tmp/custom-board/recovery-model-final/summary.json`；
含 SAVE_CONFIG 的复测见 `.codex-tmp/custom-board/recovery-save-final/summary.json`。

- A 槽活动时，B 槽仅写入 240 字节模型即通过 COM6 硬件复位；重启仍使用完整 A 槽，B 槽无效，KWS ready。
- 声明 9 类、实际输出 8 类的完整包可下载，激活返回 LOAD_FAILED，回退 A 槽并恢复 ready。
- 错误 SHA256 返回 HASH_MISMATCH，中止后恢复正常处理。
- 完整重写 B 槽后可成功激活；结束恢复内嵌模型及原 RAM 绑定。
- 保存 class 2 的可辨认 RGB 值 `[17,34,51]` 后，通过下载中硬件复位检查完整绑定读回，一致；随后恢复测试前绑定。
- 用户实际拔下 HS 数据线约 2 秒后重新插入，重新枚举和读取成功；boot_id、活动模型及全部绑定均未变化，runtime ready，无待保存配置。
  证据：`.codex-tmp/custom-board/physical-reconnect/checks.json`。

上述实板测试版本的 SAVE_CONFIG 在音频接口开启时仅返回 ACCEPTED，等接口全部关闭才落盘。
当前代码已移除播放及录音接口的保存等待规则，写入仍由优先级 1 的控制任务执行；
上述历史测试不覆盖这一新行为，开流保存的独立验证结果另行记录。
本机曾因 Windows 保留播放会话（stream_mask=1）而持续排队，音频未暂停。
停止播放声音不等于接口关闭，ACCEPTED 也不表示已持久化；待保存时更新/激活会返回 BUSY。
可关闭占用该接口的应用或断开数据连接后再操作。

## 验证边界与最终状态

- PCMD3180 在 0x4c 未应答，原始输入为静音回退。已验证 USB 播放激励及诊断信号调度，
  尚未验证真实麦克风、ADC 和声学 AEC/NsCore 音质。
- Capture1 输入本来全零，不能仅凭波形证明物理 ADC 的维护静音；两路全长零填充另由实际 refill 函数的主机测试覆盖。
- 没有 USB 总线分析仪抓包，没有做完整识别准确率基准。
- EN/RTS 复位不等于物理断电，本次没有真正的随机掉电试验。
- 最后恢复内嵌模型和原关键词绑定，保留两个已验证候选槽；保留 USB 参考 KWS 和 1-kHz 诊断模式。

## 主机回归

3 个协议/状态/存储 CTest、6 个 KWS CTest、10 个 Python 客户端测试通过。
另通过 XIP 地址分类、长暂停采集时钟、维护零填充、灯环开/关两种动作队列构建、
Capture2 15/16/17 帧及 ±1000-ppm/4-ms 与 16-ms 抖动模拟、FS/HS 描述符、ISR 无浮点指令和 DMA/PTI 补丁回归。

协议与复现命令见 [README](README.md#windows-实板自动验证)。
