# KWS SRAM 优化与实板验证（2026-10-20）

已完成修改、编译、COM6 自动复位烧录及复测。保留 PSRAM XIP，恢复当前模型热切换后的
工作张量 SRAM 分配；最后恢复内嵌模型及原关键词绑定，并再次复位确认 ready。

## 固件与测试条件

- ESP32-P4，CPU 360 MHz，32 MiB PSRAM，ESP-IDF v6.1；控制 HID 为 `303A:8003`。
- 应用：`build/esp32p4_usb_audio_hs.bin`，烧录后哈希校验通过。
- SHA256：`7d16012b7a9ba0cef1a8e2cb2f30f5d013ff75cf3deaae7ced980e160fbe8561`。
- 仓库 W16A16 模型，8 类（silence、unknown、6 个关键词），模型文件 138064 字节。
- 使用 `E:/Sony/mini_speech_commands` 的 yes/no/up/down/left/right 语音循环，USB 播放参考送入 KWS。
- 同时开启 Capture1 8ch/24 kHz、Capture2 2ch/16 kHz；Capture2 左声道为后 NS 的 1-kHz 诊断正弦波。
- 前后均开启性能日志、执行 15 秒设备查询/实时绑定编辑、完整模型下载和显式激活。
- 优化前基线采于 2026-10-19，最终复测为 2026-10-20，使用同一开发板、同一模型和相同配置。

## 修复机制

原算法从最大连续空闲块扣除 64 KiB，导致总空闲约 256 KiB 时，热切换预算仍可能归零。
仅提前计算更大的预算也不可靠：模型图、张量元数据及规划临时内存都会改变连续块。

现在在模型图与张量元数据加载后计算预算：总空闲 SRAM 保留 64 KiB，满足 SIMD 的
最大连续块另外留 4 KiB，并立即实际分配 arena。规划临时内存无法抢占该块；最终
张量分配复用 arena，模型销毁统一释放。分配失败会记录警告并使用 PSRAM。
该路径仅在构造 KWS 的当前任务内启用；其他模型保持原预算。

`kws/espdl_budget.cmake` 在构建目录生成 ESP-DL 分配源文件副本，未修改 managed component。
源码布局变化时构建报错，需审核该适配。权重、MFCC 算法、输入源、任务优先级未改变。

| 热切换后的内存 | 优化前 | 优化后 |
|---|---:|---:|
| 工作张量 SRAM arena | 0 B | 63488 B |
| 工作张量 PSRAM | 57904 B | 0 B |
| MFCC 输入 / logits | PSRAM / PSRAM | SRAM / SRAM |
| 12 个反馈状态张量 | 工作区在 PSRAM | 全部 SRAM |
| 显式权重迁移 | 45016 B → SRAM | 45016 B → SRAM |

arena 包括规划余量，不能把 arena 与 PSRAM 分配大小的差值解释为模型参数增加。
完整模型包和 XIP 指令/只读数据仍在 PSRAM；未来更大模型可将超出预算的张量放入 PSRAM。

## 性能前后对比

取下载模型激活后、下一次切换前的完整约 10 秒窗口：优化前 2 个，
优化后 3 个，平均处理耗时按帧数加权；排除跨越维护暂停的窗口。
表中处理耗时含 MFCC 与模型推理，CPU share 为整个 KWS 任务的运行时间估计。

| 指标 | 优化前 | 优化后 |
|---|---:|---:|
| 每 10 ms 音频块平均处理耗时 | 4.223 ms | 3.598 ms |
| 对应 RTF | 0.422 | 0.360 |
| KWS 占 core0 时间 | 39.75% | 33.87% |
| 所选窗口最大处理耗时 | 15.175 ms | 11.319 ms |

平均耗时改善 **14.8%**，core0 占用减少约 **5.88 个百分点**。
整场优化后测试的最大单块耗时为 **11.758 ms**；
所选窗口的最大值不代表所有帧都低于 10 ms，也不能代替未来模型的性能验收。

## 双录音与反复切换

证据：`.codex-tmp/custom-board/sram-verified-20261020/summary.json`、`uart.log`、`capture1.wav`、`capture2.wav`。

- Capture1 录制 **196.32 秒**，Capture2 录制 **196.66 秒**，30 项自动检查全部通过。
- 额外轮换内嵌/A/B **9 次**，均恢复 ready，boot_id 不变。
- 本轮记录的 11 次激活/恢复均为工作张量 PSRAM=0、反馈张量 SRAM=12。
- 内部空闲量在 132311～132507 B，最大连续块
  51200～51200 B，未见随切换次数持续下降。
- 正常阶段正弦波突变数 0；下载维护稳态为全零 PCM；两路录音均无主机溢出。
- Capture2 完成 196186 包；源欠载、零长度包、失败包、异常包长、
  FIFO 背压/短写、播放欠载/丢字节/I2S 错误均为 0。
- KWS 丢样、错误和非预期复位均为 0。

下载/激活后的 warming_up → ready 单独记录为恢复阶段；正常波形观察从 ready 后开始。
恢复静音的起止边沿属于预期行为，不计入正常流突变；USB 源欠载和异常包计数仍全程检查。

较早一轮仍用 48 ms 维护后预填，长测出现 2 次 Capture2 源欠载，其中一次造成正常区间
正弦波突变。最终改为 **维护后先积累 96 ms 新数据再结束静音**，覆盖 AFE 重置后的
起始突发；正常开流预填及稳态目标仍为 48 ms，恢复后逐渐回到该目标。
代价是维护后恢复声音可能额外延后约 48 ms。普通设备查询/绑定编辑不进入该恢复流程。

## 恢复验证及主机回归

`.codex-tmp/custom-board/sram-recovery-model-20261020/summary.json` 验证：

- 下载另一槽途中经 COM6 EN/RTS 复位，原有效槽重新加载并 ready，未完成槽无效。
- 清单类别数与实际 logits 不匹配时激活失败，自动恢复旧模型。
- 错误 SHA 被拒绝，完整有效的另一槽仍可下载并激活。
- 测试完成恢复内嵌模型和原 RAM 绑定；本轮使用 `--skip-save`，未重测配置保存策略。

最后 `.codex-tmp/custom-board/sram-final-smoke/summary.json` 再次复位后确认内嵌模型 ready，
全部关键词绑定与优化前一致，无待保存配置。

主机通过 7 项 KWS CTest、3 项协议/状态/存储 CTest、XIP 地址分类、实际生成的 ESP-DL
arena 复用/分配失败/释放测试、维护零填充和 96-ms 恢复门限测试，以及 Capture2 包长、
时钟偏差/突发抖动、FS/HS 描述符和 ISR 无浮点指令检查。

## 复现

```text
python src/custom/tools/board_validate.py update --port COM6 --reset --capture-raw --all-logs --steady-seconds 35 --reload-cycles 9 --output .codex-tmp/board-sram
python src/custom/tools/board_recovery_validate.py --port COM6 --skip-save --output .codex-tmp/board-sram-recovery
python src/custom/tools/board_validate.py smoke --port COM6 --reset --output .codex-tmp/board-sram-final
```

性能窗口、内存记录和波形图见 `.codex-tmp/custom-board/sram-comparison/summary.json`
及 `validation.png`；优化前证据为 `.codex-tmp/custom-board/sram-before-steady/`。

![性能前后对比与 Capture2 波形](../../.codex-tmp/custom-board/sram-comparison/validation.png)

PCMD3180 仍未在 0x4c 应答，当前为软件静音采集回退。上述结果验证 USB 激励 KWS 和诊断
音频连续性，不代表真实麦克风/ADC、声学 AEC/NsCore 音质或物理随机掉电已通过验收。
