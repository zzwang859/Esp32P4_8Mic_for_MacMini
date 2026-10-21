# 开流保存实板闭环：2026-10-20

设备：ESP32-P4，控制 HID `303A:8003`，序列号 `30EDA0E206CB`，复位/下载串口 COM6。
现场使用内嵌 KWS 模型；A/B 槽均有效。

## 原因与修复

旧版 `SAVE_CONFIG` 在播放接口开启时返回 ACCEPTED，但等待接口关闭才写 Flash。
Windows 保持该接口打开时，`save_pending` 一直为 true，后续保存返回 BUSY。
通过 `board_save_validate.py --expect-deferred` 在实板复现了该完整过程；
关闭播放接口后，原请求才输出 `SAVE_CONFIG complete status=OK(0)`。

当前保存只等待模型更新事务结束，不依赖播放、原始录音或 Capture2 接口状态。
写入仍在优先级 1 的控制任务中执行，低于 USB 音频、DSP、AFE、NsCore 和 KWS 任务；
保存不进入音频维护、不主动暂停/重置处理，也不发送维护静音。
受理日志改为 `save_priority=1 stream_mask=...`，不再显示接口空闲等待标志。

## 固件和下载

- 应用：`build/esp32p4_usb_audio_hs.bin`，2882288 字节。
- SHA256：`d94581f44f4910a95a112d96596458fd390425b17901bfe9b4addd352ed6565c`。
- 仅写入应用地址 `0x10000`；esptool 写入哈希校验通过。
- 未擦除 NVS、A/B 模型槽或语音资源分区。

## 开流保存结果

使用 `tools/board_save_validate.py`，保持每组接口开启并连续执行两次“修改绑定 → 保存 → 等待完成事件”。
修改包含 RGB、modifier、键码和保持时长；动作开关保持关闭，避免测试期间触发主机快捷键。

| 实测 stream_mask | 接口 | 两次保存的完成轮询耗时 | 结果 |
|---|---|---|---|
| `0x01` | 播放 | 109 / 109 ms | 均成功，无 BUSY |
| `0x04` | Capture2 | 172 / 109 ms | 均成功，无 BUSY |
| `0x06` | 原始录音 + Capture2 | 156 / 109 ms | 均成功，无 BUSY |
| `0x07` | 播放 + 两路录音 | 109 / 140 ms | 均成功，无 BUSY |

上述耗时包括查询状态的 HID 往返，不代表纯 Flash 写入时长。
八次保存均收到匹配 revision 的成功 SAVE 事件，`save_pending=false`、`config_dirty=false`。
保存窗口内，KWS 推理持续推进，错误、重置、丢样本计数均未增长；
USB 空包、失败包、异常尺寸、背压、短写及播放欠载计数均未增长，主机音频回调没有报告溢出/欠载。
没有触发 panic、断言、栈溢出或看门狗。

## 复位读回和最终现场

测试结束恢复用户原有六个关键词的颜色、键位和时长，再完成第九次保存。
随后通过 COM6 RTS 硬件复位，确认 boot_id 改变、KWS ready；
所有绑定参数与现场备份一致，动作、灯效、键盘开关均按启动规则关闭。
class 2 的 RGB 仍为 `[239,166,75]`，键码仍为 `0x1C`，灯效时长 600 ms。
活动模型仍为内嵌模型，A/B 槽的包摘要和有效状态未变化。
最终 `save_pending=false`、`config_dirty=false`。

证据：

- `.codex-tmp/save-stream-live/scene-before.json`：修改/下载前现场备份。
- `.codex-tmp/save-stream-live/reproduce/summary.json`、`uart.log`：旧版复现。
- `.codex-tmp/save-stream-live/build.log`、`flash.log`：编译和下载校验。
- `.codex-tmp/save-stream-live/verify/summary.json`、`uart.log`：八次开流保存、恢复和复位读回。

这次检查覆盖传输计数、音频回调和复位持久化，没有验证声学音质或真实断电场景。
