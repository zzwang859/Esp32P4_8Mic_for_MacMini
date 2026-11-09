# ESP32-P4 WS2812B 灯环

本目录使用 ESP32-P4 的 RMT TX + GDMA 驱动 WS2812B，并在独立低优先级 FreeRTOS 任务中运行顶层灯效状态机，不用软件延时翻转 GPIO。

## 默认配置

- 数据输出：GPIO26。GPIO24/25 留给计划中的 Full-Speed USB D-/D+。
- 灯珠数量：物理串联 50 颗，颜色线序：GRB。A 组物理编号为 0、1、11、17、23、29、35、41，其余 42 颗组成 B 组逻辑圆环。
- 刷新率：100 FPS。
- 全局亮度：20%。
- 启动自检：每次上电/复位后，A、B 两组全部 50 颗灯同时依次显示红色 2 秒、绿色 2 秒、蓝色 2 秒，共 6 秒，亮度受全局亮度参数限制。自检期间尚未启动灯效/输入任务及音频、USB、关键词等主功能。
- 自检完成后的效果：顶层模式 0，B 组绿色呼吸叠加录音红色分量与放音蓝色分量，A 组关闭。
- 音频阈值：默认 -45 dBFS；阈值以下对应颜色分量归零。
- 关键词模式：颜色由每次触发调用传入，默认闪烁 1 秒，然后返回触发前的模式。
- KWS 识别使用独立映射，上电/复位后灯效与按键默认禁用；串口 `a/A` 或控制 HID 开启后按绑定颜色闪烁并发送快捷键，`x/X` 关闭。预置闪烁时长为 0.6 秒，详见 [关键词映射](../kws/README.md#关键词颜色与快捷键)。
- 呼吸亮度：`0.04 + 0.96 * ((sin(2πt/period - π/2) + 1) / 2)^2`，默认周期 6 s。
- RMT 分辨率：20 MHz；WS2812B 数据速率：800 kbit/s；帧尾由 RMT 生成 80 us 低电平复位码。

运行 `idf.py menuconfig` 后在 `WS2812B LED ring` 菜单中可以修改 GPIO、灯珠数量、刷新率、全局亮度、音频阈值和关键词默认持续时间，也可以整体关闭该功能。关键词颜色由每次触发调用传入，不在 SDK 配置中固定。

## 接口

应用启动时由 `main.c` 调用：

```c
ESP_ERROR_CHECK(led_ring_start());
```

此调用会先阻塞执行 6 秒自检，完成后才启动正常灯效和输入任务并返回。
`main.c` 在音频后端和 USB 初始化之前调用它；关闭灯环功能时跳过该调用和自检。

运行期间可从其他 FreeRTOS 任务切换顶层模式：

```c
ESP_ERROR_CHECK(led_ring_set_mode(LED_RING_TOP_MODE_CONVERSATION));
```

顶层模式定义如下：

- 模式 0 `LED_RING_TOP_MODE_DEFAULT`：绿色呼吸；处理后录音驱动红色，双声道合并后的放音驱动蓝色。
- 模式 1 `LED_RING_TOP_MODE_CONVERSATION`：蓝色呼吸；处理后录音驱动红色，双声道合并后的放音驱动绿色。
- 模式 2 `LED_RING_TOP_MODE_KEYWORD`：B 组按指定颜色闪烁；它是暂态模式。
- 模式 4 `LED_RING_TOP_MODE_FAULT`：A 组红色 50% 亮度，B 组关闭。

自定义触发模式 2：

```c
const led_ring_rgb_t keyword_color = {.r = 255, .g = 96, .b = 0};
const led_ring_keyword_options_t keyword_options = {
    .duration_ms = 0, /* Use configured default: 1000 ms. */
    .has_next_mode = true,
    .next_mode = LED_RING_TOP_MODE_CONVERSATION,
};
ESP_ERROR_CHECK(led_ring_trigger_keyword(keyword_color, &keyword_options));
```

传入 `NULL` 可选参数时使用默认 1 秒；当 `has_next_mode=false` 时，模式 2 结束后
返回触发前的模式。每个关键词都应在调用时传入自己的 RGB 颜色。阈值可通过
`led_ring_set_audio_threshold_db()` 在运行时修改。

`ws2812.h` 还提供单像素、整环填充和 RGB 帧发送接口，适合绕过内置灯效直接显示状态。

## 接线与供电

- GPIO26 接灯环 `DIN`，ESP32-P4 与灯环电源必须共地。
- WS2812B 通常由 5 V 供电。线较长或 3.3 V 数据电平不稳定时，应增加 3.3 V 到 5 V 的高速逻辑电平转换器，并在数据线靠近灯环处串联合适的小电阻。
- 50 颗灯珠全白时电流可能达到数安培，必须使用独立且容量足够的 5 V 电源。20% 软件亮度只降低正常显示负载，不能代替供电限流、保险和电源去耦。
- 若通过 menuconfig 改 GPIO，需同时避开 TDM、I2S、I2C 以及 USB 所使用的引脚。
