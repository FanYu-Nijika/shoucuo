# TC264 摄像头四轮通用核心

统一入口为 `camera_car.h`，包含类型化 algorithm、滤波、PID、图像处理、赛道检测、
四电机混控、编码器、环形缓冲和软定时器。所有接口使用 `cc_` 前缀，不依赖具体引脚。
摄像头、PWM 与时间函数的板级边界见 `tc264_port.h`。

推荐图像链路：摄像头帧 → ROI/Otsu → 可选缩放/Sobel → 多行赛道检测 → PID。
Otsu 对全黑、全白和无有效 ROI 的情况会使用调用者给定的回退阈值；Sobel 明确拒绝
原地覆盖，避免读取到已经写坏的邻域像素。

VSCode 仅用于代码阅读、函数补全和头文件跳转；工程没有配置 VSCode 编译任务。
真实 TC264 引脚、DMA、PWM 和编码器通道应在接线确认后再适配，不要写进算法层。

## 摄像头与硬件资源默认策略

- 本模板固定使用单路 MT9V03X 摄像头。驱动位于
  `libraries/zf_device/zf_device_mt9v03x.c/.h`，默认图像尺寸为 188×120。
- 摄像头说明书、芯片手册、供电建议和尺寸图已随项目保存在 `docs/MT9V03X`，复制整个
  项目后不再依赖外部的 `MT9V03X_Product` 文件夹。
- 驱动中的引脚、ERU 和 DMA 配置是逐飞示例值，必须根据实际接线确认后再修改。
- 本模板不引入双摄像头库，因为双摄会额外占用一组 DVP、ERU 和 DMA 资源。
- `总钻风摄像头FPC排座封装.7z`、`TC264无刷驱动封装库.7z`、`四路舵机电源模块封装.7z`
  内是 Altium 的 `.PcbLib/.SchLib/.SCHLIB` 封装文件，不是 C/ADS 依赖。它们分别只在
  设计摄像头 FPC、无刷驱动板、舵机电源板时使用，不复制到 `code` 或 `libraries`。

## 模糊 PID 使用顺序

```c
cc_fuzzy_pid_t steering_pid;
cc_fuzzy_pid_config_t fuzzy_config;

cc_fuzzy_pid_config_default(&fuzzy_config);
fuzzy_config.kp = 1.2f;
fuzzy_config.ki = 0.05f;
fuzzy_config.kd = 0.08f;
fuzzy_config.error_full_scale = 1.0f;
fuzzy_config.error_rate_full_scale = 5.0f;
cc_fuzzy_pid_init(&steering_pid, &fuzzy_config);

/* 主循环中使用归一化中线误差，dt 单位为秒。 */
steering = cc_fuzzy_pid_update_error(&steering_pid, line_error, dt);
```

先把普通 PID 的基础 `kp/ki/kd` 调到车辆能够稳定运行，再逐渐增加三个
`*_adjust_max`。不要一开始同时大幅调整基础增益和模糊增益。

## 补线和状态机使用顺序

```c
cc_track_state_detector_t detector;
cc_track_features_t features;
cc_track_repair_config_t repair_config;
cc_track_repair_result_t repair_result;
cc_track_state_t state;

cc_track_state_init(&detector, 0);
cc_track_repair_config_default(&repair_config);

/* 每帧先用原始边界识别状态，不能先补线，否则会把真实丢边特征盖掉。 */
features = cc_track_features_from_edges(left_edge, right_edge,
                                        image_height, image_width,
                                        -1, zebra_transitions);
state = cc_track_state_update(&detector, &features);

/* 状态识别完成后再补线，补出的中线只用于转向控制。 */
repair_result = cc_track_repair_edges(left_edge, right_edge,
                                      image_height, image_width,
                                      &repair_config);
cc_track_build_centerline(left_edge, right_edge, center_line,
                          image_height, image_width, -1);
```

`left_edge`、`right_edge` 和 `center_line` 使用 `int16_t` 数组，无效行写 `-1`。
斑马线的 `zebra_transitions` 是选定横向采样线上的黑白跳变次数；不检测斑马线时传 `0`。
