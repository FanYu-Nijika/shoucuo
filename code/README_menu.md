# 五键简菜单说明

菜单实现位于 `car_menu.c`，只保留一个扁平节点数组。TFT 使用黑底文字显示，不依赖旧的 `menu_core`、`car_menu_app`、`car_menu_ui` 或 `car_overlay` 模块。

## 增删菜单节点

所有节点都在 `car_menu_items[]` 中，每个节点只占一行。节点字段依次为：

```c
id, parent, name, item_type, value_type, address,
minimum, maximum, step, decimals, apply, action
```

增加或删除一个变量时，只需要增加或删除对应的一行。普通参数的 `address` 直接写成 `&car_params.xxx`，菜单不会再建立参数副本或同步表。例如：

```c
{0, CAR_MENU_PAGE_RUN, "BASE SPEED", CAR_MENU_ITEM_VALUE,
 CAR_MENU_VALUE_I16, &car_params.base_speed, 0, 10000, 100, 0,
 CAR_MENU_APPLY_PARAMS, 0},
```

如果从结构体中删除了某个字段，也要同时删除菜单数组中引用该字段的行。Telemetry 节点直接读取 `car_result` 或现有运行状态变量，不能调节的节点使用 `CAR_MENU_ITEM_INFO`。

## 当前菜单内容

- `RUN CONTROL`：运行、停止、速度、PWM 限制、弯道减速、丢线停止帧数。
- `STEERING`：普通 PID 的 Kp/Kd、舵机中心、行程、反向和当前舵机指令。
- `VISION`：自动阈值、二值化阈值、摄像头曝光和增益。
- `MOTORS`：左右电机方向和当前左右电机指令。
- `TELEMETRY`：当前线有效状态、丢线计数、阈值、误差、舵机/电机指令、处理时间、帧同步信息和最长白列左右长度/列号。
- `IMAGE VIEW`：同时显示原始图像和二值化图像。
- `TOOLS`：LCD 彩条测试、恢复默认值、串口流开关。

## 五个主按键

按键为低电平有效，端口和上拉配置仍来自 `board_pins.h`：

| 开关 | 引脚 | 菜单功能 |
| --- | --- | --- |
| SW1 | P11.10 | 上移 / 调大 |
| SW2 | P11.12 | 停止时确认；运行时急停 |
| SW3 | P11.9 | 返回 / 取消 |
| SW4 | P11.11 | 下移 / 调小 |
| SW5 | P13.3 | 进入 / 保存 |

菜单只读取这五个主开关。AUX1（P11.2）和 AUX2（P11.3）保留为板级输入，但不参与菜单操作。

进入可调参数后，上下键会立即把值写回 `car_params`，因此 CPU1 下一帧可以直接使用新值；左键把本次编辑恢复为进入编辑前的值，右键或停止状态下的中心键完成保存。达到上下限后继续调节不会越界。浮点参数按节点的 `step` 和 `decimals` 显示。

运行状态不允许编辑参数。运行时按中心键直接停止电机并把舵机回到中心。`RUN` 还会检查摄像头和电机端口是否就绪；摄像头丢线达到 `car_params.lost_stop_frames` 后，现有车辆控制逻辑负责停车，菜单只更新状态提示。

## 图像和帧同步

原始图像来自 CPU0 已复制完成的 `car_gray_frame`。二值图像直接使用 `image.c` 中现有的 `binary_image` 指针，不在菜单中申请或复制图像缓存。只有 `car_frame_ready == 0` 且 `car_result_ready == 0` 时才访问这两个缓冲，避免 TFT 显示半帧数据。

如果算法分支把二值缓冲命名为其他符号，只需要在图像模块中让 `binary_image` 指向该现有缓冲，菜单本身无需修改。

## 调参建议

1. 停车进入 `STEERING`，先调整舵机中心和行程，再调整 `SERVO REVERSE`。
2. 在 `VISION` 观察原始/二值图像，先固定曝光、增益和阈值，再调普通 PID。
3. 从较低的 `BASE SPEED` 开始，逐步增加 `STEERING KP`，再增加 `STEERING KD` 抑制摆动。
4. 确认左右电机方向后再运行；`PWM LIMIT` 和丢线停止帧数应先设置为安全值。
5. 需要恢复启动参数时，在停车状态执行 `TOOLS -> RESET DEFAULTS`。

本次只重构菜单及其端口适配，没有修改 `car_params_t`、最长白列、普通 PID 或现有图像算法文件。
