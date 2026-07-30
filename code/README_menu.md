# 五键简菜单与双核图像通道

这是当前 `shoucuo` 工程的菜单说明。菜单只服务于人机交互，不建立独立的 UI 数据模型：可调参数直接绑定 `car_params`，图像直接绑定双缓冲帧槽。

## 1. 菜单节点的增删改

所有节点集中在 `code/car_menu.c` 的 `car_menu_items[]`。节点字段为：

```c
id, parent, name, item_type, value_type, address,
minimum, maximum, step, decimals, apply, action
```

普通参数的 `address` 直接写成 `&car_params.xxx`。例如：

```c
{0, CAR_MENU_PAGE_RUN, "BASE SPEED", CAR_MENU_ITEM_VALUE,
 CAR_MENU_VALUE_I16, &car_params.base_speed, 0, 10000, 100, 0,
 CAR_MENU_APPLY_PARAMS, 0},
```

因此增删菜单项只需要在这个数组中增删一行，不需要同步第二张数据表。要修改调节范围、步长或小数位，也只改该行。删除 `car_params` 字段时，同时删除引用它的节点行即可。

菜单值的实际数据源始终是 `car_params` 或现有运行结果：编辑时的临时值只用于左键取消，不是参数副本。右键或停止状态下的中心键确认后，参数会直接写回并调用对应的应用动作。

当前显示的可调项包括速度、PWM 限制、弯道减速、丢线停车帧数、普通 `steering_kp/kd`、舵机中心/行程/反向、自动阈值、固定阈值、曝光、增益和左右电机方向。遥测项为只读节点，直接读取 `car_result`、运行状态或丢帧统计。

## 2. 五个主按键

按键使用普通 GPIO 轮询，低电平有效，上拉配置和引脚来源均为 `code/board_pins.h`。CPU0 每 10 ms 扫描一次，PIT 中断只维护系统时间，不执行菜单逻辑。

| 按键 | 引脚 | 停止状态下的作用 |
| --- | --- | --- |
| SW1 / 上 | P11.10 | 上移；编辑时增大 |
| SW2 / 中心 | P11.12 | 确认；运行时立即急停 |
| SW3 / 左 | P11.9 | 返回；编辑时取消 |
| SW4 / 下 | P11.11 | 下移；编辑时减小 |
| SW5 / 右 | P13.3 | 进入；确认并保存 |

AUX1（P11.2）和 AUX2（P11.3）不参与菜单。运行状态禁止调参；按中心键会停止电机并让舵机回到中心。上下键长按会按 100 ms 重复调节，短按、长按和取消逻辑沿用参考工程的轮询方式。

## 3. TFT 刷新策略

TFT 使用逐飞官方硬件 SPI 接口；`libraries/zf_device/zf_device_ips200.h` 已设置 `IPS200_USE_SOFT_SPI` 为 `0`，并固定调用：

```c
ips200_init(IPS200_TYPE_SPI);
```

刷新策略与 `Camera4-copy` 一致：

- 普通菜单只在按键、参数修改、运行状态或错误状态变化后刷新，空闲时不周期重绘。
- 原始图像和二值化图像页面最多每 50 ms 刷新一次，并且只在有新的完整结果时刷新。
- 遥测页面每 50 ms 刷新一次。
- LCD 测试进入页面时绘制一次，测试期间不重复刷新整页。

## 4. 双核职责与最新帧策略

CPU0 负责摄像头完成标志、帧复制、PIT 定时、电机/舵机控制、菜单、TFT、串口和运行状态。CPU1 只调用现有 `image_deal()` 做图像处理，不调用 PWM、电机、舵机、菜单或 TFT 函数。

帧通道有两个槽，状态严格按下面的方向转换：

```text
FREE -> READY -> READING -> DISPLAY_READY -> FREE
```

CPU0 只把完整原始帧写入 `FREE` 槽，写完像素和帧序号后才发布为 `READY`。没有空槽时立即丢弃摄像头帧并增加 `car_capture_drop_count`，不等待 CPU1。

CPU1 每次扫描所有 `READY` 槽，选择帧序号最大的最新帧；其他旧 `READY` 槽直接释放并增加 `car_processing_drop_count`。CPU1 完成处理后把原始帧、二值帧和结果关联到同一个槽。CPU0 收到结果后把该槽设为当前 `DISPLAY_READY` 槽；如果新结果替换了尚未显示的旧槽，增加 `car_display_drop_count`。

共享数组为：

```c
car_gray_frames[2][CAR_IMAGE_HEIGHT][CAR_IMAGE_WIDTH]
car_binary_frames[2][CAR_IMAGE_HEIGHT][CAR_IMAGE_WIDTH]
```

原始数组放在 CPU1 DSRAM，二值数组放在 CPU0 DSRAM，CPU1 通过共享总线写入二值槽。它们不是菜单缓存，也不复制到第三份图像缓冲。TFT 只读取状态为 `DISPLAY_READY` 的槽，因此不会显示 CPU1 正在写入的半帧。

遥测页显示以下统计：捕获丢帧、CPU1 处理丢帧、显示丢帧、结果丢弃和图像处理最大耗时。`car_result` 仍由 CPU1 发布，CPU0 收到后继续调用现有 `car_track_update()`。

## 5. 当前图像算法接口

CPU1 直接调用当前 `code/image.h` 中的 `image_deal()` 处理指定帧：

```c
void image_deal(uint8 start_y, uint8 end_y,
                const uint8 *gray_frame,
                uint8 *binary_frame,
                car_result_t *result);
```

输入为当前帧槽的原始灰度图，输出为同一槽的二值化图。内部继续使用当前 `image.c`、Otsu/固定阈值、最长白列、左右边界、十字补线和 `control_row_near` 误差链，不迁移 `Camera4-copy` 的完整车辆控制算法。控制行越界时会限制在图像高度范围内。

CPU1 发布的结果包含线有效标志、中心、误差、左右边界、线宽、丢线计数、使用阈值、帧序号和处理耗时；CPU0 只消费结果并执行控制。

## 6. 舵机参数

舵机 PWM 计数周期为 20 ms，`PWM_DUTY_MAX` 为 10000。当前车辆实测值固定采用：

| duty | 脉宽 |
| ---: | ---: |
| 600 | 1200 us |
| 700 | 1400 us |
| 800 | 1600 us |

菜单中的 `SERVO CENTER` 保存脉宽，默认 1400 us；`SERVO TRAVEL` 保存总的单侧行程，默认 200 us。程序按 `duty = pulse_us * 10000 / 20000` 换算，并把最终 duty 限制在 600～800，所以启动阶段和菜单应用不会再使用旧的 830/700/1000 范围。

## 7. 参考工程参数换算

当前只借用 `Camera4-copy` 的菜单交互、五键轮询、硬件 SPI、50 ms 页面刷新和视觉参数默认值，不迁移它的完整 `car_control`/`car_vision` 控制算法。

- 参考工程的电机命令按 `command * 10000 / 1000` 转成占空比，因此当前直接 duty 控制对应采用参考命令乘 10：280/270/200 分别保存为 2800/2700/2000。
- 电机左右指令也直接使用 PWM duty（正负号表示方向，绝对值范围为 0～`PWM_DUTY_MAX`），不再乘除旧的 `-500…500` 控制量比例。
- 参考工程普通 P 项是舵机脉宽单位，当前 `steering_kp` 按 0.5 duty/us 换算。
- 参考工程 D 项按 20 ms 帧周期换算到当前逐帧控制，当前默认值使用 `KD_reference * 0.02 * 0.5` 的单位关系。
- 阈值、自动阈值、曝光、增益和视觉行号等字段直接沿用参考默认值；未被当前最长白列算法使用的参考字段只保留在扁平 `car_params_t` 中，不自动显示在菜单。

## 8. 与 `Camera4-copy` 的区别

- 菜单是当前工程的扁平节点数组，参数直接指向 `car_params`；没有参考工程的菜单数据表、应用层和多层 UI 页面。
- CPU1 只跑当前最长白列、左右边界、十字补线和普通 PD 输入；不迁移参考工程完整的 `car_control`、`car_vision`、运行守护和曲线控制链。
- 当前双槽通道会主动选择最新帧并释放旧 `READY` 帧，分别统计捕获、处理、显示和结果丢弃；控制仍由 CPU0 执行。
- 舵机采用车辆实测 600/700/800 duty，而参考工程默认范围为 560/700/840；当前参数保存为 1200/1400/1600 us 的 600～800 duty 范围。
- 电机在当前 `car.c` 中直接使用 PWM duty；参考工程端口函数接收命令单位后再做比例换算。
- 普通菜单、图像页和遥测页仍沿用参考工程的交互节奏，但显示内容、参数数量和页面布局为当前黑底文字简菜单。

## 9. 引脚与构建说明

显示屏使用 `SPI_2`：SCLK 为 P15.3，MOSI 为 P15.5，CS 为 P15.2，DC 为 P15.0，RESET 为 P15.1，背光为 P20.14。摄像头、PWM 和按键引脚均继续由 `board_pins.h` 提供，本次没有改变低电平有效逻辑。

工程需要 ADS/Tasking TriCore 编译器完成最终硬件编译。当前代码级检查重点是：CPU1 没有执行器调用、旧单帧/旧图像接口不再被引用、双槽状态发布顺序正确，以及菜单数组可以独立增删节点。
