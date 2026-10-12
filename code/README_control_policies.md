# 控制链路自动策略审计（2026-10-11）

本次已删除视觉失效／图像超过 120 ms 时把循迹速度目标设为 0 的逻辑。进入循迹阶段后，速度目标直接取菜单设定值，视觉有效性只决定转向是否更新。单帧丢线、持续图像超时、缺失时间戳和非有限视觉误差都不会改变速度目标。

本次也删除了 VOFA 串口命令拒绝负直立 Kp/Kd 的条件，与菜单及底层有限数校验保持一致。

## 当前仍生效的自动行为

| 行为 | 触发条件和结果 | 源码 |
| --- | --- | --- |
| 图像灰度触发完整 STOP | 循迹运行时，第 115 行横向 1/5、2/5、3/5、4/5 四点原始灰度都低于 100，一帧即提交 CENTER 停止请求；CPU0 执行完整 STOP，清 FOLLOW、速度积分及电机输出。 | `image.c:protect` → `cpu0_main.c` → `car_menu.c:car_menu_handle_center` |
| 视觉失效清转向 | 无效结果或超过 120 ms 时，差速请求归零，同时重置弯道模式、参数混合进度及 D 历史；恢复后的第一帧 D 为 0。速度目标保持菜单设定。 | `car.c:car_track_update`、`car_reset_turn` |
| 直道／弯道转向变参 | 曲率超过进入阈值切到弯道，低于退出阈值切回直道。切换时重置 D，每个新视觉结果改变 0.5 的混合权重，两帧完成 Kp/Kd 过渡。 | `car.c:car_track_update` |
| 转向微分的时间条件 | 使用误差差分乘以 `20 / 帧间隔ms`。首次结果、D 历史重置后、帧间隔为 0 或超过 100 ms 时 D 为 0。两次有效新帧之间保持上次差速请求。 | `car.c:car_track_update` |
| 动态前瞻 | 20 ms 编码器平均计数超过 250 后，每增加 20 计数把前瞻行减 1；进入弯道时冻结前瞻，退出后恢复按速度调整。改变的是视觉误差采样位置。 | `image.c:Update_Dynamic_Preview_Row` |
| FOLLOW 直立阶段也会转向 | START 先停机等 2 秒，实际 RUN 启动后直立保持 0.5 秒。速度 PI 此时关闭，但视觉转向计算和差速输出已经启用。 | `car.c:car_track_update` |
| FOLLOW 启动速度渐增 | 直立之后进入 SPEED_RAMP，开启速度 PI，目标在 2 秒内从 0 线性增加到菜单速度，再进入 ACTIVE。没有额外启动倾角，渐增结束保留积分；这是用户在 2026-10-12 指定的启动流程。 | `car.c:car_track_update` |
| 异常丢失 RUN 后自动重启 | FOLLOW 未退出且 `car_running` 意外变成 0、没有故障时进入 WAIT_RUN，再次请求 RUN，启动后重新直立等待。明确 STOP 会清 FOLLOW 状态，不走此路径。 | `car.c:car_track_update`、`balance_runtime.c:balance_runtime_update_2ms` |
| 故障退出 FOLLOW | FOLLOW 时检测到非零故障会取消循迹和速度 PI，并清积分；此调用本身保留直立运行，不等同于完整 STOP。 | `car.c:car_track_update`、`car_follow_stop` |
| IMU 读数失败保持旧 PWM | 坏样本时不执行新的直立输出。连续 8 个坏样本报 IMU 故障；若已运行，失败处理仍保留上次电机命令，不进入停机状态的 IMU 重试路径。 | `balance_runtime.c:balance_runtime_imu_failed` |
| 数值错误不写新 PWM | 运行中姿态或直立计算失败会取消 FOLLOW 指令，当前周期不更新电机；硬件可能保持此前 PWM。下一有效计算可能清故障。 | `balance_control.c:balance_update_2ms`、`balance_runtime.c:balance_runtime_update_2ms` |
| 执行时间检查 | 2 ms 控制 ISR 耗时达到 2000 µs，或连续 8 次间隔超过 2400 µs，标记 TIMING；FOLLOW 随后按故障路径退出。 | `user/cpu0_main.c:cc60_pit_ch0_isr` |
| 摄像头恢复 | 图像停帧会请求 DMA 恢复／重启摄像头。FOLLOW START 没有摄像头在线门槛；在当前速度独立策略下，摄像头持续无效也会保持设定速度、转向为 0。 | `user/cpu0_main.c`、`car.c:car_follow_start` |
| 直立优先分配 PWM | 转向只能使用直立共模输出后剩余的硬件 PWM 空间；直立饱和时转向被压到 0。 | `balance_control.c:balance_run_control` |
| 停机静止时学习陀螺零偏 | 已有有效 IMU 标定，停机静止 1 秒后以 0.00040012 系数缓慢修正 RAM 零偏；运行时关闭学习。 | `balance_runtime.c:balance_runtime_update_bias_trim` |

## 已核对的兼容字段与边界

- 速度 PI 没有 0.04° 输出步长或电机饱和时冻结积分的条件；FOLLOW 启动阶段有用户指定的 2 秒目标速度渐增。
- 测速每 2 ms 更新，滤波为当前 raw 占 90%、上次滤波值占 10%；速度 PI 每 20 ms 取完整周期内十笔滤波速度的平均值。
- 普通 RUN、FOLLOW 直立阶段、STOP、FOLLOW STOP 的速度环开关及积分清理遵循用户已指定的行为。
- 当前活动控制不使用旧 `base_speed`、`curve_slowdown`、`lost_search_steering`、`lost_stop_frames`、`stop_time` 实施开环速度、搜索、弯道降速或定时停车。
- `pwm_limit` 是旧 Flash 布局字段，已不限制直立输出；实际电机 PWM 范围为 ±10000。右轮补偿函数仍在，但当前板级补偿为 0。
- `BALANCE_FAULT_TILT` 有枚举和显示文字，当前源码没有设置该故障的分支。
- 当前增益允许正、零、负的有限浮点值；部分旧档迁移仍会把零值弯道 Kp/Kd 替换成默认值。

本次验证：24 组 Node 回归检查通过；两个实际 C 宿主测试以 `-Wall -Wextra -Werror` 编译、运行通过；新增覆盖视觉失效持续及恢复时速度目标不跳零、负串口增益及非法命令拒绝。未烧录或验证实车响应。
