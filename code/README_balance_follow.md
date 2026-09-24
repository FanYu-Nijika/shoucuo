# 双轮直立与开环摄像头循迹

## 控制链路

- ICM42688 每 5 ms 更新一次。加速度角由当前 X/Z 轴配置计算，俯仰角速度使用扣除标定零偏后的 Y 轴。
- 姿态采用互补滤波：`angle = Ka * accel_angle + (1 - Ka) * (last_angle + gyro_rate * 0.005)`。`ACC WEIGHT KA` 默认 0.01。
- 直立控制使用直接 PD：`PWM = BALANCE KP * (COMP ANGLE - TARGET ANGLE) + BALANCE KD * CONTROL RATE`。RUN 时保持这条原始公式；FOLLOW 且实际前倾角非零时，先把姿态 P 项限制到 `PWM LIMIT * 0.6`，再加入角速度 D 项。
- 编码器不参与控制，仅把四个 5 ms 增量累计成左右 20 ms 原始计数，供菜单检查方向和机械差异。
- RUN 的目标角始终等于机械中值，只启动直立。`FOLLOW START` 先请求 RUN；直立控制真正启动后保持零前倾、零转向 2 秒，再进入 FOLLOW。有效直道使用完整的 `FOLLOW SPEED`，弯道保留方向并将其幅度降为 20%，因此无编码器时它表示开环驱动力度而不是恒定实测车速。
- 摄像头直弯 PD 直接产生左右 PWM 差速。控制器先限制共同直立输出，再用剩余 PWM 空间分配差速。
- `FOLLOW SPEED` 在 5 ms 控制周期内换算为前倾角：`speed * 0.05°`，范围 −100～100 对应 −5～5°。应用角度增大时每周期最多 0.01°（2°/s），减小时每周期最多 0.02°（4°/s）；反向命令先回到 0°，再建立相反方向的前倾。
- 新的有效图像更新差速；丢线或超过 120 ms 没有新图像时立即清零差速并请求速度回零，已应用前倾按 4°/s 回到 0°。`FOLLOW STOP`、停车或平衡故障会立即清零差速和已应用前倾。等待 2 秒期间图像继续处理，但不输出循迹前倾或差速。
- 新右轮按 `BOARD_BALANCE_RIGHT_PWM_TRIM` 减少 PWM，补偿机械差异；电机测试保持原始 PWM。


## IMU 标定

IMU 零偏记录独立保存在 DFlash 第 10 页，不随四档直立参数切换。记录包含三轴陀螺零偏、重力轴、重力方向、魔数、版本及校验值。

首次使用或记录校验失败时，菜单显示 `CALREQ`，RUN 被禁止：

1. 将车保持预期直立姿态，在 IMU 页面执行 `IMU CALIBRATE`。
2. 先连续静止 0.5 秒，再连续收集 1000 个 5 ms 样本，共 5 秒。
3. 中途晃动会丢弃当前窗口并重新等待静止。
4. 重力轴、方向和零偏检查通过后，在主循环自动写入第 10 页；写入期间电机保持关闭。

后续上电从 Flash 加载标定，只预热 40 帧（约 200 ms）并用平均加速度角初始化互补滤波。停机静止 1 秒后允许以 0.001 的系数在 RAM 中慢速修正温漂；RUN、FOLLOW 和电机测试期间不会学习，也不会自动反复写 Flash。

## 菜单与存档

- BALANCE：`BALANCE KP/KD`、`MIDDLE ANGLE`、`ACC WEIGHT KA`、`PWM LIMIT` 以及互补角度、目标角、角速度和实际 PWM。
- RUN CONTROL：`FOLLOW SPEED`、`FOLLOW ANGLE APPLIED` 和 `FOLLOW ACTIVE`。`FOLLOW SPEED` 为可编辑的 −100～100 开环档位，默认 0，步长 1；`FOLLOW ANGLE APPLIED` 是只读的实际应用前倾角，显示范围 −5～5°。正值沿当前前进方向，负值沿相反方向；最终电机输出仍受 `PWM LIMIT` 约束。
- STEERING：直线/弯道 `KP/KD`、方差切换阈值、`TURN PWM LIMIT`、`TURN DIRECTION` 和实际差速。`TURN PWM LIMIT` 默认值为 `1400 PWM`。
- IMU：手动标定、Flash 有效状态、标定来源、静止/微调状态、原始三轴数据和零偏。

四档直立参数存档版本为 V9。V8 中保存的 `FOLLOW ANGLE` 是角度字段，不能直接当作新的速度档位；加载 V8 时保留直立、机械、输出和方向参数，并把每个档位的 `follow_speed` 置为 0，避免升级后上电立即产生大前倾。旧左右死区以及循迹参数中的 `BASE SPEED/CURVE SLOW` 仅保留 Flash 布局兼容，不参与活动控制。

`BALANCE KP/KD` 不设置人为调参上限，但仍要求有限且非负。最终 `PWM LIMIT` 始终限制为 1～10000，有限数和硬件占空比钳位继续有效。

## 静态验证

`node tests/control_numeric.cjs` 执行映射、斜率、弯道 20%、P 项限幅、故障清除、V8→V9 迁移和源码一致性检查，不调用 C 编译器。`tests/balance_control_host.c` 可用 GCC 与 `balance_control.c` 一起编译，检查 −100/0/100 档、反向过零、5 ms 应用角度斜率、FOLLOW P 项限幅、RUN 完整 PD、差速余量、右轮补偿和无效 IMU 样本时的输出；硬件依赖由 `tests/balance_host_stubs.h` 替代。实车应从 FOLLOW SPEED 低档开始，依次验证左右轮映射、RUN 只直立、直道与弯道速度幅度、目标角方向、停帧/故障/STOP 后前倾和差速归零，并观察 ±5° 请求范围内是否仍有持续 PWM 饱和或磕头。
