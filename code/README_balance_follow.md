# 双轮直立与开环摄像头循迹

## 控制链路

- ICM42688 每 5 ms 更新一次。加速度角由当前 X/Z 轴配置计算，俯仰角速度使用扣除标定零偏后的 Y 轴。
- 姿态采用互补滤波：`angle = Ka * accel_angle + (1 - Ka) * (last_angle + gyro_rate * 0.005)`。`ACC WEIGHT KA` 默认 0.01。
- 直立控制恢复为直接 PD：`PWM = BALANCE KP * (COMP ANGLE - TARGET ANGLE) + BALANCE KD * CONTROL RATE`。
- 编码器不参与控制，仅把四个 5 ms 增量累计成左右 20 ms 原始计数，供菜单检查方向和机械差异。
- RUN 的目标角始终等于机械中值，只启动直立。`FOLLOW START` 立即锁存循迹请求并请求直立；直立一旦运行，目标角固定偏移 `FOLLOW ANGLE` 以产生前进趋势。
- 摄像头直弯 PD 直接产生左右 PWM 差速。控制器先限制共同直立输出，再用剩余 PWM 空间分配差速。
- `LINE VALID` 只用于遥测，不参与 FOLLOW 启停或前倾角。新图像会更新差速；无新图像时保持最近差速，上电后尚无图像时差速为 0、固定前倾角仍持续生效。
- 新右轮固定比新左轮少 200 PWM，以补偿实测 `左600/右800` 的等速差异；电机测试保持原始 PWM。


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
- RUN CONTROL：`FOLLOW ANGLE`、`FOLLOW ANGLE APPLIED` 和 `FOLLOW ACTIVE`。`FOLLOW ANGLE` 不设人为上下限，正值沿当前前进方向，负值沿相反方向；最终电机输出仍受 `PWM LIMIT` 约束。
- STEERING：直线/弯道 `KP/KD`、方差切换阈值、`TURN PWM LIMIT`、`TURN DIRECTION` 和实际差速。`TURN PWM LIMIT` 默认值为 `1400 PWM`。
- IMU：手动标定、Flash 有效状态、标定来源、静止/微调状态、原始三轴数据和零偏。

四档直立参数存档版本为 V8，`FOLLOW ANGLE` 已包含在现有布局中。旧左右死区以及循迹参数中的 `BASE SPEED/CURVE SLOW` 仅保留 Flash 布局兼容，不参与活动控制。

`BALANCE KP/KD` 不设置人为调参上限，但仍要求有限且非负。最终 `PWM LIMIT` 始终限制为 1～10000，有限数和硬件占空比钳位继续有效。

## 静态验证

`node tests/control_numeric.cjs` 只执行公式、状态约束、迁移字段和源码一致性检查，不调用 C 编译器。实车应依次验证：左右轮映射、编码器方向、固定 200 PWM 补偿、RUN 只直立、FOLLOW 目标角方向，以及摄像头有帧时更新差速、无帧时仍保持固定前倾。
