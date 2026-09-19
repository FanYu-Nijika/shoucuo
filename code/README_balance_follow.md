# 双轮直立与开环摄像头循迹

## 控制链路

- ICM42688 每 5 ms 更新一次。加速度角由当前 X/Z 轴配置计算，俯仰角速度使用扣除标定零偏后的 Y 轴。
- 姿态采用互补滤波：`angle = Ka * accel_angle + (1 - Ka) * (last_angle + gyro_rate * 0.005)`。`ACC WEIGHT KA` 默认 0.01。
- 直立控制恢复为直接 PD：`PWM = BALANCE KP * (COMP ANGLE - TARGET ANGLE) + BALANCE KD * CONTROL RATE`。
- 编码器不参与控制，仅把四个 5 ms 增量累计成左右 20 ms 原始计数，供菜单检查方向和机械差异。
- RUN 的目标角等于机械中值。FOLLOW 将目标角平滑改为 `机械中值 - FOLLOW ANGLE`；因此正的 FOLLOW ANGLE 表示向前倾并产生前进驱动力，但不代表确定车速。
- 摄像头直弯 PD 直接产生左右 PWM 差速。控制器先保留共同直立输出，再用剩余 PWM 空间分配差速，最后沿用左右电机死区对齐。

速度闭环、Kalman、目标角速度及角速度串级均已移除。没有速度反馈后，车辆速度和停车位置会随电池、地面、载荷及机械中值变化。

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
- RUN CONTROL：`FOLLOW ANGLE` 和 `FOLLOW ANGLE APPLIED`。
- STEERING：`TURN PWM LIMIT`、`TURN DIRECTION`、`TURN PWM CMD`。
- IMU：手动标定、Flash 有效状态、标定来源、静止/微调状态、原始三轴数据和零偏。

四档直立参数存档版本为 V8。V1～V7 只迁移直立 Kp/Kd、机械中值、最终 PWM 限幅、方向和左右死区；旧速度、Kalman 和角速度串级参数被丢弃。迁移只在 RAM 中完成，用户执行 `SAVE FLASH` 后才写 V8。

`BALANCE KP/KD` 不设置人为调参上限，但仍要求有限且非负。最终 `PWM LIMIT` 始终限制为 1～10000，40° 倾倒保护、IMU 失效和数值异常保护继续有效。

## 静态验证

`node tests/control_numeric.cjs` 只执行公式、状态约束、迁移字段和源码一致性检查，不调用 C 编译器。实车应依次验证：IMU 标定及重启加载、RUN 纯直立、FOLLOW 前倾方向、摄像头差速方向、丢线退回纯直立。
