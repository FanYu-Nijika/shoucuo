# 双轮平衡与摄像头循线

本说明适用于当前双轮版本。旧四轮说明中的舵机、基础速度 PWM 和定时停车不参与本版本控制。

## 周期与单位

- IMU 初始化后，按 5 ms 周期采样 200 次（1 秒），至少 190 个有效样本。校准期间电机关闭，失败重试；成功后保持 STOP，必须由菜单 RUN 启动零速平衡。
- Kalman、角度外环、俯仰角速度 P 内环和电机输出为 5 ms；编码器每 5 ms 读取，速度 PI 每 20 ms 更新。
- LEFT/RIGHT COUNT、左右滤波速度和 TARGET SPEED 均使用单轮 20 ms 计数。旧的 SPEED FILTERED 字段保留为左右滤波值的平均数。
- 保留旧速度 Kp/Ki 单位：20 ms 积分累加误差的两倍，滤波系数内部按 `1-(1-a)^2` 换算。
- 左右轮各自维护速度误差、积分和 SPEED ANGLE。前进编码器与前进 PWM 均为正，因此 PI 输出先乘速度角方向 `-1`，再加到机械中值；修正速度限制为 2 度/秒。
- 摄像头转向输出作为轮速差目标：正值提高左轮目标、降低右轮目标。TURN SPEED LIMIT 的单位为单轮 20 ms 编码器计数，不再直接叠加 PWM。
- 正常平衡输出按 LEFT/RIGHT MOTOR DEADZONE 对齐两轮起转阈值；默认左 1100、右 600。MOTORS 测试仍输出原始 PWM，不经过补偿。

## 菜单使用

1. 在 RUN CONTROL 执行 STOP，设置 BALANCE 中 SPEED ENABLE、TARGET SPEED 20MS，以及 STEERING 的 PD、TURN SPEED LIMIT、TURN DIRECTION。默认目标速度为 0，轮速差限幅为 100。
2. 速度环默认 `SPEED KP=0.004`、`SPEED KI=0.000020`，对应参考工程约 4 PWM/count 和 0.02 PWM/(count-sample)。首次架空验证先将 KI 设为 0，确认车轮越快时对应 PWM 会减小，再逐步增加 KI。
3. 按需 SAVE FLASH。RUN 恢复零速平衡，不自动前进。
4. 摄像头连续三帧识别有效赛道且图像年龄不超过 100 ms 后，执行 FOLLOW START。速度环必须开启、目标速度必须大于零。
5. FOLLOW STOP 取消转向，速度目标按每秒 100 个计数率单位降至零，继续平衡。STOP 关闭电机。

丢线、异常视觉数值或图像年龄超过 100 ms 自动退出循线；图像恢复后仍需 FOLLOW START。每个新结果只更新一次视觉 PD，首帧 D 项为零，后续 D 项归一化到 20 ms。

FOLLOW STATUS：0 待命、1 循线、2 未平衡、3 速度环关闭、4 目标速度为零、5 无新鲜赛道、6 丢线退出。

原有 MOTORS 测试模块保留。TEST START 会屏蔽平衡控制；退出测试后再执行 RUN。运行中不能编辑控制参数、切档或写 Flash。

RUN 请求由 5 ms 控制中断在下一次有效 IMU 采样时执行，并用当时的加速度角重新初始化 Kalman。倾倒仍锁停且不会自动恢复；人工扶正后重新按 RUN。菜单顶部直接显示 `CAL/ARMING/RUN/TILT/IMUERR/NUMERR/TIMEERR/STOP`，不会再把平衡故障误报为 `LINE LOST`。

## 存档与验证

平衡存档为 V7。`ANGLE KP` 保留旧 `BALANCE KP` 的 PWM/degree 数值，`RATE KP` 保留旧 `BALANCE KD`；内部角度外环增益为 `ANGLE KP / RATE KP`，因此默认 1000/25 与旧 PD 在未限幅区等效。旧 V6 的角度外环参数会乘回 `RATE KP`，恢复为相同的 PWM/degree 数值。V1-V6 若仍使用旧占位速度参数 0.00025/0.000005，则在内存中迁移为 0.004/0.000020，自定义速度参数保持不变。目标角速度限幅使用 200 deg/s。只在 SAVE FLASH 时写入，原四轮存档布局和舵机占位字段保留。

`node tests/control_numeric.cjs` 执行从 C 函数提取的数值逻辑及存档布局测试，不调用 C 编译器。它不能验证 TC264 的实际执行耗时、浮点细节、电机方向和实车稳定性。固件未编译，实际耗时查看 BALANCE MAX US/PERIOD/OVERRUN 遥测。

实车验收顺序：确认电机与编码器方向，再验证零速平衡、小目标速度前进、视觉差速方向，最后验证丢线回零速。此前反馈的大电流及重启原因尚未查明，本次功能接入不代表该问题已修复。
