# COM32 串口连续记录

`record_serial.ps1` 用 Windows 自带的 PowerShell 5.1 记录串口数据，不需要安装额外模块。默认端口为 COM32，115200 波特率、8N1。记录状态只显示在窗口里，不会混进串口文件。

## 首次使用和录日志

1. 将车连接到电脑，确认设备管理器里对应端口是 COM32。
2. 关闭 VOFA+、串口助手等正在使用 COM32 的程序。Windows 串口通常只能由一个程序独占；本工具不会自动抢占端口，也不会替你关闭其他串口软件。
3. 在本目录打开 PowerShell，执行：

   ```powershell
   powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\record_serial.ps1 -Port COM32 -BaudRate 115200
   ```

   也可以双击 `record_COM32.cmd`，它使用相同的默认端口和波特率。

4. 看到 `Connected: COM32` 后，再按车上的 `FOLLOW START`。让小车完成循迹，按 `STOP` 后继续留几秒，再回到记录窗口按 `Q`，或按 `Ctrl+C` 结束。结束时程序会关闭串口并保存文件；窗口会显示日志目录。

默认日志保存在脚本旁的 `logs\年月日_时分秒_毫秒_COM32\`。每次启动新建一个目录，内含：

- `serial.bin`：收到的原始字节，不做文本或换行转换。
- `serial.txt`：收到的 ASCII 文本，保留设备文本内容；不包含程序状态提示。
- `speed.csv`：按完整数据帧解析的速度诊断表。状态提示只在窗口显示，不写入这些文件。

窗口约每秒显示累计接收字节数、完整帧数、距最近完整帧的时间 `frame-age`、最近帧的 `pi_active`、`running`、`fault`，以及速度 PI 开启的累计帧数。日志中应能看到 `pi_active=1`、`running=1`、`fault=0` 的循迹帧；如果没有，先检查采集时机、固件输出和串口设置。

## CSV 通道

`speed.csv` 的列顺序为：

```text
pc_time_iso,frame_columns,target_mps,filtered_mps,speed_angle_deg,target_angle_deg,angle_deg,balance_pwm,device_time_ms,raw_mps,pi_feedback_mps,error_mps,p_angle_deg,i_angle_deg,gyro_deg_s,turn_request_pwm,turn_actual_pwm,left_count_2ms,right_count_2ms,left_pwm,right_pwm,pi_active,running,fault
```

`pc_time_iso` 是电脑解析该帧时的 UTC 时间，同一批收到的帧可能具有相近的电脑时间；采样间隔应看 `device_time_ms`。`frame_columns` 是设备帧列数。新版 22 列 FireWater 帧按上表填写。旧版 6 列帧仍可读取，缺失的后续通道留空。PWM 和编码器计数是整数，`pi_active`、`running` 为 0/1，`fault` 为故障枚举值。数值列以设备帧为准；主循环打印间隔可能变化，不要把日志采样当成严格 50 Hz。

## 端口断开和离线解析

串口断开时程序会继续运行，每 3 秒尝试重连；重新连接后继续记录。若端口被占用，先手动关闭 VOFA+ 或其他串口程序，再启动记录器。若目录有数据但 `speed.csv` 没有完整帧，检查设备是否正输出 FireWater 文本、波特率是否正确，以及 `serial.txt` 中实际收到的内容；原始字节仍保存在 `serial.bin`。

可以指定其他日志父目录：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\record_serial.ps1 -Port COM32 -BaudRate 115200 -OutputDir D:\robot_logs
```

也可以不打开串口，重放已有的二进制日志来重建文本和 CSV，便于检查解析结果：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\record_serial.ps1 -ReplayFile .\logs\20261011_120000_123_COM32\serial.bin
```
