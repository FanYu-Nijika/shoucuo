# TC264 单摄四轮车初始项目

本目录可以直接复制为新项目。它已经包含 TC264 逐飞开源库、单路 MT9V03X 驱动、
摄像头资料、通用图像算法、赛道检测、PID、四电机控制和 VSCode 代码补全配置。

## 新项目使用方法

1. 复制整个目录，不要只复制 `user` 或 `code`。
2. 使用 VSCode 时双击 `Open_in_VSCode.code-workspace`，确保项目目录本身是工作区根目录；
   若直接打开 ADS 工作区的上一层目录，VSCode 不会读取项目内部的 `.vscode` 配置。
3. 如果需要更改 ADS 工程名称，先运行 `AURIX修改工程名称.bat`，再导入 AURIX
   Development Studio。若多个副本同时导入同一个 ADS 工作区，工程名必须不同。
4. 在 `user/cpu0_main.c` 编写应用逻辑；通用算法放在 `code`，板级驱动继续使用
   `libraries`。
5. 根据实车接线检查 `libraries/zf_device/zf_device_mt9v03x.h` 中的配置串口、PCLK、
   VSYNC、D0-D7 和 DMA 通道。模板中的数值只是逐飞默认接线，并非车辆最终接线。
6. VSCode 只负责代码补全、定义跳转和阅读；固件仍在 ADS 中编译和下载。
   补全配置使用本机 `D:/mingw64/bin/gcc.exe` 提供标准头文件（例如 `math.h`）；若换电脑，
   只需修改 `.vscode/c_cpp_properties.json` 中的 `compilerPath`。

## 默认摄像头

- 型号：单路 MT9V03X/MT9V034 总钻风灰度摄像头。
- 默认图像：188×120，50 FPS。
- 驱动：`libraries/zf_device/zf_device_mt9v03x.c/.h`。
- 资料：`docs/MT9V03X`。
- 图像算法入口：`code/camera_car.h`。

没有加入双摄开源库。三个 Altium 封装压缩包也不是固件依赖，仅在设计对应 PCB 时
单独使用，因此不放入本初始项目。
