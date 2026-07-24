# 开源算法检索与移植说明

本工程的模糊 PID、补线和赛道状态识别模块针对 TC264 重新设计，接口与代码均未直接复制无许可证仓库。

参考来源：

- [weety/gorilla fuzzy_pid.c](https://github.com/weety/gorilla/blob/master/gorilla/control/fuzzy_pid.c)：Apache-2.0。参考其“误差与误差变化率共同调度 PID 增益”的思路；本工程改为更轻量的三档隶属度、3×3 规则和加权平均解模糊。
- [HUANsic/CM2023 huansic_fuzzy_pid.c](https://github.com/HUANsic/CM2023/blob/main/SmartCar_CM_LVGL_Demo/SmartCar_CM_LVGL_Demo/User/huansic_fuzzy_pid.c)：用于对照智能车误差隶属度设计。仓库根目录未找到许可证，因此没有复制其代码或参数。
- [Chenchaol16/20-smartcar-longxin image.h](https://github.com/Chenchaol16/20-smartcar-longxin/blob/main/project/code/image.h)：用于核对常见的普通赛道、十字、斑马线、圆环状态划分。仓库根目录未找到许可证，因此状态机由本工程独立实现。
- [Bigrreedd/TDPS_MiniCar_Control Path.c](https://github.com/Bigrreedd/TDPS_MiniCar_Control/blob/LHX/competition/User/Path.c)：用于对照连续帧计数和状态转移保护。仓库根目录未找到许可证，因此没有复制其实现。

本次新增文件：

- `code/fuzzy_pid.c/.h`
- `code/track_repair.c/.h`
- `code/track_state.c/.h`

其中 `track_repair` 和 `track_state` 不固定图像宽高，可用于 MT9V03X 的任意裁剪分辨率。
