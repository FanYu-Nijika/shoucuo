/*
 * qd_device_icm42688.h
 *
 *  Created on: 2024年8月3日
 *      Author: 17263
 */

#ifndef QD_DEVICE_ICM42688_H_
#define QD_DEVICE_ICM42688_H_


#include "zf_common_typedef.h"
#include "zf_driver_delay.h"



#define ICM42688_USE_SOFT_IIC       (0)                                         // 默认使用硬件 SPI 方式驱动
#if ICM42688_USE_SOFT_IIC                                                       // 这两段 颜色正常的才是正确的 颜色灰的就是没有用的
//====================================================软件 IIC 驱动====================================================
#define ICM42688_SOFT_IIC_DELAY     (10 )                                       // 软件 IIC 的时钟延时周期 数值越小 IIC 通信速率越快
#define ICM42688_SCL_PIN            (B13)                                       // 软件 IIC SCL 引脚 连接 ICM42688 的 SCL 引脚
#define ICM42688_SDA_PIN            (B15)                                       // 软件 IIC SDA 引脚 连接 ICM42688 的 SDA 引脚
//====================================================软件 IIC 驱动====================================================
#else

//====================================================硬件 SPI 驱动====================================================
#define ICM42688_HARD_SPI
#define ICM42688_SPI_SPEED          (24*1000*1000)                              // 硬件 SPI 速率
#define ICM42688_SPI                SPI_0                                       // 硬件 SPI 号
#define ICM42688_SPC_PIN            SPI0_SCLK_P20_11                           // 硬件 SPI SCK 引脚
#define ICM42688_SDI_PIN            SPI0_MOSI_P20_14                           // 硬件 SPI MOSI 引脚
#define ICM42688_SDO_PIN            SPI0_MISO_P20_12                           // 硬件 SPI MISO 引脚

//====================================================硬件 SPI 驱动====================================================
#endif

#define ICM42688_DELAY_MS(x)    (system_delay_ms(x))
#define ICM42688_CS_PIN             (P20_13)                                       // CS 片选引脚
#define ICM42688_CS_LEVEL(x)              ((x) ? (gpio_high(ICM42688_CS_PIN)) : (gpio_low(ICM42688_CS_PIN)))

// 声明ICM42688加速度计数据
extern float icm42688_acc_x  , icm42688_acc_y  , icm42688_acc_z  ;
// 声明ICM42688角加速度数据
extern float icm42688_gyro_x , icm42688_gyro_y , icm42688_gyro_z ;

enum icm42688_afs
{
    ICM42688_AFS_16G,   // default
    ICM42688_AFS_8G,
    ICM42688_AFS_4G,
    ICM42688_AFS_2G,
    NUM_ICM42688__AFS
};
enum icm42688_aodr
{
    ICM42688_AODR_32000HZ,
    ICM42688_AODR_16000HZ,
    ICM42688_AODR_8000HZ,
    ICM42688_AODR_4000HZ,
    ICM42688_AODR_2000HZ,
    ICM42688_AODR_1000HZ,// default
    ICM42688_AODR_200HZ,
    ICM42688_AODR_100HZ,
    ICM42688_AODR_50HZ,
    ICM42688_AODR_25HZ,
    ICM42688_AODR_12_5HZ,
    ICM42688_AODR_6_25HZ,
    ICM42688_AODR_3_125HZ,
    ICM42688_AODR_1_5625HZ,
    ICM42688_AODR_500HZ,
    NUM_ICM42688_AODR
};

enum icm42688_gfs
{
    ICM42688_GFS_2000DPS,// default
    ICM42688_GFS_1000DPS,
    ICM42688_GFS_500DPS,
    ICM42688_GFS_250DPS,
    ICM42688_GFS_125DPS,
    ICM42688_GFS_62_5DPS,
    ICM42688_GFS_31_25DPS,
    ICM42688_GFS_15_625DPS,
    NUM_ICM42688_GFS
};
enum icm42688_godr
{
    ICM42688_GODR_32000HZ,
    ICM42688_GODR_16000HZ,
    ICM42688_GODR_8000HZ,
    ICM42688_GODR_4000HZ,
    ICM42688_GODR_2000HZ,
    ICM42688_GODR_1000HZ,// default
    ICM42688_GODR_200HZ,
    ICM42688_GODR_100HZ,
    ICM42688_GODR_50HZ,
    ICM42688_GODR_25HZ,
    ICM42688_GODR_12_5HZ,
    ICM42688_GODR_X0HZ,
    ICM42688_GODR_X1HZ,
    ICM42688_GODR_X2HZ,
    ICM42688_GODR_500HZ,
    NUM_ICM42688_GODR
};


/**
*
* @brief    ICM42688陀螺仪初始化
* @param
* @return   void
* @notes    用户调用
* Example:  Init_ICM42688();
*
**/
void Init_ICM42688(void);

/**
*
* @brief    获得ICM42688陀螺仪加速度
* @param
* @return   void
* @notes    单位:g(mm/s^2),用户调用
* Example:  Get_Acc_ICM42688();
*
**/
void Get_Acc_ICM42688(void);

/**
*
* @brief    获得ICM42688陀螺仪角加速度
* @param
* @return   void
* @notes    单位为:°/s,用户调用
* Example:  Get_Gyro_ICM42688();
*
**/
void Get_Gyro_ICM42688(void);

/**
*
* @brief    设置ICM42688陀螺仪低通滤波器带宽和量程
* @param    afs                 // 加速度计量程,可在dmx_icm42688.h文件里枚举定义中查看
* @param    aodr                // 加速度计输出速率,可在dmx_icm42688.h文件里枚举定义中查看
* @param    gfs                 // 陀螺仪量程,可在dmx_icm42688.h文件里枚举定义中查看
* @param    godr                // 陀螺仪输出速率,可在dmx_icm42688.h文件里枚举定义中查看
* @return   void
* @notes    ICM42688.c文件内部调用,用户无需调用尝试
* Example:  Set_LowpassFilter_Range_ICM42688(ICM42688_AFS_16G,ICM42688_AODR_32000HZ,ICM42688_GFS_2000DPS,ICM42688_GODR_32000HZ);
*
**/
void Set_LowpassFilter_Range_ICM42688(enum icm42688_afs afs, enum icm42688_aodr aodr, enum icm42688_gfs gfs, enum icm42688_godr godr);

// ICM42688Bank0内部地址
#define ICM42688_DEVICE_CONFIG             0x11
#define ICM42688_DRIVE_CONFIG              0x13
#define ICM42688_INT_CONFIG                0x14
#define ICM42688_FIFO_CONFIG               0x16
#define ICM42688_TEMP_DATA1                0x1D
#define ICM42688_TEMP_DATA0                0x1E
#define ICM42688_ACCEL_DATA_X1             0x1F
#define ICM42688_ACCEL_DATA_X0             0x20
#define ICM42688_ACCEL_DATA_Y1             0x21
#define ICM42688_ACCEL_DATA_Y0             0x22
#define ICM42688_ACCEL_DATA_Z1             0x23
#define ICM42688_ACCEL_DATA_Z0             0x24
#define ICM42688_GYRO_DATA_X1              0x25
#define ICM42688_GYRO_DATA_X0              0x26
#define ICM42688_GYRO_DATA_Y1              0x27
#define ICM42688_GYRO_DATA_Y0              0x28
#define ICM42688_GYRO_DATA_Z1              0x29
#define ICM42688_GYRO_DATA_Z0              0x2A
#define ICM42688_TMST_FSYNCH               0x2B
#define ICM42688_TMST_FSYNCL               0x2C
#define ICM42688_INT_STATUS                0x2D
#define ICM42688_FIFO_COUNTH               0x2E
#define ICM42688_FIFO_COUNTL               0x2F
#define ICM42688_FIFO_DATA                 0x30
#define ICM42688_APEX_DATA0                0x31
#define ICM42688_APEX_DATA1                0x32
#define ICM42688_APEX_DATA2                0x33
#define ICM42688_APEX_DATA3                0x34
#define ICM42688_APEX_DATA4                0x35
#define ICM42688_APEX_DATA5                0x36
#define ICM42688_INT_STATUS2               0x37
#define ICM42688_INT_STATUS3               0x38
#define ICM42688_SIGNAL_PATH_RESET         0x4B
#define ICM42688_INTF_CONFIG0              0x4C
#define ICM42688_INTF_CONFIG1              0x4D
#define ICM42688_PWR_MGMT0                 0x4E
#define ICM42688_GYRO_CONFIG0              0x4F
#define ICM42688_ACCEL_CONFIG0             0x50
#define ICM42688_GYRO_CONFIG1              0x51
#define ICM42688_GYRO_ACCEL_CONFIG0        0x52
#define ICM42688_ACCEL_CONFIG1             0x53
#define ICM42688_TMST_CONFIG               0x54
#define ICM42688_APEX_CONFIG0              0x56
#define ICM42688_SMD_CONFIG                0x57
#define ICM42688_FIFO_CONFIG1              0x5F
#define ICM42688_FIFO_CONFIG2              0x60
#define ICM42688_FIFO_CONFIG3              0x61
#define ICM42688_FSYNC_CONFIG              0x62
#define ICM42688_INT_CONFIG0               0x63
#define ICM42688_INT_CONFIG1               0x64
#define ICM42688_INT_SOURCE0               0x65
#define ICM42688_INT_SOURCE1               0x66
#define ICM42688_INT_SOURCE3               0x68
#define ICM42688_INT_SOURCE4               0x69
#define ICM42688_FIFO_LOST_PKT0            0x6C
#define ICM42688_FIFO_LOST_PKT1            0x6D
#define ICM42688_SELF_TEST_CONFIG          0x70
#define ICM42688_WHO_AM_I                  0x75
#define ICM42688_REG_BANK_SEL              0x76// Banks
#define ICM42688_SENSOR_CONFIG0            0x03
#define ICM42688_GYRO_CONFIG_STATIC2       0x0B
#define ICM42688_GYRO_CONFIG_STATIC3       0x0C
#define ICM42688_GYRO_CONFIG_STATIC4       0x0D
#define ICM42688_GYRO_CONFIG_STATIC5       0x0E
#define ICM42688_GYRO_CONFIG_STATIC6       0x0F
#define ICM42688_GYRO_CONFIG_STATIC7       0x10
#define ICM42688_GYRO_CONFIG_STATIC8       0x11
#define ICM42688_GYRO_CONFIG_STATIC9       0x12
#define ICM42688_GYRO_CONFIG_STATIC10      0x13
#define ICM42688_XG_ST_DATA                0x5F
#define ICM42688_YG_ST_DATA                0x60
#define ICM42688_ZG_ST_DATA                0x61
#define ICM42688_TMSTVAL0                  0x62
#define ICM42688_TMSTVAL1                  0x63
#define ICM42688_TMSTVAL2                  0x64
#define ICM42688_INTF_CONFIG4              0x7A
#define ICM42688_INTF_CONFIG5              0x7B
#define ICM42688_INTF_CONFIG6              0x7C

void ICM42688_filterinng(void);

#endif /* QD_DEVICE_ICM42688_H_ */
