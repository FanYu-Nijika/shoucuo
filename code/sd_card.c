/*
 * sd_card.c
 *
 *  Created on: 2021年2月20日
 *      Author: 戴尔
 */
#include "sd_card.h"
#include "zf_common_headfile.h"

uint8 SD_Type = 0;

//等待卡准备好
//返回值:0,准备好了;其他,错误代码
uint8 sd_spiReadWrite(uint8 data)
{
    uint8 res = 0;
    spi_transfer_8bit (SD_SPI_NUM,(uint8*)&data, &res,1);
//    spi_mosi(SD_SPI_NUM, SD_CS, (uint8*)&data, &res, 0, 1);
    return res;
}


static uint8 SdWaitReady(void)
{
    uint32 start = system_getval_ms();
    do {
        if (sd_spiReadWrite(0xFF) == 0xFF) return 0;
    } while (system_getval_ms() - start < 500);
    return 1;
}

//设置到低速模式  用于初始化，最高spi速度为400k
void SdSpiSpeedLow(void)
{
    spi_init(SD_SPI_NUM, 0, SPI_LOW_SPEED, SD_SCLK, SD_MOSI, SD_MISO, SD_CS);
}

//SD卡正常工作的时候,可以高速了
static void SdSpiSpeedHigh(void)
{
    spi_init(SD_SPI_NUM, 0, SPI_HIGH_SPEED, SD_SCLK, SD_MOSI, SD_MISO, SD_CS);
}


//取消选择,释放SPI总线  内部调用
void SD_DisSelect(void)
{
    gpio_set_level(CS_PIN, 1);
    sd_spiReadWrite(0xff);//提供额外的8个时钟
}

//选择sd卡,并且等待卡准备OK   返回值:0,成功;1,失败       内部使用
uint8 SdSelect(void)
{
    gpio_set_level(CS_PIN, 0);
    if(SdWaitReady()==0)return 0;//等待成功
    SD_DisSelect();
    return 1;//等待失败
}

//等待SD卡回应 Response:要得到的回应值
//返回值:0,成功得到了该回应值
//其他,得到回应值失败    期待得到的回应值
SDHCRES SdGetResponse(uint8 Response)
{
    uint16 Count=0xFFF;//等待次数
    while ((sd_spiReadWrite(0XFF)!=Response)&&Count)Count--;//等待得到准确的回应
    if (Count==0)return MSD_RESPONSE_FAILURE;//得到回应失败
    else return MSD_RESPONSE_NO_ERROR;//正确回应
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      发送命令
//  param       cmd 命令
//  param       arg 命令参数
//  param       crc 校验值
//  @return     状态
//  @since      v1.0
//  Sample usage:       内部调用
//-------------------------------------------------------------------------------------------------------------------
SDHCRES sd_sendcmd(uint8 cmd, uint32 arg, uint8 crc)
{
    uint8 r1;
    uint8 Retry=0;
    SD_DisSelect();//取消上次片选
    if(SdSelect())return 0XFF;//片选失效
    //发送
    sd_spiReadWrite(cmd | 0x40);//分别写入命令
    sd_spiReadWrite((arg >> 24) & 0xFF);
    sd_spiReadWrite((uint8)(arg >> 16) & 0xFF);
    sd_spiReadWrite((uint8)(arg >> 8) & 0xFF);
    sd_spiReadWrite((uint8)(arg) & 0xFF);
    sd_spiReadWrite(crc);
    if(cmd==CMD12)sd_spiReadWrite(0xff);//Skip a stuff byte when stop reading
    //等待响应，或超时退出
    Retry=0X1F;
    do
    {
        r1=sd_spiReadWrite(0xFF);//读取返回信息
    }while((r1&0X80) && Retry--);//超时或者其他错误 正确返回错误信息    首位总是0
    //返回状态值
    return r1;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      初始化
//  @return     状态
//  @since      v1.0
//  Sample usage:       内部调用
//-------------------------------------------------------------------------------------------------------------------
SDHCRES sd_init(void)
{
    uint8 r1 = 0;
    uint8 buf[4];
    uint16 i=0;
    gpio_init(CS_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    spi_init(SD_SPI_NUM, 0, SPI_LOW_SPEED, SD_SCLK, SD_MOSI, SD_MISO, SD_CS);//先设置低速
    for(i=0;i<10;i++)   sd_spiReadWrite(0xff);

    uint16 retry=20;
    do
    {
        r1=sd_sendcmd(CMD0,0,0x95);//进入IDLE状态
    }while((r1!=0X01) && retry--);
    SD_Type=0;//默认无卡
    if(r1==0X01)
        {
            if(sd_sendcmd(CMD8,0x1AA,0x87)==1)//SD V2.0
            {
                for(i=0;i<4;i++)    buf[i]=sd_spiReadWrite(0XFF);   //Get trailing return value of R7 resp
                if(buf[2]==0X01&&buf[3]==0XAA)//卡是否支持2.7~3.6V
                {
                    retry=0XFFFE;
                    do
                    {
                        sd_sendcmd(CMD55,0,0X01);   //发送CMD55
                        r1=sd_sendcmd(CMD41,0x40000000,0X01);//发送CMD41
                    }while(r1&&retry--);
                    if(retry&&sd_sendcmd(CMD58,0,0X01)==0)//鉴别SD2.0卡版本开始
                    {
                        for(i=0;i<4;i++)buf[i]=sd_spiReadWrite(0XFF);//得到OCR值
                        if(buf[0]&0x40)SD_Type=SD_TYPE_V2HC;    //检查CCS
                        else SD_Type=SD_TYPE_V2;
                    }
                }
            }else//SD V1.x/ MMC V3
            {
                sd_sendcmd(CMD55,0,0X01);       //发送CMD55
                r1=sd_sendcmd(CMD41,0,0X01);    //发送CMD41
                if(r1<=1)
                {
                    SD_Type=SD_TYPE_V1;
                    retry=0XFFFE;
                    do //等待退出IDLE模式
                    {
                        sd_sendcmd(CMD55,0,0X01);   //发送CMD55
                        r1=sd_sendcmd(CMD41,0,0X01);//发送CMD41
                    }while(r1&&retry--);
                }else
                {
                    SD_Type=SD_TYPE_MMC;//MMC V3
                    retry=0XFFFE;
                    do //等待退出IDLE模式
                    {
                        r1=sd_sendcmd(CMD1,0,0X01);//发送CMD1
                    }while(r1&&retry--);
                }
                if(retry==0||sd_sendcmd(CMD16,512,0X01)!=0)SD_Type=SD_TYPE_ERR;//错误的卡
            }
        }
        SD_DisSelect();//取消片选
        SdSpiSpeedHigh();//高速
        //spi_init(SD_SPI_NUM, 0, 20*100*100, SD_SCLK, SD_MOSI, SD_MISO, SD_CS);
        if(SD_Type)return MSD_RESPONSE_NO_ERROR;
        else if(r1)return r1;
        return MSD_RESPONSE_FAILURE;//其他错误
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      写入512个字节
//  @param      缓存区
//  @param      指令
//  @return     0 成功        else 失败
//  @since      v1.0
//  Sample usage:       内部调用
//-------------------------------------------------------------------------------------------------------------------
uint8 sd_SendBlockData(uint8 *buf, uint8 cmd)
{
    uint16 t;
    if(SdWaitReady())return 1;//等待准备失效
    sd_spiReadWrite(cmd);
    if(cmd!=0XFD)//不是结束指令
    {
        for(t=0;t<512;t++, buf++)
        {
            sd_spiReadWrite(*buf);
//          if (!buf)
//          {
//              sd_spiReadWrite(*buf);//提高速度,减少函数传参时间
//              buf++;
//          }
//          else    sd_spiReadWrite(0xFF);
        }
        sd_spiReadWrite(0xFF);//忽略crc
        sd_spiReadWrite(0xFF);
        t=sd_spiReadWrite(0xFF);//接收响应
        if((t&0x1F)!=0x05)  return 2;//响应错误
    }
    return 0;//写入成功
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      读取数据
//  @param      缓存区
//  @param      长度
//  @return     0 成功        else 失败
//  @since      v1.0
//  Sample usage:       内部调用
//-------------------------------------------------------------------------------------------------------------------
SDHCRES sd_ReceiveData(uint8 *buf,uint16 len)
{
    if(SdGetResponse(0xFE)) return 1;//等待SD卡发回数据起始令牌0xFE
    while(len--)//开始接收数据
    {
        *buf=sd_spiReadWrite(0xFF);
//        printf("%d\n", *buf);
        buf++;
    }
    //下面是2个伪CRC（dummy CRC）
    sd_spiReadWrite(0xFF);
    sd_spiReadWrite(0xFF);
    return MSD_RESPONSE_NO_ERROR;//读取成功
}

//获取SD卡的CID信息，包括制造商信息
//输入: u8 *cid_data(存放CID的内存，至少16Byte）
//返回值:0：NO_ERR
//       1：错误
uint8 SdGetCID(uint8 *cid_data)
{
    uint8 r1;
    //发CMD10命令，读CID
    r1=sd_sendcmd(CMD10,0,0x01);
    if(r1==0x00)
    {
        r1=sd_ReceiveData(cid_data,16);//接收16个字节的数据
    }
    SD_DisSelect();//取消片选
    if(r1)return 1;
    else return 0;
}

//获取SD卡的CSD信息，包括容量和速度信息
//输入:u8 *cid_data(存放CID的内存，至少16Byte）
//返回值:0：NO_ERR
//       1：错误
uint8 SdGetCSD(uint8 *csd_data)
{
    uint8 r1;
    r1=sd_sendcmd(CMD9,0,0x01);//发CMD9命令，读CSD
    if(r1==0)
    {
        r1=sd_ReceiveData(csd_data, 16);//接收16个字节的数据
    }
    SD_DisSelect();//取消片选
    if(r1)return 1;
    else return 0;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取SD卡的总扇区数（扇区数）
//  @return     0 出错        else 扇区数
//  @since      v1.0
//  Sample usage:       内部调用
//-------------------------------------------------------------------------------------------------------------------
uint32 sd_GetSectorCount(void)
{
    uint8 csd[16];
    uint32 Capacity;
    uint8 n;
    uint16 csize;
    //取CSD信息，如果期间出错，返回0
    if(SdGetCSD(csd)!=0) return 0;
    //如果为SDHC卡，按照下面方式计算
    if((csd[0]&0xC0)==0x40)  //V2.00的卡
    {
        csize = csd[9] + ((uint16)csd[8] << 8) + 1;
        Capacity = (uint32)csize << 10;//得到扇区数
    }else//V1.XX的卡
    {
        n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
        csize = (csd[8] >> 6) + ((uint16)csd[7] << 2) + ((uint16)(csd[6] & 3) << 10) + 1;
        Capacity= (uint32)csize << (n - 9);//得到扇区数
    }
    return Capacity;
}

//读SD卡
//buf:数据缓存区
//sector:扇区
//cnt:扇区数
//返回值:0,ok;其他,失败.
SDHCRES sd_ReadDisk(uint8 *buf,uint32 sector,uint8 cnt)
{
    uint8 r1;
    if (buf == 0 || cnt == 0) return MSD_PARAMETER_ERROR;
    if(SD_Type!=SD_TYPE_V2HC)sector <<= 9;//转换为字节地址
    if(cnt==1)
    {
        r1=sd_sendcmd(CMD17,sector,0X01);//读命令
        if(r1==0)//指令发送成功
        {
            r1=sd_ReceiveData(buf,512);//接收512个字节
        }
    }else
    {
        r1=sd_sendcmd(CMD18,sector,0X01);//连续读命令
        if (r1 != 0) {
            SD_DisSelect();
            return r1;
        }
        do
        {
            r1=sd_ReceiveData(buf,512);//接收512个字节
            buf+=512;
        }while(--cnt && r1==0);
        sd_sendcmd(CMD12,0,0X01);   //发送停止命令
    }
    SD_DisSelect();//取消片选
    return r1;//
}


//写SD卡
//buf:数据缓存区
//sector:起始扇区
//cnt:扇区数
//返回值:0,ok;其他,失败.
SDHCRES sd_WriteDisk(uint8 *buf,uint32 sector,uint8 cnt)
{
    uint8 r1;
    uint8 stop_result;
    if (buf == 0 || cnt == 0) return MSD_PARAMETER_ERROR;
    if(SD_Type!=SD_TYPE_V2HC)sector *= 512;//转换为字节地址
    if(cnt==1)
    {
        r1=sd_sendcmd(CMD24,sector,0X01);//读命令
//        printf("%d\n", r1);
        if(r1==0)//指令发送成功
        {
            r1=sd_SendBlockData(buf, 0xFE);//写512个字节
        }
    }else
    {
        if(SD_Type!=SD_TYPE_MMC)
        {
            sd_sendcmd(CMD55,0,0X01);
            sd_sendcmd(CMD23,cnt,0X01);//发送指令
        }
        r1=sd_sendcmd(CMD25,sector,0X01);//连续写命令
        if(r1==0)
        {
            do
            {
                r1=sd_SendBlockData(buf,0xFC);//接收512个字节
                buf+=512;
            }while(--cnt && r1==0);
            stop_result = sd_SendBlockData(0, 0xFD);
            if (r1 == 0) r1 = stop_result;
        }
    }
    SD_DisSelect();//取消片选
    return r1;//
}

