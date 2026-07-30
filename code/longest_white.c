#include "headfile.h"


int16 Longest_White_Column_Left[2];
int16 Longest_White_Column_Right[2];
uint16 White_Column[MT9V03X_W];
uint8 Left_Lost_Flag[MT9V03X_H];
uint8 Right_Lost_Flag[MT9V03X_H];
int16 Boundry_Start_Left;
int16 Boundry_Start_Right;
int16 Road_Wide[MT9V03X_H];
int16 Left_Lost_Time;
int16 Right_Lost_Time;
int16 Both_Lost_Time;

/*-------------------------------------------------------------------------------------------------------------------
  @brief     双最长白列巡线
  @param     null
  @return    null
  Sample     Longest_White_Column_Left();
  @note      最长白列巡线，寻找初始边界，丢线，最长白列等基础元素，后续读取这些变量来进行赛道识别
-------------------------------------------------------------------------------------------------------------------*/
void Longest_White_Column(void)//最长白列巡线
{
    int i, j;
    int start_column=20;//最长白列的搜索区间
    int end_column=MT9V03X_W-20;
    int left_border = 0, right_border = 0;//临时存储赛道位置
    Longest_White_Column_Left[0] = 0;//最长白列,[0]是最长白列的长度，[1】是第某列
    Longest_White_Column_Left[1] = 0;//最长白列,[0]是最长白列的长度，[1】是第某列
    Longest_White_Column_Right[0] = 0;//最长白列,[0]是最长白列的长度，[1】是第某列
    Longest_White_Column_Right[1] = 0;//最长白列,[0]是最长白列的长度，[1】是第某列
    Right_Lost_Time = 0;    //边界丢线数
    Left_Lost_Time  = 0;
    Boundry_Start_Left  = 0;//第一个非丢线点,常规边界起始点
    Boundry_Start_Right = 0;
    Both_Lost_Time = 0;//两边同时丢线数
 
    for (i = 0; i <=MT9V03X_H-1; i++)//数据清零
    {
        Right_Lost_Flag[i] = 0;
        Left_Lost_Flag[i] = 0;
        Left_Line[i] = 0;
        Right_Line[i] = MT9V03X_W-1;
    }
    for(i=0;i<=MT9V03X_W-1;i++)
    {
        White_Column[i] = 0;
    }
 
//环岛需要对最长白列范围进行限定
    //环岛3状态需要改变最长白列寻找范围
    if(Right_Island_Flag==1)//右环
    {
        if(Island_State==3)
        {
            start_column=40;
            end_column=MT9V03X_W-20;
        }
    }
    else if(Left_Island_Flag==1)//左环
    {
        if(Island_State==3)
        {
            start_column=20;
            end_column=MT9V03X_W-40;
        }
    }
 
    //从左到右，从下往上，遍历全图记录范围内的每一列白点数量
    for (j =start_column; j<=end_column; j++)
    {
        for (i = MT9V03X_H - 1; i >= 0; i--)
        {
            if(binary_image[i * MT9V03X_W + j] == 0)
                break;
            else
                White_Column[j]++;
        }
    }
 
    //从左到右找左边最长白列
    Longest_White_Column_Left[0] =0;
    for(i=start_column;i<=end_column;i++)
    {
        if (Longest_White_Column_Left[0] < White_Column[i])//找最长的那一列
        {
            Longest_White_Column_Left[0] = White_Column[i];//【0】是白列长度
            Longest_White_Column_Left[1] = i;              //【1】是下标，第j列
        }
    }
    //从右到左找右左边最长白列
    Longest_White_Column_Right[0] = 0;//【0】是白列长度
    for(i=end_column;i>=start_column;i--)//从右往左，注意条件，找到左边最长白列位置就可以停了
    {
        if (Longest_White_Column_Right[0] < White_Column[i])//找最长的那一列
        {
            Longest_White_Column_Right[0] = White_Column[i];//【0】是白列长度
            Longest_White_Column_Right[1] = i;              //【1】是下标，第j列
        }
    }
 
    Search_Stop_Line = Longest_White_Column_Left[0];//搜索截止行选取左或者右区别不大，他们两个理论上是一样的
    if(Search_Stop_Line < 5) {
        Search_Stop_Line = 5;
    }
    if(Search_Stop_Line > MT9V03X_H) {
        Search_Stop_Line = MT9V03X_H;
    }
    for (i = MT9V03X_H - 1; i >=MT9V03X_H-Search_Stop_Line; i--)//常规巡线
    {
        for (j = Longest_White_Column_Right[1]; j <= MT9V03X_W - 1 - 2; j++)
        {
            if (binary_image[i * MT9V03X_W + j] == 1 && binary_image[i * MT9V03X_W + j + 1] == 0 &&
                binary_image[i * MT9V03X_W + j + 2] == 0)//白黑黑，找到右边界
            {
                right_border = j;
                Right_Lost_Flag[i] = 0; //右丢线数组，丢线置1，不丢线置0
                break;
            }
            else if(j>=MT9V03X_W-1-2)//没找到右边界，把屏幕最右赋值给右边界
            {
                right_border = j;
                Right_Lost_Flag[i] = 1; //右丢线数组，丢线置1，不丢线置0
                break;
            }
        }
        for (j = Longest_White_Column_Left[1]; j >= 0 + 2; j--)//往左边扫描
        {
            if (binary_image[i * MT9V03X_W + j] == 1 && binary_image[i * MT9V03X_W + j - 1] == 0 &&
                binary_image[i * MT9V03X_W + j - 2] == 0)//黑黑白认为到达左边界
            {
                left_border = j;
                Left_Lost_Flag[i] = 0; //左丢线数组，丢线置1，不丢线置0
                break;
            }
            else if(j<=0+2)
            {
                left_border = j;//找到头都没找到边，就把屏幕最左右当做边界
                Left_Lost_Flag[i] = 1; //左丢线数组，丢线置1，不丢线置0
                break;
            }
        }
        Left_Line [i] = left_border;       //左边线线数组
        Right_Line[i] = right_border;      //右边线线数组
    }
 
    for (i = MT9V03X_H - 1; i >= 0; i--)//赛道数据初步分析
    {
        if (Left_Lost_Flag[i]  == 1)//单边丢线数
            Left_Lost_Time++;
        if (Right_Lost_Flag[i] == 1)
            Right_Lost_Time++;
        if (Left_Lost_Flag[i] == 1 && Right_Lost_Flag[i] == 1)//双边丢线数
            Both_Lost_Time++;
        if (Boundry_Start_Left ==  0 && Left_Lost_Flag[i]  != 1)//记录第一个非丢线点，边界起始点
            Boundry_Start_Left = i;
        if (Boundry_Start_Right == 0 && Right_Lost_Flag[i] != 1)
            Boundry_Start_Right = i;
        Road_Wide[i]=Right_Line[i]-Left_Line[i];
    }
 
    // //环岛3状态改变边界，看情况而定，我认为理论上的最优情况是不需要这些处理的
    // if(Island_State==3||Island_State==4)
    // {
    //     if(Right_Island_Flag==1)//右环
    //     {
    //         for (i = MT9V03X_H - 1; i >= 0; i--)//右边直接写在边上
    //         {
    //             Right_Line[i]=MT9V03X_W-1;
    //         }
    //     }
    //     else if(Left_Island_Flag==1)//左环
    //     {
    //         for (i = MT9V03X_H - 1; i >= 0; i--)//左边直接写在边上
    //         {
    //             Left_Line[i]=0;      //右边线线数组
    //         }
    //     }
    // }
    //debug使用，屏幕显示相关参数
//    ips200_showint16(0,0, Longest_White_Column_Right[0]);//【0】是白列长度
//    ips200_showint16(0,1, Longest_White_Column_Right[1]);//【1】是下标，第j列)
//    ips200_showint16(0,2, Longest_White_Column_Left[0]);//【0】是白列长度
//    ips200_showint16(0,3, Longest_White_Column_Left[1]);//【1】是下标，第j列)
}
