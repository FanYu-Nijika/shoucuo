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
static int16 Previous_Longest_White_X;
static uint8 Previous_Longest_White_Valid;

void Longest_White_Column_Reset(void)
{
    Previous_Longest_White_X = MT9V03X_W / 2;
    Previous_Longest_White_Valid = 0;
}

void Longest_White_Column(void)// 最长白列搜索和赛道边界提取
{
    int i, j;
    int start_column=20;// 初始搜索范围避开图像左右边缘
    int end_column=MT9V03X_W-20;
    int search_radius = 40;
    int reference_x = MT9V03X_W / 2;
    int longest_x = reference_x;
    int boundary_seed;
    int current_distance;
    int longest_distance;
    uint16 longest_length = 0;
    int left_border = 0, right_border = 0;// 左右边界坐标
    Longest_White_Column_Left[0] = 0;// 左侧最长白列长度初始化
    Longest_White_Column_Left[1] = 0;// 左侧最长白列位置初始化
    Longest_White_Column_Right[0] = 0;// 右侧最长白列长度初始化
    Longest_White_Column_Right[1] = 0;// 右侧最长白列位置初始化
    Right_Lost_Time = 0;    // 右边界丢失行数
    Left_Lost_Time  = 0;
    Boundry_Start_Left  = 0;// 记录左边界有效起始行
    Boundry_Start_Right = 0;
    Both_Lost_Time = 0;// 左右边界同时丢失的行数
 
    for (i = 0; i <=MT9V03X_H-1; i++)// 每帧清空边界和丢线标志
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

    /* A valid previous column is the only center used after the first frame. */
    if (Previous_Longest_White_Valid != 0) {
        reference_x = Previous_Longest_White_X;
        start_column = reference_x - search_radius;
        end_column = reference_x + search_radius;
        if (start_column < 0) start_column = 0;
        if (end_column > MT9V03X_W - 1) end_column = MT9V03X_W - 1;
    }
    longest_x = reference_x;
 
    /* The old island branch depended on removed state variables. The current
     * shoucuo pipeline keeps the normal search window and handles cross repair
     * in image.c, so no island-specific range is applied here. */
 
    // 逐列统计从图像底部开始连续的白色像素数
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
 
    /* Equal maxima stay as close as possible to the previous column. */
    for (i = start_column; i <= end_column; i++) {
        current_distance = abs(i - reference_x);
        longest_distance = abs(longest_x - reference_x);
        if (White_Column[i] > longest_length ||
            (White_Column[i] == longest_length && White_Column[i] > 0 &&
             (current_distance < longest_distance ||
              (current_distance == longest_distance && i < longest_x)))) {
            longest_length = White_Column[i];
            longest_x = i;
        }
    }

    if (longest_length > 0) {
        Previous_Longest_White_X = longest_x;
        Previous_Longest_White_Valid = 1;
    }

    Longest_White_Column_Left[0] = longest_length;
    Longest_White_Column_Left[1] = longest_x;
    Longest_White_Column_Right[0] = longest_length;
    Longest_White_Column_Right[1] = longest_x;

    /* Boundary scans need two neighboring pixels on each side. */
    boundary_seed = longest_x;
    if (boundary_seed < 2) boundary_seed = 2;
    if (boundary_seed > MT9V03X_W - 3) boundary_seed = MT9V03X_W - 3;

    Search_Stop_Line = Longest_White_Column_Left[0];// 最长白列高度作为有效搜索行数
    if(Search_Stop_Line < 5) {
        Search_Stop_Line = 5;
    }
    if(Search_Stop_Line > MT9V03X_H) {
        Search_Stop_Line = MT9V03X_H;
    }
    for (i = MT9V03X_H - 1; i >=MT9V03X_H-Search_Stop_Line; i--)// 只在有效高度内从底部向上搜索边界
    {
        for (j = boundary_seed; j <= MT9V03X_W - 1 - 2; j++)
        {
            if (binary_image[i * MT9V03X_W + j] == 1 && binary_image[i * MT9V03X_W + j + 1] == 0 &&
                binary_image[i * MT9V03X_W + j + 2] == 0)// 白点右侧连续两个黑点时判定为右边界
            {
                right_border = j;
                Right_Lost_Flag[i] = 0; // 找到右边界，0表示未丢失
                break;
            }
            else if(j>=MT9V03X_W-1-2)// 到达图像右侧仍未找到右边界
            {
                right_border = j;
                Right_Lost_Flag[i] = 1; // 未找到右边界，标记为丢失
                break;
            }
        }
        for (j = boundary_seed; j >= 0 + 2; j--)// 从种子列向左搜索边界
        {
            if (binary_image[i * MT9V03X_W + j] == 1 && binary_image[i * MT9V03X_W + j - 1] == 0 &&
                binary_image[i * MT9V03X_W + j - 2] == 0)// 白点左侧连续两个黑点时判定为左边界
            {
                left_border = j;
                Left_Lost_Flag[i] = 0; // 找到左边界，0表示未丢失
                break;
            }
            else if(j<=0+2)
            {
                left_border = j;// 到达图像左侧仍未找到左边界
                Left_Lost_Flag[i] = 1; // 未找到左边界，标记为丢失
                break;
            }
        }
        Left_Line [i] = left_border;       // 保存当前行的左边界
        Right_Line[i] = right_border;      // 保存当前行的右边界
    }
 
    for (i = MT9V03X_H - 1; i >= 0; i--)// 统计各行丢线情况并记录边界位置
    {
        if (Left_Lost_Flag[i]  == 1)// 左边界丢失时累计行数
            Left_Lost_Time++;
        if (Right_Lost_Flag[i] == 1)
            Right_Lost_Time++;
        if (Left_Lost_Flag[i] == 1 && Right_Lost_Flag[i] == 1)// 左右边界同时丢失时累计行数
            Both_Lost_Time++;
        if (Boundry_Start_Left ==  0 && Left_Lost_Flag[i]  != 1)// 记录左边界首个有效行
            Boundry_Start_Left = i;
        if (Boundry_Start_Right == 0 && Right_Lost_Flag[i] != 1)
            Boundry_Start_Right = i;
        Road_Wide[i]=Right_Line[i]-Left_Line[i];
    }
 
    // // 旧版环岛补线逻辑，当前未启用
    // if(Island_State==3||Island_State==4)
    // {
    //     if(Right_Island_Flag==1)// 右环岛时将右边界置于图像右侧
    //     {
    //         for (i = MT9V03X_H - 1; i >= 0; i--)// 将所有行的右边界设为图像最右列
    //         {
    //             Right_Line[i]=MT9V03X_W-1;
    //         }
    //     }
    //     else if(Left_Island_Flag==1)// 左环岛时将左边界置于图像左侧
    //     {
    //         for (i = MT9V03X_H - 1; i >= 0; i--)// 将所有行的左边界设为图像最左列
    //         {
    //             Left_Line[i]=0;      // 将左边界置为图像最左列
    //         }
    //     }
    // }
    // 调试输出
//    ips200_showint16(0,0, Longest_White_Column_Right[0]);// 显示右侧最长白列长度
//    ips200_showint16(0,1, Longest_White_Column_Right[1]);// 显示右侧最长白列位置
//    ips200_showint16(0,2, Longest_White_Column_Left[0]);// 显示左侧最长白列长度
//    ips200_showint16(0,3, Longest_White_Column_Left[1]);// 显示左侧最长白列位置
}
