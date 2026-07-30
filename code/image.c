#include "image.h"
#include "longest_white.h"
#include "algorithm.h"
#include "math_utils.h"
#include "zf_common_font.h"
#include "zf_device_ips200.h"
#include <string.h>

static uint16 width;
static uint16 height;
uint8* binary_image;
int16 os_threshold;
int16 lost_line_cnt;
uint8 already_line_lost;
uint8 left_edge[MT9V03X_H], left_index;
uint8 right_edge[MT9V03X_H], right_index;
uint16 Search_Stop_Line;
int a;
uint8 ostu_deal_threshold(void);
void threshold_update(void);

// struct LEFT_EDGE  L_edge[140];
// struct RIGHT_EDGE R_edge[140];
// uint8 L_edge_count=0, R_edge_count = 0;
int16 Left_Line[MT9V03X_H], Right_Line[MT9V03X_H];
uint8 dire_left,dire_right;                                 //记录上一个点的相对位置
uint8 L_search_amount = 140, R_search_amount = 140;  //左右边界搜点时最多允许的点

//int16 left_down_line = -1, right_down_line = -1, left_up_line = -1, right_up_line = -1;
// uint8 left_corner[4], right[4];
// uint8 left_corner_index = 0, right_corner_index = 0;

void image_init() {
    width = MT9V03X_W;
    height = MT9V03X_H;
    os_threshold = -1;
    lost_line_cnt = LOST_LINE;
    already_line_lost = 0;
    for (int i = 0; i < height; ++i) {
        left_edge[i] = right_edge[i] = 0;
        Left_Line[i] = -1;
        Right_Line[i] = 0x3f3f3f;
    }
    left_index = right_index = 0;
    // Search_Stop_Line = 0;
}

void image_deal(uint8 start_y, uint8 end_y) {
    memcpy(binary_image, mt9v03x_image, sizeof(mt9v03x_image));
    if (ostu_deal_threshold() || Longest_White_Column_Left[0]<10) {
        if (--lost_line_cnt <= 0) {
            already_line_lost = 1;
            return;
        }
    }
    else {
        lost_line_cnt= LOST_LINE;
    }
    threshold_update();
    Longest_White_Column();
    // get_highest();
    // image_draw_rectan(binary_image);
    // search_neighborhood();
    // edge_real_update();

    // shizibuxian();
}

// // Legacy edge-following helpers are disabled until their removed interfaces are restored.
// /*-------------------------------------------------------------------------------------------------------------------
//   @brief     左下角点检测
//   @param     起始行，终止行
//   @return    返回角点所在的行数，找不到返回0
//   Sample     left_down_guai[0]=Find_Left_Down_Point(MT9V03X_H-1,20);
//   @note      角点检测阈值可根据实际值更改
// -------------------------------------------------------------------------------------------------------------------*/
int Find_Left_Down_Point(int start,int end)//找左下角点，返回值是角点所在的行数
{
    int i,t;
    int left_down_line=0;
    if(Left_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没有拐点判断的意义
       return left_down_line;
    if(start<end)//--访问，要保证start>end
    {
        t=start;
        start=end;
        end=t;
    }
    if(start>=MT9V03X_H-1-5)//下面5行上面5行数据不稳定，不能作为边界点来判断，舍弃
        start=MT9V03X_H-1-5;//另一方面，当判断第i行时，会访问到i+3和i-4行，防止越界
    if(end<=MT9V03X_H-Search_Stop_Line)
        end=MT9V03X_H-Search_Stop_Line;
    if(end<=5)
       end=5;
    for(i=start;i>=end;--i)
    {
        if(left_down_line==0&&//只找第一个符合条件的点
           abs(Left_Line[i]-Left_Line[i+1])<=5&&//角点的阈值可以更改
           abs(Left_Line[i+1]-Left_Line[i+2])<=5&&  
           abs(Left_Line[i+2]-Left_Line[i+3])<=5&&
              (Left_Line[i]-Left_Line[i-2])>=5&&
              (Left_Line[i]-Left_Line[i-3])>=10&&
              (Left_Line[i]-Left_Line[i-4])>=10)
        {
            left_down_line=i;//获取行数即可
            break;
        }
    }
    return left_down_line;
}
int Find_Right_Down_Point(int start,int end)//找左下角点，返回值是角点所在的行数
{
    int i,t;
    int right_down_line=0;
    if(Right_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没有拐点判断的意义
       return right_down_line;
    if(start<end)//--访问，要保证start>end
    {
        t=start;
        start=end;
        end=t;
    }
    if(start>=MT9V03X_H-1-5)//下面5行上面5行数据不稳定，不能作为边界点来判断，舍弃
        start=MT9V03X_H-1-5;//另一方面，当判断第i行时，会访问到i+3和i-4行，防止越界
    if(end<=MT9V03X_H-Search_Stop_Line)
        end=MT9V03X_H-Search_Stop_Line;
    if(end<=5)
       end=5;
    for(i=start;i>=end;--i)
    {
        // if(right_down_line==0&&//只找第一个符合条件的点
        //    abs(Right_Line[i]-Right_Line[i+1])<=5&&//角点的阈值可以更改
        //    abs(Right_Line[i+1]-Right_Line[i+2])<=5&&  
        //    abs(Right_Line[i+2]-Right_Line[i+3])<=5&&
        //       (Right_Line[i]-Right_Line[i-2])>=5&&
        //       (Right_Line[i]-Right_Line[i-3])>=10&&
        //       (Right_Line[i]-Right_Line[i-4])>=10)
        // {
        //     right_down_line=i;//获取行数即可
        //     break;
        // }
        if(right_down_line==0&&
            abs(Right_Line[i]-Right_Line[i+1])<=5&&
            abs(Right_Line[i+1]-Right_Line[i+2])<=5&&
            abs(Right_Line[i+2]-Right_Line[i+3])<=5&&
            (Right_Line[i-2]-Right_Line[i])>=5&&
            (Right_Line[i-3]-Right_Line[i])>=10&&
            (Right_Line[i-4]-Right_Line[i])>=10)
        {
            right_down_line=i;
            break;
        }
    }
    return right_down_line;
}
int Find_Left_Up_Point(int start,int end)//找左下角点，返回值是角点所在的行数
{
    int i,t;
    int left_up_line=0;
    if(Left_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没有拐点判断的意义
       return left_up_line;
    if(start<end)//--访问，要保证start>end
    {
        t=start;
        start=end;
        end=t;
    }
    if(start>=MT9V03X_H-1-5)//下面5行上面5行数据不稳定，不能作为边界点来判断，舍弃
        start=MT9V03X_H-1-5;//另一方面，当判断第i行时，会访问到i+3和i-4行，防止越界
    if(end<=MT9V03X_H-Search_Stop_Line)
        end=MT9V03X_H-Search_Stop_Line;
    if(end<=5)
       end=5;
    for(i=start;i>=end;--i)
    {
        // if(left_up_line==0&&//只找第一个符合条件的点
        //    abs(Left_Line[i]-Left_Line[i+1])<=5&&//角点的阈值可以更改
        //    abs(Left_Line[i+1]-Left_Line[i+2])<=5&&  
        //    abs(Left_Line[i+2]-Left_Line[i+3])<=5&&
        //       (Left_Line[i]-Left_Line[i-2])>=5&&
        //       (Left_Line[i]-Left_Line[i-3])>=10&&
        //       (Left_Line[i]-Left_Line[i-4])>=10)
        // {
        //     left_up_line=i;//获取行数即可
        //     break;
        // }
        if(left_up_line==0&&
            abs(Left_Line[i]-Left_Line[i+1])<=5&&
            abs(Left_Line[i+1]-Left_Line[i+2])<=5&&
            abs(Left_Line[i+2]-Left_Line[i+3])<=5&&
            (Left_Line[i]-Left_Line[i+2])>=5&&
            (Left_Line[i]-Left_Line[i+3])>=10&&
            (Left_Line[i]-Left_Line[i+4])>=10)
        {
            left_up_line=i;
            break;
        }
    }
    return left_up_line;
}
int Find_Right_Up_Point(int start,int end)//找左下角点，返回值是角点所在的行数
{
    int i,t;
    int right_up_line=0;
//    if(Left_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没有拐点判断的意义
//       return right_up_line;
    if(start<end)//--访问，要保证start>end
    {
        t=start;
        start=end;
        end=t;
    }
    if(start>=MT9V03X_H-1-5)//下面5行上面5行数据不稳定，不能作为边界点来判断，舍弃
        start=MT9V03X_H-1-5;//另一方面，当判断第i行时，会访问到i+3和i-4行，防止越界
    if(end<=MT9V03X_H-Search_Stop_Line)
        end=MT9V03X_H-Search_Stop_Line;
    if(end<=5)
       end=5;
    for(i=start;i>=end;--i)
    {
        // if(right_up_line==0&&//只找第一个符合条件的点
        //    abs(Left_Line[i]-Left_Line[i+1])<=5&&//角点的阈值可以更改
        //    abs(Left_Line[i+1]-Left_Line[i+2])<=5&&  
        //    abs(Left_Line[i+2]-Left_Line[i+3])<=5&&
        //       (Left_Line[i]-Left_Line[i-2])>=5&&
        //       (Left_Line[i]-Left_Line[i-3])>=10&&
        //       (Left_Line[i]-Left_Line[i-4])>=10)
        // {
        //     right_up_line=i;//获取行数即可
        //     break;
        // }
        if(right_up_line==0&&
                abs(Right_Line[i]-Right_Line[i+1])<=5&&
                abs(Right_Line[i+1]-Right_Line[i+2])<=5&&
                abs(Right_Line[i+2]-Right_Line[i+3])<=5&&
                (Right_Line[i+2]-Right_Line[i])>=5&&
                (Right_Line[i+3]-Right_Line[i])>=10&&
                (Right_Line[i+4]-Right_Line[i])>=10)
            {
                right_up_line=i;
                break;
            }
    }
    return right_up_line;
}


// void shizibuxian() {
//     int left_down_line = Find_Left_Down_Point(0, height);
//     int left_up_line = Find_Left_Up_Point(0, height);
//     int right_down_line = Find_Right_Down_Point(0, height);
//     int right_up_line = Find_Right_Up_Point(0, height);
// }
void shizibuxian()
{
    int left_up=Find_Left_Up_Point(height-6,10);
    int right_up=Find_Right_Up_Point(height-6,10);
    //左右同时检测到，认为十字
    if(left_up>0&&right_up>0)
    {
        Left_Add_Line(
            Left_Line[left_up],
            left_up,
            Left_Line[left_up],
            height-1
        );


        Right_Add_Line(
            Right_Line[right_up],
            right_up,
            Right_Line[right_up],
            height-1
        );
    }
}

// void edge_real_update() {
//     for (int i = 0; i < L_edge_count; ++i) {
//         if (L_edge[i].flag)
//             Left_Line[L_edge[i].col] = cc_i16_max(Left_Line[L_edge[i].col], L_edge[i].row);
//     }
//     for (int i = 0; i < R_edge_count; ++i) {
//         if (R_edge[i].flag)
//             Right_Line[R_edge[i].col] = cc_i16_min(Right_Line[R_edge[i].col], R_edge[i].row);
//     }
//     for (int i = 0; i < L_edge_count && i < R_edge_count; ++i) {
//         if (!L_edge[i].flag && !R_edge[i].flag) {
//             ++a;
//         }
//     }
// }


// /*---------------------------------------------------------------
//  【函    数】search_neighborhood
//  【功    能】八邻域找边界
//  【参    数】无
//  【返 回 值】无
//  【注意事项】
//  ----------------------------------------------------------------*/

// void search_neighborhood(void)
// {
//     L_edge_count = 0;//左边点个数清0
//     R_edge_count = 0;//右边点个数清0

//     if(left_findflag)//如果左边界点存在并找到,则开始爬线
//     {
//         //变量声明
//         L_edge[0].row = L_start_y;
//         L_edge[0].col = L_start_x;
//         L_edge[0].flag = 1;
//         int16 curr_row = L_start_y;//初始化行坐标
//         int16 curr_col = L_start_x;//初始化列坐标
//         dire_left = 0; //初始化上个边界点的来向
//         //开始搜线，最多取150个点，不会往下搜，共7个方位
//         for(int i = 1;i < L_search_amount; i++)    //最多搜索150个点
//         {
//             ////越界退出 行越界和列越界（向上向下向左向右）
//             if(curr_row+1 < Boundary_search_end || curr_row>IMAGE_H-1)  break;
//             if (curr_col-1 <= 0 && curr_col+1 < IMAGE_W) break;
//             //搜线过程
//             if(dire_left != 2&&image_use[curr_row-1][curr_col-1]==0&&image_use[curr_row-1][curr_col]==1)   //左上黑，2，右边白
//             {
//                 curr_row = curr_row -1;
//                 curr_col = curr_col -1;
//                 L_edge_count = L_edge_count +1;
//                 dire_left = 7;
//                 L_edge[i].row = curr_row;
//                 L_edge[i].col = curr_col;
//                 L_edge[i].flag = 1;
//             }
//             else if(dire_left!=3&&image_use[curr_row-1][curr_col+1]==0&&image_use[curr_row][curr_col+1]==1)    //右上黑，3，下边白
//             {
//                 curr_row = curr_row -1;
//                 curr_col = curr_col + 1;
//                 L_edge_count = L_edge_count + 1;
//                 dire_left = 6;
//                 L_edge[i].row = curr_row;
//                 L_edge[i].col = curr_col;
//                 L_edge[i].flag = 1;
//             }
//             else if(image_use[curr_row-1][curr_col]==0&&image_use[curr_row-1][curr_col+1]==1)                  //正上黑，1，右白
//             {
//                 curr_row = curr_row - 1;
//                 L_edge_count = L_edge_count + 1;
//                 dire_left = 0;
//                 L_edge[i].row = curr_row;
//                 L_edge[i].col = curr_col;
//                 L_edge[i].flag = 1;
//             }
//             else if(dire_left!=5&&image_use[curr_row][curr_col-1]==0&&image_use[curr_row-1][curr_col-1]==1)     //正左黑，5，上白
//             {
//                 curr_col = curr_col - 1;
//                 L_edge_count = L_edge_count +1;
//                 dire_left = 4;
//                 L_edge[i].row = curr_row;
//                 L_edge[i].col = curr_col;
//                 L_edge[i].flag = 1;
//             }
//             else if(dire_left!=4&&image_use[curr_row][curr_col+1]==0&&image_use[curr_row+1][curr_col+1]==1)  //正右黑，4，下白
//             {
//                 curr_col = curr_col + 1;
//                 L_edge_count = L_edge_count +1;
//                 dire_left = 5;
//                 L_edge[i].row = curr_row;
//                 L_edge[i].col = curr_col;
//                 L_edge[i].flag = 1;
//             }
//             else if(dire_left!=6&&image_use[curr_row+1][curr_col-1]==0&&image_use[curr_row][curr_col-1]==1)    //左下黑，6，上白
//             {
//                 curr_row = curr_row + 1;
//                 curr_col = curr_col -1;
//                 L_edge_count = L_edge_count +1;
//                 dire_left = 3;
//                 L_edge[i].row = curr_row;
//                 L_edge[i].col = curr_col;
//                 L_edge[i].flag = 1;
//             }
//             else if(dire_left!=7&&image_use[curr_row+1][curr_col+1]==0&&image_use[curr_row+1][curr_col]==1)    //右下黑，7，左白
//             {
//                 curr_row = curr_row + 1;
//                 curr_col = curr_col + 1;
//                 L_edge_count = L_edge_count +1;
//                 dire_left = 2;
//                 L_edge[i].row = curr_row;
//                 L_edge[i].col = curr_col;
//                 L_edge[i].flag = 1;
//             }
//             else
//                 break;
//         }
//     }

//     if(right_findflag)//如果右边界存在并搜到
//     {
//         R_edge[0].row = R_start_y;
//         R_edge[0].col = R_start_x;
//         R_edge[0].flag = 1;
//         int16 curr_row = R_start_y;
//         int16 curr_col = R_start_x;
//         dire_right = 0;
//         for(int i = 1;i<R_search_amount;i++)
//         {
//             ////越界退出 行越界和列越界（向上向下向左向右）
//             if(curr_row < Boundary_search_end || curr_row>IMAGE_H-1||curr_row+1<Boundary_search_end)  break;
//             //爬线过程
//             if(curr_col<IMAGE_W&&dire_right!=3&&image_use[curr_row-1][curr_col+1]==0&&image_use[curr_row-1][curr_col]==1)    //右上黑，3，左白
//             {
//                 curr_row = curr_row - 1;
//                 curr_col = curr_col + 1;
//                 R_edge_count = R_edge_count + 1;
//                 dire_right = 6;
//                 R_edge[i].row = curr_row;
//                 R_edge[i].col = curr_col;
//                 R_edge[i].flag = 1;
//             }
//             else if(dire_right!=2&&image_use[curr_row-1][curr_col-1]==0&&image_use[curr_row][curr_col-1]==1) //左上黑，2，下白
//             {
//                 curr_row = curr_row-1;
//                 curr_col = curr_col-1;
//                 R_edge_count = R_edge_count + 1;
//                 dire_right = 7;
//                 R_edge[i].row = curr_row;
//                 R_edge[i].col = curr_col;
//                 R_edge[i].flag = 1;
//             }
//             else if(image_use[curr_row-1][curr_col]==0&&image_use[curr_row-1][curr_col-1]==1)                  //正上黑，1，左白
//             {
//                 curr_row = curr_row - 1;
//                 R_edge_count = R_edge_count + 1;
//                 dire_right = 0;
//                 R_edge[i].row = curr_row;
//                 R_edge[i].col = curr_col;
//                 R_edge[i].flag = 1;
//             }
//             else if(dire_right!=4&&image_use[curr_row][curr_col+1]==0&&image_use[curr_row-1][curr_col+1]==1)   //正右黑，4，上白
//             {
//                 curr_col = curr_col + 1;
//                 R_edge_count = R_edge_count + 1;
//                 dire_right = 5;
//                 R_edge[i].row = curr_row;
//                 R_edge[i].col = curr_col;
//                 R_edge[i].flag = 1;
//             }
//             else if(dire_right!=5&&image_use[curr_row][curr_col-1]==0&&image_use[curr_row+1][curr_col-1]==1)   //正左黑，5，下白
//             {
//                 curr_col = curr_col-1;
//                 R_edge_count = R_edge_count + 1;
//                 dire_right = 4;
//                 R_edge[i].row = curr_row;
//                 R_edge[i].col = curr_col;
//                 R_edge[i].flag = 1;
//             }


//             else if(dire_right!=6&&image_use[curr_row+1][curr_col-1]==0&&image_use[curr_row+1][curr_col]==1)   //左下黑，6，右白
//             {
//                 curr_row = curr_row + 1;
//                 curr_col = curr_col - 1;
//                 R_edge_count = R_edge_count + 1;
//                 dire_right = 3;
//                 R_edge[i].row = curr_row;
//                 R_edge[i].col = curr_col;
//                 R_edge[i].flag = 1;
//             }
//             else if(dire_right!=7&&image_use[curr_row+1][curr_col+1]==0&&image_use[curr_row][curr_col+1]==1)   //右下黑，7，上白
//             {
//                 curr_row = curr_row + 1;
//                 curr_col = curr_col + 1;
//                 R_edge_count = R_edge_count + 1;
//                 dire_right = 2;
//                 R_edge[i].row = curr_row;
//                 R_edge[i].col = curr_col;
//                 R_edge[i].flag = 1;
//             }
//             else
//                 break;
//         }
//     }
// }


// void image_draw_rectan(uint8(*image)[width])
// {
//     uint8 i = 0;
//     for (i = 0; i < height; i++)
//     {
//         image[i][0] = 0;
//         image[i][1] = 0;
//         image[i][width - 1] = 0;
//         image[i][width - 2] = 0;
//     }
//     for (i = 0; i < width; i++)
//     {
//         image[0][i] = 0;
//         image[1][i] = 0;
//     }
// }



uint8 ostu_deal_threshold() { //懒得写注释，反正大津感觉就是套公式
    uint32 white_cnt = 0;
    int16 PixelMax = 0, PixelMin = 255;
    uint32 PixelSum = width*height/4;
    int PixelCnt[GrayScale] = {0};
    float PixelPro[GrayScale] = {0};
    uint32 Graysum = 0;
    for (int i = 0; i < height; i += 2) {
        for (int j = 0; j < width; j += 2) {
            int GrayCur = binary_image[i*width + j];
            if (GrayCur > 200) {
                ++white_cnt;
            }
            Graysum += GrayCur;
            ++PixelCnt[GrayCur];
            PixelMax = cc_int16_max(PixelMax, GrayCur);
            PixelMin = cc_int16_min(PixelMin, GrayCur);
        }
    }
    for (int i = PixelMin; i < PixelMax; ++i) {
        PixelPro[i] = PixelCnt[i]/PixelSum;
    }
//    float w0, w1, u0temp, u1temp, deltatemp, deltamax;
    float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;

    w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
    for (int j = PixelMin; j < PixelMax; ++j)
    {

        w0 += PixelPro[j];  //背景部分每个灰度值的像素点所占比例之和   即背景部分的比例
        u0tmp += j * PixelPro[j];  //背景部分 每个灰度值的点的比例 *灰度值

        w1=1-w0;
        u1tmp=Graysum/PixelSum-u0tmp;

        u0 = u0tmp / w0;              //背景平均灰度
        u1 = u1tmp / w1;              //前景平均灰度
        u = u0tmp + u1tmp;            //全局平均灰度
        deltaTmp = (float)(w0 *w1* (u0 - u1)* (u0 - u1)) ;
        if (deltaTmp > deltaMax)
        {
            deltaMax = deltaTmp;
            os_threshold = j;
        }
//        if (deltaTmp < deltaMax)
//        {
//            break;
//        }

    }
//    if(os_threshold>90 && os_threshold<130)
//        last_threshold = os_threshold;
//    else
//        os_threshold = last_threshold;
    return white_cnt < PixelSum*0.1;
}

void threshold_update() {
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            if (binary_image[i*width+j] > os_threshold) {
                binary_image[i*width+j] = 1;
            }
            else {
                binary_image[i*width+j] = 0;
            }
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     寻找最长白列     CYH-NYY
// 参数说明
// 返回参数
// 使用示例     get_highest();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------

int lwline = 94;   //最长白列所在x
int lw = 119;      //最长白列顶部y
int xxx = 0;


void get_highest(void)
{
    lwline = width / 2;
    lw = height;
    xxx = 0;


    // 从中间区域向左右搜索
    for(int x = width/3; x < width*2/3; x += 3)
    {
        // 最下面一行必须是白，否则不是道路入口
        if(binary_image[(height-1)*width+x] == 1)
        {
            for(int y = height-1; y >= 0; y--)
            {
                // 白黑黑跳变
                if(binary_image[y*width+x] == 1 &&
                   binary_image[(y-1)*width+x] == 0 &&
                   binary_image[(y-2)*width+x] == 0)
                {
                    // 找最长白列
                    if(y < lw)
                    {
                        lw = y;
                        lwline = x;
                    }

                    break;
                }


                // 防止越界
                if(y <= 2)
                {
                    if(y < lw)
                    {
                        lw = y;
                        lwline = x;
                    }
                    break;
                }
            }
        }
    }
}
