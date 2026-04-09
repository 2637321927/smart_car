#include "lq_all_demo.hpp"
typedef signed short       sint16;  // 16位 有符号  <-- 你要的

// =====================================================
// 配置参数 - 根据需要修改
// =====================================================
// 目标IP地址（UDP接收端）
uint8_t otsuThreshold(uint8_t *image, uint16_t col, uint16_t row);
typedef signed short sint16;
//encoder


// 巡线全局变量
sint16 Longest_White_Column_Left[2];
sint16 Longest_White_Column_Right[2];
sint16 Longest_White_Column_Left1[2]; 
sint16 Longest_White_Column_Right1[2];
// 巡线边界、中线、丢线标记数组
sint16 Left_Line[LCDH];    // 每行左边界
sint16 Right_Line[LCDH];   // 每行右边界
sint16 Mid_Line[LCDH];     // 每行中心线（最终输出）
sint16 Left_Lost_Flag[LCDH];  // 左边界丢线标记
sint16 Right_Lost_Flag[LCDH]; // 右边界丢线标记
sint16 Road_Wide[LCDH];    // 赛道宽度（你代码也用到了）
// ======================
// 丢线计数、起始行
// ======================
sint16 Right_Lost_Time;
sint16 Left_Lost_Time;
sint16 Both_Lost_Time;
sint16 Boundry_Start_Left;
sint16 Boundry_Start_Right;
sint16 Search_Stop_Line;
// 元素标志
uint8_t CrossFlag = 0;        // 十字路口标志
uint8_t RoundLeftFlag = 0;    // 左环岛标志
uint8_t RoundRightFlag = 0;   // 右环岛标志
//======================== 拐点坐标 ========================
int Right_Down_Point[2];       // 右下拐点 (x,y)
int Left_Down_Point[2];        // 左下拐点 (x,y)
int Right_Up_Point[2];         // 右上拐点 (x,y)
int Left_Up_Point[2];          // 左上拐点

//======================== 上一帧拐点坐标 ========================
int Last_Right_Down_Point[2];  // 上一次右下拐点
int Last_Left_Down_Point[2];   // 上一次左下拐点

//======================== 拐点找到标志 ========================
uint8_t Right_Down_Point_finish_flag = 0;   // 找到右下拐点 1=找到
uint8_t Left_Down_Point_finish_flag  = 0;   // 找到左下拐点
uint8_t Right_Up_Point_finish_flag   = 0;   // 找到右上拐点
uint8_t Left_Up_Point_finish_flag    = 0;   // 找到左上拐点

//======================== 重复拐点标志 ========================
uint8_t Last_Right_Down_Point_finish_flag = 0;  // 本次与上次是同一个右下拐点
uint8_t Last_Left_Down_Point_finish_flag  = 0;  // 本次与上次是同一个左下拐点
// ====================
// 每列白色像素统计
// ====================
sint16 White_Column[LCDW]; 
const std::string TARGET_IP    = "192.168.43.213";
//192.168.43.146 huawei
//192.168.43.213 lianxiang
// UDP目标端口
const uint16_t    TARGET_PORT  = 8080;
// 摄像头参数
const uint16_t    CAM_WIDTH    = 320;     // 宽
const uint16_t    CAM_HEIGHT   = 240;     // 高
const uint16_t    CAM_FPS      = 120;     // 帧率
const uint8_t     JPEG_QUALITY = 100;
// 全局数组：存储压缩后的 60x80 灰度图
// 注意：LCDH 和 LCDW 必须定义为 60 和 80
uint8_t Image_Use[60][80]; 
static struct termios old_tio;
int my_abs(int a)
{
    return a>0 ? a : -a;
}
// 开启 非阻塞输入
void set_terminal_nonblock() {
    struct termios new_tio;
    tcgetattr(STDIN_FILENO, &old_tio);
    new_tio = old_tio;

    // 关闭 行缓冲 + 关闭回显
    new_tio.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);
}

// 恢复终端（非常重要！）
void reset_terminal() {
    tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
}

// 判断：有没有按键输入？
bool has_input() {
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);

    // 超时 0 → 不等待，直接返回
    struct timeval tv = {0, 0};
    select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv);

    return FD_ISSET(STDIN_FILENO, &fds);
}
// 输入：gray_frame - 此时是 160x120 的灰度图 (CV_8UC1)
// 输出：Image_Use - 压缩成 60x80 的灰度图
// 获取当前时间戳字符串
static std::string GetTimestamp()
{
    auto now = std::chrono::system_clock::now();
    auto ms  = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    time_t t = std::chrono::system_clock::to_time_t(now);
    tm* tm = localtime(&t);

    std::stringstream ss;
    ss << std::put_time(tm, "%Y-%m-%d %H:%M:%S") << "." << std::setfill('0') << std::setw(3) << ms.count();
    return ss.str();
}



void compressimage(const cv::Mat& gray_frame )
{
    int i, j;
    // 计算缩放比例，这里 Mh/Mw 是原图尺寸，Lh/Lw 是目标尺寸
    const float div_h = (float)gray_frame.rows / LCDH;
    const float div_w = (float)gray_frame.cols / LCDW;

    for (i = 0; i < LCDH; i++)
    {
        // 计算原图对应的行号（四舍五入）
        int row = (int)(i * div_h + 0.5f);
        
        // 获取原图第 row 行的首指针 (快速访问)
        const uint8_t* src_row_ptr = gray_frame.ptr<uint8_t>(row);
        
        for (j = 0; j < LCDW; j++)
        {
            // 计算原图对应的列号
            int col = (int)(j * div_w + 0.5f);
            
            // 直接赋值，将原图的像素拷贝到压缩数组
            // 这里不再需要 mt9v03x_image，直接用 Mat 的数据
            Image_Use[i][j] = src_row_ptr[col];
        }
    }
}
// ===================== 大津法参数 =====================
float bin_float[256];
int size = LCDH * LCDW;
float u = 0;
float w0 = 0;
float u0 = 0;
uint8_t Bin_Array[256];
int i;
float gray_hh = 0;
float var = 0;
float maxvar = 0;
float maxgray = 0;
float maxbin = 0;

struct size_point
{
    int x0;
    int y0;
    int x1;
    int y1;
};

struct size_point ostu_point[3] = {
    {0, 0, 79, 9},    // 上
    {0, 10, 79, 29},  // 中
    {0, 30, 79, 59}   // 下
};

uint8_t threshold1 = 0;
uint8_t threshold2 = 0;
uint8_t Thresholds[3] = {0};

// 外部定义的图像数组
uint8_t Image_Use1[LCDH][LCDW];


// ===================== 分块大津 =====================
void Ostu(void)
{
    int j, k;
    uint8_t (*p_image)[LCDW] = (uint8_t(*)[LCDW])Image_Use;

    threshold1 = otsuThreshold(Image_Use[0], LCDW, LCDH);

    for (k = 0; k < 3; k++)
    {
        maxvar = 0;
        w0 = 0;
        u = 0;
        gray_hh = 0;
        var = 0;
        Thresholds[k] = 0;

        for (i = 0; i < 256; i++)
            bin_float[i] = 0;

        for (i = ostu_point[k].y0; i <= ostu_point[k].y1; i++)
        {
            for (j = ostu_point[k].x0; j <= ostu_point[k].x1; j++)
            {
                ++bin_float[p_image[i][j]];
            }
        }

        size = (ostu_point[k].y1 - ostu_point[k].y0 + 1) *
               (ostu_point[k].x1 - ostu_point[k].x0 + 1);

        for (i = 0; i < 256; i++)
        {
            bin_float[i] /= size;
            u += i * bin_float[i];
        }

        for (i = 0; i < 256; i++)
        {
            w0 += bin_float[i];
            gray_hh += i * bin_float[i];
            u0 = gray_hh / w0;
            var = (u0 - u) * (u0 - u) * w0 / (1 - w0);

            if (var > maxvar)
            {
                maxgray = gray_hh;
                maxbin = w0;
                maxvar = var;
                Thresholds[k] = (uint8_t)i;
            }
        }

        // 三块自适应逻辑
        if (k == 0)
        {
            if (gray_hh > 15 && gray_hh <= 33)
            {
                if (maxbin < 0.9f)
                    Thresholds[k] = Thresholds[1] - 3;
            }
            else if (gray_hh > 41 && gray_hh <= 47)
            {
                if (maxbin < 0.64f || maxbin > 0.76f)
                    Thresholds[k] = Thresholds[1] - 3;
            }
            else if (gray_hh > 50 && gray_hh <= 60)
            {
                if (maxbin < 0.42f || maxbin > 0.58f)
                    Thresholds[k] = Thresholds[1] - 3;
            }

            if (abs(threshold1 - Thresholds[k]) >= 30)
                Thresholds[k] = threshold1;
            else
                Thresholds[k] = Thresholds[k] + 0.5f * (threshold1 - Thresholds[k]);
        }
        else if (k == 1)
        {
            if (gray_hh > 69 && gray_hh < 80)
            {
                if (maxbin > 0.15f)
                    Thresholds[k] = Thresholds[0] + 3;
            }

            if (abs(threshold1 - Thresholds[k]) >= 30)
                Thresholds[k] = threshold1;
            else
                Thresholds[k] = Thresholds[k] + 0.5f * (threshold1 - Thresholds[k]);
        }
        else if (k == 2)
        {
            if (maxbin < 0.85f && gray_hh < 28)
            {
                Thresholds[k] = Thresholds[1] - 3;
            }
            else if (gray_hh > 69 && gray_hh < 79)
            {
                if (maxbin < 0.5f || maxbin > 0.15f)
                    Thresholds[k] = Thresholds[1] - 3;
            }

            if (abs(threshold1 - Thresholds[k]) >= 30)
                Thresholds[k] = threshold1;
            else
                Thresholds[k] = Thresholds[k] + 0.5f * (threshold1 - Thresholds[k]);
        }

        // 生成二值查找表
        for (i = 0; i < Thresholds[k]; i++)
            Bin_Array[i] = 0;
        for (i = Thresholds[k]; i < 256; i++)
            Bin_Array[i] = 255;

        // 二值化写入 Image_Use1
        for (i = ostu_point[k].y0; i <= ostu_point[k].y1; i++)
        {
            for (j = ostu_point[k].x0; j <= ostu_point[k].x1; j++)
            {
                Image_Use1[i][j] = Bin_Array[p_image[i][j]];
            }
        }
    }
}

// ===================== 标准大津 =====================
uint8_t otsuThreshold(uint8_t *image, uint16_t col, uint16_t row)
{
#define GrayScale 256
    uint16_t Image_Width = col;
    uint16_t Image_Height = row;
    int X;
    uint16_t Y;
    uint8_t *data = image;
    int HistGram[GrayScale] = {0};

    uint32_t Amount = 0;
    uint32_t PixelBack = 0;
    uint32_t PixelIntegralBack = 0;
    uint32_t PixelIntegral = 0;
    int32_t PixelIntegralFore = 0;
    int32_t PixelFore = 0;

    double OmegaBack = 0, OmegaFore = 0;
    double MicroBack = 0, MicroFore = 0;
    double SigmaB = 0, Sigma = 0;

    uint8_t MinValue = 0, MaxValue = 0;
    uint8_t Threshold = 0;

    for (Y = 0; Y < Image_Height; Y++)
    {
        for (X = 0; X < Image_Width; X++)
        {
            HistGram[(int)data[Y * Image_Width + X]]++;
        }
    }

    for (MinValue = 0; MinValue < 256 && HistGram[MinValue] == 0; MinValue++);
    for (MaxValue = 255; MaxValue > MinValue && HistGram[MaxValue] == 0; MaxValue--);

    if (MaxValue == MinValue)
        return MaxValue;
    if (MinValue + 1 == MaxValue)
        return MinValue;

    for (Y = MinValue; Y <= MaxValue; Y++)
        Amount += HistGram[Y];

    PixelIntegral = 0;
    for (Y = MinValue; Y <= MaxValue; Y++)
        PixelIntegral += HistGram[Y] * Y;

    SigmaB = -1;
    for (Y = MinValue; Y < MaxValue; Y++)
    {
        PixelBack += HistGram[Y];
        PixelFore = Amount - PixelBack;
        OmegaBack = (double)PixelBack / Amount;
        OmegaFore = (double)PixelFore / Amount;
        PixelIntegralBack += HistGram[Y] * Y;
        PixelIntegralFore = PixelIntegral - PixelIntegralBack;
        MicroBack = (double)PixelIntegralBack / PixelBack;
        MicroFore = (double)PixelIntegralFore / PixelFore;
        Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);

        if (Sigma > SigmaB)
        {
            SigmaB = Sigma;
            Threshold = (uint8_t)Y;
        }
    }
    return Threshold;
}
/*---------------------------------------------------------------
 【函    数】Bin_Image_Filter
 【功    能】过滤噪点
 【参    数】无
 【返 回 值】无
 【注意事项】
 ----------------------------------------------------------------*/
void Bin_Image_Filter (uint8_t(*image)[LCDW])
{
    sint16 nr; //行
    sint16 nc; //列

    for (nr = 1; nr < LCDH - 1; nr++)
    {
        for (nc = 1; nc < LCDW - 1; nc = nc + 1)
        {
            if ((image[nr][nc] == 0)
                    && (image[nr - 1][nc] + image[nr + 1][nc] + image[nr][nc + 1] + image[nr][nc - 1] > 510))
            {
                image[nr][nc] = 255;
            }
            else if ((image[nr][nc] == 255)
                    && (image[nr - 1][nc] + image[nr + 1][nc] + image[nr][nc + 1] + image[nr][nc - 1] < 510))
            {
                image[nr][nc] = 0;
            }
        }
    }
}
//利用阈值把灰度图像转化为二值化
void get_erzhiimage(void)
{

  uint8_t Threshold =  otsuThreshold (Image_Use[0], LCDW, LCDH);      //这里是一个函数调用，通过该函数可以计算出一个效果很不错的二值化阈值。
  /* if(car_status.stop=='T'){
          Threshold=250;
      }*/
  uint8_t i, j = 0;
  for (i = 0; i < LCDH; i++)                                //遍历二维数组的每一行
  {
    for (j = 0; j < LCDW; j++)                              //遍历二维数组的每一列
    {
      if (Image_Use[i][j] > Threshold)                      //如果这个点的灰度值大于阈值Threshold
          Image_Use1[i][j] = 255;                                  //那么这个像素点就记为白点
      else                                                  //如果这个点的灰度值小于阈值Threshold
          Image_Use1[i][j] = 0;                                  //那么这个像素点就记为黑点
    }
  }
}
void lq_ncnn_photo_demo(cv::Mat& image,std::string& a)
{
    
    // 模型配置
    std::string model_param = "tiny_classifier_fp32.ncnn.param";
    std::string model_bin   = "tiny_classifier_fp32.ncnn.bin";
    int input_width    = 60;
    int input_height   = 60;
    
    // 类别标签（顺序必须与训练时一致）
    std::vector<std::string> labels = {"supplies", "vehicle", "weapon"};
    
    // 归一化参数（ImageNet标准）
    float mean_vals[3] = {123.675f, 116.28f, 103.53f};
    float norm_vals[3] = {0.01712475f, 0.017507f, 0.01742919f};
    // =================================================

    // 创建NCNN对象并配置
    LQ_NCNN ncnn;
    ncnn.SetModelPath(model_param, model_bin);
    ncnn.SetInputSize(input_width, input_height);
    ncnn.SetLabels(labels);
    ncnn.SetNormalize(mean_vals, norm_vals);

    // 初始化模型
    
   // printf("[%s] 正在加载模型...\n", GetTimestamp().c_str());
    if (!ncnn.Init()) {
        printf("[%s] 模型加载失败!\n", GetTimestamp().c_str());
        return ;
    }
   // printf("[%s] 模型加载成功!\n\n", GetTimestamp().c_str());

    //printf("[%s] 图片尺寸: %d x %d\n\n", GetTimestamp().c_str(), image.cols, image.rows);
    
    // 注意: OpenCV读取的是BGR格式，但在推理时会自动转换为RGB格式以匹配训练时的输入
    // 训练使用PIL读取的RGB格式，因此需要色彩空间转换

    // 推理
  //  printf("[%s] 开始推理...\n", GetTimestamp().c_str());
    auto start = std::chrono::high_resolution_clock::now();
    std::string result = ncnn.Infer(image);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

        a=result.c_str();
    // 输出结果
    /*
    printf("\n========================================\n");
    printf("推理结果: %s\n", result.c_str());
    printf("推理耗时: %ld ms\n", duration.count());
    printf("========================================\n");
    */
    return ;
}


void Longest_White_Column(void)//最长白列巡线{}
{

    //compressimage(gray_frame);      //图像压缩，把原始的80*170的图像压缩成60*80的,因为不需要那么多的信息，60*80能处理好的话已经足够
    // Ostu();
    //get_erzhiimage();
     Bin_Image_Filter(Image_Use1);
    int i, j;
    int start_column=10;//最长白列的搜索区间
    int end_column=LCDW-10;
    int left_border = 0, right_border = 0;//临时存储赛道位置
    float k;
    float b;
    Longest_White_Column_Left[0] = 0;//最长白列,[0]是最长白列的长度，[1】是第某列
    Longest_White_Column_Left[1] = 0;//最长白列,[0]是最长白列的长度，[1】是第某列
    Longest_White_Column_Right[0] = 0;//最长白列,[0]是最长白列的长度，[1】是第某列
    Longest_White_Column_Right[1] = 0;//最长白列,[0]是最长白列的长度，[1】是第某列
    Right_Lost_Time = 0;    //边界丢线数
    Left_Lost_Time  = 0;
    Boundry_Start_Left  = 0;//第一个非丢线点,常规边界起始点
    Boundry_Start_Right = 0;
    Both_Lost_Time = 0;//两边同时丢线数

    for (i = 0; i <=LCDH-1; i++)//数据清零
    {
        Right_Lost_Flag[i] = 0;
        Left_Lost_Flag[i] = 0;
        Left_Line[i] = 0;
        Right_Line[i] = 79;
        Mid_Line[i]=40;
    }

      for(i=0;i<=LCDW-1;i++)
      {
          White_Column[i] = 0;
      }

//环岛需要对最长白列范围进行限定
    //环岛3状态需要改变最长白列寻找范围



    //从左到右，从下往上，遍历全图记录范围内的每一列白点数量
    for (j =0; j<=79; j++)
    {
        for (i = LCDH-1; i >= 0; i--)
              {
                  if(Image_Use1[i][j] == 0)

                      break;

                  else
                      White_Column[j]=59-i;


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


    Longest_White_Column_Left1[0] =0;
    for(i=0;i<=79;i++)
    {
        if (Longest_White_Column_Left1[0] < White_Column[i])//找最长的那一列
        {
            Longest_White_Column_Left1[0] = White_Column[i];//【0】是白列长度
            Longest_White_Column_Left1[1] = i;              //【1】是下标，第j列
        }
    }
    //从右到左找右左边最长白列
    Longest_White_Column_Right1[0] = 0;//【0】是白列长度
    for(i=79;i>=0;i--)//从右往左，注意条件，找到左边最长白列位置就可以停了
    {
        if (Longest_White_Column_Right1[0] < White_Column[i])//找最长的那一列
        {
            Longest_White_Column_Right1[0] = White_Column[i];//【0】是白列长度
            Longest_White_Column_Right1[1] = i;              //【1】是下标，第j列
        }
    }


    Search_Stop_Line = Longest_White_Column_Left[0];//搜索截止行选取左或者右区别不大，他们两个理论上是一样的


    for (i = LCDH - 1; i >=LCDH-Search_Stop_Line-1; i--)//常规巡线
    {
        for (j = Longest_White_Column_Right[1]; j <= LCDW - 1 - 2; j++)
                {
                    if (Image_Use1[i][j] ==255 && Image_Use1[i][j + 1] == 0 && Image_Use1[i][j + 2] == 0)//白黑黑，找到右边界
                    {
                        right_border = j+1;
                        Right_Lost_Flag[i] = 1; //右丢线数组，丢线置1，不丢线置0
                        break;
                    }
                    else if(j>=LCDW-1-2)//没找到右边界，把屏幕最右赋值给右边界
                    {
                        right_border = 79;
                        Right_Lost_Flag[i] = 2; //右丢线数组，丢线置1，不丢线置0
                        break;
                    }


        }

        for (j = Longest_White_Column_Left[1]; j >= 0 + 2; j--)
        {
        if (Image_Use1[i][j] ==255 && Image_Use1[i][j - 1] == 0 && Image_Use1[i][j - 2] == 0)//黑黑白认为到达左边界
              {
                  left_border = j-1;
                  Left_Lost_Flag[i] = 1; //左丢线数组，丢线置1，不丢线置0
                  break;
              }
              else if(j<=0+2)
              {
                  left_border = 0;//找到头都没找到边，就把屏幕最左右当做边界
                  Left_Lost_Flag[i] = 2; //左丢线数组，丢线置1，不丢线置0
                  break;
              }
        }

        Left_Line  [i] = left_border;       //左边线线数组
        Right_Line [i] = right_border;      //右边线线数组
    }

    /*for (i = LCDH-Search_Stop_Line; i <=0; i--)
    {
        l_border  [i]=2;
        r_border  [i]=77;
        center_line[i]=39;
    }*/

    for (i = LCDH - 1; i >=0; i--)//赛道数据初步分析
       {
           if (Left_Lost_Flag[i]  == 2)//单边丢线数
               Left_Lost_Time++;
           if (Right_Lost_Flag[i] == 2)
               Right_Lost_Time++;
           if (Left_Lost_Flag[i] == 2 && Right_Lost_Flag[i] == 2)//双边丢线数
               Both_Lost_Time++;
           if (Boundry_Start_Left ==  0 && Left_Lost_Flag[i]  != 2)//记录第一个非丢线点，边界起始点
               Boundry_Start_Left = i;
           if (Boundry_Start_Right == 0 && Right_Lost_Flag[i] != 2)
               Boundry_Start_Right = i;
           /*if(Right_Lost_Flag[i]==0)
           {
               Right_Line[i]=Right_Line[LCDH-Search_Stop_Line];
           }
           if(Left_Lost_Flag[i]==0)
                      {
                          Left_Line[i]=Left_Line[LCDH-Search_Stop_Line];
                      }*/
}


    for (i = LCDH-Search_Stop_Line; i <= LCDH-1; i++)//赛道数据初步分析
           {
     Road_Wide[i]=Right_Line[i]-Left_Line[i];

     /*if(Left_Lost_Flag[i]==1&&Right_Lost_Flag[i]==1)
          Mid_Line[i] = (Left_Line[i] + Right_Line[i]) >> 1;
     if(Left_Lost_Flag[i]==2&&Right_Lost_Flag[i]==1)
             Mid_Line[i] = Right_Line[i] -zhidao[i]/2;
     if(Left_Lost_Flag[i]==1&&Right_Lost_Flag[i]==2)
             Mid_Line[i] = Left_Line[i] + zhidao[i]/2;
     if(Left_Lost_Flag[i]==2&&Right_Lost_Flag[i]==2)
             Mid_Line[i] = (Left_Line[i] + Right_Line[i]) >> 1;*/

    /* if((Left_Line[25] + Right_Line[25]) >> 1>40)
        {
         Mid_Line[i] = Left_Line[i] + zhidao[i]/2;
        }
     if((Left_Line[25] + Right_Line[25]) >> 1<=40)
            {
         Mid_Line[i] = Right_Line[i] -zhidao[i]/2;
            }
     if(Left_Lost_Flag[i]==1&&Right_Lost_Flag[i]==1)*/
                  Mid_Line[i] = (Left_Line[i] + Right_Line[i]) >> 1;

           }
}
/********************************************************************************
 * 
 * @brief   UDP 图像传输测试.
 * @param   none.
 * @return  none.
 * @note    测试内容为 UDP 图像传输，使用OpenCV 读取摄像头图像，并使用 UDP 发送图像数据.
 * @note    使用时需搭配对应上位机 LoongHost.exe.
 ********************************************************************************/
void cut(void){
          pwm1.atim_pwm_disable();
pwm2.atim_pwm_disable();
}
/*-------------------------------------------------------------------------------------------------------------------
  @brief     右赛道连续性检测
  @param     起始点，终止点
  @return    连续返回0，不连续返回断线出行数
  Sample     continuity_change_flag=Continuity_Change_Right(int start,int end)
  @note      连续性的阈值设置为5，可更改
-------------------------------------------------------------------------------------------------------------------*/
int Continuity_Change_Right(int start,int end)
{
    int i;
    int t;
    int continuity_change_flag=0;
    if(Right_Lost_Time>=0.9*LCDH)//大部分都丢线，没必要判断了
       return 1;
    if(start>=LCDH-5)//数组越界保护
        start=LCDH-5;
    if(end<=5)
       end=5;
    if(start<end)//都是从下往上计算的，反了就互换一下
    {
       t=start;
       start=end;
       end=t;
    }

    for(i=start;i>=end;i--)
    {
        if(abs(Right_Line[i]-Right_Line[i-1])>=5&&Right_Lost_Flag[i-1]!=0)//连续性阈值是5，可更改
       {
            continuity_change_flag=i;
            break;
       }
    }
    return continuity_change_flag;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     左赛道连续性检测
  @param     起始点，终止点
  @return    连续返回0，不连续返回断线出行数
  Sample     continuity_change_flag=Continuity_Change_Right(int start,int end)
  @note      连续性的阈值设置为5，可更改
-------------------------------------------------------------------------------------------------------------------*/
int Continuity_Change_Left(int start,int end)
{
    int i;
    int t;
    int continuity_change_flag=0;
    if(Left_Lost_Time>=0.9*LCDH)//大部分都丢线，没必要判断了
       return 1;
    if(start>=LCDH-5)//数组越界保护
        start=LCDH-5;
    if(end<=5)
       end=5;
    if(start<end)//都是从下往上计算的，反了就互换一下
    {
       t=start;
       start=end;
       end=t;
    }

    for(i=start;i>=end;i--)
    {
        if(abs(Left_Line[i]-Left_Line[i-1])>=5&&Left_Lost_Flag[i-1]!=0)//连续性阈值是5，可更改
       {
            continuity_change_flag=i;
            break;
       }
    }
    return continuity_change_flag;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      拐点寻找
//  @param      findlcount   拐点距离丢线的行数，用于判断大小圆环，和区分P字和圆环
//  @param
//  @param
//  @param
//  @return     void
//  @note       采用5邻域的原理寻找拐点，下拐点从图像低端往上扫，上拐点从图像上方向下扫，左右扫线会在斑马线出现问题，
//-------------------------------------------------------------------------------------------------------------------

void Identify(void)
{
    uint8_t  findr_x = 0;    //右点
    uint8_t  findr_y = 0;
    uint8_t  examr_x = 0;
    uint8_t  examr_y = 0;
    uint8_t  findl_x = 0;    //左点
    uint8_t  findl_y = 0;
    uint8_t  examl_x = 0;
    uint8_t  examl_y = 0;
    uint8_t  star = 0;
    uint8_t  end = 0;
    uint8_t  examcount = 0;
    //uint8 count;
    //uint8 examerror;
//    uint8 dircount;
    int directionrd[5][2] =  {{-1,1}, {-1,0}, {-1,-1}, {0,-1}, {1,-1}};  //顺时针下方向数组先x再y
    int directionld[5][2] =  {{1,1}, {1,0}, {1,-1}, {0,-1}, {-1,-1}};  //逆时针下方向数组
    int directionru[5][2] =  {{1,1}, {0,1}, {-1,1}, {-1,0}, {-1,-1}};  //逆时针上方向数组
    int directionlu[5][2] =  {{-1,1}, {0,1}, {1,1}, {1,0}, {1,-1}};  //逆时针上方向数组

    //每次采集后都对拐点标志位清零
    if(Right_Up_Point_finish_flag == 1)
        Right_Up_Point_finish_flag = 0;
    if(Left_Up_Point_finish_flag == 1)
        Left_Up_Point_finish_flag = 0;
    if(Right_Down_Point_finish_flag == 1)
        Right_Down_Point_finish_flag = 0;
    if(Left_Down_Point_finish_flag == 1)
        Left_Down_Point_finish_flag = 0;
    if(Last_Right_Down_Point_finish_flag == 1)
        Last_Right_Down_Point_finish_flag = 0;
    if(Last_Left_Down_Point_finish_flag == 1)
        Last_Left_Down_Point_finish_flag = 0;

    for(uint8_t  y = 55 ; y >= 10 ; y--)
    {
        if(Right_Down_Point_finish_flag == 0)
        {
            if(y > 0&& /*&&Right_Line[y-1]==79 &&Right_Line[y-2]==79*/Right_Line[y-1]<=79 &&Right_Line[y-1]>=77 &&Right_Line[y-2]<=79 &&Right_Line[y-2]>=77&&Image_Use1[y][Right_Line[y]-6] == 255
               && y > 0 && Image_Use1[y-2][Right_Line[y]] == 255 && Image_Use1[y-5][Right_Line[y]] == 255)    //右下拐点
            {
                star=y;
                // 修复：uint8死循环 + 越界，增加边界保护
                for(int y = star; y <= star+20 && y < LCDH-2; y++)
                {
                    if(Right_Line[y]<77 && my_abs(Right_Line[y+1]-Right_Line[y])<3
                    && my_abs(Right_Line[y+2]-Right_Line[y])<4)
                    {
                        findr_x=Right_Line[y];
                        findr_y=y;

                        examcount = 0; // 每次清零
                        for(uint8_t  dircount = 0;dircount<5;dircount++)
                        {
                            examr_x=findr_x+directionrd[dircount][0];
                            examr_y=findr_y+directionrd[dircount][1];
                            // 安全检查：防止越界
                            if(examr_x >=0 && examr_x < LCDW && examr_y >=0 && examr_y < LCDH)
                            {
                                if(Image_Use1[examr_y][examr_x]==255)
                                {
                                    examcount++;
                                }
                            }
                        }
                        if(examcount >= 4)
                        {
                            examcount=0;
                            Right_Down_Point[0]=findr_x;
                            Right_Down_Point[1]=findr_y;

                            if(abs( Last_Right_Down_Point[1]-Right_Down_Point[1])<=5)
                            {
                                Last_Right_Down_Point_finish_flag = 1;
                            }
                            else
                                Last_Right_Down_Point_finish_flag = 0;

                            Last_Right_Down_Point[0]=Right_Down_Point[0];
                            Last_Right_Down_Point[1]=Right_Down_Point[1];

                            Right_Down_Point_finish_flag = 1;
                            break;
                        }
                        else
                        {
                            Right_Down_Point_finish_flag = 0;
                            examcount=0;
                        }
                    }
                    if(y>55)
                    {
                        Right_Down_Point_finish_flag=0;
                    }
                }
            }
        }

       if(Left_Down_Point_finish_flag == 0)
        {
            if(y > 0 && Left_Line[y-1]>=0 && Left_Line[y-1]<=2 && Left_Line[y-2]>=0 &&Left_Line[y-2]<=2&& Image_Use1[y][Left_Line[y]+6] == 255
               && y > 0 && Image_Use1[y-2][Left_Line[y]] == 255 && Image_Use1[y-5][Left_Line[y]] == 255)     //左下拐点
            {
                star=y;
                // 修复：uint8死循环 + 越界，增加边界保护
                for(int y = star; y <= star+20 && y < LCDH-2; y++)
                {
                    if(Left_Line[y]>2 && my_abs(Left_Line[y+1]-Left_Line[y])<3
                       && my_abs(Left_Line[y+2]-Left_Line[y])<4)
                    {
                        findl_x=Left_Line[y];
                        findl_y=y;

                        examcount = 0; // 每次清零
                        for(uint8_t  dircount = 0;dircount<5;dircount++)
                        {
                            examl_x=findl_x+directionld[dircount][0];
                            examl_y=findl_y+directionld[dircount][1];
                            // 安全检查：防止越界
                            if(examl_x >=0 && examl_x < LCDW && examl_y >=0 && examl_y < LCDH)
                            {
                                if(Image_Use1[examl_y][examl_x]==255)
                                {
                                    examcount++;
                                }
                            }
                        }
                        if(examcount>=4 )
                        {
                            examcount=0;
                            Left_Down_Point[0]=findl_x;
                            Left_Down_Point[1]=findl_y;
                            if(abs( Last_Left_Down_Point[1]-Left_Down_Point[1])<=5)
                            {
                                Last_Left_Down_Point_finish_flag = 1;
                            }
                            else
                                Last_Left_Down_Point_finish_flag = 0;

                            Last_Left_Down_Point[0]=Left_Down_Point[0];
                            Last_Left_Down_Point[1]=Left_Down_Point[1];

                            Left_Down_Point_finish_flag = 1;
                            break;
                        }
                        else
                        {
                            Left_Down_Point_finish_flag = 0;
                            examcount=0;
                        }
                    }
                    if(y>55)
                    {
                        Left_Down_Point_finish_flag=0;
                    }
                }
            }
        }
    }

    if(Left_Down_Point_finish_flag==1 && Right_Down_Point_finish_flag==1)
        end=Right_Down_Point[1];
    else if(Left_Down_Point_finish_flag==1)
        end=Left_Down_Point[1];
    else if(Right_Down_Point_finish_flag==1)
        end=Right_Down_Point[1];
    else
        end=55;

    for(uint8_t  y=10;y<=55;y++)
    {
        if(Right_Up_Point_finish_flag == 0)
        {
            if(y > 0 && Right_Line[y+1]==79 &&Right_Line[y+2]==79 && Right_Line[y+3]==79
                    &&Right_Lost_Flag[y+1]==2&&Right_Lost_Flag[y+2]==2&&Right_Lost_Flag[y+3]==2)   //右上拐点
            {
               star=y;
               // 修复：uint8死循环 + 越界，增加边界保护
               for(int y = star; y >= star-20 && y >= 2; y--)
               {
                   if(Right_Line[y]<77 && my_abs(Right_Line[y-1]-Right_Line[y])<4
                   && my_abs(Right_Line[y-2]-Right_Line[y])<4 && Image_Use1[y][Right_Line[y]-6] == 255
                   && Image_Use1[y-1][Right_Line[y-1]-5] == 255 && Image_Use1[y-2][Right_Line[y]-5] == 255
                   && Image_Use1[y-3][Right_Line[y]-5] == 255 && Right_Line[y] > Mid_Line[y] && Image_Use1[y+3][Right_Line[y]] == 255
                   && Image_Use1[y+5][Right_Line[y]] == 255 && Image_Use1[y+7][Right_Line[y]] == 255)
                   {
                       findr_x=Right_Line[y];
                       findr_y=y;

                       examcount = 0; // 每次清零
                       for(uint8_t  dircount = 0;dircount<5;dircount++)
                       {
                           examr_x=findr_x+directionru[dircount][0];
                           examr_y=findr_y+directionru[dircount][1];
                           // 安全检查：防止越界
                           if(examr_x >=0 && examr_x < LCDW && examr_y >=0 && examr_y < LCDH)
                           {
                               if(Image_Use1[examr_y][examr_x]==255)
                               {
                                   examcount++;
                               }
                           }
                       }
                       if(examcount>=4 && findr_y >0)
                       {
                           examcount=0;
                           Right_Up_Point[0]=findr_x;
                           Right_Up_Point[1]=findr_y;
                           Right_Up_Point_finish_flag = 1;
                           break;
                       }
                       else
                       {
                           Right_Up_Point_finish_flag = 0;
                           examcount=0;
                       }
                   }
                   if(y<5)
                   {
                       Right_Up_Point_finish_flag=0;
                       break;
                   }
               }
            }
        }

        if(Left_Up_Point_finish_flag == 0)
        {
            if(y > 0 && Left_Line[y+1]==0 && Left_Line[y+2]==0 && Left_Line[y+3]==0
                    &&Left_Lost_Flag[y+1]==2&&Left_Lost_Flag[y+2]==2&&Left_Lost_Flag[y+3]==2)     //左上拐点
            {
                star=y;
                // 修复：uint8死循环 + 越界，增加边界保护
                for(int y = star; y >= star-20 && y >= 2; y--)
                {
                    if(Left_Line[y]>2 && my_abs(Left_Line[y-1]-Left_Line[y])<4
                   && my_abs(Left_Line[y-2]-Left_Line[y])<4 && Image_Use1[y][Left_Line[y]+6] == 255
                   && Image_Use1[y-1][Left_Line[y-1]+5] == 255 && Image_Use1[y-2][Left_Line[y]+5] == 255
                   && Image_Use1[y-3][Left_Line[y]+5] == 255 && Left_Line[y] < Mid_Line[y] && Image_Use1[y+3][Left_Line[y]] == 255
                   && Image_Use1[y+5][Left_Line[y]] == 255 && Image_Use1[y+7][Left_Line[y]] == 255)
                    {
                        findl_x=Left_Line[y];
                        findl_y=y;

                        examcount = 0; // 每次清零
                        for(uint8_t dircount = 0;dircount<5;dircount++)
                        {
                            examl_x=findl_x+directionlu[dircount][0];
                            examl_y=findl_y+directionlu[dircount][1];
                            // 安全检查：防止越界
                            if(examl_x >=0 && examl_x < LCDW && examl_y >=0 && examl_y < LCDH)
                            {
                                if(Image_Use1[examl_y][examl_x]==255)
                                {
                                    examcount++;
                                }
                            }
                        }
                        if(examcount>=4 && findl_y > 0)
                        {
                            examcount=0;
                            Left_Up_Point[0]=findl_x;
                            Left_Up_Point[1]=findl_y;
                            Left_Up_Point_finish_flag = 1;
                            break;
                        }
                        else
                        {
                            Left_Up_Point_finish_flag = 0;
                            examcount=0;
                        }
                    }
                    if(y<5)
                    {
                        Left_Up_Point_finish_flag=0;
                        break;
                    }
                }
            }
        }
    }
}
void Judge_Track_Element(void)
{
    int conti_left, conti_right;

    // 1. 先调用你已有的连续性检测（从近到远扫一段）
    conti_left  = Continuity_Change_Left(LCDH-10, 15);
    conti_right = Continuity_Change_Right(LCDH-10, 15);

    // 2. 先清零所有标志
    CrossFlag = 0;
    RoundLeftFlag = 0;
    RoundRightFlag = 0;
    // ==============================================
    // 【1】十字路口判断（更稳定）
    // ==============================================
    if( Both_Lost_Time > 25                // 双边大量丢线
     && Road_Wide[LCDH-10] > 40            // 赛道变宽
     && Road_Wide[LCDH-20] > 40
     && conti_left !=0 && conti_right !=0  // 左右都不连续
     && !Left_Down_Point_finish_flag       // 没有单侧拐点
     && !Right_Down_Point_finish_flag )
    {
        CrossFlag = 1;
    }

    // ==============================================
    // 【2】左环岛判断（更精准）
    // ==============================================
    if( Left_Lost_Time > 8                // 左边大量丢线
     && Right_Lost_Time < 5               // 右边正常
     //&& Road_Wide[25] > 38                 // 赛道明显变宽
     && conti_left != 0                    // 左边不连续
     && (Left_Down_Point_finish_flag || Left_Up_Point_finish_flag) )
    {
        RoundLeftFlag = 1;
    }

    // ==============================================
    // 【3】右环岛判断（更精准）
    // ==============================================
    if( Right_Lost_Time > 8              // 右边大量丢线
     && Left_Lost_Time < 5               // 左边正常
     //&& Road_Wide[25] > 38                 // 赛道明显变宽
     && conti_right != 0                   // 右边不连续
     && (Right_Down_Point_finish_flag || Right_Up_Point_finish_flag) )
    {
       RoundRightFlag= 1;
    }
}
void send_udp(){
        char encoder_str[64];
        // 如果只有一个编码器
        // snprintf(encoder_str, sizeof(encoder_str), "ch1:%.2f", ch1);
        // 如果有两个编码器
            int conti_left, conti_right;
 Identify();
    // 1. 先调用你已有的连续性检测（从近到远扫一段）
    conti_left  = Continuity_Change_Left(LCDH-10, 15);
    conti_right = Continuity_Change_Right(LCDH-10, 15);

    // 2. 先清零所有标志
    CrossFlag = 0;
    
    RoundLeftFlag = 0;
    RoundRightFlag = 0;
     //   snprintf(encoder_str, sizeof(encoder_str), "RL:%d,LL:%d,RW:%d,LD:%d,LU:%d,CL:%d,CR:%d,RD:%d,RU:%d", Right_Lost_Time,Left_Lost_Time,Road_Wide[25],Left_Down_Point_finish_flag,Left_Up_Point_finish_flag
    //,conti_left ,conti_right,Right_Down_Point_finish_flag,Right_Up_Point_finish_flag);
    Judge_Track_Element();
   snprintf(encoder_str, sizeof(encoder_str), "Rh:%d,Lh:%d",  RoundRightFlag ,RoundLeftFlag);
        // 发送编码器数据
        udp_client.udp_send_string(encoder_str);

}
void img_test(void)
{
      //pwm1.atim_pwm_disable();
//pwm2.atim_pwm_disable();
    printf("=========================================\r\n");
    printf("  UDP Camera + Encoder Stream\r\n");
    printf("=========================================\r\n");
    printf("Target IP:   %s\r\n", TARGET_IP.c_str());
    printf("Target Port: %d\r\n", TARGET_PORT);
    printf("Resolution:  %dx%d\r\n", CAM_WIDTH, CAM_HEIGHT);
    printf("FPS:         %d\r\n", CAM_FPS);
    printf("=========================================\r\n");

    // 初始化UDP客户端
    udp_client.udp_client_init(TARGET_IP, TARGET_PORT);
    printf("UDP client initialized\r\n");

    // 初始化摄像头
    lq_camera cam(CAM_WIDTH, CAM_HEIGHT, CAM_FPS);
    if (!cam.is_opened()) {
        printf("ERROR: Failed to open camera!\r\n");
        return;
    }
    printf("Camera opened: %dx%d @ %dfps\r\n", cam.get_width(), cam.get_height(), cam.get_fps());

    // 发送帧计数
    uint32_t frame_count = 0;
    uint32_t encoder_count = 0;
    // 记录开始时间
    auto start_time = std::chrono::high_resolution_clock::now();

    printf("Start streaming... Press Ctrl+C to stop\r\n");


//轮胎pd调速测试：输入你想的转速

// 轮胎PD调速：输入目标转速（纯输入板块）




    while (true) {
        if (has_input()) {
            char c = getchar();
            if (c == 'q') {
                std::cout<<"caonima"<<std::endl;
                cut();
                 while (getchar() != EOF); 
                break;
            } 
        }
        // ===================== 获取并发送图像 =====================
        // 获取原始图像
       // cv::Mat gray_frame = cam.get_gray_frame();
       cv::Mat frame = cam.get_raw_frame();
               cv::flip(frame, frame, -1); //颠倒上下左右
       // 原图 img
int cut_border = 50; // 裁掉边缘的宽度


// 裁剪规则：左、上、右 裁剪，底部不裁剪
//图像识别部分
/*
cv::Mat crop_img = frame(
    cv::Rect(
        cut_border+50,    // 左边裁掉 cut_border 像素
        cut_border,    // 上边裁掉 cut_border 像素
        frame.cols - 2 * cut_border,  // 宽度 = 总宽 - 左 - 右
        frame.rows - cut_border       // 高度 = 总高 - 上边（底部不动）
    )
);
*/
cv::Mat  crop_img= frame(cv::Rect(45, 0, frame.cols-45, frame.rows));
/*
// 裁剪后的图
        cv::Mat image;
           cv::resize(crop_img, image, cv::Size(60, 60), 0, 0, cv::INTER_NEAREST);
           std::string a;
            lq_ncnn_photo_demo(image,a);
            int rows = crop_img.rows;
        int cols = image.cols;
        int x1 = (cols - 20) / 2;
        int y1 = (rows - 20) / 2;
        std::cout<<2<<a<<1<<std::endl;
        if(a=="vehicle"){
        cv::rectangle(crop_img, cv::Point(x1, y1), cv::Point(x1 + 20, y1 + 20), cv::Scalar(0, 255, 0), 2);//green
        }
        else if(a=="weapon"){
cv::rectangle(crop_img, cv::Point(x1, y1), cv::Point(x1 + 20, y1 + 20), cv::Scalar(0, 0, 255), 2);//red
        }
        else{
cv::rectangle(crop_img, cv::Point(x1, y1), cv::Point(x1 + 20, y1 + 20), cv::Scalar(255, 0, 0), 2);//blue
        }
*/
      //  cv::Mat frame = cam.get_binary_frame();
        if (frame.empty()) {
            printf("ERROR: Failed to read frame\r\n");
            continue;
        }
        cv::Mat gray_frame;

       cv::cvtColor(crop_img, gray_frame, cv::COLOR_BGR2GRAY);
        compressimage(gray_frame);  // 压缩
        Ostu();      
        Longest_White_Column();
       // send_udp();
       // std::cout<<Mid_Line[40]<<std::endl; 

     // char encoder_str[64];
     //   snprintf(encoder_str, sizeof(encoder_str), "mid:%d",Mid_Line[40]);
        
        // 发送编码器数据
     //   udp_client.udp_send_string(encoder_str);
     /*------------below begin pid test-------------------*/

     mid=Mid_Line[40];
   //  Identify();
   // Judge_Track_Element();

     // std::cout<<Right_Lost_Time<<"  "<<Left_Lost_Time<<"  "<<"  "<<Road_Wide[25]<<"  "<<Left_Down_Point_finish_flag <<" "<<Left_Up_Point_finish_flag<<std::endl;
     //if(RoundLeftFlag){
     //   std::cout<<"leftround"<<std::endl;
  //  }
  //  if(RoundRightFlag){
   //     std::cout<<"rightround"<<std::endl;
  //  }
PID_control_test(Mid_Line[40]);


      //  printf("【全行列中线】\n");
//for(int i=0; i<LCDH; i++){
 //   printf("行%2d: %d\n", 40, Mid_Line[i]);
//}
           cv::Mat binary_mat(LCDH, LCDW, CV_8UC1, Image_Use1);

cv::Mat color_mat;
cv::cvtColor(cv::Mat(LCDH, LCDW, CV_8UC1, Image_Use1), color_mat, cv::COLOR_GRAY2BGR);

// 2. 在每一行画红色中线 (BGR格式：0,0,255 = 红色)
for (int i = 0; i < LCDH; i++) {
    int mx = Mid_Line[i];
    if (mx >= 1 && mx < LCDW - 1) {
        // 画三个连续红点，更明显
        if(i!=40){
        color_mat.at<cv::Vec3b>(i, mx-1) = cv::Vec3b(0, 0, 255);
        color_mat.at<cv::Vec3b>(i, mx  ) = cv::Vec3b(0, 0, 255);
        color_mat.at<cv::Vec3b>(i, mx+1) = cv::Vec3b(0, 0, 255);
        }
        else{
               color_mat.at<cv::Vec3b>(i, mx-1) = cv::Vec3b(255, 0, 0);
        color_mat.at<cv::Vec3b>(i, mx  ) = cv::Vec3b(255, 0, 0);
        color_mat.at<cv::Vec3b>(i, mx+1) = cv::Vec3b(255, 0, 0);
        }
    }
}
    // 放大一下，不然60x80太小了A，看不见
    //std::cout<<color_mat.cols<<std::endl;
    cv::Mat big_mat;
   // cv::resize(binary_mat, big_mat, cv::Size(320, 240), 0, 0, cv::INTER_NEAREST);
    cv::resize(color_mat, big_mat, cv::Size(320, 240), 0, 0, cv::INTER_NEAREST);
        // 发送JPEG压缩图像
        ssize_t sent = udp_client.udp_send_image(big_mat, JPEG_QUALITY);
        if (sent < 0) {
            printf("ERROR: Failed to send image\r\n");
        }

        frame_count++;

        // ===================== 每秒打印状态 =====================
        auto now = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();
        if (elapsed >= 1) {
            float fps = (float)frame_count / (float)elapsed;
           // printf("FPS: %.2f\r\n", fps);
            frame_count = 0;
            encoder_count = 0;
            start_time = now;
        }
    }
    std::cout<<"caonissma"<<std::endl;
     reset_terminal(); // 必须恢复终端！
     std::cout<<"caonimssa"<<std::endl;
}
float img_return(void)
{

    // 初始化UDP客户端
    lq_udp_client udp_client;
    udp_client.udp_client_init(TARGET_IP, TARGET_PORT);
    printf("UDP client initialized\r\n");

    // 初始化摄像头
    lq_camera cam(CAM_WIDTH, CAM_HEIGHT, CAM_FPS);
    if (!cam.is_opened()) {
        printf("ERROR: Failed to open camera!\r\n");
        return -1;
    }

    // 发送帧计数
    uint32_t frame_count = 0;
    uint32_t encoder_count = 0;
    // 记录开始时间
    auto start_time = std::chrono::high_resolution_clock::now();


        // ===================== 获取并发送图像 =====================
        // 获取原始图像
        cv::Mat gray_frame = cam.get_gray_frame();
        cv::flip(gray_frame, gray_frame, -1); //颠倒上下左右
      //  cv::Mat frame = cam.get_binary_frame();
        if (gray_frame.empty()) {
            printf("ERROR: Failed to read frame\r\n");
           return -1;
        }
        //cv::Mat gray_frame;

       // cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);
        compressimage(gray_frame);  // 压缩
        Ostu();      
        Longest_White_Column();
      cv::Mat binary_mat(LCDH, LCDW, CV_8UC1, Image_Use1);

    // 放大一下，不然60x80太小了A，看不见
    cv::Mat big_mat;
    cv::resize(binary_mat, big_mat, cv::Size(320, 240), 0, 0, cv::INTER_NEAREST);
        // 发送JPEG压缩图像
        ssize_t sent = udp_client.udp_send_image(big_mat, JPEG_QUALITY);
        if (sent < 0) {
            printf("ERROR: Failed to send image\r\n");
        }
    printf("%d\n",Mid_Line[20]);
    return Mid_Line[20];
    
}
