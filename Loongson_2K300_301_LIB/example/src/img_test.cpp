#include "lq_all_demo.hpp"
typedef signed short       sint16;  // 16位 有符号  <-- 你要的
#define LCDH    60   // 图像高度（行）
#define LCDW    80   // 图像宽度（列）
// =====================================================
// 配置参数 - 根据需要修改
// =====================================================
// 目标IP地址（UDP接收端）
uint8_t otsuThreshold(uint8_t *image, uint16_t col, uint16_t row);
ls_atim_pwm pwm1(ATIM_PWM0_PIN81, 100, 0);
ls_atim_pwm pwm2(ATIM_PWM1_PIN82, 100, 0); 
typedef signed short sint16;


//encoder
ls_encoder_pwm enc1(ENC_PWM0_PIN64, PIN_72);
ls_encoder_pwm enc2(ENC_PWM1_PIN65, PIN_73);

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
// ====================
// 每列白色像素统计
// ====================
sint16 White_Column[LCDW]; 
const std::string TARGET_IP    = "192.168.43.146";
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


void Longest_White_Column(void)//最长白列巡线
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
    lq_udp_client udp_client;
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
int expected_speed_of_motor1_rps = 0;
int expected_speed_of_motor2_rps = 0;
const int MAX_SPEED = 300;
const int MIN_SPEED = 0;
    pwm1.atim_pwm_set_duty(0);
    pwm2.atim_pwm_set_duty(0);
printf("请输入电机1、电机2目标转速(rps，空格分隔)：");
// 读取两个int型数据
int res = scanf("%d %d", &expected_speed_of_motor1_rps, &expected_speed_of_motor2_rps);

// 合法性判断
if (res == 2) 
{
    if (expected_speed_of_motor1_rps >= MIN_SPEED && expected_speed_of_motor1_rps <= MAX_SPEED &&
        expected_speed_of_motor2_rps >= MIN_SPEED && expected_speed_of_motor2_rps <= MAX_SPEED)
    {
        printf("输入正确！电机1：%d，电机2：%d\n", expected_speed_of_motor1_rps, expected_speed_of_motor2_rps);
        // 这里可以直接调用你的闭环控制函数
    }
    else
    {
        printf("输入错误：转速超出范围！\n");
    }
}
else
{
    printf("输入错误：请输入两个整数！\n");
    fflush(stdin); // 清空输入缓存
}




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

cv::Mat crop_img = frame(
    cv::Rect(
        cut_border+50,    // 左边裁掉 cut_border 像素
        cut_border,    // 上边裁掉 cut_border 像素
        frame.cols - 2 * cut_border,  // 宽度 = 总宽 - 左 - 右
        frame.rows - cut_border       // 高度 = 总高 - 上边（底部不动）
    )
);
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
       // std::cout<<Mid_Line[40]- 40<<std::endl; 
     //  PID_control_test(pwm1,pwm2,Mid_Line[40]- 40);

//以下开始测试轮胎闭环控制
float speed_of_motor1=enc1.encoder_get_count();
float speed_of_motor2=enc2.encoder_get_count();
close_circle_control(
    pwm1,pwm2,
   speed_of_motor1,speed_of_motor2,
    expected_speed_of_motor1_rps,
    expected_speed_of_motor2_rps
    );
    



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
    std::cout<<color_mat.cols<<std::endl;
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
