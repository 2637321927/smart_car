#include "lq_all_demo.hpp"
#define LCDH    60   // 图像高度（行）
#define LCDW    80   // 图像宽度（列）
// =====================================================
// 配置参数 - 根据需要修改
// =====================================================
// 目标IP地址（UDP接收端）
uint8_t otsuThreshold(uint8_t *image, uint16_t col, uint16_t row);
const std::string TARGET_IP    = "192.168.43.213";
// UDP目标端口
const uint16_t    TARGET_PORT  = 8080;
// 摄像头参数
const uint16_t    CAM_WIDTH    = 160;     // 宽
const uint16_t    CAM_HEIGHT   = 120;     // 高
const uint16_t    CAM_FPS      = 120;     // 帧率
const uint8_t     JPEG_QUALITY = 30;
// 全局数组：存储压缩后的 60x80 灰度图
// 注意：LCDH 和 LCDW 必须定义为 60 和 80
uint8_t Image_Use[60][80]; 

// 输入：gray_frame - 此时是 160x120 的灰度图 (CV_8UC1)
// 输出：Image_Use - 压缩成 60x80 的灰度图
void compressimage(const cv::Mat& gray_frame)
{
    int i, j;
    // 计算缩放比例，这里 Mh/Mw 是原图尺寸，Lh/Lw 是目标尺寸
    const float div_h = (float)CAM_HEIGHT / LCDH;
    const float div_w = (float)CAM_WIDTH / LCDW;

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
/********************************************************************************
 * @brief   UDP 图像传输测试.
 * @param   none.
 * @return  none.
 * @note    测试内容为 UDP 图像传输，使用OpenCV 读取摄像头图像，并使用 UDP 发送图像数据.
 * @note    使用时需搭配对应上位机 LoongHost.exe.
 ********************************************************************************/
void img_test(void)
{
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

    while (true) {
        // ===================== 获取并发送图像 =====================
        // 获取原始图像
        cv::Mat gray_frame = cam.get_gray_frame();
        cv::flip(gray_frame, gray_frame, -1); //颠倒上下左右
      //  cv::Mat frame = cam.get_binary_frame();
        if (gray_frame.empty()) {
            printf("ERROR: Failed to read frame\r\n");
            continue;
        }
        //cv::Mat gray_frame;

       // cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);
        compressimage(gray_frame);  // 压缩
        Ostu();      
           cv::Mat binary_mat(LCDH, LCDW, CV_8UC1, Image_Use1);

    // 放大一下，不然60x80太小了，看不见
    cv::Mat big_mat;
    cv::resize(binary_mat, big_mat, cv::Size(320, 240), 0, 0, cv::INTER_NEAREST);
        // 发送JPEG压缩图像
        ssize_t sent = udp_client.udp_send_image(gray_frame, JPEG_QUALITY);
        if (sent < 0) {
            printf("ERROR: Failed to send image\r\n");
        }

        frame_count++;

        // ===================== 每秒打印状态 =====================
        auto now = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();
        if (elapsed >= 1) {
            float fps = (float)frame_count / (float)elapsed;
            printf("FPS: %.2f\r\n", fps);
            frame_count = 0;
            encoder_count = 0;
            start_time = now;
        }
    }
}