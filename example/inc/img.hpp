#ifndef IMG_HPP
#define IMG_HPP

#include <stdint.h>
#include <opencv2/opencv.hpp>
typedef struct image {
    uint8_t *data;
    uint32_t width;
    uint32_t height;
    uint32_t step;
} image_t;

typedef struct fimage {
    float *data;
    uint32_t width;
    uint32_t height;
    uint32_t step;
} fimage_t;
#define AT_IMAGE(img, x, y)          ((img)->data[(y)*(img)->step+(x)])
#define AT_IMAGE_CLIP(img, x, y)     AT_IMAGE(img, clip(x, 0, (img)->width-1), clip(y, 0, (img)->height-1))

#define DEF_IMAGE(ptr, w, h)         {.data=ptr, .width=w, .height=h, .step=w}
#define ROI_IMAGE(img, x1, y1, w, h) {.data=&AT_IMAGE(img, x1, y1), .width=w, .height=h, .step=img.width}

#define uint8 uint8_t
#define uint16 uint16_t
#define PI               3.14159265358979f
#define IMG_W 320
#define IMG_H 240
extern image_t img_raw;
extern image_t img0;
extern image_t img_thres;
extern image_t img_line;

extern bool line_show_sample;
extern bool line_show_blur;
extern bool track_left;

extern float angle;
extern float mapx[IMG_H][IMG_W];
extern float mapy[IMG_H][IMG_W];
#define ROAD_WIDTH      (0.45)
#define POINTS_MAX_LEN  (240)//图像高度

#define FAR_POINTS_MAX_LEN  (POINTS_MAX_LEN)
extern  float thres;            // 固定二值化阈值（判断黑线/背景）
extern float block_size ;         // 自适应阈值的窗口大小
extern float clip_value ;         // 自适应阈值减去的偏移量
extern float begin_x ;           // 巡线起始点 水平偏移
extern float begin_y ;          // 巡线起始点 垂直位置（靠近车底）
extern float track_min_y ;   // 越小 → 巡得越远
extern float track_max_y ;  // 越大 → 巡到最底部
extern float line_blur_kernel ;   // 边线滤波平滑程度
extern float pixel_per_meter ;  // 像素 → 实际距离换算比例
extern float sample_dist ;     // 点集等距采样步长（米extern ）
extern float angle_dist ;       // 计算弯道角度的窗口长度
extern float far_rate ;         // 远处点权重（控制转向柔和度）
extern float aim_distance ;    // 目标点距离车身多远（米）
// 原图左右边线
extern int ipts0[POINTS_MAX_LEN][2];
extern int ipts1[POINTS_MAX_LEN][2];
extern int ipts0_num, ipts1_num;
// 变换后左右边线
extern float rpts0[POINTS_MAX_LEN][2];
extern float rpts1[POINTS_MAX_LEN][2];
extern int rpts0_num, rpts1_num;
// 变换后左右边线+滤波
extern float rpts0b[POINTS_MAX_LEN][2];
extern float rpts1b[POINTS_MAX_LEN][2];
extern int rpts0b_num, rpts1b_num;
// 变换后左右边线+等距采样
extern float rpts0s[POINTS_MAX_LEN][2];
extern float rpts1s[POINTS_MAX_LEN][2];
extern int rpts0s_num, rpts1s_num;
// 左右边线局部角度变化率
extern float rpts0a[POINTS_MAX_LEN];
extern float rpts1a[POINTS_MAX_LEN];
extern int rpts0a_num, rpts1a_num;
// 左右边线局部角度变化率+非极大抑制
extern float rpts0an[POINTS_MAX_LEN];
extern float rpts1an[POINTS_MAX_LEN];
extern int rpts0an_num, rpts1an_num;
// 左/右中线
extern float rptsc0[POINTS_MAX_LEN][2];
extern float rptsc1[POINTS_MAX_LEN][2];
extern int rptsc0_num, rptsc1_num;
// 中线
extern float (*rpts)[2];
extern int rpts_num;
// 归一化中线
extern float rptsn[POINTS_MAX_LEN][2];
extern int rptsn_num;

// Y角点
extern int Ypt0_rpts0s_id, Ypt1_rpts1s_id;
extern bool Ypt0_found, Ypt1_found;

// L角点
extern int Lpt0_rpts0s_id, Lpt1_rpts1s_id;
extern bool Lpt0_found, Lpt1_found;

//以下为十字控制寻远线操作,算法与常规寻线相同

extern bool far_Lpt0_found, far_Lpt1_found;
extern int far_Lpt0_rpts0s_id, far_Lpt1_rpts1s_id;
//原图远线左右边线
extern int far_ipts0[FAR_POINTS_MAX_LEN][2];
extern int far_ipts1[FAR_POINTS_MAX_LEN][2];
extern int far_ipts0_num, far_ipts1_num;
//变换后左右远边线
extern float far_rpts0[FAR_POINTS_MAX_LEN][2];
extern float far_rpts1[FAR_POINTS_MAX_LEN][2];
extern int far_rpts0_num, far_rpts1_num;
//变换后左右远边线+滤波
extern float far_rpts0b[FAR_POINTS_MAX_LEN][2];
extern float far_rpts1b[FAR_POINTS_MAX_LEN][2];
extern int far_rpts0b_num, far_rpts1b_num;
//变换后左右远边线+等距采样
extern float far_rpts0s[FAR_POINTS_MAX_LEN][2];
extern float far_rpts1s[FAR_POINTS_MAX_LEN][2];
extern int far_rpts0s_num, far_rpts1s_num;
// 左右远边线局部角度变化率
extern float far_rpts0a[FAR_POINTS_MAX_LEN];
extern float far_rpts1a[FAR_POINTS_MAX_LEN];
extern int far_rpts0a_num, far_rpts1a_num;
// 左右远边线局部角度变化率+非极大抑制
extern float far_rpts0an[FAR_POINTS_MAX_LEN];
extern float far_rpts1an[FAR_POINTS_MAX_LEN];
extern int far_rpts0an_num, far_rpts1an_num;

extern bool is_straight0, is_straight1;

enum track_type_e {
    TRACK_LEFT,
    TRACK_RIGHT,
    midd,
};
extern enum track_type_e track_type;

int clip(int x, int low, int up);

float clipf(float x, float low, float up);
void process_image () ;

bool cvMat2image_t(const cv::Mat &src, image_t *dst);

// 深拷贝图片，img0和img1不可以指向相同图片
void clone_image(image_t *img0, image_t *img1);

// 清空图片
void clear_image(image_t *img);

// 基础二值化，img0和img1可以指向相同图片
void threshold(image_t *img0, image_t *img1, uint8_t thres, uint8_t low_value, uint8_t high_value);

// 自适应二值化，img0和img1不可指向相同图片
void adaptive_threshold(image_t *img0, image_t *img1, int block_size, int down_value, uint8_t low_value, uint8_t high_value);

// 像素逻辑与，即都为255时才为255
void image_and(image_t *img0, image_t *img1, image_t *img2);

// 像素逻辑或，即都为255时才为255
void image_or(image_t *img0, image_t *img1, image_t *img2);

// 最小值降采样x2
void minpool2(image_t *img0, image_t *img1);

// 3x3高斯滤波，img0和img1不可以指向相同图片，最外圈像素点不参与计算
void blur3(image_t *img0, image_t *img1);

// 3x3 Sobel梯度计算，img0和img1不可以指向相同图片，最外圈像素点不参与计算
void sobel3(image_t *img0, image_t *img1);

// 3x3腐蚀运算，img0和img1不可以指向相同图片，最外圈像素点不参与计算
void erode3(image_t *img0, image_t *img1);

// 3x3膨胀运算，img0和img1不可以指向相同图片，最外圈像素点不参与计算
void dilate3(image_t *img0, image_t *img1);

// 图像变换，最近邻插值。img0和img1不可以指向相同图片（去畸变，投影变换等。使用opencv生成变换表）
// 未测试，可能有BUG
void remap(image_t *img0, image_t *img1, fimage_t *mapx, fimage_t *mapy);

// 左手巡线同时自适应二值化，从(x,y)开始向上沿白边线走
void findline_lefthand_adaptive(image_t *img, int block_size, int clip_value, int x, int y, int pts[][2], int *num);

// 右手巡线同时自适应二值化，从(x,y)开始向上沿白边线走
void findline_righthand_adaptive(image_t *img, int block_size, int clip_value, int x, int y, int pts[][2], int *num);

// 大津法计算二值化阈值
uint16_t getOSTUThreshold(image_t *img, uint8_t MinThreshold, uint8_t MaxThreshold, uint8_t x0, uint8_t x1, uint8_t y0, uint8_t y1);

// 点集三角滤波
void blur_points(float pts_in[][2], int num, float pts_out[][2], int kernel);

// 点集等距采样
void resample_points(float pts_in[][2], int num1, float pts_out[][2], int *num2, float dist);

// 点集局部角度变化率
void local_angle_points(float pts_in[][2], int num, float angle_out[], int dist);

// 角度变化率非极大抑制
void nms_angle(float angle_in[], int num, float angle_out[], int kernel);

// 左边线跟踪中线
void track_leftline(float pts_in[][2], int num, float pts_out[][2], int approx_num, float dist);

// 右边线跟踪中线
void track_rightline(float pts_in[][2], int num, float pts_out[][2], int approx_num, float dist);

// 绘制X
void draw_x(image_t *img, int x, int y, int len, uint8_t value);

// 绘制O
void draw_o(image_t *img, int x, int y, int radius, uint8_t value);

#endif // IMG_HPP