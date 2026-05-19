#include "img.hpp"
#include "lq_all_demo.hpp"
int min(int a,int b){
    return a<b?a:b;
}
// ====================== 核心转换函数 ======================
bool cvMat2image_t(const cv::Mat &src, image_t *dst)
{
    // 安全检查
    if (dst == nullptr || src.empty() || src.channels() != 1)
        return false;

    // 只支持 8位灰度图
    if (src.type() != CV_8UC1)
        return false;

    // 赋值
    dst->data = src.data;
    dst->width = src.cols;
    dst->height = src.rows;
    dst->step = src.step;

    return true;
}

#define AT                  AT_IMAGE
#define AT_CLIP(img, x, y)  AT_IMAGE((img), clip((x), 0, (img)->width-1), clip((y), 0, (img)->height-1));

int clip(int x, int low, int up)
{
    if (x < low) return low;
    if (x > up) return up;
    return x;
}
float clipf(float x, float low, float up)
{
    if (x < low) return low;
    if (x > up) return up;
    return x;
}
void search_leftline()
{
    int x, y;
    bool found_flag = false;

    // 1. 设置起点：中心 - begin_x
    y = begin_y;
    x = img_raw.width / 2 - begin_x;

    // 安全裁剪
    if (y < 0) y = 0;
    if (y >= img_raw.height) y = img_raw.height - 1;
    if (x < 0) x = 0;
    if (x >= img_raw.width) x = img_raw.width - 1;

    auto safe_AT = [&](int px, int py) -> uint8_t {
        if (px < 0 || px >= img_raw.width) return 0;
        if (py < 0 || py >= img_raw.height) return 0;
        return AT_IMAGE(&img_raw, px, py);
    };

    // ==============================================
    // 原来逻辑：向左找突变点
    // ==============================================
    for (; x > 0; x--)
    {
        if (safe_AT(x - 1, y) < thres)
        {
            break;
        }
    }

    // ==============================================
    // 判断是否找到有效点
    // ==============================================
    if (safe_AT(x, y) >= thres)
    {
        found_flag = true;
    }
    else
    {
        found_flag = false;
    }

    // ==============================================
    // 兜底逻辑：找不到 → 强制从【左边缘】开始巡线
    // ==============================================
    if (!found_flag)
    {
        x = 2;
        if (safe_AT(x, y) >= thres) {
            found_flag = true;
        }
    }

    // ==============================================
    // 找到就巡线
    // ==============================================
    if (found_flag)
    {
        ipts0_num = sizeof(ipts0) / sizeof(ipts0[0]);
        findline_lefthand_adaptive(&img_raw, block_size, clip_value, x, y, ipts0, &ipts0_num);
    }
    else
    {
        ipts0_num = 0;
    }

    // ====================== ✅ 关键：高度过滤 ======================
    int valid_num = 0;
    for (int i = 0; i < ipts0_num; i++) {
        int py = ipts0[i][1];
        // 只保留 Y 在 [track_min_y, track_max_y] 之间的点
        if (py >= track_min_y && py <= track_max_y) {
            ipts0[valid_num][0] = ipts0[i][0];
            ipts0[valid_num][1] = ipts0[i][1];
            valid_num++;
        }
    }
    ipts0_num = valid_num;
}
void search_rightline()
{
    int x, y;
    bool found_flag = false;

    // 1. 设置起点：中心 + begin_x
    y = begin_y;
    x = img_raw.width / 2 + begin_x;

    // 安全裁剪
    if (y < 0) y = 0;
    if (y >= img_raw.height) y = img_raw.height - 1;
    if (x < 0) x = 0;
    if (x >= img_raw.width) x = img_raw.width - 1;

    auto safe_AT = [&](int px, int py) -> uint8_t {
        if (px < 0 || px >= img_raw.width) return 0;
        if (py < 0 || py >= img_raw.height) return 0;
        return AT_IMAGE(&img_raw, px, py);
    };

    // ==============================================
    // 原来逻辑：向右找突变点
    // ==============================================
    for (; x < img_raw.width - 1; x++)
    {
        if (safe_AT(x + 1, y) < thres)
        {
            break;
        }
    }

    // ==============================================
    // 判断是否找到有效点
    // ==============================================
    if (safe_AT(x, y) >= thres)
        found_flag = true;
    else
        found_flag = false;

    // ==============================================
    // 兜底逻辑：找不到 → 强制从【右边缘】开始巡线
    // ==============================================
    if (!found_flag)
    {
        x = img_raw.width - 3;
        if (safe_AT(x, y) >= thres) {
            found_flag = true;
        }
    }

    // ==============================================
    // 找到就巡线
    // ==============================================
    if (found_flag)
    {
        ipts1_num = sizeof(ipts1) / sizeof(ipts1[0]);
        findline_righthand_adaptive(&img_raw, block_size, clip_value, x, y, ipts1, &ipts1_num);
    }
    else
    {
        ipts1_num = 0;
    }

    // ====================== ✅ 关键：高度过滤 ======================
    int valid_num = 0;
    for (int i = 0; i < ipts1_num; i++) {
        int py = ipts1[i][1];
        if (py >= track_min_y && py <= track_max_y) {
            ipts1[valid_num][0] = ipts1[i][0];
            ipts1[valid_num][1] = ipts1[i][1];
            valid_num++;
        }
    }
    ipts1_num = valid_num;
}
void process_image() {
    search_leftline();
    search_rightline();

    // ===================== 左右边线照常逆透视（完全不动！）=====================
    for (int i = 0; i < ipts0_num; i++) {
        rpts0[i][0] = mapx[ipts0[i][1]][ipts0[i][0]];
        rpts0[i][1] = mapy[ipts0[i][1]][ipts0[i][0]];
    }
    rpts0_num = ipts0_num;

    for (int i = 0; i < ipts1_num; i++) {
        rpts1[i][0] = mapx[ipts1[i][1]][ipts1[i][0]];
        rpts1[i][1] = mapy[ipts1[i][1]][ipts1[i][0]];
    }
    rpts1_num = ipts1_num;

    // 边线滤波（不变）
    blur_points(rpts0, rpts0_num, rpts0b, (int)round(line_blur_kernel));
    rpts0b_num = rpts0_num;
    blur_points(rpts1, rpts1_num, rpts1b, (int)round(line_blur_kernel));
    rpts1b_num = rpts1_num;

    // 等距采样（不变）
    rpts0s_num = sizeof(rpts0s) / sizeof(rpts0s[0]);
    resample_points(rpts0b, rpts0b_num, rpts0s, &rpts0s_num, sample_dist * pixel_per_meter);
    rpts1s_num = sizeof(rpts1s) / sizeof(rpts1s[0]);
    resample_points(rpts1b, rpts1b_num, rpts1s, &rpts1s_num, sample_dist * pixel_per_meter);
    

    // ===================== 【中线选择逻辑：只改中线，不改线】=====================
    int approx = (int)round(angle_dist / sample_dist);
/*
    // 🔹 条件：左右线都不丢 → 使用【原图中线→逆透视】
    if (ipts0_num > 10 && ipts1_num > 10)  
    {
        float original_mid[200][2];
        float ipm_mid[200][2];
        int mid_num =ipts0_num<ipts1_num?ipts0_num:ipts1_num;
        int rptsn_num =rpts0s_num<rpts1s_num?rpts0s_num:rpts1s_num;

        // 1. 原图算中线
        for (int i = 0; i < mid_num; i++) {
            original_mid[i][0] = (ipts0[i][0] + ipts1[i][0]) / 2.0f;
            original_mid[i][1] = (ipts0[i][1] + ipts1[i][1]) / 2.0f;
        }

        // 2. 中线逆透视
        for (int i = 0; i < mid_num; i++) {
            int x = clip((int)original_mid[i][0], 0, img_raw.width - 1);
            int y = clip((int)original_mid[i][1], 0, img_raw.height - 1);
            ipm_mid[i][0] = mapx[y][x];
            ipm_mid[i][1] = mapy[y][x];
        }

        // 3. 平滑 + 重采样
        float mid_blur[240][2];
        blur_points(ipm_mid, mid_num, mid_blur, (int)round(line_blur_kernel));

        float mid_final[240][2];
        int mid_final_num = sizeof(mid_final) / sizeof(mid_final[0]);;
        resample_points(mid_blur, mid_num, mid_final, &mid_final_num, sample_dist * pixel_per_meter);

        // 4. 只赋值给中线，不影响左右线
        rptsn_num = mid_final_num;
        for (int i = 0; i < rptsn_num; i++) {
            rptsn[i][0] = mid_final[i][0];
            rptsn[i][1] = mid_final[i][1];
        }
    }
    else  
    {
        */
        // 丢线 → 使用你原来的中线
        track_leftline(rpts0s, rpts0s_num, rptsc0, approx,  pixel_per_meter * ROAD_WIDTH / 2);
        rptsc0_num = rpts0s_num;

        track_rightline(rpts1s, rpts1s_num, rptsc1, approx,  pixel_per_meter * ROAD_WIDTH / 2);
        rptsc1_num = rpts1s_num;
   // }
}


extern int clip(int x, int low, int up);

void clone_image(image_t *img0, image_t *img1){
    assert(img0 && img0->data);
    assert(img1 && img1->data);
    assert(img0->width == img1->width && img0->height == img1->height);
    assert(img0 != img1 && img0->data != img1->data);

    if (img0->width == img0->step && img1->width == img1->step) {
        memcpy(img1->data, img0->data, img0->width * img0->height);
    } else {
        for (int y = 0; y < img0->height; y++) {
            memcpy(&AT(img1, 0, y), &AT(img0, 0, y), img0->width);
        }
    }
}

void clear_image(image_t *img) {
    assert(img && img->data);
    if (img->width == img->step) {
        memset(img->data, 0, img->width * img->height);
    } else {
        for (int y = 0; y < img->height; y++) {
            memset(&AT(img, 0, y), 0, img->width);
        }
    }
}

// 固定阈值二值化
void threshold(image_t *img0, image_t *img1, uint8_t thres, uint8_t low_value, uint8_t high_value){
    assert(img0 && img0->data);
    assert(img1 && img1->data);
    assert(img0->width == img1->width && img0->height == img1->height);

    // 先遍历y后遍历x比较cache-friendly
    for (int y = 0; y < img0->height; y++) {
        for (int x = 0; x < img0->width; x++) {
                    AT(img1, x, y) = AT(img0, x, y) < thres ? low_value : high_value;
        }
    }
}

// 自适应阈值二值化
void adaptive_threshold(image_t *img0, image_t *img1, int block_size, int down_value, uint8_t low_value, uint8_t high_value) {
    assert(img0 && img0->data);
    assert(img1 && img1->data);
    assert(img0->data != img1->data);
    assert(img0->width == img1->width && img0->height == img1->height);
    assert(block_size > 1 && block_size % 2 == 1);

    int half = block_size / 2;
    // 先遍历y后遍历x比较cache-friendly
    for (int y = 0; y < img0->height; y++) {
        for (int x = 0; x < img0->width; x++) {
            int thres_value = 0;
            for (int dy = -half; dy <= half; dy++) {
                for (int dx = -half; dx <= half; dx++) {
                    thres_value += AT_CLIP(img0, x + dx, y + dy);
                }
            }
            thres_value /= block_size * block_size;
            thres_value -= down_value;
                    AT(img1, x, y) = AT(img0, x, y) < thres_value ? low_value : high_value;
        }
    }
}

// 图像逻辑与
void image_and(image_t *img0, image_t *img1, image_t *img2){
    assert(img0 && img0->data);
    assert(img1 && img1->data);
    assert(img2 && img2->data);
    assert(img0->width == img1->width && img0->height == img1->height);
    assert(img0->width == img2->width && img0->height == img2->height);

    // 先遍历y后遍历x比较cache-friendly
    for (int y = 0; y < img0->height; y++) {
        for (int x = 0; x < img0->width; x++) {
                    AT(img2, x, y) = (AT(img0, x, y) == 0 || AT(img1, x, y) == 0) ? 0 : 255;
        }
    }
}

// 图像逻辑或
void image_or(image_t *img0, image_t *img1, image_t *img2){
    assert(img0 && img0->data);
    assert(img1 && img1->data);
    assert(img2 && img2->data);
    assert(img0->width == img1->width && img0->height == img1->height);
    assert(img0->width == img2->width && img0->height == img2->height);

    // 先遍历y后遍历x比较cache-friendly
    for (int y = 0; y < img0->height; y++) {
        for (int x = 0; x < img0->width; x++) {
                    AT(img2, x, y) = (AT(img0, x, y) == 0 && AT(img1, x, y) == 0) ? 0 : 255;
        }
    }
}

// 2x2最小池化(赛道边界是黑色，最小池化可以较好保留赛道边界)
void minpool2(image_t *img0, image_t *img1) {
    assert(img0 && img0->data);
    assert(img1 && img1->data);
    assert(img0->width / 2 == img1->width && img0->height / 2 == img1->height);
    assert(img0 != img1 && img0->data != img1->data);

    uint8_t min_value;
    // 先遍历y后遍历x比较cache-friendly
    for (int y = 1; y < img0->height; y += 2) {
        for (int x = 1; x < img0->width; x += 2) {
            min_value = 255;
            if (AT(img0, x, y) < min_value) min_value = AT(img0, x, y);
            if (AT(img0, x - 1, y) < min_value) min_value = AT(img0, x - 1, y);
            if (AT(img0, x, y - 1) < min_value) min_value = AT(img0, x, y - 1);
            if (AT(img0, x - 1, y - 1) < min_value) min_value = AT(img0, x - 1, y - 1);
                    AT(img1, x / 2, y / 2) = min_value;
        }
    }
}

// 图像滤波降噪
void blur(image_t *img0, image_t *img1, uint32_t kernel) {
    assert(img0 && img0->data);
    assert(img1 && img1->data);
    assert(img0->width == img1->width && img0->height == img1->height);
    assert(img0 != img1 && img0->data != img1->data);

    // 先遍历y后遍历x比较cache-friendly
    for (int y = 1; y < img0->height - 1; y++) {
        for (int x = 1; x < img0->width - 1; x++) {
                    AT(img1, x, y) = (1 * AT(img0, x - 1, y - 1) + 2 * AT(img0, x, y - 1) + 1 * AT(img0, x + 1, y - 1) +
                                      2 * AT(img0, x - 1, y) + 4 * AT(img0, x, y) + 2 * AT(img0, x + 1, y) +
                                      1 * AT(img0, x - 1, y + 1) + 2 * AT(img0, x, y + 1) + 1 * AT(img0, x + 1, y + 1)) / 16;
        }
    }
}

// 3x3sobel边缘提取
void sobel3(image_t *img0, image_t *img1){
    assert(img0 && img0->data);
    assert(img1 && img1->data);
    assert(img0->width == img1->width && img0->height == img1->height);
    assert(img0 != img1 && img0->data != img1->data);

    int gx, gy;
    // 先遍历y后遍历x比较cache-friendly    
    for (int y = 1; y < img0->height - 1; y++) {
        for (int x = 1; x < img0->width - 1; x++) {
            gx = (-1 * AT(img0, x - 1, y - 1) + 1 * AT(img0, x + 1, y - 1) +
                  -2 * AT(img0, x - 1, y) + 2 * AT(img0, x + 1, y) +
                  -1 * AT(img0, x - 1, y + 1) + 1 * AT(img0, x + 1, y + 1)) / 4;
            gy = (1 * AT(img0, x - 1, y - 1) + 2 * AT(img0, x, y - 1) + 1 * AT(img0, x + 1, y - 1) +
                  -1 * AT(img0, x - 1, y + 1) - 2 * AT(img0, x, y + 1) - 1 * AT(img0, x + 1, y + 1)) / 4;
                    AT(img1, x, y) = (abs(gx) + abs(gy)) / 2;
        }
    }
}

// 3x3腐蚀
void erode3(image_t *img0, image_t *img1) {
    assert(img0 && img0->data);
    assert(img1 && img1->data);
    assert(img0->width == img1->width && img0->height == img1->height);
    assert(img0 != img1 && img0->data != img1->data);

    int min_value;
    // 先遍历y后遍历x比较cache-friendly    
    for (int y = 1; y < img0->height - 1; y++) {
        for (int x = 1; x < img0->width - 1; x++) {
            min_value = 255;
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    if (AT(img0, x + dx, y + dy) < min_value) min_value = AT(img0, x + dx, y + dy);
                }
            }
                    AT(img1, x, y) = min_value;
        }
    }
}

// 3x3膨胀
void dilate3(image_t *img0, image_t *img1) {
    assert(img0 && img0->data);
    assert(img1 && img1->data);
    assert(img0->width == img1->width && img0->height == img1->height);
    assert(img0 != img1 && img0->data != img1->data);

    int max_value;
    // 先遍历y后遍历x比较cache-friendly    
    for (int y = 1; y < img0->height - 1; y++) {
        for (int x = 1; x < img0->width - 1; x++) {
            max_value = 0;
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    if (AT(img0, x + dx, y + dy) > max_value) max_value = AT(img0, x + dx, y + dy);
                }
            }
                    AT(img1, x, y) = max_value;
        }
    }
}

//
void remap(image_t *img0, image_t *img1, fimage_t *mapx, fimage_t *mapy){
    assert(img0 && img0->data);
    assert(img1 && img1->data);
    assert(mapx && mapx->data);
    assert(mapy && mapy->data);
    assert(img0 != img1 && img0->data != img1->data);
    assert(img0->width == img1->width && img0->height == img1->height);
    assert(mapx->width == mapy->width && mapx->height == mapy->height);
    assert(img0->width == mapx->width && img0->height == mapx->height);

    // 先遍历y后遍历x比较cache-friendly    
    for (int y = 1; y < img0->height - 1; y++) {
        for (int x = 1; x < img0->width - 1; x++) {
                    AT(img1, x, y) = AT(img0, (int) (AT(mapx, x, y) + 0.5), (int) (AT(mapy, x, y) + 0.5));
        }
    }
}

/* 前进方向定义：
 *   0
 * 3   1
 *   2
 */
const int dir_front[4][2]= {{0,  -1},
                            {1,  0},
                             {0,  1},
                            {-1, 0}};
const int dir_frontleft[4][2] = {{-1, -1},
                                   {1,  -1},
                                 {1,  1},
                                 {-1, 1}};
const int dir_frontright[4][2] = {{1,  -1},
                                  {1,  1},
                                   {-1, 1},
                                       {-1, -1}};

// 左手迷宫巡线
void findline_lefthand_adaptive(image_t *img, int block_size, int clip_value, int x, int y, int pts[][2], int *num) {
    assert(img && img->data);
    assert(num && *num >= 0);
    assert(block_size > 1 && block_size % 2 == 1);
    int half = block_size / 2;
    int step = 0, dir = 0, turn = 0;

    // ✅ 真正正确的条件：允许走到图像边缘！
    while (step < *num
        && x >= 0 && x < img->width
        && y >= 0 && y < img->height
        && y > track_min_y   // 远处截断，不回头
        && turn < 4)
    {
        // ✅ 边缘安全求和（不会越界！）
        int local_thres = 0;
        int count = 0;
        for (int dy = -half; dy <= half; dy++) {
            for (int dx = -half; dx <= half; dx++) {
                int cx = x + dx;
                int cy = y + dy;
                if (cx >= 0 && cx < img->width && cy >= 0 && cy < img->height) {
                    local_thres += AT(img, cx, cy);
                    count++;
                }
            }
        }
        local_thres /= count; // 用有效像素平均
        local_thres -= clip_value;

        int front_value = AT_CLIP(img, x + dir_front[dir][0], y + dir_front[dir][1]);
        int frontleft_value = AT_CLIP(img, x + dir_frontleft[dir][0], y + dir_frontleft[dir][1]);

        if (front_value < local_thres) {
            dir = (dir + 1) % 4;
            turn++;
        } else if (frontleft_value < local_thres) {
            x += dir_front[dir][0];
            y += dir_front[dir][1];
            pts[step][0] = x;
            pts[step][1] = y;
            step++;
            turn = 0;
        } else {
            x += dir_frontleft[dir][0];
            y += dir_frontleft[dir][1];
            dir = (dir + 3) % 4;
            pts[step][0] = x;
            pts[step][1] = y;
            step++;
            turn = 0;
        }
    }
    *num = step;
}

// 右手迷宫巡线
// 右手迷宫巡线
void findline_righthand_adaptive(image_t *img, int block_size, int clip_value, int x, int y, int pts[][2], int *num) {
    assert(img && img->data);
    assert(num && *num >= 0);
    assert(block_size > 1 && block_size % 2 == 1);
    int half = block_size / 2;
    int step = 0, dir = 0, turn = 0;

    // ✅ 和左手完全一样！允许走到边缘！
    while (step < *num
        && x >= 0 && x < img->width
        && y >= 0 && y < img->height
        && y > track_min_y
        && turn < 4)
    {
        // ✅ 边缘安全求和
        int local_thres = 0;
        int count = 0;
        for (int dy = -half; dy <= half; dy++) {
            for (int dx = -half; dx <= half; dx++) {
                int cx = x + dx;
                int cy = y + dy;
                if (cx >= 0 && cx < img->width && cy >= 0 && cy < img->height) {
                    local_thres += AT(img, cx, cy);
                    count++;
                }
            }
        }
        local_thres /= count;
        local_thres -= clip_value;

        int front_value = AT_CLIP(img, x + dir_front[dir][0], y + dir_front[dir][1]);
        int frontright_value = AT_CLIP(img, x + dir_frontright[dir][0], y + dir_frontright[dir][1]);

        if (front_value < local_thres) {
            dir = (dir + 3) % 4;
            turn++;
        } else if (frontright_value < local_thres) {
            x += dir_front[dir][0];
            y += dir_front[dir][1];
            pts[step][0] = x;
            pts[step][1] = y;
            step++;
            turn = 0;
        } else {
            x += dir_frontright[dir][0];
            y += dir_frontright[dir][1];
            dir = (dir + 1) % 4;
            pts[step][0] = x;
            pts[step][1] = y;
            step++;
            turn = 0;
        }
    }
    *num = step;
}

// 折线段近似
void approx_lines(int pts[][2], int pts_num, float epsilon, int lines[][2], int *lines_num) {
    assert(pts);
    assert(epsilon > 0);

    int dx = pts[pts_num - 1][0] - pts[0][0];
    int dy = pts[pts_num - 1][1] - pts[0][1];
    float nx = -dy / sqrtf(dx * dx + dy * dy);
    float ny = dx / sqrtf(dx * dx + dy * dy);
    float max_dist = 0, dist;
    int idx = -1;
    for (int i = 1; i < pts_num - 1; i++) {
        dist = fabs((pts[i][0] - pts[0][0]) * nx + (pts[i][1] - pts[0][1]) * ny);
        if (dist > max_dist) {
            max_dist = dist;
            idx = i;
        }
    }
    if (max_dist >= epsilon) {
        int num1 = *lines_num;
        approx_lines(pts, idx + 1, epsilon, lines, &num1);
        int num2 = *lines_num - num1 - 1;
        approx_lines(pts + idx, pts_num - idx, epsilon, lines + num1 - 1, &num2);
        *lines_num = num1 + num2 - 1;
    } else {
        lines[0][0] = pts[0][0];
        lines[0][1] = pts[0][1];
        lines[1][0] = pts[pts_num - 1][0];
        lines[1][1] = pts[pts_num - 1][1];
        *lines_num = 2;
    }
}

// float类型的折线段近似
void approx_lines_f(float pts[][2], int pts_num, float epsilon, float lines[][2], int *lines_num){
    assert(pts);
    assert(epsilon > 0);

    int dx = pts[pts_num - 1][0] - pts[0][0];
    int dy = pts[pts_num - 1][1] - pts[0][1];
    float nx = -dy / sqrtf(dx * dx + dy * dy);
    float ny = dx / sqrtf(dx * dx + dy * dy);
    float max_dist = 0, dist;
    int idx = -1;
    for (int i = 1; i < pts_num - 1; i++) {
        dist = fabs((pts[i][0] - pts[0][0]) * nx + (pts[i][1] - pts[0][1]) * ny);
        if (dist > max_dist) {
            max_dist = dist;
            idx = i;
        }
    }
    if (max_dist >= epsilon && *lines_num > 2) {
        int num1 = *lines_num;
        approx_lines_f(pts, idx + 1, epsilon, lines, &num1);
        int num2 = *lines_num - num1 - 1;
        approx_lines_f(pts + idx, pts_num - idx, epsilon, lines + num1 - 1, &num2);
        *lines_num = num1 + num2 - 1;
    } else {
        lines[0][0] = pts[0][0];
        lines[0][1] = pts[0][1];
        lines[1][0] = pts[pts_num - 1][0];
        lines[1][1] = pts[pts_num - 1][1];
        *lines_num = 2;
    }
}

void draw_line(image_t *img, int pt0[2], int pt1[2], uint8_t value) {
    int dx = pt1[0] - pt0[0];
    int dy = pt1[1] - pt0[1];
    if (abs(dx) > abs(dy)) {
        for (int x = pt0[0]; x != pt1[0]; x += (dx > 0 ? 1 : -1)) {
            int y = pt0[1] + (x - pt0[0]) * dy / dx;
                    AT(img, clip(x, 0, img->width - 1), clip(y, 0, img->height - 1)) = value;
        }
    } else {
        for (int y = pt0[1]; y != pt1[1]; y += (dy > 0 ? 1 : -1)) {
            int x = pt0[0] + (y - pt0[1]) * dx / dy;
                    AT(img, clip(x, 0, img->width - 1), clip(y, 0, img->height - 1)) = value;
        }
    }
}

// 分块计算大津阈值
// x0到x1,y0到y1
uint16_t getOSTUThreshold(image_t *img, uint8_t MinThreshold, uint8_t MaxThreshold, uint8_t x0, uint8_t x1, uint8_t y0, uint8_t y1) {
    /*灰度直方图参数*/
    uint16_t histogram[256] = {0}; // 灰度直方图
    uint32_t min_value, max_value;

    uint32_t pix_amount = 0;   // 像素点总数
    uint32_t pix_integral = 0; // 灰度值总数

    uint32_t pix_back_amount = 0;   // 前景像素点总数
    uint32_t pix_back_integral = 0; // 前景灰度值

    int32_t pix_fore_amount = 0;   // 背景像素点总数
    int32_t pix_fore_integral = 0; // 背景灰度值

    float omega_back, omega_fore, micro_back, micro_fore, sigma_beta, sigma; // 类间方差：浮点型更精确

    uint16_t thres_result = 0;

    // 隔一行取一个值，更快
    for (uint8_t y = y0; y < y1; y += 2)
        for (uint8_t x = x0; x < x1; x += 2)
            ++histogram[AT_IMAGE(img, x, y)];

    for (min_value = 0; min_value < 256 && histogram[min_value] == 0; min_value++)
    {
        ; // 获取最小灰度的值
    }
    for (max_value = 255; max_value > min_value && histogram[min_value] == 0; max_value--)
    {
        ; // 获取最大灰度的值
    }
    if (max_value == min_value)
    {
        return ((uint8)(max_value)); // 图像中只有一个颜色
    }
    if (min_value + 1 == max_value)
    {
        return ((uint8)(min_value)); // 图像中只有二个颜色
    }

    /*OSTU大律法*/
    for (uint16_t j = (uint16)min_value; j <= max_value; j++)
    {
        pix_amount += histogram[j]; //  像素总数
    }
    pix_integral = 0;
    for (uint16_t j = (uint16)min_value; j <= max_value; j++)
    {
        pix_integral += histogram[j] * j; // 灰度值总数
    }
    sigma_beta = -1;

    for (uint16_t j = (uint16)min_value; j < max_value; j++)
    {
        pix_back_amount = pix_back_amount + histogram[j];                                        // 前景像素点数
        pix_fore_amount = pix_amount - pix_back_amount;                                          // 背景像素点数
        omega_back = (float)pix_back_amount / pix_amount;                                        // 前景像素百分比
        omega_fore = (float)pix_fore_amount / pix_amount;                                        // 背景像素百分比
        pix_back_integral += histogram[j] * j;                                                   // 前景灰度值
        pix_fore_integral = pix_integral - pix_back_integral;                                    // 背景灰度值
        micro_back = (float)pix_back_integral / pix_back_amount;                                 // 前景灰度百分比
        micro_fore = (float)pix_fore_integral / pix_fore_amount;                                 // 背景灰度百分比
        sigma = omega_back * omega_fore * (micro_back - micro_fore) * (micro_back - micro_fore); // 计算类间方差
        if (sigma > sigma_beta)                                                                  // 遍历最大的类间方差g //找出最大类间方差以及对应的阈值
        {
            sigma_beta = sigma;
            thres_result = (uint8)j;
        }
    }

    return thres_result; // 返回最佳阈值;
}

// 点集三角滤波
// 功能：对一串连续的2D点进行【三角加权平滑/模糊】
// 作用：让抖动、折线变得顺滑（车道线/轮廓点/坐标序列去噪）
// 输入：pts_in  输入点集 (x,y)
//       num     点的总个数
//       kernel  平滑窗口大小（必须是奇数，如3、5、7）
// 输出：pts_out 平滑后的点集
void blur_points(float pts_in[][2], int num, float pts_out[][2], int kernel) {
    // 断言：窗口大小必须是奇数（保证中心点对称）
    assert(kernel % 2 == 1);
    
    // 半窗口宽度 = 窗口中心向左右各取多少个点
    int half = kernel / 2;

    // 遍历每一个输入点，逐个计算平滑后的点
    for (int i = 0; i < num; i++) {
        // 初始化输出点坐标为0
        pts_out[i][0] = pts_out[i][1] = 0;

        // 窗口内循环：从当前点左边half个 → 右边half个
        for (int j = -half; j <= half; j++) {
            // clip：防止索引越界，把 i+j 限制在 [0, num-1] 范围内
            int idx = clip(i + j, 0, num - 1);
            
            // 权重公式：离中心点越近，权重越大（三角权重）
            float weight = (half + 1 - abs(j));
            
            // 累加 x、y：每个点 × 对应权重
            pts_out[i][0] += pts_in[idx][0] * weight;
            pts_out[i][1] += pts_in[idx][1] * weight;
        }

        // 分母 = 所有权重之和（归一化，保证平均后数值合理）
        float sum_weight = (2 * half + 2) * (half + 1) / 2;
        
        // 除以总权重，得到最终平滑后的 x、y
        pts_out[i][0] /= sum_weight;
        pts_out[i][1] /= sum_weight;
    }
}
// 功能：点集【等距重采样】
// 原理：沿着原始折线行走，**每走固定距离 dist 就取一个点**
// 效果：输出点与点之间的距离 ≈ dist，分布均匀
// 输入：pts_in  输入点集
//       num1    输入点个数
//       dist    采样间隔距离
// 输出：pts_out 输出的均匀点集
//       num2    输出点的个数（传入最大容量，返回真实数量）
void resample_points(float pts_in[][2], int num1, float pts_out[][2], int *num2, float dist){
    // 剩余距离：记录当前段还剩多少距离没走到下一个采样点
    float remain = 0.f;
    // 输出点的计数器
    int len = 0;

    // 遍历输入点集的每一段（i 到 i+1）
    for(int i=0; i<num1-1 && len < *num2; i++){
        // 当前段起点坐标
        float x0 = pts_in[i][0];
        float y0 = pts_in[i][1];
        // 当前段终点与起点的差值
        float dx = pts_in[i+1][0] - x0;
        float dy = pts_in[i+1][1] - y0;
        // 当前段的总长度
        float dn = sqrt(dx*dx+dy*dy);
        
        // 把 dx/dy 转为单位方向向量（长度=1）
        dx /= dn;
        dy /= dn;

        // 循环：在当前线段上，按 dist 间隔取点
        while(remain < dn && len < *num2){
            // 从起点往前走【remain】距离，就是新采样点
            x0 += dx * remain;
            pts_out[len][0] = x0;
            y0 += dy * remain;
            pts_out[len][1] = y0;
            
            // 输出点数量+1
            len++;
            // 当前段剩余长度减去已走的 remain
            dn -= remain;
            // 下一次要走满一个 dist 距离
            remain = dist;
        }
        // 当前段走完了，更新剩余距离
        remain -= dn;
    }
    // 返回最终输出的点数量
    *num2 = len;
}
// 功能：点集等距采样2（意图：让输出点之间距离 = dist）
// 说明：代码有BUG，作者标记了 TODO: fix bug，不推荐使用
// 输入：pts_in  输入点集
//       num1    输入点数量
//       dist    目标采样间隔
// 输出：pts_out 采样后的点
//       num2    输出点数量
void resample_points2(float pts_in[][2], int num1, float pts_out[][2], int *num2, float dist){
    // 安全判断：输入无点时直接返回
    if (num1 < 0) {
        *num2 = 0;
        return;
    }

    // 输出第一个点 = 输入第一个点（起点必须保留）
    pts_out[0][0] = pts_in[0][0];
    pts_out[0][1] = pts_in[0][1];
    // 输出点数量从 1 开始计数
    int len = 1;

    // 遍历输入点的每一段
    for (int i = 0; i < num1 - 1 && len < *num2; i++) {
        // 当前段起点 & 终点
        float x0 = pts_in[i][0];
        float y0 = pts_in[i][1];
        float x1 = pts_in[i+1][0];
        float y1 = pts_in[i+1][1];

        // 循环：在当前线段上继续采样
        do {
            // 取上一个输出的点作为参考点
            float x = pts_out[len - 1][0];
            float y = pts_out[len - 1][1];

            // 计算参考点到当前段起点、终点的距离
            float dx0 = x0 - x;
            float dy0 = y0 - y;
            float dx1 = x1 - x;
            float dy1 = y1 - y;
            float dist0 = sqrt(dx0 * dx0 + dy0 * dy0);
            float dist1 = sqrt(dx1 * dx1 + dy1 * dy1);

            // 错误的插值计算（逻辑有问题，是BUG根源）
            float r0 = (dist1 - dist) / (dist1 - dist0);
            float r1 = 1 - r0;

            // 插值系数无效就跳出（不在线段上）
            if (r0 < 0 || r1 < 0) break;
            
            // 按错误系数插值生成新点
            x0 = x0 * r0 + x1 * r1;
            y0 = y0 * r0 + y1 * r1;
            pts_out[len][0] = x0;
            pts_out[len][1] = y0;
            len++;
        } while (len < *num2);
    }

    // 返回最终输出点数量
    *num2 = len;
}
// 功能：计算点集每个点的【局部角度变化率】（即该点前后方向的夹角）
// 用途：判断车道线弯曲程度、直道/弯道、拐点检测
// 输入：
//   pts_in    输入点集（车道线/中线点）
//   num       点的数量
//   dist      前后采样距离（取前后第 dist 个点计算方向）
// 输出：
//   angle_out 每个点的角度变化值（弧度，正左负右/负左正右取决于方向）
void local_angle_points(float pts_in[][2], int num, float angle_out[], int dist) {
    // 遍历每个点
    for (int i = 0; i < num; i++) {
        // 边界点（第一个/最后一个）无角度变化，直接赋 0
        if (i <= 0 || i >= num - 1) {
            angle_out[i] = 0;
            continue;
        }

        // ==================== 计算【前向】向量（i-dist → i）====================
        // 向量1：当前点 指向 前面dist个点的方向
        float dx1 = pts_in[i][0] - pts_in[clip(i - dist, 0, num - 1)][0];
        float dy1 = pts_in[i][1] - pts_in[clip(i - dist, 0, num - 1)][1];
        float dn1 = sqrtf(dx1 * dx1 + dy1 * dy1);  // 向量长度

        // ==================== 计算【后向】向量（i → i+dist）====================
        // 向量2：当前点 指向 后面dist个点的方向
        float dx2 = pts_in[clip(i + dist, 0, num - 1)][0] - pts_in[i][0];
        float dy2 = pts_in[clip(i + dist, 0, num - 1)][1] - pts_in[i][1];
        float dn2 = sqrtf(dx2 * dx2 + dy2 * dy2);  // 向量长度

        // ==================== 向量归一化（单位向量）====================
        float c1 = dx1 / dn1;  // 向量1 x 单位分量
        float s1 = dy1 / dn1;  // 向量1 y 单位分量
        float c2 = dx2 / dn2;  // 向量2 x 单位分量
        float s2 = dy2 / dn2;  // 向量2 y 单位分量

        // ==================== 计算两个方向向量的【夹角】====================
        // 公式：向量点积求cos，叉积求sin → atan2得到夹角（弧度）
        // 结果越大 → 弯曲越剧烈；结果接近0 → 直道
        angle_out[i] = atan2f(c1 * s2 - c2 * s1, c2 * c1 + s2 * s1);
    }
}
// 功能：对角度变化率做【非极大值抑制 NMS】
// 原理：在一个局部窗口内，只保留【绝对值最大】的角度值，其余清零
// 用途：过滤抖动噪声，只保留真正的弯道/拐点，消除小波动
// 输入：
//   angle_in  输入的原始角度变化率
//   num       数组长度
//   kernel    抑制窗口大小（必须奇数，如 3、5、7）
// 输出：
//   angle_out 经过NMS抑制后的角度（仅局部峰值保留，其余=0）
void nms_angle(float angle_in[], int num, float angle_out[], int kernel) {
    // 确保窗口大小是奇数（对称窗口）
    assert(kernel % 2 == 1);
    int half = kernel / 2;  // 半窗口大小

    // 遍历每个点
    for (int i = 0; i < num; i++) {
        // 初始把当前点角度作为候选峰值
        angle_out[i] = angle_in[i];

        // 遍历窗口内所有点（左右 half 范围）
        for (int j = -half; j <= half; j++) {
            // 如果窗口内存在【比当前点更大】的角度
            if (fabs(angle_in[clip(i + j, 0, num - 1)]) > fabs(angle_out[i])) {
                angle_out[i] = 0;  // 当前点不是局部最大 → 清零
                break;             // 直接退出，不用再比
            }
        }
    }
}
// 功能：输入【左边线】，沿着车道【向内】偏移，生成中线
// 作用：左边线 → 向右偏移 → 中线
// 输入：左边线 pts_in
// 输出：中线 pts_out
// dist：偏移距离（车道半宽）
void track_leftline(float pts_in[][2], int num, float pts_out[][2], int approx_num, float dist)
{
    for (int i = 0; i < num; i++)
    {
        // 取前后点，计算左边线的切线方向
        float dx = pts_in[clip(i+approx_num,0,num-1)][0] - pts_in[clip(i-approx_num,0,num-1)][0];
        float dy = pts_in[clip(i+approx_num,0,num-1)][1] - pts_in[clip(i-approx_num,0,num-1)][1];
        float dn = sqrt(dx*dx+dy*dy);
        dx /= dn;
        dy /= dn;

        // ====================== 关键 ======================
        // 左边线 往 右 偏移 = 朝向车道中心
        // 向右垂直方向：(-dy, dx)
        // ==================================================
        pts_out[i][0] = pts_in[i][0] - dy * dist;
        pts_out[i][1] = pts_in[i][1] + dx * dist;
    }
}
// 功能：输入【右边线】，沿着车道【向内】偏移，生成中线
// 作用：右边线 → 向左偏移 → 中线
// 输入：右边线 pts_in
// 输出：中线 pts_out
// dist：偏移距离（车道半宽）
void track_rightline(float pts_in[][2], int num, float pts_out[][2], int approx_num, float dist)
{
    for (int i = 0; i < num; i++)
    {
        // 取前后点，计算右边线的切线方向
        float dx = pts_in[clip(i+approx_num,0,num-1)][0] - pts_in[clip(i-approx_num,0,num-1)][0];
        float dy = pts_in[clip(i+approx_num,0,num-1)][1] - pts_in[clip(i-approx_num,0,num-1)][1];
        float dn = sqrt(dx*dx+dy*dy);
        dx /= dn;
        dy /= dn;

        // ====================== 关键 ======================
        // 右边线 往 左 偏移 = 朝向车道中心
        // 向左垂直方向：(dy, -dx)
        // ==================================================
        pts_out[i][0] = pts_in[i][0] + dy * dist;
        pts_out[i][1] = pts_in[i][1] - dx * dist;
    }
}
void draw_x(image_t *img, int x, int y, int len, uint8_t value) {
    for (int i = -len; i <= len; i++) {
                AT(img, clip(x + i, 0, img->width - 1), clip(y + i, 0, img->height - 1)) = value;
                AT(img, clip(x - i, 0, img->width - 1), clip(y + i, 0, img->height - 1)) = value;
    }
}

void draw_o(image_t *img, int x, int y, int radius, uint8_t value) {
    for (float i = -PI; i <= PI; i += PI / 10) {
                AT(img, clip(x + radius * cosf(i), 0, img->width - 1), clip(y + radius * sinf(i), 0, img->height - 1)) = value;
    }
}

