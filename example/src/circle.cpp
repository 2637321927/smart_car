#include "circle.hpp"
#include "img.hpp"
#include <algorithm>
#include<cmath>
#include <chrono>
#include <iostream>
#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif
enum cross_type_e cross_type = CROSS_NONE;

const char *cross_type_name[CROSS_NUM] = {
        "CROSS_NONE",
        "CROSS_BEGIN", "CROSS_IN",
};
enum circle_type_e circle_type = CIRCLE_NONE;



// 编码器，用于防止一些重复触发等。
int64_t circle_encoder;

int none_left_line = 0, none_right_line = 0;
int have_left_line = 0, have_right_line = 0;

namespace {

// 识别到一次环岛后，20 秒内不允许再次从 CIRCLE_NONE 进入环岛。
constexpr auto kCircleEnterCooldown = std::chrono::seconds(2);
constexpr auto kCircleOUTCooldown = std::chrono::seconds(0);
bool circle_enter_cooldown = false;
std::chrono::steady_clock::time_point last_circle_enter_time;
std::chrono::steady_clock::time_point last_cross_time;


bool circle_entry_is_blocked()
{
    if (!circle_enter_cooldown) {
        return false;
    }

    if (std::chrono::steady_clock::now() - last_circle_enter_time >= kCircleEnterCooldown&&std::chrono::steady_clock::now() - last_cross_time >= kCircleOUTCooldown) {
        circle_enter_cooldown = false;
        std::cout<<"not circle"<<std::endl;
        return false;
    }

    return true;
}

void start_circle_enter_cooldown()
{
    circle_enter_cooldown = true;
    last_circle_enter_time = std::chrono::steady_clock::now();
}
void start_cross_cooldown(){

    circle_enter_cooldown = true;
    last_cross_time = std::chrono::steady_clock::now();
}

} // namespace
// 功能：输入一个点(px, py)，向左右扫描，判断这个位置是不是斑马线
// 返回值：true = 是斑马线，false = 不是
bool check_is_zebra(image_t *img_raw, int px, int py, int thres) {
    int zebra_cross_flag0[100];
    int zebra_cross_flag0_num = 0;
    int zebra_cross_flag1[100];
    int zebra_cross_flag1_num = 0;

    // 判断起点是黑还是白
    int zebra_cross_flag_begin = AT_IMAGE(img_raw, px, py) > thres;

    //向左扫描（80像素）
    memset(zebra_cross_flag0, 0, sizeof(zebra_cross_flag0));
    zebra_cross_flag0_num = 0;
    for (int x = px - 1; x >= MAX(0, px - 80); x--) {
        if (zebra_cross_flag_begin == 0) { // 偶数白，奇数黑
            if (zebra_cross_flag0_num % 2 == 0 && AT_IMAGE(img_raw, x, py) > thres) {
                zebra_cross_flag0[zebra_cross_flag0_num]++;
            } else if (zebra_cross_flag0_num % 2 == 0 && AT_IMAGE(img_raw, x, py) < thres) {
                zebra_cross_flag0[++zebra_cross_flag0_num]++;
            } else if (zebra_cross_flag0_num % 2 == 1 && AT_IMAGE(img_raw, x, py) > thres) {
                zebra_cross_flag0[++zebra_cross_flag0_num]++;
            } else if (zebra_cross_flag0_num % 2 == 1 && AT_IMAGE(img_raw, x, py) < thres) {
                zebra_cross_flag0[zebra_cross_flag0_num]++;
            }
        } else { // 偶数黑，奇数白
            if (zebra_cross_flag0_num % 2 == 0 && AT_IMAGE(img_raw, x, py) > thres) {
                zebra_cross_flag0[++zebra_cross_flag0_num]++;
            } else if (zebra_cross_flag0_num % 2 == 0 && AT_IMAGE(img_raw, x, py) < thres) {
                zebra_cross_flag0[zebra_cross_flag0_num]++;
            } else if (zebra_cross_flag0_num % 2 == 1 && AT_IMAGE(img_raw, x, py) > thres) {
                zebra_cross_flag0[zebra_cross_flag0_num]++;
            } else if (zebra_cross_flag0_num % 2 == 1 && AT_IMAGE(img_raw, x, py) < thres) {
                zebra_cross_flag0[++zebra_cross_flag0_num]++;
            }
        }
    }

    //向右扫描（80像素）
    memset(zebra_cross_flag1, 0, sizeof(zebra_cross_flag1));
    zebra_cross_flag1_num = 0;
    for (int x = px + 1; x <= MIN(img_raw->width - 1, px + 80); x++) {
        if (zebra_cross_flag_begin == 0) { // 偶数白，奇数黑
            if (zebra_cross_flag1_num % 2 == 0 && AT_IMAGE(img_raw, x, py) > thres) {
                zebra_cross_flag1[zebra_cross_flag1_num]++;
            } else if (zebra_cross_flag1_num % 2 == 0 && AT_IMAGE(img_raw, x, py) < thres) {
                zebra_cross_flag1[++zebra_cross_flag1_num]++;
            } else if (zebra_cross_flag1_num % 2 == 1 && AT_IMAGE(img_raw, x, py) > thres) {
                zebra_cross_flag1[++zebra_cross_flag1_num]++;
            } else if (zebra_cross_flag1_num % 2 == 1 && AT_IMAGE(img_raw, x, py) < thres) {
                zebra_cross_flag1[zebra_cross_flag1_num]++;
            }
        } else { // 偶数黑，奇数白
            if (zebra_cross_flag1_num % 2 == 0 && AT_IMAGE(img_raw, x, py) > thres) {
                zebra_cross_flag1[++zebra_cross_flag1_num]++;
            } else if (zebra_cross_flag1_num % 2 == 0 && AT_IMAGE(img_raw, x, py) < thres) {
                zebra_cross_flag1[zebra_cross_flag1_num]++;
            } else if (zebra_cross_flag1_num % 2 == 1 && AT_IMAGE(img_raw, x, py) > thres) {
                zebra_cross_flag1[zebra_cross_flag1_num]++;
            } else if (zebra_cross_flag1_num % 2 == 1 && AT_IMAGE(img_raw, x, py) < thres) {
                zebra_cross_flag1[++zebra_cross_flag1_num]++;
            }
        }
    }

// ====================== 极简版：总合格段数 >6 就算斑马线 ======================
int total_valid = 0;

// 统计左边合格段数
for (int i = 1; i < zebra_cross_flag0_num - 1; i++) {
    int w = zebra_cross_flag0[i];
    int diff = abs(zebra_cross_flag0[i+1] - w);
    if (w >= 5 && w < 40 && diff < 10) {
        total_valid++;
    }

}

// 统计右边合格段数
for (int i = 1; i < zebra_cross_flag1_num - 1; i++) {
    int w = zebra_cross_flag1[i];
    int diff = abs(zebra_cross_flag1[i+1] - w);
    if (w >= 5 && w < 40 && diff < 10) {
        total_valid++;
    }
}
//printf("ttl:%d\n",total_valid);
// 只要总合格段数 >6 就返回 true（最宽松！）
return total_valid > 10;
// ==========================================================================
}
void check_circle() {
    if (circle_type != CIRCLE_NONE || circle_entry_is_blocked()) {
        return;
    }

    // 非圆环模式下，单边L角点, 单边长直道
    if (circle_type == CIRCLE_NONE && Lpt0_found && !Lpt1_found) {
        if(Lpt0_rpts0s_id<rpts0s_num*0.4){
        circle_type = CIRCLE_LEFT_BEGIN;
        start_circle_enter_cooldown();
        std::cout << "begin" << std::endl;
        }
    }
}

void run_circle() {

    // 左环开始，寻外直道右线
    if (circle_type == CIRCLE_LEFT_BEGIN) {
        track_type = TRACK_RIGHT;
        if(std::chrono::steady_clock::now() - last_circle_enter_time >= kCircleEnterCooldown){
            circle_type=CIRCLE_NONE;
        }
        //先丢左线后有线
        if (rpts0s_num < 0.2 / sample_dist) { none_left_line++; }
        if (rpts0s_num > 0.5 / sample_dist && none_left_line > 2) {
            have_left_line++;
            if (have_left_line > 1) {
                circle_type = CIRCLE_LEFT_IN;
                none_left_line = 0;
                have_left_line = 0;
                circle_encoder = 0;
            }
        }
    }
    //入环，寻内圆左线
    else if (circle_type == CIRCLE_LEFT_IN) {
        track_type = TRACK_LEFT;

        //先丢右线后有线
        if (rpts1s_num < 0.2 / sample_dist) { none_right_line++; }
        if (rpts1s_num > 0.5 / sample_dist && none_right_line > 2) {
            have_right_line++;
            if (have_right_line > 1) {
                circle_type = CIRCLE_LEFT_RUNNING;
                none_right_line = 0;
                have_right_line = 0;
                circle_encoder = 0;
            }
        }
    //todo：改成先丢线后有线感觉可以
        //编码器打表过1/4圆   应修正为右线为转弯无拐点
     //   if (rpts0s_num < 0.1 / sample_dist ||
         //   current_encoder - circle_encoder >= ENCODER_PER_METER * (3.14 * 1 / 2)) { circle_type = CIRCLE_LEFT_RUNNING; }
    }

    //正常巡线，寻外圆右线
    else if (circle_type == CIRCLE_LEFT_RUNNING) {
        track_type = TRACK_RIGHT;

        if (Lpt1_found) rpts1s_num = rptsc1_num = Lpt1_rpts1s_id;
        //外环拐点(右L点)
        if ((Lpt1_found && Lpt1_rpts1s_id < 0.4 / sample_dist)||std::chrono::steady_clock::now() - last_circle_enter_time >=kCircleOUTCooldown) {
            circle_type = CIRCLE_LEFT_OUT;
        }
    }
    //出环，寻内圆
    else if (circle_type == CIRCLE_LEFT_OUT) {
        track_type = TRACK_LEFT;

        //右线为长直道.，这个地方要改
                //先丢右线后有线
        if (rpts1s_num < 0.2 / sample_dist) { none_right_line++; }
        if (rpts1s_num > 0.5 / sample_dist && none_right_line > 2) {
            have_right_line++;
            if (have_right_line > 1) {
                circle_type = CIRCLE_LEFT_END;
                none_right_line = 0;
                have_right_line = 0;
                circle_encoder = 0;
            }
        }
         
    }
    //走过圆环，寻右线
    else if (circle_type == CIRCLE_LEFT_END) {
        track_type = TRACK_RIGHT;

        //左线先丢后有
        if (rpts0s_num < 0.2 / sample_dist) { none_left_line++; }
        if (rpts0s_num > 1.0 / sample_dist && none_left_line > 3) {
            circle_type = CIRCLE_NONE;
            none_left_line = 0;
        }
    }
    //右环控制，前期寻左直道
    else if (circle_type == CIRCLE_RIGHT_BEGIN) {
        track_type = TRACK_LEFT;
                if(std::chrono::steady_clock::now() - last_circle_enter_time >= kCircleEnterCooldown){
            circle_type=CIRCLE_NONE;
        }
        //先丢右线后有线
        if (rpts1s_num < 0.2 / sample_dist) { none_right_line++; }
        if (rpts1s_num > 0.5 / sample_dist && none_right_line > 2) {//这里可以调
            have_right_line++;
            if (have_right_line > 1) {
                circle_type = CIRCLE_RIGHT_IN;
                none_right_line = 0;
                have_right_line = 0;
                circle_encoder = 0;
            }
        }
    }
    //入右环，寻右内圆环
    else if (circle_type == CIRCLE_RIGHT_IN) {
        track_type = TRACK_RIGHT;
        //先丢左线后有线
        if (rpts0s_num < 0.2 / sample_dist) { none_left_line++; }
        if (rpts0s_num > 0.5 / sample_dist && none_left_line > 2) {
            have_left_line++;
            if (have_left_line > 1) {
                circle_type = CIRCLE_RIGHT_RUNNING;
                none_left_line = 0;
                have_left_line = 0;
                circle_encoder = 0;
            }
        }
        //编码器打表过1/4圆   应修正为左线为转弯无拐点
       // if (rpts1s_num < 0.1 / sample_dist ||
          //  current_encoder - circle_encoder >= ENCODER_PER_METER * (3.14 * 1 / 2)) { circle_type = CIRCLE_RIGHT_RUNNING; }

    }
    //正常巡线，寻外圆左线
    else if (circle_type == CIRCLE_RIGHT_RUNNING) {
        track_type = TRACK_LEFT;

        //外环存在拐点,可再加拐点距离判据(左L点)s
        if (Lpt0_found) rpts0s_num = rptsc0_num = Lpt0_rpts0s_id;
        if ((Lpt0_found && Lpt0_rpts0s_id < 0.4 / sample_dist)||std::chrono::steady_clock::now() - last_circle_enter_time >=kCircleOUTCooldown) {
            circle_type = CIRCLE_RIGHT_OUT;
        }
    }
    //出环，寻内圆
    else if (circle_type == CIRCLE_RIGHT_OUT) {
        track_type = TRACK_RIGHT;
        //先丢左线后有线
        if (rpts0s_num < 0.2 / sample_dist) { none_left_line++; }
        if (rpts0s_num > 0.5 / sample_dist && none_left_line > 2) {
            have_left_line++;
            if (have_left_line > 1) {
                 circle_type = CIRCLE_RIGHT_END;
                none_left_line = 0;
                have_left_line = 0;
                circle_encoder = 0;
            }
        }
        //左长度加倾斜角度  应修正左右线找到且为直线
        //if((rpts1s_num >100 && !Lpt1_found))  {have_right_line++;}
    }
        //走过圆环，寻左线
    else if (circle_type == CIRCLE_RIGHT_END) {
        track_type = TRACK_LEFT;

        //左线先丢后有
        if (rpts1s_num < 0.2 / sample_dist) { none_right_line++; }
        if (rpts1s_num > 1.0 / sample_dist && none_right_line > 2) {
            circle_type = CIRCLE_NONE;
            none_right_line = 0;
        }
    }
}

// 绘制圆环模式下的调试图像
void draw_circle() {

}


// 编码器值，用于防止一些重复触发等。
int64_t cross_encoder;

bool far_Lpt0_found, far_Lpt1_found;
int far_Lpt0_rpts0s_id, far_Lpt1_rpts1s_id;


// 以下定义为十字寻远线设定
int far_ipts0[FAR_POINTS_MAX_LEN][2];
int far_ipts1[FAR_POINTS_MAX_LEN][2];
int far_ipts0_num, far_ipts1_num;

float far_rpts0[FAR_POINTS_MAX_LEN][2];
float far_rpts1[FAR_POINTS_MAX_LEN][2];
int far_rpts0_num, far_rpts1_num;

float far_rpts0b[FAR_POINTS_MAX_LEN][2];
float far_rpts1b[FAR_POINTS_MAX_LEN][2];
int far_rpts0b_num, far_rpts1b_num;

float far_rpts0s[FAR_POINTS_MAX_LEN][2];
float far_rpts1s[FAR_POINTS_MAX_LEN][2];
int far_rpts0s_num, far_rpts1s_num;

float far_rpts0a[FAR_POINTS_MAX_LEN];
float far_rpts1a[FAR_POINTS_MAX_LEN];
int far_rpts0a_num, far_rpts1a_num;

float far_rpts0an[FAR_POINTS_MAX_LEN];
float far_rpts1an[FAR_POINTS_MAX_LEN];
int far_rpts0an_num, far_rpts1an_num;

int not_have_line = 0;

int far_x1 = 86, far_x2 = 280, far_y1, far_y2;
void find_corners() {
    // 识别L拐点
     Lpt0_found = Lpt1_found = false;
    is_straight0 = rpts0s_num > 0.5 / sample_dist;
    is_straight1 = rpts1s_num > 0.5 / sample_dist;
    for (int i = rpts0s_num*0.15; i < rpts0s_num*0.5; i++) {//不要找的太远
        if (rpts0an[i] == 0) continue;
        int im1 = clip(i - (int) round(angle_dist / sample_dist), 0, rpts0s_num - 1);
        int ip1 = clip(i + (int) round(angle_dist / sample_dist), 0, rpts0s_num - 1);
        float conf = fabs(rpts0a[i]) - (fabs(rpts0a[im1]) + fabs(rpts0a[ip1])) / 2;

        //L角点阈值
        if (Lpt0_found == false && 70. / 180. * PI < conf && conf < 140. / 180. * PI && i < 0.8 / sample_dist) {
            Lpt0_rpts0s_id = i;
            Lpt0_found = true;
        }
        //长直道阈值
        if (conf > 15. / 180. * PI && 0.1 / sample_dist <i < 0.4 / sample_dist) is_straight0 = false;
        if ( Lpt0_found == true && is_straight0 == false) break;
    }
    for (int i = rpts1s_num*0.15; i < rpts1s_num*0.8; i++) {
        if (rpts1an[i] == 0) continue;
        int im1 = clip(i - (int) round(angle_dist / sample_dist), 0, rpts1s_num - 1);
        int ip1 = clip(i + (int) round(angle_dist / sample_dist), 0, rpts1s_num - 1);
        float conf = fabs(rpts1a[i]) - (fabs(rpts1a[im1]) + fabs(rpts1a[ip1])) / 2;
        if (Lpt1_found == false && 70. / 180. * PI < conf && conf < 140. / 180. * PI && i < 0.8 / sample_dist) {
            Lpt1_rpts1s_id = i;
            Lpt1_found = true;
        }

        if (conf > 15. / 180. * PI && 0.1 / sample_dist <i < 0.4 / sample_dist) is_straight1 = false;

        if ( Lpt1_found == true && is_straight1 == false) break;
    }
    // L点二次检查，车库模式不检查, 依据L角点距离及角点后张开特性，暂未实现车库
    if (1) {
        if (Lpt0_found && Lpt1_found) {
            float dx = rpts0s[Lpt0_rpts0s_id][0] - rpts1s[Lpt1_rpts1s_id][0];
            float dy = rpts0s[Lpt0_rpts0s_id][1] - rpts1s[Lpt1_rpts1s_id][1];
            float dn = sqrtf(dx * dx + dy * dy);
            if (fabs(dn - 0.45 * pixel_per_meter) < 0.15 * pixel_per_meter) {
                float dwx = rpts0s[clip(Lpt0_rpts0s_id + 50, 0, rpts0s_num - 1)][0] -
                            rpts1s[clip(Lpt1_rpts1s_id + 50, 0, rpts1s_num - 1)][0];
                float dwy = rpts0s[clip(Lpt0_rpts0s_id + 50, 0, rpts0s_num - 1)][1] -
                            rpts1s[clip(Lpt1_rpts1s_id + 50, 0, rpts1s_num - 1)][1];
                float dwn = sqrtf(dwx * dwx + dwy * dwy);
                if (!(dwn > 0.7 * pixel_per_meter &&
                      rpts0s[clip(Lpt0_rpts0s_id + 50, 0, rpts0s_num - 1)][0] < rpts0s[Lpt0_rpts0s_id][0] &&
                      rpts1s[clip(Lpt1_rpts1s_id + 50, 0, rpts1s_num - 1)][0] > rpts1s[Lpt1_rpts1s_id][0])) {
                    Lpt0_found = Lpt1_found = false;
                }
            } else {
                Lpt0_found = Lpt1_found = false;
            }
        }
    }
}

//双L角点,切十字模式
void check_cross() {
    bool Xfound = Lpt0_found && Lpt1_found;
    if (cross_type == CROSS_NONE && Xfound) {cross_type = CROSS_BEGIN;
        std::cout<<"cross"<<std::endl;
    }
}

void run_cross() {
    bool Xfound = Lpt0_found && Lpt1_found;
    int64_t current_encoder = 0 ;//get_total_encoder();//总里程
    float Lpt0y = rpts0s[Lpt0_rpts0s_id][1];
    float Lpt1y = rpts1s[Lpt1_rpts1s_id][1];
    //检测到十字，先按照近线走
    if (cross_type == CROSS_BEGIN) {
        if (Lpt0_found) {
            rptsc0_num = rpts0s_num = Lpt0_rpts0s_id;
        }
        if (Lpt1_found) {
            rptsc1_num = rpts1s_num = Lpt1_rpts1s_id;
        }

        //aim_distance = 0.4;
        //近角点过少，进入远线控制
        if ((Xfound && (Lpt0_rpts0s_id < 0.03 / sample_dist || Lpt1_rpts1s_id < 0.03 / sample_dist))|| (rpts1_num <10 && rpts0_num<10)) {
            cross_type = CROSS_IN;
            std::cout<<"in"<<std::endl;
            cross_encoder = current_encoder;
        }
    }
        //远线控制进十字,begin_y渐变靠近防丢线
    else if (cross_type == CROSS_IN) {
        //寻远线,算法与近线相同
        cross_farline();

        if (rpts1s_num < 5 && rpts0s_num < 5) { not_have_line++; }
        if (not_have_line > 2 && rpts1s_num > 20 && rpts0s_num > 20) {
            cross_type = CROSS_NONE;
            start_cross_cooldown();
            not_have_line = 0;
        }
        if (far_Lpt1_found) { track_type = TRACK_RIGHT; }
        else if (far_Lpt0_found) { track_type = TRACK_LEFT; }
        else if (not_have_line > 0 && rpts1s_num < 5) { track_type = TRACK_RIGHT; }
        else if (not_have_line > 0 && rpts0s_num < 5) { track_type = TRACK_LEFT; }

    }
}

// 绘制十字模式下的调试图像
void draw_cross() {
    if (cross_type == CROSS_IN && line_show_sample) {
        for (int i = 0; i < far_rpts0s_num; i++) {
            AT_IMAGE(&img_line, clip(far_rpts0s[i][0], 0, img_line.width - 1), clip(far_rpts0s[i][1], 0, img_line.height - 1)) = 100;
        }
        for (int i = 0; i < far_rpts1s_num; i++) {
            AT_IMAGE(&img_line, clip(far_rpts1s[i][0], 0, img_line.width - 1), clip(far_rpts1s[i][1], 0, img_line.height - 1)) = 100;
        }

        if (far_Lpt0_found) {
            draw_o(&img_line, far_rpts0s[far_Lpt0_rpts0s_id][0], far_rpts0s[far_Lpt0_rpts0s_id][1], 3, 255);
        }
        if (far_Lpt1_found) {
            draw_o(&img_line, far_rpts1s[far_Lpt1_rpts1s_id][0], far_rpts1s[far_Lpt1_rpts1s_id][1], 3, 255);
        }

        draw_o(&img_line, clip(mapx[(int) begin_y][far_x1], 0, img_line.width - 1), clip(mapy[(int) begin_y][far_x1], 0, img_line.height - 1), 3, 255);
        draw_o(&img_line, clip(mapx[(int) begin_y][far_x2], 0, img_line.width - 1), clip(mapy[(int) begin_y][far_x1], 0, img_line.height - 1), 3, 255);
        draw_o(&img_line, clip(mapx[far_y1][far_x1], 0, img_line.width - 1), clip(mapy[far_y1][far_x1], 0, img_line.height - 1), 3, 255);
        draw_o(&img_line, clip(mapx[far_y2][far_x2], 0, img_line.width - 1), clip(mapy[far_y2][far_x2], 0, img_line.height - 1), 3, 255);

        /*
        for(int y1=begin_y; y1>far_y1; y1--){
            AT_IMAGE(&img_raw, far_x1, y1) = 128;
        }
        for(int y2=begin_y; y2>far_y2; y2--){
            AT_IMAGE(&img_raw, far_x2, y2) = 128;
        }
        */
    }
}

void cross_farline() {
    int cross_width = 4;
//    far_x1 = cross_width, far_x2 = img_raw.width -cross_width;
    far_y1 = 0, far_y2 = 0;


    int x1 = img_raw.width / 2 - begin_x, y1 = begin_y;
    bool white_found = false;
    far_ipts0_num = sizeof(far_ipts0) / sizeof(far_ipts0[0]);

    //在begin_y向两边找黑线
//    for(;x1>cross_width*2; x1--) 
//    {
//      if(AT_IMAGE(&img_raw, x1-1, y1) < low_thres) {
//        far_x1 = x1 - cross_width;
//        break;
//      }   
//    }
    //全白  far_x1 = 0,从边界找
    for (; y1 > 0; y1--) {
        //先黑后白，先找white
        if (AT_IMAGE(&img_raw, far_x1, y1) >= thres) { white_found = true; }
        if (AT_IMAGE(&img_raw, far_x1, y1) < thres && (white_found || far_x1 == cross_width)) {
            far_y1 = y1;
            break;
        }
    }

    //从找到角点位置开始寻找
    if (AT_IMAGE(&img_raw, far_x1, far_y1 + 1) >= thres)
        findline_lefthand_adaptive(&img_raw, block_size, clip_value, far_x1, far_y1 + 1, far_ipts0, &far_ipts0_num);
    else far_ipts0_num = 0;

    int x2 = img_raw.width / 2 + begin_x, y2 = 200;
    white_found = false;
    far_ipts1_num = sizeof(far_ipts1) / sizeof(far_ipts1[0]);

    //在begin_y向两边找黑线
//    for(;x2<img_raw.width-cross_width*2; x2++) 
//    {
//      if(AT_IMAGE(&img_raw, x2+1, y2) < low_thres) {
//        far_x2 = x2 + cross_width;
//        break;
//      }   
//    }
    //全白  far_x2 = 0,从边界找
    for (; y2 > 0; y2--) {
        //先黑后白，先找white
        if (AT_IMAGE(&img_raw, far_x2, y2) >= thres) { white_found = true; }
        if (AT_IMAGE(&img_raw, far_x2, y2) < thres && (white_found || far_x2 == img_raw.width - cross_width)) {
            far_y2 = y2;
            break;
        }
    }

    //从找到角点位置开始寻找
    if (AT_IMAGE(&img_raw, far_x2, far_y2 + 1) >= thres)
        findline_righthand_adaptive(&img_raw, block_size, clip_value, far_x2, far_y2 + 1, far_ipts1, &far_ipts1_num);
    else far_ipts1_num = 0;


    // 去畸变+透视变换
    for (int i = 0; i < far_ipts0_num; i++) {
        far_rpts0[i][0] = mapx[far_ipts0[i][1]][far_ipts0[i][0]];
        far_rpts0[i][1] = mapy[far_ipts0[i][1]][far_ipts0[i][0]];
    }
    far_rpts0_num = far_ipts0_num;
    for (int i = 0; i < far_ipts1_num; i++) {
        far_rpts1[i][0] = mapx[far_ipts1[i][1]][far_ipts1[i][0]];
        far_rpts1[i][1] = mapy[far_ipts1[i][1]][far_ipts1[i][0]];
    }
    far_rpts1_num = far_ipts1_num;


    // 边线滤波
    blur_points(far_rpts0, far_rpts0_num, far_rpts0b, (int) round(line_blur_kernel));
    far_rpts0b_num = far_rpts0_num;
    blur_points(far_rpts1, far_rpts1_num, far_rpts1b, (int) round(line_blur_kernel));
    far_rpts1b_num = far_rpts1_num;

    // 边线等距采样
    far_rpts0s_num = sizeof(far_rpts0s) / sizeof(far_rpts0s[0]);
    resample_points(far_rpts0b, far_rpts0b_num, far_rpts0s, &far_rpts0s_num, sample_dist * pixel_per_meter);
    far_rpts1s_num = sizeof(far_rpts1s) / sizeof(far_rpts1s[0]);
    resample_points(far_rpts1b, far_rpts1b_num, far_rpts1s, &far_rpts1s_num, sample_dist * pixel_per_meter);

    // 边线局部角度变化率
    local_angle_points(far_rpts0s, far_rpts0s_num, far_rpts0a, (int) round(angle_dist / sample_dist));
    far_rpts0a_num = far_rpts0s_num;
    local_angle_points(far_rpts1s, far_rpts1s_num, far_rpts1a, (int) round(angle_dist / sample_dist));
    far_rpts1a_num = far_rpts1s_num;

    // 角度变化率非极大抑制
    nms_angle(far_rpts0a, far_rpts0a_num, far_rpts0an, (int) round(angle_dist / sample_dist) * 2 + 1);
    far_rpts0an_num = far_rpts0a_num;
    nms_angle(far_rpts1a, far_rpts1a_num, far_rpts1an, (int) round(angle_dist / sample_dist) * 2 + 1);
    far_rpts1an_num = far_rpts1a_num;

    // 找远线上的L角点
    far_Lpt0_found = far_Lpt1_found = false;
    for (int i = 0; i < MIN(far_rpts0s_num, 80); i++) {
        if (far_rpts0an[i] == 0) continue;
        int im1 = clip(i - (int) round(angle_dist / sample_dist), 0, far_rpts0s_num - 1);
        int ip1 = clip(i + (int) round(angle_dist / sample_dist), 0, far_rpts0s_num - 1);
        float conf = fabs(far_rpts0a[i]) - (fabs(far_rpts0a[im1]) + fabs(far_rpts0a[ip1])) / 2;
        if (70. / 180. * PI < conf && conf < 110. / 180. * PI && i < 100) {
            far_Lpt0_rpts0s_id = i;
            far_Lpt0_found = true;
            break;
        }
    }
    for (int i = 0; i < MIN(far_rpts1s_num, 80); i++) {
        if (far_rpts1an[i] == 0) continue;
        int im1 = clip(i - (int) round(angle_dist / sample_dist), 0, far_rpts1s_num - 1);
        int ip1 = clip(i + (int) round(angle_dist / sample_dist), 0, far_rpts1s_num - 1);
        float conf = fabs(far_rpts1a[i]) - (fabs(far_rpts1a[im1]) + fabs(far_rpts1a[ip1])) / 2;

        if (70. / 180. * PI < conf && conf < 110. / 180. * PI && i < 100) {
            far_Lpt1_rpts1s_id = i;
            far_Lpt1_found = true;
            break;
        }
    }
}