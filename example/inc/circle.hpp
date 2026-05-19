#ifndef CIRCLE_HPP
#define CIRCLE_HPP

enum cross_type_e {
    CROSS_NONE = 0,     // 非十字模式
    CROSS_BEGIN,        // 找到左右两个L角点
    CROSS_IN,           // 两个L角点很近，即进入十字内部(此时切换远线控制)
    CROSS_NUM,
};
enum circle_type_e {
    CIRCLE_NONE = 0,                            // 非圆环模式
    CIRCLE_LEFT_BEGIN, CIRCLE_RIGHT_BEGIN,      // 圆环开始，识别到单侧L角点另一侧长直道。
    CIRCLE_LEFT_IN, CIRCLE_RIGHT_IN,            // 圆环进入，即走到一侧直道，一侧圆环的位置。
    CIRCLE_LEFT_RUNNING, CIRCLE_RIGHT_RUNNING,  // 圆环内部。
    CIRCLE_LEFT_OUT, CIRCLE_RIGHT_OUT,          // 准备出圆环，即识别到出环处的L角点。
    CIRCLE_LEFT_END, CIRCLE_RIGHT_END,          // 圆环结束，即再次走到单侧直道的位置。
    CIRCLE_NUM,                                 //
};

extern const char *circle_type_name[CIRCLE_NUM];




void check_circle();

void run_circle();

void draw_circle();
extern enum circle_type_e circle_type;
extern enum cross_type_e cross_type;

extern const char *cross_type_name[CROSS_NUM];

void find_corners();
void check_cross();

void run_cross();

void draw_cross();

void cross_farline();

#endif // CORSS_H