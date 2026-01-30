#include "LQ_demo.hpp"

Mat frame, resized_pic;
/********************************************************************************
 * @brief   YOLO 测试函数
 * @param   none.
 * /*
    使用YOLO_Detecion函数进行图像检测
    1、修改#define Category_NUM 5        为你的分类数量
    2、修改#define DATA_LEN 10           为你的数据长度，计算方法为：Category_NUM + 5
    3、修改#define img_size 64           为你的输入图片尺寸
    4、修改class_name数组为你的分类名称
    5、修改Obtain_high_confidence函数中的参数：
    float conf = 0.75          为你的高置信度阈值
    int len_data = 10          为你的数据长度，计算方法同DATA_LEN
    6、修改out_name数组为你的输出层名称
 * @return  none.
 ********************************************************************************/
void YOLO_Demo()
{
    // 初始化YOLO模型
    YOLO_Init();
    // 打开USB摄像头
    VideoCapture cap(0);
    if (!cap.isOpened())
    {
        cerr << "Could not open the camera" << endl;
    }
    /***********************************摄像头初始化******************************* */
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 160);                                    // 设置分辨率宽度为160
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 120);                                   // 设置分辨率高度为120
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G')); // 设置视频编码格式为MJPG
    cap.set(cv::CAP_PROP_FPS, 120);                                            // 设置帧率为120FPS
    /***********************************摄像头初始化******************************* */
    while (1)
    {
        cap >> frame;
        if (frame.empty())
        {
            cerr << "Could not read a frame from the camera" << endl;
            break;
        }
        /******************************图像初步处理*******************************/
        // 旋转图像180度
        rotate(frame, frame, ROTATE_180);
        // 缩放图像
        resize(frame, resized_pic, Size(64, 64));
        /******************************图像初步处理*******************************/
        YOLO_Detecion(resized_pic);
    }
}