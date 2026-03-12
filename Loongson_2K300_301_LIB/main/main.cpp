#include "main.hpp"

int main()
{
    // lq_ips20_drv_init(0);
    // lq_ips20_drv_cls(U16BLUE);
    lq_camera_ex camera(160, 120, 30, LQ_CAMERA_0CPU_MJPG);

    cv::Mat img;
    printf("width: %d, height: %d, fps: %d\n", camera.get_camera_width(), camera.get_camera_height(), camera.get_camera_fps());

    while (1)
    {
        // camera.get_frame(img, true);
        if (!camera.get_frame(img, true)) {
            break;
        }
        // lq_ips20_drv_road_color(0, 0, img);
        // sleep(1);
    }

    return 0;
}
