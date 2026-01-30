#include "LQ_demo.hpp"

/********************************************************************************
 * @brief   TFT 屏幕驱动测试程序
 * @param   none.
 * @return  none.
 * @note    none.
 ********************************************************************************/
void TFT_Dri_Demo()
{
    TFTSPI_dri_init(1);

    TFTSPI_dri_draw_line(10, 5, 10, 50, u16RED);
    TFTSPI_dri_draw_line(10, 5, 100, 5, u16RED);
    TFTSPI_dri_draw_line(10, 5, 100, 50, u16RED);
    TFTSPI_dri_draw_circle(10, 10, 15, u16RED);
    TFTSPI_dri_draw_rectangle(1, 60, 120, 120, u16RED);
    TFTSPI_dri_fill_area(20, 40, 80, 80, u16RED);
    TFTSPI_dir_P6X8Str(2, 2, "LongQiu", u16BLACK, u16GREEN);
    TFTSPI_dir_P8X8Str(2, 3, "LongQiu", u16BLACK, u16GREEN);
    TFTSPI_dir_P8X16Str(2, 4, "LongQiu", u16BLACK, u16GREEN);

    sleep(1);
    char txt[32];
    unsigned short count = 1;

    while(1)
    {
        memset(txt, 0, sizeof(txt));
        sprintf(txt, "variate:%05d", count);
        TFTSPI_dir_P8X16Str(0, 6, txt, u16RED, u16BLUE);
        usleep(500000);
        count++;
    }
}
