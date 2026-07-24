#include "car_overlay.h"

#include "image.h"

void car_overlay_update(car_overlay_t *overlay)
{
    if (overlay == 0) return;
    overlay->center_col = best_col;
    overlay->center_row = best_row;
    overlay->valid = track_valid;
}

void car_overlay_draw_dashboard(const car_overlay_t *overlay)
{
    uint16 x;
    uint16 y;

    if (overlay == 0 || overlay->valid == 0U) return;
    x = 6U + (uint16)((uint32)overlay->center_col * 148U / MT9V03X_W);
    y = 46U + (uint16)((uint32)overlay->center_row * 94U / MT9V03X_H);
    ips200_draw_point(x, y, RGB565_RED);
}
