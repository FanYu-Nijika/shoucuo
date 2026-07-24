#ifndef CC_CAR_OVERLAY_H
#define CC_CAR_OVERLAY_H

#include "zf_common_headfile.h"

typedef struct {
    uint16 center_col;
    uint16 center_row;
    uint8 valid;
} car_overlay_t;

void car_overlay_update(car_overlay_t *overlay);
void car_overlay_draw_dashboard(const car_overlay_t *overlay);

#endif
