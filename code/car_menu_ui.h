#ifndef CC_CAR_MENU_UI_H
#define CC_CAR_MENU_UI_H

#include <stdint.h>

#include "car_menu_app.h"
#include "image.h"

#define CC_CAR_UI_KEY_COUNT (7U)

/* UI 状态只保存页面、选中项和刷新节拍；屏幕绘制在主循环中完成，不放入中断。 */
typedef struct {
    uint16_t current_page;
    uint16_t selected_item;
    uint16_t scroll_item;
    uint16_t editing_data_id;
    cc_menu_value_t editing_value;
    uint8_t editing;
    uint8_t dirty;
    uint8_t dashboard_static_drawn;
    uint8_t long_latched[CC_CAR_UI_KEY_COUNT];
    uint8_t previous_key_mask;
    uint32_t last_repeat_ms[CC_CAR_UI_KEY_COUNT];
    uint32_t key_pressed_ms[CC_CAR_UI_KEY_COUNT];
    uint32_t last_telemetry_draw_ms;
    uint32_t last_camera_draw_ms;
    uint32_t test_until_ms;
} cc_car_menu_ui_t;

void cc_car_menu_ui_init(cc_car_menu_ui_t *ui, const cc_car_menu_app_t *app);
void cc_car_menu_ui_process_input(cc_car_menu_ui_t *ui, cc_car_menu_app_t *app);
void cc_car_menu_ui_task(cc_car_menu_ui_t *ui, cc_car_menu_app_t *app,
                         const cc_image_u8_t *gray_frame,
                         const cc_image_u8_t *binary_frame,
                         uint8_t new_frame);

#endif
