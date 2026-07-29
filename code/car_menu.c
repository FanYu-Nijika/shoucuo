#include "car_menu.h"

#include <string.h>

#include "board_pins.h"
#include "car.h"
#include "car_overlay.h"
#include "car_menu_app.h"
#include "car_menu_port.h"
#include "car_menu_ui.h"
#include "car_params.h"
#include "car_shared.h"
#include "car_uart_stream.h"
#include "image.h"
#include "zf_common_headfile.h"

static cc_car_menu_app_t car_menu_app;
static cc_car_menu_ui_t car_menu_ui;
static uint8 car_menu_ready;
static uint8 car_menu_gray_valid;
static uint8 car_menu_new_frame;
static car_overlay_t car_menu_overlay;

static void car_menu_publish_legacy_result(void)
{
    car_result.line_valid = track_valid;
    car_result.valid_rows = track_valid != 0U ? 1U : 0U;
    car_result.lost_count = car_lost_count;
    car_result.left_edge = best_col;
    car_result.right_edge = best_col;
    car_result.line_width = track_valid != 0U ? 1U : 0U;
    car_result.center_x = (float)best_col;
    car_result.error_pixels = (float)track_error;
    car_result.error_normalized = (float)track_error / (float)(MT9V03X_W / 2U);
    car_result.threshold_used = threshold;
    car_result.strength = track_valid != 0U ? 1.0f : 0.0f;
    car_result.servo_command_us = (int16)(car_servo_duty * 2U);
    car_result.left_command = car_left_command;
    car_result.right_command = car_right_command;
    car_result.frame_sequence = car_frame_sequence;
    car_result.processing_time_us = car_frame_period_ms * 1000U;
}

void car_menu_init(void)
{
    uint8 camera_ready;

    cc_tc264_board_init();
    car_init();
    ips200_set_dir(IPS200_CROSSWISE);
    ips200_init(IPS200_TYPE_SPI);

    car_uart_stream_init();
    cc_car_menu_init(&car_menu_app);
    cc_car_menu_ui_init(&car_menu_ui, &car_menu_app);

    /* Load menu defaults before sending exposure and gain to the camera. */
    camera_ready = cc_tc264_camera_init();
    car_set_camera_ready(camera_ready);
    if (camera_ready != 0U) {
        cc_tc264_camera_set_exposure(car_params.exposure);
        cc_tc264_camera_set_gain(car_params.gain);
    }
    gpio_set_level(BOARD_LED1_PIN, camera_ready != 0U ? GPIO_LOW : GPIO_HIGH);
    gpio_set_level(BOARD_LED2_PIN, GPIO_HIGH);

    car_menu_ready = 1U;
}

void car_menu_frame_accepted(uint32_t frame_period_ms)
{
    if (car_menu_ready == 0U) return;
    car_menu_gray_valid = 1U;
    cc_car_menu_note_frame(&car_menu_app, frame_period_ms);
}

void car_menu_result_accepted(void)
{
    if (car_menu_ready == 0U) return;
    /* Binary data is written by CPU1, so redraw both camera panes only after
     * CPU0 has consumed the complete result. */
    car_menu_new_frame = 1U;
}

void car_menu_task(void)
{
    uint32 previous_camera_draw_ms;
    uint8 previous_dashboard_static;

    if (car_menu_ready == 0U) return;

    if (car_frame_ready == 0U && car_result_ready == 0U) car_menu_publish_legacy_result();
    car_overlay_update(&car_menu_overlay);
    cc_car_menu_ui_process_input(&car_menu_ui, &car_menu_app);
    cc_car_menu_process_events(&car_menu_app);

    if (car_menu_app.safety_state == CC_CAR_SAFE_RUNNING && car_running == 0U) {
        car_params.running = 0U;
        car_menu_app.safety_state = CC_CAR_SAFE_LOCKED;
        car_menu_app.last_error = CC_CAR_ERROR_LINE_LOST;
    }

    /* CPU1 writes car_result in one pass; read it only while the channel is
     * idle so the menu never displays a half-written telemetry record. */
    if (car_frame_ready == 0U && car_result_ready == 0U) {
        cc_car_menu_sync_telemetry(&car_menu_app);
    }
    previous_camera_draw_ms = car_menu_ui.last_camera_draw_ms;
    previous_dashboard_static = car_menu_ui.dashboard_static_drawn;
    car_menu_display();
    if (car_menu_ui.current_page == CC_CAR_PAGE_DASHBOARD &&
        (car_menu_ui.last_camera_draw_ms != previous_camera_draw_ms ||
         (previous_dashboard_static == 0U && car_menu_ui.dashboard_static_drawn != 0U))) {
        car_overlay_draw_dashboard(&car_menu_overlay);
    }
    car_menu_new_frame = 0U;
}

void car_menu_display(void)
{
    cc_image_u8_t gray_frame;
    cc_image_u8_t binary_frame;

    if (car_menu_ready == 0U) return;
    gray_frame = cc_image_u8_make(car_menu_gray_valid != 0U ? &car_gray_frame[0][0] : 0,
                                  CAR_IMAGE_WIDTH, CAR_IMAGE_HEIGHT, CAR_IMAGE_WIDTH);
    binary_frame = cc_image_u8_make(&image_buffer[0][0], MT9V03X_W, MT9V03X_H, MT9V03X_W);
    cc_car_menu_ui_task(&car_menu_ui, &car_menu_app, &gray_frame, &binary_frame, car_menu_new_frame);
}
