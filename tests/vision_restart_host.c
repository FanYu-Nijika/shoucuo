#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "car_params.h"
#include "car_shared.h"
#include "balance_control.h"
#include "image.h"
#include "longest_white.h"

extern int16 lost_line_cnt;
extern uint8 already_line_lost;
extern uint8 Cross_Flag;
extern uint8 xieru_type;

volatile car_params_t car_params;
volatile balance_state_t balance_state;
volatile uint8_t car_center_stop_request;
volatile uint8_t car_vision_reset_request;

static uint8_t gray_frame[MT9V03X_IMAGE_SIZE];
static uint8_t binary_frame[MT9V03X_IMAGE_SIZE];
static car_result_t result;

void __dsync(void) {}

static void setup_vision(void)
{
    memset((void *)&car_params, 0, sizeof(car_params));
    memset((void *)&balance_state, 0, sizeof(balance_state));
    car_params.running = 1;
    car_params.control_row_near = 75;
    car_params.minimum_line_pixels = 10;
    car_params.threshold = 0;
    car_params.automatic_threshold = 0;
    car_params.curvature_scale = 1.0f;
    car_params.curve_enter_threshold = 0.2f;
    car_params.curve_exit_threshold = 0.1f;
    car_params.lookhead = 100;
    car_params.path_weight_far = 0.4f;
    car_params.path_weight_middle = 1.0f;
    car_params.path_weight_near = 1.5f;
    car_params.cross_enabled = 0;
    car_vision_reset_request = 0;
    car_center_stop_request = 0;
    image_init();
}

static void make_column_frame(int column)
{
    int row;

    memset(gray_frame, 0, sizeof(gray_frame));
    if (column >= 0) {
        for (row = 0; row < MT9V03X_H; row++) gray_frame[row * MT9V03X_W + column] = 200;
    }
    gray_frame[115 * MT9V03X_W + MT9V03X_W / 5] = 200;
}

static void make_curve_frame(void)
{
    int row;
    int column;
    int center;

    memset(gray_frame, 0, sizeof(gray_frame));
    for (row = 0; row < MT9V03X_H; row++) {
        center = 68 + row / 5;
        for (column = center - 24; column <= center + 24; column++) {
            gray_frame[row * MT9V03X_W + column] = 200;
        }
    }
    gray_frame[115 * MT9V03X_W + MT9V03X_W / 5] = 200;
}

static void process_frame(void)
{
    image_deal(0, MT9V03X_H, gray_frame, binary_frame, &result);
}

static void test_stop_request_reacquires_far_shifted_line(void)
{
    setup_vision();
    make_column_frame(50);
    process_frame();
    assert(car_center_stop_request == 0);
    assert(Longest_White_Column_Left[0] == MT9V03X_H);
    assert(Longest_White_Column_Left[1] == 50);

    lost_line_cnt = 1;
    already_line_lost = 1;
    xieru_type = CROSS_SKEW_RIGHT;
    Search_Stop_Line = 87;
    car_params.automatic_threshold = 1;
    car_vision_reset_request = 1;
    make_column_frame(140);
    process_frame();

    assert(car_vision_reset_request == 0);
    assert(car_center_stop_request == 0);
    assert(result.line_valid == 1);
    assert(Longest_White_Column_Left[0] == MT9V03X_H);
    assert(Longest_White_Column_Left[1] == 140);
    assert(lost_line_cnt == LOST_LINE - 1);
    assert(already_line_lost == 0);
    assert(xieru_type == CROSS_SKEW_NONE);
    assert(Search_Stop_Line == MT9V03X_H);
}

static void test_parked_empty_frame_clears_history_before_restart(void)
{
    setup_vision();
    make_column_frame(50);
    process_frame();
    assert(Longest_White_Column_Left[1] == 50);

    car_params.running = 0;
    car_vision_reset_request = 1;
    make_column_frame(50);
    process_frame();
    assert(car_vision_reset_request == 0);
    assert(result.line_valid == 1);
    assert(Longest_White_Column_Left[1] == 50);

    make_column_frame(140);
    process_frame();
    assert(car_vision_reset_request == 0);
    assert(result.line_valid == 1);
    assert(Longest_White_Column_Left[1] == 140);

    make_column_frame(-1);
    process_frame();
    assert(result.line_valid == 0);

    car_params.running = 1;
    make_column_frame(140);
    process_frame();
    assert(result.line_valid == 1);
    assert(Longest_White_Column_Left[1] == 140);
}

static void test_running_search_keeps_its_forty_pixel_window(void)
{
    setup_vision();
    make_column_frame(70);
    process_frame();
    assert(Longest_White_Column_Left[1] == 70);

    make_column_frame(100);
    process_frame();
    assert(result.line_valid == 1);
    assert(Longest_White_Column_Left[1] == 100);

    make_column_frame(150);
    process_frame();
    assert(result.line_valid == 0);
    assert(Longest_White_Column_Left[0] == 0);

    car_vision_reset_request = 1;
    process_frame();
    assert(result.line_valid == 1);
    assert(Longest_White_Column_Left[1] == 150);
}

static void test_image_init_clears_image_state_and_preview_freeze(void)
{
    setup_vision();
    balance_state.encoder_average_count = 500;
    make_column_frame(70);
    process_frame();
    assert(result.line_valid == 1);
    assert(result.preview_row == 63);

    balance_state.encoder_average_count = 0;
    make_curve_frame();
    process_frame();
    assert(result.line_valid == 1);
    assert(result.curvature > car_params.curve_enter_threshold);
    assert(result.preview_row == 63);

    lost_line_cnt = 1;
    already_line_lost = 1;
    xieru_type = CROSS_SKEW_LEFT;
    Search_Stop_Line = 89;
    car_vision_reset_request = 1;
    make_curve_frame();
    process_frame();
    assert(car_vision_reset_request == 0);
    assert(result.line_valid == 1);
    assert(result.curvature > car_params.curve_enter_threshold);
    assert(result.preview_row == car_params.control_row_near);
    assert(lost_line_cnt == LOST_LINE);
    assert(already_line_lost == 0);
    assert(xieru_type == CROSS_SKEW_NONE);

    image_init();
    assert(lost_line_cnt == LOST_LINE);
    assert(already_line_lost == 0);
    assert(xieru_type == CROSS_SKEW_NONE);
    assert(Search_Stop_Line == 0);
    assert(Cross_Flag == 0);
    assert(Cross_Count == 0);
}

static void test_new_line_loss_after_restart_remains_invalid(void)
{
    setup_vision();
    make_column_frame(50);
    process_frame();
    assert(result.line_valid == 1);

    car_vision_reset_request = 1;
    make_column_frame(-1);
    process_frame();
    assert(result.line_valid == 0);
    assert(already_line_lost == 1);
    assert(Longest_White_Column_Left[0] < 10);
}

int main(void)
{
    test_stop_request_reacquires_far_shifted_line();
    test_parked_empty_frame_clears_history_before_restart();
    test_running_search_keeps_its_forty_pixel_window();
    test_image_init_clears_image_state_and_preview_freeze();
    test_new_line_loss_after_restart_remains_invalid();
    puts("PASS vision STOP/FOLLOW restart host regression");
    return 0;
}
