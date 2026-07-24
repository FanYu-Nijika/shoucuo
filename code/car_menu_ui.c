#include "car_menu_ui.h"

#include <stdio.h>
#include <string.h>

#include "car_params.h"
#include "car_menu_port.h"
#include "car_shared.h"
#include "car_uart_stream.h"
#include "zf_common_headfile.h"

#define CC_UI_VISIBLE_ROWS          (8U)
#define CC_UI_TELEMETRY_PERIOD_MS   (50U)
#define CC_UI_CAMERA_PERIOD_MS      (50U)

enum {
    CC_UI_KEY_UP,
    CC_UI_KEY_DOWN,
    CC_UI_KEY_LEFT,
    CC_UI_KEY_RIGHT,
    CC_UI_KEY_CENTER,
    CC_UI_KEY_AUX1,
    CC_UI_KEY_AUX2,
    CC_UI_KEY_COUNT
};

enum {
    CC_UI_KEY_SHORT_PRESS = 1,
    CC_UI_KEY_LONG_PRESS
};

static const uint16 CC_UI_BG = 0x0861U;
static const uint16 CC_UI_PANEL = 0x10C3U;
static const uint16 CC_UI_PANEL_ALT = 0x1924U;
static const uint16 CC_UI_TEXT = 0xE71CU;
static const uint16 CC_UI_MUTED = 0x8410U;
static const uint16 CC_UI_CYAN = 0x07FFU;
static const uint16 CC_UI_GREEN = 0x07E0U;
static const uint16 CC_UI_YELLOW = 0xFFE0U;
static const uint16 CC_UI_RED = 0xF800U;
static const uint16 CC_UI_BLACK = 0x0000U;

static void cc_ui_back(cc_car_menu_ui_t *ui, cc_car_menu_app_t *app);

static void cc_ui_text(uint16 x, uint16 y, const char *text, uint8_t max_chars, uint16 color, uint16 background)
{
    char buffer[41];
    uint8_t length;

    if (text == 0 || max_chars == 0U) return;
    length = strlen(text);
    if (length > max_chars) length = max_chars;
    if (length > 40U) length = 40U;
    memcpy(buffer, text, length);
    buffer[length] = '\0';
    ips200_set_color(color, background);
    ips200_show_string(x, y, buffer);
}

static void cc_ui_draw_icon(uint16 x, uint16 y, uint8_t icon, uint16 color, uint16 background)
{
    ips200_fill_rect(x, y, 12U, 12U, background);
    if (icon == 0U) {
        ips200_draw_rect(x + 1U, y + 2U, 10U, 8U, color);
        ips200_fill_rect(x + 4U, y + 5U, 4U, 2U, color);
    } else if (icon == 1U) {
        ips200_draw_rect(x + 1U, y + 3U, 10U, 6U, color);
        ips200_fill_rect(x + 3U, y + 1U, 6U, 2U, color);
    } else if (icon == 2U) {
        ips200_draw_line(x + 1U, y + 10U, x + 5U, y + 2U, color);
        ips200_draw_line(x + 5U, y + 2U, x + 10U, y + 10U, color);
    } else if (icon == 3U) {
        ips200_draw_rect(x + 1U, y + 1U, 10U, 10U, color);
        ips200_draw_line(x + 1U, y + 6U, x + 10U, y + 6U, color);
    } else {
        ips200_draw_rect(x + 2U, y + 2U, 8U, 8U, color);
        ips200_fill_rect(x + 5U, y, 2U, 12U, color);
    }
}

static void cc_ui_format_data(const cc_menu_data_t *data, char *buffer, uint8_t size)
{
    if (buffer == 0 || size == 0U) return;
    buffer[0] = '\0';
    if (data == 0) {
        snprintf(buffer, size, "--");
        return;
    }

    switch (data->type) {
        case CC_MENU_DATA_BOOL:
            snprintf(buffer, size, "%s", data->value.boolean != 0U ? "ON" : "OFF");
            break;
        case CC_MENU_DATA_INT32:
            snprintf(buffer, size, "%ld", (long)data->value.int32_value);
            break;
        case CC_MENU_DATA_UINT32:
            snprintf(buffer, size, "%lu", (unsigned long)data->value.uint32_value);
            break;
        case CC_MENU_DATA_FLOAT:
            if (data->decimals >= 2U) snprintf(buffer, size, "%.2f", (double)data->value.float_value);
            else snprintf(buffer, size, "%.1f", (double)data->value.float_value);
            break;
        case CC_MENU_DATA_ENUM:
            if (data->id == CC_CAR_DATA_LEFT_DIRECTION || data->id == CC_CAR_DATA_RIGHT_DIRECTION) {
                snprintf(buffer, size, "%s", data->value.enum_value != 0 ? "FWD" : "REV");
            } else if (data->id == CC_CAR_DATA_SAFETY_STATE) {
                static const char *states[] = {"LOCKED", "ARMING", "RUNNING", "FAULT"};
                int32_t state = data->value.enum_value;
                snprintf(buffer, size, "%s", state >= 0 && state <= 3 ? states[state] : "FAULT");
            } else {
                snprintf(buffer, size, "%ld", (long)data->value.enum_value);
            }
            break;
        default:
            snprintf(buffer, size, "--");
            break;
    }
}

static uint16_t cc_ui_count_before(const cc_menu_t *menu, uint16_t first, uint16_t item)
{
    uint16_t count = 0U;
    uint16_t current = first;

    while (current != CC_MENU_ID_NONE && current != item) {
        count++;
        current = cc_menu_next_sibling(menu, current);
    }
    return count;
}

static void cc_ui_keep_selected_visible(cc_car_menu_ui_t *ui, const cc_menu_t *menu)
{
    uint16_t first = cc_menu_first_child(menu, ui->current_page);
    uint16_t selected_index;
    uint16_t scroll_index;
    uint16_t current;
    uint16_t target_scroll_index;

    if (first == CC_MENU_ID_NONE) return;
    if (ui->scroll_item == CC_MENU_ID_NONE) ui->scroll_item = first;
    selected_index = cc_ui_count_before(menu, first, ui->selected_item);
    scroll_index = cc_ui_count_before(menu, first, ui->scroll_item);

    if (selected_index < scroll_index) {
        ui->scroll_item = ui->selected_item;
    } else if (selected_index >= scroll_index + CC_UI_VISIBLE_ROWS) {
        target_scroll_index = selected_index - CC_UI_VISIBLE_ROWS + 1U;
        current = first;
        while (target_scroll_index > 0U && current != CC_MENU_ID_NONE) {
            current = cc_menu_next_sibling(menu, current);
            target_scroll_index--;
        }
        if (current != CC_MENU_ID_NONE) ui->scroll_item = current;
    }
}

static void cc_ui_draw_header(const cc_car_menu_ui_t *ui, const cc_car_menu_app_t *app)
{
    const cc_menu_node_t *page = cc_menu_find(&app->menu, ui->current_page);
    uint16 status_color = CC_UI_YELLOW;

    ips200_fill_rect(0U, 0U, 320U, 24U, CC_UI_PANEL);
    cc_ui_text(8U, 4U, page != 0 ? page->label : "Camera Car", 20U, CC_UI_TEXT, CC_UI_PANEL);
    if (app->safety_state == CC_CAR_SAFE_RUNNING) status_color = CC_UI_GREEN;
    else if (app->safety_state == CC_CAR_SAFE_FAULT) status_color = CC_UI_RED;
    cc_ui_text(208U, 4U, cc_car_menu_safety_text(app), 8U, status_color, CC_UI_PANEL);
    cc_ui_text(280U, 4U, app->camera_ready != 0U ? "CAM" : "OFF", 3U,
               app->camera_ready != 0U ? CC_UI_GREEN : CC_UI_RED, CC_UI_PANEL);
}

static void cc_ui_draw_footer(const cc_car_menu_ui_t *ui, const cc_car_menu_app_t *app)
{
    ips200_fill_rect(0U, 216U, 320U, 24U, CC_UI_PANEL);
    if (ui->current_page == CC_CAR_PAGE_DASHBOARD) {
        if (app->safety_state == CC_CAR_SAFE_ARMING || app->safety_state == CC_CAR_SAFE_RUNNING) {
            cc_ui_text(8U, 220U, "UART LOCKED WHILE RUNNING", 38U, CC_UI_YELLOW, CC_UI_PANEL);
        } else if (car_uart_stream_is_enabled()) {
            cc_ui_text(8U, 220U, "RAW TX ON  LEFT Stop", 38U, CC_UI_GREEN, CC_UI_PANEL);
        } else {
            cc_ui_text(8U, 220U, "AUX1 Back AUX2 Run RIGHT UART", 38U, CC_UI_MUTED, CC_UI_PANEL);
        }
    } else if (ui->editing != 0U) {
        cc_ui_text(8U, 220U, "UP/DN Adjust RIGHT Save LEFT Cancel", 38U, CC_UI_MUTED, CC_UI_PANEL);
    } else {
        cc_ui_text(8U, 220U, "UP/DN Move RIGHT Enter AUX1 Back", 38U, CC_UI_MUTED, CC_UI_PANEL);
    }
}

static void cc_ui_draw_list(cc_car_menu_ui_t *ui, const cc_car_menu_app_t *app)
{
    const cc_menu_node_t *node;
    const cc_menu_data_t *data;
    uint16_t current;
    uint16_t y;
    uint8_t row;
    char value[18];
    uint16 background;
    uint16 foreground;

    cc_ui_keep_selected_visible(ui, &app->menu);
    ips200_fill_rect(0U, 24U, 320U, 192U, CC_UI_BG);
    current = ui->scroll_item;
    for (row = 0U; row < CC_UI_VISIBLE_ROWS && current != CC_MENU_ID_NONE; row++) {
        node = cc_menu_find(&app->menu, current);
        if (node == 0) break;
        y = 26U + row * 23U;
        background = current == ui->selected_item ? CC_UI_CYAN : (row & 1U) != 0U ? CC_UI_PANEL_ALT : CC_UI_BG;
        foreground = current == ui->selected_item ? CC_UI_BLACK : CC_UI_TEXT;
        ips200_fill_rect(4U, y, 312U, 21U, background);
        cc_ui_draw_icon(10U, y + 4U, node->type, foreground, background);
        cc_ui_text(28U, y + 3U, node->label, 23U, foreground, background);
        if (node->type == CC_MENU_ITEM_PAGE) {
            cc_ui_text(298U, y + 3U, ">", 1U, foreground, background);
        } else if (node->data_id != CC_MENU_ID_NONE) {
            data = cc_menu_data_find(&app->menu, node->data_id);
            cc_ui_format_data(data, value, sizeof(value));
            cc_ui_text(232U, y + 3U, value, 10U, foreground, background);
        }
        current = node->next;
    }
}

static uint8_t cc_ui_dashboard_threshold(const cc_car_menu_app_t *app)
{
    uint8_t threshold = car_result.threshold_used;

    (void)app;
    if (threshold == 0U) threshold = car_params.threshold;
    return threshold == 0U ? 1U : threshold;
}

static void cc_ui_draw_dashboard_images(const cc_car_menu_app_t *app, const cc_image_u8_t *gray_frame,
                                        const cc_image_u8_t *binary_frame)
{
    uint8_t gray_valid = cc_image_u8_is_valid(gray_frame);
    uint8_t binary_valid = cc_image_u8_is_valid(binary_frame);

    if (gray_valid != 0U) {
        ips200_show_gray_image(6U, 46U, gray_frame->data, gray_frame->width, gray_frame->height, 148U, 94U, 0U);
    } else {
        ips200_fill_rect(6U, 46U, 148U, 94U, CC_UI_BLACK);
        cc_ui_text(42U, 84U, "NO RAW FRAME", 14U, CC_UI_RED, CC_UI_BLACK);
    }

    if (binary_valid != 0U) {
        ips200_show_gray_image(166U, 46U, binary_frame->data, binary_frame->width, binary_frame->height, 148U, 94U, 1U);
    } else if (gray_valid != 0U) {
        ips200_show_gray_image(166U, 46U, gray_frame->data, gray_frame->width, gray_frame->height,
                               148U, 94U, cc_ui_dashboard_threshold(app));
    } else {
        ips200_fill_rect(166U, 46U, 148U, 94U, CC_UI_BLACK);
        cc_ui_text(202U, 84U, "NO BIN FRAME", 14U, CC_UI_RED, CC_UI_BLACK);
    }
}

static void cc_ui_draw_dashboard_metrics(const cc_car_menu_app_t *app, uint8_t draw_static)
{
    char value[20];
    const cc_menu_data_t *data;

    if (draw_static != 0U) {
        ips200_fill_rect(4U, 148U, 312U, 64U, CC_UI_PANEL);
        cc_ui_text(10U, 154U, "THR", 4U, CC_UI_MUTED, CC_UI_PANEL);
        cc_ui_text(94U, 154U, "ERR", 4U, CC_UI_MUTED, CC_UI_PANEL);
        cc_ui_text(198U, 154U, "FPS", 4U, CC_UI_MUTED, CC_UI_PANEL);
        cc_ui_text(10U, 182U, "Status", 7U, CC_UI_MUTED, CC_UI_PANEL);
        cc_ui_text(200U, 182U, "Servo", 6U, CC_UI_MUTED, CC_UI_PANEL);
    }

    ips200_fill_rect(42U, 152U, 44U, 18U, CC_UI_PANEL);
    snprintf(value, sizeof(value), "%u", (unsigned int)cc_ui_dashboard_threshold(app));
    cc_ui_text(42U, 154U, value, 5U, CC_UI_TEXT, CC_UI_PANEL);

    ips200_fill_rect(126U, 152U, 64U, 18U, CC_UI_PANEL);
    data = cc_menu_data_find(&app->menu, CC_CAR_DATA_LINE_ERROR);
    cc_ui_format_data(data, value, sizeof(value));
    cc_ui_text(126U, 154U, value, 7U, CC_UI_TEXT, CC_UI_PANEL);

    ips200_fill_rect(230U, 152U, 54U, 18U, CC_UI_PANEL);
    data = cc_menu_data_find(&app->menu, CC_CAR_DATA_FRAME_RATE);
    cc_ui_format_data(data, value, sizeof(value));
    cc_ui_text(230U, 154U, value, 6U, CC_UI_TEXT, CC_UI_PANEL);

    ips200_fill_rect(66U, 180U, 126U, 18U, CC_UI_PANEL);
    cc_ui_text(66U, 182U, cc_car_menu_error_text(app), 15U,
               app->last_error == CC_CAR_ERROR_NONE ? CC_UI_GREEN : CC_UI_YELLOW, CC_UI_PANEL);

    ips200_fill_rect(248U, 180U, 62U, 18U, CC_UI_PANEL);
    data = cc_menu_data_find(&app->menu, CC_CAR_DATA_SERVO_COMMAND_US);
    cc_ui_format_data(data, value, sizeof(value));
    cc_ui_text(248U, 182U, value, 7U, CC_UI_TEXT, CC_UI_PANEL);
}

static void cc_ui_draw_dashboard(const cc_car_menu_app_t *app, const cc_image_u8_t *gray_frame,
                                 const cc_image_u8_t *binary_frame)
{
    ips200_fill_rect(0U, 24U, 320U, 192U, CC_UI_BG);
    cc_ui_text(8U, 27U, "GRAY 0-255", 12U, CC_UI_CYAN, CC_UI_BG);
    cc_ui_text(168U, 27U, "BINARY 0/1", 12U, CC_UI_CYAN, CC_UI_BG);
    ips200_draw_rect(4U, 44U, 152U, 98U, CC_UI_PANEL_ALT);
    ips200_draw_rect(164U, 44U, 152U, 98U, CC_UI_PANEL_ALT);
    cc_ui_draw_dashboard_images(app, gray_frame, binary_frame);
    cc_ui_draw_dashboard_metrics(app, 1U);
}

static void cc_ui_draw_dashboard_delta(const cc_car_menu_app_t *app, const cc_image_u8_t *gray_frame,
                                       const cc_image_u8_t *binary_frame, uint8_t draw_images, uint8_t draw_telemetry)
{
    if (draw_images != 0U) cc_ui_draw_dashboard_images(app, gray_frame, binary_frame);
    if (draw_telemetry != 0U) cc_ui_draw_dashboard_metrics(app, 0U);
}

static void cc_ui_draw_edit(const cc_car_menu_ui_t *ui, const cc_car_menu_app_t *app)
{
    const cc_menu_data_t *data = cc_menu_data_find(&app->menu, ui->editing_data_id);
    cc_menu_data_t display;
    char value[20];

    if (data == 0) return;
    display = *data;
    display.value = ui->editing_value;
    cc_ui_format_data(&display, value, sizeof(value));
    ips200_fill_rect(36U, 66U, 248U, 108U, CC_UI_PANEL);
    ips200_draw_rect(36U, 66U, 248U, 108U, CC_UI_CYAN);
    cc_ui_text(52U, 82U, "EDIT VALUE", 16U, CC_UI_CYAN, CC_UI_PANEL);
    ips200_fill_rect(52U, 112U, 216U, 36U, CC_UI_PANEL_ALT);
    cc_ui_text(116U, 122U, value, 12U, CC_UI_TEXT, CC_UI_PANEL_ALT);
}

static void cc_ui_draw_color_test(void)
{
    ips200_fill_rect(0U, 0U, 64U, 240U, 0xF800U);
    ips200_fill_rect(64U, 0U, 64U, 240U, 0x07E0U);
    ips200_fill_rect(128U, 0U, 64U, 240U, 0x001FU);
    ips200_fill_rect(192U, 0U, 64U, 240U, 0xFFFFU);
    ips200_fill_rect(256U, 0U, 64U, 240U, 0x0000U);
    cc_ui_text(92U, 108U, "LCD COLOR TEST", 18U, CC_UI_BLACK, 0x07E0U);
}

static void cc_ui_select_next(cc_car_menu_ui_t *ui, const cc_menu_t *menu, int8_t direction)
{
    uint16_t next;

    if (direction > 0) {
        next = cc_menu_next_sibling(menu, ui->selected_item);
        if (next == CC_MENU_ID_NONE) next = cc_menu_first_child(menu, ui->current_page);
    } else {
        next = cc_menu_previous_sibling(menu, ui->selected_item);
        if (next == CC_MENU_ID_NONE) {
            next = cc_menu_first_child(menu, ui->current_page);
            while (cc_menu_next_sibling(menu, next) != CC_MENU_ID_NONE) next = cc_menu_next_sibling(menu, next);
        }
    }

    if (next != CC_MENU_ID_NONE) {
        ui->selected_item = next;
        ui->dirty = 1U;
    }
}

static void cc_ui_adjust_value(cc_car_menu_ui_t *ui, const cc_menu_data_t *data, int8_t direction)
{
    if (data == 0) return;

    switch (data->type) {
        case CC_MENU_DATA_BOOL:
            ui->editing_value.boolean = ui->editing_value.boolean == 0U ? 1U : 0U;
            break;
        case CC_MENU_DATA_INT32:
            ui->editing_value.int32_value += direction * data->step.int32_value;
            if (ui->editing_value.int32_value < data->minimum.int32_value) ui->editing_value.int32_value = data->minimum.int32_value;
            if (ui->editing_value.int32_value > data->maximum.int32_value) ui->editing_value.int32_value = data->maximum.int32_value;
            break;
        case CC_MENU_DATA_UINT32:
            if (direction < 0 && ui->editing_value.uint32_value < data->minimum.uint32_value + data->step.uint32_value) {
                ui->editing_value.uint32_value = data->minimum.uint32_value;
            } else if (direction > 0 && ui->editing_value.uint32_value > data->maximum.uint32_value - data->step.uint32_value) {
                ui->editing_value.uint32_value = data->maximum.uint32_value;
            } else if (direction < 0) {
                ui->editing_value.uint32_value -= data->step.uint32_value;
            } else {
                ui->editing_value.uint32_value += data->step.uint32_value;
            }
            break;
        case CC_MENU_DATA_FLOAT:
            ui->editing_value.float_value += direction * data->step.float_value;
            if (ui->editing_value.float_value < data->minimum.float_value) ui->editing_value.float_value = data->minimum.float_value;
            if (ui->editing_value.float_value > data->maximum.float_value) ui->editing_value.float_value = data->maximum.float_value;
            break;
        case CC_MENU_DATA_ENUM:
            ui->editing_value.enum_value += direction * data->step.enum_value;
            if (ui->editing_value.enum_value < data->minimum.enum_value) ui->editing_value.enum_value = data->minimum.enum_value;
            if (ui->editing_value.enum_value > data->maximum.enum_value) ui->editing_value.enum_value = data->maximum.enum_value;
            break;
        default:
            break;
    }
    ui->dirty = 1U;
}

static void cc_ui_enter_selected(cc_car_menu_ui_t *ui, cc_car_menu_app_t *app, uint8_t long_press)
{
    const cc_menu_node_t *node = cc_menu_find(&app->menu, ui->selected_item);
    const cc_menu_data_t *data;

    if (node == 0) return;
    if (node->type == CC_MENU_ITEM_PAGE) {
        ui->current_page = node->id;
        ui->selected_item = cc_menu_first_child(&app->menu, node->id);
        ui->scroll_item = ui->selected_item;
        ui->dirty = 1U;
    } else if (node->type == CC_MENU_ITEM_VALUE) {
        data = cc_menu_data_find(&app->menu, node->data_id);
        if (data != 0 && (data->flags & CC_MENU_DATA_READ_ONLY) == 0U && cc_car_menu_can_edit(app) != 0U) {
            ui->editing = 1U;
            ui->editing_data_id = data->id;
            ui->editing_value = data->value;
            ui->dirty = 1U;
        }
    } else if (node->type == CC_MENU_ITEM_ACTION) {
        cc_car_menu_activate(app, node->id, long_press);
        ui->dirty = 1U;
    }
}

static void cc_ui_back(cc_car_menu_ui_t *ui, cc_car_menu_app_t *app)
{
    const cc_menu_node_t *page;

    if (ui->editing != 0U) {
        ui->editing = 0U;
        ui->editing_data_id = CC_MENU_ID_NONE;
        ui->dirty = 1U;
        return;
    }
    if (ui->current_page == CC_CAR_PAGE_ROOT) return;
    page = cc_menu_find(&app->menu, ui->current_page);
    if (page == 0) return;
    ui->selected_item = page->id;
    ui->current_page = page->parent;
    ui->scroll_item = cc_menu_first_child(&app->menu, ui->current_page);
    ui->dirty = 1U;
}

void cc_car_menu_ui_init(cc_car_menu_ui_t *ui, const cc_car_menu_app_t *app)
{
    if (ui == 0 || app == 0) return;
    memset(ui, 0, sizeof(*ui));
    ui->current_page = CC_CAR_PAGE_DASHBOARD;
    ui->selected_item = cc_menu_first_child(&app->menu, CC_CAR_PAGE_DASHBOARD);
    ui->scroll_item = ui->selected_item;
    ui->editing_data_id = CC_MENU_ID_NONE;
    ui->dirty = 1U;
}

void cc_car_menu_ui_process_input(cc_car_menu_ui_t *ui, cc_car_menu_app_t *app)
{
    uint8_t states[CC_UI_KEY_COUNT] = {0};
    const cc_menu_data_t *data;
    uint32_t now;
    uint8_t key_mask;
    uint8_t key_bit;
    uint8_t right_action;
    uint8_t left_action;
    uint8_t center_action;
    uint8_t i;

    if (ui == 0 || app == 0) return;
    now = system_getval_ms();
    key_mask = cc_tc264_menu_key_mask();
    key_mask &= CC_KEY_UP_MASK | CC_KEY_DOWN_MASK | CC_KEY_LEFT_MASK | CC_KEY_RIGHT_MASK |
                CC_KEY_CENTER_MASK | CC_KEY_AUX1_MASK | CC_KEY_AUX2_MASK;

    for (i = 0U; i < CC_UI_KEY_COUNT; i++) {
        key_bit = 1U << i;
        if ((key_mask & key_bit) && !(ui->previous_key_mask & key_bit)) {
            ui->key_pressed_ms[i] = now;
            ui->last_repeat_ms[i] = now;
            ui->long_latched[i] = 0U;
        } else if (!(key_mask & key_bit) && (ui->previous_key_mask & key_bit)) {
            if (!ui->long_latched[i]) states[i] = CC_UI_KEY_SHORT_PRESS;
            ui->long_latched[i] = 0U;
        } else if ((key_mask & key_bit) && now - ui->key_pressed_ms[i] >= 1000U) {
            states[i] = CC_UI_KEY_LONG_PRESS;
        }
    }
    ui->previous_key_mask = key_mask;

    if (ui->test_until_ms != 0U) {
        if (states[CC_UI_KEY_LEFT] == CC_UI_KEY_SHORT_PRESS || now >= ui->test_until_ms) {
            ui->test_until_ms = 0U;
            ui->dirty = 1U;
        }
        return;
    }

    if (states[CC_UI_KEY_AUX1] == CC_UI_KEY_SHORT_PRESS) {
        cc_ui_back(ui, app);
        return;
    }
    if (states[CC_UI_KEY_AUX2] == CC_UI_KEY_SHORT_PRESS) {
        if (app->safety_state == CC_CAR_SAFE_RUNNING) cc_car_menu_emergency_stop(app);
        else cc_car_menu_activate(app, CC_CAR_ITEM_RUN, 0U);
        ui->dirty = 1U;
        return;
    }

    right_action = states[CC_UI_KEY_RIGHT] == CC_UI_KEY_SHORT_PRESS ||
                   (states[CC_UI_KEY_RIGHT] == CC_UI_KEY_LONG_PRESS && !ui->long_latched[CC_UI_KEY_RIGHT]);
    left_action = states[CC_UI_KEY_LEFT] == CC_UI_KEY_SHORT_PRESS ||
                  (states[CC_UI_KEY_LEFT] == CC_UI_KEY_LONG_PRESS && !ui->long_latched[CC_UI_KEY_LEFT]);
    center_action = states[CC_UI_KEY_CENTER] == CC_UI_KEY_SHORT_PRESS ||
                    (states[CC_UI_KEY_CENTER] == CC_UI_KEY_LONG_PRESS && !ui->long_latched[CC_UI_KEY_CENTER]);

    if (center_action && (app->safety_state == CC_CAR_SAFE_ARMING || app->safety_state == CC_CAR_SAFE_RUNNING)) {
        if (states[CC_UI_KEY_CENTER] == CC_UI_KEY_LONG_PRESS) ui->long_latched[CC_UI_KEY_CENTER] = 1U;
        cc_car_menu_emergency_stop(app);
        ui->dirty = 1U;
        return;
    }
    if (center_action) right_action = 1U;

    if (ui->current_page == CC_CAR_PAGE_DASHBOARD && right_action) {
        if (states[CC_UI_KEY_RIGHT] == CC_UI_KEY_LONG_PRESS) ui->long_latched[CC_UI_KEY_RIGHT] = 1U;
        if (app->safety_state != CC_CAR_SAFE_ARMING && app->safety_state != CC_CAR_SAFE_RUNNING) car_uart_stream_start();
        ui->dirty = 1U;
        return;
    }
    if (ui->current_page == CC_CAR_PAGE_DASHBOARD && left_action && car_uart_stream_is_enabled()) {
        if (states[CC_UI_KEY_LEFT] == CC_UI_KEY_LONG_PRESS) ui->long_latched[CC_UI_KEY_LEFT] = 1U;
        car_uart_stream_stop();
        ui->dirty = 1U;
        return;
    }

    data = ui->editing != 0U ? cc_menu_data_find(&app->menu, ui->editing_data_id) : 0;
    if (states[CC_UI_KEY_UP] == CC_UI_KEY_SHORT_PRESS) {
        if (ui->editing != 0U) cc_ui_adjust_value(ui, data, 1);
        else cc_ui_select_next(ui, &app->menu, -1);
    }
    if (states[CC_UI_KEY_DOWN] == CC_UI_KEY_SHORT_PRESS) {
        if (ui->editing != 0U) cc_ui_adjust_value(ui, data, -1);
        else cc_ui_select_next(ui, &app->menu, 1);
    }
    for (i = 0U; i < 2U; i++) {
        if (states[i] == CC_UI_KEY_LONG_PRESS && now - ui->last_repeat_ms[i] >= 100U) {
            if (ui->editing != 0U) cc_ui_adjust_value(ui, data, i == 0U ? 1 : -1);
            else cc_ui_select_next(ui, &app->menu, i == 0U ? -1 : 1);
            ui->last_repeat_ms[i] = now;
            ui->long_latched[i] = 1U;
        }
    }

    if (states[CC_UI_KEY_RIGHT] == CC_UI_KEY_SHORT_PRESS) {
        if (ui->editing != 0U) {
            cc_menu_data_update(&app->menu, ui->editing_data_id, ui->editing_value);
            ui->editing = 0U;
            ui->editing_data_id = CC_MENU_ID_NONE;
            ui->dirty = 1U;
        } else {
            cc_ui_enter_selected(ui, app, 0U);
        }
    }
    if (states[CC_UI_KEY_RIGHT] == CC_UI_KEY_LONG_PRESS && ui->long_latched[CC_UI_KEY_RIGHT] == 0U) {
        ui->long_latched[CC_UI_KEY_RIGHT] = 1U;
        if (ui->editing == 0U) cc_ui_enter_selected(ui, app, 1U);
    }
    if (left_action) {
        if (states[CC_UI_KEY_LEFT] == CC_UI_KEY_LONG_PRESS) ui->long_latched[CC_UI_KEY_LEFT] = 1U;
        cc_ui_back(ui, app);
    }
}

void cc_car_menu_ui_task(cc_car_menu_ui_t *ui, cc_car_menu_app_t *app, const cc_image_u8_t *gray_frame,
                         const cc_image_u8_t *binary_frame, uint8_t new_frame)
{
    uint32_t now;
    uint8_t draw_images = 0U;
    uint8_t draw_telemetry = 0U;

    if (ui == 0 || app == 0) return;
    now = system_getval_ms();
    if (app->lcd_test_requested != 0U) {
        app->lcd_test_requested = 0U;
        ui->test_until_ms = now + 1500U;
        cc_ui_draw_color_test();
        return;
    }
    if (ui->test_until_ms != 0U) return;
    if (new_frame != 0U && now - ui->last_camera_draw_ms >= CC_UI_CAMERA_PERIOD_MS) {
        draw_images = 1U;
        ui->last_camera_draw_ms = now;
    }
    if (now - ui->last_telemetry_draw_ms >= CC_UI_TELEMETRY_PERIOD_MS) {
        draw_telemetry = 1U;
        ui->last_telemetry_draw_ms = now;
    }
    if (ui->current_page != CC_CAR_PAGE_DASHBOARD) ui->dashboard_static_drawn = 0U;

    if (ui->dirty != 0U) {
        if (ui->current_page == CC_CAR_PAGE_DASHBOARD) {
            if (ui->dashboard_static_drawn == 0U) {
                cc_ui_draw_header(ui, app);
                cc_ui_draw_dashboard(app, gray_frame, binary_frame);
                cc_ui_draw_footer(ui, app);
                ui->dashboard_static_drawn = 1U;
            } else {
                cc_ui_draw_dashboard_delta(app, gray_frame, binary_frame, draw_images, 1U);
                cc_ui_draw_footer(ui, app);
            }
        } else {
            cc_ui_draw_header(ui, app);
            cc_ui_draw_list(ui, app);
            cc_ui_draw_footer(ui, app);
            if (ui->editing != 0U) cc_ui_draw_edit(ui, app);
        }
        ui->dirty = 0U;
    } else if (ui->current_page == CC_CAR_PAGE_DASHBOARD && (draw_images != 0U || draw_telemetry != 0U)) {
        cc_ui_draw_dashboard_delta(app, gray_frame, binary_frame, draw_images, draw_telemetry);
    }
}
