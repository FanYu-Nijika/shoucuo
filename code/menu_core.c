#include "menu_core.h"

#include <string.h>

static cc_menu_data_t *cc_menu_data_find_writable(cc_menu_t *menu, uint16_t id)
{
    uint16_t index;

    for (index = 0U; index < menu->data_count; index++) {
        if (menu->data[index].id == id) return &menu->data[index];
    }
    return 0;
}

static void cc_menu_value_from_float(cc_menu_value_t *destination,
                                     cc_menu_data_type_t type, float value)
{
    if (type == CC_MENU_DATA_BOOL) {
        destination->boolean = value != 0.0f ? 1U : 0U;
    } else if (type == CC_MENU_DATA_INT32) {
        destination->int32_value = (int32_t)value;
    } else if (type == CC_MENU_DATA_UINT32) {
        destination->uint32_value = (uint32_t)value;
    } else if (type == CC_MENU_DATA_FLOAT) {
        destination->float_value = value;
    } else {
        destination->enum_value = (int32_t)value;
    }
}

static cc_menu_result_t cc_menu_validate_value(const cc_menu_data_t *data,
                                                cc_menu_value_t value)
{
    if (data->type == CC_MENU_DATA_BOOL) {
        return value.boolean <= 1U ? CC_MENU_OK : CC_MENU_ERROR_RANGE;
    }
    if (data->type == CC_MENU_DATA_INT32) {
        return value.int32_value >= data->minimum.int32_value &&
               value.int32_value <= data->maximum.int32_value ?
               CC_MENU_OK : CC_MENU_ERROR_RANGE;
    }
    if (data->type == CC_MENU_DATA_UINT32) {
        return value.uint32_value >= data->minimum.uint32_value &&
               value.uint32_value <= data->maximum.uint32_value ?
               CC_MENU_OK : CC_MENU_ERROR_RANGE;
    }
    if (data->type == CC_MENU_DATA_FLOAT) {
        return value.float_value >= data->minimum.float_value &&
               value.float_value <= data->maximum.float_value ?
               CC_MENU_OK : CC_MENU_ERROR_RANGE;
    }
    return value.enum_value >= data->minimum.enum_value &&
           value.enum_value <= data->maximum.enum_value ?
           CC_MENU_OK : CC_MENU_ERROR_RANGE;
}

cc_menu_result_t cc_menu_init(cc_menu_t *menu,
                              const cc_menu_node_spec_t *nodes,
                              uint16_t node_count,
                              const cc_menu_data_spec_t *data,
                              uint16_t data_count)
{
    uint16_t index;
    uint16_t next;

    if (menu == 0 || nodes == 0 || data == 0) return CC_MENU_ERROR_ARGUMENT;
    if (node_count > CC_MENU_NODE_CAPACITY || data_count > CC_MENU_DATA_CAPACITY) {
        return CC_MENU_ERROR_FULL;
    }

    memset(menu, 0, sizeof(*menu));
    menu->node_count = node_count;
    menu->data_count = data_count;

    for (index = 0U; index < node_count; index++) {
        menu->nodes[index].id = nodes[index].id;
        menu->nodes[index].parent = nodes[index].parent;
        menu->nodes[index].next = CC_MENU_ID_NONE;
        menu->nodes[index].label = nodes[index].label;
        menu->nodes[index].type = nodes[index].type;
        menu->nodes[index].data_id = nodes[index].data_id;
        menu->nodes[index].action_id = nodes[index].action_id;
        menu->nodes[index].flags = nodes[index].flags;

        /* 同一父页面中，数组里的下一行就是下一个菜单项。 */
        for (next = (uint16_t)(index + 1U); next < node_count; next++) {
            if (nodes[next].parent == nodes[index].parent) {
                menu->nodes[index].next = nodes[next].id;
                break;
            }
        }
    }

    for (index = 0U; index < data_count; index++) {
        menu->data[index].id = data[index].id;
        menu->data[index].type = data[index].type;
        menu->data[index].decimals = data[index].decimals;
        menu->data[index].flags = data[index].flags;
        cc_menu_value_from_float(&menu->data[index].minimum,
                                 data[index].type, data[index].minimum);
        cc_menu_value_from_float(&menu->data[index].maximum,
                                 data[index].type, data[index].maximum);
        cc_menu_value_from_float(&menu->data[index].step,
                                 data[index].type, data[index].step);
    }
    return CC_MENU_OK;
}

const cc_menu_node_t *cc_menu_find(const cc_menu_t *menu, uint16_t id)
{
    uint16_t index;

    if (menu == 0) return 0;
    for (index = 0U; index < menu->node_count; index++) {
        if (menu->nodes[index].id == id) return &menu->nodes[index];
    }
    return 0;
}

uint16_t cc_menu_first_child(const cc_menu_t *menu, uint16_t parent_id)
{
    uint16_t index;

    if (menu == 0) return CC_MENU_ID_NONE;
    for (index = 0U; index < menu->node_count; index++) {
        if (menu->nodes[index].parent == parent_id) return menu->nodes[index].id;
    }
    return CC_MENU_ID_NONE;
}

uint16_t cc_menu_next_sibling(const cc_menu_t *menu, uint16_t id)
{
    const cc_menu_node_t *node = cc_menu_find(menu, id);
    return node == 0 ? CC_MENU_ID_NONE : node->next;
}

uint16_t cc_menu_previous_sibling(const cc_menu_t *menu, uint16_t id)
{
    const cc_menu_node_t *node;
    uint16_t previous = CC_MENU_ID_NONE;
    uint16_t index;

    if (menu == 0) return CC_MENU_ID_NONE;
    node = cc_menu_find(menu, id);
    if (node == 0) return CC_MENU_ID_NONE;

    for (index = 0U; index < menu->node_count; index++) {
        if (menu->nodes[index].id == id) return previous;
        if (menu->nodes[index].parent == node->parent) previous = menu->nodes[index].id;
    }
    return CC_MENU_ID_NONE;
}

cc_menu_result_t cc_menu_data_update(cc_menu_t *menu, uint16_t id,
                                     cc_menu_value_t value)
{
    cc_menu_data_t *data;
    cc_menu_result_t result;

    if (menu == 0) return CC_MENU_ERROR_ARGUMENT;
    data = cc_menu_data_find_writable(menu, id);
    if (data == 0) return CC_MENU_ERROR_NOT_FOUND;
    if ((data->flags & CC_MENU_DATA_READ_ONLY) != 0U) return CC_MENU_ERROR_READ_ONLY;

    result = cc_menu_validate_value(data, value);
    if (result != CC_MENU_OK) return result;
    data->value = value;

    menu->pending_event.type = CC_MENU_EVENT_VALUE_CHANGED;
    menu->pending_event.item_id = CC_MENU_ID_NONE;
    menu->pending_event.action_id = CC_MENU_ID_NONE;
    menu->pending_event.data_id = id;
    menu->pending_event.value = value;
    menu->event_pending = 1U;
    return CC_MENU_OK;
}

cc_menu_result_t cc_menu_data_set_runtime(cc_menu_t *menu, uint16_t id,
                                          cc_menu_value_t value)
{
    cc_menu_data_t *data;

    if (menu == 0) return CC_MENU_ERROR_ARGUMENT;
    data = cc_menu_data_find_writable(menu, id);
    if (data == 0) return CC_MENU_ERROR_NOT_FOUND;
    data->value = value;
    return CC_MENU_OK;
}

const cc_menu_data_t *cc_menu_data_find(const cc_menu_t *menu, uint16_t id)
{
    uint16_t index;

    if (menu == 0) return 0;
    for (index = 0U; index < menu->data_count; index++) {
        if (menu->data[index].id == id) return &menu->data[index];
    }
    return 0;
}

cc_menu_result_t cc_menu_emit_action(cc_menu_t *menu, uint16_t item_id)
{
    const cc_menu_node_t *node;

    if (menu == 0) return CC_MENU_ERROR_ARGUMENT;
    node = cc_menu_find(menu, item_id);
    if (node == 0 || node->type != CC_MENU_ITEM_ACTION) return CC_MENU_ERROR_NOT_FOUND;

    menu->pending_event.type = CC_MENU_EVENT_ACTION;
    menu->pending_event.item_id = item_id;
    menu->pending_event.action_id = node->action_id;
    menu->pending_event.data_id = CC_MENU_ID_NONE;
    menu->event_pending = 1U;
    return CC_MENU_OK;
}

uint8_t cc_menu_take_event(cc_menu_t *menu, cc_menu_event_t *event)
{
    if (menu == 0 || event == 0 || menu->event_pending == 0U) return 0U;
    *event = menu->pending_event;
    menu->event_pending = 0U;
    menu->pending_event.type = CC_MENU_EVENT_NONE;
    return 1U;
}

