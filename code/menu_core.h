#ifndef CC_MENU_CORE_H
#define CC_MENU_CORE_H

#include <stdint.h>

#define CC_MENU_NODE_CAPACITY       (96U)
#define CC_MENU_DATA_CAPACITY       (72U)
#define CC_MENU_ID_NONE             (0xFFFFU)

typedef enum {
    CC_MENU_OK = 0,
    CC_MENU_ERROR_ARGUMENT,
    CC_MENU_ERROR_FULL,
    CC_MENU_ERROR_NOT_FOUND,
    CC_MENU_ERROR_READ_ONLY,
    CC_MENU_ERROR_RANGE
} cc_menu_result_t;

typedef enum {
    CC_MENU_ITEM_PAGE = 0,
    CC_MENU_ITEM_VALUE,
    CC_MENU_ITEM_ACTION,
    CC_MENU_ITEM_INFO
} cc_menu_item_type_t;

typedef enum {
    CC_MENU_DATA_BOOL = 0,
    CC_MENU_DATA_INT32,
    CC_MENU_DATA_UINT32,
    CC_MENU_DATA_FLOAT,
    CC_MENU_DATA_ENUM
} cc_menu_data_type_t;

typedef enum {
    CC_MENU_EVENT_NONE = 0,
    CC_MENU_EVENT_ACTION,
    CC_MENU_EVENT_VALUE_CHANGED
} cc_menu_event_type_t;

enum {
    CC_MENU_NODE_VISIBLE = 0x01U,
    CC_MENU_NODE_ENABLED = 0x02U
};

enum {
    CC_MENU_DATA_READ_ONLY = 0x01U
};

typedef union {
    uint8_t boolean;
    int32_t int32_value;
    uint32_t uint32_value;
    float float_value;
    int32_t enum_value;
} cc_menu_value_t;

/*
 * 页面、数值、动作和信息都在一张表里定义。
 * 数组中的先后顺序就是屏幕中的显示顺序。
 */
typedef struct {
    uint16_t id;
    uint16_t parent;
    const char *label;
    cc_menu_item_type_t type;
    uint16_t data_id;
    uint16_t action_id;
    uint8_t flags;
} cc_menu_node_spec_t;

/* 范围和步长统一用 float 写，初始化时再按 type 转成实际类型。 */
typedef struct {
    uint16_t id;
    cc_menu_data_type_t type;
    float minimum;
    float maximum;
    float step;
    uint8_t decimals;
    uint8_t flags;
} cc_menu_data_spec_t;

typedef struct {
    uint16_t id;
    uint16_t parent;
    uint16_t next;
    const char *label;
    uint16_t data_id;
    uint16_t action_id;
    cc_menu_item_type_t type;
    uint8_t flags;
} cc_menu_node_t;

typedef struct {
    uint16_t id;
    cc_menu_data_type_t type;
    cc_menu_value_t value;
    cc_menu_value_t minimum;
    cc_menu_value_t maximum;
    cc_menu_value_t step;
    uint8_t decimals;
    uint8_t flags;
} cc_menu_data_t;

typedef struct {
    cc_menu_event_type_t type;
    uint16_t item_id;
    uint16_t action_id;
    uint16_t data_id;
    cc_menu_value_t value;
} cc_menu_event_t;

typedef struct {
    cc_menu_node_t nodes[CC_MENU_NODE_CAPACITY];
    cc_menu_data_t data[CC_MENU_DATA_CAPACITY];
    uint16_t node_count;
    uint16_t data_count;
    cc_menu_event_t pending_event;
    uint8_t event_pending;
} cc_menu_t;

cc_menu_result_t cc_menu_init(cc_menu_t *menu,
                              const cc_menu_node_spec_t *nodes,
                              uint16_t node_count,
                              const cc_menu_data_spec_t *data,
                              uint16_t data_count);

const cc_menu_node_t *cc_menu_find(const cc_menu_t *menu, uint16_t id);
uint16_t cc_menu_first_child(const cc_menu_t *menu, uint16_t parent_id);
uint16_t cc_menu_next_sibling(const cc_menu_t *menu, uint16_t id);
uint16_t cc_menu_previous_sibling(const cc_menu_t *menu, uint16_t id);

cc_menu_result_t cc_menu_data_update(cc_menu_t *menu, uint16_t id,
                                     cc_menu_value_t value);
cc_menu_result_t cc_menu_data_set_runtime(cc_menu_t *menu, uint16_t id,
                                          cc_menu_value_t value);
const cc_menu_data_t *cc_menu_data_find(const cc_menu_t *menu, uint16_t id);

cc_menu_result_t cc_menu_emit_action(cc_menu_t *menu, uint16_t item_id);
uint8_t cc_menu_take_event(cc_menu_t *menu, cc_menu_event_t *event);

#endif

