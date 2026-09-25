#ifndef CAR_VOFA_PROTOCOL_H
#define CAR_VOFA_PROTOCOL_H

#include <stdint.h>

enum {
    CAR_VOFA_LINE_CAPACITY = 64
};

typedef struct {
    char buffer[CAR_VOFA_LINE_CAPACITY];
    uint8_t length;
    uint8_t discard;
} car_vofa_line_t;

typedef enum {
    CAR_VOFA_LINE_NONE = 0,
    CAR_VOFA_LINE_READY,
    CAR_VOFA_LINE_OVERFLOW
} car_vofa_line_result_t;

void car_vofa_line_init(car_vofa_line_t *line);
car_vofa_line_result_t car_vofa_line_feed(car_vofa_line_t *line, uint8_t data);

typedef enum {
    CAR_VOFA_CMD_SET = 0,
    CAR_VOFA_CMD_GET,
    CAR_VOFA_CMD_SAVE
} car_vofa_cmd_type_t;

typedef enum {
    CAR_VOFA_PARAM_NONE = -1,
    CAR_VOFA_PARAM_BALANCE_KP = 0,
    CAR_VOFA_PARAM_BALANCE_KD,
    CAR_VOFA_PARAM_MIDDLE_ANGLE,
    CAR_VOFA_PARAM_SPEED_KP,
    CAR_VOFA_PARAM_SPEED_KI,
    CAR_VOFA_PARAM_TARGET_SPEED_MPS,
    CAR_VOFA_PARAM_SPEED_ANGLE_LIMIT,
    CAR_VOFA_PARAM_COUNT
} car_vofa_param_t;

typedef struct {
    car_vofa_cmd_type_t type;
    car_vofa_param_t index;
    float value;
} car_vofa_command_t;

typedef enum {
    CAR_VOFA_PARSE_OK = 0,
    CAR_VOFA_PARSE_SYNTAX,
    CAR_VOFA_PARSE_NAME,
    CAR_VOFA_PARSE_NUMBER
} car_vofa_parse_result_t;

car_vofa_parse_result_t car_vofa_parse_command(const char *text, car_vofa_command_t *command);
const char *car_vofa_param_name(car_vofa_param_t param);

#endif
