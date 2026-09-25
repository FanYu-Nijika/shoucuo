#include "car_vofa_protocol.h"

#include <errno.h>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>

static int car_vofa_is_space(char value)
{
    return value == ' ' || value == '\t';
}

static const char *car_vofa_skip_spaces(const char *text)
{
    while (car_vofa_is_space(*text)) text++;
    return text;
}

static const char *car_vofa_match_keyword(const char *text, const char *keyword)
{
    size_t index = 0;

    while (keyword[index] != '\0' && text[index] == keyword[index]) index++;
    if (keyword[index] != '\0') return NULL;
    if (text[index] != '\0' && !car_vofa_is_space(text[index])) return NULL;
    return &text[index];
}

static int car_vofa_token_equals(const char *token, size_t length, const char *name)
{
    size_t index = 0;

    while (index < length && name[index] != '\0' && token[index] == name[index]) index++;
    return index == length && name[index] == '\0';
}

static car_vofa_param_t car_vofa_param_from_token(const char *token, size_t length)
{
    if (car_vofa_token_equals(token, length, "BALANCE_KP")) return CAR_VOFA_PARAM_BALANCE_KP;
    if (car_vofa_token_equals(token, length, "BALANCE_KD")) return CAR_VOFA_PARAM_BALANCE_KD;
    if (car_vofa_token_equals(token, length, "MIDDLE_ANGLE")) return CAR_VOFA_PARAM_MIDDLE_ANGLE;
    if (car_vofa_token_equals(token, length, "SPEED_KP")) return CAR_VOFA_PARAM_SPEED_KP;
    if (car_vofa_token_equals(token, length, "SPEED_KI")) return CAR_VOFA_PARAM_SPEED_KI;
    if (car_vofa_token_equals(token, length, "TARGET_SPEED_MPS")) return CAR_VOFA_PARAM_TARGET_SPEED_MPS;
    if (car_vofa_token_equals(token, length, "SPEED_ANGLE_LIMIT")) return CAR_VOFA_PARAM_SPEED_ANGLE_LIMIT;
    return CAR_VOFA_PARAM_NONE;
}

void car_vofa_line_init(car_vofa_line_t *line)
{
    if (line == NULL) return;
    line->length = 0;
    line->discard = 0;
    line->buffer[0] = '\0';
}

car_vofa_line_result_t car_vofa_line_feed(car_vofa_line_t *line, uint8_t data)
{
    if (line == NULL) return CAR_VOFA_LINE_NONE;
    if (data == '\n') {
        if (line->discard != 0) {
            car_vofa_line_init(line);
            return CAR_VOFA_LINE_OVERFLOW;
        }
        line->buffer[line->length] = '\0';
        line->length = 0;
        return CAR_VOFA_LINE_READY;
    }
    if (line->discard != 0 || data == '\r') return CAR_VOFA_LINE_NONE;
    if (line->length >= CAR_VOFA_LINE_CAPACITY - 1) {
        line->discard = 1;
        return CAR_VOFA_LINE_NONE;
    }
    line->buffer[line->length] = (char)data;
    line->length++;
    return CAR_VOFA_LINE_NONE;
}

const char *car_vofa_param_name(car_vofa_param_t param)
{
    switch (param) {
    case CAR_VOFA_PARAM_BALANCE_KP: return "BALANCE_KP";
    case CAR_VOFA_PARAM_BALANCE_KD: return "BALANCE_KD";
    case CAR_VOFA_PARAM_MIDDLE_ANGLE: return "MIDDLE_ANGLE";
    case CAR_VOFA_PARAM_SPEED_KP: return "SPEED_KP";
    case CAR_VOFA_PARAM_SPEED_KI: return "SPEED_KI";
    case CAR_VOFA_PARAM_TARGET_SPEED_MPS: return "TARGET_SPEED_MPS";
    case CAR_VOFA_PARAM_SPEED_ANGLE_LIMIT: return "SPEED_ANGLE_LIMIT";
    default: return NULL;
    }
}

car_vofa_parse_result_t car_vofa_parse_command(const char *text, car_vofa_command_t *command)
{
    const char *cursor;
    const char *name_start;
    const char *number_end;
    char *parsed_end;
    size_t name_length;
    float value;

    if (text == NULL || command == NULL) return CAR_VOFA_PARSE_SYNTAX;
    cursor = car_vofa_skip_spaces(text);
    if (car_vofa_match_keyword(cursor, "GET") != NULL) {
        cursor = car_vofa_skip_spaces(car_vofa_match_keyword(cursor, "GET"));
        if (*cursor != '\0') return CAR_VOFA_PARSE_SYNTAX;
        command->type = CAR_VOFA_CMD_GET;
        command->index = CAR_VOFA_PARAM_NONE;
        command->value = 0.0f;
        return CAR_VOFA_PARSE_OK;
    }
    if (car_vofa_match_keyword(cursor, "SAVE") != NULL) {
        cursor = car_vofa_skip_spaces(car_vofa_match_keyword(cursor, "SAVE"));
        if (*cursor != '\0') return CAR_VOFA_PARSE_SYNTAX;
        command->type = CAR_VOFA_CMD_SAVE;
        command->index = CAR_VOFA_PARAM_NONE;
        command->value = 0.0f;
        return CAR_VOFA_PARSE_OK;
    }
    cursor = car_vofa_match_keyword(cursor, "SET");
    if (cursor == NULL) return CAR_VOFA_PARSE_SYNTAX;
    cursor = car_vofa_skip_spaces(cursor);
    if (*cursor == '\0') return CAR_VOFA_PARSE_SYNTAX;
    name_start = cursor;
    while (*cursor != '\0' && !car_vofa_is_space(*cursor)) cursor++;
    name_length = (size_t)(cursor - name_start);
    command->index = car_vofa_param_from_token(name_start, name_length);
    if (command->index == CAR_VOFA_PARAM_NONE) return CAR_VOFA_PARSE_NAME;
    cursor = car_vofa_skip_spaces(cursor);
    if (*cursor == '\0') return CAR_VOFA_PARSE_NUMBER;
    errno = 0;
    value = strtof(cursor, &parsed_end);
    if (parsed_end == cursor || errno == ERANGE || !isfinite(value)) return CAR_VOFA_PARSE_NUMBER;
    number_end = parsed_end;
    while (*number_end != '\0' && !car_vofa_is_space(*number_end)) number_end++;
    if (number_end != parsed_end) return CAR_VOFA_PARSE_NUMBER;
    cursor = car_vofa_skip_spaces(number_end);
    if (*cursor != '\0') return CAR_VOFA_PARSE_SYNTAX;
    command->type = CAR_VOFA_CMD_SET;
    command->value = value;
    return CAR_VOFA_PARSE_OK;
}
