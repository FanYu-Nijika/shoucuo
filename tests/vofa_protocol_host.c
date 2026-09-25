#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "car_vofa_protocol.h"

static void expect_value(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.000001f);
}

static void feed_text(car_vofa_line_t *line, const char *text)
{
    while (*text != '\0') {
        assert(car_vofa_line_feed(line, (uint8_t)*text) == CAR_VOFA_LINE_NONE);
        text++;
    }
}

static void test_line_collection(void)
{
    car_vofa_line_t line;
    uint8_t index;

    car_vofa_line_init(&line);
    feed_text(&line, "GE");
    assert(car_vofa_line_feed(&line, (uint8_t)'T') == CAR_VOFA_LINE_NONE);
    assert(car_vofa_line_feed(&line, (uint8_t)'\r') == CAR_VOFA_LINE_NONE);
    assert(car_vofa_line_feed(&line, (uint8_t)'\n') == CAR_VOFA_LINE_READY);
    assert(strcmp(line.buffer, "GET") == 0);
    feed_text(&line, "SA");
    assert(car_vofa_line_feed(&line, (uint8_t)'V') == CAR_VOFA_LINE_NONE);
    assert(car_vofa_line_feed(&line, (uint8_t)'E') == CAR_VOFA_LINE_NONE);
    assert(car_vofa_line_feed(&line, (uint8_t)'\n') == CAR_VOFA_LINE_READY);
    assert(strcmp(line.buffer, "SAVE") == 0);
    for (index = 0; index < 64; index++) assert(car_vofa_line_feed(&line, (uint8_t)'x') == CAR_VOFA_LINE_NONE);
    assert(car_vofa_line_feed(&line, (uint8_t)'\n') == CAR_VOFA_LINE_OVERFLOW);
    assert(car_vofa_line_feed(&line, (uint8_t)'G') == CAR_VOFA_LINE_NONE);
    assert(car_vofa_line_feed(&line, (uint8_t)'E') == CAR_VOFA_LINE_NONE);
    assert(car_vofa_line_feed(&line, (uint8_t)'T') == CAR_VOFA_LINE_NONE);
    assert(car_vofa_line_feed(&line, (uint8_t)'\n') == CAR_VOFA_LINE_READY);
    assert(strcmp(line.buffer, "GET") == 0);
}

static void test_valid_commands(void)
{
    static const char *names[] = {
        "BALANCE_KP", "BALANCE_KD", "MIDDLE_ANGLE", "SPEED_KP", "SPEED_KI", "TARGET_SPEED_MPS", "SPEED_ANGLE_LIMIT"
    };
    static const float values[] = {1.0f, -2.5f, 0.25f, 3.0f, 0.125f, 1.5f, 4.0f};
    car_vofa_command_t command;
    char text[64];
    int index;

    assert(car_vofa_parse_command("GET", &command) == CAR_VOFA_PARSE_OK);
    assert(command.type == CAR_VOFA_CMD_GET && command.index == CAR_VOFA_PARAM_NONE);
    assert(car_vofa_parse_command("SAVE   ", &command) == CAR_VOFA_PARSE_OK);
    assert(command.type == CAR_VOFA_CMD_SAVE && command.index == CAR_VOFA_PARAM_NONE);
    for (index = 0; index < CAR_VOFA_PARAM_COUNT; index++) {
        (void)snprintf(text, sizeof(text), "SET %s %.6f", names[index], values[index]);
        assert(car_vofa_parse_command(text, &command) == CAR_VOFA_PARSE_OK);
        assert(command.type == CAR_VOFA_CMD_SET && command.index == (car_vofa_param_t)index);
        expect_value(command.value, values[index]);
        assert(strcmp(car_vofa_param_name((car_vofa_param_t)index), names[index]) == 0);
    }
}

static void test_invalid_commands(void)
{
    car_vofa_command_t command;

    assert(car_vofa_parse_command("GET extra", &command) == CAR_VOFA_PARSE_SYNTAX);
    assert(car_vofa_parse_command("GETX", &command) == CAR_VOFA_PARSE_SYNTAX);
    assert(car_vofa_parse_command("SAVE 1", &command) == CAR_VOFA_PARSE_SYNTAX);
    assert(car_vofa_parse_command("SET", &command) == CAR_VOFA_PARSE_SYNTAX);
    assert(car_vofa_parse_command("SET UNKNOWN 1", &command) == CAR_VOFA_PARSE_NAME);
    assert(car_vofa_parse_command("SET BALANCE_KP", &command) == CAR_VOFA_PARSE_NUMBER);
    assert(car_vofa_parse_command("SET BALANCE_KP ", &command) == CAR_VOFA_PARSE_NUMBER);
    assert(car_vofa_parse_command("SET BALANCE_KP .", &command) == CAR_VOFA_PARSE_NUMBER);
    assert(car_vofa_parse_command("SET BALANCE_KP 1abc", &command) == CAR_VOFA_PARSE_NUMBER);
    assert(car_vofa_parse_command("SET BALANCE_KP 1 extra", &command) == CAR_VOFA_PARSE_SYNTAX);
    assert(car_vofa_parse_command("SET BALANCE_KP nan", &command) == CAR_VOFA_PARSE_NUMBER);
    assert(car_vofa_parse_command("SET BALANCE_KP inf", &command) == CAR_VOFA_PARSE_NUMBER);
    assert(car_vofa_parse_command("SET BALANCE_KP 1e999", &command) == CAR_VOFA_PARSE_NUMBER);
}

int main(void)
{
    test_line_collection();
    test_valid_commands();
    test_invalid_commands();
    return 0;
}
