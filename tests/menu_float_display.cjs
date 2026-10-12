const fs = require('fs');
const path = require('path');
const os = require('os');
const { spawnSync } = require('child_process');
const root = path.resolve(__dirname, '..');
const menu = fs.readFileSync(path.join(root, 'code/car_menu.c'), 'utf8');
const library = fs.readFileSync(path.join(root, 'libraries/zf_common/zf_common_function.c'), 'utf8');
const draw = menu.slice(menu.indexOf('static void car_menu_draw_value('), menu.indexOf('static void car_menu_draw_footer('));
const legacyStart = library.indexOf('void func_double_to_str');
const legacyEnd = library.indexOf('\n//-----', legacyStart);
const legacy = library.slice(legacyStart, legacyEnd);
const temp = fs.mkdtempSync(path.join(os.tmpdir(), 'menu-float-display-'));
const cPath = path.join(temp, 'check.c');
const exe = path.join(temp, process.platform === 'win32' ? 'check.exe' : 'check');
fs.writeFileSync(cPath, `
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef uint8_t uint8;
enum { CAR_MENU_VALUE_BOOL, CAR_MENU_VALUE_DIRECTION, CAR_MENU_VALUE_FLOAT, CAR_MENU_VALUE_I32,
       CAR_MENU_VALUE_I16, CAR_MENU_VALUE_I8 };
typedef struct { const volatile void *address; uint8_t value_type; uint8_t decimals; } car_menu_item_t;
static struct { float speed_raw_mps; float speed_filtered_mps; uint8_t encoder_calibration_valid; } balance_state;
static char shown[80];
static void zf_assert(int condition) { assert(condition); }
static int func_abs(int value) { return value < 0 ? -value : value; }
static void ips200_set_color(uint16_t a, uint16_t b) { (void)a; (void)b; }
static void ips200_show_string(uint16_t x, uint16_t y, const char *text) {
    (void)x; (void)y;
    snprintf(shown, sizeof(shown), "%s", text);
}
static void ips200_show_int(uint16_t x, uint16_t y, int32_t value, uint8_t digits) {
    (void)x; (void)y; (void)digits;
    snprintf(shown, sizeof(shown), "%ld", (long)value);
}
static void ips200_show_uint(uint16_t x, uint16_t y, uint32_t value, uint8_t digits) {
    (void)x; (void)y; (void)digits;
    snprintf(shown, sizeof(shown), "%lu", (unsigned long)value);
}
${legacy}
${draw}
int main(void) {
    car_menu_item_t item = {0, CAR_MENU_VALUE_FLOAT, 2};
    char old[20] = {0};
    float value = 0.02f;
    func_double_to_str(old, value, 2);
    assert(strcmp(old, "0.01") == 0);
    car_menu_draw_value(&item, value, 0, 0, 0, 0);
    assert(strcmp(shown, "0.02") == 0);
    car_menu_draw_value(&item, 0.01f, 0, 0, 0, 0);
    assert(strcmp(shown, "0.01") == 0);
    car_menu_draw_value(&item, -0.02f, 0, 0, 0, 0);
    assert(strcmp(shown, "-0.02") == 0);
    car_menu_draw_value(&item, FLT_MAX, 0, 0, 0, 0);
    assert(strlen(shown) <= 11 && strchr(shown, 'e') != NULL);
    car_menu_draw_value(&item, -FLT_MAX, 0, 0, 0, 0);
    assert(strlen(shown) <= 11 && shown[0] == '-');
    item.address = &balance_state.speed_filtered_mps;
    balance_state.encoder_calibration_valid = 0;
    car_menu_draw_value(&item, NAN, 0, 0, 0, 0);
    assert(strcmp(shown, "UNCAL") == 0);
    puts("PASS actual menu draw: legacy 0.02->0.01 reproduced; rounded gains, signed/large values, UNCAL verified");
    return 0;
}
`);
if (process.argv.includes('--source-only')) {
    console.log(cPath);
    process.exit(0);
}
const build = spawnSync('gcc', ['-std=c99', '-Wall', '-Wextra', '-Werror', cPath, '-lm', '-o', exe], { encoding: 'utf8' });
if (build.status !== 0) throw new Error(build.error || build.stdout + build.stderr);
const run = spawnSync(exe, [], { encoding: 'utf8' });
process.stdout.write(run.stdout || '');
if (run.status !== 0) throw new Error(run.error || run.stderr || `Exit ${run.status}`);
