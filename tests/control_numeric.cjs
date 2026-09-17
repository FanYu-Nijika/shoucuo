/* Run: node tests/control_numeric.cjs
 * No C compiler or firmware build. Execute the arithmetic/control branches
 * extracted from C in a JS sandbox; this does not check target ABI or timing.
 * Hardware I/O is stubbed. JS numbers have more precision than target floats.
 */
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const root = path.resolve(__dirname, '..');
const read = name => fs.readFileSync(path.join(root, name), 'utf8');
const balance = read('code/balance_control.c');
const car = read('code/car.c');
const runtime = read('code/balance_runtime.c');
const header = read('code/balance_control.h');
const strip = text => text.replace(/\/\*[\s\S]*?\*\//g, '').replace(/\/\/[^\n]*/g, '');
const scalarTypes = 'float|uint8_t|uint16_t|uint32_t|int16_t|int32_t|uint8|uint16|uint32|int16|balance_fault_t|balance_params_t';
function extract(source, name) {
    const text = strip(source);
    const start = text.search(new RegExp('(?:static\\s+)?(?:' + scalarTypes + '|void)\\s+' + name + '\\s*\\('));
    assert(start >= 0, name);
    const open = text.indexOf('{', start);
    let depth = 1, end = open + 1;
    while (depth) { if (text[end] === '{') depth++; if (text[end] === '}') depth--; end++; }
    return text.slice(start, end);
}
function js(source) {
    return source.replace(/\b(?:static\s+)?(?:float|uint8_t|uint32_t|int16_t|uint8|void)\s+(\w+)\(([^)]*)\)/,
        (_, name, args) => 'function ' + name + '(' + args.replace(/\b(?:const|volatile|void)\b/g, '').replace(new RegExp('\\b(' + scalarTypes + ')\\b', 'g'), '').replace(/\*/g, '') + ')')
        .replace(/\(void\)\w+;/g, '')
        .replace(/->/g, '.')
        .replace(/&(?=applied_params|balance_params|stored)/g, '')
        .replace(/\(int16_t\)value/g, 'Math.trunc(value)')
        .replace(new RegExp('\\((?:' + scalarTypes + ')\\)', 'g'), '')
        .replace(/\bfloat\s+(\w+)\[3\];/g, 'let $1 = Array(3).fill(0);')
        .replace(new RegExp('\\b(?:' + scalarTypes + ')\\s+(?=\\w)', 'g'), 'let ')
        .replace(/\b(\d+(?:\.\d*)?)f\b/g, '$1')
        .replace(/\b(0x[\dA-Fa-f]+|\d+)u\b/g, '$1')
        .replace(/(applied_params|balance_params|stored) = (balance_default_params|balance_params|stored_params);/g, '$1 = {...$2};');
}
const defaults = Object.fromEntries([...balance.match(/balance_default_params = \{([\s\S]*?)\};/)[1]
    .matchAll(/\.(\w+)\s*=\s*([\d.]+)f?/g)].map(m => [m[1], Number(m[2])]));
const c = vm.createContext({balance_default_params: defaults, balance_params: {}, applied_params: {}, balance_state: {},
    balance_dt_s: .005, balance_tilt_limit_deg: 40, balance_is_finite: Number.isFinite, car_value_finite: Number.isFinite,
    fabsf: Math.abs, atan2f: Math.atan2, cc_math_clamp_f32: (v, lo, hi) => Math.min(hi, Math.max(lo, v)),
    now: 0, system_getval_ms: () => c.now, cc_tc264_menu_motor_stop: () => {},
    BOARD_BALANCE_ACCEL_HORIZONTAL_AXIS: 0, BOARD_BALANCE_GYRO_AXIS: 1, BOARD_BALANCE_ACCEL_SIGN: -1,
    BOARD_BALANCE_SPEED_ANGLE_SIGN: -1});
for (const src of [header, read('code/car.h'), read('code/balance_runtime.h')]) {
    for (const block of strip(src).matchAll(/enum\s*\{([^}]+)\}/g)) {
        let n = 0;
        for (const entry of block[1].split(',')) {
            const m = entry.trim().match(/^(\w+)(?:\s*=\s*(\d+))?$/);
            if (m) { if (m[2]) n = +m[2]; c[m[1]] = n++; }
        }
    }
}
for (const src of [runtime, read('code/balance_runtime.h')]) {
    for (const m of src.matchAll(/#define\s+(BALANCE_CALIBRATION_\w+)\s+\(([\d.]+)f?\)/g)) c[m[1]] = +m[2];
}
for (const name of ['kalman_p00','kalman_p01','kalman_p10','kalman_p11','kalman_initialized','left_count_sum',
    'right_count_sum','speed_sample_phase','requested_speed','requested_turn_speed','car_running','car_camera_ready',
    'car_lost_count','car_follow_status','car_turn_request','follow_command','following','valid_frames',
    'last_valid_capture_ms','derivative_ready','last_control_capture_ms','last_error','curve_blend','car_curve_mode']) c[name] = 0;
c.steering_kp = 2; c.steering_kd = 3.5; c.curve_steering_kp = 2.7; c.curve_steering_kd = 3;
c.curve_exit_threshold = .2; c.curve_enter_threshold = .24;
for (const name of ['balance_reset_kalman','balance_reset_speed','balance_clear_outputs','balance_latch_fault',
    'balance_params_are_valid','balance_update_kalman','balance_clamp','balance_pwm_from_float','balance_apply_motor_deadzone',
    'balance_calculate_rate_output',
    'balance_update_wheel_speed','balance_run_speed',
    'balance_run_control','balance_set_motion_target','balance_sample_encoders_5ms','balance_init',
    'balance_seed_attitude','balance_start','balance_stop','balance_apply_params','balance_update_5ms']) {
    vm.runInContext(js(extract(balance, name)), c, {filename: name});
}
for (const name of ['car_reset_turn','car_follow_check','car_follow_start','car_follow_stop','car_track_update'])
    vm.runInContext(js(extract(car, name)), c, {filename: name});
for (const name of ['balance_runtime_collect_calibration','balance_runtime_finish_calibration','balance_runtime_retry_calibration'])
    vm.runInContext(js(extract(runtime, name)), c, {filename: name});
function reset(enabled = 1) {
    c.balance_init(); c.balance_params.speed_enabled = enabled; c.balance_apply_params(); c.balance_seed_attitude(0);
    assert.equal(c.balance_start(), 1); c.car_running = 1; c.car_camera_ready = 1;
    c.following = 0; c.follow_command = 0; c.valid_frames = 0; c.car_follow_status = 0; c.car_reset_turn();
}
function near(a, b, eps = 1e-8) { assert(Math.abs(a - b) < eps, `${a} != ${b}`); }
function tick(left = 0, right = 0, imuValid = true) {
    c.balance_sample_encoders_5ms(left, right);
    if (imuValid) c.balance_update_5ms(0, 0, 0, 0, 1);
}
let groups = 0;
function test(name, fn) { fn(); groups++; console.log('PASS ' + name); }
test('5 ms output, 20 ms full counts including one skipped IMU read', () => {
    reset(); for (let i = 0; i < 3; i++) { tick(3, 7, i !== 1); assert.equal(c.balance_state.left_count, 0); }
    tick(3, 7); assert.equal(c.balance_state.left_count, 12); assert.equal(c.balance_state.right_count, 28);
    near(c.balance_state.speed_filtered, 20 * .2944);
    tick(); assert.equal(c.balance_state.output_updated, 1);
});
test('zero/forward/backward feedback signs and 100 units/s target ramp', () => {
    reset(); for (let i = 0; i < 4; i++) tick(); near(c.balance_state.speed_output, 0);
    for (let i = 0; i < 4; i++) tick(5, 5); assert(c.balance_state.speed_output > 0);
    reset(); for (let i = 0; i < 4; i++) tick(-5, -5); assert(c.balance_state.speed_output < 0);
    reset(); c.balance_set_motion_target(20, 0); for (let i = 0; i < 4; i++) tick();
    near(c.balance_state.effective_target_speed, 2); c.balance_set_motion_target(0, 0);
    for (let i = 0; i < 4; i++) tick(); near(c.balance_state.effective_target_speed, 0);
});
test('angle/rate limiting, integral freeze and unwinding under PWM saturation', () => {
    reset(); c.applied_params.speed_filter = 1; c.applied_params.speed_kp = .1;
    for (let i = 0; i < 4; i++) tick(-100, -100); near(c.balance_state.left_speed_output, -.04);
    near(c.balance_state.right_speed_output, -.04); near(c.balance_state.left_speed_integral, 0);
    for (let i = 0; i < 400; i++) tick(-100, -100); assert(c.balance_state.speed_output >= -1.5);
    reset(); c.applied_params.speed_filter = 1; c.applied_params.speed_kp = 0;
    c.balance_state.angle = -10; c.balance_state.left_count = -10; c.balance_state.right_count = -10;
    c.balance_run_speed(); near(c.balance_state.left_speed_integral, 20); near(c.balance_state.right_speed_integral, 20);
    c.balance_state.left_speed_integral = 100; c.balance_state.right_speed_integral = 100;
    c.balance_state.left_count = 10; c.balance_state.right_count = 10;
    c.balance_run_speed(); near(c.balance_state.left_speed_integral, 100); near(c.balance_state.right_speed_integral, 100);
});
test('disabled speed equals legacy PD; independent targets and differential preserve common torque', () => {
    reset(0); c.balance_state.angle = 1; c.balance_state.gyro_rate = 2; c.balance_run_control();
    const expected = Math.trunc(defaults.angle_kp + defaults.rate_kp * 2);
    assert.equal(c.balance_state.left_control_pwm, expected); assert.equal(c.balance_state.right_control_pwm, expected);
    assert.equal(c.balance_state.left_pwm, expected); assert.equal(c.balance_state.right_pwm, expected - 500);
    reset(); c.balance_set_motion_target(20, 5); for (let i = 0; i < 4; i++) tick();
    near(c.balance_state.left_target_speed, 7); near(c.balance_state.right_target_speed, -3);
    c.applied_params.rate_limit = 500;
    c.balance_state.angle = 4.8; c.balance_state.left_target_angle = -.4; c.balance_state.right_target_angle = .4;
    c.balance_run_control(); assert(Math.abs(c.balance_state.left_pwm) <= 5000);
    assert(Math.abs(c.balance_state.right_pwm) <= 5000);
    near((c.balance_state.left_control_pwm + c.balance_state.right_control_pwm) / 2, 4800, 1);
    c.balance_stop(); assert.equal(c.balance_state.left_pwm, 0); near(c.balance_state.turn_output, 0);
    near(c.balance_state.left_speed_integral, 0); near(c.balance_state.right_speed_integral, 0);
});
test('angle outer loop and rate inner loop signs, telemetry and target-rate limit', () => {
    reset(0); c.balance_state.angle = 1; c.balance_state.gyro_rate = 2; c.balance_run_control();
    assert(c.balance_state.left_angle_error > 0); assert(c.balance_state.left_target_rate < 0);
    assert(c.balance_state.left_rate_error < 0); assert(c.balance_state.left_rate_output > 0);
    near(c.balance_state.left_rate_output, defaults.angle_kp + defaults.rate_kp * 2, 1e-5);
    c.balance_state.angle = -1; c.balance_state.gyro_rate = -2; c.balance_run_control();
    assert(c.balance_state.left_target_rate > 0); assert(c.balance_state.left_rate_output < 0);
    c.applied_params.rate_limit = 20; c.balance_state.angle = 10; c.balance_state.gyro_rate = 0; c.balance_run_control();
    near(c.balance_state.left_target_rate, -20); near(c.balance_state.right_target_rate, -20);
    near(c.balance_state.left_rate_output, defaults.rate_kp * 20, 1e-5);
    c.balance_stop(); near(c.balance_state.left_target_rate, 0); near(c.balance_state.left_rate_output, 0);
});
test('left and right PI states respond independently', () => {
    reset(); c.applied_params.speed_filter = 1; c.balance_set_motion_target(0, 0);
    for (let i = 0; i < 4; i++) tick(0, 10);
    near(c.balance_state.left_speed_error, 0); assert(c.balance_state.right_speed_error < 0);
    near(c.balance_state.left_speed_integral, 0); assert(c.balance_state.right_speed_integral < 0);
    assert(c.balance_state.right_speed_output > c.balance_state.left_speed_output);
    c.balance_state.angle = 2; c.balance_run_control();
    assert(c.balance_state.right_control_pwm < c.balance_state.left_control_pwm);
    assert(c.balance_state.right_pwm < c.balance_state.left_pwm);
});
test('motor dead-zone difference mapping is symmetric and preserves high-speed slope', () => {
    reset(0);
    for (const [input, left, right] of [[0, 0, 0], [500, 500, 0], [1100, 1100, 600], [2000, 2000, 1500],
        [-500, -500, 0], [-1100, -1100, -600], [-2000, -2000, -1500]]) {
        assert.equal(c.balance_apply_motor_deadzone(input, 1100), left);
        assert.equal(c.balance_apply_motor_deadzone(input, 600), right);
    }
});
function frame(time, error = 10, valid = 1, isNew = 1) {
    c.now = time; c.car_track_update(error, 0, valid, isNew, time, time);
}
test('explicit follow start, first-frame D=0, time-normalized D and lost-line latch', () => {
    reset(); c.balance_params.target_speed = 20;
    frame(20); frame(40); assert.equal(c.car_follow_start(), 0);
    frame(60); assert.equal(c.car_follow_start(), 1); frame(80);
    near(c.car_turn_request, 20); frame(120, 12); near(c.car_turn_request, 24 + 3.5);
    frame(140, 0, 0); assert.equal(c.following, 0); near(c.requested_turn_speed, 0); near(c.requested_speed, 0);
    frame(160); frame(180); frame(200); assert.equal(c.following, 0);
    assert.equal(c.car_follow_start(), 1); frame(220); c.now = 321;
    c.car_track_update(10, 0, 1, 0, 220, 321); assert.equal(c.following, 0);
    assert.equal(c.car_follow_status, c.CAR_FOLLOW_LINE_LOST);
});
test('1 second calibration: sample threshold, biases, axis and quality warning', () => {
    for (const validCount of [189, 190, 200]) {
        reset(); c.balance_stop();
        c.balance_imu_status = {calibration_samples: 0, calibration_valid_samples: 0, calibration_quality: 0,
            accel_x: 0, accel_y: 0, accel_z: 1, gyro_x: 1, gyro_y: 2, gyro_z: 3};
        for (const name of ['calibration_accel_sum','calibration_accel_square_sum','calibration_gyro_sum','calibration_gyro_square_sum']) c[name] = [0, 0, 0];
        for (let i = 0; i < 200; i++) c.balance_runtime_collect_calibration(i < validCount ? 1 : 0);
        assert.equal(c.balance_imu_status.calibration_samples, 200);
        assert.equal(c.balance_runtime_finish_calibration(), validCount >= 190 ? 1 : 0);
        if (validCount >= 190) { near(c.balance_imu_status.gyro_bias_y, 2); assert.equal(c.balance_imu_status.vertical_axis, 2); }
    }
    c.balance_stop(); c.calibration_gyro_square_sum[0] = 200 * 40;
    assert.equal(c.balance_runtime_finish_calibration(), 1); assert.equal(c.balance_imu_status.calibration_quality, 1);
    c.balance_stop(); c.calibration_accel_sum[2] = 0;
    assert.equal(c.balance_runtime_finish_calibration(), 0); assert.equal(c.balance_state.left_pwm, 0);
});

// Model little-endian target layout, then execute the actual Flash migration branches.
const types = {float: [4, 'Float'], int16_t: [2, 'Int16'], uint16_t: [2, 'UInt16'], uint8_t: [1, 'UInt8'], int8_t: [1, 'Int8']};
let size = 0;
const fields = [...strip(header).match(/typedef struct \{([\s\S]*?)\} balance_params_t;/)[1].matchAll(/(float|int16_t|uint16_t|uint8_t|int8_t)\s+(\w+);/g)]
    .map(([, type, name]) => { const [bytes, kind] = types[type]; size = Math.ceil(size / bytes) * bytes; const f = {name, kind, offset: size}; size += bytes; return f; });
size = Math.ceil(size / 4) * 4;
assert.equal(fields.find(f => f.name === 'target_speed').offset, 48);
assert.equal(fields.find(f => f.name === 'left_motor_deadzone').offset, 56);
assert.equal(fields.find(f => f.name === 'rate_limit').offset, 60);
const page = Buffer.alloc(4096, 255);
c.flash_union_buffer = Array.from({length: 1024}, (_, i) => Object.defineProperty({}, 'uint32_type', {
    get: () => page.readUInt32LE(i * 4), set: value => page.writeUInt32LE(value >>> 0, i * 4)}));
function writeParams(offset, params) {
    for (const f of fields) page['write' + f.kind + (f.kind.endsWith('8') ? '' : 'LE')](params[f.name], offset * 4 + f.offset);
}
c.writeParams = writeParams;
c.readParams = (offset, length) => {
    const p = {...defaults};
    for (const f of fields) if (f.offset < length) p[f.name] = page['read' + f.kind + (f.kind.endsWith('8') ? '' : 'LE')](offset * 4 + f.offset);
    return p;
};
Object.assign(c, {BALANCE_PARAMS_FLASH_MAGIC: 0x42414C31, BALANCE_PARAMS_FLASH_VERSION: 7,
    BALANCE_PARAMS_FLASH_PROFILE_COUNT: 4, BALANCE_PARAMS_FLASH_PROFILE_START: 2,
    BALANCE_PARAMS_FLASH_PROFILE_WORDS: size / 4, BALANCE_PARAMS_FLASH_OLD_WORDS: 12,
    BALANCE_PARAMS_FLASH_V4_WORDS: 14, BALANCE_PARAMS_FLASH_V5_WORDS: 15, EEPROM_PAGE_LENGTH: 1024});
vm.runInContext(js(extract(balance, 'balance_params_profile_offset')), c);
let migration = extract(balance, 'balance_params_flash_prepare')
    .replace(/memcpy\(&stored, &flash_union_buffer\[(\w+)\], ([^;]+)\);/g, 'stored = readParams($1, $2);')
    .replace(/memcpy\(&flash_union_buffer\[offset\], &(stored|balance_default_params), [^;]+\);/g, 'writeParams(offset, $1);')
    .replace(/sizeof\((?:stored|balance_params_t)\)/g, String(size));
vm.runInContext(js(migration), c);
test('V1/V2 four-profile expansion preserves old PD scale and migrates placeholder speed gains', () => {
    for (const version of [1, 2]) {
        page.fill(255); page.writeUInt32LE(0x42414C31, 0); page.writeUInt32LE(version, 4);
        for (let gear = 1; gear <= 4; gear++) {
            const temp = Buffer.alloc(size); const p = {...defaults, angle_kp: 100 * gear, rate_kp: 20 + gear, middle_angle: gear,
                speed_kp: version === 1 ? 1 : .00025, speed_ki: version === 1 ? .01 : .000005,
                speed_output_limit: version === 1 ? 500 : 1.5};
            for (const f of fields) temp['write' + f.kind + (f.kind.endsWith('8') ? '' : 'LE')](p[f.name], f.offset);
            temp.copy(page, (2 + (gear - 1) * 12) * 4, 0, 48);
        }
        c.balance_params_flash_prepare(); assert.equal(page.readUInt32LE(4), 7);
        for (let gear = 1; gear <= 4; gear++) {
            const p = c.readParams(2 + (gear - 1) * size / 4, size);
            near(p.angle_kp, 100 * gear, 1e-6); near(p.rate_kp, 20 + gear);
            near(p.rate_limit, 200); near(p.middle_angle, gear); near(p.target_speed, 0);
            near(p.speed_kp, version === 1 ? 1 / (100 * gear) : .004, 1e-7);
            assert.equal(p.turn_limit, 100); assert.equal(p.turn_direction, 1);
            assert.equal(p.left_motor_deadzone, 1100); assert.equal(p.right_motor_deadzone, 600);
        }
        const before = Buffer.from(page); c.balance_params_flash_prepare(); assert.deepEqual(page, before);
    }
});
test('V3/V4 old-stride profiles expand to V6 without crossing profile boundaries', () => {
    for (const version of [3, 4]) {
        page.fill(255); page.writeUInt32LE(0x42414C31, 0); page.writeUInt32LE(version, 4);
        for (let gear = 1; gear <= 4; gear++) {
            const temp = Buffer.alloc(size); const p = {...defaults, angle_kp: 100 * gear, rate_kp: 20 + gear, middle_angle: gear,
                turn_limit: version === 3 ? 500 : 77, turn_direction: -1};
            for (const f of fields) temp['write' + f.kind + (f.kind.endsWith('8') ? '' : 'LE')](p[f.name], f.offset);
            temp.copy(page, (2 + (gear - 1) * 14) * 4, 0, 56);
        }
        c.balance_params_flash_prepare(); assert.equal(page.readUInt32LE(4), 7);
        for (let gear = 1; gear <= 4; gear++) {
            const p = c.readParams(2 + (gear - 1) * size / 4, size);
            near(p.angle_kp, 100 * gear, 1e-6); near(p.rate_kp, 20 + gear);
            near(p.rate_limit, 200); near(p.middle_angle, gear);
            assert.equal(p.turn_limit, version === 3 ? 100 : 77); assert.equal(p.turn_direction, -1);
            assert.equal(p.left_motor_deadzone, 1100); assert.equal(p.right_motor_deadzone, 600);
        }
    }
});
test('V5 profiles expand to V6 and preserve dead-zone parameters', () => {
    page.fill(255); page.writeUInt32LE(0x42414C31, 0); page.writeUInt32LE(5, 4);
    for (let gear = 1; gear <= 4; gear++) {
        const temp = Buffer.alloc(size); const p = {...defaults, angle_kp: 500 + 100 * gear, rate_kp: 25,
            middle_angle: gear, left_motor_deadzone: 1000 + gear, right_motor_deadzone: 500 + gear};
        for (const f of fields) temp['write' + f.kind + (f.kind.endsWith('8') ? '' : 'LE')](p[f.name], f.offset);
        temp.copy(page, (2 + (gear - 1) * 15) * 4, 0, 60);
    }
    c.balance_params_flash_prepare(); assert.equal(page.readUInt32LE(4), 7);
    for (let gear = 1; gear <= 4; gear++) {
        const p = c.readParams(2 + (gear - 1) * size / 4, size);
        near(p.angle_kp, 500 + 100 * gear, 1e-6); near(p.rate_kp, 25); near(p.rate_limit, 200);
        assert.equal(p.left_motor_deadzone, 1000 + gear); assert.equal(p.right_motor_deadzone, 500 + gear);
    }
});
test('V6 outer-loop gain returns to the old PWM-per-degree scale', () => {
    page.fill(255); page.writeUInt32LE(0x42414C31, 0); page.writeUInt32LE(6, 4);
    for (let gear = 1; gear <= 4; gear++) {
        const temp = Buffer.alloc(size); const p = {...defaults, angle_kp: 40 + gear, rate_kp: 25,
            speed_kp: .00025, speed_ki: .000005};
        for (const f of fields) temp['write' + f.kind + (f.kind.endsWith('8') ? '' : 'LE')](p[f.name], f.offset);
        temp.copy(page, (2 + (gear - 1) * size / 4) * 4, 0, size);
    }
    c.balance_params_flash_prepare(); assert.equal(page.readUInt32LE(4), 7);
    for (let gear = 1; gear <= 4; gear++) {
        const p = c.readParams(2 + (gear - 1) * size / 4, size);
        near(p.angle_kp, (40 + gear) * 25, 1e-5); near(p.rate_kp, 25);
        near(p.speed_kp, .004, 1e-7); near(p.speed_ki, .00002, 1e-8);
    }
});
test('menu display bounds, stopped-only edits and capture generation checks', () => {
    const menu = read('code/car_menu.c');
    for (const call of menu.matchAll(/ips200_show_int\([^;]*,\s*(\d+)\);/g)) assert(+call[1] <= 10);
    assert(menu.includes('car_running == 0 && car_menu_start_pending == 0'));
    const main = read('user/cpu0_main.c');
    assert.equal((main.match(/car_camera_capture_generation == generation_before/g) || []).length, 2);
    assert(!main.includes('mt9v03x_finish_flag = 0'));
    assert(!read('code/car_menu_port.c').includes('servo_write'));
    assert(!car.includes('pwm_set_duty'));
});
console.log(`${groups} groups passed; source-level numerical checks only, no firmware compilation.`);
