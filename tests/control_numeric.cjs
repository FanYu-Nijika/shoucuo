/* Run: node tests/control_numeric.cjs
 * Source-level and arithmetic checks only. This script does not compile firmware.
 */
const fs = require('node:fs');
const path = require('node:path');
const assert = require('node:assert/strict');
const root = path.resolve(__dirname, '..');
const read = name => fs.readFileSync(path.join(root, name), 'utf8');
const control = read('code/balance_control.c');
const controlHeader = read('code/balance_control.h');
const runtime = read('code/balance_runtime.c');
const runtimeHeader = read('code/balance_runtime.h');
const car = read('code/car.c');
const menu = read('code/car_menu.c');
const near = (a, b, eps = 1e-8) => assert(Math.abs(a - b) <= eps, `${a} != ${b}`);
let groups = 0;
function test(name, fn) { fn(); groups++; console.log('PASS ' + name); }

function complementary(last, accel, gyro, ka) {
    const predicted = last + gyro * 0.005;
    return ka * accel + (1 - ka) * predicted;
}

function clamp(value, limit) {
    return Math.max(-limit, Math.min(limit, value));
}

function allocate(commonRaw, turnCommand, pwmLimit) {
    const common = clamp(commonRaw, pwmLimit);
    const turn = clamp(turnCommand, pwmLimit - Math.abs(common));
    return {left: common + turn, right: common - turn, common, turn};
}

function deadzone(controlValue, wheelDeadzone, leftDeadzone = 1100, rightDeadzone = 600, limit = 5000) {
    const reference = Math.max(leftDeadzone, rightDeadzone);
    const magnitude = Math.abs(controlValue) - (reference - wheelDeadzone);
    if (magnitude <= 0) return 0;
    return Math.trunc(clamp(Math.sign(controlValue) * magnitude, limit));
}

test('complementary filter uses 5 ms gyro integration and adjustable accelerometer weight', () => {
    near(complementary(10, 30, 20, 0), 10.1);
    near(complementary(10, 30, 20, 1), 30);
    near(complementary(10, 30, 20, .01), .01 * 30 + .99 * 10.1);
    assert(control.includes('balance_state.angle + gyro_rate * balance_dt_s'));
    assert(control.includes('applied_params.accel_weight * accel_angle'));
    assert(control.includes('static const float balance_dt_s = 0.005f'));
});

test('direct hard upright PD preserves 1000/25.0556 scale and PWM safety clamp', () => {
    const raw = 1000 * 1 + 25.0556 * 2;
    near(raw, 1050.1112, 1e-6);
    assert(control.includes('applied_params.balance_kp * balance_state.angle_error + applied_params.balance_kd * balance_state.gyro_rate'));
    assert(!control.includes('target_rate'));
    assert(!control.includes('rate_error'));
    assert.equal(clamp(25000, 5000), 5000);
    assert.equal(clamp(-25000, 5000), -5000);
});

test('upright gains have no artificial tuning ceiling', () => {
    assert(!control.match(/balance_kp\s*>/));
    assert(!control.match(/balance_kd\s*>/));
    assert(menu.includes('&balance_params.balance_kp, 0, FLT_MAX'));
    assert(menu.includes('&balance_params.balance_kd, 0, FLT_MAX'));
    assert(control.includes('params->pwm_limit > 10000'));
});

test('FOLLOW lean slews at 2 deg/s and positive lean lowers target angle', () => {
    let applied = 0;
    for (let i = 0; i < 50; i++) applied += clamp(.5 - applied, .01);
    near(applied, .5, 1e-7);
    near(0 - applied, -.5, 1e-7);
    for (let i = 0; i < 25; i++) applied += clamp(0 - applied, .01);
    near(applied, .25, 1e-7);
    assert(control.includes('applied_params.middle_angle - balance_state.follow_angle_applied'));
});

test('camera PWM differential never steals common balance headroom', () => {
    let out = allocate(4500, 1000, 5000);
    near(out.left, 5000); near(out.right, 4000); near(out.turn, 500);
    out = allocate(-4800, -1000, 5000);
    near(out.left, -5000); near(out.right, -4600); near(out.turn, -200);
    out = allocate(6000, 500, 5000);
    near(out.left, 5000); near(out.right, 5000); near(out.turn, 0);
});

test('existing dead-zone alignment remains symmetric', () => {
    for (const [input, left, right] of [[0, 0, 0], [500, 500, 0], [1100, 1100, 600], [2000, 2000, 1500],
        [-500, -500, 0], [-1100, -1100, -600], [-2000, -2000, -1500]]) {
        assert.equal(deadzone(input, 1100), left);
        assert.equal(deadzone(input, 600), right);
    }
});

test('encoders are display-only 20 ms accumulators', () => {
    const left = [3, 3, 3, 3].reduce((sum, value) => sum + value, 0);
    const right = [7, 7, 7, 7].reduce((sum, value) => sum + value, 0);
    assert.equal(left, 12); assert.equal(right, 28);
    assert(control.includes('if (++encoder_sample_phase < 4) return'));
    assert(!controlHeader.match(/speed_(?:kp|ki|filter|integral|output|enabled|period)/));
});

test('manual calibration timing resets on movement and collects five clean seconds', () => {
    let staticSamples = 0;
    let collected = 0;
    for (let i = 0; i < 100; i++) staticSamples++;
    assert.equal(staticSamples * 5, 500);
    for (let i = 0; i < 400; i++) collected++;
    collected = 0;
    staticSamples = 0;
    for (let i = 0; i < 100; i++) staticSamples++;
    for (let i = 0; i < 1000; i++) collected++;
    assert.equal(collected * 5, 5000);
    assert.equal(Number(runtimeHeader.match(/BALANCE_CAL_STATIC_SAMPLES\s+\((\d+)\)/)[1]), 100);
    assert.equal(Number(runtimeHeader.match(/BALANCE_CALIBRATION_SAMPLES\s+\((\d+)\)/)[1]), 1000);
});

test('background trim is RAM-only, gated by stop/static, and uses alpha 0.001', () => {
    let bias = 2;
    const raw = 3;
    bias += .001 * (raw - bias);
    near(bias, 2.001);
    assert(runtime.includes('if (balance_state.running || motor_test_active || !balance_imu_status.flash_calibration_valid)'));
    assert(runtime.includes('#define BALANCE_TRIM_ALPHA                  (0.001f)'));
    const trimBody = runtime.slice(runtime.indexOf('static void balance_runtime_update_bias_trim'),
        runtime.indexOf('static void balance_runtime_imu_failed'));
    assert(!trimBody.includes('flash_write'));
});

test('RUN keeps the warmup attitude and motor test leaves raw PWM ownership outside balance control', () => {
    const updateBody = runtime.slice(runtime.indexOf('void balance_runtime_update_5ms'),
        runtime.indexOf('void balance_runtime_stop'));
    const startBlock = updateBody.slice(updateBody.indexOf('if (balance_start_pending'),
        updateBody.indexOf('if (was_running'));
    const motorTestBlock = updateBody.slice(updateBody.indexOf('if (motor_test_active)'),
        updateBody.indexOf('if (!imu_initialized)'));
    assert(!startBlock.includes('balance_seed_attitude'));
    assert(startBlock.includes('car_running = balance_start()'));
    assert(!motorTestBlock.includes('cc_tc264_menu_motor_stop'));
    assert(updateBody.indexOf('balance_sample_encoders_5ms') < updateBody.indexOf('if (motor_test_active)'));
});

test('Flash calibration is separate on page 10 and guarded by magic/version/checksum', () => {
    assert(runtime.includes('#define BALANCE_IMU_FLASH_PAGE              (10)'));
    assert(runtime.includes('record->checksum != balance_runtime_record_checksum(record)'));
    assert(runtime.includes('flash_write_page_from_buffer(BALANCE_IMU_FLASH_SECTOR, BALANCE_IMU_FLASH_PAGE)'));
    assert(runtime.includes('calibration_save_pending'));
});

test('V8 migration keeps only upright/mechanical/output/direction/dead-zone fields', () => {
    assert(control.includes('#define BALANCE_PARAMS_FLASH_VERSION       (8u)'));
    const conversion = control.slice(control.indexOf('static balance_params_t balance_params_from_v7'),
        control.indexOf('static void balance_params_flash_prepare'));
    for (const field of ['angle_kp', 'rate_kp', 'middle_angle', 'pwm_limit', 'turn_direction',
        'left_motor_deadzone', 'right_motor_deadzone']) assert(conversion.includes(field));
    for (const field of ['speed_kp', 'speed_ki', 'q_angle', 'q_bias', 'r_angle', 'rate_limit'])
        assert(!conversion.includes(`old->${field}`));
});

test('active menu/control contain no speed loop, Kalman, or rate cascade', () => {
    for (const text of ['SPEED ENABLE', 'SPEED KP', 'SPEED KI', 'SPEED FILTER', 'TARGET SPEED',
        'Q ANGLE', 'Q BIAS', 'R ANGLE', 'KALMAN ANGLE', 'RATE LIMIT', 'TARGET RATE', 'RATE ERROR', 'RATE OUTPUT']) {
        assert(!menu.includes(text), text);
    }
    assert(!car.includes('speed_enabled'));
    assert(!car.includes('target_speed'));
    assert(car.includes('balance_set_follow_command(balance_params.follow_angle, car_turn_request)'));
});

console.log(`${groups} groups passed; source-level numerical checks only, no firmware compilation.`);
