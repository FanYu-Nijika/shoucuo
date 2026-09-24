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
const board = read('code/board_pins.h');
const configuredRightTrim = Number(board.match(/#define BOARD_BALANCE_RIGHT_PWM_TRIM\s+\((\d+)\)/)[1]);
const params = read('code/car_params.c');
const image = read('code/image.c');
const cpu0 = read('user/cpu0_main.c');
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

function followAngleFromSpeed(speed) {
    return clamp(speed, 100) * 0.05;
}

function followAngleRamp(current, target) {
    const effectiveTarget = current * target < 0 ? 0 : target;
    const delta = effectiveTarget - current;
    if (Math.abs(delta) <= 1e-12) return target;
    const step = Math.abs(effectiveTarget) > Math.abs(current) ? 0.01 : 0.02;
    if (Math.abs(delta) <= step) return effectiveTarget;
    const next = current + Math.sign(delta) * step;
    if (current * target < 0 && current * next <= 0) return effectiveTarget;
    return next;
}

function followBalanceOutput(angleError, gyroRate, kp, kd, pwmLimit, active) {
    const pLimit = active ? pwmLimit * 0.6 : pwmLimit;
    return clamp(kp * angleError, pLimit) + kd * gyroRate;
}

function allocate(balanceRaw, turnCommand, pwmLimit) {
    const common = clamp(balanceRaw, pwmLimit);
    const turn = clamp(turnCommand, pwmLimit - Math.abs(common));
    return {left: common + turn, right: common - turn, common, turn};
}

function rightTrim(controlValue, trim = configuredRightTrim, limit = 5000) {
    const magnitude = Math.abs(controlValue) - trim;
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
    assert(control.includes('balance_state.p_output = applied_params.balance_kp * balance_state.angle_error'));
    assert(control.includes('balance_state.d_output = applied_params.balance_kd * balance_state.gyro_rate'));
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

test('FOLLOW SPEED maps -100..100 to -5..5 degrees and keeps the command open loop', () => {
    near(followAngleFromSpeed(-100), -5);
    near(followAngleFromSpeed(0), 0);
    near(followAngleFromSpeed(100), 5);
    near(followAngleFromSpeed(250), 5);
    near(followAngleFromSpeed(-250), -5);
    assert(control.includes('.follow_speed = 0.0f'));
    assert(control.includes('follow_angle_per_speed = 0.05f'));
    assert(control.includes('params->follow_speed < -100.0f || params->follow_speed > 100.0f'));
    assert(control.includes('balance_set_follow_command(float follow_speed'));
    assert(control.includes('balance_cancel_follow_command(void)'));
    assert(control.includes('.turn_pwm_limit = 1400'));
    assert(control.includes('BOARD_FOLLOW_FORWARD_PWM_SIGN * balance_state.follow_angle_applied'));
    assert(control.includes('common = balance_clamp(balance_state.balance_output, applied_params.pwm_limit)'));
    assert(control.includes('requested_follow_speed = 0.0f'));
    assert(control.includes('balance_state.target_angle = applied_params.middle_angle'));
    assert(menu.includes('"FOLLOW SPEED"'));
    assert(menu.includes('&balance_params.follow_speed, -100, 100, 1, 0'));
    assert(!menu.includes('"FOLLOW ANGLE", CAR_MENU_ITEM_VALUE'));
    assert(!control.includes('requested_base_pwm'));
    assert(!controlHeader.includes('base_pwm_applied'));
});

test('FOLLOW angle ramp uses 2 degrees/s up, 4 degrees/s down, and passes through zero', () => {
    near(followAngleRamp(0, 5), 0.01);
    near(followAngleRamp(0.03, 5), 0.04);
    near(followAngleRamp(0, -5), -0.01);
    near(followAngleRamp(-0.03, -5), -0.04);
    near(followAngleRamp(0.03, -5), 0.01);
    near(followAngleRamp(0.01, -5), 0);
    assert(control.includes('follow_angle_rise_step = 0.01f'));
    assert(control.includes('follow_angle_fall_step = 0.02f'));
});

test('FOLLOW limits the posture P term to 60 percent of PWM LIMIT before adding D, while RUN keeps full P', () => {
    near(followBalanceOutput(10, 0, 1000, 25.0556, 5000, true), 3000);
    near(followBalanceOutput(-10, 0, 1000, 25.0556, 5000, true), -3000);
    near(followBalanceOutput(10, 20, 1000, 25.0556, 5000, true), 3501.112);
    near(followBalanceOutput(10, 0, 1000, 25.0556, 5000, false), 5000);
    assert(control.includes('follow_p_pwm_fraction = 0.6f'));
    assert(control.includes('applied_params.pwm_limit * follow_p_pwm_fraction'));
    assert(control.includes('balance_state.p_output + balance_state.d_output'));
});

test('camera PWM differential never steals common balance headroom', () => {
    let out = allocate(4500, 1000, 5000);
    near(out.left, 5000); near(out.right, 4000); near(out.turn, 500);
    out = allocate(-4800, -1000, 5000);
    near(out.left, -5000); near(out.right, -4600); near(out.turn, -200);
    out = allocate(6000, 500, 5000);
    near(out.left, 5000); near(out.right, 5000); near(out.turn, 0);
});

test('configured new-right PWM trim is symmetric and motor tests stay outside balance compensation', () => {
    const trim = configuredRightTrim;
    for (const [input, left, right] of [[0, 0, 0], [trim, trim, 0], [800, 800, 800 - trim],
        [2000, 2000, 2000 - trim], [-trim, -trim, 0], [-800, -800, trim - 800], [-2000, -2000, trim - 2000]]) {
        assert.equal(input, left);
        assert.equal(rightTrim(input), right);
    }
    assert(control.includes('BOARD_BALANCE_RIGHT_PWM_TRIM'));
    assert(!menu.includes('MOTOR DEADZONE'));
});

test('new logical wheels use old opposite-side motor and encoder resources', () => {
    assert(board.includes('#define BOARD_LEFT_MOTOR_REVERSE_PWM_PIN   (BOARD_MOTOR3_PWM_PIN)'));
    assert(board.includes('#define BOARD_RIGHT_MOTOR_REVERSE_PWM_PIN  (BOARD_MOTOR1_PWM_PIN)'));
    assert(board.includes('#define BOARD_LEFT_ENCODER_INDEX           (BOARD_ENCODER_H4_INDEX)'));
    assert(board.includes('#define BOARD_RIGHT_ENCODER_INDEX          (BOARD_ENCODER_H7_INDEX)'));
    assert(board.includes('#define BOARD_BALANCE_LEFT_ENCODER_SIGN    (1)'));
    assert(board.includes('#define BOARD_BALANCE_RIGHT_ENCODER_SIGN   (-1)'));
});

test('FOLLOW holds upright for two seconds after RUN starts, while stale visual commands expire', () => {
    const delay = Number(car.match(/#define CAR_FOLLOW_DELAY_MS\s+\((\d+)\)/)[1]);
    assert.equal(delay, 2000);
    const started = 0xffffff00;
    assert(((((started + 1995) >>> 0) - started) >>> 0) < delay);
    assert(((((started + 2000) >>> 0) - started) >>> 0) >= delay);
    assert(car.includes('car_follow_status == CAR_FOLLOW_WAIT_RUN && car_running != 0'));
    assert(car.includes('now_ms - follow_upright_started_ms >= CAR_FOLLOW_DELAY_MS'));
    assert(car.includes('balance_cancel_follow_command()'));
    assert(!menu.includes('FOLLOW WAIT MS'));
    assert(!menu.includes('FOLLOW STATUS'));
    assert(car.includes('now_ms - capture_ms <= visual_timeout_ms'));
    assert(car.includes('uint8 visual_valid = visual_fresh && valid && car_value_finite(error) && car_value_finite(curvature)'));
    assert(car.includes('if (!visual_valid)'));
    assert(car.includes('car_reset_turn()'));
});

test('FOLLOW START requests RUN before enabling vision control', () => {
    const startBody = car.slice(car.indexOf('uint8 car_follow_start(void)'), car.indexOf('void car_follow_stop(void)'));
    assert(startBody.includes('car_follow_enabled = 0'));
    assert(startBody.includes('car_follow_status = CAR_FOLLOW_WAIT_RUN'));
    assert(startBody.includes('if (car_running == 0) balance_runtime_request_start()'));
    assert(runtime.includes('car_follow_status != CAR_FOLLOW_IDLE && !balance_state.running'));
    assert(car.includes('car_follow_status = CAR_FOLLOW_ACTIVE'));
    const stopBody = car.slice(car.indexOf('void car_follow_stop(void)'), car.indexOf('void car_track_update('));
    assert(stopBody.includes('car_follow_status = CAR_FOLLOW_IDLE'));
    assert(stopBody.includes('follow_upright_started_ms = 0'));
    assert(!startBody.includes('car_camera_ready'));
    assert(!startBody.includes('line_valid'));
});

test('bottom-row protection sends CENTER request and performs full STOP', () => {
    const protectBody = image.slice(image.indexOf('uint8_t protect('), image.indexOf('static int Calculate_Max_Preview_Length'));
    const centerBody = menu.slice(menu.indexOf('void car_menu_handle_center(void)'), menu.indexOf('uint8_t car_menu_current_profile'));
    const stopBody = menu.slice(menu.indexOf('static void car_menu_stop(void)'), menu.indexOf('static void car_menu_start(void)'));
    assert(protectBody.includes('os_threshold < 60'));
    assert(protectBody.includes('binary_image[row * width + i] == 0'));
    assert(protectBody.includes('threshold_lost != 0 || sum >= width - 18'));
    assert(image.includes('protect(gray_frame) != 0) car_center_stop_request = 1'));
    assert(cpu0.includes('car_menu_handle_center()'));
    assert(centerBody.includes('car_menu_stop()'));
    assert(stopBody.includes('car_stop()'));
    assert(stopBody.includes('cc_tc264_menu_motor_stop()'));
});

test('runtime STOP and image CENTER request clear FOLLOW while status prioritizes FOLLOW over RUN', () => {
    assert.equal((runtime.match(/car_follow_stop\(\)/g) || []).length, 1);
    const runtimeStop = runtime.slice(runtime.indexOf('void balance_runtime_stop(void)'),
        runtime.indexOf('void balance_runtime_set_motor_test'));
    assert(runtimeStop.includes('car_follow_stop()'));
    const requestStart = runtime.slice(runtime.indexOf('void balance_runtime_request_start(void)'),
        runtime.indexOf('uint8 balance_runtime_request_calibration'));
    assert(!requestStart.includes('car_follow_stop'));
    const statusBody = menu.slice(menu.indexOf('static const char *car_menu_status_text(void)'),
        menu.indexOf('static const char *car_menu_balance_fault_text(void)'));
    assert(statusBody.indexOf('car_follow_enabled') < statusBody.indexOf('car_running'));
    assert(statusBody.includes('return "FOLLOW"'));
});

test('valid FOLLOW keeps straight speed, scales curve speed to 20 percent, and clears stale vision', () => {
    near(60 * 0.2, 12);
    near(-60 * 0.2, -12);
    assert(car.includes('0.2f'));
    assert(car.includes('balance_params.follow_speed'));
    assert(car.includes('balance_cancel_follow_command()'));
    assert(car.includes('follow_speed = visual_valid ? balance_params.follow_speed : 0.0f'));
    assert(car.includes('BOARD_FOLLOW_TURN_PWM_SIGN'));
    for (const text of ['FOLLOW SPEED', 'FOLLOW ANGLE APPLIED', 'STEERING KP', 'STEERING KD', 'CURVE KP', 'CURVE KD'])
        assert(menu.includes(text), text);
    assert(!menu.includes('"FOLLOW ANGLE", CAR_MENU_ITEM_VALUE'));
    assert(image.includes('if (car_params.running == 0)'));
    assert(!image.includes('car_params.base_speed <= 0'));
});

test('FOLLOW fault handling stops the follow command immediately while RUN remains separate', () => {
    const trackBody = car.slice(car.indexOf('void car_track_update('));
    assert(trackBody.includes('car_follow_status != CAR_FOLLOW_IDLE && balance_state.fault != BALANCE_FAULT_NONE'));
    assert(trackBody.includes('car_follow_stop()'));
    assert(trackBody.includes('balance_cancel_follow_command()'));
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

test('V8 migration keeps upright fields and resets the old angle field to V9 speed zero', () => {
    assert(control.includes('#define BALANCE_PARAMS_FLASH_VERSION       (9u)'));
    assert(control.includes('typedef struct {'));
    assert(control.includes('balance_v8_follow_check'));
    assert(control.includes('version == 8u'));
    assert(control.includes('profiles[gear - 1u].follow_speed = 0.0f'));
    const conversion = control.slice(control.indexOf('static balance_params_t balance_params_from_v7'),
        control.indexOf('static void balance_params_flash_prepare'));
    for (const field of ['angle_kp', 'rate_kp', 'middle_angle', 'pwm_limit', 'turn_direction',
        'left_motor_deadzone', 'right_motor_deadzone']) assert(conversion.includes(field));
    for (const field of ['speed_kp', 'speed_ki', 'q_angle', 'q_bias', 'r_angle', 'rate_limit'])
        assert(!conversion.includes(`old->${field}`));
});

test('car parameter V5 migration preserves layout but resets all old base PWM values to 800', () => {
    assert(params.includes('#define CAR_PARAMS_FLASH_FORMAT_VERSION  (5)'));
    assert(params.includes('static void car_params_flash_migrate_v4'));
    assert(params.includes('.base_speed = 800'));
    assert(params.includes('stored_params.base_speed = car_default_params.base_speed'));
});

test('active menu/control contain no speed loop, Kalman, or rate cascade', () => {
    for (const text of ['SPEED ENABLE', 'SPEED KP', 'SPEED KI', 'SPEED FILTER', 'TARGET SPEED',
        'Q ANGLE', 'Q BIAS', 'R ANGLE', 'KALMAN ANGLE', 'RATE LIMIT', 'TARGET RATE', 'RATE ERROR', 'RATE OUTPUT']) {
        assert(!menu.includes(text), text);
    }
    assert(!car.includes('speed_enabled'));
    assert(!car.includes('target_speed'));
    assert(menu.includes('FOLLOW SPEED'));
    assert(menu.includes('&balance_params.follow_speed'));
    assert(!menu.includes('"FOLLOW ANGLE", CAR_MENU_ITEM_VALUE'));
    assert(car.includes('balance_params.follow_speed'));
});

console.log(`${groups} groups passed; source-level numerical checks only, no firmware compilation.`);
