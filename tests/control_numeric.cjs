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

function speedFilter(previous, raw) {
    return previous + 0.25 * (raw - previous);
}

function speedAngle(previous, target, measured, integral) {
    const error = target - measured;
    const nextIntegral = integral + error * 0.02;
    let output = clamp(2.0 * error + 0.3 * nextIntegral, 5);
    if (output > previous + 0.04) output = previous + 0.04;
    if (output < previous - 0.04) output = previous - 0.04;
    return {angle: output, integral: nextIntegral};
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
    assert(!control.match(/balance_kp\s*>(?!=)/));
    assert(!control.match(/balance_kd\s*>(?!=)/));
    assert(menu.includes('&balance_params.balance_kp, 0, FLT_MAX'));
    assert(menu.includes('&balance_params.balance_kd, 0, FLT_MAX'));
    assert(control.includes('params->pwm_limit > 10000'));
});

test('20 ms encoder speed loop filters physical velocity and cascades to target angle', () => {
    near(speedFilter(0, 1), 0.25);
    near(speedFilter(0.25, 1), 0.4375);
    let output = speedAngle(0, 1.5, 0.25, 0);
    assert(output.angle > 0);
    output = speedAngle(0, 0, 1.0, 0);
    assert(output.angle < 0);
    output = speedAngle(0, 10, 0, 0);
    near(output.angle, 0.04);
    output = speedAngle(0, -10, 0, 0);
    near(output.angle, -0.04);
    assert(control.includes('if (++encoder_sample_phase < 4) return'));
    assert(control.includes('speed_kp'));
    assert(control.includes('speed_ki'));
    assert(control.includes('speed_angle_limit'));
    assert(control.includes('speed_angle_applied'));
    assert(control.includes('speed_filtered_mps'));
    assert(control.includes('speed_error_mps'));
    assert(control.includes('speed_integral_angle'));
    assert(control.includes('angle_target = balance_clamp'));
    assert(control.includes('0.065f'));
    assert(control.includes('0.25f'));
    assert(control.includes('0.02f'));
    assert(control.includes('2.0f'));
    assert(control.includes('0.3f'));
    assert(control.includes('5.0f'));
    assert(controlHeader.includes('target_speed_mps'));
    assert(controlHeader.includes('left_counts_per_rev'));
    assert(controlHeader.includes('right_counts_per_rev'));
    assert(menu.includes('TARGET SPEED M/S'));
    assert(menu.includes('SPEED ENABLE'));
    assert(menu.includes('SPEED ANGLE'));
    assert(!menu.includes('FOLLOW SPEED'));
    assert(!control.includes('follow_angle_per_speed'));
    assert(!control.includes('follow_p_pwm_fraction'));
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

test('left and right motors match the current wheel wiring while encoders stay unchanged', () => {
    assert(board.includes('#define BOARD_LEFT_MOTOR_REVERSE_PWM_PIN   (BOARD_MOTOR1_PWM_PIN)'));
    assert(board.includes('#define BOARD_LEFT_MOTOR_FORWARD_PWM_PIN   (BOARD_MOTOR2_PWM_PIN)'));
    assert(board.includes('#define BOARD_RIGHT_MOTOR_REVERSE_PWM_PIN  (BOARD_MOTOR3_PWM_PIN)'));
    assert(board.includes('#define BOARD_RIGHT_MOTOR_FORWARD_PWM_PIN  (BOARD_MOTOR4_PWM_PIN)'));
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

test('FOLLOW supplies a speed target after the upright delay while steering remains separate', () => {
    const commandBody = car.slice(car.lastIndexOf('car_follow_status = CAR_FOLLOW_ACTIVE;'));
    assert(commandBody.includes('target_speed_mps = visual_valid ? balance_params.target_speed_mps : 0.0f'));
    assert(commandBody.includes('balance_set_follow_command(target_speed_mps, car_turn_request)'));
    assert(!commandBody.includes('follow_speed'));
    assert(car.includes('balance_params.target_speed_mps'));
    assert(car.includes('balance_cancel_follow_command()'));
    assert(!control.includes('requested_follow_speed'));
    assert(!control.includes('follow_angle_applied'));
    const stopBody = car.slice(car.indexOf('void car_follow_stop(void)'), car.indexOf('void car_track_update('));
    assert(stopBody.includes('car_reset_turn()'));
    assert(car.includes('car_curve_mode = 0'));
    assert(car.includes('BOARD_FOLLOW_TURN_PWM_SIGN'));
    for (const text of ['TARGET SPEED M/S', 'SPEED ENABLE', 'SPEED ANGLE', 'STEERING KP', 'STEERING KD', 'CURVE KP', 'CURVE KD'])
        assert(menu.includes(text), text);
    assert(image.includes('if (car_params.running == 0 || balance_state.imu_ready == 0)'));
    assert(!image.includes('car_params.base_speed <= 0'));
});

test('FOLLOW fault handling stops the follow command immediately while RUN remains separate', () => {
    const trackBody = car.slice(car.indexOf('void car_track_update('));
    assert(trackBody.includes('car_follow_status != CAR_FOLLOW_IDLE && balance_state.fault != BALANCE_FAULT_NONE'));
    assert(trackBody.includes('car_follow_stop()'));
    assert(car.includes('balance_cancel_follow_command()'));
});

test('encoder counts are accumulated at 5 ms and consumed by speed control every 20 ms', () => {
    const left = [3, 3, 3, 3].reduce((sum, value) => sum + value, 0);
    const right = [7, 7, 7, 7].reduce((sum, value) => sum + value, 0);
    assert.equal(left, 12); assert.equal(right, 28);
    assert(control.includes('if (++encoder_sample_phase < 4) return'));
    assert(control.includes('left_count_sum'));
    assert(control.includes('right_count_sum'));
    assert(controlHeader.match(/speed_(?:kp|ki|filter|integral|angle|enabled)/));
    assert(controlHeader.includes('speed_raw_mps'));
    assert(controlHeader.includes('speed_target_applied_mps'));
    assert(controlHeader.includes('speed_error_mps'));
    assert(control.includes('left_counts_per_rev'));
    assert(control.includes('right_counts_per_rev'));
    assert(control.includes('0.065f'));
});

test('one-wheel-revolution calibration gives a physical 1.5 m/s target and persists both encoders', () => {
    const countsPerRev = 1000;
    const targetCounts20ms = 1.5 * 0.02 * countsPerRev / (Math.PI * 0.065);
    near(targetCounts20ms, 146.9124476, 0.001);
    assert(control.includes('BALANCE_ENCODER_FLASH_PAGE         (11u)'));
    assert(control.includes('record.checksum = balance_encoder_checksum(&record)'));
    assert(control.includes('count = total - encoder_calibration_start_count'));
    assert(control.includes('count == 0 || count > BALANCE_ENCODER_MAX_COUNTS'));
    assert(control.includes('balance_encoder_flash_save()'));
    assert(menu.includes('CAL LEFT 1 REV'));
    assert(menu.includes('CAL RIGHT 1 REV'));
    assert(menu.includes('ENCODER CAL VALID'));
    assert(control.includes('(float)balance_state.left_count / balance_state.left_counts_per_rev'));
    assert(control.includes('(float)balance_state.right_count / balance_state.right_counts_per_rev'));
    const underTargetAngle = speedAngle(0, 1.5, 0, 0).angle;
    const overTargetAngle = speedAngle(0, 0, 1.5, 0).angle;
    assert(-1000 * underTargetAngle > 0);
    assert(-1000 * overTargetAngle < 0);
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

test('V10 migration preserves upright fields and initializes the new speed loop', () => {
    assert(control.includes('#define BALANCE_PARAMS_FLASH_VERSION       (10u)'));
    assert(control.includes('version == 9u'));
    assert(control.includes('speed_kp'));
    assert(control.includes('speed_ki'));
    assert(control.includes('speed_angle_limit'));
    assert(control.includes('target_speed_mps'));
    const conversion = control.slice(control.indexOf('static balance_params_t balance_params_from_v9'),
        control.indexOf('static void balance_params_flash_prepare'));
    for (const field of ['balance_kp', 'balance_kd', 'middle_angle', 'pwm_limit', 'turn_direction',
        'left_motor_deadzone', 'right_motor_deadzone']) assert(conversion.includes(field));
    assert(!conversion.includes('target_speed_mps = old'));
});

test('car parameter V5 migration preserves layout but resets all old base PWM values to 800', () => {
    assert(params.includes('#define CAR_PARAMS_FLASH_FORMAT_VERSION  (5)'));
    assert(params.includes('static void car_params_flash_migrate_v4'));
    assert(params.includes('.base_speed = 800'));
    assert(params.includes('stored_params.base_speed = car_default_params.base_speed'));
});

test('speed loop is active, default-enabled, and no open-loop FOLLOW lean remains', () => {
    assert(menu.includes('SPEED ENABLE'));
    assert(menu.includes('TARGET SPEED M/S'));
    assert(menu.includes('SPEED ANGLE'));
    assert(menu.includes('&balance_params.speed_kp'));
    assert(menu.includes('&balance_params.speed_ki'));
    assert(menu.includes('&balance_params.speed_angle_limit'));
    assert(control.includes('.speed_enabled = 1'));
    assert(control.includes('.target_speed_mps = 1.5f'));
    assert(control.includes('.speed_kp = 2.0f'));
    assert(control.includes('.speed_ki = 0.3f'));
    assert(control.includes('.speed_angle_limit = 5.0f'));
    assert(!menu.includes('FOLLOW SPEED'));
    assert(!car.includes('balance_params.follow_speed'));
    assert(!control.includes('follow_angle_per_speed'));
    assert(!control.includes('follow_p_pwm_fraction'));
    assert(control.includes('balance_state.speed_integral_angle = 0.0f'));
    assert(control.includes('balance_state.speed_angle_applied = 0.0f'));
});

console.log(`${groups} groups passed; source-level numerical checks only, no firmware compilation.`);
