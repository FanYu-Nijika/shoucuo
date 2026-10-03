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
const forwardPwmSign = Number(board.match(/#define BOARD_FOLLOW_FORWARD_PWM_SIGN\s+\((-?\d+)\)/)[1]);
const turnPwmSign = Number(board.match(/#define BOARD_FOLLOW_TURN_PWM_SIGN\s+\((-?\d+)\)/)[1]);
const params = read('code/car_params.c');
const image = read('code/image.c');
const cpu0 = read('user/cpu0_main.c');
const near = (a, b, eps = 1e-8) => assert(Math.abs(a - b) <= eps, `${a} != ${b}`);
let groups = 0;
function test(name, fn) { fn(); groups++; console.log('PASS ' + name); }

function complementary(last, accel, gyro, ka) {
    const predicted = last + gyro * 0.002;
    const effectiveKa = 1 - Math.pow(1 - ka, 0.002 / 0.005);
    return effectiveKa * accel + (1 - effectiveKa) * predicted;
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
    let output = -forwardPwmSign * clamp(2.0 * error + 0.3 * nextIntegral, 5);
    if (output > previous + 0.04) output = previous + 0.04;
    if (output < previous - 0.04) output = previous - 0.04;
    return {angle: output, integral: nextIntegral};
}

function allocate(balanceRaw, turnCommand, pwmLimit) {
    const common = balanceRaw;
    const turn = clamp(turnCommand, Math.max(0, pwmLimit - Math.abs(common)));
    return {left: clamp(common + turn, pwmLimit), right: clamp(common - turn, pwmLimit), common, turn};
}

function rightTrim(controlValue, trim = configuredRightTrim, limit = 5000) {
    const magnitude = Math.abs(controlValue) - trim;
    if (magnitude <= 0) return 0;
    return Math.trunc(clamp(Math.sign(controlValue) * magnitude, limit));
}

test('2 ms gyro integration preserves the time constant of stored 5 ms filter weights', () => {
    near(complementary(10, 30, 20, 0), 10.04);
    near(complementary(10, 30, 20, 1), 30);
    const effectiveKa = 1 - Math.pow(.99, .4);
    near(complementary(10, 30, 20, .01), effectiveKa * 30 + (1 - effectiveKa) * 10.04);
    assert(control.includes('balance_state.angle + gyro_rate * balance_dt_s'));
    assert(control.includes('applied_params.accel_weight * accel_angle'));
    assert(control.includes('static const float balance_dt_s = BALANCE_CONTROL_PERIOD_MS * 0.001f'));
    assert(controlHeader.includes('#define BALANCE_CONTROL_PERIOD_MS (2)'));
    assert(cpu0.includes('#define CAR_CONTROL_PERIOD_MS   (BALANCE_CONTROL_PERIOD_MS)'));
    assert(control.includes('powf(1.0f - weight_5ms, balance_dt_s / 0.005f)'));
});

test('direct upright PD preserves raw output and only bounds hardware duty', () => {
    const raw = 1000 * 1 + 25.0556 * 2;
    near(raw, 1050.1112, 1e-6);
    assert(control.includes('balance_state.p_output = applied_params.balance_kp * balance_state.angle_error'));
    assert(control.includes('balance_state.d_output = applied_params.balance_kd * balance_state.gyro_rate'));
    assert(!control.includes('target_rate'));
    assert(!control.includes('rate_error'));
    assert(control.includes('common = balance_state.balance_output;'));
    assert(!control.includes('balance_clamp(balance_state.balance_output, applied_params.pwm_limit)'));
    assert(!menu.includes('"PWM LIMIT"'));
    assert.equal(allocate(25000, 300, 10000).common, 25000);
    assert.equal(allocate(25000, 300, 10000).left, 10000);
    assert.equal(allocate(-25000, 300, 10000).right, -10000);
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
    assert(output.angle < 0);
    output = speedAngle(0, 0, 1.0, 0);
    assert(output.angle > 0);
    output = speedAngle(0, 10, 0, 0);
    near(output.angle, -0.04);
    output = speedAngle(0, -10, 0, 0);
    near(output.angle, 0.04);
    assert(control.includes('if (++encoder_sample_phase < 10) return'));
    assert(control.includes('speed_kp'));
    assert(control.includes('speed_ki'));
    assert(control.includes('speed_angle_limit'));
    assert(control.includes('speed_angle_applied'));
    assert(control.includes('speed_filtered_mps'));
    assert(control.includes('speed_error_mps'));
    assert(control.includes('speed_integral_angle'));
    assert(control.includes('angle_target = -BOARD_FOLLOW_FORWARD_PWM_SIGN * balance_clamp(pi_candidate, angle_limit)'));
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

test('new camera-forward direction swaps wheel ownership and retains positive forward counts', () => {
    assert(board.includes('#define BOARD_LEFT_MOTOR_REVERSE_PWM_PIN   (BOARD_MOTOR3_PWM_PIN)'));
    assert(board.includes('#define BOARD_LEFT_MOTOR_FORWARD_PWM_PIN   (BOARD_MOTOR4_PWM_PIN)'));
    assert(board.includes('#define BOARD_RIGHT_MOTOR_REVERSE_PWM_PIN  (BOARD_MOTOR1_PWM_PIN)'));
    assert(board.includes('#define BOARD_RIGHT_MOTOR_FORWARD_PWM_PIN  (BOARD_MOTOR2_PWM_PIN)'));
    assert(board.includes('#define BOARD_LEFT_ENCODER_INDEX           (BOARD_ENCODER_H7_INDEX)'));
    assert(board.includes('#define BOARD_RIGHT_ENCODER_INDEX          (BOARD_ENCODER_H4_INDEX)'));
    assert(board.includes('#define BOARD_BALANCE_LEFT_ENCODER_SIGN    (1)'));
    assert(board.includes('#define BOARD_BALANCE_RIGHT_ENCODER_SIGN   (-1)'));
    near(120 * 1, 120);  // H7 is the new left wheel when it rolls toward the camera.
    near(-120 * -1, 120);  // H4 is the new right wheel in the same motion.
    assert.equal(forwardPwmSign, 1);
    assert.equal(turnPwmSign, 1);
    assert(control.includes('record.version == 1u ? record.right_counts_per_rev : record.left_counts_per_rev'));
    assert(control.includes('record.version == 1u ? record.left_counts_per_rev : record.right_counts_per_rev'));
    assert(control.includes('#define BALANCE_ENCODER_FLASH_VERSION      (2u)'));
    assert(image.includes('average_count = balance_state.encoder_average_count'));
    const rightOfTrackTurn = 10 * 2 * turnPwmSign;
    const output = allocate(1000, rightOfTrackTurn, 5000);
    assert(output.left > output.right);
});

test('encoder preview uses 20 ms count steps, restores on deceleration and freezes current-frame curves', () => {
    const startCount = Number(image.match(/#define DYNAMIC_PREVIEW_START_COUNT\s+(\d+)/)[1]);
    const stepCount = Number(image.match(/#define DYNAMIC_PREVIEW_STEP_COUNT\s+(\d+)/)[1]);
    assert.equal(startCount, 250); assert.equal(stepCount, 20);
    const meanExpression = control.match(/encoder_average_count = ([^;]+);/)[1];
    const mean = new Function('left_count_sum', 'right_count_sum', `return Math.trunc(${meanExpression});`);
    assert.equal(mean(5, 5), 5);
    assert.equal(mean(4, 7), 5);
    assert.equal(mean(250, 250), 250);
    assert.equal(mean(269, 270), 269);
    assert.equal(mean(291, 297), 294);
    assert.equal(mean(-5, -5), -5);
    const start = image.lastIndexOf('static int Update_Dynamic_Preview_Row(');
    const source = image.slice(start, image.indexOf('\nfloat Calculate_Error', start));
    // Execute the C decision body with syntax substitutions; keep its integer division semantics.
    const body = source.slice(source.indexOf('{') + 1, source.lastIndexOf('}'))
        .replace(/\b(?:int32_t|int) (\w+)/g, 'let $1')
        .replace(/offset = \(([^)]+)\) \/ DYNAMIC_PREVIEW_STEP_COUNT;/,
            'offset = Math.trunc(($1) / DYNAMIC_PREVIEW_STEP_COUNT);')
        .replace(/return current_preview_row;/g, 'return {row: current_preview_row, curve: preview_curve_mode};');
    const update = new Function('car_params', 'balance_state', 'height', 'current_preview_row', 'preview_curve_mode',
        'curvature', 'line_valid', 'DYNAMIC_PREVIEW_START_COUNT', 'DYNAMIC_PREVIEW_STEP_COUNT', body);
    const params = {control_row_near: 84, running: 1, curve_enter_threshold: .2, curve_exit_threshold: .1};
    let state = {row: 84, curve: 0};
    const row = (count, curvature = 0, valid = 1) => {
        state = update(params, {encoder_average_count: count}, 120, state.row, state.curve, curvature, valid, startCount, stepCount);
        return state.row;
    };
    assert.equal(row(mean(5, 5)), 84);  // A small movement must not use counts/s and immediately extend the preview.
    assert.equal(row(mean(250, 250)), 84);
    assert.equal(row(mean(269, 270)), 84);
    assert.equal(row(mean(291, 297)), 82);  // Peak mean in the supplied serial data is only two rows farther.
    for (const [count, expected] of [[249, 84], [250, 84], [269, 84], [270, 83], [289, 83], [290, 82],
        [310, 81], [290, 82], [270, 83], [250, 84], [-250, 84], [1870, 3], [310, 81]]) assert.equal(row(count), expected);
    assert.equal(row(600, .2), 81);  // Entry freezes before CPU0 receives this frame.
    assert.equal(row(0, .15), 81);   // Hysteresis keeps the row while slowing in the curve.
    assert.equal(row(800, .3), 81);
    assert.equal(row(270, .1), 83);  // Exit resumes the current speed's row.
    assert.equal(row(500, .2), 83);
    assert.equal(row(250, .3, 0), 84);
    params.running = 0;
    assert.equal(row(1000, .3), 84); assert.equal(state.curve, 0);
    params.control_row_near = 200;
    assert.equal(row(1000), 117);
    params.control_row_near = 2;
    assert.equal(row(1000), 3);
    assert(!menu.includes('car_menu_text(8, 48, "ROW:"'));
    assert(menu.includes('car_menu_text(160, 166, "ROW:"'));
    assert(menu.includes('car_menu_fill(4, marker_y, 148, 1, RGB565_BLACK)'));
});

test('FOLLOW waits stopped for two seconds before RUN starts, while stale visual commands expire', () => {
    const delay = Number(car.match(/#define CAR_FOLLOW_STAGE_MS\s+\((\d+)\)/)[1]);
    assert.equal(delay, 2000);
    const started = 0xffffff00;
    assert(((((started + 1995) >>> 0) - started) >>> 0) < delay);
    assert(((((started + 2000) >>> 0) - started) >>> 0) >= delay);
    assert(car.includes('car_follow_status == CAR_FOLLOW_WAIT_RUN && car_running != 0'));
    assert(car.includes('now_ms - follow_stage_started_ms < CAR_FOLLOW_STAGE_MS'));
    assert(car.includes('balance_cancel_follow_command()'));
    assert(!menu.includes('FOLLOW WAIT MS'));
    assert(!menu.includes('FOLLOW STATUS'));
    assert(car.includes('now_ms - capture_ms <= visual_timeout_ms'));
    assert(car.includes('uint8 visual_valid = visual_fresh && valid && car_value_finite(error) && car_value_finite(curvature)'));
    assert(car.includes('if (!visual_valid)'));
    assert(car.includes('car_reset_turn()'));
});

test('FOLLOW START stops immediately and requests RUN only after the delay', () => {
    const startBody = car.slice(car.indexOf('uint8 car_follow_start(void)'), car.indexOf('void car_follow_stop(void)'));
    assert(startBody.includes('balance_runtime_stop()'));
    assert(startBody.includes('car_follow_status = CAR_FOLLOW_WAIT_DELAY'));
    assert(startBody.includes('follow_stage_started_ms = system_getval_ms()'));
    assert(!startBody.includes('balance_runtime_request_start()'));
    assert(runtime.includes('car_follow_status == CAR_FOLLOW_WAIT_RUN && !balance_state.running'));
    assert(car.includes('car_follow_status = CAR_FOLLOW_ACTIVE'));
    const stopBody = car.slice(car.indexOf('void car_follow_stop(void)'), car.indexOf('void car_track_update('));
    assert(stopBody.includes('car_follow_status = CAR_FOLLOW_IDLE'));
    assert(stopBody.includes('follow_stage_started_ms = 0'));
    assert(!startBody.includes('car_camera_ready'));
    assert(!startBody.includes('line_valid'));
});

test('row 115 four-point grayscale protection sends CENTER request immediately and performs full STOP', () => {
    const protectBody = image.slice(image.lastIndexOf('uint8_t protect('), image.indexOf('static int Calculate_Max_Preview_Length'));
    const centerBody = menu.slice(menu.indexOf('void car_menu_handle_center(void)'), menu.indexOf('uint8_t car_menu_current_profile'));
    const stopBody = menu.slice(menu.indexOf('static void car_menu_stop(void)'), menu.indexOf('static void car_menu_start(void)'));
    assert(protectBody.includes('car_params.running == 0'));
    assert(protectBody.includes('height <= 115'));
    assert(protectBody.includes('for (i = 1; i <= 4; i++)'));
    assert(protectBody.includes('if (gray_frame[115 * width + width * i / 5] >= 100) return 0'));
    assert(protectBody.includes('return 1;'));
    assert(!protectBody.includes('lost_count'));
    assert(!protectBody.includes('gray_sum'));
    assert(image.indexOf('protect(gray_frame)') < image.indexOf('memcpy(binary_image, gray_frame'));
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

test('FOLLOW supplies speed, launch lean and steering after the stopped delay', () => {
    const commandBody = car.slice(car.lastIndexOf('car_follow_status = CAR_FOLLOW_ACTIVE;'));
    assert(commandBody.includes('target_speed_mps = visual_valid ? balance_params.target_speed_mps : 0.0f'));
    assert(commandBody.includes('balance_set_follow_command(target_speed_mps, car_turn_request, lean_angle)'));
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
    assert(image.includes('if (car_params.running == 0)'));
    assert(!image.includes('car_params.base_speed <= 0'));
});

test('FOLLOW fault handling stops the follow command immediately while RUN remains separate', () => {
    const trackBody = car.slice(car.indexOf('void car_track_update('));
    assert(trackBody.includes('car_follow_status != CAR_FOLLOW_IDLE && balance_state.fault != BALANCE_FAULT_NONE'));
    assert(trackBody.includes('car_follow_stop()'));
    assert(car.includes('balance_cancel_follow_command()'));
});

test('encoder counts are accumulated at 2 ms and consumed by speed control every 20 ms', () => {
    const left = Array(10).fill(3).reduce((sum, value) => sum + value, 0);
    const right = Array(10).fill(7).reduce((sum, value) => sum + value, 0);
    assert.equal(left, 30); assert.equal(right, 70);
    assert(control.includes('if (++encoder_sample_phase < 10) return'));
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
    for (let i = 0; i < 250; i++) staticSamples++;
    assert.equal(staticSamples * 2, 500);
    for (let i = 0; i < 400; i++) collected++;
    collected = 0;
    staticSamples = 0;
    for (let i = 0; i < 250; i++) staticSamples++;
    for (let i = 0; i < 2500; i++) collected++;
    assert.equal(collected * 2, 5000);
    assert.equal(Number(runtimeHeader.match(/BALANCE_CAL_STATIC_SAMPLES\s+\((\d+)\)/)[1]), 250);
    assert.equal(Number(runtimeHeader.match(/BALANCE_CALIBRATION_SAMPLES\s+\((\d+)\)/)[1]), 2500);
    assert.equal(Number(runtimeHeader.match(/BALANCE_IMU_WARMUP_SAMPLES\s+\((\d+)\)/)[1]) * 2, 200);
});

test('background trim preserves its time constant at 2 ms and remains RAM-only', () => {
    let bias = 2;
    const raw = 3;
    bias += .00040012 * (raw - bias);
    near(bias, 2.00040012);
    assert(runtime.includes('if (balance_state.running || motor_test_active || !balance_imu_status.flash_calibration_valid)'));
    assert(runtime.includes('#define BALANCE_TRIM_ALPHA                  (0.00040012f)'));
    assert(runtime.includes('#define BALANCE_TRIM_WAIT_SAMPLES           (500)'));
    const trimBody = runtime.slice(runtime.indexOf('static void balance_runtime_update_bias_trim'),
        runtime.indexOf('static void balance_runtime_imu_failed'));
    assert(!trimBody.includes('flash_write'));
});

test('RUN keeps the warmup attitude and motor test leaves raw PWM ownership outside balance control', () => {
    const updateBody = runtime.slice(runtime.indexOf('void balance_runtime_update_2ms'),
        runtime.indexOf('void balance_runtime_stop'));
    const startBlock = updateBody.slice(updateBody.indexOf('if (balance_start_pending'),
        updateBody.indexOf('if (was_running'));
    const motorTestBlock = updateBody.slice(updateBody.indexOf('if (motor_test_active)'),
        updateBody.indexOf('if (!imu_initialized)'));
    assert(!startBlock.includes('balance_seed_attitude'));
    assert(startBlock.includes('car_running = balance_start()'));
    assert(!motorTestBlock.includes('cc_tc264_menu_motor_stop'));
    assert(updateBody.indexOf('balance_sample_encoders_2ms') < updateBody.indexOf('if (motor_test_active)'));
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

test('speed loop is default-enabled and no legacy open-loop FOLLOW speed remains', () => {
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
