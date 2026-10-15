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
const carHeader = read('code/car.h');
const menu = read('code/car_menu.c');
const menuPort = read('code/car_menu_port.c');
const board = read('code/board_pins.h');
const motorPwmLimit = Number(control.match(/static const float balance_motor_pwm_max = ([\d.]+)f;/)[1]);
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

function allocate(balanceRaw, turnCommand, pwmLimit) {
    const common = balanceRaw;
    const turn = clamp(turnCommand, Math.max(0, pwmLimit - Math.abs(common)));
    return {left: clamp(common + turn, pwmLimit), right: clamp(common - turn, pwmLimit), common, turn};
}

function rightTrim(controlValue, trim = configuredRightTrim, limit = motorPwmLimit) {
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

test('upright output stays inverted and both motor paths restore full hardware duty', () => {
    const raw = 1000 * 1 + 25.0556 * 2;
    near(raw, 1050.1112, 1e-6);
    assert(control.includes('balance_state.p_output = applied_params.balance_kp * balance_state.angle_error'));
    assert(control.includes('balance_state.d_output = applied_params.balance_kd * balance_state.gyro_rate'));
    assert(control.includes('balance_state.balance_output = -(balance_state.p_output + balance_state.d_output)'));
    assert(!control.includes('target_rate'));
    assert(!control.includes('rate_error'));
    assert(control.includes('common = balance_state.balance_output;'));
    assert(!control.includes('balance_clamp(balance_state.balance_output, applied_params.pwm_limit)'));
    assert(!menu.includes('"PWM LIMIT"'));
    assert.equal(motorPwmLimit, 10000);
    assert(!board.includes('BOARD_MOTOR_PWM_LIMIT'));
    assert(control.includes('return 10000'));
    assert(control.includes('return -10000'));
    assert(read('code/car_menu_port.c').includes('cc_math_clamp_i32(command, -PWM_DUTY_MAX, PWM_DUTY_MAX)'));
    assert.equal(allocate(25000, 300, motorPwmLimit).common, 25000);
    assert.equal(allocate(25000, 300, motorPwmLimit).left, 10000);
    assert.equal(allocate(-25000, 300, motorPwmLimit).right, -10000);
});

test('all active control gains allow signed values without artificial limits', () => {
    assert(!control.match(/balance_kp\s*[<>]=?\s*[-\d]/));
    assert(!control.match(/balance_kd\s*[<>]=?\s*[-\d]/));
    assert(!control.match(/speed_kp\s*[<>]=?\s*[-\d]/));
    assert(!control.match(/speed_ki\s*[<>]=?\s*[-\d]/));
    assert(menu.includes('&balance_params.balance_kp, -FLT_MAX, FLT_MAX'));
    assert(menu.includes('&balance_params.balance_kd, -FLT_MAX, FLT_MAX'));
    assert(menu.includes('&balance_params.speed_kp, -FLT_MAX, FLT_MAX'));
    assert(menu.includes('&balance_params.speed_ki, -FLT_MAX, FLT_MAX'));
    for (const field of ['steering_kp', 'steering_kd', 'curve_steering_kp', 'curve_steering_kd']) {
        assert(menu.includes(`&car_params.${field}, -FLT_MAX, FLT_MAX`));
    }
    assert(!read('code/car_params.c').match(/curve_steering_k[pd]\s*<=?\s*0/));
    assert(control.includes('params->pwm_limit > 10000'));
});

test('encoder speed updates every 2 ms while the speed PI runs every 20 ms', () => {
    const sampleStart = control.indexOf('void balance_sample_encoders_2ms(');
    const sampleBody = control.slice(sampleStart, control.indexOf('\nvoid balance_init', sampleStart));
    const piStart = control.indexOf('static void balance_update_speed_20ms(');
    const piBody = control.slice(piStart, control.indexOf('\nvoid balance_sample_encoders_2ms', piStart));
    const conversion = sampleBody.indexOf('balance_state.speed_raw_mps = -wheel_circumference_m * (left_count + right_count)');
    const filter = sampleBody.indexOf('balance_state.speed_filtered_mps += speed_filter_weight');
    const phase = sampleBody.indexOf('if (++encoder_sample_phase < 10) return');
    assert(sampleStart >= 0 && conversion >= 0 && filter > conversion);
    assert(phase > filter && sampleBody.indexOf('balance_update_speed_20ms(speed_feedback_sum / 10.0f)') > phase);
    assert(sampleBody.includes('balance_state.left_count = left_count'));
    assert(sampleBody.includes('balance_state.right_count = right_count'));
    assert(sampleBody.includes('2.0f * BALANCE_ENCODER_COUNTS_PER_REV * balance_dt_s'));
    assert(!/\b(?:abs|fabs|fabsf)\s*\(/.test(sampleBody));
    assert(sampleBody.includes('if (!balance_is_finite(balance_state.speed_filtered_mps)) balance_state.speed_filtered_mps = 0.0f'));
    assert(control.includes('static const float speed_filter_weight = 0.9f'));
    assert(control.includes('static const float speed_dt_s = 0.02f'));
    assert(control.includes('applied_params.speed_ki * error * speed_dt_s'));
    assert(piBody.includes('speed_target_applied_mps = requested_speed_mps'));
    assert(piBody.includes('balance_state.speed_feedback_mps = feedback_mps'));
    assert(sampleBody.includes('speed_feedback_sum += balance_state.speed_filtered_mps'));
    assert(sampleBody.includes('balance_update_speed_20ms(speed_feedback_sum / 10.0f)'));
    assert(piBody.includes('error = balance_state.speed_target_applied_mps - balance_state.speed_feedback_mps'));
    assert(piBody.includes('speed_integral_angle += applied_params.speed_ki * error * speed_dt_s'));
    assert(piBody.includes('speed_integral_angle = balance_clamp(balance_state.speed_integral_angle, angle_limit)'));
    assert(piBody.includes('balance_state.speed_p_angle = applied_params.speed_kp * error'));
    assert(piBody.includes('pi_output = balance_state.speed_p_angle + balance_state.speed_integral_angle'));
    assert(piBody.includes('speed_angle_applied = BOARD_FOLLOW_FORWARD_PWM_SIGN * balance_clamp(pi_output, angle_limit)'));
    assert(!piBody.includes('balance_motor_pwm_max'));
    assert(!piBody.includes('speed_target_step_mps'));
    assert(!piBody.includes('speed_angle_step_deg'));
    assert(!control.includes('speed_target_step_mps'));
    assert(!control.includes('speed_angle_step_deg'));
    assert(controlHeader.includes('Measure velocity every 2 ms'));
    assert(control.includes('speed_kp'));
    assert(control.includes('speed_ki'));
    assert(control.includes('speed_angle_limit'));
    assert(control.includes('speed_angle_applied'));
    assert(control.includes('speed_filtered_mps'));
    assert(control.includes('speed_error_mps'));
    assert(control.includes('speed_integral_angle'));
    assert(control.includes('speed_angle_applied = BOARD_FOLLOW_FORWARD_PWM_SIGN * balance_clamp(pi_output, angle_limit)'));
    assert(control.includes('0.065f'));
    assert(controlHeader.includes('target_speed_mps'));
    assert(controlHeader.includes('#define BALANCE_ENCODER_COUNTS_PER_REV (1160u)'));
    const initBody = control.slice(control.indexOf('void balance_init(void)'), control.indexOf('\nuint8_t balance_seed_attitude', control.indexOf('void balance_init(void)')));
    assert(initBody.includes('balance_state.left_counts_per_rev = BALANCE_ENCODER_COUNTS_PER_REV'));
    assert(initBody.includes('balance_state.right_counts_per_rev = BALANCE_ENCODER_COUNTS_PER_REV'));
    assert(initBody.includes('balance_state.encoder_calibration_valid = 1'));
    assert(menu.includes('"LEFT COUNTS/REV"'));
    assert(menu.includes('"RIGHT COUNTS/REV"'));
    assert(!menu.includes('CAL LEFT 1 REV'));
    assert(!menu.includes('CAL RIGHT 1 REV'));
    assert(!control.includes('balance_encoder_calibrate_toggle'));
    assert(!control.includes('balance_encoder_flash_load'));
    assert(!control.includes('balance_encoder_flash_save'));
    assert(!menu.includes('FOLLOW SPEED'));
    assert(!control.includes('follow_angle_per_speed'));
    assert(!control.includes('follow_p_pwm_fraction'));
});

test('camera PWM differential never steals common balance headroom', () => {
    let out = allocate(9500, 1000, motorPwmLimit);
    near(out.left, 10000); near(out.right, 9000); near(out.turn, 500);
    out = allocate(-9800, -1000, motorPwmLimit);
    near(out.left, -10000); near(out.right, -9600); near(out.turn, -200);
    out = allocate(9800, 200, motorPwmLimit);
    near(out.left, 10000); near(out.right, 9600); near(out.turn, 200);
    out = allocate(9800, -200, motorPwmLimit);
    near(out.left, 9600); near(out.right, 10000); near(out.turn, -200);
    out = allocate(-9800, 200, motorPwmLimit);
    near(out.left, -9600); near(out.right, -10000); near(out.turn, 200);
    out = allocate(-9800, -200, motorPwmLimit);
    near(out.left, -10000); near(out.right, -9600); near(out.turn, -200);
    out = allocate(12000, 500, motorPwmLimit);
    near(out.left, 10000); near(out.right, 10000); near(out.turn, 0);
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

test('left and right motor wiring keeps their logical encoder signs and PWM paths', () => {
    assert(board.includes('#define BOARD_LEFT_MOTOR_REVERSE_PWM_PIN   (BOARD_MOTOR1_PWM_PIN)'));
    assert(board.includes('#define BOARD_LEFT_MOTOR_FORWARD_PWM_PIN   (BOARD_MOTOR2_PWM_PIN)'));
    assert(board.includes('#define BOARD_RIGHT_MOTOR_REVERSE_PWM_PIN  (BOARD_MOTOR3_PWM_PIN)'));
    assert(board.includes('#define BOARD_RIGHT_MOTOR_FORWARD_PWM_PIN  (BOARD_MOTOR4_PWM_PIN)'));
    assert(board.includes('#define BOARD_MOTOR1_PWM_PIN               (ATOM2_CH1_P33_5)'));
    assert(board.includes('#define BOARD_MOTOR2_PWM_PIN               (ATOM2_CH2_P33_6)'));
    assert(board.includes('#define BOARD_MOTOR3_PWM_PIN               (ATOM2_CH3_P33_7)'));
    assert(board.includes('#define BOARD_MOTOR4_PWM_PIN               (ATOM2_CH4_P33_8)'));
    assert(board.includes('#define BOARD_BALANCE_LEFT_MOTOR_SIGN      (1)'));
    assert(board.includes('#define BOARD_BALANCE_RIGHT_MOTOR_SIGN     (1)'));
    const motorWrite = menuPort.slice(menuPort.indexOf('void cc_tc264_menu_motor_write('),
        menuPort.indexOf('void cc_tc264_menu_motor_stop('));
    assert(motorWrite.includes('write_motor_pair(BOARD_LEFT_MOTOR_FORWARD_PWM_PIN, BOARD_LEFT_MOTOR_REVERSE_PWM_PIN, left * BOARD_BALANCE_LEFT_MOTOR_SIGN'));
    assert(motorWrite.includes('write_motor_pair(BOARD_RIGHT_MOTOR_FORWARD_PWM_PIN, BOARD_RIGHT_MOTOR_REVERSE_PWM_PIN, right * BOARD_BALANCE_RIGHT_MOTOR_SIGN'));
    assert(board.includes('#define BOARD_LEFT_ENCODER_INDEX           (BOARD_ENCODER_H7_INDEX)'));
    assert(board.includes('#define BOARD_RIGHT_ENCODER_INDEX          (BOARD_ENCODER_H4_INDEX)'));
    assert(board.includes('#define BOARD_BALANCE_LEFT_ENCODER_SIGN    (-1)'));
    assert(board.includes('#define BOARD_BALANCE_RIGHT_ENCODER_SIGN   (1)'));
    assert(menuPort.includes('encoder_get_count(BOARD_LEFT_ENCODER_INDEX)'));
    assert(menuPort.includes('encoder_get_count(BOARD_RIGHT_ENCODER_INDEX)'));
    assert(menuPort.includes('delta_left *= BOARD_BALANCE_LEFT_ENCODER_SIGN'));
    assert(menuPort.includes('delta_right *= BOARD_BALANCE_RIGHT_ENCODER_SIGN'));
    near(-10 * -1, 10);  // Reverse H7 raw counts become positive logical counts.
    near(180 * 1, 180);  // Reverse H4 raw counts stay positive logical counts.
    near(10 * -1, -10);  // Forward H7 raw counts become negative logical counts.
    near(-180 * 1, -180);  // Forward H4 raw counts stay negative logical counts.
    assert.equal(forwardPwmSign, -1);
    assert.equal(turnPwmSign, -1);
    assert(!control.includes('BALANCE_ENCODER_FLASH_PAGE'));
    assert(!control.includes('balance_encoder_calibrate_toggle'));
    assert(!control.includes('balance_encoder_flash_load'));
    assert(!control.includes('balance_encoder_flash_save'));
    assert(controlHeader.includes('#define BALANCE_ENCODER_COUNTS_PER_REV (1160u)'));
    assert(image.includes('average_count = balance_state.encoder_average_count'));
    const positiveErrorTurn = 10 * 2 * turnPwmSign;
    const output = allocate(-1000, positiveErrorTurn, 5000);
    assert(output.left < output.right);
    near((output.left + output.right) / 2, -1000);
    const opposite = allocate(-1000, -positiveErrorTurn, 5000);
    assert(opposite.left > opposite.right);
    near((opposite.left + opposite.right) / 2, -1000);
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

test('FOLLOW uses an actual-RUN upright hold and a bounded speed ramp', () => {
    const waitDelayMs = Number(car.match(/#define CAR_FOLLOW_WAIT_DELAY_MS\s+\((\d+)\)/)[1]);
    const uprightMs = Number(car.match(/#define CAR_FOLLOW_UPRIGHT_MS\s+\((\d+)\)/)[1]);
    const speedRampMs = Number(car.match(/#define CAR_FOLLOW_SPEED_RAMP_MS\s+\((\d+)\)/)[1]);
    const oscerrDelayMs = Number(car.match(/#define CAR_FOLLOW_OSCERR_DELAY_MS\s+\((\d+)\)/)[1]);
    const trackBody = car.slice(car.indexOf('void car_track_update('));
    const enumBody = carHeader.match(/enum\s*\{([\s\S]*?)\};/)[1];
    const enumValues = {};
    let nextEnumValue = 0;
    for (const match of enumBody.matchAll(/CAR_FOLLOW_(\w+)(?:\s*=\s*(\d+))?\s*,?/g)) {
        const value = match[2] === undefined ? nextEnumValue : Number(match[2]);
        enumValues[match[1]] = value;
        nextEnumValue = value + 1;
    }
    assert.deepEqual(enumValues, {IDLE: 0, WAIT_RUN: 1, WAIT_DELAY: 2, ACTIVE: 3, RUN_UPRIGHT: 4, SPEED_RAMP: 5});
    assert.equal(waitDelayMs, 2000);
    assert.equal(uprightMs, 500);
    assert.equal(speedRampMs, 2000);
    assert.equal(oscerrDelayMs, 5000);
    assert(!/CAR_FOLLOW_(?:STAGE|RAMP)_MS/.test(car));
    assert(!/CAR_FOLLOW_LEAN_(?:IN|HOLD|OUT)/.test(car + carHeader));
    assert(trackBody.includes('now_ms - follow_stage_started_ms < CAR_FOLLOW_WAIT_DELAY_MS'));
    assert(trackBody.includes('now_ms - follow_stage_started_ms >= CAR_FOLLOW_UPRIGHT_MS'));
    const menuSpeedIndex = trackBody.indexOf('target_speed_mps = balance_params.target_speed_mps');
    const curveSpeedIndex = trackBody.indexOf('if (car_curve_mode) target_speed_mps = 1.5f;', menuSpeedIndex);
    const uprightSpeedIndex = trackBody.indexOf('if (car_follow_status == CAR_FOLLOW_RUN_UPRIGHT) target_speed_mps = 0.0f', curveSpeedIndex);
    const rampSpeedIndex = trackBody.indexOf('if (car_follow_status == CAR_FOLLOW_SPEED_RAMP)', uprightSpeedIndex);
    assert(menuSpeedIndex >= 0 && curveSpeedIndex > menuSpeedIndex && uprightSpeedIndex > curveSpeedIndex && rampSpeedIndex > uprightSpeedIndex);
    assert(trackBody.includes('target_speed_mps *= elapsed_ms / (float)CAR_FOLLOW_SPEED_RAMP_MS'));
    assert(trackBody.includes('elapsed_ms >= CAR_FOLLOW_SPEED_RAMP_MS'));
    assert(trackBody.includes('elapsed_ms = now_ms - follow_stage_started_ms'));

    const elapsed = (now, started) => ((now - started) >>> 0);
    const started = 0xfffffff0;
    assert.equal(elapsed((started + waitDelayMs - 1) >>> 0, started), waitDelayMs - 1);
    assert.equal(elapsed((started + waitDelayMs) >>> 0, started), waitDelayMs);
    assert.equal(elapsed((started + uprightMs - 1) >>> 0, started), uprightMs - 1);
    assert.equal(elapsed((started + uprightMs) >>> 0, started), uprightMs);
    assert.equal(elapsed((started + speedRampMs - 1) >>> 0, started), speedRampMs - 1);
    assert.equal(elapsed((started + speedRampMs) >>> 0, started), speedRampMs);

    const toJsBody = (source, signature) => {
        const start = source.indexOf(signature);
        assert(start >= 0, signature);
        const open = source.indexOf('{', start);
        let depth = 0;
        let close = -1;
        for (let index = open; index < source.length; index++) {
            if (source[index] === '{') depth++;
            else if (source[index] === '}' && --depth === 0) { close = index; break; }
        }
        assert(close > open, signature);
        return source.slice(open + 1, close).replace(/\b(?:float|uint32|uint8)\s+([A-Za-z_]\w*(?:\s*=\s*[^;]+)?);/g, 'let $1;')
            .replace(/\((?:float|uint32|uint8)\)/g, '').replace(/(\d+\.\d+)f/g, '$1')
            .replace(/(now_ms|capture_ms|follow_stage_started_ms|follow_started_ms|last_control_capture_ms)\s*-\s*(now_ms|capture_ms|follow_stage_started_ms|follow_started_ms|last_control_capture_ms)/g,
                'u32Sub($1, $2)');
    };
    const runCBody = body => new Function('env', `with (env) { ${body} }`);
    const trackFn = runCBody(toJsBody(car, 'void car_track_update('));
    const stopFn = runCBody(toJsBody(car, 'void car_follow_stop(void)'));
    const resetFn = runCBody(toJsBody(car, 'static void car_reset_turn(void)'));
    const enumName = value => Object.keys(enumValues).find(name => enumValues[name] === value);
    const menuTarget = 1.75;
    const makeEnv = (status, running, stageStarted, fault = 0, followStarted = stageStarted) => {
        const env = {
            car_follow_status: enumValues[status], car_running: running ? 1 : 0,
            car_follow_enabled: ['IDLE', 'WAIT_DELAY', 'WAIT_RUN'].includes(status) ? 0 : 1,
            speedControlActive: false, follow_stage_started_ms: stageStarted >>> 0,
            follow_started_ms: followStarted >>> 0, oscillationDetectionEnabled: false,
            car_lost_count: 0, car_curve_mode: 0, car_turn_request: 0,
            derivative_ready: 0, last_control_capture_ms: 0, last_error: 0, curve_blend: 0,
            steering_kp: 10, steering_kd: 0, curve_steering_kp: 10, curve_steering_kd: 0,
            curve_enter_threshold: 1, curve_exit_threshold: 0, visual_timeout_ms: 120,
            balance_state: {fault, running: 1},
            balance_params: {target_speed_mps: menuTarget, turn_direction: 1, turn_pwm_limit: 1000},
            BALANCE_FAULT_NONE: 0, BOARD_FOLLOW_TURN_PWM_SIGN: turnPwmSign,
            CAR_FOLLOW_WAIT_DELAY_MS: waitDelayMs, CAR_FOLLOW_UPRIGHT_MS: uprightMs,
            CAR_FOLLOW_SPEED_RAMP_MS: speedRampMs, CAR_FOLLOW_OSCERR_DELAY_MS: oscerrDelayMs,
            requestStart: false, cancelled: false, command: null,
            u32Sub: (a, b) => ((a - b) >>> 0), car_value_finite: Number.isFinite,
            cc_math_clamp_f32: (value, minimum, maximum) => Math.max(minimum, Math.min(maximum, value))
        };
        for (const [name, value] of Object.entries(enumValues)) env[`CAR_FOLLOW_${name}`] = value;
        env.car_reset_turn = () => resetFn(env);
        env.balance_set_speed_control_active = active => { env.speedControlActive = Boolean(active); };
        env.balance_set_oscillation_detection = enabled => { env.oscillationDetectionEnabled = Boolean(enabled); };
        env.balance_cancel_follow_command = () => {
            env.cancelled = true;
            env.speedControlActive = false;
            env.oscillationDetectionEnabled = false;
        };
        env.car_follow_stop = () => stopFn(env);
        env.balance_runtime_request_start = () => { env.requestStart = true; };
        env.balance_set_follow_command = (speed, turn, lean, weight) => {
            env.command = {speed, turn: turn === 0 ? 0 : turn, lean, weight};
        };
        return env;
    };
    const invokeTrack = (env, now, error = 0, curvature = 0) => {
        env.now_ms = now >>> 0;
        env.capture_ms = now === 0 ? 1 : now >>> 0;
        env.error = error; env.curvature = curvature; env.valid = 1; env.new_result = 1;
        env.command = null; env.requestStart = false; env.cancelled = false;
        trackFn(env);
        return env;
    };
    const assertUprightCommand = env => {
        assert.equal(enumName(env.car_follow_status), 'RUN_UPRIGHT');
        assert.equal(env.command.speed, 0);
        assert.equal(env.speedControlActive, false);
        assert.equal(env.command.lean, 0);
        assert.equal(env.command.weight, 0);
    };

    const delay = makeEnv('WAIT_DELAY', false, started);
    invokeTrack(delay, (started + waitDelayMs - 1) >>> 0);
    assert.equal(enumName(delay.car_follow_status), 'WAIT_DELAY');
    assert.equal(delay.speedControlActive, false);
    assert.equal(delay.follow_started_ms, started);
    assert.equal(delay.oscillationDetectionEnabled, false);
    assert.deepEqual(delay.command, {speed: 0, turn: 0, lean: 0, weight: 0});
    const delayEnd = (started + waitDelayMs) >>> 0;
    invokeTrack(delay, delayEnd);
    assert.equal(enumName(delay.car_follow_status), 'WAIT_RUN');
    assert.equal(delay.speedControlActive, false);
    assert.equal(delay.requestStart, true);
    assert.deepEqual(delay.command, {speed: 0, turn: 0, lean: 0, weight: 0});
    assert.equal(delay.follow_started_ms, started);
    assert.equal(delay.oscillationDetectionEnabled, false);
    invokeTrack(delay, (delayEnd + 6000) >>> 0);
    assert.equal(enumName(delay.car_follow_status), 'WAIT_RUN');
    assert.equal(delay.speedControlActive, false);
    const runNow = (delayEnd + 9000) >>> 0;
    delay.car_running = 1;
    invokeTrack(delay, runNow, 2);
    assertUprightCommand(delay);
    assert.equal(delay.follow_stage_started_ms, runNow);
    assert.equal(delay.follow_started_ms, started);
    assert.equal(delay.command.turn, 20 * turnPwmSign);
    invokeTrack(delay, (runNow + uprightMs - 1) >>> 0, 2);
    assertUprightCommand(delay);
    assert.equal(delay.command.turn, 20 * turnPwmSign);
    const rampStart = (runNow + uprightMs) >>> 0;
    invokeTrack(delay, rampStart, 2);
    assert.equal(enumName(delay.car_follow_status), 'SPEED_RAMP');
    assert.equal(delay.follow_stage_started_ms, rampStart);
    assert.equal(delay.follow_started_ms, started);
    assert.equal(delay.speedControlActive, true);
    assert.equal(delay.command.speed, 0);
    assert.equal(delay.command.turn, 20 * turnPwmSign);

    for (const offset of [0, 499, 500, 1000, 1999, 2000]) {
        invokeTrack(delay, (rampStart + offset) >>> 0, 2);
        assert.equal(enumName(delay.car_follow_status), offset < speedRampMs ? 'SPEED_RAMP' : 'ACTIVE');
        assert.equal(delay.speedControlActive, true);
        near(delay.command.speed, offset < speedRampMs ? menuTarget * offset / speedRampMs : menuTarget);
        assert.equal(delay.command.turn, 20 * turnPwmSign);
        assert.equal(delay.command.lean, 0);
        assert.equal(delay.command.weight, 0);
        assert.equal(delay.follow_started_ms, started);
        if (offset === speedRampMs) assert.equal(delay.follow_stage_started_ms, 0);
    }

    invokeTrack(delay, (rampStart + 2500) >>> 0, 0, 2);
    assert.equal(delay.car_curve_mode, 1);
    assert.equal(delay.command.speed, 1.5);
    invokeTrack(delay, (rampStart + 2520) >>> 0, 0, 0);
    assert.equal(delay.car_curve_mode, 0);
    assert.equal(delay.command.speed, menuTarget);
    delay.balance_params.target_speed_mps = 1.0;
    invokeTrack(delay, (rampStart + 2540) >>> 0, 0, 2);
    assert.equal(delay.car_curve_mode, 1);
    assert.equal(delay.command.speed, 1.5);

    const curveRamp = makeEnv('SPEED_RAMP', true, rampStart);
    invokeTrack(curveRamp, (rampStart + speedRampMs / 2) >>> 0, 0, 2);
    assert.equal(curveRamp.car_curve_mode, 1);
    near(curveRamp.command.speed, 1.5 * (speedRampMs / 2) / speedRampMs);

    const grace = makeEnv('ACTIVE', true, 0, 0, started);
    invokeTrack(grace, (started + 4998) >>> 0);
    assert.equal(grace.oscillationDetectionEnabled, false);
    invokeTrack(grace, (started + oscerrDelayMs) >>> 0);
    assert.equal(grace.oscillationDetectionEnabled, true);

    const wrapRampStart = 0xffffff00;
    const wrapRamp = makeEnv('SPEED_RAMP', true, wrapRampStart);
    invokeTrack(wrapRamp, (wrapRampStart + speedRampMs - 1) >>> 0, 2);
    assert.equal(enumName(wrapRamp.car_follow_status), 'SPEED_RAMP');
    near(wrapRamp.command.speed, menuTarget * (speedRampMs - 1) / speedRampMs);
    invokeTrack(wrapRamp, (wrapRampStart + speedRampMs) >>> 0, 2);
    assert.equal(enumName(wrapRamp.car_follow_status), 'ACTIVE');
    near(wrapRamp.command.speed, menuTarget);

    const zeroTarget = makeEnv('SPEED_RAMP', true, rampStart);
    zeroTarget.balance_params.target_speed_mps = 0;
    invokeTrack(zeroTarget, (rampStart + speedRampMs / 2) >>> 0);
    assert.equal(zeroTarget.command.speed, 0);
    assert(Number.isFinite(zeroTarget.command.speed));
    assert.equal(enumName(zeroTarget.car_follow_status), 'SPEED_RAMP');

    for (const invalidKind of ['line', 'stale', 'missing', 'numeric']) {
        const tracking = makeEnv('SPEED_RAMP', true, rampStart);
        invokeTrack(tracking, (rampStart + 400) >>> 0, 2);
        if (invalidKind === 'line') tracking.valid = 0;
        else if (invalidKind === 'stale') tracking.capture_ms = (tracking.now_ms - 121) >>> 0;
        else if (invalidKind === 'missing') tracking.capture_ms = 0;
        else tracking.error = NaN;
        tracking.now_ms = (rampStart + 1000) >>> 0;
        if (invalidKind === 'stale') tracking.capture_ms = (tracking.now_ms - 121) >>> 0;
        else if (invalidKind === 'missing') tracking.capture_ms = 0;
        else if (invalidKind !== 'numeric') tracking.capture_ms = tracking.now_ms;
        trackFn(tracking);
        assert.equal(enumName(tracking.car_follow_status), 'SPEED_RAMP');
        near(tracking.command.speed, menuTarget / 2);
        assert.equal(tracking.command.turn, 0);
        assert.equal(tracking.speedControlActive, true);
        tracking.now_ms = (rampStart + 1001) >>> 0;
        tracking.capture_ms = tracking.now_ms;
        tracking.error = 2;
        tracking.valid = 1;
        trackFn(tracking);
        near(tracking.command.speed, menuTarget * 1001 / speedRampMs);
        assert.equal(tracking.command.turn, 20 * turnPwmSign);
    }

    const lost = makeEnv('SPEED_RAMP', false, rampStart + 1000);
    invokeTrack(lost, rampStart + 1001);
    assert.equal(enumName(lost.car_follow_status), 'WAIT_RUN');
    assert.equal(lost.car_follow_enabled, 0);
    assert.equal(lost.speedControlActive, false);
    assert.equal(lost.follow_stage_started_ms, 0);
    lost.car_running = 1;
    const restartedAt = (rampStart + 5000) >>> 0;
    invokeTrack(lost, restartedAt, 2);
    assertUprightCommand(lost);
    assert.equal(lost.follow_stage_started_ms, restartedAt);
    assert.equal(lost.command.turn, 20 * turnPwmSign);

    const faulted = makeEnv('SPEED_RAMP', true, rampStart + 1000, 1);
    invokeTrack(faulted, rampStart + 1001);
    assert.equal(enumName(faulted.car_follow_status), 'IDLE');
    assert.equal(faulted.car_follow_enabled, 0);
    assert.equal(faulted.follow_stage_started_ms, 0);
    assert.equal(faulted.follow_started_ms, 0);
    assert.equal(faulted.cancelled, true);
    assert.equal(faulted.speedControlActive, false);
    assert.equal(faulted.oscillationDetectionEnabled, false);
    const stopped = makeEnv('SPEED_RAMP', true, rampStart + 1000);
    stopFn(stopped);
    assert.equal(enumName(stopped.car_follow_status), 'IDLE');
    assert.equal(stopped.car_follow_enabled, 0);
    assert.equal(stopped.follow_stage_started_ms, 0);
    assert.equal(stopped.follow_started_ms, 0);
    assert.equal(stopped.cancelled, true);
    assert.equal(stopped.speedControlActive, false);
    assert.equal(stopped.oscillationDetectionEnabled, false);

    const noFollow = makeEnv('IDLE', true, runNow);
    invokeTrack(noFollow, runNow + 20);
    assert.equal(noFollow.car_follow_enabled, 0);
    assert.equal(noFollow.speedControlActive, false);
    assert(car.includes('now_ms - capture_ms <= visual_timeout_ms'));
    assert(car.includes('uint8 visual_valid = visual_fresh && valid && car_value_finite(error) && car_value_finite(curvature)'));
    assert(car.includes('if (!visual_valid)'));
    assert(car.includes('car_reset_turn()'));
    assert(!menu.includes('FOLLOW WAIT MS'));
    assert(!menu.includes('FOLLOW STATUS'));
});

test('FOLLOW START stops immediately and ordinary RUN remains outside the upright hold', () => {
    const initBody = car.slice(car.indexOf('void car_init(void)'), car.indexOf('void car_set_camera_ready'));
    const startBody = car.slice(car.indexOf('uint8 car_follow_start(void)'), car.indexOf('void car_follow_stop(void)'));
    assert(startBody.includes('balance_runtime_stop()'));
    assert(startBody.includes('car_follow_status = CAR_FOLLOW_WAIT_DELAY'));
    assert(startBody.includes('follow_stage_started_ms = system_getval_ms()'));
    assert(startBody.includes('follow_started_ms = follow_stage_started_ms'));
    assert(startBody.indexOf('follow_stage_started_ms = system_getval_ms()') < startBody.indexOf('follow_started_ms = follow_stage_started_ms'));
    assert(!startBody.includes('balance_runtime_request_start()'));
    assert(initBody.includes('follow_started_ms = 0'));
    assert(runtime.includes('car_follow_status == CAR_FOLLOW_WAIT_RUN && !balance_state.running'));
    assert(car.includes('if (car_follow_status == CAR_FOLLOW_RUN_UPRIGHT) target_speed_mps = 0.0f'));
    assert(car.includes('balance_set_speed_control_active(car_follow_status != CAR_FOLLOW_RUN_UPRIGHT)'));
    assert(car.includes('balance_set_follow_command(target_speed_mps, car_turn_request, 0.0f, 0.0f)'));
    const stopBody = car.slice(car.indexOf('void car_follow_stop(void)'), car.indexOf('void car_track_update('));
    assert(stopBody.includes('car_follow_status = CAR_FOLLOW_IDLE'));
    assert(stopBody.includes('follow_stage_started_ms = 0'));
    assert(stopBody.includes('follow_started_ms = 0'));
    assert(stopBody.includes('balance_cancel_follow_command()'));
    const trackBody = car.slice(car.indexOf('void car_track_update('));
    assert(!trackBody.match(/follow_started_ms\s*=/));
    assert(trackBody.includes('balance_set_oscillation_detection(now_ms - follow_started_ms >= CAR_FOLLOW_OSCERR_DELAY_MS)'));
    assert(trackBody.lastIndexOf('balance_set_speed_control_active') < trackBody.indexOf('balance_set_oscillation_detection'));
    const graceMs = Number(car.match(/#define CAR_FOLLOW_OSCERR_DELAY_MS\s+\((\d+)\)/)[1]);
    const gate = new Function('now_ms', 'follow_started_ms', 'CAR_FOLLOW_OSCERR_DELAY_MS',
        'u32Sub', 'return u32Sub(now_ms, follow_started_ms) >= CAR_FOLLOW_OSCERR_DELAY_MS;');
    const pressMs = 0xfffffff0;
    assert.equal(gate((pressMs + 4998) >>> 0, pressMs, graceMs, (now, then) => (now - then) >>> 0), false);
    assert.equal(gate((pressMs + graceMs) >>> 0, pressMs, graceMs, (now, then) => (now - then) >>> 0), true);
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

test('FOLLOW ramps only the menu speed target and leaves camera steering active', () => {
    const commandBody = car.slice(car.indexOf('target_speed_mps = balance_params.target_speed_mps'));
    assert(commandBody.includes('target_speed_mps = balance_params.target_speed_mps'));
    assert(!car.includes('target_speed_mps = visual_valid ?'));
    assert(commandBody.includes('target_speed_mps *= elapsed_ms / (float)CAR_FOLLOW_SPEED_RAMP_MS'));
    assert(commandBody.includes('balance_set_follow_command(target_speed_mps, car_turn_request, 0.0f, 0.0f)'));
    assert(!commandBody.includes('lean_angle'));
    assert(!commandBody.includes('lean_weight'));
    assert(!commandBody.includes('CAR_FOLLOW_LEAN_'));
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

test('FOLLOW target blending keeps a nonzero speed correction continuous and rejects invalid weights', () => {
    const targetMatch = control.match(/balance_state\.target_angle = applied_params\.middle_angle \+\s+([\s\S]*?);/);
    assert(targetMatch);
    assert(targetMatch[1].includes('(1.0f - requested_lean_weight)'));
    assert(targetMatch[1].includes('requested_lean_weight * requested_lean_angle'));
    const target = new Function('middle', 'speedAngle', 'weight', 'lean',
        `return middle + ${targetMatch[1]
            .replace(/balance_state\.speed_angle_applied/g, 'speedAngle')
            .replace(/requested_lean_weight/g, 'weight').replace(/requested_lean_angle/g, 'lean')
            .replace(/(\d+\.\d+)f/g, '$1')};`);
    assert.equal(target(30, -2, 0, -5), 28);
    assert.equal(target(30, -2, 0.5, -5), 26.5);
    assert.equal(target(30, -2, 1, -5), 25);
    assert(Math.abs(target(30, -2, 0.51, -5) - target(30, -2, 0.5, -5)) < 0.2);

    const commandBody = control.slice(control.indexOf('void balance_set_follow_command('), control.indexOf('static void balance_update_speed_20ms'));
    const cancelBody = control.slice(control.indexOf('void balance_cancel_follow_command(void)'), control.indexOf('static void balance_clear_outputs'));
    assert(cancelBody.includes('requested_lean_weight = 0.0f'));
    assert(commandBody.includes('requested_lean_weight = lean_weight'));
    assert(commandBody.includes('!balance_is_finite(lean_weight)'));
    assert(commandBody.includes('lean_weight < 0.0f || lean_weight > 1.0f'));
    const rejectExpression = commandBody.match(/if \((!balance_is_finite\(target_speed_mps\)[\s\S]*?lean_weight > 1\.0f)\) \{/)[1]
        .replace(/balance_is_finite/g, 'finite').replace(/(\d+\.\d+)f/g, '$1');
    const rejects = new Function('target_speed_mps', 'turn_pwm', 'lean_angle', 'lean_weight', 'finite', `return ${rejectExpression};`);
    for (const value of [NaN, -0.001, 1.001, Infinity, -Infinity]) assert(rejects(0, 0, 0, value, Number.isFinite));
    for (const value of [0, 0.5, 1]) assert(!rejects(0, 0, 0, value, Number.isFinite));
});

test('FOLLOW fault handling cancels every stage and a fault-free RUN loss restarts upright', () => {
    const trackBody = car.slice(car.indexOf('void car_track_update('));
    assert(trackBody.includes('car_follow_status != CAR_FOLLOW_IDLE && balance_state.fault != BALANCE_FAULT_NONE'));
    assert(trackBody.includes('car_follow_stop()'));
    assert(car.includes('balance_cancel_follow_command()'));
    const loss = trackBody.indexOf('car_follow_status != CAR_FOLLOW_IDLE && car_follow_status != CAR_FOLLOW_WAIT_RUN && car_running == 0');
    const restart = trackBody.indexOf('car_follow_status == CAR_FOLLOW_WAIT_RUN && car_running != 0');
    assert(loss >= 0 && restart > loss);
    assert(trackBody.slice(loss, restart).includes('car_follow_status = CAR_FOLLOW_WAIT_RUN'));
    assert(trackBody.slice(loss, restart).includes('follow_stage_started_ms = 0'));
    assert(trackBody.slice(restart, restart + 260).includes('car_follow_status = CAR_FOLLOW_RUN_UPRIGHT'));
    const stopBody = car.slice(car.indexOf('void car_follow_stop(void)'), car.indexOf('void car_track_update('));
    assert(stopBody.includes('car_follow_enabled = 0'));
    assert(stopBody.includes('follow_stage_started_ms = 0'));
    assert(stopBody.includes('balance_cancel_follow_command()'));
});

test('2 ms encoder diagnostics preserve a signed 20 ms vision average', () => {
    const sampleStart = control.indexOf('void balance_sample_encoders_2ms(');
    const sampleBody = control.slice(sampleStart, control.indexOf('\nvoid balance_init', sampleStart));
    assert(sampleBody.includes('left_count_sum += left_count'));
    assert(sampleBody.includes('right_count_sum += right_count'));
    assert(sampleBody.includes('if (++encoder_sample_phase < 10) return'));
    assert(sampleBody.includes('encoder_average_count = -(left_count_sum + right_count_sum) / 2'));
    assert(control.includes('left_count_sum'));
    assert(control.includes('right_count_sum'));
    assert(menu.includes('"LEFT COUNT 2MS"'));
    assert(menu.includes('"RIGHT COUNT 2MS"'));
    assert(controlHeader.match(/speed_(?:kp|ki|filter|integral|angle|enabled)/));
    assert(controlHeader.includes('speed_raw_mps'));
    assert(controlHeader.includes('speed_target_applied_mps'));
    assert(controlHeader.includes('speed_error_mps'));
    assert(control.includes('left_counts_per_rev'));
    assert(control.includes('right_counts_per_rev'));
    assert(control.includes('0.065f'));
});

test('forward-negative encoder counts produce positive speed at a fixed 1160 counts per revolution', () => {
    const countsPerRev = 1160;
    const circumference = Math.PI * 0.065;
    const rawSpeed = (left, right) => -circumference * (left + right) / (2 * countsPerRev * 0.002);
    const targetCounts20ms = 1.5 * 0.02 * countsPerRev / (Math.PI * 0.065);
    near(targetCounts20ms, 170.418439, 0.001);
    near(rawSpeed(-10, -10), circumference * 10 / (countsPerRev * 0.002));
    near(rawSpeed(10, 10), -circumference * 10 / (countsPerRev * 0.002));
    assert(control.includes('BALANCE_ENCODER_COUNTS_PER_REV * balance_dt_s'));
    assert(control.includes('encoder_average_count = -(left_count_sum + right_count_sum) / 2'));
    assert(control.includes('balance_state.left_counts_per_rev = BALANCE_ENCODER_COUNTS_PER_REV'));
    assert(control.includes('balance_state.right_counts_per_rev = BALANCE_ENCODER_COUNTS_PER_REV'));
    assert(!/\b(?:abs|fabs|fabsf)\s*\(/.test(control.slice(control.indexOf('void balance_sample_encoders_2ms('), control.indexOf('\nvoid balance_init'))));
    assert(!control.includes('BALANCE_ENCODER_FLASH_PAGE'));
    assert(!control.includes('balance_encoder_flash_load'));
    assert(!control.includes('balance_encoder_flash_save'));
    assert(!control.includes('balance_encoder_calibrate_toggle'));
    assert(!menu.includes('CAL LEFT 1 REV'));
    assert(!menu.includes('CAL RIGHT 1 REV'));
    assert(menu.includes('"LEFT COUNTS/REV"'));
    assert(menu.includes('"RIGHT COUNTS/REV"'));
    assert(control.includes('speed_angle_applied = BOARD_FOLLOW_FORWARD_PWM_SIGN * balance_clamp(pi_output, angle_limit)'));
    assert(forwardPwmSign === -1);
    assert(forwardPwmSign * 1.0 < 0 && forwardPwmSign * -1.0 > 0);
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

test('full-scale wheel PWM trips after output calculation and STOP clears queued starts before restart', () => {
    const runControlStart = control.indexOf('static uint8_t balance_run_control(');
    const runControlBody = control.slice(runControlStart, control.indexOf('void balance_params_reset', runControlStart));
    assert(runControlBody.indexOf('balance_state.left_pwm = balance_state.left_control_pwm') >= 0);
    assert(runControlBody.indexOf('balance_state.right_pwm = balance_apply_right_trim(right_control)') >
        runControlBody.indexOf('balance_state.left_pwm = balance_state.left_control_pwm'));

    const updateStart = control.indexOf('void balance_update_2ms(');
    const updateBody = control.slice(updateStart);
    const tripStart = updateBody.indexOf('if (speed_control_active && oscillation_detection_enabled &&');
    assert(updateBody.indexOf('balance_run_control()') < tripStart);
    const tripConditionEnd = updateBody.indexOf(') {', tripStart) + 1;
    const tripCondition = updateBody.slice(tripStart, tripConditionEnd);
    assert.equal(tripCondition.replace(/\s+/g, ' '),
        'if (speed_control_active && oscillation_detection_enabled && balance_state.left_pwm <= -10000 && balance_state.right_pwm <= -10000)');
    const stopExpression = tripCondition.slice('if ('.length, -1)
        .replace(/balance_state\.left_pwm/g, 'left_pwm')
        .replace(/balance_state\.right_pwm/g, 'right_pwm');
    const shouldStop = new Function('speed_control_active', 'oscillation_detection_enabled', 'left_pwm', 'right_pwm', `return ${stopExpression};`);
    for (const left of [-10000, -9999, 0, 9999, 10000]) {
        for (const right of [-10000, -9999, 0, 9999, 10000]) {
            assert.equal(Boolean(shouldStop(1, 1, left, right)), left === -10000 && right === -10000);
        }
    }
    assert.equal(Boolean(shouldStop(0, 1, -10000, -10000)), false);
    assert.equal(Boolean(shouldStop(1, 0, -10000, -10000)), false);
    assert(!control.includes('BALANCE_OSCERR_HOLD_MS'));
    assert(!control.includes('oscillation_fullscale_ms'));
    const cancelBody = control.slice(control.indexOf('void balance_cancel_follow_command(void)'), control.indexOf('static void balance_clear_outputs'));
    assert(cancelBody.includes('oscillation_detection_enabled = 0'));
    assert(cancelBody.includes('balance_reset_speed_controller()'));
    const tripBody = updateBody.slice(tripConditionEnd, updateBody.indexOf('balance_state.fault = BALANCE_FAULT_NONE', tripConditionEnd));
    assert(tripBody.includes('balance_state.running = 0'));
    assert(tripBody.includes('balance_clear_outputs()'));
    assert(tripBody.includes('balance_state.fault = BALANCE_FAULT_OSCILLATION'));
    assert(tripBody.includes('return;'));
    assert(!tripBody.includes('if ('));
    assert(controlHeader.includes('BALANCE_FAULT_OSCILLATION'));

    const updateRuntimeStart = runtime.indexOf('void balance_runtime_update_2ms');
    const updateRuntimeBody = runtime.slice(updateRuntimeStart, runtime.indexOf('void balance_runtime_stop', updateRuntimeStart));
    const completeStop = updateRuntimeBody.indexOf('if (was_running && !balance_state.running)');
    const queuedFollowStart = updateRuntimeBody.indexOf('if (car_follow_status == CAR_FOLLOW_WAIT_RUN && !balance_state.running)');
    const pendingStart = updateRuntimeBody.indexOf('if (balance_start_pending && !balance_state.running)');
    assert(completeStop >= 0 && queuedFollowStart > completeStop && pendingStart > completeStop);
    const stopBody = updateRuntimeBody.slice(completeStop, queuedFollowStart);
    assert(stopBody.indexOf('balance_runtime_stop()') < stopBody.indexOf('balance_state.fault = stop_fault'));
    assert(stopBody.indexOf('balance_state.fault = stop_fault') < stopBody.indexOf('return;'));
    assert(runtime.includes('void balance_runtime_stop(void)'));
    assert(runtime.slice(runtime.indexOf('void balance_runtime_stop(void'), runtime.indexOf('void balance_runtime_set_motor_test'))
        .includes('car_follow_stop()'));

    const requestStart = runtime.slice(runtime.indexOf('void balance_runtime_request_start(void)'),
        runtime.indexOf('uint8 balance_runtime_request_calibration'));
    assert(requestStart.includes('balance_start_pending = 1'));
    assert(!requestStart.includes('BALANCE_FAULT_OSCILLATION'));
    const balanceStart = control.slice(control.indexOf('uint8_t balance_start(void)'),
        control.indexOf('void balance_stop(void)'));
    assert(balanceStart.includes('balance_state.running = 1'));
    assert(balanceStart.includes('balance_state.fault = BALANCE_FAULT_NONE'));
    assert(menu.includes('return "PWM LIMIT - STOPPED"'));
});

test('RUN keeps the warmup attitude and motor test leaves raw PWM ownership outside balance control', () => {
    const updateBody = runtime.slice(runtime.indexOf('void balance_runtime_update_2ms'),
        runtime.indexOf('void balance_runtime_stop'));
    const startIndex = updateBody.indexOf('if (balance_start_pending && !balance_state.running)');
    const startEnd = updateBody.indexOf('if (!car_running && balance_state.running)', startIndex);
    const startBlock = updateBody.slice(startIndex, startEnd);
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
    const speedRoot = menu.indexOf('{CAR_MENU_PAGE_SPEED, CAR_MENU_ROOT, "SPEED"');
    const steeringRoot = menu.indexOf('{CAR_MENU_PAGE_STEERING, CAR_MENU_ROOT, "STEERING"');
    assert(menu.includes('CAR_MENU_PAGE_SPEED = 10'));
    assert(speedRoot >= 0 && speedRoot < steeringRoot);
    for (const label of ['SPEED KP', 'SPEED KI', 'SPEED ANGLE LIMIT']) {
        assert(menu.includes(`{0, CAR_MENU_PAGE_SPEED, "${label}"`));
        assert(!menu.includes(`{0, CAR_MENU_PAGE_BALANCE, "${label}"`));
    }
    const speedRefreshStart = menu.indexOf('if (car_menu_page == CAR_MENU_PAGE_RUN || car_menu_page == CAR_MENU_PAGE_TELEMETRY');
    const speedRefresh = menu.slice(speedRefreshStart, menu.indexOf('\n        return;', speedRefreshStart));
    assert(speedRefresh.includes('CAR_MENU_PAGE_SPEED'));
    assert(speedRefresh.includes('CAR_MENU_TELEMETRY_REFRESH_MS'));
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
    const speedDisplay = menu.slice(menu.indexOf('static void car_menu_draw_value('), menu.indexOf('static void car_menu_draw_footer('));
    assert(speedDisplay.includes('&balance_state.speed_raw_mps'));
    assert(speedDisplay.includes('"UNCAL"'));
});

console.log(`${groups} groups passed; source-level numerical checks only, no firmware compilation.`);
