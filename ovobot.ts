
/**
 * 使用此文件来定义自定义函数和图形块。
 * 想了解更详细的信息，请前往 https://makecode.microbit.org/blocks/custom
 */
enum MoveDir {
    //% block="forward"
    Forward,
    //% block="backward"
    Backward
}

enum RobotMoveType {
    RobotMoveStop = 0,
    RobotMove = 1,
    RobotMoveCalibrate = 2
}

enum LineSensor {
    //% block="left"
    Left,
    //% block="right"
    Right
}

enum clock_sel_e {
    INV_CLK_INTERNAL = 0,
    INV_CLK_PLL,
    NUM_CLK
}

enum gyro_fsr_e {
    INV_FSR_250DPS = 0,
    INV_FSR_500DPS,
    INV_FSR_1000DPS,
    INV_FSR_2000DPS,
    NUM_GYRO_FSR
}

enum Color {
    //% block="red"
    Red,
    //% block="orange"
    Orange,
    //% block="yellow"
    Yellow,
    //% block="green"
    Green,
    //% block="blue"
    Blue,
    //% block="indigo"
    Indigo,
    //% block="purple"
    Purple,
    //% block="white"
    White,
    //% block="black"
    Black
}

enum accel_fsr_e {
    INV_FSR_2G = 0,
    INV_FSR_4G,
    INV_FSR_8G,
    INV_FSR_16G,
    NUM_ACCEL_FSR
}

class Stdev_t {
    m_oldM: number;
    m_newM: number;
    m_oldS: number;
    m_newS: number;
    m_n: number;
}

class DesiredMove_t {
    heading: number;
    speed: number;
    state: number;
    newMove: boolean;
    reachHeading: boolean;
    rawMotorMode: boolean;
}

class Robot {
    heading: number;
    rotateSpeed: number;
    speed: number;
    constructor(heading: number, rs: number, sp: number) {
        this.heading = heading;
        this.rotateSpeed = rs;
        this.speed = sp;
    }
}

/**
 * 自定义图形块
 */
//% weight=100 color=#00d1b5 icon="\uf036"
//% block="ovobot"
namespace ovobot {
    const MPU_RA_PWR_MGMT_1 = 0x6b
    const MPU6887_BIT_RESET = 0x80
    const MPU_RA_SIGNAL_PATH_RESET = 0x68
    const MPU_RA_GYRO_CONFIG = 0x1b
    const MPU_RA_ACCEL_CONFIG = 0x1c
    const MPU_RA_CONFIG = 0x1a
    const MPU_RA_SMPLRT_DIV = 0x19
    const MPU_RA_INT_PIN_CFG = 0x37
    const MPU6887_WHO_AM_I_CONST = 0x2e
    const MPU_RA_WHO_AM_I = 0x75
    const MPU_RA_GYRO_YOUT_H = 0x45
    const MPU_RA_GYRO_ZOUT_H = 0x47
    const SONAR_ADDRESS = 0x52
    const LED_ADDRESS = 0x53
    const MPU_ADDRESS = 0x69
    const CALIBRATING_GYRO_CYCLES = 300
    const lowBright = 8
    const LineSensor_ADDRESS = 0x54
    let _distance: number = 9999;
    let _lightStrength: number = 9999;
    let _lineSensorLeft = 0;
    let _lineSensorRight = 0;
    let _initSysEvent = 0;
    let gyroZRaw = 0;
    let gyroZ = 0;
    let gyroZero = 0;
    let calibratingG = 0;
    let g = 0;
    let stdev_t: Stdev_t = new Stdev_t();
    let previous = 0;
    let attitudeYaw = 0;
    let yaw = 0;
    let previous_tick = 0;
    let desiredMove: DesiredMove_t = new DesiredMove_t();
    let speed_out = 0;
    let last_desired_speed = 0;
    let timeout_cnt = 0;
    let stop_timeout = 0;
    let last_heading = 0;
    let stop_timeout_cnt = 0;
    let heading_offset_init_flag = 0;
    let headingOffset = 0;
    let lastITerm = 0;
    let lastMeanSpeed = 0;
    let selectColors = [0xff0000, 0xffa500, 0xffff00, 0x00ff00, 0x00ffff, 0x0000ff, 0x800080, 0xffffff, 0x000000];
    let robot: Robot = new Robot(0, 0, 0);
    let previous_state = 0;
    let gyroWorked = false;
    let initMotor = false;
    let rotate_start = -1;
    let rotate_total = 0;
    let rotate_target = 0;
    let previous_rotateMicroTimes = 0;
    let start_heading = 0;
    let isSonicEnabled = false;
    let isLineSensorEnable = false;
    let needPidCtl = true;
    //private function
    function setupMotorPWM() {
        if (!initMotor) {
            initMotor = true;
            pins.analogSetPeriod(AnalogPin.P15, 1000);
            pins.analogSetPeriod(AnalogPin.P13, 1000);
            pins.analogSetPeriod(AnalogPin.P16, 1000);
            pins.analogSetPeriod(AnalogPin.P8, 1000);
        }
    }

    function isGyroCalibrationComplete(): boolean {
        return calibratingG == 0;
    }

    function isOnFirstGyroCalibrationCycle(): boolean {
        return calibratingG == CALIBRATING_GYRO_CYCLES;
    }

    function isOnFinalGyroCalibrationCycle(): boolean {
        return calibratingG == 1;
    }

    function gyroSetCalibrationCycles(calibrationCyclesRequired: number) {
        calibratingG = calibrationCyclesRequired;
    }

    function devClear() {
        stdev_t.m_n = 0;
    }

    function devPush(x: number) {
        stdev_t.m_n++;
        if (stdev_t.m_n == 1) {
            stdev_t.m_oldM = stdev_t.m_newM = x;
            stdev_t.m_oldS = 0.0;
        } else {
            stdev_t.m_newM = stdev_t.m_oldM + (x - stdev_t.m_oldM) / stdev_t.m_n;
            stdev_t.m_newS = stdev_t.m_oldS + (x - stdev_t.m_oldM) * (x - stdev_t.m_newM);
            stdev_t.m_oldM = stdev_t.m_newM;
            stdev_t.m_oldS = stdev_t.m_newS;
        }
    }

    function devVariance(): number {
        return ((stdev_t.m_n > 1) ? stdev_t.m_newS / (stdev_t.m_n - 1) : 0.0);
    }

    function devStandardDeviation(): number {
        return Math.sqrt(devVariance());
    }

    function performAcclerationCalibration(gyroMovementCalibrationThreshold: number) {
        if (isOnFirstGyroCalibrationCycle()) {
            g = 0;
            devClear();
        }
        g += gyroZ;
        devPush(gyroZ);
        gyroZ = 0;
        gyroZero = 0;
        if (isOnFinalGyroCalibrationCycle()) {
            let dev = devStandardDeviation();
            // check deviation and startover in case the model was moved
            if (gyroMovementCalibrationThreshold && dev > gyroMovementCalibrationThreshold) {
                gyroSetCalibrationCycles(CALIBRATING_GYRO_CYCLES);
                return;
            }
            gyroZero = (g + (CALIBRATING_GYRO_CYCLES / 2)) / CALIBRATING_GYRO_CYCLES;
        }
        if (isOnFinalGyroCalibrationCycle()) {
            music.playTone(262, 500);
            gyroWorked = true;
        }
        calibratingG--;
    }

    function applyGyroZero() {
        gyroZ -= gyroZero;
    }

    function gyroUpdate() {
        gyroZRaw = readGyroZ();

        gyroZ = gyroZRaw;
        if (!isGyroCalibrationComplete()) {
            performAcclerationCalibration(32);
        }

        applyGyroZero();
    }

    //calc yaw attitude
    function angleUpdate() {
        let delta = 0;
        let current = input.runningTimeMicros();
        delta = current - previous;
        yaw -= gyroZ * delta / 16.4;
        yaw %= 360000000;
        attitudeYaw = yaw * 0.00001;
        if (attitudeYaw < 0)
            attitudeYaw += 3600;
        previous = current;
    }

    function constract(val: number, minVal: number, maxVal: number): number {
        if (val > maxVal) {
            return maxVal;
        } else if (val < minVal) {
            return minVal;
        }
        return val;

    }

    function pidController() {
        let yaw_angle_error = 0;
        let yaw_out = 0;
        let yaw_out_inner = 0;
        let desiredHeading = 0;
        let yaw_rate_error = 0;
        let speed_error = 0;
        let left_out = 0, right_out = 0;
        if (desiredMove.speed == 0
            && desiredMove.heading == last_heading) {
            if (++stop_timeout_cnt > 1000) {
                stop_timeout_cnt = 1000;
                stop_timeout = 1;
            }
        } else {
            stop_timeout_cnt = 0;
            stop_timeout = 0;
        }
        last_heading = desiredMove.heading;
        if ((heading_offset_init_flag && !stop_timeout)
            || desiredMove.state == 2) {

            desiredHeading = (desiredMove.heading + headingOffset / 10) % 360;
            yaw_angle_error = desiredHeading - attitudeYaw / 10;

            if (yaw_angle_error > 180)
                yaw_angle_error = yaw_angle_error - 360;

            if (yaw_angle_error < -180)
                yaw_angle_error = yaw_angle_error + 360;

            yaw_out_inner = yaw_angle_error * 15.0;// 2

            yaw_rate_error = yaw_out_inner + gyroZ * 10 / 164.0;
            let PTerm = yaw_rate_error * 5 / 10.0;
            let ITerm = lastITerm + yaw_rate_error * 0.000;
            constract(ITerm, -50, 50);
            lastITerm = ITerm;
            yaw_out = PTerm + ITerm;

            speed_out = desiredMove.speed;
            if (speed_out < -20)
                speed_out -= 80;
            else if (speed_out > 20)
                speed_out += 80;
            speed_out = constract(speed_out, -230, 230);
            left_out = speed_out + yaw_out;
            right_out = speed_out - yaw_out;
            left_out = constract(left_out, -255, 255);
            right_out = constract(right_out, -255, 255);

            moveMotorOut(left_out, right_out);

        } else {
            heading_offset_init_flag = 0;
            moveMotorOut(0, 0);
        }
    }

    function moveMotorOut(left: number, right: number) {
        if (right < 0) {
            let right_out = 1023 + right * 1023 / 255;
            pins.digitalWritePin(DigitalPin.P16, 1);
            pins.analogWritePin(AnalogPin.P8, right_out);
        } else {
            let right_out = right * 1023 / 255;
            pins.digitalWritePin(DigitalPin.P16, 0);
            pins.analogWritePin(AnalogPin.P8, right_out);
        }
        if (left < 0) {
            let left_out = 1023 + left * 1023 / 255;
            pins.digitalWritePin(DigitalPin.P15, 1);
            pins.analogWritePin(AnalogPin.P13, left_out);
        } else {
            let left_out = left * 1023 / 255;
            pins.digitalWritePin(DigitalPin.P15, 0);
            pins.analogWritePin(AnalogPin.P13, left_out);
        }
    }

    function setTimeout(fn: () => void, ms: number) {
        control.inBackground(() => {
            basic.pause(ms);
            fn();
        });
    }

    function setInterval(fn: () => void, ms: number) {
        control.inBackground(() => {
            basic.pause(ms);
            fn();
            setInterval(fn, ms);
        });
    }

    function mpu6887I2CWrite(address: number, reg: number, data: number) {
        let buf = pins.createBuffer(2);
        buf[0] = reg;
        buf[1] = data;
        pins.i2cWriteBuffer(address, buf);
    }

    function mpu6887Init() {
        calibratingG = 0;
        gyroZRaw = 0;
        gyroZ = 0;
        gyroZero = 0;
        mpu6887I2CWrite(MPU_ADDRESS, MPU_RA_PWR_MGMT_1, MPU6887_BIT_RESET);
        basic.pause(100);
        mpu6887I2CWrite(MPU_ADDRESS, MPU_RA_SIGNAL_PATH_RESET, 0x07);
        basic.pause(100);
        mpu6887I2CWrite(MPU_ADDRESS, MPU_RA_PWR_MGMT_1, 0);
        basic.pause(100);
        mpu6887I2CWrite(MPU_ADDRESS, MPU_RA_PWR_MGMT_1, clock_sel_e.INV_CLK_PLL);
        mpu6887I2CWrite(MPU_ADDRESS, MPU_RA_GYRO_CONFIG, gyro_fsr_e.INV_FSR_2000DPS << 3);
        mpu6887I2CWrite(MPU_ADDRESS, MPU_RA_ACCEL_CONFIG, accel_fsr_e.INV_FSR_8G << 3);
        mpu6887I2CWrite(MPU_ADDRESS, MPU_RA_CONFIG, 4);
        mpu6887I2CWrite(MPU_ADDRESS, MPU_RA_SMPLRT_DIV, 0); // Get Divider

        mpu6887I2CWrite(MPU_ADDRESS, MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 0 << 1 | 0 << 0);  // INT_ANYRD_2CLEAR, BYPASS_EN
        calibratingG = CALIBRATING_GYRO_CYCLES;
    }

    function mpu6887Detect(): boolean {
        let who = 0x00;
        pins.i2cWriteNumber(MPU_ADDRESS, MPU_RA_WHO_AM_I, NumberFormat.UInt8BE);
        who = pins.i2cReadNumber(MPU_ADDRESS, NumberFormat.UInt8BE);
        if (who != MPU6887_WHO_AM_I_CONST) {
            basic.showString("gyro not found");
            return false;
        }
        return true;
    }

    function sonicEnable() {
        let buf = pins.createBuffer(2);
        buf[0] = 0x00;
        buf[1] = 0x01;
        pins.i2cWriteBuffer(SONAR_ADDRESS, buf);
        isSonicEnabled = true;
    }

    function readGyroZ(): number {
        pins.i2cWriteNumber(MPU_ADDRESS, MPU_RA_GYRO_ZOUT_H, NumberFormat.UInt8BE, true);
        let gyrozVal = pins.i2cReadNumber(MPU_ADDRESS, NumberFormat.Int16BE);
        return gyrozVal;
    }

    function updateDistance() {
        let buf = pins.createBuffer(1);
        buf[0] = 0x01;
        pins.i2cWriteBuffer(SONAR_ADDRESS, buf, true);
        let sonarVal = pins.i2cReadNumber(SONAR_ADDRESS, NumberFormat.Int16LE);
        _distance = sonarVal / 29;
    }

    function updateLightStrength() {
        _lightStrength = input.lightLevel();
    }

    function updateLineSensor() {
        let buf = pins.createBuffer(1);
        buf[0] = 0x01;
        pins.i2cWriteBuffer(LineSensor_ADDRESS, buf, true);
        let val = pins.i2cReadNumber(LineSensor_ADDRESS, NumberFormat.Int32LE);
        _lineSensorRight = val >> 16;
        _lineSensorLeft = val & 0xffff;
    }

    function registerSysPeriodEvent() {
        if (!_initSysEvent) {
            _initSysEvent = 1;
            mpu6887Init();
            mpu6887Detect();
            setInterval(function () {
                gyroUpdate();
                if (gyroWorked) {
                    angleUpdate();
                    if (isSonicEnabled) {
                        updateDistance();
                    }
                    updateLightStrength();
                    updateLineSensor();
                    if (robot.rotateSpeed != 0) {
                        let current = input.runningTimeMicros();
                        let delta = current - previous_rotateMicroTimes;
                        robot.heading = start_heading + robot.rotateSpeed * delta / 1000000.0;
                        robot.heading = robot.heading % 360;
                        if (robot.heading < 0) {
                            robot.heading += 360;
                        }
                        if (robot.heading > 360) {
                            robot.heading -= 360;
                        }
                        appMove(robot.heading, robot.speed, RobotMoveType.RobotMove);
                    }
                    if (needPidCtl) {
                        pidController();
                    }
                }
            }, 6);
        }
    }
    /**
     * TODO: 点亮三色灯函数
     * @param lCol 左边颜色灯
     * @param rCol 右边颜色灯
     */
    //% blockId=set_ledGroup_Color block="set led color|left %lCol|right %rCol"
    //% weight=96
    export function setledGroupColor(lCol: Color, rCol: Color) {
        let buf = pins.createBuffer(8);
        buf[0] = 0x00;
        buf[1] = 0x00;
        buf[2] = ((selectColors[lCol] >> 8) & 0xff) / lowBright;
        buf[3] = ((selectColors[lCol] >> 16) & 0xff) / lowBright;
        buf[4] = (selectColors[lCol] & 0xff) / lowBright;
        buf[5] = ((selectColors[rCol] >> 8) & 0xff) / lowBright;
        buf[6] = ((selectColors[rCol] >> 16) & 0xff) / lowBright;
        buf[7] = (selectColors[rCol] & 0xff) / lowBright;
        pins.i2cWriteBuffer(LED_ADDRESS, buf);
    }

    function appMove(heading: number, speed: number, state: number) {
        if (state == 0x00) {
            heading_offset_init_flag = 0;
            desiredMove.heading = (attitudeYaw - headingOffset) / 10;
        } else if (state == 0x01) {
            if (!heading_offset_init_flag || state != previous_state) {
                heading_offset_init_flag = 1;
                headingOffset = attitudeYaw - heading * 10;
                desiredMove.heading = heading;
            } else {
                desiredMove.heading = heading;
            }
        } else {
            if (state != previous_state) {
                headingOffset = attitudeYaw - heading * 10;
            }
            desiredMove.heading = heading;
        }
        previous_state = state;
        desiredMove.speed = speed;
        desiredMove.state = state;
        desiredMove.newMove = true;
        desiredMove.reachHeading = false;
        desiredMove.rawMotorMode = false;
    }

    /**
     * TODO: 陀螺仪初始化函数
     */
    //% block weight=100
    export function initGyroSensor() {
        registerSysPeriodEvent();
        setupMotorPWM();
    }

    /**
     * TODO: 运动函数
     * @param movedir 前进方向
     * @param speed 前进速度, speed.min=0 speed.max=255 eg: 80
     * @param duration 运动时间(单位：秒)，duration.min=0 duration.max=999999 eg:3000
     */
    //% blockId=move_robot block="move direction|%movedir |at speed %speed |duration (ms) %duration"
    //% weight=95
    export function move(movedir: MoveDir, speed: number, duration: number) {
        while (!gyroWorked) {
            basic.pause(20);
        }
        needPidCtl = true;
        setupMotorPWM();
        let sp = constract(speed, 0, 255);
        speed = sp;
        if (movedir == MoveDir.Backward) {
            robot.speed = -speed;
            appMove(robot.heading, -speed, RobotMoveType.RobotMove);
        } else {
            robot.speed = speed;
            appMove(robot.heading, speed, RobotMoveType.RobotMove);
        }
        basic.pause(duration);
        stopMove();
    }

    /**
     * TODO: 停止运动函数
     */
    //% block weight=90
    export function stopMove() {
        setupMotorPWM();
        robot.speed = 0;
        robot.rotateSpeed = 0;
        appMove(robot.heading, 0, RobotMoveType.RobotMoveStop);
    }

    /**
     * TODO: 设置运动速度函数，小车将会以此速度一直运动。
     * @param speed 前进速度, speed.min=0 speed.max=255 eg: 80
     */
    //% blockId=set_move_speed block="move direction|%movedir |at speed %speed"
    //% weight=80
    export function moveAtSpeed(movedir: MoveDir, speed: number) {
        while (!gyroWorked) {
            basic.pause(20);
        }
        setupMotorPWM();
        needPidCtl = true;
        let sp = constract(speed, 0, 255);
        speed = sp;
        if (movedir == MoveDir.Backward) {
            robot.speed = -speed;
            appMove(robot.heading, -speed, RobotMoveType.RobotMove);
        } else {
            robot.speed = speed;
            appMove(robot.heading, speed, RobotMoveType.RobotMove);
        }
    }

    /**
     * TODO: 设置旋转角度函数，角度大于零为顺时针旋转，否则逆时针旋转。
     * @param angle 旋转角度, eg: 90
     * @param duration 旋转时间，eg:1000 
     */
    //% blockId=rotate_robot block="rotate angle|%angle |duration (ms) %duration"
    //% weight=70
    export function rotate(angle: number, duration: number) {
        while (!gyroWorked) {
            basic.pause(20);
        }
        needPidCtl = true;
        rotate_start = input.runningTimeMicros();
        rotate_total = duration * 1e3;
        rotate_target = angle;
        let startAngle = robot.heading;
        if (angle != 0 && duration != 0) {
            continueRotate(rotate_start, startAngle);
            basic.pause(duration);
        }
    }


    function continueRotate(startTime: number, startAngle: number) {
        let current = input.runningTimeMicros();
        let delta = (current - startTime);
        if (delta < rotate_total) {
            let newAngle = startAngle + delta * rotate_target / rotate_total;
            robot.heading = newAngle;
            appMove(newAngle, robot.speed, RobotMoveType.RobotMove);
            control.inBackground(() => {
                continueRotate(startTime, startAngle);
            });
        } else {
            robot.heading = startAngle + rotate_target;
            appMove(robot.heading, robot.speed, RobotMoveType.RobotMove);
        }
    }

    /**
     * TODO: 设置旋转角速度函数。
     * @param angleSpeed 角速度,left.min=-255 left.max=255 eg: 0
     */
    //% blockId=rotate_at_speed block="rotate at speed %angleSpeed"
    //% weight=65
    export function rotateAtSpeed(angleSpeed: number) {
        while (!gyroWorked) {
            basic.pause(20);
        }
        needPidCtl = true;
        if (angleSpeed == robot.rotateSpeed) {
            return;
        }
        angleSpeed = constract(angleSpeed, -255, 255);
        robot.rotateSpeed = angleSpeed;
        start_heading = robot.heading;
        previous_rotateMicroTimes = input.runningTimeMicros();
    }

    /**
     * TODO: 独立设置左右轮速度函数。
     * @param left 左轮速度,left.min=-255 left.max=255 eg: 0
     * @param right 右轮速度，right.min=-255 right.max=255 eg:0
     * @param duration 旋转时间，eg:0 
     */
    //% blockId=set_rawMotor block="rawMotor |left %left|right %right |duration (ms) %duration"
    //% weight=60
    export function rawMotor(left: number, right: number, duration: number) {
        setupMotorPWM();
        needPidCtl = false;
        left = constract(left, -255, 255);
        right = constract(right, -255, 255);
        moveMotorOut(left, right);
        basic.pause(duration);
        moveMotorOut(0, 0);
    }

    /**
     * TODO: 独立设置左右轮速度函数。
     * @param left 左轮速度,left.min=-255 left.max=255 eg: 0
     * @param right 右轮速度，right.min=-255 right.max=255 eg:0
     */
    //% blockId=set_rawMotor_withpwm block="rawMotor |left %left|right %right"
    //% weight=55
    export function rawMotorWithPwm(left: number, right: number) {
        setupMotorPWM();
        needPidCtl = false;
        left = constract(left, -255, 255);
        right = constract(right, -255, 255);
        moveMotorOut(left, right);
    }

    /**
     * TODO: 获取超声波传感器与前方障碍物的距离函数。
     */
    //% block weight=50
    export function readDistance(): number {
        sonicEnable();
        return _distance;
    }

    /**
     * TODO: 获取光电强度函数。
     */
    //% block weight=45
    export function readLightStrength(): number {
        return _lightStrength;
    }

    /**
     * TODO: 获取左右巡线传感器数据函数。
     */
    //% blockId=read_LineSensorData
    //% block="read line sensor from %lineSensor"
    //% weight=40
    export function readLineSensorData(lineSensor: LineSensor): number {
        isLineSensorEnable = true;
        if (lineSensor == LineSensor.Left) {
            return _lineSensorLeft;
        } else {
            return _lineSensorRight;
        }
    }
}
