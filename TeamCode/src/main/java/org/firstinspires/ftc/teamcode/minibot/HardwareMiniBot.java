package org.firstinspires.ftc.teamcode.minibot;

//import com.qualcomm.ftccommon.DbgLog;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Thread.sleep;

/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 * <p>
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 * <p>
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */

public class HardwareMiniBot { // extends LinearOpMode {
    final static int ONE_ROTATION = 1120; // for AndyMark-40 motor encoder one rotation
    final static double RROBOT = 6.63;  // number of wheel turns to get chassis 360-degree turn
    final static double INCHES_PER_ROTATION = 12.57; // inches per chassis motor rotation based on 16/24 gear ratio
    final static double GYRO_ROTATION_RATIO_L = 0.65; // 0.83; // Ratio of Gyro Sensor Left turn to prevent overshooting the turn.
    final static double GYRO_ROTATION_RATIO_R = 0.65; // 0.84; // Ratio of Gyro Sensor Right turn to prevent overshooting the turn.
    final static double IMU_ROTATION_RATIO_L = 0.80; // 0.84; // Ratio of IMU Sensor Left turn to prevent overshooting the turn.
    final static double IMU_ROTATION_RATIO_R = 0.80; // 0.84; // Ratio of IMU Sensor Right turn to prevent overshooting the turn.
    final static double DRIVE_RATIO_L = 1.0; //control veering by lowering left motor power
    final static double DRIVE_RATIO_R = 1.0; //control veering by lowering right motor power

    final static double L_KICKER_INIT = 0.558;
    final static double L_KICKER_UP = 0.46;
    final static double L_KICKER_DOWN = 0.71;
    final static double R_KICKER_INIT = 0.63;
    final static double R_KICKER_UP = 0.78;
    final static double R_KICKER_DOWN = 0.44;
    final static double KICKER_INIT = 0;
    final static double KICKER_UP = 0;
    final static double KICKER_DOWN = 0.3;
    private boolean isKickerUp;

    final static double ELBOW_INIT = 0.5;
    final static double SHOULDER_INIT = 0.5;

    /* Public OpMode members. */
    public DcMotor leftMotor = null;
    public DcMotor rightMotor = null;
    public DcMotor encMotor = null;
    public Servo sv_shoulder = null;
    public Servo sv_elbow = null;
    public Servo sv_l_kicker = null;
    public Servo sv_r_kicker = null;
    public ModernRoboticsI2cRangeSensor rangeSensor;

    public boolean use_minibot = true;
    public boolean use_imu = false;
    public boolean use_imu_correction = false;
    public boolean use_encoder = true;
    public boolean use_color_sensor = false;
    public boolean use_arm = false;
    public boolean use_kicker = true;
    public double target_heading = 0.0;
    public boolean straight_mode = false;
    public float leftPower = 0;
    public float rightPower = 0;
    public double correction_ratio = 0.9;
    public int leftCnt = 0; // left motor target counter
    public int rightCnt = 0; // right motor target counter
    public static int color_white = 200;

    // The IMU sensor object
    BNO055IMU imu;
    ColorSensor colorSensor;    // Hardware Device Object


    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public HardwareMiniBot() {

    }

    //@Override
    //public void runOpMode() throws InterruptedException {
//
    //  }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        if (use_imu) {
            imu = hwMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);
        }
        // Define and Initialize Motors
        leftMotor = hwMap.dcMotor.get("left_drive");
        rightMotor = hwMap.dcMotor.get("right_drive");

        //rangeSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "range_sensor");
        //encMotor    = hwMap.dcMotor.get("enmotor");

        //encMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Define and Initialize Servos
        //sv1 = hwMap.servo.get("servo1");
        //sv1.setPosition(0.5);
        if (use_arm) {
            sv_elbow = hwMap.servo.get("sv_elbow");
            sv_shoulder = hwMap.servo.get("sv_shoulder");
            sv_elbow.setPosition(ELBOW_INIT);
            sv_shoulder.setPosition(SHOULDER_INIT);
        }
        //if (use_kicker) {
        sv_l_kicker = hwMap.servo.get("sv_l_kicker");
        sv_l_kicker.setPosition(L_KICKER_INIT);
        sv_r_kicker = hwMap.servo.get("sv_r_kicker");
        sv_r_kicker.setPosition(R_KICKER_INIT);
        //}
        if (use_color_sensor) {
            colorSensor = hwMap.colorSensor.get("rev_co");
            colorSensor.enableLed(true);
        }
        // Set all motors to zero power
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        // Use RUN_USING_ENCODERS for better speed control even encoder is not used.
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        change_chassis_braking_mode(true);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long remaining = periodMs - (long) period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }

    public void change_chassis_braking_mode(boolean braking) {
        if (braking) {
            leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    public double imu_heading() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public void driveTT(double lp, double rp) {
        if (straight_mode) { // expect to go straight
            if (use_imu_correction) {
                double cur_heading = imu_heading();
                if (cur_heading - target_heading > 0.7) { // crook to left,  slow down right motor
                    if (rp > 0) rp *= correction_ratio;
                    else lp *= correction_ratio;
                } else if (cur_heading - target_heading < -0.7) { // crook to right, slow down left motor
                    if (lp > 0) lp *= correction_ratio;
                    else rp *= correction_ratio;
                }
            }
        }
        if (Math.abs(rp) > 0.3 && Math.abs(lp) > 0.3) {
            rightMotor.setPower(rp * DRIVE_RATIO_R);
            leftMotor.setPower(lp * DRIVE_RATIO_L);
        } else {
            rightMotor.setPower(rp);
            leftMotor.setPower(lp);
        }
        if (use_encoder)
            ;//encMotor.setPower(Math.max(lp,rp));
    }

    void stop_chassis() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        //encMotor.setPower(0);
    }

    void reset_chassis() {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftCnt = 0;
        rightCnt = 0;
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void stop_tobot() {
        stop_chassis();
    }

    public void goUntilWhite(double power) throws InterruptedException {
        ElapsedTime runtime = new ElapsedTime();
        driveTT(power, power);
        int i = 0;
        while (!detectWhite() && (runtime.seconds() < 3)) {
            if ((++i % 10) == 0)
                driveTT(power, power);
        }
        stop_chassis();
    }

    public void run_until_encoder(int leftCnt, double leftPower, int rightCnt, double rightPower) throws InterruptedException {
        ElapsedTime runtime = new ElapsedTime();
        driveTT(leftPower, rightPower);
        runtime.reset();
        //while (motorFR.isBusy() || motorBL.isBusy()) {
        while (!have_drive_encoders_reached(leftCnt, rightCnt) && (runtime.seconds() < 5)) {
            driveTT(leftPower, rightPower);
        }
        stop_chassis();
    }

    boolean has_left_drive_encoder_reached(double p_count) {
        DcMotor mt = leftMotor;
        if (leftPower < 0) {
            //return (Math.abs(motorFL.getCurrentPosition()) < p_count);
            return (mt.getCurrentPosition() <= p_count);
        } else {
            //return (Math.abs(motorFL.getCurrentPosition()) > p_count);
            return (mt.getCurrentPosition() >= p_count);
        }
    } // has_left_drive_encoder_reached

    boolean has_right_drive_encoder_reached(double p_count) {
        DcMotor mt = rightMotor;
        if (rightPower < 0) {
            return (mt.getCurrentPosition() <= p_count);
        } else {
            return (mt.getCurrentPosition() >= p_count);
        }

    } // has_right_drive_encoder_reached

    /**
     * Indicate whether the drive motors' encoders have reached specified values.
     */
    boolean have_drive_encoders_reached(double p_left_count, double p_right_count) {
        boolean l_return = false;
        if (has_left_drive_encoder_reached(p_left_count) && has_right_drive_encoder_reached(p_right_count)) {
            l_return = true;
        } else if (has_left_drive_encoder_reached(p_left_count)) { // shift target encoder value from right to left
            double diff = Math.abs(p_right_count - rightMotor.getCurrentPosition()) / 2;
            if (leftPower < 0) {
                leftCnt -= diff;
            } else {
                leftCnt += diff;
            }
            if (rightPower < 0) {
                rightCnt += diff;
            } else {
                rightCnt -= diff;
            }
        } else if (has_right_drive_encoder_reached(p_right_count)) { // shift target encoder value from left to right
            double diff = Math.abs(p_left_count - leftMotor.getCurrentPosition()) / 2;
            if (rightPower < 0) {
                rightCnt -= diff;
            } else {
                rightCnt += diff;
            }
            if (leftPower < 0) {
                leftCnt += diff;
            } else {
                leftCnt -= diff;
            }
        }
        return l_return;
    } // have_encoders_reached

    public void TurnRightD(double power, double degree) throws InterruptedException {

        double adjust_degree_gyro = GYRO_ROTATION_RATIO_R * degree;
        double adjust_degree_navx = IMU_ROTATION_RATIO_R * degree;
        double current_pos = 0;
        boolean heading_cross_zero = false;
        ElapsedTime runtime = new ElapsedTime();
        reset_chassis();
        //set_drive_modes(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //motorFR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //motorBL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        int leftEncode = leftMotor.getCurrentPosition();
        int rightEncode = rightMotor.getCurrentPosition();
        leftCnt = (int) (ONE_ROTATION * RROBOT * degree / 720.0);
        rightCnt = (int) (-ONE_ROTATION * RROBOT * degree / 720.0);
        rightPower = (float) -power;

        leftCnt += leftEncode;
        rightCnt += rightEncode;
        leftPower = (float) power;
        // DbgLog.msg(String.format("imu Right Turn %.2f degree with %.2f power.", degree, power));
        if (use_imu) {
            current_pos = imu_heading();
            target_heading = current_pos - adjust_degree_navx;
            if (target_heading <= -180) {
                target_heading += 360;
                heading_cross_zero = true;
            }
            if (heading_cross_zero && (current_pos <= 0)) {
                current_pos += 360;
            }
            while ((current_pos >= target_heading) && (runtime.seconds() < 4.0)) {
                current_pos = imu_heading();
                // DbgLog.msg(String.format("imu current/target heading = %.2f/%.2f",current_pos,target_heading));

                if (heading_cross_zero && (current_pos <= 0)) {
                    current_pos += 360;
                }
                driveTT(leftPower, rightPower);
            }
        } else {
            if (use_encoder) {
                run_until_encoder(leftCnt, leftPower, rightCnt, rightPower);
            } else {
                long degree_in_ms = 33 * (long) degree;
                driveTT(leftPower, rightPower);
                sleep(degree_in_ms);
                driveTT(0, 0);
            }
        }
        driveTT(0, 0);
    }

    public void TurnLeftD(double power, double degree) throws InterruptedException {
        double adjust_degree_gyro = GYRO_ROTATION_RATIO_L * (double) degree;
        double adjust_degree_navx = IMU_ROTATION_RATIO_L * (double) degree;
        double current_pos = 0;
        boolean heading_cross_zero = false;
        ElapsedTime runtime = new ElapsedTime();
        reset_chassis();
        //set_drive_modes(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //motorFR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //motorBL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS)
        int leftEncode = leftMotor.getCurrentPosition();
        int rightEncode = rightMotor.getCurrentPosition();
        leftCnt = (int) (-ONE_ROTATION * RROBOT * degree / 720.0);
        rightCnt = (int) (ONE_ROTATION * RROBOT * degree / 720.0);

        leftPower = (float) -power;
        rightPower = (float) power;

        leftCnt += leftEncode;
        rightCnt += rightEncode;


        //DbgLog.msg(String.format("imu Left Turn %.2f degree with %.2f power.", degree, power));
        if (use_imu) {
            current_pos = imu_heading();
            target_heading = current_pos + adjust_degree_navx;
            if (target_heading >= 180) {
                target_heading -= 360;
                heading_cross_zero = true;
            }
            if (heading_cross_zero && (current_pos >= 0)) {
                current_pos -= 360;
            }
            //DbgLog.msg(String.format("imu Left Turn curr/target pos = %.2f/%.2f.", current_pos, target_heading));
            while ((current_pos <= target_heading) && (runtime.seconds() < 5.0)) {
                current_pos = imu_heading();
                if (heading_cross_zero && (current_pos >= 0)) {
                    current_pos -= 360;
                }
                driveTT(leftPower, rightPower);
            }
        } else {
            if (use_encoder) {
                run_until_encoder(leftCnt, leftPower, rightCnt, rightPower);
            } else {
                long degree_in_ms = 33 * (long) degree;
                driveTT(leftPower, rightPower);
                sleep(degree_in_ms);
                driveTT(0, 0);
            }
        }
        driveTT(0, 0);
    }

    /**
     *
     * @param power
     * @param seconds
     * @param rotation
     *
     * 0.5s should rotate roughly 90 degrees at 0.5 power. This rule should be able to be extended with an inverse relationship (e.g. 1.0 power roughly corrolates to 0.25s).
     * This is only for the "dance" movement and cannot be expected to be positionally accurate as it is based upon time.
     */
    public void driveByTime(double power, double seconds, int rotation) {
        ElapsedTime runtime = new ElapsedTime();
        double leftPower = power;
        double rightPower = power;

        reset_chassis();

        if (rotation == 1) {
            leftPower*=-1;
        }
        else if (rotation == 2) {
            rightPower*=-1;
        }

        while (runtime.seconds() <= seconds) {
            leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);

        }
    }

    public void StraightR(double power, double n_rotations) throws InterruptedException {
        straight_mode = true;
        reset_chassis();

        int leftEncode = leftMotor.getCurrentPosition();
        int rightEncode = rightMotor.getCurrentPosition();

        leftCnt = (int) (ONE_ROTATION * n_rotations);
        rightCnt = (int) (ONE_ROTATION * n_rotations);
        leftPower = rightPower = (float) power;
        if (power < 0) { // move backward
            leftCnt = leftEncode - leftCnt;
            rightCnt = rightEncode - rightCnt;
        } else {
            leftCnt += leftEncode;
            rightCnt += rightEncode;
        }
        run_until_encoder(leftCnt, leftPower, rightCnt, rightPower);
        straight_mode = false;
    }

    public void StraightIn(double power, double in) throws InterruptedException {
        if (use_imu) {
            target_heading = imu_heading();
        }
        if (use_encoder) {
            double numberR = in / INCHES_PER_ROTATION;
            StraightR(power, numberR);
        } else { // using timer
            double in_per_ms = 0.014 * power / 0.8;
            if (in_per_ms < 0) in_per_ms *= -1.0;
            long msec = (long) (in / in_per_ms);
            if (msec < 100) msec = 100;
            if (msec > 6000) msec = 6000; // limit to 6 sec
            driveTT(power, power);
            sleep(msec);
            driveTT(0, 0);
        }
    }

    public boolean detectWhite() {
        int cur_sum_of_color_sensor = 0;
        if (!use_color_sensor) {
            return false;
        }
        cur_sum_of_color_sensor = colorSensor.alpha() + colorSensor.red() + colorSensor.green() + colorSensor.blue();
        if (cur_sum_of_color_sensor >= color_white) {
            return true;
        }
        return false;
    }


    public void autoKicker() {
        if (isKickerUp)
            closeKicker();
        else{
            openKicker();
        }
    }

    public void openKicker() {
        isKickerUp = true;
        sv_l_kicker.setPosition(KICKER_UP);
    }

    public void closeKicker() {
        isKickerUp = false;
        sv_l_kicker.setPosition(KICKER_DOWN);
    }

    void l_kicker_up() {
        if (sv_l_kicker == null)
            return;
        sv_l_kicker.setPosition(L_KICKER_UP);
    }

    void l_kicker_down() {
        if (sv_l_kicker == null)
            return;
        sv_l_kicker.setPosition(L_KICKER_DOWN);
    }

    void r_kicker_up() {
        if (sv_r_kicker == null)
            return;
        sv_r_kicker.setPosition(R_KICKER_UP);
    }

    void r_kicker_down() {
        if (sv_r_kicker == null)
            return;
        sv_r_kicker.setPosition(R_KICKER_DOWN);
    }
}
