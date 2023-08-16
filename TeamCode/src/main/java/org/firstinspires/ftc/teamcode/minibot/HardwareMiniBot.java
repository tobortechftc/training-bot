package org.firstinspires.ftc.teamcode.minibot;

//import com.qualcomm.ftccommon.DbgLog;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    final static double DRIVE_RATIO_L = 1.0; //control veering by lowering left motor power
    final static double DRIVE_RATIO_R = 1.0; //control veering by lowering right motor power

    final static double CLAW_CLOSED = Claw.CLAW_CLOSED;
    final static double CLAW_OPEN = Claw.CLAW_OPEN;
    final static double CLAW_INIT = Claw.CLAW_INIT;
    private boolean isClawOpen;

    /* Public OpMode members. */
    public DcMotor leftMotor = null;
    public DcMotor rightMotor = null;
    public Servo claw = null;

    public boolean use_minibot = true;
    public boolean use_imu = false;
    public boolean use_imu_correction = false;
    public boolean use_encoder = true;
    public boolean use_claw = true;
    public double target_heading = 0.0;
    public boolean straight_mode = false;
    public float leftPower = 0;
    public float rightPower = 0;
    public double correction_ratio = 0.9;
    public int leftCnt = 0; // left motor target counter
    public int rightCnt = 0; // right motor target counter

    BNO055IMU imu;

    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    public HardwareMiniBot() {

    }

    public void init(HardwareMap ahwMap) {
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


        leftMotor = hwMap.dcMotor.get("left_drive");
        rightMotor = hwMap.dcMotor.get("right_drive");

        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftMotor.setPower(0);
        rightMotor.setPower(0);

        claw = hwMap.servo.get("claw");
        claw.setPosition(CLAW_INIT);

        // Use RUN_USING_ENCODERS for better speed control even encoder is not used.
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        change_chassis_braking_mode(true);
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


    public void driveForward(double power, double seconds){
        ElapsedTime runtime = new ElapsedTime();
        double leftPower = power;
        double rightPower = power;
        reset_chassis();
        while (runtime.seconds() <= seconds) {
            leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);

        }
    }

    public void turnLeft(double power, double seconds){
        ElapsedTime runtime = new ElapsedTime();
        double leftPower = power;
        double rightPower = power;
        reset_chassis();
        while (runtime.seconds() <= seconds) {
            leftMotor.setPower(-leftPower);
            rightMotor.setPower(rightPower);

        }
    }

    public void turnRight(double power, double seconds){
        ElapsedTime runtime = new ElapsedTime();
        double leftPower = power;
        double rightPower = power;
        reset_chassis();
        while (runtime.seconds() <= seconds) {
            leftMotor.setPower(leftPower);
            rightMotor.setPower(-rightPower);

        }
    }

    public void clawAuto() {
        if (isClawOpen)
            clawClose();
        else{
            clawOpen();
        }
    }

    public void clawOpen() {
        isClawOpen = true;
        claw.setPosition(CLAW_OPEN);
        try {
            sleep(50);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    public void clawClose() {
        isClawOpen = false;
        claw.setPosition(CLAW_CLOSED);
        try {
            sleep(50);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    public void clawInc(boolean wristFast, boolean isUp) {
        double pos = 0.0;
        if(isUp)
            pos = claw.getPosition() + (wristFast?0.05:.001);
        else
            pos = claw.getPosition() - (wristFast?0.05:.001);

        if ( pos <= 1 && pos >= 0) {
            claw.setPosition(pos);
        }
    }

}