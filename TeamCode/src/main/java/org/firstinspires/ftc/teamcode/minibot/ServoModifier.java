package org.firstinspires.ftc.teamcode.minibot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class ServoModifier {

    /**
     * Robot 0: Messy minds
     * Robot 1: Ballz
     * Robot 2: Liam
     *
     * Set the arrays to correspond to each team:
     * initPos = {Messy minds, Ballz, Liam};
     *
     * The teams can request servo values to make their mechanisms work
     *
     * However, do not allow teams to jump between values too much, make them do the math to estimate
     *
     * You should change the values in the arrays to set the servo positions. Dw it works just do it here.
     *
     * Be sure you set the correct robotIndex before download to ensure you are downloading the correct team's code.
     */
    private int robotIndex = 2; //Change this before download to ensure you are downloading the correct values
    private double[] initPos = {0.9,0,0};
    private double[] upPos = {0.7,0.5,0.15};
    private double[] downPos = {1,0,0};
    //Do not touch below here
    public void setRobot(int i) {
        robotIndex = i;
    }

    public double getInitPos () {
        return initPos[robotIndex];
    }

    public double getUpPos () {
        return upPos[robotIndex];
    }

    public double getDownPos () {
        return downPos[robotIndex];
    }

    public ServoModifier() {

    }

}




