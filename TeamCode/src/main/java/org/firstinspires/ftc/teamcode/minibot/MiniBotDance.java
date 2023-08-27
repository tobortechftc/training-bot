package org.firstinspires.ftc.teamcode.minibot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Minibot: Autonomous", group = "Minibot")
public class MiniBotDance extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Initializing Robot", "Please Wait ...");
        telemetry.update();

        HardwareMiniBot robot = new HardwareMiniBot();
        robot.init(hardwareMap);

        waitForStart();

        if (opModeIsActive()) {
            //Do the auto dance here
            robot.stop_chassis();
            robot.driveForward(0, 6);
            robot.turnLeft(0.15, 1);
            robot.driveForward(0,0.5);
            robot.turnLeft(-0.10, 1.75);
            robot.turnLeft(0.15, 1.5);
            robot.driveForward(0, 0.5);
            robot.turnRight(0.5, 0.25);
            robot.turnLeft(0.5, 0.25);
            robot.turnRight(0.5, 0.25);
            robot.turnLeft(0.5, 0.25);
            robot.turnRight(0.5, 0.25);
            robot.turnLeft(0.5, 0.25);
            robot.turnRight(0.5, 0.25);
            robot.turnLeft(0.5, 0.25);
            robot.turnRight(0.5, 1.75);
            robot.turnLeft(0.5, 1.75);
            robot.driveForward(0.15,1);
            robot.driveForward(-0.15,1);
        }

    }
}
