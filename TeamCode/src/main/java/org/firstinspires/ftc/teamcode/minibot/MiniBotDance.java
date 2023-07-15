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
            robot.drive_forward(0, 6);
            robot.Turn_Left(0.15, 1);
            robot.drive_forward(0,0.5);
            robot.Turn_Left(-0.10, 1.75);
            robot.Turn_Left(0.15, 1.5);
            robot.drive_forward(0, 0.5);
            robot.Turn_Right(0.5, 0.25);
            robot.Turn_Left(0.5, 0.25);
            robot.Turn_Right(0.5, 0.25);
            robot.Turn_Left(0.5, 0.25);
            robot.Turn_Right(0.5, 0.25);
            robot.Turn_Left(0.5, 0.25);
            robot.Turn_Right(0.5, 0.25);
            robot.Turn_Left(0.5, 0.25);
            robot.Turn_Right(0.5, 1.75);
            robot.Turn_Left(0.5, 1.75);
            robot.drive_forward(0.15,1);
            robot.drive_forward(-0.15,1);
        }

    }
}
