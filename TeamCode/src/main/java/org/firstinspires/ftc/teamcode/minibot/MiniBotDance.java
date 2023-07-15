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
        }

    }
}
