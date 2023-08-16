package org.firstinspires.ftc.teamcode.minibot;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    public Chassis chassis;
    public Claw claw;
    public Pusher pusher;

    public Robot() {

        chassis = new Chassis();
        claw = new Claw();
        pusher = new Pusher();

    }

    public void init(HardwareMap hwMap) {
        chassis.init(hwMap);
        claw.init(hwMap);
        pusher.init(hwMap);
    }
}
