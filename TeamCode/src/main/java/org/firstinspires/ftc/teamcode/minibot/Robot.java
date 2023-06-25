package org.firstinspires.ftc.teamcode.minibot;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    public Chassis chassis;
    public Kicker kicker;
    public Pusher pusher;

    public Robot() {

        chassis = new Chassis();
        kicker = new Kicker();
        pusher = new Pusher();

    }

    public void init(HardwareMap hwMap) {
        chassis.init(hwMap);
        kicker.init(hwMap);
        pusher.init(hwMap);
    }
}
