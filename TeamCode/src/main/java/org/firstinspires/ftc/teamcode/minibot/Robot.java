package org.firstinspires.ftc.teamcode.minibot;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    public Chassis chassis;
    public Kicker kicker;
    public Pusher pusher;
    public Core core;

    public Robot() {
        core = new Core();
        chassis = new Chassis(core);
        kicker = new Kicker(core);
        pusher = new Pusher(core);

    }

    public void init(HardwareMap hwMap) {
        chassis.init(hwMap);
        kicker.init(hwMap);
        pusher.init(hwMap);
    }
}
