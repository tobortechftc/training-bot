package org.firstinspires.ftc.teamcode.minibot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    Servo claw;

    final static double CLAW_CLOSED = 0;
    final static double CLAW_OPEN = 0.3; //change this
    final static double CLAW_INIT = CLAW_OPEN;
    private boolean isClawOpen;

    public Claw(){

    }

    public void init(HardwareMap hwMap) {
        claw = hwMap.servo.get("claw"); // should be kicker, not sv_l_kicker
        claw.setPosition(CLAW_INIT);
    }

    public void clawOpen() {
        claw.setPosition(CLAW_OPEN);
    }

    public void clawClose() {
        claw.setPosition(CLAW_CLOSED);
    }
}
