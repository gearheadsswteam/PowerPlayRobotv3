package org.firstinspires.ftc.teamcode.teleop.newrobot;

import static org.firstinspires.ftc.teamcode.classes.ValueStorage.gripperHold;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.gripperRelease;

public class Claw {

    private Robot robot;
    private double lastClawTime;

    public Claw(Robot robot) {
        this.robot = robot;
    }

    public double openClaw(double currTime) {
        robot.gripper.setPosition(gripperRelease);
        if (lastClawTime < currTime) {
            lastClawTime = currTime + 500;
        }
        return lastClawTime;
    }

    public double closeClaw(double currTime) {
        robot.gripper.setPosition(gripperHold);
        if (lastClawTime < currTime) {
            lastClawTime = currTime + 500;
        }
        return lastClawTime;
    }

    public double getLastClawTime() {
        return lastClawTime;
    }
}
