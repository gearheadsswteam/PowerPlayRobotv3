package org.firstinspires.ftc.teamcode.teleop.newrobot;

import static org.firstinspires.ftc.teamcode.classes.ValueStorage.armDropPosition;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.armGrabPosition;

public class Arm {

    private Robot robot;
    private double nextArmStateCompletionTime;

    public Arm(Robot robot) {
        this.robot = robot;
    }

    public double moveToGrabPosition(double currTime) {
        robot.armL.setPosition(armGrabPosition[0]);
        robot.armR.setPosition(armGrabPosition[1]);
        if (nextArmStateCompletionTime < currTime) {
            nextArmStateCompletionTime = currTime + 500;
        }
        return nextArmStateCompletionTime;
    }

    public double moveToDropPosition(double currTime) {
        robot.armL.setPosition(armDropPosition[0]);
        robot.armR.setPosition(armDropPosition[1]);
        if (nextArmStateCompletionTime < currTime) {
            nextArmStateCompletionTime = currTime + 500;
        }
        return nextArmStateCompletionTime;
    }

    public double getNextArmStateCompletionTime() {
        return nextArmStateCompletionTime;
    }
}
