package org.firstinspires.ftc.teamcode.teleop.newrobot;

import static org.firstinspires.ftc.teamcode.classes.ValueStorage.armRest;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.liftLowClose;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.liftMedClose;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.odoUp;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.side;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.wristRest;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.pow;
import static java.lang.Math.sin;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.classes.ValueStorage;

@TeleOp(name = "TeleOpRedBlueTwoDriverOptiNewRobot")
public class TeleOpRedBlueTwoDriverOpti extends LinearOpMode {

    Robot robot = new Robot();
    int state = 0;
    double initialHeading = ValueStorage.lastPose.getHeading() - side * PI / 2;
    double robotHeading;
    double moveAngle;
    double moveMagnitude;
    double turn;
    double stateTime = 0;
    double time;

    boolean aPressed = false;
    boolean bPressed = false;
    boolean aReleased = true;
    boolean bReleased = true;
    boolean xPressed = false;
    boolean xReleased = false;
    boolean yPressed = false;
    boolean yReleased = false;
    boolean lbPressed = false;
    boolean lbReleased = false;
    boolean rbPressed = false;
    boolean rbReleased = false;
    String elevatorButtonLatched = null;
    ElapsedTime clock = new ElapsedTime();


    @Override
    public void runOpMode() {
        robot.init(hardwareMap, armRest, wristRest);

//        robot.extendArmProfile(time, liftLowClose[1], 0);
//        robot.claw.openClaw(clock.seconds());
//        robot.retract.setPosition(odoUp);

        while (!isStarted() && !isStopRequested()) {
//            robot.update(clock.seconds());
        }
        while (opModeIsActive() && !isStopRequested()) {
            readGamePad();
            time = clock.seconds();

//            switch (state) {
//                case 0:
//                    //Elevator Med + Arm Grab + Claw open
//                    robot.extendLiftProfile(time, liftLowClose[0], 0);
//                    robot.arm.moveToGrabPosition(time);
//                    robot.claw.openClaw(time);
//                    stateTime = robot.liftArmClawTime();
//
//                    //Move to next state
//                    if (rbPressed) {
//                        state = 1;
//                    }
//                    break;
//
//                case 1:
//                    //Elevator goes down,
//                    if (time > stateTime) {
//                        //Elevator down
//                        robot.extendLiftProfile(time, liftLowClose[0], 0);
//                        stateTime = robot.liftArmTime();
//                        state = 2;
//                    } else {
//                        //waiting on state 1 to complete
//                    }
//                    break;
//
//                case 2://claw closes
//                    if (time > stateTime) {
//                        //Claw closes
//                        robot.claw.closeClaw(time);
//                        stateTime = robot.clawTime();
//                        state = 3;
//                    } else {
//                        //waiting on state 2 to complete
//                    }
//                    break;
//
//
//                case 3://elevator comes up with cone and arm moves to drop position
//                    if (time > stateTime) {
//                        //Claw closes
//                        robot.extendLiftProfile(time, liftMedClose[0], 0);
//                        robot.arm.moveToDropPosition(time);
//                        stateTime = robot.liftArmTime();
//                        state = 4;
//                    } else {
//                        //waiting on state 3 to complete
//                    }
//                    break;
//
//                case 4:
//                    if (time > stateTime) {
//                        if ("a".equals(elevatorButtonLatched)) {
//                            state = 5;
//                            robot.extendLiftProfile(time, liftLowClose[0], 0);
//                            stateTime = robot.liftTime();
//                        } else if ("b".equals(elevatorButtonLatched)) {
//                            state = 5;
//                            robot.extendLiftProfile(time, liftLowClose[0], 0);
//                            stateTime = robot.liftTime();
//                        } else if ("y".equals(elevatorButtonLatched)) {
//                            state = 5;
//                            robot.extendLiftProfile(time, liftLowClose[0], 0);
//                            stateTime = robot.liftTime();
//                        } else if ("x".equals(elevatorButtonLatched)) {
//                            state = 5;
//                            robot.extendLiftProfile(time, liftLowClose[0], 0);
//                            stateTime = robot.liftTime();
//                        }
//                    } else {
//                        //Waiting on state 3 to complete
//                    }
//
//                    break;
//
//                case 5://Claw opens
//                    if (time > stateTime) {
//                        //Claw opens
//                        if (rbPressed) {
//                            elevatorButtonLatched = null;
//                            robot.claw.openClaw(time);
//                            stateTime = robot.clawTime();
//                            state = 6;
//                        }
//                    } else {
//                        //waiting on state 4 to complete
//                    }
//                    break;
//
//                case 6:
//                    //Claw opens waits and arm moves to grab position
//                    if (time > stateTime) {
//                        state = 0;
//                    } else {
//                        //waiting on state 5 to complete
//                    }
//                    break;
//            }
//
//
//            if (gamepad1.dpad_up && (state == 2 || state == 3) && time > stateTime) {
//                //robot.extendLiftProfile(time, adjustUp(robot.liftProfile.getX(time)), 0);
//            } else if (gamepad1.dpad_down && (state == 2 || state == 3) && time > stateTime) {
//                //robot.extendLiftProfile(time, adjustDown(robot.liftProfile.getX(time)), 0);
//            }
//            robot.update(time);
            robotHeading = robot.getHeading() + initialHeading;
            moveAngle =

                    atan2(-gamepad2.left_stick_x, -gamepad2.left_stick_y) - robotHeading;
            moveMagnitude =

                    abs(pow(gamepad2.left_stick_x, 3)) +

                            abs(pow(gamepad2.left_stick_y, 3));
            if (moveMagnitude < 0.01) {
                moveMagnitude = 0;
            }

            turn =

                    pow(gamepad2.right_stick_x, 3);
            if (gamepad2.right_trigger < 0.1) {
                robot.setDrivePowers(moveMagnitude * Range.clip(sin(PI / 4 - moveAngle) / abs(cos(PI / 4 - moveAngle)), -1, 1) + turn,
                        moveMagnitude * Range.clip(sin(PI / 4 + moveAngle) / abs(cos(PI / 4 + moveAngle)), -1, 1) - turn,
                        moveMagnitude * Range.clip(sin(PI / 4 + moveAngle) / abs(cos(PI / 4 + moveAngle)), -1, 1) + turn,
                        moveMagnitude * Range.clip(sin(PI / 4 - moveAngle) / abs(cos(PI / 4 - moveAngle)), -1, 1) - turn);
            } else {
                robot.setDrivePowers((moveMagnitude * Range.clip(sin(PI / 4 - moveAngle) / abs(cos(PI / 4 - moveAngle)), -1, 1) + turn) / 4,
                        (moveMagnitude * Range.clip(sin(PI / 4 + moveAngle) / abs(cos(PI / 4 + moveAngle)), -1, 1) - turn) / 4,
                        (moveMagnitude * Range.clip(sin(PI / 4 + moveAngle) / abs(cos(PI / 4 + moveAngle)), -1, 1) + turn) / 4,
                        (moveMagnitude * Range.clip(sin(PI / 4 - moveAngle) / abs(cos(PI / 4 - moveAngle)), -1, 1) - turn) / 4);
            }

            moveElevator();
        }

    }

    private void readGamePad() {
        if (gamepad1.a) {
            aPressed = aReleased;
            elevatorButtonLatched = "a";
            aReleased = false;
        } else {
            aPressed = false;
            aReleased = true;
        }
        if (gamepad1.b) {
            elevatorButtonLatched = "b";
            bPressed = bReleased;
            bReleased = false;
        } else {
            bPressed = false;
            bReleased = true;
        }
        if (gamepad1.x) {
            elevatorButtonLatched = "x";
            xPressed = xReleased;
            xReleased = false;
        } else {
            xPressed = false;
            xReleased = true;
        }
        if (gamepad1.y) {
            elevatorButtonLatched = "y";
            yPressed = yReleased;
            yReleased = false;
        } else {
            yPressed = false;
            yReleased = true;
        }
        if (gamepad1.left_bumper) {
            lbPressed = lbReleased;
            lbReleased = false;
        } else {
            lbPressed = false;
            lbReleased = true;
        }
        if (gamepad1.right_bumper) {
            rbPressed = rbReleased;
            rbReleased = false;
        } else {
            rbPressed = false;
            rbReleased = true;
        }
        if (gamepad2.ps) {
            initialHeading -= robotHeading;
        }
    }

    private void moveElevator(){
        float elevatorPower = gamepad1.left_stick_x;
        robot.liftL.setPower(elevatorPower);
        robot.liftR.setPower(elevatorPower);
    }
}