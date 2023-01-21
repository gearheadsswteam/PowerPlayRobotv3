package org.firstinspires.ftc.teamcode.teleop.newrobot;

import static org.firstinspires.ftc.teamcode.classes.ValueStorage.armIn;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.forwardArmProfile1;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.forwardArmProfile2;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.forwardWristProfile1;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.forwardWristProfile2;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.gripperHold;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.holderDetectionThreshold;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.liftLowClose;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.liftMedClose;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.rollerRetract;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.rollerUp;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.sides;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.wristIn;
import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.autonomous.AbstractAutonomous;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "RightStackSafe", group = "Right")
public class AutonomousRightStackSafe extends AbstractAutonomous {
    Pose2d dropPose = new Pose2d(-39, 15, 0.45);
    Pose2d[] parkPose = {new Pose2d(-11, 11, 0), new Pose2d(-35, 11, 0), new Pose2d(-59, 11, 0)};
    Pose2d stackPose = new Pose2d(-60, 11, 0);
    Pose2d knockPose = new Pose2d(-50, 11, 0);
    Pose2d backPose = new Pose2d(-54, 11, 0);
    Pose2d intakePose = new Pose2d(-62, 11, 0);
    ElapsedTime clock = new ElapsedTime();
    double time = 0;
    double dropTrajTime = 1000;
    double retractTime = 1000;
    double doneTime = 1000;

    boolean endDropTraj = false;
    boolean dropTrajDone = false;
    boolean traj5Done = false;
    boolean intakeTrajDone = false;
    boolean parkCompleted = false;
    boolean coneAvailableForDrop = false;

    boolean retractDone = true;
    boolean usingSensor = false;

    int cycles = 0;

    TrajectorySequence traj1; //From start to drop point
    TrajectorySequence traj2; //From drop to stack
    TrajectorySequence traj3; //From stack to drop
    TrajectorySequence traj4; //From drop to stack
    TrajectorySequence[] traj5; //From drop to park

    @Override
    public void initialize() {
        //Start to drop point
        traj1 = robot.drive.trajectorySequenceBuilder(initPose())
                .splineTo(new Vector2d(-35, 40), -PI / 2)
                .lineTo(new Vector2d(-35, 25))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(25))
                .splineToSplineHeading(dropPose, 2)
                .waitSeconds(0.3)//Waiting for the cone drop
                .resetConstraints()
                .addTemporalMarker(1, -0.7, () -> {
                    robot.extendLiftProfile(time, liftMedClose[0], 0);
                })
                .waitSeconds(0.5)//Allows the cone to fall
                .addTemporalMarker(1, 0, () -> {
                            endDropTraj = true;
                            dropTrajDone = true;
                            dropTrajTime = time;
                        }
                )
                .build();

        //Drop point to stack
        traj2 = robot.drive.trajectorySequenceBuilder(dropPose)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .setReversed(true)
                .splineTo(stackPose.vec(), stackPose.getHeading() + PI)
                .addDisplacementMarker(() -> {
                    //Med, grab position
                    robot.extendLiftProfile(time, liftMedClose[0], 0);
                    robot.claw.openClaw(time);
                })
                .addTemporalMarker(1, 0, () -> {
                    intakeTrajDone = true;
                })
                .build();

        //Stack to the drop point
        traj3 = robot.drive.trajectorySequenceBuilder(backPose)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .splineTo(dropPose.vec(), dropPose.getHeading())
                .addTemporalMarker(1, -1, () -> {
                    robot.extendLiftProfile(time, liftMedClose[0], 0);
                    //Arm in drop position
                })
                .addTemporalMarker(1, 0, () -> {
                    endDropTraj = true;
                    dropTrajDone = true;
                    dropTrajTime = time;
                    cycles++;
                })
                .build();

        //Drop point to park
        traj5 = new TrajectorySequence[]{
                robot.drive.trajectorySequenceBuilder(dropPose)
                        .setReversed(true)
                        .splineTo(parkPose[2].vec(), PI)
                        .lineTo(parkPose[0].vec())
                        .addTemporalMarker(1, 0, () -> {
                            traj5Done = true;
                        })
                        .build(),
                robot.drive.trajectorySequenceBuilder(dropPose)
                        .setReversed(true)
                        .splineTo(parkPose[2].vec(), PI)
                        .lineTo(parkPose[1].vec())
                        .addTemporalMarker(1, 0, () -> {
                            traj5Done = true;
                        })
                        .build(),
                robot.drive.trajectorySequenceBuilder(dropPose)
                        .setReversed(true)
                        .splineTo(parkPose[2].vec(), PI)
                        .addTemporalMarker(1, 0, () -> {
                            traj5Done = true;
                        })
                        .build()

        };
    }


    @Override
    public void run() {
        clock.reset();

        //Lift Med
        robot.extendLiftProfile(time, liftMedClose[0], 0);
        //Arm drop position
        robot.arm.moveToDropPosition(clock.seconds());

        //Drive to drop the first cone
        robot.drive.followTrajectorySequenceAsync(traj1);

        while (opModeIsActive() && !isStopRequested() && (!traj5Done || time > retractTime)) {
            time = clock.seconds();

            //You are at drop position
            if (endDropTraj) {
                robot.claw.openClaw(time);
                endDropTraj = false;
            }

            //Execute Traj2 from Drop position to stack
            if (dropTrajDone && time - dropTrajTime > 0.5) {
                if (cycles < 3) {
                    robot.drive.followTrajectorySequenceAsync(traj2);
                    //At cone pick up point
                    if(intakeTrajDone) {
                        collectCone(1);
                        coneAvailableForDrop = true;
                        dropTrajDone = false;
                    }
                } else {//go to park
                    if(!parkCompleted) {
                        robot.drive.followTrajectorySequenceAsync(traj5[runCase - 1]);
                        retractDone = true;
                        dropTrajDone = false;
                        parkCompleted = true;
                    }
                }
            }

            //**
            // Cone is inside
            //Go to drop position
            if (intakeTrajDone ) {//cone is inside
                robot.drive.followTrajectorySequenceAsync(traj3);
                intakeTrajDone = false;
                cycles++;
            }


            //You are at drop position

            if (retractDone && time > retractTime) {
                robot.armProfile = forwardArmProfile2(time);
                robot.wristProfile = forwardWristProfile2(time);
                doneTime = robot.armTime();
                retractDone = false;
            }


            robot.drive.update();
            robot.update(time);
        }
    }

    @Override
    public int side() {
        return sides.BLUE;
    }

    @Override
    public Pose2d initPose() {
        return new Pose2d(-32, 60, -PI / 2);
    }

    private double stateTime;

    private void collectCone(int state) {
        switch (state) {
            case 0:
                //Elevator Med + Arm Grab + Claw open
                robot.extendLiftProfile(time, liftLowClose[0], 0);
                robot.arm.moveToGrabPosition(time);
                robot.claw.openClaw(time);
                stateTime = robot.liftArmClawTime();

                //Move to next state
                if (rbPressed) {
                    state = 1;
                }
                break;

            case 1:
                //Elevator goes down,
                if (time > stateTime) {
                    //Elevator down
                    robot.extendLiftProfile(time, liftLowClose[0], 0);
                    stateTime = robot.liftArmTime();
                    state = 2;
                } else {
                    //waiting on state 1 to complete
                }
                break;

            case 2://claw closes
                if (time > stateTime) {
                    //Claw closes
                    robot.claw.closeClaw(time);
                    stateTime = robot.clawTime();
                    state = 3;
                } else {
                    //waiting on state 2 to complete
                }
                break;


            case 3://elevator comes up with cone and arm moves to drop position
                if (time > stateTime) {
                    //Claw closes
                    robot.extendLiftProfile(time, liftMedClose[0], 0);
                    robot.arm.moveToDropPosition(time);
                    stateTime = robot.liftArmTime();
                    state = 4;
                } else {
                    //waiting on state 3 to complete
                }
                break;

        }
    }
}
