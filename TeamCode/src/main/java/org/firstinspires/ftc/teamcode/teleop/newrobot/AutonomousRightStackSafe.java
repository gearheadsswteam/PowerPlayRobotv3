package org.firstinspires.ftc.teamcode.teleop.newrobot;

import static org.firstinspires.ftc.teamcode.classes.ValueStorage.liftLowClose;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.liftMedClose;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.sides;
import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.AbstractAutonomous;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "RightStackSafe", group = "NewRobot")
public class AutonomousRightStackSafe extends AbstractAutonomous {

    //Drop point position
    Pose2d dropPose = new Pose2d(-39, 15, 0.45);
    //Park positions
    Pose2d[] parkPose = {new Pose2d(-11, 11, 0), new Pose2d(-35, 11, 0), new Pose2d(-59, 11, 0)};

    //Cone Stack pick up position
    Pose2d stackPose = new Pose2d(-60, 11, 0);

    ElapsedTime clock = new ElapsedTime();

    double time = 0;
    double dropTrajTime = 1000;
    double retractTime = 1000;
    double doneTime = 1000;

    boolean endDropTraj = false; // true if drop trajectory has completed
    boolean dropTrajDone = false; // true if drop trajectory has completed
    boolean traj5Done = false; //true if parking trajectory has completed
    boolean atConeStackPosition = false; //True if robot at stack position
    boolean parkCompleted = false; //True if parking is completed
    boolean coneAvailableForDrop = false; // True of cone has been picked up from stack


    int cycles = 0; //Number cone cycles completed
    int state = 0; // state machine to pick up the cone

    TrajectorySequence traj1; //From start to drop point
    TrajectorySequence traj2; //From drop to stack
    TrajectorySequence traj3; //From stack to drop
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
                            atConeStackPosition = false;
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
                    robot.arm.moveToGrabPosition(time);
                    robot.claw.openClaw(time);
                })
                .addTemporalMarker(1, 0, () -> {
                    atConeStackPosition = true;
                    endDropTraj = false;
                    dropTrajDone = false;
                })
                .build();

        //Stack to the drop point
        traj3 = robot.drive.trajectorySequenceBuilder(stackPose)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .splineTo(dropPose.vec(), dropPose.getHeading())
                .addTemporalMarker(1, -1, () -> {
                    robot.extendLiftProfile(time, liftMedClose[0], 0);
                })
                .addTemporalMarker(1, 0, () -> {
                    endDropTraj = true;
                    dropTrajDone = true;
                    atConeStackPosition = false;
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
                    if (atConeStackPosition) {
                        state = 1;
                        collectCone();
                    }
                } else {//go to park
                    if (!parkCompleted) {
                        robot.drive.followTrajectorySequenceAsync(traj5[runCase - 1]);
                        parkCompleted = true;
                        dropTrajDone = false;
                        endDropTraj = false;
                    }
                }
            }

            //**
            // Cone is inside
            //Go to drop position
            if (coneAvailableForDrop) {//cone is inside
                robot.drive.followTrajectorySequenceAsync(traj3);
                atConeStackPosition = false;
            }


            //You are at drop position
            if (time - dropTrajTime > 0.5) {
                robot.claw.openClaw(time);
                robot.clawTime();
                dropTrajDone = true;
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

    private void collectCone() {
        switch (state) {
            case 0:
                //Elevator Med + Arm Grab + Claw open
                robot.extendLiftProfile(time, liftLowClose[0], 0);
                robot.arm.moveToGrabPosition(time);
                robot.claw.openClaw(time);
                stateTime = robot.liftArmClawTime();

                //Move to next state
                if (time > stateTime) {
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
                } else {
                    //waiting on state 3 to complete
                }
                coneAvailableForDrop = true;
                dropTrajDone = true;
                break;

        }
    }
}
